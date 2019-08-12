#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <asm/io.h>

#define DEVICE_NAME "irblaster"
#define CLASS_NAME "ir"

// If BCM peripherals base address is not defined
// use most popular one = rpi 2/3
//
#ifndef BCM_P_BASE
#define BCM_P_BASE          0x3F000000
#endif

#define LINUX_BLOCK_SIZE        (4 * 1024)
#define GPIO_BASE               (BCM_P_BASE + 0x200000)
#define PWM_BASE                (BCM_P_BASE + 0x20C000)
#define CLK_BASE                (BCM_P_BASE + 0x101000)
#define CLK_CNTL_OFFSET         40
#define CLK_CNTL_BUSY_OFFSET    7
#define CLK_DIV_OFFSET          41

#define PWM_CONTROL_OFFSET      0
#define PWM_STATUS_OFFSET       1
#define PWM0_RANGE_OFFSET       4
#define PWM0_DATA_OFFSET        5

#define PWM_STATUS_STA1_OFFSET  9
#define PWM_STATUS_RERR_OFFSET  3
#define PWM_STATUS_WERR_OFFSET  2
#define PWM_STATUS_BERR_OFFSET  8

// Set GPIO as input or zero out the function selector. 
#define INP_GPIO(g) *(base_gpio+((g )/10)) &= ~(7<<(((g)%10)*3))
#define SET_GPIO_ALT(g,a) *(base_gpio+(((g)/10))) |= (((a)<=3?(a)+4:((a)==4?3:2))<<(((g)%10)*3))
#define IS_ERR_FLAG(offset) (readl(&base_pwm[PWM_STATUS_OFFSET]) >> offset) & 1
#define CLEAR_ERR_FLAG(offset) writel(base_pwm[PWM_STATUS_OFFSET] | (1 << offset), &base_pwm[PWM_STATUS_OFFSET])

// Logging shorthands.
#define log(...) printk(KERN_INFO "irblaster: " __VA_ARGS__);
#define alert(...) printk(KERN_ALERT "irblaster: Warning - " __VA_ARGS__);	

#define CODE_BUFFER_LENGTH 0x200
#define USM_BUFFER_LENGTH 0x228

MODULE_LICENSE("Dual MIT/GPL");
MODULE_AUTHOR("illuminati1911");
MODULE_DESCRIPTION("An infrared LED driver for Raspberry Pi.");
MODULE_VERSION("0.1");

// Driver major number
//
static int ib_mn;

/* 
 * BCM layout number of the GPIO pin used to trasmit IR.
 *
 * You can change this to any GPIO supporting hardware PWM and 
 * using channel 0 on your Raspberry Pi. On RPi 3 that would be
 * 12 and 18. GPIO 18 is available on all RPis. 
 *
 */
static unsigned int ib_gpio = 18;
static struct class* ib_class = NULL;
static struct device* ib_device = NULL;

// Mutex to control/limit access to the device.
//
static DEFINE_MUTEX(dev_access_mtx);

// Spinlock to hold scheduler during critical operations.
//
static spinlock_t transmit_spinlock;

// Data received from the user space.
//
static char usm_buffer[USM_BUFFER_LENGTH] = {0};

// Infrared config to which the user space data will be mapped into.
//
struct ir_config {
   unsigned leadingPulseWidth;
   unsigned leadingGapWidth;
   unsigned onePulseWidth;
   unsigned oneGapWidth;
   unsigned zeroPulseWidth;
   unsigned zeroGapWidth;
   unsigned trailingPulseWidth;
   unsigned frequency;
   unsigned dc_n;
   unsigned dc_m;
   unsigned char code[CODE_BUFFER_LENGTH];
} *cfg;

// Function definitions
//
static int dev_open(struct inode *, struct file *);
static int dev_release(struct inode *, struct file *);
static ssize_t dev_read(struct file *, char *, size_t, loff_t *);
static ssize_t dev_write(struct file *, const char *, size_t, loff_t *);
static void pwm_exit(void);
static void start_transmission(int *codes, size_t len);
static int verify_data_integrity(void);
static void limit_range(unsigned *num, unsigned min, unsigned max);
static void map_devmem_virtmem(void);
static void unmap_devmem_virtmem(void);
static int generate_trms_data(int *trms);
static void set_frequency(unsigned f);
static void set_duty_cycle(unsigned n, unsigned m);

// Character driver file operations.
//
static struct file_operations fops = {
    .open = dev_open,
    .read = dev_read,
    .write = dev_write,
    .release = dev_release,
};

/* BCM2835 Device registers:
 *
 * Size: All are 32 bit
 * Reference: https://www.raspberrypi.org/app/uploads/2012/02/BCM2835-ARM-Peripherals.pdf
 *
 */
static unsigned *base_gpio = 0;
static unsigned *base_pwm = 0;
static unsigned *base_clk = 0;
/*   Relevant PWM CTL register flags:
 *   0 : PWEN1
 *   1 : MODE1
 *   2 : RPTL1
 *   3 : SBIT1
 *   4 : POLA1
 *   5 : USEF1
 *   6 : MSEN1
 *   6 : CLRF1
 *   7 : MSEN1
 *
 *   0x0 = disable PWM
 *   0x1 = enable PWM
 *
 *   P. 141-143, BCM2835 ARM Peripherals
 */
static unsigned *pwm_ctl = 0;
/*
 * Relevant PWM STA register flags {
 *   0 : FULL1
 *   1 : EMPT1
 *   2 : WERR1
 *   3 : RERR1
 *   4 : GAP01
 *   5 : GAP02
 *   6 : GAP03
 *   7 : GAP04
 *   8 : BERR
 *   9 : STA1
 *
 *   WERR = FIFO write error
 *   RERR = FIFO read error
 *   BERR = Bus error
 *   STA1 = Channel 1 state
 *
 *   P. 144-145, BCM2835 ARM Peripherals
 */
static unsigned *pwm_sta = 0;

/*
 *   RNG1: Channel 1 range
 *   This defines the period length of PWM.      
 *
 *   P. 145, BCM2835 ARM Peripherals
 */
static unsigned *pwm_rng1 = 0;
/*
 *   RNG1: Channel 1 data
 *   This defines the number on pulses sent within the period
 *   defined by RNG1.
 *
 *   P. 145, BCM2835 ARM Peripherals
 */
static unsigned *pwm_dat1 = 0;

/*
 *   CNTL: Clock control
 *   DIV: Clock divisor
 *
 *   The BCM2835 documentation is missing specs for the PWM CLK,
 *   but it's completely similar to the general purpose GPIO clocks: 
 *
 *   P. 105-108, BCM2835 ARM Peripherals
 */
static unsigned *clk_cntl = 0;
static unsigned *clk_div = 0;