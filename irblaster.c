#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/kernel.h>

#define DEVICE_NAME "irblaster"
#define CLASS_NAME "ir"

#define PWM_CONTROL_OFFSET 0
#define PWM_STATUS_OFFSET  1
#define PWM0_RANGE_OFFSET  4
#define PWM0_DATA_OFFSET   5

#define BCM2835_P_BASE   0x3F000000
#define LINUX_BLOCK_SIZE  (4 * 1024)
#define GPIO_BASE           (BCM2835_P_BASE + 0x200000)
#define PWM_BASE            (BCM2835_P_BASE + 0x20C000)
#define CLK_BASE            (BCM2835_P_BASE + 0x101000)
#define CLK_CNTL_OFFSET     40
#define CLK_DIV_OFFSET      41

// Set GPIO as input or zero out the function selector. 
//
#define INP_GPIO(g) *(base_gpio+((g )/10)) &= ~(7<<(((g)%10)*3))
#define SET_GPIO_ALT(g,a) *(base_gpio+(((g)/10))) |= (((a)<=3?(a)+4:((a)==4?3:2))<<(((g)%10)*3))

MODULE_LICENSE("Dual MIT/GPL");
MODULE_AUTHOR("illuminati1911");
MODULE_DESCRIPTION("An infrared LED driver for Raspberry Pi.");
MODULE_VERSION("0.1");

static int major_number;
static char message[300] = {0};
static unsigned int ir_gpio = 18;
static struct class* irblaster_class = NULL;
static struct device* irblaster_device = NULL;
// Mutex to control/limit access to the device.
//
static DEFINE_MUTEX(irblaster_mutex);

/*  BCM2835 Device registers:
*
*   Size: All are 32 bit
*   Reference: https://www.raspberrypi.org/app/uploads/2012/02/BCM2835-ARM-Peripherals.pdf
*/
static unsigned *base_gpio = 0;
static unsigned *base_pwm = 0;
static unsigned *base_clk = 0;
/*  Relevant PWM CTL register flags:
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
*   0x40 = disable PWM
*   0x41 = enable PWM
*/
static unsigned *pwm_ctl = 0;

static unsigned *pwm_sta = 0;

static unsigned *pwm_rng1 = 0;
static unsigned *pwm_dat1 = 0;

static unsigned *clk_cntl = 0;
static unsigned *clk_div = 0;

static int __init irblaster_init(void) {
    printk(KERN_INFO "irblaster: Initializing driver...\n");
    return 0;
}

static void __exit irblaster_exit(void) {
    printk(KERN_INFO "irblaster: Driver closing... Bye!\n");
}

module_init(irblaster_init);
module_exit(irblaster_exit);