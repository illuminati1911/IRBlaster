#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <asm/io.h>

#define DEVICE_NAME "irblaster"
#define CLASS_NAME "ir"

#define BCM2835_P_BASE   0x3F000000
#define LINUX_BLOCK_SIZE  (4 * 1024)
#define GPIO_BASE           (BCM2835_P_BASE + 0x200000)
#define PWM_BASE            (BCM2835_P_BASE + 0x20C000)
#define CLK_BASE            (BCM2835_P_BASE + 0x101000)
#define CLK_CNTL_OFFSET     40
#define CLK_DIV_OFFSET      41

#define PWM_CONTROL_OFFSET 0
#define PWM_STATUS_OFFSET  1
#define PWM0_RANGE_OFFSET  4
#define PWM0_DATA_OFFSET   5

// Set GPIO as input or zero out the function selector. 
#define INP_GPIO(g) *(base_gpio+((g )/10)) &= ~(7<<(((g)%10)*3))
#define SET_GPIO_ALT(g,a) *(base_gpio+(((g)/10))) |= (((a)<=3?(a)+4:((a)==4?3:2))<<(((g)%10)*3))

// Logging shorthands.
#define log(...) printk(KERN_INFO "irblaster: " __VA_ARGS__);
#define alert(...) printk(KERN_ALERT "irblaster: Warning - " __VA_ARGS__);	

MODULE_LICENSE("Dual MIT/GPL");
MODULE_AUTHOR("illuminati1911");
MODULE_DESCRIPTION("An infrared LED driver for Raspberry Pi.");
MODULE_VERSION("0.1");

// Driver major number.
//
static int ib_mn;
static char message[300] = {0};
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

// Function definitions
//
static int dev_open(struct inode *, struct file *);
static int dev_release(struct inode *, struct file *);
static ssize_t dev_read(struct file *, char *, size_t, loff_t *);
static ssize_t dev_write(struct file *, const char *, size_t, loff_t *);
static void map_devmem_virtmem(void);
static void unmap_devmem_virtmem(void);

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
 *   0x40 = disable PWM
 *   0x41 = enable PWM
 */
static unsigned *pwm_ctl = 0;

static unsigned *pwm_sta = 0;

static unsigned *pwm_rng1 = 0;
static unsigned *pwm_dat1 = 0;

static unsigned *clk_cntl = 0;
static unsigned *clk_div = 0;

static int __init ib_init(void) {
    log("Initializing driver...");

    // Dynamic allocation of major number for the device.
    //
    ib_mn = register_chrdev(0, DEVICE_NAME, &fops);
    if (ib_mn < 0) {
        alert("Failed to register a major number!");
        return ib_mn;
    }
    log("Chrdev registered correctly with major number %d.", ib_mn);

    // Register device class.
    //
    ib_class = class_create(THIS_MODULE, CLASS_NAME);
    if (IS_ERR(ib_class)) {
        unregister_chrdev(ib_mn, DEVICE_NAME);
        log("Failed to register device class!");
        return PTR_ERR(ib_class);
    }
    log("Device class registered correctly.");

    // Register device driver.
    //
    ib_device = device_create(ib_class, NULL, MKDEV(ib_mn, 0), NULL, DEVICE_NAME);
    if (IS_ERR(ib_device)) {
        class_destroy(ib_class);
        unregister_chrdev(ib_mn, DEVICE_NAME);
        alert("Failed to create device!");
        return PTR_ERR(ib_device);
    }
    log("Device class created successfully!");
    
    // Initialize locks.
    //
    mutex_init(&dev_access_mtx);
    spin_lock_init(&transmit_spinlock);

    // Map device memory to kernel virtual memory.
    //
    map_devmem_virtmem();
    log("Driver loaded succesfully!");
    return 0;
}

static void __exit ib_exit(void) {
    device_destroy(ib_class, MKDEV(ib_mn, 0));
    class_destroy(ib_class);
    unregister_chrdev(ib_mn, DEVICE_NAME);
    mutex_destroy(&dev_access_mtx);
    unmap_devmem_virtmem();
    log("Driver closing... Bye!");
}

static int dev_open(struct inode *inodep, struct file *filep) {
    return 0;
}

static int dev_release(struct inode *inodep, struct file *filep) {
    return 0;
}

static ssize_t dev_read(struct file *filep, char *buffer, size_t len, loff_t *offset) {
    return 0;
}

static ssize_t dev_write(struct file *filep, const char *buffer, size_t len, loff_t *offset) {
    return 0;
}

/* Maps peripherals physical memory into kernel virtual memory.
 * 
 * Peripherals: GPIO, PWM, CLK
 */
static void map_devmem_virtmem(void) {
    // GPIO
    base_gpio = (unsigned *)ioremap(GPIO_BASE, LINUX_BLOCK_SIZE);
    // PWM
    base_pwm = (unsigned *)ioremap(PWM_BASE, LINUX_BLOCK_SIZE);
    pwm_ctl = &base_pwm[PWM_CONTROL_OFFSET];
    pwm_sta = &base_pwm[PWM_STATUS_OFFSET];
    pwm_rng1 = &base_pwm[PWM0_RANGE_OFFSET];
    pwm_dat1 = &base_pwm[PWM0_DATA_OFFSET];
    // CLK
    base_clk = (unsigned *)ioremap(CLK_BASE, LINUX_BLOCK_SIZE);
    clk_cntl = &base_clk[CLK_CNTL_OFFSET];
    clk_div = &base_clk[CLK_DIV_OFFSET];
}

/*
 * Unmap peripheral memory.
 */
static void unmap_devmem_virtmem(void) {
    iounmap(base_gpio);
    iounmap(base_pwm);
    iounmap(base_clk);
}

module_init(ib_init);
module_exit(ib_exit);