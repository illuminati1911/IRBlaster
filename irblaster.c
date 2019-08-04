#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/fs.h>

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
//
#define INP_GPIO(g) *(base_gpio+((g )/10)) &= ~(7<<(((g)%10)*3))
#define SET_GPIO_ALT(g,a) *(base_gpio+(((g)/10))) |= (((a)<=3?(a)+4:((a)==4?3:2))<<(((g)%10)*3))

#define log(...) printk(KERN_INFO "irblaster: " __VA_ARGS__);
#define alert(...) printk(KERN_ALERT "irblaster: Warning - " __VA_ARGS__);	


MODULE_LICENSE("Dual MIT/GPL");
MODULE_AUTHOR("illuminati1911");
MODULE_DESCRIPTION("An infrared LED driver for Raspberry Pi.");
MODULE_VERSION("0.1");

static int major_number;
static char message[300] = {0};
static unsigned int ir_gpio = 18;
static struct class* ib_class = NULL;
static struct device* ib_device = NULL;
// Mutex to control/limit access to the device.
//
static DEFINE_MUTEX(ib_mutex);

// Function definitions
//
static int dev_open(struct inode *, struct file *);
static int dev_release(struct inode *, struct file *);
static ssize_t dev_read(struct file *, char *, size_t, loff_t *);
static ssize_t dev_write(struct file *, const char *, size_t, loff_t *);

// Character driver operations.
//
static struct file_operations fops = {
    .open = dev_open,
    .read = dev_read,
    .write = dev_write,
    .release = dev_release,
};

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

static int __init ib_init(void) {
    log("Initializing driver...");

    // Dynamic allocation of major number for the device.
    //
    major_number = register_chrdev(0, DEVICE_NAME, &fops);
    if (major_number < 0) {
        alert("Failed to register a major number!");
        return major_number;
    }
    log("Chrdev registered correctly with major number %d", major_number);

    // Register device class 
    //
    ib_class = class_create(THIS_MODULE, CLASS_NAME);
    if (IS_ERR(ib_class)) {
        unregister_chrdev(major_number, DEVICE_NAME);
        log("Failed to register device class!");
        return PTR_ERR(ib_class);
    }
    log("Device class registered correctly.");

    // Register device driver
    //
    ib_device = device_create(ib_class, NULL, MKDEV(major_number, 0), NULL, DEVICE_NAME);
    if (IS_ERR(ib_device)) {
        class_destroy(ib_class);
        unregister_chrdev(major_number, DEVICE_NAME);
        alert("Failed to create device!");
        return PTR_ERR(ib_device);
    }

    log("Device class created successfully!");
    mutex_init(&ib_mutex);
    return 0;
}

static void __exit ib_exit(void) {
    device_destroy(ib_class, MKDEV(major_number, 0));
    class_destroy(ib_class);
    unregister_chrdev(major_number, DEVICE_NAME);
    mutex_destroy(&ib_mutex);
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

module_init(ib_init);
module_exit(ib_exit);