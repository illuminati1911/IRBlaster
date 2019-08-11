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

#define BCM2835_P_BASE          0x3F000000
#define LINUX_BLOCK_SIZE        (4 * 1024)
#define GPIO_BASE               (BCM2835_P_BASE + 0x200000)
#define PWM_BASE                (BCM2835_P_BASE + 0x20C000)
#define CLK_BASE                (BCM2835_P_BASE + 0x101000)
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

// Driver major number.
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
    if (!mutex_trylock(&dev_access_mtx)) {
        alert("Device is busy with another process!");
        return -EBUSY;
    }
    return 0;
}

static int dev_release(struct inode *inodep, struct file *filep) {
    mutex_unlock(&dev_access_mtx);
    log("Device successfully closed.");
    return 0;
}

static ssize_t dev_read(struct file *filep, char *buffer, size_t len, loff_t *offset) {
    return 0;
}

static ssize_t dev_write(struct file *filep, const char *buffer, size_t len, loff_t *offset) {
    int integrity_sta, trms_data_size;
    int *transmission_data;
    // Save buffer
    //
    sprintf(usm_buffer, "%s", buffer);
    
    // Map buffer to ir_config
    //
    cfg = (struct ir_config *)buffer;

    // Check that received data is correct.
    //
    integrity_sta = verify_data_integrity();
    if (integrity_sta < 0) {
        return integrity_sta;
    }
    log("Message: %s", cfg->code);

    // Generate pulse/gap data for transmission
    //
    trms_data_size = strlen(cfg->code) * 2 + 3;
    transmission_data = (int*) kmalloc(trms_data_size * sizeof(int), GFP_KERNEL);
    generate_trms_data(transmission_data);

    // Set frequency and duty cycle with arbitrary pauses to
    // make sure hardware PWM is properly initiated.
    //
    set_frequency(cfg->frequency);
    udelay(100);
    set_duty_cycle(cfg->dc_n, cfg->dc_m);
    udelay(100);
    start_transmission(transmission_data, trms_data_size);

    // Cleanup
    //
    pwm_exit();
    kfree(transmission_data);
    return strlen(usm_buffer);
}

static void pwm_exit(void) {
    writel(0x0, pwm_ctl);
    udelay(10);
}

/*
 * Transmitting IR is extremely precise and time constrainted operation.
 * Therefore, PWM is handled by onboard Hardware PWM. Switching PWM and
 * sleeping is handled within spinlock, which also disables interrupts for
 * the duration of the transmission.
 * 
 * This is to avoid overhead of the OS scheduler and context switching.
 *
 */
static void start_transmission(int *codes, size_t len) {
    int i;
    unsigned long irq_flags;
    ktime_t edge;
    s32 delta;

    // Hold CPU and disable interrupts
    //
    spin_lock_irqsave(&transmit_spinlock, irq_flags);
    edge = ktime_get();
    for (i = 0; i < len; i++) {
        if (i % 2 == 0) {
            writel(0x1, pwm_ctl);
        }
        edge = ktime_add_us(edge, codes[i]);
		delta = ktime_us_delta(edge, ktime_get());
		if (delta >= 2000) {
            usleep_range(delta, delta + 10);
		} else if (delta > 0) {
			udelay(delta);
		}
        if (i % 2 == 0) {
            writel(0x0, pwm_ctl);
        }
    }

    // Release CPU and restore interrupts
    //
    spin_unlock_irqrestore(&transmit_spinlock, irq_flags);
    writel(0x0, pwm_ctl);
    udelay(100);
    log("Transmitted");
}

static int verify_data_integrity(void) {
    int i;
    limit_range(&cfg->leadingPulseWidth, 0, 10000);
    limit_range(&cfg->leadingGapWidth, 0, 10000);
    limit_range(&cfg->onePulseWidth, 0, 5000);
    limit_range(&cfg->oneGapWidth, 0, 5000);
    limit_range(&cfg->zeroPulseWidth, 0, 5000);
    limit_range(&cfg->zeroGapWidth, 0, 5000);
    limit_range(&cfg->trailingPulseWidth, 0, 5000);

    for (i = 0; i < strlen(cfg->code); i++) {
        if (cfg->code[i] != '0' && cfg->code[i] != '1') {
            return -EINVAL;
        }
    }
    return 0;
}

static void limit_range(unsigned *num, unsigned min, unsigned max) {
    if (*num > max) {
        *num = max;
    } else if (*num < min) {
        *num = min;
    }
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

/*
 * TODO: Add comments for this
 */
static int generate_trms_data(int *trms) {
    int i;
    int *p;
    trms[0] = cfg->leadingPulseWidth;
    trms[1] = cfg->leadingGapWidth;
    p = &trms[2];
    for (i = 0; i < strlen(cfg->code); i++) {
        if (cfg->code[i] == '0') {
            *(p + (i * 2)) = cfg->zeroPulseWidth;
            *(p + (i * 2 + 1)) = cfg->zeroGapWidth;
        } else if (cfg->code[i] == '1') {
            *(p + (i * 2)) = cfg->onePulseWidth;
            *(p + (i * 2 + 1)) = cfg->oneGapWidth;
        } else {
            return -EINVAL;
        }
    }
    i--;
    *(p + (i * 2 + 2)) = cfg->trailingPulseWidth;
    return 0;
}

static void set_frequency(unsigned f) {
    // Raspberry Pi PWM clock is running at 19.2 MHz
    //
    const unsigned clock_rate = 19200000;
    long idiv;
    unsigned max_busy_check = 5;
    
    // Kill the clock and zero PWM: 
    //
    writel(0x5A000020, clk_cntl);
    writel(0x0, pwm_ctl); 

    // Wait for busy flag to read zero
    //
    while ((readl(clk_cntl) >> CLK_CNTL_BUSY_OFFSET) & 1) {
        if (!max_busy_check) {
            return;
        }
        udelay(10);
        max_busy_check--;
    }
    
    // Calculate divisor and range limit
    //
    idiv = (long)( clock_rate / f );
    if ( idiv < 1 ) {
        idiv = 1;
    } else if ( idiv >= 0x1000 ) {
        idiv = 0xFFF;
    }
    
    // Move the integer part of the divisor to CLK_DIV. (Float will be ignored)
    //
    writel(0x5A000000 | ( idiv << 12 ), clk_div);
    
    /*
     * CLK CNTL
     * Source:          Oscillator
     * Clock enabled:   true
     */
    writel(0x5A000011, clk_cntl);
   
    // Clean GPIO register and set to Function 5
    //
    INP_GPIO(ib_gpio);
    SET_GPIO_ALT(ib_gpio, 5);

    // Disable PWM
    //
    writel(0x0, pwm_ctl);
}

static void set_duty_cycle(unsigned n, unsigned m) {
    // Disable PWM
    //
    writel(0x0, pwm_ctl);
    
    // Set duty cycle n/m
    //
    writel(m, pwm_rng1);
    writel(n, pwm_dat1);
    
    // Check channel 1 error state
    //
    if (!(IS_ERR_FLAG(PWM_STATUS_STA1_OFFSET))) {
        // Fifo read error
        //
        if (IS_ERR_FLAG(PWM_STATUS_RERR_OFFSET)) {
            CLEAR_ERR_FLAG(PWM_STATUS_RERR_OFFSET);
            // Fifo write error
            //
            if (IS_ERR_FLAG(PWM_STATUS_WERR_OFFSET)) {
                CLEAR_ERR_FLAG(PWM_STATUS_WERR_OFFSET);
                // Bus error
                //
                if (IS_ERR_FLAG(PWM_STATUS_BERR_OFFSET)) {
                    CLEAR_ERR_FLAG(PWM_STATUS_BERR_OFFSET);
                }
            }
        }
    }
}

module_init(ib_init);
module_exit(ib_exit);