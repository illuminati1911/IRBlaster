#include "irblaster.h"

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

static ssize_t dev_read(struct file *filep, char *buffer, size_t len,
                        loff_t *offset) {
  return 0;
}

static ssize_t dev_write(struct file *filep, const char *buffer, size_t len,
                         loff_t *offset) {
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
  transmission_data = (int *)kmalloc(trms_data_size * sizeof(int), GFP_KERNEL);
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
  idiv = (long)(clock_rate / f);
  if (idiv < 1) {
    idiv = 1;
  } else if (idiv >= 0x1000) {
    idiv = 0xFFF;
  }

  // Move the integer part of the divisor to CLK_DIV. (Float will be ignored)
  //
  writel(0x5A000000 | (idiv << 12), clk_div);

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