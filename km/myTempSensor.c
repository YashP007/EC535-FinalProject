/* myTempSensor.c — EC535 Final Project "Temperature Sensor" kernel module
 *
 * Kernel module for BeagleBone Black to sense temperature from thermistor-based
 * resistive divider using BBB ADC. Features timer-based measurements and
 * hardware comparator IRQ for immediate threshold detection.
 *
 * ============================================================================
 * USERSPACE INTERFACE DOCUMENTATION
 * ============================================================================
 *
 * Device File: /dev/mytempsensor (major 62, minor 0)
 *
 * After loading the module, create the device file:
 *   sudo mknod /dev/mytempsensor c 62 0
 *   sudo chmod 666 /dev/mytempsensor
 *
 * ----------------------------------------------------------------------------
 * READING STATUS
 * ----------------------------------------------------------------------------
 *
 * Read current temperature and module status:
 *   cat /dev/mytempsensor
 *
 * Output format:
 *   temp=<temp>°F ref_temp=<ref>°F period=<ms> adc_above=<0|1> comparator=<0|1> comparator_gpio=<0|1> adc_awake=<0|1>
 *
 * Example output:
 *   temp=75.5°F ref_temp=150.0°F period=1000 ms adc_above=0 comparator=0 comparator_gpio=0 adc_awake=1
 *
 * ----------------------------------------------------------------------------
 * WRITING COMMANDS (Changing Parameters)
 * ----------------------------------------------------------------------------
 *
 * Write commands to /dev/mytempsensor to change module parameters:
 *
 * 1. Set Reference Temperature Threshold:
 *    echo "ref_temp=<temperature>" > /dev/mytempsensor
 *
 *    Parameters:
 *      <temperature> - Temperature in Fahrenheit (supports decimals)
 *                      Range: -459.67 to 1000.0°F
 *                      Default: 150.0°F
 *
 *    Examples:
 *      echo "ref_temp=150.0" > /dev/mytempsensor    # Set to 150°F
 *      echo "ref_temp=200" > /dev/mytempsensor      # Set to 200°F
 *      echo "ref_temp=75.5" > /dev/mytempsensor     # Set to 75.5°F
 *
 * 2. Set Measurement Period (Timer Interval):
 *    echo "period=<milliseconds>" > /dev/mytempsensor
 *
 *    Parameters:
 *      <milliseconds> - Timer period in milliseconds
 *                       Range: 100 to 60000 ms
 *                       Default: 1000 ms (1 second)
 *
 *    Examples:
 *      echo "period=1000" > /dev/mytempsensor       # 1 second intervals
 *      echo "period=500" > /dev/mytempsensor         # 0.5 second intervals
 *      echo "period=5000" > /dev/mytempsensor        # 5 second intervals
 *
 * ----------------------------------------------------------------------------
 * INTERRUPT-DRIVEN NOTIFICATIONS (poll/select)
 * ----------------------------------------------------------------------------
 *
 * The module supports poll()/select() for interrupt-driven notifications.
 * When a temperature threshold is exceeded (either by ADC measurement or
 * hardware comparator), the module wakes up waiting processes.
 *
 * Usage in C program:
 *   #include <poll.h>
 *   
 *   int fd = open("/dev/mytempsensor", O_RDONLY);
 *   struct pollfd pfd;
 *   pfd.fd = fd;
 *   pfd.events = POLLIN | POLLRDNORM;
 *   
 *   // Wait for threshold event (with timeout)
 *   int ret = poll(&pfd, 1, 5000);  // 5 second timeout
 *   
 *   if (ret > 0 && (pfd.revents & POLLIN)) {
 *       // Threshold event occurred - read notification
 *       char buf[512];
 *       read(fd, buf, sizeof(buf));
 *       printf("Event: %s", buf);
 *   }
 *
 * ----------------------------------------------------------------------------
 * DEFAULT VALUES
 * ----------------------------------------------------------------------------
 *
 * - Reference Temperature: 150.0°F
 * - Measurement Period: 1000 ms (1 second)
 * - ADC Channel: 0 (AIN0)
 * - Comparator GPIO: 26
 *
 * ----------------------------------------------------------------------------
 * MODULE OPERATION
 * ----------------------------------------------------------------------------
 *
 * The module operates in two modes:
 *
 * 1. Timer-based measurements:
 *    - Periodically reads ADC and calculates temperature
 *    - Compares against software threshold (ref_temp)
 *    - Updates status on each measurement
 *
 * 2. Hardware comparator interrupts:
 *    - Monitors GPIO 26 for hardware comparator signals
 *    - Immediately notifies userspace when triggered
 *    - Works in parallel with timer-based measurements
 *
 * Both modes can trigger threshold notifications via poll()/select().
 *
 * ============================================================================
 */

/* --- Includes: core driver headers --- */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>      /* printk() */
#include <linux/slab.h>        /* kmalloc()/kfree() */
#include <linux/fs.h>          /* register_chrdev(), file_operations */
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/uaccess.h>     /* copy_to_user()/copy_from_user() */
#include <linux/string.h>

/* --- Includes: timing, IRQ, GPIO, file operations --- */
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/gpio.h>        /* legacy integer GPIO API */
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/poll.h>

/* --- Includes: file operations for ADC sysfs access --- */
#include <linux/file.h>
#include <linux/fdtable.h>
#include <linux/fs.h>

/* --- Math operations --- */
/* Fixed-point natural logarithm approximation for kernel space */
/* Input: x in fixed-point (scaled by 1000), returns ln(x) scaled by 1000 */
static long simple_log_fixed(long x)
{
  /* For fixed-point: x is scaled by 1000, so x=1000 means 1.0
   * We use polynomial approximation: ln(x) ≈ 2*((x-1)/(x+1) + ((x-1)/(x+1))³/3)
   * All values scaled by 1000
   */
  long result = 0;
  long term;
  long power;
  long y;
  int i;
  
  if (x <= 0) return -1000000; /* Invalid */
  if (x == 1000) return 0; /* ln(1) = 0 */
  
  /* For values near 1 (near 1000 in fixed-point) */
  if (x > 500 && x < 2000) {
    /* y = (x - 1000) / (x + 1000), scaled by 1000 */
    y = ((x - 1000) * 1000) / (x + 1000);
    /* result = 2 * (y + y³/3), scaled by 1000 */
    result = (2 * 1000 * (y + ((y * y / 1000) * y / 1000) / 3)) / 1000;
    return result;
  }
  
  /* For other values, use iterative method */
  /* term = (x - 1000) / x, scaled by 1000 */
  term = ((x - 1000) * 1000) / x;
  power = term;
  
  for (i = 1; i < 20; i++) {
    result += power / i;
    power = (power * term) / 1000;
    if (power < 100) break; /* Convergence check (0.1 scaled) */
  }
  
  return result;
}

MODULE_LICENSE("Dual BSD/GPL");
MODULE_DESCRIPTION("Temperature sensor kernel module for BBB");
MODULE_AUTHOR("Yash Patel");

/* --- GPIO assignments (BBB header numbers -> kernel GPIO numbers)
 * TODO: verify these numeric GPIO IDs against your wiring.
 */
#define COMPARATOR_GPIO    26   /* GPIO pin for hardware comparator IRQ (active high) */

/* --- ADC configuration --- */
#define ADC_CHANNEL        0    /* ADC channel (AIN0) - adjust as needed */
#define ADC_SYSFS_PATH     "/sys/bus/iio/devices/iio:device0/in_voltage0_raw"
#define ADC_VREF           1800  /* ADC reference voltage in mV (1.8V for BBB) */
#define ADC_MAX_VALUE      4095  /* 12-bit ADC max value */

/* --- Thermistor parameters (fixed-point: scaled by 1000) --- */
#define THERMISTOR_R_NOM   10000000  /* 10000.0Ω = 10000000 milliohms */
#define THERMISTOR_BETA    3950000   /* 3950.0 scaled by 1000 */
#define THERMISTOR_T_NOM   25000     /* 25.0°C = 25000 millidegrees */
#define DIVIDER_R          10000000  /* 10000.0Ω = 10000000 milliohms */

/* --- Buffer sizing for char device I/O --- */
enum { capacity = 4096, bite = 128 };

/* --- Forward declarations: file ops --- */
static int     mytempsensor_open(struct inode *inode, struct file *filp);
static int     mytempsensor_release(struct inode *inode, struct file *filp);
static ssize_t mytempsensor_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos);
static ssize_t mytempsensor_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos);
static unsigned int mytempsensor_poll(struct file *filp, poll_table *wait);

/* --- Forward declarations: module init/exit --- */
static int  mytempsensor_init(void);
static void mytempsensor_exit(void);

/* --- Forward declarations: ADC and temperature functions --- */
static int  adc_read_raw(void);
static int  adc_wake(void);
static void adc_sleep(void);
static long adc_voltage_to_resistance(int adc_raw);  /* Returns milliohms */
static long resistance_to_temperature_c(long resistance);  /* Returns millidegrees C */
static long celsius_to_fahrenheit(long celsius);  /* Returns millidegrees F */
static long read_temperature_f(void);  /* Returns millidegrees F */

/* --- Forward declarations: GPIO and IRQ --- */
static int  temp_gpio_request_config(void);
static void temp_gpio_free(void);
static int  temp_irq_request(void);
static void temp_irq_free(void);

/* --- Forward declarations: timer and threshold checking --- */
static void temp_timer_callback(struct timer_list *t);
static void temp_schedule_next_measurement(void);
static void temp_check_thresholds(void);
static void temp_notify_userspace(const char *message);

/* --- Forward declarations: userspace interface --- */
static int  temp_build_status(char *dst, size_t maxlen);
static void temp_parse_write_command(const char *buf, size_t count);
static int  temp_parse_temp(const char *str, long *result);  /* Returns millidegrees */

/* --- File operations table --- */
static struct file_operations mytempsensor_fops = {
  .read    = mytempsensor_read,
  .write   = mytempsensor_write,
  .open    = mytempsensor_open,
  .release = mytempsensor_release,
  .poll    = mytempsensor_poll,
};

/* --- Module entry/exit wiring --- */
module_init(mytempsensor_init);
module_exit(mytempsensor_exit);

/* --- Globals: char device bookkeeping --- */
static int   mytempsensor_major = 62;    /* Use major 62, minor 0 */
static char *mytempsensor_buffer;         /* backing store for /dev reads/writes */
static int   mytempsensor_len;

/* --- Globals: IRQ bookkeeping --- */
static int comparator_irq = -1;          /* assigned by request_irq */

/* --- Globals: Temperature and threshold state (fixed-point: millidegrees) --- */
static atomic_t adc_temp_above_threshold = ATOMIC_INIT(0);  /* ADC measurement threshold */
static atomic_t comparator_triggered = ATOMIC_INIT(0);      /* Hardware comparator triggered */
static long current_temperature_f = 0;        /* Current temp in millidegrees F */
static long reference_temperature_f = 150000;  /* Reference temp: 150.0°F = 150000 millidegrees */
static unsigned long measurement_period_ms = 1000;          /* Timer period in milliseconds (default 1s) */

/* --- Globals: ADC state --- */
static atomic_t adc_awake = ATOMIC_INIT(0);                 /* ADC awake state */

/* --- Globals: Wait queue for poll/select --- */
static DECLARE_WAIT_QUEUE_HEAD(temp_wait_queue);
static atomic_t data_available = ATOMIC_INIT(0);           /* New data available flag */

/* --- Timer for periodic measurements --- */
static DEFINE_TIMER(temp_timer, temp_timer_callback);

/* ========== Module init/exit ========== */

static int mytempsensor_init(void)
{
  int result;

  /* Register character device */
  result = register_chrdev(mytempsensor_major, "mytempsensor", &mytempsensor_fops);
  if (result < 0) {
    printk(KERN_ALERT "mytempsensor: cannot obtain major %d\n", mytempsensor_major);
    return result;
  }

  /* Allocate kernel buffer */
  mytempsensor_buffer = kmalloc(capacity, GFP_KERNEL);
  if (!mytempsensor_buffer) {
    printk(KERN_ALERT "mytempsensor: insufficient kernel memory\n");
    result = -ENOMEM;
    goto fail_chrdev;
  }
  memset(mytempsensor_buffer, 0, capacity);
  mytempsensor_len = 0;

  /* Request and configure GPIO for comparator */
  result = temp_gpio_request_config();
  if (result) {
    printk(KERN_ALERT "mytempsensor: gpio request/config failed (%d)\n", result);
    goto fail_buf;
  }

  /* Request IRQ for comparator */
  result = temp_irq_request();
  if (result) {
    printk(KERN_ALERT "mytempsensor: irq request failed (%d)\n", result);
    goto fail_gpio;
  }

  /* Initialize ADC (wake it up for first measurement) */
  adc_wake();
  
  /* Start periodic timer for temperature measurements */
  temp_schedule_next_measurement();

  printk(KERN_INFO "mytempsensor: module inserted\n");
  printk(KERN_INFO "mytempsensor: Device available at /dev/mytempsensor\n");
  printk(KERN_INFO "mytempsensor: Default reference temp: %ld.%ld°F, period: %lu ms\n",
         reference_temperature_f / 1000, (reference_temperature_f % 1000) / 100, measurement_period_ms);
  printk(KERN_INFO "mytempsensor: Commands: ref_temp=<F>, period=<ms>\n");
  return 0;

fail_gpio:
  temp_gpio_free();
fail_buf:
  kfree(mytempsensor_buffer);
fail_chrdev:
  unregister_chrdev(mytempsensor_major, "mytempsensor");
  return result;
}

static void mytempsensor_exit(void)
{
  /* Stop timer before tearing down */
  del_timer_sync(&temp_timer);

  /* Put ADC to sleep */
  adc_sleep();

  /* Free IRQ and GPIO */
  temp_irq_free();
  temp_gpio_free();

  /* Unregister char device */
  unregister_chrdev(mytempsensor_major, "mytempsensor");

  /* Free buffer */
  kfree(mytempsensor_buffer);

  printk(KERN_INFO "mytempsensor: module removed\n");
}

/* ========== File operations ========== */

static int mytempsensor_open(struct inode *inode, struct file *filp)
{
  return 0;
}

static int mytempsensor_release(struct inode *inode, struct file *filp)
{
  return 0;
}

static ssize_t mytempsensor_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
  /* Return status information */
  char tbuf[512];
  int n;

  /* Build fresh status string each read start */
  if (*f_pos == 0) {
    n = temp_build_status(tbuf, sizeof(tbuf));
    n = min(n, (int)capacity);
    memcpy(mytempsensor_buffer, tbuf, n);
    mytempsensor_len = n;
    atomic_set(&data_available, 0);
  }

  /* EOF if f_pos at end */
  if (*f_pos >= mytempsensor_len)
    return 0;

  /* Do not exceed tail or user's count */
  if (count > mytempsensor_len - *f_pos)
    count = mytempsensor_len - *f_pos;
  if (count > bite)
    count = bite;

  if (copy_to_user(buf, mytempsensor_buffer + *f_pos, count))
    return -EFAULT;

  *f_pos += count;
  return count;
}

static ssize_t mytempsensor_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos)
{
  /* Write path: accept commands to set reference temperature and period */
  char kbuf[128];
  size_t c = min(count, sizeof(kbuf) - 1);

  if (c == 0)
    return 0;

  if (copy_from_user(kbuf, buf, c))
    return -EFAULT;

  kbuf[c] = '\0';

  /* Parse command */
  temp_parse_write_command(kbuf, c);

  return count;
}

static unsigned int mytempsensor_poll(struct file *filp, poll_table *wait)
{
  unsigned int mask = 0;

  poll_wait(filp, &temp_wait_queue, wait);

  if (atomic_read(&data_available))
    mask |= POLLIN | POLLRDNORM;

  return mask;
}

/* ========== ADC functions ========== */

/* Read raw ADC value from sysfs */
static int adc_read_raw(void)
{
  mm_segment_t old_fs;
  struct file *f;
  char adc_path[] = ADC_SYSFS_PATH;
  char read_buf[16];
  int ret, adc_value = 0;
  loff_t pos = 0;

  /* Save current file system context */
  old_fs = get_fs();
  set_fs(KERNEL_DS);

  /* Open ADC sysfs file */
  f = filp_open(adc_path, O_RDONLY, 0);
  if (IS_ERR(f)) {
    printk(KERN_ERR "mytempsensor: failed to open ADC sysfs file: %s\n", adc_path);
    set_fs(old_fs);
    return -1;
  }

  /* Read ADC value */
  memset(read_buf, 0, sizeof(read_buf));
  ret = kernel_read(f, read_buf, sizeof(read_buf) - 1, &pos);
  filp_close(f, NULL);

  /* Restore file system context */
  set_fs(old_fs);

  if (ret > 0) {
    if (kstrtoint(read_buf, 10, &adc_value) != 0) {
      printk(KERN_ERR "mytempsensor: failed to parse ADC value\n");
      return -1;
    }
  } else {
    printk(KERN_ERR "mytempsensor: failed to read ADC value\n");
    return -1;
  }

  return adc_value;
}

/* Wake ADC (enable it) */
static int adc_wake(void)
{
  /* On BBB, ADC is typically always on, but we can mark it as awake */
  atomic_set(&adc_awake, 1);
  return 0;
}

/* Put ADC to sleep (disable it to save power) */
static void adc_sleep(void)
{
  /* On BBB, we can't really disable the ADC hardware, but we mark it as asleep */
  atomic_set(&adc_awake, 0);
}

/* Convert ADC raw value to voltage, then to resistance (returns milliohms) */
static long adc_voltage_to_resistance(int adc_raw)
{
  long voltage_mv;
  long resistance;

  if (adc_raw < 0 || adc_raw > ADC_MAX_VALUE)
    return -1;

  /* Convert ADC reading to voltage (mV) - integer math */
  voltage_mv = ((long)adc_raw * ADC_VREF) / ADC_MAX_VALUE;

  /* Calculate thermistor resistance using voltage divider:
   * R_thermistor = R_divider * (Vref / Vout - 1)
   * All in milliohms and millivolts
   */
  if (voltage_mv > 0 && voltage_mv < ADC_VREF) {
    /* resistance = DIVIDER_R * ((ADC_VREF * 1000) / voltage_mv - 1000) / 1000 */
    resistance = (DIVIDER_R * ((ADC_VREF * 1000) / voltage_mv - 1000)) / 1000;
  } else {
    return -1;
  }

  return resistance;
}

/* Convert resistance to temperature using Steinhart-Hart (returns millidegrees C) */
static long resistance_to_temperature_c(long resistance)
{
  long temp_kelvin, temp_celsius;
  long ln_r, beta_term;
  long t0_kelvin = 298150;  /* 298.15K = 298150 millikelvin */

  if (resistance <= 0)
    return -273150; /* Invalid: -273.15°C */

  /* Steinhart-Hart: 1/T = 1/T0 + (1/B) * ln(R/R0)
   * All in fixed-point (scaled by 1000)
   * T0 = 298.15K = 298150 millikelvin
   */
  /* ln_r = ln(resistance / THERMISTOR_R_NOM), scaled by 1000 */
  ln_r = simple_log_fixed((resistance * 1000) / THERMISTOR_R_NOM);
  
  /* beta_term = (1 / THERMISTOR_BETA) * ln_r, scaled by 1000 */
  beta_term = (1000 * 1000 * ln_r) / THERMISTOR_BETA;
  
  /* temp_kelvin = 1 / ((1 / t0_kelvin) + beta_term), all scaled by 1000 */
  /* 1/T = (1/T0) + beta_term = (1000/t0_kelvin) + beta_term/1000 */
  /* T = 1000 / ((1000*1000/t0_kelvin) + beta_term) */
  temp_kelvin = (1000 * 1000 * 1000) / ((1000 * 1000 * 1000) / t0_kelvin + beta_term);
  
  /* temp_celsius = temp_kelvin - 273150 (convert K to C, both in millidegrees) */
  temp_celsius = temp_kelvin - 273150;

  return temp_celsius;
}

/* Convert Celsius to Fahrenheit (both in millidegrees) */
static long celsius_to_fahrenheit(long celsius)
{
  /* F = (C * 9/5) + 32, all in millidegrees */
  return ((celsius * 9) / 5) + 32000;
}

/* Read temperature in Fahrenheit (returns millidegrees F) */
static long read_temperature_f(void)
{
  int adc_raw;
  long resistance, temp_c, temp_f;

  /* Check if ADC is awake */
  if (!atomic_read(&adc_awake)) {
    adc_wake();
    msleep(10); /* Small delay for ADC to stabilize */
  }

  /* Read ADC */
  adc_raw = adc_read_raw();
  if (adc_raw < 0) {
    printk(KERN_WARNING "mytempsensor: ADC read failed\n");
    return -459670; /* Absolute zero in Fahrenheit (millidegrees) */
  }

  /* Convert to resistance (milliohms) */
  resistance = adc_voltage_to_resistance(adc_raw);
  if (resistance < 0) {
    printk(KERN_WARNING "mytempsensor: invalid resistance value\n");
    return -459670;
  }

  /* Convert to temperature (millidegrees) */
  temp_c = resistance_to_temperature_c(resistance);
  temp_f = celsius_to_fahrenheit(temp_c);

  return temp_f;
}

/* ========== GPIO and IRQ ========== */

static int temp_gpio_request_config(void)
{
  int ret = 0;

  /* Request comparator GPIO as input */
  ret = gpio_request(COMPARATOR_GPIO, "temp_comparator");
  if (ret) goto err;

  ret = gpio_direction_input(COMPARATOR_GPIO);
  if (ret) goto err;

  return 0;

err:
  printk(KERN_ERR "mytempsensor: gpio request/config error\n");
  return ret ?: -EINVAL;
}

static void temp_gpio_free(void)
{
  gpio_free(COMPARATOR_GPIO);
}

/* IRQ handler for comparator */
static irqreturn_t comparator_isr(int irq, void *dev_id)
{
  /* Hardware comparator triggered (active high) */
  atomic_set(&comparator_triggered, 1);
  
  /* Check both thresholds and notify */
  temp_check_thresholds();
  
  return IRQ_HANDLED;
}

static int temp_irq_request(void)
{
  int ret;

  /* Map GPIO to IRQ number */
  comparator_irq = gpio_to_irq(COMPARATOR_GPIO);
  if (comparator_irq < 0) {
    printk(KERN_ERR "mytempsensor: failed to get IRQ for GPIO %d\n", COMPARATOR_GPIO);
    return -EINVAL;
  }

  /* Request IRQ for rising edge (active high) */
  ret = request_irq(comparator_irq, comparator_isr, IRQF_TRIGGER_RISING,
                    "mytempsensor_comparator", NULL);
  if (ret) {
    printk(KERN_ERR "mytempsensor: failed to request IRQ %d\n", comparator_irq);
    return ret;
  }

  return 0;
}

static void temp_irq_free(void)
{
  if (comparator_irq >= 0) {
    free_irq(comparator_irq, NULL);
    comparator_irq = -1;
  }
}

/* ========== Timer and threshold checking ========== */

static void temp_timer_callback(struct timer_list *t)
{
  /* Timer expired - take temperature measurement */
  long temp_f;

  /* Wake ADC if sleeping */
  if (!atomic_read(&adc_awake)) {
    adc_wake();
    msleep(10); /* Small delay for ADC to stabilize */
  }

  /* Read temperature (in millidegrees F) */
  temp_f = read_temperature_f();
  current_temperature_f = temp_f;

  /* Check if above software threshold (both in millidegrees) */
  if (temp_f >= reference_temperature_f) {
    atomic_set(&adc_temp_above_threshold, 1);
  } else {
    atomic_set(&adc_temp_above_threshold, 0);
  }

  /* Check both thresholds and notify if needed */
  temp_check_thresholds();

  /* Put ADC to sleep to save power */
  adc_sleep();

  /* Schedule next measurement */
  temp_schedule_next_measurement();
}

static void temp_schedule_next_measurement(void)
{
  unsigned long interval = msecs_to_jiffies(measurement_period_ms);
  mod_timer(&temp_timer, jiffies + interval);
}

/* Check both ADC and comparator thresholds and notify userspace */
static void temp_check_thresholds(void)
{
  int adc_above = atomic_read(&adc_temp_above_threshold);
  int comp_triggered = atomic_read(&comparator_triggered);
  char msg[256];
  long temp_f = current_temperature_f;
  long ref_f = reference_temperature_f;
  int temp_int, temp_frac, ref_int, ref_frac;

  /* Convert for display */
  temp_int = temp_f / 1000;
  temp_frac = (temp_f % 1000) / 100;
  if (temp_frac < 0) temp_frac = -temp_frac;
  ref_int = ref_f / 1000;
  ref_frac = (ref_f % 1000) / 100;
  if (ref_frac < 0) ref_frac = -ref_frac;

  /* If either threshold is exceeded, check the other and notify */
  if (adc_above || comp_triggered) {
    /* Read current comparator state (may have changed) */
    int comp_current = gpio_get_value(COMPARATOR_GPIO);
    if (comp_current) {
      atomic_set(&comparator_triggered, 1);
    }

    /* Build notification message */
    snprintf(msg, sizeof(msg),
             "THRESHOLD_EXCEEDED: ADC_temp=%d.%d°F (ref=%d.%d°F, above=%d), "
             "Comparator=%d (GPIO=%d)\n",
             temp_int, temp_frac, ref_int, ref_frac, adc_above,
             comp_triggered, comp_current);

    temp_notify_userspace(msg);

    /* Reset comparator flag after notification */
    if (comp_triggered) {
      atomic_set(&comparator_triggered, 0);
    }
  }
}

/* Notify userspace of threshold events */
static void temp_notify_userspace(const char *message)
{
  /* Set data available flag for poll/select */
  atomic_set(&data_available, 1);
  
  /* Wake up any waiting processes */
  wake_up_interruptible(&temp_wait_queue);
  
  /* Also print to kernel log */
  printk(KERN_WARNING "mytempsensor: %s", message);
}

/* ========== Userspace interface ========== */

static int temp_build_status(char *dst, size_t maxlen)
{
  int n = 0;
  int adc_above = atomic_read(&adc_temp_above_threshold);
  int comp_triggered = atomic_read(&comparator_triggered);
  int comp_current = gpio_get_value(COMPARATOR_GPIO);
  int adc_awake_state = atomic_read(&adc_awake);
  long temp_f = current_temperature_f;
  long ref_f = reference_temperature_f;
  int temp_int, temp_frac, ref_int, ref_frac;

  /* Convert millidegrees to integer and fractional parts */
  temp_int = temp_f / 1000;
  temp_frac = (temp_f % 1000) / 100;  /* One decimal place */
  if (temp_frac < 0) temp_frac = -temp_frac;  /* Handle negative */
  
  ref_int = ref_f / 1000;
  ref_frac = (ref_f % 1000) / 100;
  if (ref_frac < 0) ref_frac = -ref_frac;

  /* Format status string using integer formatting */
  n = scnprintf(dst, maxlen,
      "temp=%d.%d°F ref_temp=%d.%d°F period=%lu ms "
      "adc_above=%d comparator=%d comparator_gpio=%d adc_awake=%d\n",
      temp_int, temp_frac, ref_int, ref_frac, measurement_period_ms,
      adc_above, comp_triggered, comp_current, adc_awake_state);
  return n;
}

/* Parse temperature from string (returns millidegrees) */
static int temp_parse_temp(const char *str, long *result)
{
  long int_part = 0;
  long frac_part = 0;
  int frac_digits = 0;
  int negative = 0;
  const char *p = str;
  
  if (!str || !result)
    return -1;
  
  /* Skip whitespace */
  while (*p == ' ' || *p == '\t')
    p++;
  
  /* Check for negative sign */
  if (*p == '-') {
    negative = 1;
    p++;
  } else if (*p == '+') {
    p++;
  }
  
  /* Parse integer part */
  if (*p < '0' || *p > '9')
    return -1;
  
  while (*p >= '0' && *p <= '9') {
    int_part = int_part * 10 + (*p - '0');
    p++;
  }
  
  /* Parse fractional part if decimal point exists */
  if (*p == '.') {
    p++;
    while (*p >= '0' && *p <= '9' && frac_digits < 3) {
      frac_part = frac_part * 10 + (*p - '0');
      frac_digits++;
      p++;
    }
  }
  
  /* Skip trailing whitespace */
  while (*p == ' ' || *p == '\t')
    p++;
  
  /* Must have consumed entire string */
  if (*p != '\0' && *p != '\n' && *p != '\r')
    return -1;
  
  /* Combine: convert to millidegrees */
  *result = int_part * 1000;
  if (frac_digits == 1) {
    *result += frac_part * 100;
  } else if (frac_digits == 2) {
    *result += frac_part * 10;
  } else if (frac_digits >= 3) {
    *result += frac_part;
  }
  
  if (negative)
    *result = -(*result);
  
  return 0;
}

/* Parse write commands from userspace:
 * Commands:
 *   "ref_temp=<F>" - set reference temperature in Fahrenheit
 *   "period=<ms>" - set measurement period in milliseconds
 */
static void temp_parse_write_command(const char *buf, size_t count)
{
  unsigned long val;
  long temp_val;

  if (strncmp(buf, "ref_temp=", 9) == 0) {
    /* Parse temperature value (returns millidegrees) */
    const char *num_str = buf + 9;
    
    if (temp_parse_temp(num_str, &temp_val) == 0 && 
        temp_val >= -459670 && temp_val <= 1000000) {
      reference_temperature_f = temp_val;
      printk(KERN_INFO "mytempsensor: reference temperature set to %ld.%ld°F\n",
             temp_val / 1000, (temp_val % 1000) / 100);
    } else {
      printk(KERN_WARNING "mytempsensor: invalid ref_temp value (range: -459.67 to 1000.0°F)\n");
    }
  } else if (strncmp(buf, "period=", 7) == 0) {
    if (kstrtoul(buf + 7, 10, &val) == 0 && val >= 100 && val <= 60000) {
      measurement_period_ms = val;
      printk(KERN_INFO "mytempsensor: measurement period set to %lu ms\n", measurement_period_ms);
      /* Reschedule timer with new period */
      del_timer_sync(&temp_timer);
      temp_schedule_next_measurement();
    } else {
      printk(KERN_WARNING "mytempsensor: invalid period value (range: 100-60000 ms)\n");
    }
  } else {
    printk(KERN_WARNING "mytempsensor: unknown command: %.*s\n", (int)count, buf);
    printk(KERN_INFO "mytempsensor: Valid commands: ref_temp=<F>, period=<ms>\n");
  }
}
