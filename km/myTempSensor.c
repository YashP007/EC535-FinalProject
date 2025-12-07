/* myTempSensor.c — EC535 Final Project "Temperature Sensor" kernel module
 *
 * Kernel module for BeagleBone Black to sense temperature from thermistor-based
 * resistive divider using BBB ADC. Features timer-based measurements and
 * hardware comparator IRQ for immediate threshold detection.
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
/* Simple natural logarithm approximation for kernel space */
static double simple_log(double x)
{
  /* Taylor series approximation: ln(1+x) ≈ x - x²/2 + x³/3 - ...
   * For better accuracy, we use: ln(x) = 2 * ((x-1)/(x+1) + ((x-1)/(x+1))³/3 + ...)
   * Simplified approximation for x > 0
   */
  double result = 0.0;
  double term;
  double power;
  double y;
  int i;
  
  if (x <= 0) return -1000.0; /* Invalid */
  if (x == 1.0) return 0.0;
  
  /* For values near 1, use direct approximation */
  if (x > 0.5 && x < 2.0) {
    y = (x - 1.0) / (x + 1.0);
    return 2.0 * (y + (y * y * y) / 3.0);
  }
  
  /* For other values, use iterative method or lookup */
  /* Simple approximation: ln(x) ≈ (x-1) - (x-1)²/2 for x near 1 */
  /* For general case, we'll use a polynomial approximation */
  term = (x - 1.0) / x;
  power = term;
  
  for (i = 1; i < 20; i++) {
    result += power / i;
    power *= term;
    if (power < 0.0001) break; /* Convergence */
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

/* --- Thermistor parameters (adjust for your thermistor) --- */
#define THERMISTOR_R_NOM   10000.0  /* Nominal resistance at 25°C (10kΩ) */
#define THERMISTOR_BETA    3950.0   /* Beta value (B25/85) - adjust for your thermistor */
#define THERMISTOR_T_NOM   25.0     /* Nominal temperature in °C */
#define DIVIDER_R          10000.0  /* Voltage divider resistor value (10kΩ) */

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
static double adc_voltage_to_resistance(int adc_raw);
static double resistance_to_temperature_c(double resistance);
static double celsius_to_fahrenheit(double celsius);
static double read_temperature_f(void);

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
static int  temp_parse_double(const char *str, double *result);

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

/* --- Globals: Temperature and threshold state --- */
static atomic_t adc_temp_above_threshold = ATOMIC_INIT(0);  /* ADC measurement threshold */
static atomic_t comparator_triggered = ATOMIC_INIT(0);      /* Hardware comparator triggered */
static double current_temperature_f = 0.0;                  /* Current temperature in Fahrenheit */
static double reference_temperature_f = 150.0;              /* Software reference temperature (°F) */
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
  printk(KERN_INFO "mytempsensor: Default reference temp: %.1f°F, period: %lu ms\n",
         reference_temperature_f, measurement_period_ms);
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

/* Convert ADC raw value to voltage, then to resistance */
static double adc_voltage_to_resistance(int adc_raw)
{
  double voltage_mv;
  double resistance;

  if (adc_raw < 0 || adc_raw > ADC_MAX_VALUE)
    return -1.0;

  /* Convert ADC reading to voltage (mV) */
  voltage_mv = ((double)adc_raw * ADC_VREF) / ADC_MAX_VALUE;

  /* Calculate thermistor resistance using voltage divider equation:
   * Vout = Vin * (R_thermistor / (R_thermistor + R_divider))
   * Solving for R_thermistor:
   * R_thermistor = R_divider * (Vin / Vout - 1)
   * But we have Vout, so: R_thermistor = R_divider * (Vref / Vout - 1)
   */
  if (voltage_mv > 0 && voltage_mv < ADC_VREF) {
    resistance = DIVIDER_R * ((ADC_VREF / voltage_mv) - 1.0);
  } else {
    return -1.0;
  }

  return resistance;
}

/* Convert resistance to temperature using Steinhart-Hart equation (simplified Beta equation) */
static double resistance_to_temperature_c(double resistance)
{
  double temp_kelvin, temp_celsius;
  double ln_r, beta_term;

  if (resistance <= 0)
    return -273.15; /* Invalid */

  /* Simplified Steinhart-Hart using Beta equation:
   * 1/T = 1/T0 + (1/B) * ln(R/R0)
   * Where:
   * T = temperature in Kelvin
   * T0 = reference temperature (25°C = 298.15K)
   * B = Beta value
   * R = measured resistance
   * R0 = reference resistance at T0
   */
  ln_r = simple_log(resistance / THERMISTOR_R_NOM);
  beta_term = (1.0 / THERMISTOR_BETA) * ln_r;
  temp_kelvin = 1.0 / ((1.0 / (THERMISTOR_T_NOM + 273.15)) + beta_term);
  temp_celsius = temp_kelvin - 273.15;

  return temp_celsius;
}

/* Convert Celsius to Fahrenheit */
static double celsius_to_fahrenheit(double celsius)
{
  return (celsius * 9.0 / 5.0) + 32.0;
}

/* Read temperature in Fahrenheit */
static double read_temperature_f(void)
{
  int adc_raw;
  double resistance, temp_c, temp_f;

  /* Check if ADC is awake */
  if (!atomic_read(&adc_awake)) {
    adc_wake();
    msleep(10); /* Small delay for ADC to stabilize */
  }

  /* Read ADC */
  adc_raw = adc_read_raw();
  if (adc_raw < 0) {
    printk(KERN_WARNING "mytempsensor: ADC read failed\n");
    return -459.67; /* Absolute zero in Fahrenheit */
  }

  /* Convert to resistance */
  resistance = adc_voltage_to_resistance(adc_raw);
  if (resistance < 0) {
    printk(KERN_WARNING "mytempsensor: invalid resistance value\n");
    return -459.67;
  }

  /* Convert to temperature */
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
  double temp_f;

  /* Wake ADC if sleeping */
  if (!atomic_read(&adc_awake)) {
    adc_wake();
    msleep(10); /* Small delay for ADC to stabilize */
  }

  /* Read temperature */
  temp_f = read_temperature_f();
  current_temperature_f = temp_f;

  /* Check if above software threshold */
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

  /* If either threshold is exceeded, check the other and notify */
  if (adc_above || comp_triggered) {
    /* Read current comparator state (may have changed) */
    int comp_current = gpio_get_value(COMPARATOR_GPIO);
    if (comp_current) {
      atomic_set(&comparator_triggered, 1);
    }

    /* Build notification message */
    snprintf(msg, sizeof(msg),
             "THRESHOLD_EXCEEDED: ADC_temp=%.1f°F (ref=%.1f°F, above=%d), "
             "Comparator=%d (GPIO=%d)\n",
             current_temperature_f, reference_temperature_f, adc_above,
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

  /* Format status string */
  n = scnprintf(dst, maxlen,
      "temp=%.1f°F ref_temp=%.1f°F period=%lu ms "
      "adc_above=%d comparator=%d comparator_gpio=%d adc_awake=%d\n",
      current_temperature_f, reference_temperature_f, measurement_period_ms,
      adc_above, comp_triggered, comp_current, adc_awake_state);
  return n;
}

/* Simple double parser for kernel space (replaces kstrtod which doesn't exist) */
static int temp_parse_double(const char *str, double *result)
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
    while (*p >= '0' && *p <= '9' && frac_digits < 10) {
      frac_part = frac_part * 10 + (*p - '0');
      frac_digits++;
      p++;
    }
  }
  
  /* Skip trailing whitespace */
  while (*p == ' ' || *p == '\t')
    p++;
  
  /* Must have consumed entire string (or reached end) */
  if (*p != '\0' && *p != '\n' && *p != '\r')
    return -1;
  
  /* Combine integer and fractional parts */
  *result = (double)int_part;
  if (frac_digits > 0) {
    double frac_divisor = 1.0;
    int i;
    for (i = 0; i < frac_digits; i++)
      frac_divisor *= 10.0;
    *result += (double)frac_part / frac_divisor;
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
  double temp_val;

  if (strncmp(buf, "ref_temp=", 9) == 0) {
    /* Parse double value using manual parser */
    const char *num_str = buf + 9;
    
    if (temp_parse_double(num_str, &temp_val) == 0 && temp_val >= -459.67 && temp_val <= 1000.0) {
      reference_temperature_f = temp_val;
      printk(KERN_INFO "mytempsensor: reference temperature set to %.1f°F\n", reference_temperature_f);
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
