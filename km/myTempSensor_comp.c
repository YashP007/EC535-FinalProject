/* myTempSensor_comp.c — EC535 Final Project "Temperature Sensor" kernel module (Comparator-Only)
 *
 * Kernel module for BeagleBone Black to monitor temperature threshold using
 * hardware comparator only. This version does NOT use ADC - it only monitors
 * the comparator GPIO for threshold detection via interrupts and periodic polling.
 *
 * ============================================================================
 * USERSPACE INTERFACE DOCUMENTATION
 * ============================================================================
 *
 * Device File: /dev/mytempsensor_comp (major 63, minor 0)
 *
 * After loading the module, create the device file:
 *   sudo mknod /dev/mytempsensor_comp c 63 0
 *   sudo chmod 666 /dev/mytempsensor_comp
 *
 * ----------------------------------------------------------------------------
 * READING STATUS
 * ----------------------------------------------------------------------------
 *
 * Read current comparator status:
 *   cat /dev/mytempsensor_comp
 *
 * Output format:
 *   comparator=<0|1> comparator_gpio=<0|1> period=<ms> triggered=<0|1>
 *
 * Example output:
 *   comparator=1 comparator_gpio=1 period=1000 ms triggered=0
 *
 * ----------------------------------------------------------------------------
 * WRITING COMMANDS (Changing Parameters)
 * ----------------------------------------------------------------------------
 *
 * Write commands to /dev/mytempsensor_comp to change module parameters:
 *
 * 1. Set Measurement Period (Timer Interval):
 *    echo "period=<milliseconds>" > /dev/mytempsensor_comp
 *
 *    Parameters:
 *      <milliseconds> - Timer period in milliseconds for periodic checking
 *                       Range: 100 to 60000 ms
 *                       Default: 1000 ms (1 second)
 *
 *    Examples:
 *      echo "period=1000" > /dev/mytempsensor_comp    # 1 second intervals
 *      echo "period=500" > /dev/mytempsensor_comp     # 0.5 second intervals
 *      echo "period=5000" > /dev/mytempsensor_comp    # 5 second intervals
 *
 * ----------------------------------------------------------------------------
 * INTERRUPT-DRIVEN NOTIFICATIONS (poll/select)
 * ----------------------------------------------------------------------------
 *
 * The module supports poll()/select() for interrupt-driven notifications.
 * When the hardware comparator triggers (GPIO goes high), the module
 * immediately wakes up waiting processes.
 *
 * Usage in C program:
 *   #include <poll.h>
 *   
 *   int fd = open("/dev/mytempsensor_comp", O_RDONLY);
 *   struct pollfd pfd;
 *   pfd.fd = fd;
 *   pfd.events = POLLIN | POLLRDNORM;
 *   
 *   // Wait for comparator event (with timeout)
 *   int ret = poll(&pfd, 1, 5000);  // 5 second timeout
 *   
 *   if (ret > 0 && (pfd.revents & POLLIN)) {
 *       // Comparator triggered - read notification
 *       char buf[512];
 *       read(fd, buf, sizeof(buf));
 *       printf("Event: %s", buf);
 *   }
 *
 * ----------------------------------------------------------------------------
 * DEFAULT VALUES
 * ----------------------------------------------------------------------------
 *
 * - Measurement Period: 1000 ms (1 second)
 * - Comparator GPIO: 26
 *
 * ----------------------------------------------------------------------------
 * MODULE OPERATION
 * ----------------------------------------------------------------------------
 *
 * The module operates in two modes:
 *
 * 1. Hardware comparator interrupts:
 *    - Monitors GPIO 26 for hardware comparator signals (rising edge)
 *    - Immediately notifies userspace when triggered via interrupt
 *    - Sets triggered flag when interrupt occurs
 *
 * 2. Periodic polling:
 *    - Timer periodically checks comparator GPIO state
 *    - Updates status information
 *    - Can detect state changes between interrupts
 *
 * Both modes can trigger threshold notifications via poll()/select().
 * The comparator output directly indicates when temperature threshold is exceeded.
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

/* --- Includes: timing, IRQ, GPIO --- */
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/gpio.h>        /* legacy integer GPIO API */
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/poll.h>

MODULE_LICENSE("Dual BSD/GPL");
MODULE_DESCRIPTION("Temperature sensor kernel module for BBB (Comparator-Only)");
MODULE_AUTHOR("Yash Patel");

/* --- GPIO assignments (BBB header numbers -> kernel GPIO numbers) --- */
#define COMPARATOR_GPIO    26   /* GPIO pin for hardware comparator IRQ (active high) */

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

/* --- Forward declarations: GPIO and IRQ --- */
static int  temp_gpio_request_config(void);
static void temp_gpio_free(void);
static int  temp_irq_request(void);
static void temp_irq_free(void);

/* --- Forward declarations: timer and threshold checking --- */
static void temp_timer_callback(struct timer_list *t);
static void temp_schedule_next_check(void);
static void temp_check_comparator(void);
static void temp_notify_userspace(const char *message);

/* --- Forward declarations: userspace interface --- */
static int  temp_build_status(char *dst, size_t maxlen);
static void temp_parse_write_command(const char *buf, size_t count);

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
static int   mytempsensor_major = 63;    /* Use major 63, minor 0 */
static char *mytempsensor_buffer;         /* backing store for /dev reads/writes */
static int   mytempsensor_len;

/* --- Globals: IRQ bookkeeping --- */
static int comparator_irq = -1;          /* assigned by request_irq */

/* --- Globals: Comparator state --- */
static atomic_t comparator_triggered = ATOMIC_INIT(0);      /* Hardware comparator triggered */
static unsigned long check_period_ms = 1000;                 /* Timer period in milliseconds (default 1s) */

/* --- Globals: Wait queue for poll/select --- */
static DECLARE_WAIT_QUEUE_HEAD(temp_wait_queue);
static atomic_t data_available = ATOMIC_INIT(0);           /* New data available flag */

/* --- Timer for periodic checking --- */
static DEFINE_TIMER(temp_timer, temp_timer_callback);

/* ========== Module init/exit ========== */

static int mytempsensor_init(void)
{
  int result;

  /* Register character device */
  result = register_chrdev(mytempsensor_major, "mytempsensor_comp", &mytempsensor_fops);
  if (result < 0) {
    printk(KERN_ALERT "mytempsensor_comp: cannot obtain major %d\n", mytempsensor_major);
    return result;
  }

  /* Allocate kernel buffer */
  mytempsensor_buffer = kmalloc(capacity, GFP_KERNEL);
  if (!mytempsensor_buffer) {
    printk(KERN_ALERT "mytempsensor_comp: insufficient kernel memory\n");
    result = -ENOMEM;
    goto fail_chrdev;
  }
  memset(mytempsensor_buffer, 0, capacity);
  mytempsensor_len = 0;

  /* Request and configure GPIO for comparator */
  result = temp_gpio_request_config();
  if (result) {
    printk(KERN_ALERT "mytempsensor_comp: gpio request/config failed (%d)\n", result);
    goto fail_buf;
  }

  /* Request IRQ for comparator */
  result = temp_irq_request();
  if (result) {
    printk(KERN_ALERT "mytempsensor_comp: irq request failed (%d)\n", result);
    goto fail_gpio;
  }

  /* Start periodic timer for comparator checking */
  temp_schedule_next_check();

  printk(KERN_INFO "mytempsensor_comp: module inserted\n");
  printk(KERN_INFO "mytempsensor_comp: Device available at /dev/mytempsensor_comp\n");
  printk(KERN_INFO "mytempsensor_comp: Default check period: %lu ms\n", check_period_ms);
  printk(KERN_INFO "mytempsensor_comp: Commands: period=<ms>\n");
  return 0;

fail_gpio:
  temp_gpio_free();
fail_buf:
  kfree(mytempsensor_buffer);
fail_chrdev:
  unregister_chrdev(mytempsensor_major, "mytempsensor_comp");
  return result;
}

static void mytempsensor_exit(void)
{
  /* Stop timer before tearing down */
  del_timer_sync(&temp_timer);

  /* Free IRQ and GPIO */
  temp_irq_free();
  temp_gpio_free();

  /* Unregister char device */
  unregister_chrdev(mytempsensor_major, "mytempsensor_comp");

  /* Free buffer */
  kfree(mytempsensor_buffer);

  printk(KERN_INFO "mytempsensor_comp: module removed\n");
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
  /* Write path: accept commands to set check period */
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
  printk(KERN_ERR "mytempsensor_comp: gpio request/config error\n");
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
  
  /* Check comparator and notify */
  temp_check_comparator();
  
  return IRQ_HANDLED;
}

static int temp_irq_request(void)
{
  int ret;

  /* Map GPIO to IRQ number */
  comparator_irq = gpio_to_irq(COMPARATOR_GPIO);
  if (comparator_irq < 0) {
    printk(KERN_ERR "mytempsensor_comp: failed to get IRQ for GPIO %d\n", COMPARATOR_GPIO);
    return -EINVAL;
  }

  /* Request IRQ for rising edge (active high) */
  ret = request_irq(comparator_irq, comparator_isr, IRQF_TRIGGER_RISING,
                    "mytempsensor_comp_comparator", NULL);
  if (ret) {
    printk(KERN_ERR "mytempsensor_comp: failed to request IRQ %d\n", comparator_irq);
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
  /* Timer expired - check comparator state */
  temp_check_comparator();

  /* Schedule next check */
  temp_schedule_next_check();
}

static void temp_schedule_next_check(void)
{
  unsigned long interval = msecs_to_jiffies(check_period_ms);
  mod_timer(&temp_timer, jiffies + interval);
}

/* Check comparator state and notify userspace if triggered */
static void temp_check_comparator(void)
{
  int comp_triggered = atomic_read(&comparator_triggered);
  int comp_current = gpio_get_value(COMPARATOR_GPIO);
  char msg[256];

  /* If comparator is currently high, set triggered flag */
  if (comp_current) {
    atomic_set(&comparator_triggered, 1);
    comp_triggered = 1;
  }

  /* If comparator was triggered, notify userspace */
  if (comp_triggered) {
    /* Build notification message */
    snprintf(msg, sizeof(msg),
             "THRESHOLD_EXCEEDED: Comparator=%d (GPIO=%d, current=%d)\n",
             comp_triggered, COMPARATOR_GPIO, comp_current);

    temp_notify_userspace(msg);

    /* Reset comparator flag after notification */
    atomic_set(&comparator_triggered, 0);
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
  printk(KERN_WARNING "mytempsensor_comp: %s", message);
}

/* ========== Userspace interface ========== */

static int temp_build_status(char *dst, size_t maxlen)
{
  int n = 0;
  int comp_triggered = atomic_read(&comparator_triggered);
  int comp_current = gpio_get_value(COMPARATOR_GPIO);

  /* Format status string */
  n = scnprintf(dst, maxlen,
      "comparator=%d comparator_gpio=%d period=%lu ms triggered=%d\n",
      comp_current, COMPARATOR_GPIO, check_period_ms, comp_triggered);
  return n;
}

/* Parse write commands from userspace:
 * Commands:
 *   "period=<ms>" - set check period in milliseconds
 */
static void temp_parse_write_command(const char *buf, size_t count)
{
  unsigned long val;

  if (strncmp(buf, "period=", 7) == 0) {
    if (kstrtoul(buf + 7, 10, &val) == 0 && val >= 100 && val <= 60000) {
      check_period_ms = val;
      printk(KERN_INFO "mytempsensor_comp: check period set to %lu ms\n", check_period_ms);
      /* Reschedule timer with new period */
      del_timer_sync(&temp_timer);
      temp_schedule_next_check();
    } else {
      printk(KERN_WARNING "mytempsensor_comp: invalid period value (range: 100-60000 ms)\n");
    }
  } else {
    printk(KERN_WARNING "mytempsensor_comp: unknown command: %.*s\n", (int)count, buf);
    printk(KERN_INFO "mytempsensor_comp: Valid commands: period=<ms>\n");
  }
}

