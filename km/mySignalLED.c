/* mySignalLED.c — EC535 Final Project "Stove Monitoring LED" kernel module
 *
 * Kernel module to control an RGB LED for embedded stove monitoring system.
 * Displays three states: server connection (green), stove on/off (blue), 
 * temperature threshold (red).
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

/* --- Includes: timing, GPIO --- */
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/gpio.h>        /* legacy integer GPIO API */
#include <linux/delay.h>
#include <linux/string.h>

MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("Yash Patel");
MODULE_DESCRIPTION("Kernel module to control an RGB LED for embedded stove monitoring system.");

/* --- GPIO assignments (BBB header numbers -> kernel GPIO numbers)
 * TODO: verify these numeric GPIO IDs against your wiring.
 */
#define RED     67   /* Temperature threshold (above/below) */
#define GREEN   44   /* Server connection (connected/disconnected) */
#define BLUE    68   /* Stove on/off (was YELLOW, now BLUE) */

/* --- Buffer sizing for char device I/O --- */
enum { capacity = 4096, bite = 128 };

/* --- Forward declarations: file ops --- */
static int     mysignal_open(struct inode *inode, struct file *filp);
static int     mysignal_release(struct inode *inode, struct file *filp);
static ssize_t mysignal_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos);
static ssize_t mysignal_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos);

/* --- Forward declarations: module init/exit --- */
static int  mysignal_init(void);
static void mysignal_exit(void);

/* --- Forward declarations: LED control and state machine --- */
static int  led_gpio_request_config(void);
static void led_gpio_free(void);
static void led_all_off(void);
static void led_set_red(int on);
static void led_set_green(int on);
static void led_set_blue(int on);
static void led_set_all(int on);

static void led_tick(struct timer_list *t);
static void led_schedule_next_tick(unsigned long msec);
static void led_apply_state_step(void);

/* --- Forward declarations: userspace interface --- */
static int  led_build_status(char *dst, size_t maxlen);
static void led_parse_write_command(const char *buf, size_t count);

/* --- File operations table --- */
static struct file_operations mysignal_fops = {
  .read    = mysignal_read,
  .write   = mysignal_write,
  .open    = mysignal_open,
  .release = mysignal_release,
};

/* --- Module entry/exit wiring --- */
module_init(mysignal_init);
module_exit(mysignal_exit);

/* --- Globals: char device bookkeeping --- */
static int   mysignal_major = 61;    /* Use major 61, minor 0 */
static char *mysignal_buffer;         /* backing store for /dev reads/writes */
static int   mysignal_len;

/* --- Globals: Stove state variables --- */
static atomic_t stove_on = ATOMIC_INIT(0);           /* 1 = stove on, 0 = stove off */
static atomic_t temp_above_threshold = ATOMIC_INIT(0); /* 1 = above, 0 = below */
static atomic_t server_connected = ATOMIC_INIT(0);    /* 1 = connected, 0 = disconnected */

/* --- Globals: Timing configuration (in milliseconds) --- */
static unsigned long color_duration_ms = 1000;  /* Default: 1 second per color */
static unsigned long sleep_duration_ms = 2000;  /* Default: 2 seconds sleep between cycles */

/* --- State machine state --- */
typedef enum {
  STATE_SLEEP = 0,
  STATE_SHOW_GREEN,
  STATE_SHOW_BLUE,
  STATE_SHOW_RED,
  STATE_SHOW_ALL  /* Special case: show all when green is off */
} led_state_t;

static led_state_t current_state = STATE_SLEEP;
static unsigned long state_start_jiffies = 0;  /* When current state started */

/* --- Timer for state machine --- */
static DEFINE_TIMER(led_timer, led_tick);

/* ========== Module init/exit ========== */

static int mysignal_init(void)
{
  int result;

  /* Register character device: creates major number for /dev/mysignal */
  result = register_chrdev(mysignal_major, "mysignal", &mysignal_fops);
  if (result < 0) {
    printk(KERN_ALERT "mysignal: cannot obtain major %d\n", mysignal_major);
    return result;
  }

  /* Allocate kernel buffer for read/write */
  mysignal_buffer = kmalloc(capacity, GFP_KERNEL);
  if (!mysignal_buffer) {
    printk(KERN_ALERT "mysignal: insufficient kernel memory\n");
    result = -ENOMEM;
    goto fail_chrdev;
  }
  memset(mysignal_buffer, 0, capacity);
  mysignal_len = 0;

  /* Request and configure GPIOs */
  result = led_gpio_request_config();
  if (result) {
    printk(KERN_ALERT "mysignal: gpio request/config failed (%d)\n", result);
    goto fail_buf;
  }

  /* Initialize LEDs to off */
  led_all_off();

  /* Start state machine */
  current_state = STATE_SLEEP;
  state_start_jiffies = jiffies;
  led_schedule_next_tick(sleep_duration_ms);

  printk(KERN_INFO "mysignal: module inserted (color_duration=%lu ms, sleep_duration=%lu ms)\n",
         color_duration_ms, sleep_duration_ms);
  printk(KERN_INFO "mysignal: Device available at /dev/mysignal\n");
  printk(KERN_INFO "mysignal: Commands: stove_on/off, temp_above/below, server_connected/disconnected\n");
  printk(KERN_INFO "mysignal: Timing: color_duration=<ms>, sleep_duration=<ms>\n");
  printk(KERN_INFO "mysignal: Read status: cat /dev/mysignal\n");
  return 0;

fail_buf:
  kfree(mysignal_buffer);
fail_chrdev:
  unregister_chrdev(mysignal_major, "mysignal");
  return result;
}

static void mysignal_exit(void)
{
  /* Stop timer before tearing down GPIO */
  del_timer_sync(&led_timer);

  /* Free GPIOs */
  led_gpio_free();

  /* Unregister char device major */
  unregister_chrdev(mysignal_major, "mysignal");

  /* Free buffer */
  kfree(mysignal_buffer);

  printk(KERN_INFO "mysignal: module removed\n");
}

/* ========== File operations ========== */

static int mysignal_open(struct inode *inode, struct file *filp)
{
  return 0;
}

static int mysignal_release(struct inode *inode, struct file *filp)
{
  return 0;
}

static ssize_t mysignal_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
  /* Return a human-readable snapshot of status */
  char tbuf[256];
  int n;

  /* Build fresh status string each read start */
  if (*f_pos == 0) {
    n = led_build_status(tbuf, sizeof(tbuf));
    n = min(n, (int)capacity);
    memcpy(mysignal_buffer, tbuf, n);
    mysignal_len = n;
  }

  /* EOF if f_pos at end */
  if (*f_pos >= mysignal_len)
    return 0;

  /* Do not exceed tail or user's count */
  if (count > mysignal_len - *f_pos)
    count = mysignal_len - *f_pos;
  if (count > bite)
    count = bite;

  if (copy_to_user(buf, mysignal_buffer + *f_pos, count))
    return -EFAULT;

  *f_pos += count;
  return count;
}

static ssize_t mysignal_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos)
{
  /* Write path: accept commands to set states and timing */
  char kbuf[128];
  size_t c = min(count, sizeof(kbuf) - 1);

  if (c == 0)
    return 0;

  if (copy_from_user(kbuf, buf, c))
    return -EFAULT;

  kbuf[c] = '\0';

  /* Parse command */
  led_parse_write_command(kbuf, c);

  return count;
}

/* ========== GPIO implementation ========== */

static int led_gpio_request_config(void)
{
  int ret = 0;

  /* Request LEDs as outputs, default LOW */
  ret |= gpio_request(RED,   "signal_red");
  ret |= gpio_request(GREEN, "signal_green");
  ret |= gpio_request(BLUE,  "signal_blue");
  if (ret) goto err;

  ret |= gpio_direction_output(RED,   0);
  ret |= gpio_direction_output(GREEN, 0);
  ret |= gpio_direction_output(BLUE,  0);
  if (ret) goto err;

  return 0;

err:
  printk(KERN_ERR "mysignal: gpio request/config error\n");
  return ret ?: -EINVAL;
}

static void led_gpio_free(void)
{
  /* Free in reverse order; ensure LEDs off */
  led_all_off();
  gpio_free(BLUE);
  gpio_free(GREEN);
  gpio_free(RED);
}

/* ========== LED control helpers ========== */

static void led_all_off(void)
{
  gpio_set_value(RED,   0);
  gpio_set_value(GREEN, 0);
  gpio_set_value(BLUE,  0);
}

static void led_set_red(int on)    { gpio_set_value(RED,   !!on); }
static void led_set_green(int on)  { gpio_set_value(GREEN, !!on); }
static void led_set_blue(int on)   { gpio_set_value(BLUE,  !!on); }

static void led_set_all(int on)
{
  gpio_set_value(RED,   !!on);
  gpio_set_value(GREEN, !!on);
  gpio_set_value(BLUE,  !!on);
}

/* ========== Timer + state machine ========== */

static void led_schedule_next_tick(unsigned long msec)
{
  unsigned long interval = msecs_to_jiffies(msec);
  mod_timer(&led_timer, jiffies + interval);
}

/* Periodic callback: advances state machine */
static void led_tick(struct timer_list *t)
{
  led_apply_state_step();
}

/* State machine implementation:
 * Sequence: SLEEP -> (if green off: SHOW_ALL) -> SHOW_GREEN -> SHOW_BLUE -> SHOW_RED -> SLEEP
 * Each color shown for color_duration_ms, sleep for sleep_duration_ms
 */
static void led_apply_state_step(void)
{
  int server_conn = atomic_read(&server_connected);
  int stove_state = atomic_read(&stove_on);
  int temp_state = atomic_read(&temp_above_threshold);

  switch (current_state) {
    case STATE_SLEEP:
      /* Sleep period complete, start showing colors */
      led_all_off();
      if (!server_conn) {
        /* Special case: if green (connected) is off, show all colors first */
        current_state = STATE_SHOW_ALL;
        led_set_all(1);
      } else {
        /* Normal case: start with green (show server connection state) */
        current_state = STATE_SHOW_GREEN;
        if (server_conn) {
          led_set_green(1);
        } else {
          led_set_green(0);
        }
      }
      state_start_jiffies = jiffies;
      led_schedule_next_tick(color_duration_ms);
      break;

    case STATE_SHOW_ALL:
      /* Show all colors complete, now show individual states */
      led_all_off();
      current_state = STATE_SHOW_BLUE;
      if (stove_state) {
        led_set_blue(1);
      } else {
        /* Stove is off, keep LED off */
        led_set_blue(0);
      }
      state_start_jiffies = jiffies;
      led_schedule_next_tick(color_duration_ms);
      break;

    case STATE_SHOW_GREEN:
      /* Green (server connection) complete */
      led_all_off();
      current_state = STATE_SHOW_BLUE;
      if (stove_state) {
        led_set_blue(1);
      } else {
        /* Stove is off, keep LED off */
        led_set_blue(0);
      }
      state_start_jiffies = jiffies;
      led_schedule_next_tick(color_duration_ms);
      break;

    case STATE_SHOW_BLUE:
      /* Blue (stove on/off) complete */
      led_all_off();
      current_state = STATE_SHOW_RED;
      if (temp_state) {
        led_set_red(1);
      } else {
        led_set_red(0);
      }
      state_start_jiffies = jiffies;
      led_schedule_next_tick(color_duration_ms);
      break;

    case STATE_SHOW_RED:
      /* Red (temperature) complete, go to sleep */
      led_all_off();
      current_state = STATE_SLEEP;
      state_start_jiffies = jiffies;
      led_schedule_next_tick(sleep_duration_ms);
      break;

    default:
      /* Reset to sleep */
      led_all_off();
      current_state = STATE_SLEEP;
      state_start_jiffies = jiffies;
      led_schedule_next_tick(sleep_duration_ms);
      break;
  }
}

/* ========== Userspace interface ========== */

static int led_build_status(char *dst, size_t maxlen)
{
  int n = 0;
  int r = gpio_get_value(RED);
  int g = gpio_get_value(GREEN);
  int b = gpio_get_value(BLUE);
  int stove = atomic_read(&stove_on);
  int temp = atomic_read(&temp_above_threshold);
  int server = atomic_read(&server_connected);

  /* Format status string */
  n = scnprintf(dst, maxlen,
      "stove=%s temp=%s server=%s color_duration=%lu sleep_duration=%lu "
      "leds={red:%s,green:%s,blue:%s} state=%d\n",
      stove ? "on" : "off",
      temp ? "above" : "below",
      server ? "connected" : "disconnected",
      color_duration_ms, sleep_duration_ms,
      r ? "on" : "off", g ? "on" : "off", b ? "on" : "off",
      current_state);
  return n;
}

/* Parse write commands from userspace:
 * Commands:
 *   "stove_on" / "stove_off" - set stove state
 *   "temp_above" / "temp_below" - set temperature state
 *   "server_connected" / "server_disconnected" - set server connection state
 *   "color_duration=<ms>" - set color display duration in milliseconds
 *   "sleep_duration=<ms>" - set sleep duration in milliseconds
 */
static void led_parse_write_command(const char *buf, size_t count)
{
  if (strncmp(buf, "stove_on", 8) == 0) {
    atomic_set(&stove_on, 1);
    printk(KERN_INFO "mysignal: stove set to ON\n");
  } else if (strncmp(buf, "stove_off", 9) == 0) {
    atomic_set(&stove_on, 0);
    printk(KERN_INFO "mysignal: stove set to OFF\n");
  } else if (strncmp(buf, "temp_above", 10) == 0) {
    atomic_set(&temp_above_threshold, 1);
    printk(KERN_INFO "mysignal: temperature set to ABOVE threshold\n");
  } else if (strncmp(buf, "temp_below", 10) == 0) {
    atomic_set(&temp_above_threshold, 0);
    printk(KERN_INFO "mysignal: temperature set to BELOW threshold\n");
  } else if (strncmp(buf, "server_connected", 16) == 0) {
    atomic_set(&server_connected, 1);
    printk(KERN_INFO "mysignal: server set to CONNECTED\n");
  } else if (strncmp(buf, "server_disconnected", 19) == 0) {
    atomic_set(&server_connected, 0);
    printk(KERN_INFO "mysignal: server set to DISCONNECTED\n");
  } else if (strncmp(buf, "color_duration=", 16) == 0) {
    unsigned long val;
    if (kstrtoul(buf + 16, 10, &val) == 0 && val > 0 && val <= 10000) {
      color_duration_ms = val;
      printk(KERN_INFO "mysignal: color_duration set to %lu ms\n", color_duration_ms);
    } else {
      printk(KERN_WARNING "mysignal: invalid color_duration value\n");
    }
  } else if (strncmp(buf, "sleep_duration=", 16) == 0) {
    unsigned long val;
    if (kstrtoul(buf + 16, 10, &val) == 0 && val > 0 && val <= 60000) {
      sleep_duration_ms = val;
      printk(KERN_INFO "mysignal: sleep_duration set to %lu ms\n", sleep_duration_ms);
    } else {
      printk(KERN_WARNING "mysignal: invalid sleep_duration value\n");
    }
  } else {
    printk(KERN_WARNING "mysignal: unknown command: %.*s\n", (int)count, buf);
    printk(KERN_INFO "mysignal: Valid commands: stove_on, stove_off, temp_above, temp_below, ");
    printk(KERN_CONT "server_connected, server_disconnected, color_duration=<ms>, sleep_duration=<ms>\n");
  }
}
