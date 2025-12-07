/* ul_program.c — EC535 Final Project Userspace Program
 *
 * Userspace program to interface with myTempSensor.ko and mySignalLED.ko
 * - Reads temperature from sensor
 * - Displays temperature on screen
 * - Controls LED based on temperature
 * - Handles timer and interrupt events from temperature sensor
 * - Allows user to set stove state (on/off)
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <poll.h>
#include <pthread.h>
#include <signal.h>
#include <time.h>
#include <sys/select.h>
#include <termios.h>

/* Device file paths */
#define TEMP_DEVICE    "/dev/mytempsensor"
#define LED_DEVICE     "/dev/mysignal"

/* Buffer sizes */
#define BUFFER_SIZE    512
#define CMD_BUFFER     128

/* Temperature threshold for LED control (Fahrenheit) */
#define DEFAULT_TEMP_THRESHOLD  150.0

/* Global state */
static int temp_fd = -1;
static int led_fd = -1;
static volatile int running = 1;
static double current_temp = 0.0;
static double temp_threshold = DEFAULT_TEMP_THRESHOLD;
static int stove_state = 0;  /* 0 = off, 1 = on */

/* Function prototypes */
static int open_devices(void);
static void close_devices(void);
static int read_temperature(double *temp);
static int update_led_temp_state(double temp);
static int set_stove_state(int on);
static void *temperature_monitor_thread(void *arg);
static void *user_input_thread(void *arg);
static void signal_handler(int sig);
static void print_status(double temp, int stove_on);
static int parse_temp_status(const char *buf, double *temp);
static void setup_terminal(void);
static void restore_terminal(void);

/* ========== Main program ========== */

int main(int argc, char *argv[])
{
    pthread_t temp_thread, input_thread;
    int ret;

    printf("EC535 Final Project - Stove Monitoring System\n");
    printf("=============================================\n\n");

    /* Setup signal handlers for graceful shutdown */
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    /* Open device files */
    if (open_devices() < 0) {
        fprintf(stderr, "Failed to open device files\n");
        return 1;
    }

    /* Setup terminal for non-blocking input */
    setup_terminal();

    printf("Devices opened successfully\n");
    printf("Temperature threshold: %.1f°F\n", temp_threshold);
    printf("\nCommands:\n");
    printf("  'on'  - Turn stove ON\n");
    printf("  'off' - Turn stove OFF\n");
    printf("  'q'   - Quit program\n");
    printf("\nStarting monitoring...\n\n");

    /* Create temperature monitoring thread */
    ret = pthread_create(&temp_thread, NULL, temperature_monitor_thread, NULL);
    if (ret != 0) {
        fprintf(stderr, "Failed to create temperature thread: %s\n", strerror(ret));
        goto cleanup;
    }

    /* Create user input thread */
    ret = pthread_create(&input_thread, NULL, user_input_thread, NULL);
    if (ret != 0) {
        fprintf(stderr, "Failed to create input thread: %s\n", strerror(ret));
        running = 0;
        goto cleanup;
    }

    /* Wait for threads to complete */
    pthread_join(temp_thread, NULL);
    pthread_join(input_thread, NULL);

cleanup:
    restore_terminal();
    close_devices();
    printf("\nProgram terminated.\n");
    return 0;
}

/* ========== Device management ========== */

static int open_devices(void)
{
    /* Open temperature sensor device (blocking mode for poll() to work with interrupts) */
    temp_fd = open(TEMP_DEVICE, O_RDWR);
    if (temp_fd < 0) {
        perror("Failed to open " TEMP_DEVICE);
        fprintf(stderr, "Make sure myTempSensor.ko is loaded and device exists:\n");
        fprintf(stderr, "  sudo mknod %s c 62 0\n", TEMP_DEVICE);
        fprintf(stderr, "  sudo chmod 666 %s\n", TEMP_DEVICE);
        return -1;
    }

    /* Open LED control device (non-blocking for writes) */
    led_fd = open(LED_DEVICE, O_RDWR | O_NONBLOCK);
    if (led_fd < 0) {
        perror("Failed to open " LED_DEVICE);
        fprintf(stderr, "Make sure mySignalLED.ko is loaded and device exists:\n");
        fprintf(stderr, "  sudo mknod %s c 61 0\n", LED_DEVICE);
        fprintf(stderr, "  sudo chmod 666 %s\n", LED_DEVICE);
        close(temp_fd);
        temp_fd = -1;
        return -1;
    }

    return 0;
}

static void close_devices(void)
{
    if (temp_fd >= 0) {
        close(temp_fd);
        temp_fd = -1;
    }
    if (led_fd >= 0) {
        close(led_fd);
        led_fd = -1;
    }
}

/* ========== Temperature reading and parsing ========== */

static int read_temperature(double *temp)
{
    char buffer[BUFFER_SIZE];
    ssize_t n;
    int ret;

    if (temp_fd < 0 || !temp)
        return -1;

    /* Seek to beginning */
    lseek(temp_fd, 0, SEEK_SET);

    /* Read status from device */
    n = read(temp_fd, buffer, sizeof(buffer) - 1);
    if (n < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK)
            perror("read temperature");
        return -1;
    }

    if (n == 0)
        return -1;

    buffer[n] = '\0';

    /* Parse temperature from status string */
    ret = parse_temp_status(buffer, temp);
    return ret;
}

static int parse_temp_status(const char *buf, double *temp)
{
    /* Parse format: "temp=75.5°F ref_temp=150.0°F period=1000 ms ..." */
    const char *temp_str = strstr(buf, "temp=");
    if (!temp_str)
        return -1;

    temp_str += 5; /* Skip "temp=" */
    
    /* Parse temperature value */
    if (sscanf(temp_str, "%lf", temp) != 1)
        return -1;

    return 0;
}

/* ========== LED control ========== */

static int update_led_temp_state(double temp)
{
    char cmd[CMD_BUFFER];
    ssize_t n;

    if (led_fd < 0)
        return -1;

    /* Update LED based on temperature threshold */
    if (temp >= temp_threshold) {
        snprintf(cmd, sizeof(cmd), "temp_above");
    } else {
        snprintf(cmd, sizeof(cmd), "temp_below");
    }

    n = write(led_fd, cmd, strlen(cmd));
    if (n < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK)
            perror("write LED temp state");
        return -1;
    }

    return 0;
}

static int set_stove_state(int on)
{
    char cmd[CMD_BUFFER];
    ssize_t n;

    if (led_fd < 0)
        return -1;

    if (on) {
        snprintf(cmd, sizeof(cmd), "stove_on");
        stove_state = 1;
    } else {
        snprintf(cmd, sizeof(cmd), "stove_off");
        stove_state = 0;
    }

    n = write(led_fd, cmd, strlen(cmd));
    if (n < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK)
            perror("write stove state");
        return -1;
    }

    return 0;
}

/* ========== Thread functions ========== */

static void *temperature_monitor_thread(void *arg)
{
    struct pollfd pfd;
    char buffer[BUFFER_SIZE];
    double temp;
    time_t last_display = 0;
    const time_t display_interval = 1; /* Update display every 1 second */

    (void)arg;

    pfd.fd = temp_fd;
    pfd.events = POLLIN | POLLRDNORM;

    printf("Temperature Monitor: Started\n");

    while (running) {
        int ret;
        time_t now;

        /* Poll for temperature threshold events (interrupts) */
        ret = poll(&pfd, 1, 1000); /* 1 second timeout */

        if (ret > 0 && (pfd.revents & (POLLIN | POLLRDNORM))) {
            /* Threshold event occurred - read notification */
            ssize_t n = read(temp_fd, buffer, sizeof(buffer) - 1);
            if (n > 0) {
                buffer[n] = '\0';
                printf("\n[THRESHOLD EVENT] %s", buffer);
            }
        }

        /* Periodically read temperature and update display */
        now = time(NULL);
        if (now - last_display >= display_interval) {
            if (read_temperature(&temp) == 0) {
                current_temp = temp;
                
                /* Update LED state based on temperature */
                update_led_temp_state(temp);
                
                /* Display status */
                print_status(temp, stove_state);
                
                last_display = now;
            }
        }
    }

    printf("Temperature Monitor: Stopped\n");
    return NULL;
}

static void *user_input_thread(void *arg)
{
    char input[CMD_BUFFER];
    int n;

    (void)arg;

    printf("User Input Handler: Started\n");

    while (running) {
        /* Read user input (non-blocking) */
        if (fgets(input, sizeof(input), stdin) != NULL) {
            /* Remove newline */
            n = strlen(input);
            if (n > 0 && input[n-1] == '\n')
                input[n-1] = '\0';

            /* Process commands */
            if (strcmp(input, "on") == 0 || strcmp(input, "ON") == 0) {
                if (set_stove_state(1) == 0) {
                    printf("Stove set to ON\n");
                }
            } else if (strcmp(input, "off") == 0 || strcmp(input, "OFF") == 0) {
                if (set_stove_state(0) == 0) {
                    printf("Stove set to OFF\n");
                }
            } else if (strcmp(input, "q") == 0 || strcmp(input, "Q") == 0 || 
                       strcmp(input, "quit") == 0) {
                printf("Quitting...\n");
                running = 0;
                break;
            } else if (strlen(input) > 0) {
                printf("Unknown command: '%s'\n", input);
                printf("Commands: 'on', 'off', 'q'\n");
            }
        } else {
            /* Check if stdin was closed or error occurred */
            if (feof(stdin)) {
                running = 0;
                break;
            }
            usleep(100000); /* 100ms sleep to avoid busy-waiting */
        }
    }

    printf("User Input Handler: Stopped\n");
    return NULL;
}

/* ========== Signal handling ========== */

static void signal_handler(int sig)
{
    (void)sig;
    printf("\nReceived signal, shutting down...\n");
    running = 0;
}

/* ========== Display functions ========== */

static void print_status(double temp, int stove_on)
{
    static int first_call = 1;
    const char *stove_str = stove_on ? "ON " : "OFF";
    const char *temp_status = (temp >= temp_threshold) ? "ABOVE" : "BELOW";

    /* Clear line and move cursor to beginning (for updating display) */
    if (!first_call) {
        printf("\r\033[K"); /* Clear to end of line */
    }
    first_call = 0;

    printf("Temperature: %6.1f°F [%s threshold] | Stove: %s | LED: %s",
           temp, temp_status, stove_str, temp_status);

    fflush(stdout);
}

/* ========== Terminal setup ========== */

static struct termios old_termios;

static void setup_terminal(void)
{
    struct termios new_termios;

    /* Save old terminal settings */
    if (tcgetattr(STDIN_FILENO, &old_termios) < 0) {
        perror("tcgetattr");
        return;
    }

    /* Get new terminal settings */
    new_termios = old_termios;

    /* Disable canonical mode and echo (for better input handling) */
    new_termios.c_lflag &= ~(ICANON | ECHO);
    new_termios.c_cc[VMIN] = 0;
    new_termios.c_cc[VTIME] = 0;

    /* Apply new settings */
    if (tcsetattr(STDIN_FILENO, TCSANOW, &new_termios) < 0) {
        perror("tcsetattr");
    }
}

static void restore_terminal(void)
{
    /* Restore old terminal settings */
    if (tcsetattr(STDIN_FILENO, TCSANOW, &old_termios) < 0) {
        perror("tcsetattr restore");
    }
}

