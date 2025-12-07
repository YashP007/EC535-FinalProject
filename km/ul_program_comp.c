/* ul_program_comp.c — EC535 Final Project Userspace Program (Comparator-Only)
 *
 * Userspace program to interface with myTempSensor_comp.ko and mySignalLED.ko
 * - Monitors comparator state from hardware comparator
 * - Displays comparator status on screen
 * - Controls LED based on comparator state
 * - Handles timer and interrupt events from comparator module
 * - Allows user to set stove state (on/off) and timer period
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
#define TEMP_DEVICE    "/dev/mytempsensor_comp"
#define LED_DEVICE     "/dev/mysignal"

/* Buffer sizes */
#define BUFFER_SIZE    512
#define CMD_BUFFER     128

/* Global state */
static int temp_fd = -1;
static int led_fd = -1;
static volatile int running = 1;
static int comparator_state = 0;      /* Current comparator state (0=LOW, 1=HIGH) */
static int comparator_triggered = 0;  /* Flag if threshold was exceeded */
static unsigned long check_period = 1000;  /* Timer period in ms */
static int stove_state = 0;  /* 0 = off, 1 = on */

/* Function prototypes */
static int open_devices(void);
static void close_devices(void);
static int read_comparator_status(void);
static int update_led_temp_state(int comp_state);
static int set_stove_state(int on);
static int set_timer_period(unsigned long period);
static void *comparator_monitor_thread(void *arg);
static void *user_input_thread(void *arg);
static void signal_handler(int sig);
static void print_status(int comp_state, int stove_on, int triggered);
static int parse_comparator_status(const char *buf);
static void setup_terminal(void);
static void restore_terminal(void);

/* ========== Main program ========== */

int main(int argc, char *argv[])
{
    pthread_t comp_thread, input_thread;
    int ret;

    printf("EC535 Final Project - Stove Monitoring System (Comparator-Only)\n");
    printf("==============================================================\n\n");

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

    /* Read initial status */
    read_comparator_status();

    printf("Devices opened successfully\n");
    printf("Comparator GPIO: 26\n");
    printf("Current comparator state: %s\n", comparator_state ? "HIGH" : "LOW");
    printf("Check period: %lu ms\n", check_period);
    printf("\nCommands:\n");
    printf("  'on'      - Turn stove ON\n");
    printf("  'off'     - Turn stove OFF\n");
    printf("  'period=<ms>' - Set timer period (e.g., 'period=2000')\n");
    printf("  'status'  - Read current status\n");
    printf("  'q'       - Quit program\n");
    printf("\nStarting monitoring...\n");
    printf("Waiting for comparator events and timer updates...\n\n");

    /* Create comparator monitoring thread */
    ret = pthread_create(&comp_thread, NULL, comparator_monitor_thread, NULL);
    if (ret != 0) {
        fprintf(stderr, "Failed to create comparator thread: %s\n", strerror(ret));
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
    pthread_join(comp_thread, NULL);
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
    /* Open comparator sensor device (blocking mode for poll() to work with interrupts) */
    temp_fd = open(TEMP_DEVICE, O_RDWR);
    if (temp_fd < 0) {
        perror("Failed to open " TEMP_DEVICE);
        fprintf(stderr, "Make sure myTempSensor_comp.ko is loaded and device exists:\n");
        fprintf(stderr, "  sudo mknod %s c 63 0\n", TEMP_DEVICE);
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

/* ========== Comparator status reading and parsing ========== */

static int read_comparator_status(void)
{
    char buffer[BUFFER_SIZE];
    ssize_t n;

    if (temp_fd < 0)
        return -1;

    /* Seek to beginning */
    lseek(temp_fd, 0, SEEK_SET);

    /* Read status from device */
    n = read(temp_fd, buffer, sizeof(buffer) - 1);
    if (n < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK)
            perror("read comparator status");
        return -1;
    }

    if (n == 0)
        return -1;

    buffer[n] = '\0';

    /* Parse status string */
    return parse_comparator_status(buffer);
}

static int parse_comparator_status(const char *buf)
{
    /* Parse format: "comparator=1 comparator_gpio=26 period=1000 ms triggered=0" */
    const char *comp_str = strstr(buf, "comparator=");
    const char *period_str = strstr(buf, "period=");
    const char *triggered_str = strstr(buf, "triggered=");
    
    if (comp_str) {
        comp_str += 11; /* Skip "comparator=" */
        if (sscanf(comp_str, "%d", &comparator_state) != 1)
            return -1;
    } else {
        return -1;
    }
    
    if (period_str) {
        period_str += 7; /* Skip "period=" */
        if (sscanf(period_str, "%lu", &check_period) != 1)
            return -1;
    }
    
    if (triggered_str) {
        triggered_str += 10; /* Skip "triggered=" */
        if (sscanf(triggered_str, "%d", &comparator_triggered) != 1)
            return -1;
    }
    
    return 0;
}

/* ========== LED control ========== */

static int update_led_temp_state(int comp_state)
{
    char cmd[CMD_BUFFER];
    ssize_t n;

    if (led_fd < 0)
        return -1;

    /* Update LED based on comparator state */
    if (comp_state) {
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

/* ========== Timer period control ========== */

static int set_timer_period(unsigned long period)
{
    char cmd[CMD_BUFFER];
    ssize_t n;

    if (temp_fd < 0)
        return -1;

    if (period < 100 || period > 60000) {
        fprintf(stderr, "Invalid period: must be between 100 and 60000 ms\n");
        return -1;
    }

    snprintf(cmd, sizeof(cmd), "period=%lu", period);
    n = write(temp_fd, cmd, strlen(cmd));
    if (n < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK)
            perror("write timer period");
        return -1;
    }

    check_period = period;
    return 0;
}

/* ========== Thread functions ========== */

static void *comparator_monitor_thread(void *arg)
{
    struct pollfd pfd;
    char buffer[BUFFER_SIZE];
    time_t last_status_read = 0;
    const time_t status_interval = 2; /* Read status every 2 seconds */

    (void)arg;

    pfd.fd = temp_fd;
    pfd.events = POLLIN | POLLRDNORM;

    printf("[Monitor] Comparator monitor thread started\n");

    while (running) {
        int ret;
        time_t now;

        /* Poll for comparator events (interrupts and timer updates) */
        ret = poll(&pfd, 1, 1000); /* 1 second timeout */

        if (ret > 0 && (pfd.revents & (POLLIN | POLLRDNORM))) {
            /* Event occurred - read notification */
            ssize_t n = read(temp_fd, buffer, sizeof(buffer) - 1);
            if (n > 0) {
                buffer[n] = '\0';
                
                /* Check message type */
                if (strstr(buffer, "THRESHOLD_EXCEEDED") != NULL) {
                    /* Interrupt event - threshold exceeded */
                    printf("\n[!] THRESHOLD EXCEEDED - Comparator triggered!\n");
                    printf("    %s", buffer);
                    comparator_triggered = 1;
                    comparator_state = 1;
                    
                    /* Update LED to show threshold exceeded */
                    update_led_temp_state(1);
                    
                    /* Display updated status */
                    print_status(comparator_state, stove_state, comparator_triggered);
                } else if (strstr(buffer, "STATUS_UPDATE") != NULL) {
                    /* Timer update - periodic status */
                    printf("\n[Timer] %s", buffer);
                    
                    /* Parse the status update */
                    const char *state_str = strstr(buffer, "state=");
                    if (state_str) {
                        int new_state;
                        if (sscanf(state_str + 6, "%d", &new_state) == 1) {
                            if (new_state != comparator_state) {
                                comparator_state = new_state;
                                printf("    [State changed to: %s]\n", 
                                       comparator_state ? "HIGH" : "LOW");
                                
                                /* Update LED based on new state */
                                update_led_temp_state(comparator_state);
                            }
                        }
                    }
                    
                    /* Display updated status */
                    print_status(comparator_state, stove_state, comparator_triggered);
                } else {
                    /* Unknown message type */
                    printf("\n[Event] %s", buffer);
                }
            }
        }

        /* Periodically read status to keep display updated */
        now = time(NULL);
        if (now - last_status_read >= status_interval) {
            if (read_comparator_status() == 0) {
                /* Update LED state based on comparator */
                update_led_temp_state(comparator_state);
                
                /* Display status */
                print_status(comparator_state, stove_state, comparator_triggered);
                
                last_status_read = now;
            }
        }
    }

    printf("[Monitor] Comparator monitor thread stopped\n");
    return NULL;
}

static void *user_input_thread(void *arg)
{
    char input[CMD_BUFFER];
    int n;
    unsigned long period_val;

    (void)arg;

    printf("[Input] User input handler started\n");

    while (running) {
        /* Read user input */
        if (fgets(input, sizeof(input), stdin) != NULL) {
            /* Remove newline */
            n = strlen(input);
            if (n > 0 && input[n-1] == '\n')
                input[n-1] = '\0';

            /* Process commands */
            if (strcmp(input, "on") == 0 || strcmp(input, "ON") == 0) {
                if (set_stove_state(1) == 0) {
                    printf("[Command] Stove set to ON\n");
                    print_status(comparator_state, stove_state, comparator_triggered);
                } else {
                    printf("[Error] Failed to set stove state\n");
                }
            } else if (strcmp(input, "off") == 0 || strcmp(input, "OFF") == 0) {
                if (set_stove_state(0) == 0) {
                    printf("[Command] Stove set to OFF\n");
                    print_status(comparator_state, stove_state, comparator_triggered);
                } else {
                    printf("[Error] Failed to set stove state\n");
                }
            } else if (strncmp(input, "period=", 7) == 0) {
                if (sscanf(input + 7, "%lu", &period_val) == 1) {
                    if (set_timer_period(period_val) == 0) {
                        printf("[Command] Timer period set to %lu ms\n", period_val);
                    } else {
                        printf("[Error] Failed to set timer period\n");
                    }
                } else {
                    printf("[Error] Invalid period format. Use: period=<milliseconds>\n");
                }
            } else if (strcmp(input, "status") == 0 || strcmp(input, "STATUS") == 0) {
                if (read_comparator_status() == 0) {
                    printf("[Status] Comparator: %s | Period: %lu ms | Triggered: %d\n",
                           comparator_state ? "HIGH" : "LOW", 
                           check_period, 
                           comparator_triggered);
                    print_status(comparator_state, stove_state, comparator_triggered);
                } else {
                    printf("[Error] Failed to read status\n");
                }
            } else if (strcmp(input, "q") == 0 || strcmp(input, "Q") == 0 || 
                       strcmp(input, "quit") == 0) {
                printf("[Command] Quitting...\n");
                running = 0;
                break;
            } else if (strlen(input) > 0) {
                printf("[Error] Unknown command: '%s'\n", input);
                printf("Commands: 'on', 'off', 'period=<ms>', 'status', 'q'\n");
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

    printf("[Input] User input handler stopped\n");
    return NULL;
}

/* ========== Signal handling ========== */

static void signal_handler(int sig)
{
    (void)sig;
    printf("\n[Signal] Received signal, shutting down...\n");
    running = 0;
}

/* ========== Display functions ========== */

static void print_status(int comp_state, int stove_on, int triggered)
{
    static int first_call = 1;
    const char *stove_str = stove_on ? "ON " : "OFF";
    const char *comp_str = comp_state ? "HIGH" : "LOW";
    const char *trigger_str = triggered ? "TRIGGERED" : "normal";

    /* Clear line and move cursor to beginning (for updating display) */
    if (!first_call) {
        printf("\r\033[K"); /* Clear to end of line */
    }
    first_call = 0;

    printf("Comparator: %s [%s] | Stove: %s | Period: %lu ms",
           comp_str, trigger_str, stove_str, check_period);

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
