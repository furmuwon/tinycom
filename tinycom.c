/*
 * ----------------------------------------------------------------------------
 * "THE BEER-WARE LICENSE" (Revision 42):
 * JongHo Kim <furmuwon@gmail.com> wrote this file. As long as you retain 
 * this notice you can do whatever you want with this stuff. If we meet some day, 
 * and you think this stuff is worth it, you can buy me a beer in return. 
 * ----------------------------------------------------------------------------
 */
 
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <poll.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>
#include <pthread.h>
#include <signal.h>
#include <errno.h>

#define RBUF_SIZE 64
#define USE_SIGHANDLER 0

int program_out = 0;
static void sighandler(int sig)
{
    if (sig == SIGINT) {
        printf("Receive the sigint, will out program\n");
        program_out = 1;
    }
}

static void setup_sighandler(void)
{
    struct sigaction sa;
    sa.sa_handler = sighandler;
    sigaction(SIGINT, &sa, NULL);
}

static int send_serial(int fd, void *command, int length)
{
    int status = 0;
    status = write(fd, command, length);
    tcdrain(fd);

    if (status < 0) {
        perror("write");
    }

    return status;
}

static int setup_serial(char *port_name, int baud, int flow)
{
    int fd = -1;
    speed_t spd = B9600;
    struct termios options;

    fd = open(port_name, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        perror("open");
        return fd;
    }

    tcgetattr(fd, &options);
    tcflush(fd, TCIFLUSH);
    options.c_cflag = (options.c_cflag & ~ CSIZE) | CS8;
    options.c_cflag |= CLOCAL | CREAD | CS8;

    if (flow)
        options.c_cflag |= CRTSCTS;
    else
        options.c_cflag &= ~ CRTSCTS;

    options.c_cflag &= ~ CSTOPB;
    options.c_cflag &= ~(PARENB | PARODD);
    options.c_iflag = IGNBRK;
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_oflag = 0;
    options.c_lflag = 0;

    switch (baud) {
    case 1200: spd = B1200; break;
    case 2400: spd = B2400; break;
    case 4800: spd = B4800; break;
    case 9600: spd = B9600; break;
    case 19200: spd = B19200; break;
    case 38400: spd = B38400; break;
    case 57600: spd = B57600; break;
    case 115200: spd = B115200; break;
    case 230400: spd = B230400; break;
    default: spd = B9600; break;
    }

    cfsetispeed(&options, spd);
    cfsetospeed(&options, spd);
    tcsetattr(fd, TCSANOW, &options);

    return fd;
}

static void *tty_reader_thread(void *arg)
{
    int err;
    char rbuf[RBUF_SIZE];
    int count;
    int fd = (int)arg;
    struct pollfd pfd;
    pfd.fd = fd;
    pfd.events = POLLIN | POLLERR;

    while(1) {
        err = poll(&pfd, 1, 1000); /* 1000ms */
        if (err < 0) {
            perror("poll error");
            break;
        }

        if (err == 0)
            continue;

        if (pfd.revents & POLLERR) {
            perror("poll error revents");
            break;
        }

        if (!(pfd.revents & POLLIN))
            continue;

        do {
            count = read(fd, rbuf, RBUF_SIZE-1);
            if (count > 0) {
                rbuf[count] = '\0';
                printf("%s", rbuf);
            }
        } while (count < 0 && errno == EINTR);
    };

    return NULL;
}

void usage(void)
{
    printf("Usage: tinycom [OPTIONS] ttydevice\n");
    printf("Options:\n");
    printf("  -s SPEED : set the baudrate serial port\n");
    printf("  -c       : flow control(RTS/CTS)\n");
    printf("  -n       : disable auto carriage return\n");
    printf("\nExample: tinycom -s 115200 /dev/ttyACM0\n");
    exit(1);
}

int main(int argc, char **argv)
{
    int ret = 0;
    int i;
    int fd;
    int ch;
    int speed = 0;
    int auto_carriage_return = 1; /* for at command */
    int no_newline = 0;
    int flow_ctl = 0;
    char line[1024];
    pthread_t tty_reader;
    pthread_attr_t attr;
    char *ttydev;
    int line_len;
    int remove_cnt;

    while ((ch = getopt(argc, argv, "cns:")) != -1) {
        switch (ch) {
        case 's':
            speed = optarg? atoi(optarg) : 0;
            break;
        case 'n':
            auto_carriage_return = 0;
            break;
        case 'c':
            flow_ctl = 1;
            break;
        default:
            usage();
            break;
        }
    }
    argc -= optind;
    argv += optind;

    if (!argc)
        usage();

    ttydev = *argv++;

    if (!strlen(ttydev)) {
        usage();
    }

#if USE_SIGHANDLER
    setup_sighandler();
#endif

    fd = setup_serial(ttydev, speed, flow_ctl);
    if (fd < 0) {
        perror("setup serial");
        return -1;
    }

    pthread_attr_init (&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
    if (0 > pthread_create(&tty_reader, &attr, tty_reader_thread, (void *)fd)) {
        close(fd);
        perror("pthread_create");
        return -1;
    }

    ch = '\r';

    while(1) {
        fgets(line, sizeof(line) - 1, stdin);
        line_len = strlen(line);
        remove_cnt = 0;
        if (auto_carriage_return) {
            for(i = 0; i < line_len; i++) {
                if (line[i] == '\r' || line[i]== '\n') {
                    line[i] = 0;
                    remove_cnt++;
                }
            }
            send_serial(fd, line, line_len - remove_cnt);
            send_serial(fd, &ch, 1);
        } else {
            send_serial(fd, line, line_len);
        }
    }
    close(fd);

    return 0;
}
