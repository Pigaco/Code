#include <errno.h>
#include <fcntl.h>
#include <libevdev/libevdev-uinput.h>
#include <libevdev/libevdev.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/poll.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

#include "../pigaprotocol/serial_protocol.h"

#define PLAYER_COUNT 2

struct pollfd fds[PLAYER_COUNT + 1] = { 0, 0, 0 };
int err;
struct libevdev* dev;
struct libevdev_uinput* uidev;

// Mappings from players to buttons.
unsigned int buttonMapping[2][11] = { { KEY_W,
                                        KEY_S,
                                        KEY_A,
                                        KEY_D,
                                        KEY_SPACE,
                                        KEY_1,
                                        KEY_2,
                                        KEY_3,
                                        KEY_4,
                                        KEY_5,
                                        KEY_6 },
                                      { KEY_UP,
                                        KEY_DOWN,
                                        KEY_LEFT,
                                        KEY_RIGHT,
                                        KEY_0,
                                        KEY_X,
                                        KEY_C,
                                        KEY_V,
                                        KEY_B,
                                        KEY_N,
                                        KEY_M } };

void
serial_setup(int fd)
{
  // Following a guide from
  // https://chrisheydrick.com/2012/06/17/how-to-read-serial-data-from-an-arduino-in-linux-with-c-part-3/
  struct termios toptions;

  /* Get currently set options for the tty */
  tcgetattr(fd, &toptions);

  /* Set custom options */

  /* 9600 baud */
  cfsetispeed(&toptions, B9600);
  cfsetospeed(&toptions, B9600);
  /* 8 bits, no parity, no stop bits */
  toptions.c_cflag &= ~PARENB;
  toptions.c_cflag &= ~CSTOPB;
  toptions.c_cflag &= ~CSIZE;
  toptions.c_cflag |= CS8;
  /* no hardware flow control */
  toptions.c_cflag &= ~CRTSCTS;
  /* enable receiver, ignore status lines */
  toptions.c_cflag |= CREAD | CLOCAL;
  /* disable input/output flow control, disable restart chars */
  toptions.c_iflag &= ~(IXON | IXOFF | IXANY);
  /* disable canonical input, disable echo,
  disable visually erase chars,
  disable terminal-generated signals */
  toptions.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  /* disable output processing */
  toptions.c_oflag &= ~OPOST;
}

int
open_serial(char* ttys[])
{
  for(size_t i = 0; i < PLAYER_COUNT; ++i) {
    fds[i].events = POLLIN;// Event for data ready.
    fds[i].fd = open(ttys[i], O_RDWR);
    if(fds[i].fd < 0) {
      fprintf(stderr, "Could not open tty from %s\n", ttys[i]);
      return -1;
    }
    serial_setup(fds[i].fd);
  }
  return 0;
}

int
open_pipe(const char* fifo)
{
  mkfifo(fifo, 0777);
  fds[PLAYER_COUNT].fd = open(fifo, O_RDONLY);
  if(fds[PLAYER_COUNT].fd < 0) {
    fprintf(stderr, "Could not open pipe from %s\n", fifo);
    return -1;
  }
  return 0;
}

void
close_serial()
{
  for(size_t i = 0; i < PLAYER_COUNT; ++i) {
    close(fds[i].fd);
  }
}

int
inject_key_state(unsigned int key, unsigned int state)
{
  err = libevdev_uinput_write_event(uidev, EV_KEY, key, state);
  if(err != 0)
    return err;
  libevdev_uinput_write_event(uidev, EV_SYN, SYN_REPORT, 0);
  if(err != 0)
    return err;

  return 0;
}

enum ReadSource
{
  Player1,
  Player2,
  Pipe
};

void
handle_read_player(enum ReadSource source, char* buf, ssize_t length)
{
  char* b = buf;
  while(b < buf + length) {
    // Can only be input commands and can therefore be directly interpreted
    // byte-per-byte. Correct player has to be determined by examining the
    // source.
    char data = *(b++);

    if(data & PACKET_LENGTH_MULTIPLE || data & PACKET_TYPE_HELLO) {
      // INVALID PACKET! Only singles (button updates) are allowed.
      continue;
    }
    uint8_t button = data >> 4U;
    if(button > 10) {
      // INVALID PACKET! There are only 11 buttons.
      continue;
    }
    inject_key_state(buttonMapping[source][button], data & BUTTON_STATE_ON);
  }
}

void
handle_read_pipe(char* buf, ssize_t length)
{
  char* b = buf;
  char data[2];
  while(b < buf + length) {
    data[0] = *(b++);
    if(data[0] & PACKET_TYPE_HELLO) {
      // INVALID PACKET! Only LED control allowed.
      continue;
    }
    if(data[0] & PACKET_TYPE_LED && b < buf + length) {
      // LED packet detected! This is two bytes long.
      data[1] = *(b++);

      // Push message to respective serial ports. Both can receive the same
      // message if both players are targeted.
      if(data[0] & PROTOC_PLAYER_1) {
        write(fds[0].fd, data, 2);
      }
      if(data[0] & PROTOC_PLAYER_2) {
        write(fds[1].fd, data, 2);
      }
    }
  }
}

void
handle_read(enum ReadSource source, char* buf, ssize_t length)
{
  switch(source) {
    case Player1:
    case Player2:
      handle_read_player(source, buf, length);
      break;
    case Pipe:
      handle_read_pipe(buf, length);
      break;
  }
}

int
main(int argc, char* argv[])
{
  /* Expected Arguments:
   * <Player1 TTY> <Player2 TTY> <Pipe to read from>
   */

  if(argc != 4) {
    fprintf(stderr,
            "Must provide 3 arguments! <Player1 TTY> <Player2 TTY> <Pipe to "
            "read from>\n");
    return -1;
  }

  // Open TTYs and create pipe for control inputs.
  if(open_serial(argv + 1)) {
    fprintf(stderr, "Could not open serial connections! Quitting.\n");
    return -1;
  }
  if(open_pipe(argv[2])) {
    fprintf(stderr, "Could not create & open pipe! Quitting.\n");
    return -1;
  }

  // Create virtual input device for players.

  dev = libevdev_new();
  libevdev_set_name(dev, "PiGaCo-Keyboard");
  libevdev_enable_event_type(dev, EV_REL);
  libevdev_enable_event_code(dev, EV_REL, REL_X, NULL);
  libevdev_enable_event_code(dev, EV_REL, REL_Y, NULL);
  libevdev_enable_event_type(dev, EV_KEY);
  libevdev_enable_event_code(dev, EV_KEY, BTN_LEFT, NULL);
  libevdev_enable_event_code(dev, EV_KEY, BTN_MIDDLE, NULL);
  libevdev_enable_event_code(dev, EV_KEY, BTN_RIGHT, NULL);

  // First Player
  libevdev_enable_event_code(dev, EV_KEY, KEY_W, NULL);
  libevdev_enable_event_code(dev, EV_KEY, KEY_A, NULL);
  libevdev_enable_event_code(dev, EV_KEY, KEY_S, NULL);
  libevdev_enable_event_code(dev, EV_KEY, KEY_D, NULL);
  libevdev_enable_event_code(dev, EV_KEY, KEY_SPACE, NULL);
  libevdev_enable_event_code(dev, EV_KEY, KEY_1, NULL);
  libevdev_enable_event_code(dev, EV_KEY, KEY_2, NULL);
  libevdev_enable_event_code(dev, EV_KEY, KEY_3, NULL);
  libevdev_enable_event_code(dev, EV_KEY, KEY_4, NULL);
  libevdev_enable_event_code(dev, EV_KEY, KEY_5, NULL);
  libevdev_enable_event_code(dev, EV_KEY, KEY_6, NULL);

  // Second Player
  libevdev_enable_event_code(dev, EV_KEY, KEY_UP, NULL);
  libevdev_enable_event_code(dev, EV_KEY, KEY_DOWN, NULL);
  libevdev_enable_event_code(dev, EV_KEY, KEY_LEFT, NULL);
  libevdev_enable_event_code(dev, EV_KEY, KEY_RIGHT, NULL);
  libevdev_enable_event_code(dev, EV_KEY, KEY_0, NULL);
  libevdev_enable_event_code(dev, EV_KEY, KEY_X, NULL);
  libevdev_enable_event_code(dev, EV_KEY, KEY_C, NULL);
  libevdev_enable_event_code(dev, EV_KEY, KEY_V, NULL);
  libevdev_enable_event_code(dev, EV_KEY, KEY_B, NULL);
  libevdev_enable_event_code(dev, EV_KEY, KEY_N, NULL);
  libevdev_enable_event_code(dev, EV_KEY, KEY_M, NULL);

  err = libevdev_uinput_create_from_device(
    dev, LIBEVDEV_UINPUT_OPEN_MANAGED, &uidev);
  if(err != 0) {
    fprintf(stderr, "Could not create virtual device! Quitting.\n");
    return err;
  }

  // Create poll structure
  ssize_t length;
  char buf[1024];

  for(;;) {
    int n = 0;
    n = poll(fds, PLAYER_COUNT + 1, 2000);
    for(size_t i = 0; i < PLAYER_COUNT + 1 && n > 0; ++i) {
      if(fds[i].revents & POLLIN) {
        length = read(fds[i].fd, buf, sizeof(buf));
        handle_read(i, buf, length);
      }
    }
  }

  while(1) {
    // post a REL_X event
    err = libevdev_uinput_write_event(uidev, EV_REL, REL_X, -20);
    if(err != 0)
      return err;
    err = libevdev_uinput_write_event(uidev, EV_REL, REL_Y, -20);
    if(err != 0)
      return err;

    err = libevdev_uinput_write_event(uidev, EV_KEY, KEY_A, 1);
    if(err != 0)
      return err;
    libevdev_uinput_write_event(uidev, EV_SYN, SYN_REPORT, 0);
    if(err != 0)
      return err;

    err = libevdev_uinput_write_event(uidev, EV_KEY, KEY_A, 0);
    if(err != 0)
      return err;
    libevdev_uinput_write_event(uidev, EV_SYN, SYN_REPORT, 0);
    if(err != 0)
      return err;

    sleep(1);
  }

  close_serial();
  close(fds[PLAYER_COUNT].fd);

  libevdev_uinput_destroy(uidev);
  libevdev_free(dev);
}
