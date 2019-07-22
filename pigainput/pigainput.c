#include <errno.h>
#include <fcntl.h>
#include <libevdev/libevdev-uinput.h>
#include <libevdev/libevdev.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

int
main(void)
{
  int err;
  struct libevdev* dev;
  struct libevdev_uinput* uidev;

  dev = libevdev_new();
  libevdev_set_name(dev, "PiGaCo-Keyboard");
  libevdev_enable_event_type(dev, EV_REL);
  libevdev_enable_event_code(dev, EV_REL, REL_X, NULL);
  libevdev_enable_event_code(dev, EV_REL, REL_Y, NULL);
  libevdev_enable_event_type(dev, EV_KEY);
  libevdev_enable_event_code(dev, EV_KEY, BTN_LEFT, NULL);
  libevdev_enable_event_code(dev, EV_KEY, BTN_MIDDLE, NULL);
  libevdev_enable_event_code(dev, EV_KEY, BTN_RIGHT, NULL);

  libevdev_enable_event_code(dev, EV_KEY, KEY_A, NULL);
  err = libevdev_uinput_create_from_device(
    dev, LIBEVDEV_UINPUT_OPEN_MANAGED, &uidev);
  if(err != 0)
    return err;

  while(1) {
    printf("Posting Event.\n");

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

  libevdev_uinput_destroy(uidev);
  libevdev_free(dev);
}
