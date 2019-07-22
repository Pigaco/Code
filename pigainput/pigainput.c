#include <fcntl.h>
#include <libevdev/libevdev.h>

int
main(void)
{
  int err;
  int fd, new_fd, uifd;
  struct libevdev* dev;
  struct libevdev_uinput* uidev;
  struct input_event ev[2];
  fd = open("/dev/input/event0", O_RDONLY);
  if(fd < 0)
    return err;
  err = libevdev_new_from_fd(fd, &dev);
  if(err != 0)
    return err;
  uifd = open("/dev/uinput", O_RDWR);
  if(uifd < 0)
    return -errno;
  err = libevdev_uinput_create_from_device(dev, uifd, &uidev);
  if(err != 0)
    return err;
  // post a REL_X event
  err = libevdev_uinput_write_event(uidev, EV_REL, REL_X, -1);
  if(err != 0)
    return err;
  libevdev_uinput_write_event(uidev, EV_SYN, SYN_REPORT, 0);
  if(err != 0)
    return err;
  libevdev_uinput_destroy(uidev);
  libevdev_free(dev);
  close(uifd);
  close(fd);
}
