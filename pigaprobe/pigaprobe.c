#include <argp.h>
#include <fcntl.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "../pigaprotocol/serial_protocol.h"

static struct argp_option options[] = {
  { "led", 'l', "LED_ID", 0, "Specify LED (0-5)" },
  { "brightness", 'b', "BRIGHTNESS", 0, "Specify brightness (0-254)" },
  { "player", 'p', "PLAYER_ID", 0, "Specify Player (0-1)" },
  { "action", 'a', "ACTION", 0, "Specify Action (\"led\")" },
  { "pipe", 'i', "PIPE_PATH", 0, "Path to pipe from pigainput" },
  { 0 }
};

char doc[] = "A small utility program to interact with the piga system.";

enum Action
{
  Unspecified,
  LED
};

struct Arguments
{
  uint8_t led;
  uint8_t brightness;
  uint8_t player;
  enum Action action;
  char* pipe;
};

static uint8_t
str_to_number_in_range(char* str, uint8_t max, const char* msg)
{
  char* end;
  uint8_t number = (uint8_t)strtol(str, &end, 10);

  if(number > max) {
    fprintf(stderr, "%s\n", msg);
    number = max;
  }
  return number;
}

static error_t
parse_option(int key, char* arg, struct argp_state* state)
{
  struct Arguments* args = state->input;
  switch(key) {
    case 'l':
      args->led =
        str_to_number_in_range(arg, 5, "LED must be between 0 and 5.");
      break;
    case 'p':
      args->player =
        str_to_number_in_range(arg, 1, "Player must be between 0 and 1.");
      break;
    case 'b':
      args->brightness = str_to_number_in_range(
        arg, 255, "Brightness must be between 0 and 255.");
      break;
    case 'a':
      if(strcmp(arg, "led") == 0)
        args->action = LED;
      else
        fprintf(stderr, "Invalid action specified.\n");
      break;
    case 'i':
      args->pipe = arg;
      break;
    default:
      return ARGP_ERR_UNKNOWN;
  }
  return 0;
}

struct argp argp = { options, parse_option, 0, doc };

int
action_set_led(struct Arguments* args)
{
  // Open pipe to pigainput.
  int fd = open(args->pipe, O_WRONLY);
  if(fd < 0) {
    fprintf(stderr,
            "Could not open pipe \"%s\" to pigainput. Error: %s\n",
            args->pipe,
            strerror(errno));
    return -1;
  }

  // Write according to specs.
  uint8_t data[2] = { args->led << 3U |
                        PACKET_LENGTH_MULTIPLE |
                        (args->player == 0 ? PROTOC_PLAYER_1 : 0) |
                        (args->player == 1 ? PROTOC_PLAYER_2 : 0),
                      args->brightness };
  ssize_t written = write(fd, data, sizeof(data));
  if(written != sizeof(data)) {
    fprintf(
      stderr, "Could not write to pipe \"%s\" to pigainput.\n", args->pipe);
    return -1;
  }

  close(fd);
  return 0;
}

int
main(int argc, char* argv[])
{
  struct Arguments args = { 0 };
  args.led = 0;
  args.player = 0;
  args.brightness = 0;
  args.pipe = PIPE_PATH;
  args.action = Unspecified;

  argp_parse(&argp, argc, argv, 0, 0, &args);

  switch(args.action) {
    case LED:
      return action_set_led(&args);
      break;
    default:
      break;
  }

  return 0;
}
