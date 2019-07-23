#ifndef SERIAL_PROTOCOL_H_INCLUDED
#define SERIAL_PROTOCOL_H_INCLUDED

#define SERIAL_BAUD_RATE        9600

#define PACKET_TYPE_BUTTON      1u   // 00000001
#define PACKET_TYPE_LED         0u   // 00000000
#define PACKET_TYPE_HELLO       0b00000010u   // 00000010

#define PACKET_LENGTH_SINGLE    0u   // 00000000
#define PACKET_LENGTH_MULTIPLE  4u   // 00000100

#define BUTTON_STATE_ON         8u   // 00001000
#define BUTTON_STATE_OFF        0u   // 00000000

#define BUTTON_ID_0             0u   // 00000000
#define BUTTON_ID_1             16u  // 00010000
#define BUTTON_ID_2             32u  // 00100000
#define BUTTON_ID_3             48u  // 00110000
#define BUTTON_ID_4             64u  // 01000000
#define BUTTON_ID_5             80u  // 01010000
#define BUTTON_ID_6             96u  // 01100000
#define BUTTON_ID_7             112u // 01110000
#define BUTTON_ID_8             128u // 10000000
#define BUTTON_ID_9             144u // 10010000
#define BUTTON_ID_10            160u // 10100000
#define BUTTON_ID_11            176u // 10110000
#define BUTTON_ID_12            192u // 11000000
#define BUTTON_ID_13            208u // 11010000
#define BUTTON_ID_14            224u // 11100000
#define BUTTON_ID_15            240u // 11110000

#define LED_ID_0                0u   // 00000000
#define LED_ID_1                8u   // 00001000
#define LED_ID_2                16u  // 00010000
#define LED_ID_3                24u  // 00011000
#define LED_ID_4                32u  // 00100000
#define LED_ID_5                40u  // 00101000
#define LED_ID_6                56u  // 00111000

// Player IDs for LED control.
#define PROTOC_PLAYER_1     1 << 6  // 01000000
#define PROTOC_PLAYER_2     1 << 7  // 10000000

#define PIPE_PATH  "/var/run/pigainput_control"

#endif
