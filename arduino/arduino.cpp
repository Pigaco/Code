#include "../pigaprotocol/serial_protocol.h"
#include <Arduino.h>
#include <inttypes.h>

class LED
{
  public:
  LED(const uint8_t pin);
  ~LED();

  void loop();

  void setPwr(uint8_t pwr);
  uint8_t getPwr();

  void setCounter(uint8_t counter);
  uint8_t getCounter();

  private:
  const uint8_t m_pin;
  uint8_t m_pwr;
  uint8_t m_counter;
  uint8_t m_state;
};

class SerialConnector
{
  public:
  explicit SerialConnector(const int baudRate);
  ~SerialConnector();

  void setup();
  void loop();
  void flush();

  void sendInput(uint8_t button, bool state);
  LED** leds;

  void sendHello();
  void handleHello(byte packet);

  private:
  void handleLongPacket();
  void handleShortPacket();

  const int m_baudRate;
  char m_serialInput[3] = { 0 };
};

/*
  BOUNCE LIBRARY
  The MIT License (MIT)

  Copyright (c) 2013 thomasfredericks

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
*/

/* * * * * * * * * * * * * * * * * * * * * * * * * * * *
  Main code by Thomas O Fredericks (tof@t-o-f.info)
  Previous contributions by Eric Lowry, Jim Schimpf and Tom Harkaway
  * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

class Bounce
{
  public:
  // Create an instance of the bounce library
  Bounce();

  // Attach to a pin (and also sets initial state)
  void attach(int pin);

  // Sets the debounce interval
  void interval(uint16_t interval_millis);

  // Updates the pin
  // Returns 1 if the state changed
  // Returns 0 if the state did not change
  bool update();

  // Returns the updated pin state
  bool read();

  // Returns the falling pin state
  bool fell();

  // Returns the rising pin state
  bool rose();

  protected:
  unsigned long previous_millis;
  uint16_t interval_millis;
  uint8_t state;
  uint8_t pin;
};

/*
 * LED packet format
 * =================
 * FIRST PACKET:  IIIIILTT
 * SECOND PACKET: PPPPPPPP
 *
 * All characters stand for one bit.
 *
 * I: The LED-ID. Only 0-5 are used in this implementation.
 * L: long packet. Has to be 1
 * T: type of the packet. Has to be PACKET_TYPE_LED.
 * P: new power of the LED.
 */

SerialConnector::SerialConnector(const int baudRate)
  : m_baudRate(baudRate)
{}
SerialConnector::~SerialConnector() {}

void
SerialConnector::setup()
{
  Serial.begin(m_baudRate, SERIAL_8N1);
  flush();
  Serial.write("Hello Flush");
  sendHello();
}
void
SerialConnector::flush()
{
  Serial.flush();
}
void
SerialConnector::loop()
{
  while(Serial.available() > 0) {
    delay(1);

    // Read the new byte into the internal buffer.
    Serial.readBytes(&m_serialInput[2], 1);

    if(m_serialInput[0] & PACKET_LENGTH_MULTIPLE) {
      m_serialInput[1] = m_serialInput[2];

      handleLongPacket();

      m_serialInput[0] = 0;
      m_serialInput[1] = 0;
    } else {
      m_serialInput[0] = m_serialInput[2];

      if(!(m_serialInput[0] & PACKET_LENGTH_MULTIPLE)) {
        handleShortPacket();
      }
    }
  }
}
void
SerialConnector::sendHello()
{
  byte hello = 2;
  Serial.write(hello);
}
void
SerialConnector::handleHello(byte packet)
{
  // Turn off the status LED -> Everything OK
  digitalWrite(13, LOW);
  sendHello();
  flush();
}
void
SerialConnector::handleLongPacket()
{
  if(m_serialInput[0] & PACKET_TYPE_LED) {
    // This packet controls a LED!
    uint8_t ledID = m_serialInput[0] >> 3;
    leds[ledID]->setPwr(m_serialInput[1]);
  }
}
void
SerialConnector::handleShortPacket()
{
  if(m_serialInput[0] & PACKET_TYPE_HELLO) {
    // This was a hello packet! We need to send a hello packet back.
    handleHello(m_serialInput[0]);
  }
  m_serialInput[0] = 0;
}

void
SerialConnector::sendInput(uint8_t button, bool state)
{
  byte packet = 0;

  if(state) {
    packet = BUTTON_STATE_ON | PACKET_TYPE_BUTTON | PACKET_LENGTH_SINGLE;
  } else {
    packet = BUTTON_STATE_OFF | PACKET_TYPE_BUTTON | PACKET_LENGTH_SINGLE;
  }

  switch(button) {
    case 0:
      packet |= BUTTON_ID_0;
      break;
    case 1:
      packet |= BUTTON_ID_1;
      break;
    case 2:
      packet |= BUTTON_ID_2;
      break;
    case 3:
      packet |= BUTTON_ID_3;
      break;
    case 4:
      packet |= BUTTON_ID_4;
      break;
    case 5:
      packet |= BUTTON_ID_5;
      break;
    case 6:
      packet |= BUTTON_ID_6;
      break;
    case 7:
      packet |= BUTTON_ID_7;
      break;
    case 8:
      packet |= BUTTON_ID_8;
      break;
    case 9:
      packet |= BUTTON_ID_9;
      break;
    case 10:
      packet |= BUTTON_ID_10;
      break;
    case 11:
      packet |= BUTTON_ID_11;
      break;
    case 12:
      packet |= BUTTON_ID_12;
      break;
    case 13:
      packet |= BUTTON_ID_13;
      break;
    case 14:
      packet |= BUTTON_ID_14;
      break;
    case 15:
      packet |= BUTTON_ID_15;
      break;
  }

  // Send the packet!
  Serial.write(packet);
}

#define DEBOUNCED_STATE 0
#define UNSTABLE_STATE 1
#define STATE_CHANGED 3

Bounce::Bounce()
  : previous_millis(0)
  , interval_millis(10)
  , state(0)
  , pin(0)
{}

void
Bounce::attach(int pin)
{
  this->pin = pin;
  bool read = digitalRead(pin);
  state = 0;
  if(digitalRead(pin)) {
    state = _BV(DEBOUNCED_STATE) | _BV(UNSTABLE_STATE);
  }
#ifdef BOUNCE_LOCK_OUT
  previous_millis = 0;
#else
  previous_millis = millis();
#endif
}

void
Bounce::interval(uint16_t interval_millis)
{
  this->interval_millis = interval_millis;
}

bool
Bounce::update()
{
#ifdef BOUNCE_LOCK_OUT
  state &= ~_BV(STATE_CHANGED);
  // Ignore everything if we are locked out
  if(millis() - previous_millis >= interval_millis) {
    bool currentState = digitalRead(pin);
    if((bool)(state & _BV(DEBOUNCED_STATE)) != currentState) {
      previous_millis = millis();
      state ^= _BV(DEBOUNCED_STATE);
      state |= _BV(STATE_CHANGED);
    }
  }
  return state & _BV(STATE_CHANGED);
#else
  // Read the state of the switch in a temporary variable.
  bool currentState = digitalRead(pin);
  state &= ~_BV(STATE_CHANGED);

  // If the reading is different from last reading, reset the debounce counter
  if(currentState != (bool)(state & _BV(UNSTABLE_STATE))) {
    previous_millis = millis();
    state ^= _BV(UNSTABLE_STATE);
  } else if(millis() - previous_millis >= interval_millis) {
    // We have passed the threshold time, so the input is now stable
    // If it is different from last state, set the STATE_CHANGED flag
    if((bool)(state & _BV(DEBOUNCED_STATE)) != currentState) {
      previous_millis = millis();
      state ^= _BV(DEBOUNCED_STATE);
      state |= _BV(STATE_CHANGED);
    }
  }

  return state & _BV(STATE_CHANGED);
#endif
}

bool
Bounce::read()
{
  return state & _BV(DEBOUNCED_STATE);
}

bool
Bounce::rose()
{
  return (state & _BV(DEBOUNCED_STATE)) && (state & _BV(STATE_CHANGED));
}

bool
Bounce::fell()
{
  return !(state & _BV(DEBOUNCED_STATE)) && (state & _BV(STATE_CHANGED));
}

LED::LED(const uint8_t pin)
  : m_pin(pin)
  , m_pwr(0)
  , m_counter(0)
  , m_state(0)
{}
LED::~LED() {}

void
LED::loop()
{
  m_counter -= 1;
  if(m_counter == 0) {
    // When 0 is reached, the counter will be set up to 255.
    // 0 means off, so the LED must not be switched.
    m_counter = 255;
  }
  if(m_pwr >= m_counter) {
    digitalWrite(m_pin, m_state);
    if(m_state == 1)
      m_state = 0;
    else
      m_state = 1;
  }
}

void
LED::setPwr(uint8_t pwr)
{
  m_pwr = pwr;
}
uint8_t
LED::getPwr()
{
  return m_pwr;
}
void
LED::setCounter(uint8_t counter)
{
  m_counter = counter;
}
uint8_t
LED::getCounter()
{
  return m_counter;
}

// The button bounce interval for the Bounce2 library (in ms).
#define BUTTON_BOUNCE_INTERVAL 8

#define BOUNCE_LOCK_OUT

// Taken from the host api
#define BUTTON_UP 0
#define BUTTON_DOWN 1
#define BUTTON_LEFT 2
#define BUTTON_RIGHT 3
#define BUTTON_ACTION 4
#define BUTTON_BUTTON1 5
#define BUTTON_BUTTON2 6
#define BUTTON_BUTTON3 7
#define BUTTON_BUTTON4 8
#define BUTTON_BUTTON5 9
#define BUTTON_BUTTON6 10

class SerialConnector;

class Button
{
  public:
  Button(const uint8_t pin, const uint8_t button);
  ~Button();

  void loop();

  static SerialConnector* serialConnector;

  private:
  Bounce m_pinBouncer;
  /**
   * Button-IDs:
   *
   * The buttons are kept as bytes, because enums would
   * be a source of problems in this context.
   *
   * See host.h for a detailed explanation. Here is a short list:
   *   - 0: UP
   *   - 1: DOWN
   *   - 2: LEFT
   *   - 3: RIGHT
   *   - 4: ACTION
   *   - 5: BUTTON 1
   *   - 6: BUTTON 2
   *   - 7: BUTTON 3
   *   - 8: BUTTON 4
   *   - 9: BUTTON 5
   *   - 10: BUTTON 6
   */
  const uint8_t m_button;
};

SerialConnector* Button::serialConnector = 0;

Button::Button(const uint8_t pin, const uint8_t button)
  : m_button(button)
{
  pinMode(pin, INPUT);
  digitalWrite(pin, HIGH);

  m_pinBouncer.attach(pin);
  m_pinBouncer.interval(BUTTON_BOUNCE_INTERVAL);
}
Button::~Button() {}

void
Button::loop()
{
  if(m_pinBouncer.update()) {
    // Only update if the button has a new state.
    if(m_pinBouncer.read()) {
      // Button went from LOW to HIGH
      serialConnector->sendInput(m_button, true);
    } else {
      // Button went from HIGH to LOW
      serialConnector->sendInput(m_button, false);
    }
  }
}

// If you change the baud rate here, you also have
// to change it in the makefile.
SerialConnector serialConnector(SERIAL_BAUD_RATE);

// Workaround for an arduino compiler bug
void* __dso_handle;
void* __cxa_atexit;

// LEDs
LED* leds[6];

// Buttons
Button* buttons[11];

void
setup(void)
{
  // Status-LED
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  serialConnector.setup();

  Button::serialConnector = &serialConnector;

  // Create LEDs and buttons.
  // Be cautious with the IDs and pins!
  //
  //!!!!!!!!!!!
  // CHANGE THIS
  //!!!!!!!!!!!

  // LEDs use the analog pins.
  leds[0] = new LED(A0);
  leds[1] = new LED(A1);
  leds[2] = new LED(A2);
  leds[3] = new LED(A3);
  leds[4] = new LED(A4);
  leds[5] = new LED(A5);

  // Buttons use the digital pins.
  buttons[0] = new Button(10, BUTTON_UP);
  buttons[1] = new Button(11, BUTTON_DOWN);
  buttons[2] = new Button(12, BUTTON_LEFT);
  buttons[3] = new Button(2, BUTTON_RIGHT);
  buttons[4] = new Button(9, BUTTON_ACTION);
  buttons[5] = new Button(3, BUTTON_BUTTON1);
  buttons[6] = new Button(4, BUTTON_BUTTON2);
  buttons[7] = new Button(5, BUTTON_BUTTON3);
  buttons[8] = new Button(6, BUTTON_BUTTON4);
  buttons[9] = new Button(7, BUTTON_BUTTON5);
  buttons[10] = new Button(8, BUTTON_BUTTON6);

  serialConnector.leds = leds;
}

void
loop(void)
{
  int i;

  serialConnector.loop();

  for(i = 0; i < 6; ++i) {
    leds[i]->loop();
  }

  for(i = 0; i < 11; ++i) {
    buttons[i]->loop();
  }
}
