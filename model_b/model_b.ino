//
//    ┏━━━━━━━━━━━━━━━━━━━┱─────────┐
//    ┃ DEC-HEX Converter ┃ model B │
//    ┗━━━━━━━━━━━━━━━━━━━┹─────────┘
//             Oskar Fornander, 2025
//
//
//    DEC-HEX Converter, Oskar Fornander, July-October 2025
//    ATtiny85/45, seven segment displays or LCD and a rotary encoder, driven by 3xAAA/AA-batteries.
//      Model A: Shift registers (74HC595) and seven segment displays, with power button.
//      Model B: LCD-display with I2C-module. No power button; enters sleep mode after being inactive a few moments.
//    This device will simultaneously count in decimal (0-255) and hexadecimal (00-FF) and in that way work as a converter between the two bases.
//    The code is optimized for speed, efficient use of memory and low power consumption on the ATtiny85 or ATtiny45, running at 8MHz.
//    
//    This file contains code for model B:
//      The device has no power button but is put to sleep after a time of inactivity. The last state of the counter is recovered when the device is woken with a turn on the rotary encoder knob (but reset when the knob is pushed).
//      Power consumption: ~25mA (active), <1uA (sleeping). With 3 AAA-batteries at 1000mAh the device can be active for about 36 hours and sleeping for a very long time.


//Ideas to implement some day ... maybe:
  //Better error-handling!
  //Make it not turn off the LCD when resetting? - not really possible ...
  //Add feature to only update the changed digits and not the whole numbers on LCD???

/*
How does model B work?
  Start by pressing knob or rotating, last displayed value is shown.
  Rotate knob to increase/decrease value shown as decimal and hexadecimal.
  Press knob to reset the counter to 0 - by restarting the ATtiny85 using RESET pin.
  Go to sleep after being inactive for some time. (no power button)
  Power to LCD is controlled by mcu via a transistor.
  Let mcu wake up by interrupt on both A and B (at least one of them).



Connections to the ATtiny85:

// Rotary Encoder: 3 pins (A, B + RESET)
// LCD (I2C): 3 pins (SDA, SCL, VCC)

// LCD 1602 with I2C connection: 2 lines of 16 characters

pin  Arduino pin type            connection
----|-----------|---------------|--------------
1    PB5/RESET                   Push button   | (Push button with external pull up resistor)
2    PB3         INPUT           B             | Rotary encoder, low pass filter and external pull up resistor for both A and B. 
3    PB4         OUTPUT          LCD power     | Power to LCD via transistor
4    -                           GND
5    PB0         OUTPUT          SDA           | Serial data for LCD (I2C)
6    PB1         OUTPUT          SCL           | Serial clock line for LCD (I2C)
7    PB2         INPUT,interrupt A             | (This pin can have external interrupt)
8    -                           VCC


Reset button will reset the ATtiny85, so the start up sequence must be short and reset the counter.

Text in LCD display (text to LCD is sent via I2C protocol) (2 x 16 characters):
   ┌────────────────┐    ┌────────────────┐
   │▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉│    │DEC: 165 HEX: A5│
   │▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉│    │BIN: 10100101   │
   └────────────────┘    └────────────────┘
*/

//#### Libraries #################################
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/interrupt.h>
//#include <EEPROM.h>

//#### Defining pins #############################
//Define pins for rotary encoder
#define PIN_ROTARY_ENCODER_A PB2
#define PIN_ROTARY_ENCODER_B PB3

//Define pins for LCD
#define PIN_DISPLAY_DATA PB0
#define PIN_DISPLAY_CLOCK PB1
#define PIN_DISPLAY_VCC PB4

//#### Defining constants for LCD ################
//Pin mapping PCF8574 I2C module to LCD
#define RS 0  //Register select (COMMAND/DATA)
#define RW 1  //READ/WRITE
#define EN 2  //ENABLE (pulsed HIGH to LOW)
#define BL 3  //BACKLIGHT
#define D4 4  //Data bit 4
#define D5 5 
#define D6 6 
#define D7 7 
//LCD pin values
#define LCD_RS (1 << RS)
#define LCD_RW (1 << RW)
#define LCD_EN (1 << EN)
#define LCD_BL (1 << BL)
#define LCD_D4 (1 << D4)
#define LCD_D5 (1 << D5)
#define LCD_D6 (1 << D6)
#define LCD_D7 (1 << D7)

//Defining LCD constants used in the I2C communication with the LCD
//Commands and flags
#define LCD_ADDRESS 0x27 //LCD_ADDRESS is the I2C address to the LCD (defaults to 0x27))
#define READ 1 //READ/WRITE and COMMAND/DATA are sent to RW and RS respectively
#define WRITE 0
#define COMMAND 0
#define DATA 1
#define BACKLIGHT_ON LCD_BL
#define BACKLIGHT_OFF 0

//Clear display
#define LCD_CLEAR_DISPLAY 0x01
//Return home
#define LCD_RETURN_HOME 0x02
//Entry mode set
#define LCD_ENTRY_MODE_SET 0x04 //Example: LCD_ENTRY_MODE_SET | LCD_INCREMENT | LCD_DISPLAY_NO_SHIFT
#define LCD_INCREMENT 0x02
#define LCD_DECREMENT 0x00
#define LCD_DISPLAY_SHIFT 0x01
#define LCD_DISPLAY_NO_SHIFT 0x00
//Display on/off control
#define LCD_ON_OFF 0x08 //Example: LCD_ON_OFF | LCD_DISPLAY_ON | LCD_CURSOR_OFF | LCD_BLINKS_OFF
#define LCD_DISPLAY_ON 0x04
#define LCD_DISPLAY_OFF 0x00
#define LCD_CURSOR_ON 0x02
#define LCD_CURSOR_OFF 0x00
#define LCD_BLINKS_ON 0x01
#define LCD_BLINKS_OFF 0x00
//Cursor or display shift (used to correct or search the display)
#define LCD_CURSOR_OR_DISPLAY_SHIFT 0x10
#define LCD_SHIFT_CURSOR 0x00
#define LCD_SHIFT_DISPLAY 0x08
#define LCD_SHIFT_RIGHT 0x04
#define LCD_SHIFT_LEFT 0x00
//Function set
#define LCD_FUNCTION_SET_8BIT 0x30 //This is not used, only 4-bit mode
#define LCD_FUNCTION_SET_4BIT 0x20 //Example: LCD_FUNCTION_SET_4BIT | LCD_TWO_LINES | LCD_5X8_FONT 
#define LCD_ONE_LINE 0x00
#define LCD_TWO_LINES 0x08
#define LCD_5X8_FONT 0x00 //Only the 5x8 font can be used with display in 2 line mode
#define LCD_5X10_FONT 0x04
//Set Character Generator RAM Address
#define LCD_SET_CGRAM_ADDRESS 0x40 //0b01aaaaaa where aaaaaa is the address
//Set Display Data RAM Address
#define LCD_SET_DDRAM_ADDRESS 0x80 //0b1aaaaaaa where aaaaaa is the address. one line display: 0x00-0x4F, two line display: 0x00-0x27 for first line and 0x40-0x67 for second line.


//#### Constants and variables for sleep #########
#define WAIT_FOR_SLEEP 3000 //number of ms to wait from last action before going to sleep
#define WAKE_UP_DELAY 1000 //1 second
unsigned long lastTimeActive = 0; //Stores the time millis() after the last activity, to check for millis() - lastTimeActive > WAIT_FOR_SLEEP
unsigned long wakeUpTime = -WAKE_UP_DELAY; //millis() when woken up. Used to ignore rotation of the rotary encoder for a moment after wake up. Set to negative value to ignore this at startup and reset.

//#### Defining other variables ##################
byte counter = 0; //global counter variable in interval [0, 255], this is the value to be displayed in decimal and hexadecimal

//#### Defining variables for rotary encoder #####
//Bit fields to store the flags that keep track of the rotary encoder status
struct {
  uint8_t currentA : 1; //Previous and current readings of the rotary encoder pins A and B. 1 for true and 0 for false. Using single bit sto store information for each.
  uint8_t prevA : 1;
  uint8_t currentB : 1;
  uint8_t prevB : 1;
  uint8_t transition : 4; //Transitions from one state to another (4 bits: A'B'AB)
  int8_t counter : 4; //Counting states of the rotary encoder. 4 bits to store values from -4 to 4 (-8 to 7)
  uint8_t valueChanged : 1; //Flag to indicate that the value of counter has changed (using one bit instead of one byte with a comparison number to save space)
} rotaryStates;

//#### Defining look up tables ###################
//Look up tables for decimal digits, used to speed up the calculations (from ~80 cycles (80us) to ~3 cycles(375ns)). These tables takes 3 * 256 bytes of memory. Stored in Progmem to save space in the dynamic memory for global variables. This way the program can fit on an ATtiny45 as well.
const uint8_t dec1_table[256] PROGMEM = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 1, 2, 3, 4, 5};
const uint8_t dec2_table[256] PROGMEM = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5};
const uint8_t dec3_table[256] PROGMEM = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2};


void setup() {
  //Pins for display (using I2C)
  //DDRB -> set direction (0 = input, 1 = output)
  PORTB &= ~(1 << PIN_DISPLAY_CLOCK); //I2C SCL for LCD (constant value: LOW; only the DDRB is changed between input and output: output LOW or input without internal pull up resistor)
  PORTB &= ~(1 << PIN_DISPLAY_DATA); //I2C SDA for LCD (constant value: LOW)
  PORTB &= ~(1 << PIN_DISPLAY_VCC); //Pin controling power to the LCD via NPN transistor (PN2222A). OFF: input, with no pullup (high impedance). ON: output HIGH
  DDRB &= ~(1 << PIN_DISPLAY_VCC); 

  DDRB &= ~(1 << PIN_DISPLAY_CLOCK); //Set I2C SCL and SDA as input (0) (without pullup), i.e. HIGH with external pull up resistor
  DDRB &= ~(1 << PIN_DISPLAY_DATA);

  //Pins for Rotary encoder (push button is connected to reset pin)
  pinMode(PIN_ROTARY_ENCODER_A, INPUT_PULLUP); //Internal and external pullup resistors
  pinMode(PIN_ROTARY_ENCODER_B, INPUT_PULLUP);
  //Using polling instead of interrupt for changing the counter with the rotary encoder. But interrupts are used to wake the mcu up from sleep.

  sei(); //Set global interrupts

  //Set starting values for these flags
  rotaryStates.currentA = 1;
  rotaryStates.currentB = 1;
  rotaryStates.prevA = 1;
  rotaryStates.prevB = 1;
  rotaryStates.counter = 0;
  rotaryStates.valueChanged = 0;

  bool error = true;
  int tries = 3;
  while (error && tries--) {
    error = !displayInitialize(); //Initialize and clear the display as soon as possible after startup, since the RESET button is used to reset the counter (by restarting the ATtiny85). 
    delay(100);
  }
  if (error) { //Failing to initialize the display
    goToSleep();
  }
  //displayFill(0xff);
  //displayClear(); //Not needed; after displayInitialize() the LCD is cleared

  showValue(counter, true); //Show the counter value on the display (after reset it will be 0), true makes sure to also write the extra data and not only the numbers themselves.
}

ISR(PCINT0_vect) {
  //This ISR is called when PB2 (PCINT2) gives an interrupt.
  //Used only to wake up from sleep. No need to do anything here ...
}

void loop() {
  //Use polling instead of interrupt for reading the quadrature signal of the rotary encoder.
  //
  //  A----|    |------
  //       |----|
  //  
  //  B------|    |------
  //         |----|
  //  
  //  AB 11 01 00 10 11
  //    CW-->   <-- CCW
  
  if (millis() - wakeUpTime < WAKE_UP_DELAY) return; //Do nothing for a moment after wake up

  //Polling the status of the rotary encoder to detect rotations
  rotaryStates.currentA = !!(PINB & (1 << PIN_ROTARY_ENCODER_A)); //Read pins, faster than digitalRead()
  rotaryStates.currentB = (PINB >> PIN_ROTARY_ENCODER_B) & 1; //An equivalent way of reading the value. Both gives 1 or 0 as the result.
  rotaryStates.transition = (rotaryStates.prevA << 3) | (rotaryStates.prevB << 2) | (rotaryStates.currentA << 1) | rotaryStates.currentB; //A'B'AB

  switch (rotaryStates.transition) { //All valid transitions (gray code) are listed as cases here
    case 0b1101: //Valid transitions for clockwise rotation
    case 0b0100: //01 -> 00
    case 0b0010:
    case 0b1011:
      rotaryStates.counter++; //increasing the small counter with one for each valid transition.
      rotaryStates.prevA = rotaryStates.currentA;
      rotaryStates.prevB = rotaryStates.currentB;
      break;
    case 0b1110: //Valid transitions for counter clockwise rotation
    case 0b1000:
    case 0b0001:
    case 0b0111:
      rotaryStates.counter--; //decreasing small counter
      rotaryStates.prevA = rotaryStates.currentA;
      rotaryStates.prevB = rotaryStates.currentB;
      break;
    default:
      //No valid transition
      break;
  }
  if (rotaryStates.counter >= 4) { //Clock wise: advance counter when 4 states are registered (that is one tick on the rotary encoder)
    rotaryStates.counter = 0; //Reset the small counter
    counter++; //Increase the global counter
    rotaryStates.valueChanged = 1; //Indicate that the value of counter has changed
  } else if (rotaryStates.counter <= -4) { //Counter clock wise
    rotaryStates.counter = 0;
    counter--; //Decrease the global counter
    rotaryStates.valueChanged = 1; 
  }

  //Update display if value of the counter is changed
  if (rotaryStates.valueChanged) {
    showValue(counter, false); //Update value (only the numbers) (~175us?)
    rotaryStates.valueChanged = 0; //Clear flag
    lastTimeActive = millis(); //Update the last time of activity on the device, i.e. reset the sleep timer
  }

  if (millis() - lastTimeActive > WAIT_FOR_SLEEP) {
    goToSleep(); //Time to shut down after a long time of inactivity
  }
}

void goToSleep() {
  //This is used as an automatic power saving feature, after the unit has been left turned on for some time. Turn off the LCD and let the ATtiny85 go into deep sleep to save power, from a few mA down to below 1uA
  //displayClear(); //Display clear - no need for this
  displayOff(); //Display off
  ADCSRA &= ~(1 << ADEN); //Turn off ADC
  power_adc_disable();
  power_timer0_disable();
  power_timer1_disable();
  power_usi_disable();
  // Leave PCINT on to enable interrupt for wake up without restarting the code (running setup())
  GIFR |= (1 << PCIF); //Clear Pin change interrupt flag  
  GIMSK |= (1 << PCIE); //Enable pin change interrupts
  PCMSK |= (1 << PCINT2); //Enable interrupt on PB2 (ISR declared above)
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_bod_disable(); //Turn off BOD
  sleep_enable();
  sleep_cpu(); //Activate sleeping mode

  // --- MCU is sleeping --- //

  sleep_disable(); //mcu just woke up
  GIMSK &= ~(1 << PCIE); //Disable pin change interrupts
  power_all_enable();
  wakeUpTime = millis();
  lastTimeActive = millis();
  displayInitialize(); //Restart the display at wake up
  showValue(counter, true); //Show the current value on the display again after wake up, the value is still in the variable counter when woken by a turn of the knob, if reset by press it is cleared and the unit restarts from scratch. true to not just update the values but also write the text.
}

//####################################

void shortDelay() {
  //no-operation gives 125ns of delay at 8MHz, instead of delayMicroseconds(1), about up to 100ns is needed.
  //delayMicroseconds(1);
  __asm__("nop\n\t"); //nop -- this is removed by the compiler ... Use _delay_ms() and delayMicroseconds() instead, none of which is dependent on global interrupts.
}

void dataHigh() {
  //Set data bit (I2C SDA) for display to 1 (send a 1)
  DDRB &= ~(1 << PIN_DISPLAY_DATA); //Set SDA as input without pullup - external pullup resistors pulls the serial data line HIGH
}
void dataLow() {
  //Clear data bit (I2C SDA) for display to 0 (send a 0)
  DDRB |= (1 << PIN_DISPLAY_DATA); //Set SDA as output
}

void clockHigh() {
  //Set clock (I2C SCL) HIGH
  DDRB &= ~(1 << PIN_DISPLAY_CLOCK); //SCL as input without pullup -> HIGH
}
void clockLow() {
  //Set clock (I2C SCL) LOW
  DDRB |= (1 << PIN_DISPLAY_CLOCK); //SCL as output low -> LOW
}
void pulseClock() {
  //Pulse I2C SCL high with SDA stable
  clockHigh();
  delayMicroseconds(5); //Delay (>4us)
  clockLow();
}

bool I2CStart(byte mode) {
  //Send start condition before transmitting data, followed by address and read/write bit
  //mode is either READ (1) or WRITE (0). Address is sent together with read/write so data can be transmited after this.
  //Both data and clock lines remain HIGH when the bus is not busy. A HIGH-to-LOW transition of the data line, while the clock is HIGH is defined as the start condition. 
  dataHigh(); //Make sure SCL remains HIGH and SDA is HIGH from start
  clockHigh();
  delayMicroseconds(5); //Bus free time and start condition set-up time is >4.7us
  dataLow(); //Set SDA LOW 
  delayMicroseconds(5); //Start condition hold time is >4us
  clockLow(); //Set SCL LOW
  //Send address and mode (READ/WRITE)
  int tries = 3; //Make a few tries before quitting
  while (!sendByte((LCD_ADDRESS << 1) | mode)) { //Address is 7 bits and are combined with the read/write bit. ACK is true if ok and ACK is false if not received properly.
    if (!tries--) { 
      I2CStop();
      return false;
    }
  }
  return true;
}
void I2CStop() {
  //Send stop condition after transmitting data
  //A LOW-to-HIGH transition of the data line while the clock is HIGH is defined as the stop condition. 
  dataLow(); //Make sure SDA is LOW from start
  clockHigh(); //Set SCL HIGH
  delayMicroseconds(5); //Stop condition set-up time is >4us
  dataHigh(); //Set SDA HIGH 
}
bool ack() {
  //Read Acknowledge bit
  dataHigh(); //Let go of the SDA line
  delayMicroseconds(5);
  clockHigh();
  delayMicroseconds(5);
  bool ACK = (PINB & (1 << PIN_DISPLAY_DATA)); //read SDA, 0 is ACK 1 is NACK
  clockLow();                                                 
  return !ACK; //Return true for ACK and false for NACK (the negation makes sure the value is either true (1) or false (0) no matter the pin in PORTB)
}

bool sendByte(byte value) {
  //Send one byte of data to LCD with I2C protocoll via PCF8574
  //One data bit is transferred during each clock pulse. The data on the SDA line must remain stable during the HIGH period of the clock pulse
  //The number of data bytes transferred between the start and the stop conditions from transmitter to receiver is not limited. Each byte of eight bits is followed by one acknowledge bit. The acknowledge bit is a HIGH level put on the bus by the transmitter whereas the master generates an extra acknowledge related clock pulse.
  for (int i = 0; i < 8; i++) {
    if ((value & 0x80) == 0) { //Send MSB first
      dataLow(); 
    } else {
      dataHigh();
    }
    delayMicroseconds(5); //>4.7us
    pulseClock();
    value = value << 1;
  } 
  //Acknowledge bit
  if (ack()) { //Acknowledged
    return true;
  } else { //Not acknowledged
    return false; //Let the calling function decide what to do with ACK or NACK.
  }
}

byte buildByte(byte nibble, byte mode) {
  //Build the data to send with I2C based on values for D4-D7 etc.
  byte value = 0; 
  value |= (nibble & 0x1)? LCD_D4 : 0;
  value |= (nibble & 0x2)? LCD_D5 : 0;
  value |= (nibble & 0x4)? LCD_D6 : 0;
  value |= (nibble & 0x8)? LCD_D7 : 0;
  value |= BACKLIGHT_ON | LCD_EN | (WRITE << RW) | (mode << RS); //Backlight, enable (high), read/write and command/data
  return value;
}

bool sendDataSingleByte(byte data, byte mode) {
  //Reuse function sendData() but for just one single byte
  return sendData(&data, 1, mode);
}

bool sendData(const byte* data, uint8_t data_length, byte mode) {
  //Send one or more bytes to LCD with I2C, split into two nibbles (4 bit length mode)
  //mode is COMMAND or DATA, data_length is the length of the data array
  //Call this function like this: sendData({0x64, 0x65, 0x66}, 3, DATA)
  if (!I2CStart(WRITE)) { //Send start condition followed by address and read/write bit (multiple tries in I2CStart())
      return false; //Error
  }
  byte EN_LOW = ~LCD_EN; //And byte with this to toggle EN (enable) LOW
  for (uint8_t i = 0; i < data_length; i++) { //Send each byte
    for (int n = 1; n >= 0; n--) { //Same procedure for first high then low nibble
      byte nibble = (data[i] >> (4 * n)) & 0x0f; //This is first high nibble of data[i] and then low nibble
      byte value = buildByte(nibble, mode);
      if (!sendByte(value)) return false; //Send one nibble (first high then low)
      delayMicroseconds(2);
      if (!sendByte(value & EN_LOW)) return false; //Pulse EN from HIGH to LOW
      delayMicroseconds(50);
    }
  }
  I2CStop(); //Send stop condition (it is ok(?) to not ignore last ACK-bit)
  return true;
}

bool sendDataSingleNibble(byte data, byte mode) {
  //Send one nibble to LCD with I2C, the lower nibble of data is sent. 
  //mode is COMMAND or DATA
  //This is used in the initialization when 8-bit mode is used
  byte EN_LOW = ~LCD_EN; //And with this to toggle EN (enable) LOW
  byte value = buildByte(data, mode);
  if (!sendByte(value)) return false; //Send byte (high nibble)
  delayMicroseconds(2);
  if (!sendByte(value & EN_LOW)) return false; //Pulse EN (P2) from HIGH to LOW
  delayMicroseconds(50);
  return true;
}

bool displayInitialize() {
  //Send commands to initialize the display (LCD1602, SPLC780D). 4-bit interface (data sheet p 16)
  DDRB |= (1 << PIN_DISPLAY_VCC); //Pin controling power to the LCD via NPN transistor (PN2222A). OFF: input, with no pullup (high impedance). ON: output HIGH
  PORTB |= (1 << PIN_DISPLAY_VCC); 
  _delay_ms(50); //Wait >15ms after power on at VDD > 4.5V. Delay at least 50ms

  if (!I2CStart(WRITE)) return false; //Send start condition for sending nibbles on I2C
  if (!sendDataSingleNibble(LCD_FUNCTION_SET_8BIT >> 4, COMMAND)) return false; //In the initialization, send only high nibble since LCD expects data in 8-bit mode. 0b0011 is Function set with DL=1 for 8-bit mode
  _delay_ms(4);
  if (!sendDataSingleNibble(LCD_FUNCTION_SET_8BIT >> 4, COMMAND)) return false;
  delayMicroseconds(100);
  if (!sendDataSingleNibble(LCD_FUNCTION_SET_8BIT >> 4, COMMAND)) return false;
  delayMicroseconds(100); //Is this delay needed?
  //Now, activate 4-bit interface mode and set the display to 2 lines and 5x8 font
  if (!sendDataSingleNibble(LCD_FUNCTION_SET_4BIT >> 4, COMMAND)) return false; //Function set (high nibble only)
  I2CStop(); //Send stop condition for sending nibbles (this is handled within sendDataByte() function used below)
  if (!sendDataSingleByte(LCD_FUNCTION_SET_4BIT | LCD_TWO_LINES | LCD_5X8_FONT, COMMAND)) return false; //2 lines and 5x8 font
  if (!sendDataSingleByte(LCD_ON_OFF | LCD_DISPLAY_ON | LCD_CURSOR_OFF | LCD_BLINKS_OFF, COMMAND)) return false; //Display on, cursor off, no blinks
  if (!displayClear()) return false; //This command is handled in its own function
  if (!sendDataSingleByte(LCD_ENTRY_MODE_SET | LCD_INCREMENT | LCD_DISPLAY_NO_SHIFT, COMMAND)) return false; //entry mode: increment and no shift 
  if (!displayReturnHome()) return false;
  return true;
}

bool displayClear() {
  //Send command to clear the display (and return home?)
  bool response = sendDataSingleByte(LCD_CLEAR_DISPLAY, COMMAND);
  _delay_ms(2); //Clearing the display takes some time
  return response;
}

bool displayReturnHome() {
  //Return to start position
  bool response = sendDataSingleByte(LCD_RETURN_HOME, COMMAND);
  _delay_ms(2); //This command takes some time
  return response;
}

bool displaySetPos(byte row, byte col) {
  //Set position of the cursor before writing to display. Set Display Data RAM Address
  //row = 0 or 1, col = 0 ... 15
  //Set Display Data RAM Address (aaaaaaa)2 to the Address Counter. Display data RAM can be read or written after this setting.
  //In one­line display (N = 0), (aaaaaaa)2: (00)16 - (4F)16. 
  //In two­line display (N = 1), (aaaaaaa)2: (00)16 - (27)16 for the first line, (aaaaaaa)2: (40)16 - (67)16 for the second line.
  byte pos = row * 0x40 + col; 
  return sendDataSingleByte(LCD_SET_DDRAM_ADDRESS | pos, COMMAND); //0b10000000 | position
}

bool displayWriteCharacter(char c) {
  //Send character to show on the LCD at current position
  return sendDataSingleByte(c, DATA); 
}

bool displayWriteText(char* text) {
  //Write some text on the display
  while (*text) { //Loop until pointer hits the null terminator '\0' as end of string
    if (!displayWriteCharacter(*text++)) return false;
  }
  return true;
}

void displayFill(byte character) {
  //Show all segments of the LCD 
  for (int i = 0; i < 16; i++) {
    displaySetPos(0, i);
    displayWriteCharacter(character);
    displaySetPos(1, i);
    displayWriteCharacter(character);
  }
}

void displayOff() {
  //Cut the power to the display.
  sendDataSingleByte(LCD_ON_OFF, COMMAND); //Display off, cursor off, blinks off
  _delay_ms(10);
  PORTB &= ~(1 << PIN_DISPLAY_VCC); //Pin controling power to the LCD via NPN transistor (PN2222A). OFF: input, with no pullup (high impedance). ON: output HIGH
  DDRB &= ~(1 << PIN_DISPLAY_VCC);
  dataHigh(); //Set data lines (SDA and SCL) in tri-state mode to prevent current leak when sleeping (input without internal pullup)
  clockHigh();
}

void showValue(byte value, bool updateText) {
  //Show the given value on the display as a decimal, hexadecimal and binary values.
  
  //Calculate the digits to show
  //Hexadecimal digits
  byte hex1 = value & 0x0F; //low nibble
  byte hex2 = value >> 4; //high nibble

  //Decimal digits 
  //byte dec3 = value / 100; //integer values since the data type is byte/uint8_t
  //byte dec2 = (value % 100) / 10;
  //byte dec1 = value % 10;
  //The above calculation is time consuming, therefore the program uses a look up table instead, as below.
  byte dec1 = pgm_read_byte(&(dec1_table[value])); //Use look up table to make this operation much faster (from ~80 cycles (80us) to ~3 cycles(<1us))
  byte dec2 = pgm_read_byte(&(dec2_table[value])); //Look up tables are stored in PROGMEM (flash) in order to save the dynamic RAM, using pgm_read_byte()
  byte dec3 = pgm_read_byte(&(dec3_table[value])); 

  //Binary digits
  //Calculated as they are shown below

  // Text in LCD display (2 x 16):
  // ▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉
  // ▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉▉
  // DEC: 165 HEX: A5
  // BIN: 10100101      
  
  //Show text and values in LCD 
  //Decimal values
  if (updateText) {
    displaySetPos(0, 0);
    displayWriteText("DEC:");
  }
  displaySetPos(0, 5);
  displayWriteCharacter(0x30 + dec3); //Find the right ASCII value for the digits 0-9 (they are ordered from 0x30 for 0)
  displayWriteCharacter(0x30 + dec2);
  displayWriteCharacter(0x30 + dec1);
  
  //Hexadecimal values
  if (updateText) {
    displaySetPos(0, 9);
    displayWriteText("HEX:");
  }
  displaySetPos(0, 14);
  byte asciiHex1 = (hex1 > 9)? 0x41 - 10 + hex1 : 0x30 + hex1; //As for the decimal but special treatment of A-F
  byte asciiHex2 = (hex2 > 9)? 0x41 - 10 + hex2 : 0x30 + hex2;
  displayWriteCharacter(asciiHex2);
  displayWriteCharacter(asciiHex1);
  
  //Binary values
  if (updateText) {
    displaySetPos(1, 0);
    displayWriteText("BIN:");
  }
  displaySetPos(1, 5);
  for (int i = 7; i >= 0; i--) {
    if ((value & (1 << i)) == 0) { //Evaluate bit by bit
      displayWriteCharacter(0x30); //Write 0
    } else {
      displayWriteCharacter(0x31); //Write 1
    }
  }

}

