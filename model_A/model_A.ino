//
//    ┏━━━━━━━━━━━━━━━━━━━┱─────────┐
//    ┃ DEC-HEX Converter ┃ model A │
//    ┗━━━━━━━━━━━━━━━━━━━┹─────────┘
//             Oskar Fornander, 2025
//


//    DEC-HEX Converter, Oskar Fornander, July-October 2025
//    ATtiny85/45, seven segment displays or LCD and a rotary encoder, driven by 3xAAA/AA-batteries.
//    Power consumption on average is about 70mA (~21 segments on). With AAA-batteries at 1000mAh it can be on for about 14 hours.
//      Model A: Shift registers (74HC595) and seven segment displays, with power button.
//      Model B: LCD-display with I2C-module. No power button(?)
//    This device will simultaneously count in decimal (0-255) and hexadecimal (00-FF) and in that way work as a converter between the two bases.
//    The code is optimized for speed, efficient use of memory and low power consumption on the ATtiny85 or ATtiny45, running at 8MHz.
//    
//    This file contains code for model A
//      The device clears the display and enters sleep mode automatically after some time of inactivity to save battery. Can be woken by turning the rotary encoder or reset by pressing (or turned off with power button).
//      Note: even in sleep mode the device draws just under 1mA (compared to about 70mA when running), so the batteries would last as most up to about 45 days. It is the shift registers (74HC595) that each draws around 80uA in this mode, and there is no easy way around this.
//      In the power saving mode the displays are flashing once every minute (64s) to let the user know it is still turned on.



/*
Connections to the ATtiny85:

pin  Arduino pin type            connection
----|-----------|---------------|--------------
1    PB5/RESET                   Push button   | (Push button with external pull up resistor)
2    PB3         INPUT           B             | Rotary encoder, low pass filter and external pull up resistor for both A and B. 
3    PB4         OUTPUT          Clock (SRCLK) | Shift register 74HC595
4    -                           GND
5    PB0         OUTPUT          Data (SER)    |
6    PB1         OUTPUT          Latch (RCLK)  |
7    PB2         INPUT           A             | (This pin can have interrupt)
8    -                           VCC

Reset button will reset the ATtiny85, so the start up sequence must be short and reset the counter and display 0 at the displays.


Displays:
Decimal: 3 x seven segment displays
Hexadecimal: 2 x seven segment displays

All is controlled with negative logic, values for each digit stored in memory, and bits inverted when sent.
Bits are sent to shift register in MSB first order: Hex, Dec. 

5 Shift registers (74HC595) are used in series. Three pins control Data (connected in series), Clock and Latch (common for all registers).
Output enable (OE) for all shift registers are connected to an RC-circuit with a transistor to delay showing data on the displays at startup, allowing the reset sequence to finnish first.
The outputs of the shift registers A-G are connected, in order, to seven segment digits G-A, H and DP are not connected.

*/

#include <avr/pgmspace.h>

#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>
//#include <EEPROM.h>

//Define pins
#define PIN_ROTARY_ENCODER_A PB2
#define PIN_ROTARY_ENCODER_B PB3

#define PIN_DISPLAY_DATA PB0
#define PIN_DISPLAY_LATCH PB1
#define PIN_DISPLAY_CLOCK PB4

//Define constants and variables for sleep
#define WAIT_FOR_SLEEP 60000 //1 * 60 * 1000 ms, i.e. 1 minutes
#define WAKE_UP_DELAY 1000 //1 second
#define WDT_CYCLES 1 //Blink display after this many sleep cycles: 1*8=8s (Watchdog timer interval for sleep: 8s)
unsigned long lastTimeActive = 0; //Stores the time millis() after the last activity, to check for millis() - lastTimeActive > WAIT_FOR_SLEEP
unsigned long wakeUpTime = -WAKE_UP_DELAY; //millis() when woken up. Used to ignore rotation of the rotary encoder for a moment after wake up. Set to negative value to ignore this at startup and reset.
volatile uint8_t sleepCycles = 0; //Count the number of sleep cycles (at most WDT_CYCLES as defined above)
volatile bool externalWakeup = false; //Device is woken by external interrupt 


byte counter = 0; //global counter variable in interval [0, 255], this is the value to be displayed in decimal and hexadecimal

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

byte sevenSegmentDigit[16] = { //Values for each digit, 1 for segment on and 0 for segment off. Decimal point is not used. Segments in this order: .abcdefgh (decimal point is MSB, h is LSB) Byte is sent MSB fist.
  0b01111110,  //0
  0b00110000,  //1
  0b01101101,  //2
  0b01111001,  //3
  0b00110011,  //4
  0b01011011,  //5
  0b01011111,  //6
  0b01110000,  //7
  0b01111111,  //8
  0b01110011,  //9
  0b01110111,  //A
  0b00011111,  //B
  0b01001110,  //C
  0b00111101,  //D
  0b01001111,  //E
  0b01000111}; //F
byte sevenSegmentBlank = 0b00000000; //Blank display
byte sevenSegmentFull = 0b11111111; //Full display

//Look up tables for decimal digits, used to speed up the calculations (from ~80 cycles (80us) to ~3 cycles(375ns)). These tables takes 3 * 256 bytes of memory. Stored in Progmem to save space in the dynamic memory for global variables. This way the program can fit on an ATtiny45 as well.
const uint8_t dec1_table[256] PROGMEM = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 1, 2, 3, 4, 5};
const uint8_t dec2_table[256] PROGMEM = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5};
const uint8_t dec3_table[256] PROGMEM = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2};


void setup() {
  //Pins for Shift registers (74HC595) controlling seven segment displays
  //DDRB -> set direction (0 = input, 1 = output)
  PORTB &= ~(1 << PIN_DISPLAY_CLOCK); //Clock for shift registers (initial value: LOW)
  PORTB &= ~(1 << PIN_DISPLAY_LATCH); //Latch for shift registers (initial value: LOW)
  DDRB |= (1 << PIN_DISPLAY_DATA) | (1 << PIN_DISPLAY_CLOCK) | (1 << PIN_DISPLAY_LATCH); //Set these three pins as outputs (1): data, clock and latch (faster than pinMode())

  resetDisplays(); //Reset displays as soon as possible after startup, since the RESET button is used to reset the counter (by restarting the ATtiny85). 
                   //This operation, resetDisplay(), takes <100us. The whole upstart sequence, from releasing RESET-button until device is on and displays are reset, takes <100ms (measured to approximately 75ms).

  //Pins for Rotary encoder (push button is connected to reset pin)
  pinMode(PIN_ROTARY_ENCODER_A, INPUT_PULLUP); //Internal and external pullup resistors
  pinMode(PIN_ROTARY_ENCODER_B, INPUT_PULLUP);
  //Using polling instead of interrupt. // attachInterrupt(0, rotationDetected, FALLING); //Attach an interrupt service routine to the rotary encoder pin A on falling edge. A must be connected to PB2 (pin 7) for INT0

  setupWDT(); //Setup Watchdog timer for handling sleep cycles
  sei(); //Set global interrupts, these must be active to be able to wake the unit from sleep

  //Set starting values for these flags
  rotaryStates.currentA = 1;
  rotaryStates.currentB = 1;
  rotaryStates.prevA = 1;
  rotaryStates.prevB = 1;
  rotaryStates.counter = 0;
  rotaryStates.valueChanged = 0;
}

void rotationDetected() {
  //Interrupt service routine (ISR)
  //An RC-filter and pull up resistor are used for each of the rotary encoder's terminals.
  //Rotation is detected from A as interrupt
  //rotationFlag = true; //Set flag to indicate rotation. Logic is checked inside loop().
  //
}

ISR(PCINT0_vect) {
  //This ISR is called when PB2 (PCINT2) gives an interrupt.
  //Used only to wake up from sleep.
  wdt_disable(); //stop watchdog timer
  externalWakeup = true; //mcu is no longer in sleep mode, woken by external interrupt
  sleepCycles = 0;
}
ISR(WDT_vect) {
  //This ISR is called when the mcu wakes up from sleep by the Watchdog timer.
  sleepCycles++;
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
    showValue(counter); //Update value (~175us?)
    rotaryStates.valueChanged = 0; //Clear flag
    lastTimeActive = millis(); //Update the last time of activity on the device, i.e. reset the sleep timer
  }

  if (millis() - lastTimeActive > WAIT_FOR_SLEEP) {
    goToSleep(); //Time to shut down after a long time of inactivity
  }
}

void setupWDT() {
  //Set up Watchdog timer to wake mcu from sleep (but not reset it)
  MCUSR &= ~(1 << WDRF); //Inactivate WDT-reset 
  wdt_reset(); //clear WDT before reconfiguration
  WDTCR |= (1 << WDCE) | (1 << WDE); //Change temporary register (needed to change WDE/WDIE)
  WDTCR = (1 << WDIE) | (1 << WDP3) | (1 << WDP0); //WDIE (WDT interrupt enable) activates ISR, WDP3 adn WDP0: prescaler for 8 seconds
  //WDT is running, generating an interrupt once every 8 seconds
}

void goToSleep() {
  //This can be used as an automatic power saving feature, after the unit has been left turned on for some time. Turn off the led displays and let the ATtiny85 go into deep sleep to save power, from a few mA down to below 1uA
  sleepCycles = 0; //Reset before entering sleep (to avoid a flash as the display clears)

  //### 1. Initial shut down
  clearDisplays(); //Display off: turn off all segments of all five seven segement displays
  ADCSRA &= ~(1 << ADEN); //Turn off ADC
  power_adc_disable();
  power_timer0_disable();
  power_timer1_disable();
  power_usi_disable();
  // Leave PCINT on to enable interrupt for wake up without restarting the code (running setup())
  GIFR |= (1 << PCIF); //Clear Pin change interrupt flag  
  GIMSK |= (1 << PCIE); //Enable pin change interrupts
  PCMSK |= (1 << PCINT2); //Enable interrupt on PB2 (ISR declared above)
  setupWDT(); //Configure Watchdog timer for periodical wake up
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_bod_disable(); //Turn off BOD

  //### 2. Cyclic sleep and flashing of the display
  while (!externalWakeup) { //Run as long as the device is not woken up by external interrupt (a turn of the rotary encoder knob)
    if (sleepCycles >= WDT_CYCLES) { //after a given number of cycles: blink the display
      flashDisplays(); //Flash all five displays for a moment to call the users attention of the unit still being turned on
      sleepCycles = 0; //Reset counter of sleep cycles
    }
    //Go to sleep again
    sleep_enable();
    sleep_cpu(); //Activate sleeping mode
    // --- MCU is sleeping --- //
    sleep_disable();
  }

  //### 3. Wake up (by external interrupt) - mcu should not go to sleep again
  GIMSK &= ~(1 << PCIE); //Disable pin change interrupts
  power_all_enable();
  externalWakeup = false;
  sleepCycles = 0;
  wakeUpTime = millis();
  lastTimeActive = millis();
  showValue(counter); //Show the current value on the displays again after wake up, the value is still in the variable counter when woken by a turn of the knob, if reset by press it is cleared and the unit restarts from scratch.
}



//################
void shortDelay() {
  //no-operation gives 125ns of delay at 8MHz, instead of delayMicroseconds(1), about up to 100ns is needed.
  //delayMicroseconds(1);
  __asm__("nop\n\t"); //nop
}

void setDataBit() {
  //Set data bit for display to 1 (send a 1)
  PORTB |= (1 << PIN_DISPLAY_DATA);
}
void clearDataBit() {
  //Clear data bit for display to 0 (send a 0)
  PORTB &= ~(1 << PIN_DISPLAY_DATA);
}

void latch() {
  //Show data in storage registers on outputs, i.e. show the data on the dispays after changing
  shortDelay();
  PORTB |= (1 << PIN_DISPLAY_LATCH); //Set Latch pin HIGH
  shortDelay(); 
  PORTB &= ~(1 << PIN_DISPLAY_LATCH); //Set Latch pin LOW
  shortDelay();
}

void shiftByte(byte value, bool inverted = true) {
  //Shift out this byte to the shift register 74HC595, MSB first
  //Invert the bits sent if inverted is true (default); we use positive logic in code but the seven segment displays use negative logic, so 0 and 1 are swapped.
  for (int i = 0; i < 8; i++) { //Loop through all bits in the byte to send
    if (!!(value & 0x80) == inverted) { //Determine wether to send 0 or 1. (value & 0x80) is the current MSB in the byte to send, using double negation (!!) to get a value of 0 or 1.
      clearDataBit(); //Value is 0 and it shall not be inverted, or value is 1 and it shall be inverted -> send 0
    } else {
      setDataBit();   //Value is 0 and it shall be inverted, or value is 1 and it shall not be inverted -> send 1
    }
    /* //A less convoluted way of coding the same if statement as above:
    if (value & 0x80) { //Bit (MSB) is 1
      if (inverted) {
        clearDataBit(); //invert and send 0
      } else {
        setDataBit(); //send 1
      }
    } else { //Bit (MSB) is 0
      if (inverted) {
        setDataBit();
      } else {
        clearDataBit();
      }
    }
    */
    value <<= 1; //Shift out the bits one by one, sending the MSB first, value must be updated before sending next bit.
    PORTB |= (1 << PIN_DISPLAY_CLOCK); //Toggle clock HIGH and LOW
    shortDelay();
    PORTB &= ~(1 << PIN_DISPLAY_CLOCK); 
    shortDelay();
  }
}

void showValue(byte value) {
  //Show the given value on the displays as a decimal and a hexadecimal value
  /*
  Seven segments displays:

  DECIMAL:         HEX:  
  ┏━┓ ┏━┓ ┏━┓      ┏━┓ ┏━┓      
  ┣━┫ ┣━┫ ┣━┫      ┣━┫ ┣━┫
  ┗━┛ ┗━┛ ┗━┛      ┗━┛ ┗━┛

  Bytes are shifted in this direction, MSB first –––>
  */
  
  //Send hexadecimal digits
  byte hex1 = value & 0x0F; //low nibble
  byte hex2 = value >> 4; //high nibble
  shiftByte(sevenSegmentDigit[hex1]); //Shift out the correct bytes to show the digits of this hexadecimal number
  shiftByte(sevenSegmentDigit[hex2]);

  //Send decimal digits 
  //byte dec3 = value / 100; //integer values since the data type is byte/uint8_t
  //byte dec2 = (value % 100) / 10;
  //byte dec1 = value % 10;
  //The above calculation is time consuming, therefore the program uses a look up table instead, as below.
  byte dec1 = pgm_read_byte(&(dec1_table[value])); //Use look up table to make this operation much faster (from ~80 cycles (80us) to ~3 cycles(<1us))
  byte dec2 = pgm_read_byte(&(dec2_table[value])); //Look up tables are stored in PROGMEM (flash) in order to save the dynamic RAM, using pgm_read_byte()
  byte dec3 = pgm_read_byte(&(dec3_table[value])); 
  shiftByte(sevenSegmentDigit[dec1]); //Shift out the correct bytes to show the digits of this decimal number
  //shiftByte(sevenSegmentDigit[dec2]);
  //shiftByte(sevenSegmentDigit[dec3]);
  //Handling numbers with leading zeroes
  shiftByte((dec2 == 0 && dec3 == 0)? sevenSegmentBlank: sevenSegmentDigit[dec2]); //Show digit 2 as blank if single digit number
  shiftByte((dec3 == 0)? sevenSegmentBlank: sevenSegmentDigit[dec3]); //Show digit 3 as blank if two digit number

  latch();
}

void resetDisplays() {
  //Quic reset at start up
  //Start up/reset values should be: _ _ 0, 0 0

  PORTB &= ~(1 << PIN_DISPLAY_LATCH); //Set Latch pin LOW

  //HEXADECIMAL
  shiftByte(sevenSegmentDigit[0]); //Send unit digit of the hexadecimal value (0)
  shiftByte(sevenSegmentDigit[0]); //Send 16th digit of the hexadecimal value (0)
  
  //DECIMAL
  shiftByte(sevenSegmentDigit[0]); //Send unit digit of the decimal value: 0
  shiftByte(sevenSegmentBlank); //10th digit, use 0 or blank?
  shiftByte(sevenSegmentBlank); //100th digit, use 0 or blank?

  latch();
}

void clearDisplays() {
  //Clear all displays (all segments off). Can be used before entering sleep mode.
  PORTB &= ~(1 << PIN_DISPLAY_LATCH); //Set Latch pin LOW
  for (int i = 0; i < 5; i++) {
    shiftByte(sevenSegmentBlank); //All segments off for all five digits
  }
  latch();
}
void fillDisplays() {
  //Turn on all segments in the displays.
  PORTB &= ~(1 << PIN_DISPLAY_LATCH); //Set Latch pin LOW
  for (int i = 0; i < 5; i++)  {
    shiftByte(sevenSegmentFull); //All segments on for all five digits
  }
  latch();
}
void flashDisplays() {
  //Flash all five displays in sleep mode to alert the user that the unit is not turned off.
  PORTB &= ~(1 << PIN_DISPLAY_LATCH); //Set Latch pin LOW
  for (int i = 0; i < 10; i++) { //These loops create a running light of the middle segments
    shiftByte(i < 5); //Middle segment on for all five digits: 0b00000001, all off: 0b00000000 (true and false are evaluated as 1 and 0)
    latch();
    _delay_ms(50); //The normal delay() function cannot be used since Timer0 is deactivated
  }
}
