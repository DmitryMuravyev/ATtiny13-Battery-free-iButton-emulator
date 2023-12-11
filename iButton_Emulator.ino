/*
  iButton_Emulator.c - Battery-free iButton emulator based on ATtiny13 microcontroller.
  
  Copyright (c) 2022 Dmitry Muravyev. All right reserved.

  MIT License

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


///////////////////////////////////////////////////////////////////////////////////////////////////////////
// Constants, types and definitions
///////////////////////////////////////////////////////////////////////////////////////////////////////////
//

#define clock_speed     8000000                 // CPU speed ~8Mhz

const uint8_t timeslot_divider = (1000000 / (clock_speed / 64));        // Constant for calculating timer clock cycles to control timeslots in microseconds
const uint8_t one_wire_master_pin = 0;          // iButton key programming pin
const uint8_t one_wire_slave_pin = 1;           // Intercom pin
const uint8_t led_pin = 4;                      // LED pin
const uint8_t battery_pin = 3;                  // Battery power pin
const uint8_t function_pin = 2;                 // Function button pin
const uint8_t read_ROM_command = 0x33;          // Read ROM command
const uint8_t search_ROM_command = 0xF0;        // Search ROM command
const uint8_t skip_ROM_command = 0xCC;          // Search ROM command
const uint8_t EEPROM_size = 64;                 // EEPROM capacity
const uint8_t alternate_key_position = 8 * 3;   // EEPROM address of the additional keys


///////////////////////////////////////////////////////////////////////////////////////////////////////////
// Variables
///////////////////////////////////////////////////////////////////////////////////////////////////////////
//

static uint8_t dataBytes[8];                    // Buffer for 1-Wire data
static bool slaveMode = false;                  // Programming mode
static uint8_t rom_addr = 0;                    // Address of the current key ROM


///////////////////////////////////////////////////////////////////////////////////////////////////////////
// Interrupts
///////////////////////////////////////////////////////////////////////////////////////////////////////////
//

EMPTY_INTERRUPT(INT0_vect);                     // We can save two more bytes if we use assembler and the iret command, but I haven't found an easy way to do this.
ISR(WDT_vect, ISR_ALIASOF(INT0_vect));
ISR(PCINT0_vect, ISR_ALIASOF(INT0_vect));
ISR(TIM0_OVF_vect, ISR_ALIASOF(INT0_vect));
ISR(EE_RDY_vect, ISR_ALIASOF(INT0_vect));
ISR(ANA_COMP_vect, ISR_ALIASOF(INT0_vect));
ISR(TIM0_COMPA_vect, ISR_ALIASOF(INT0_vect));
ISR(TIM0_COMPB_vect, ISR_ALIASOF(INT0_vect));
ISR(ADC_vect, ISR_ALIASOF(INT0_vect));


///////////////////////////////////////////////////////////////////////////////////////////////////////////
// Implementation
///////////////////////////////////////////////////////////////////////////////////////////////////////////
// Delay and sleep functions
///////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Delay for microseconds
// Since the only timer is busy by controlling timeslots - we cannot use it directly, so we will use workaround based on processor cycles.
void delayMicros(uint16_t microseconds) {

// Small cycle for less than 10 microseconds
    if (microseconds < 10) {
        __asm__ __volatile__ (
            "nop                    \n"         // 1 cycle
            "nop                    \n"         // 1 cycle
            "1: dec  %A[counter]    \n"         // 1 cycle
            "brne 1b                \n"         // 2 cycles
            :
            // Output operands
            :
            // Input operands
            [counter] "r" (microseconds)
            :
            // Clobbers
        );
        return;
    }

    microseconds -= 3;              // Correction for function initialization time (you can play with this parameter to get the best results over the entire range)

// Large cycle for large delays
    __asm__ __volatile__ (
        "1: nop                     \n"         // 1 cycle
        "nop                        \n"         // 1 cycle
        "nop                        \n"         // 1 cycle
        "nop                        \n"         // 1 cycle
        "subi %A[counter], 1        \n"         // 1 cycle
        "sbci %B[counter], 0        \n"         // 1 cycle
        "brcc 1b                    \n"         // 2 cycles

        //
        // We have 8 CPU cycles in total, so DELAY = 8MHz / (microseconds * 8 cycles)
        //

        :
        // Output operands
        :
        // Input operands
        [counter] "r" (microseconds)
        :
        // Clobbers
    );
}


// Delay for N x 0.05s
void delay005s(uint8_t milliseconds) {
    while (--milliseconds) delayMicros(50000);
}


// Power down sleep mode
void powerDown() {
    DIDR0 = 0xFF;                                           // Disabling digital input circuits.
    __asm__ __volatile__ ( "wdr   \n" :: );                 // Reset Watchdog timer
    WDTCR = _BV(WDCE);
    if ((MCUSR & _BV(PORF)) > 0) {                          // Check if just powered on
        MCUSR = 0x00;
//      WDTCR = _BV(WDTIE) | _BV(WDP0);                     // Sleep for 32ms (the first sleep after powering on to stabilize the processes)
        WDTCR = _BV(WDTIE) | _BV(WDP1);                     // Sleep for 64ms (the first sleep after powering on to stabilize the processes)
    } else {
//      WDTCR = _BV(WDTIE) | _BV(WDP2);                     // Sleep for 0.25s before processing the next key ROM
        WDTCR = _BV(WDTIE) | _BV(WDP0) | _BV(WDP2);         // Sleep for 0.5s before processing the next key ROM
    }
    MCUCR = _BV(SE) | _BV(SM1);                             // Sleep Enable, power-down mode.
    __asm__ __volatile__ ( "sleep   \n" :: );
    DIDR0 = 0;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////
// OneWire functions (based on this project: http://www.technoblogy.com/show?2G8A)
///////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Configure OneWire pin as OUTPUT (low level was pre-configured in setup())
void pinLow() {
    if (slaveMode) {
        DDRB = DDRB | (1 << one_wire_slave_pin);
    } else {
        DDRB = DDRB | (1 << one_wire_master_pin);
    }
}


// Configure OneWire pin as INPUT
void pinRelease() {
    if (slaveMode) {
        DDRB = DDRB & ~_BV(one_wire_slave_pin);
    } else {
        DDRB = DDRB & ~_BV(one_wire_master_pin);
    }
}


// Read OneWire pin
uint8_t pinRead() {
    if (slaveMode) {
        return ((PINB >> one_wire_slave_pin) & 1);
    } else {
        return ((PINB >> one_wire_master_pin) & 1);
    }
}


// Transmit inverted 1-0 sequence
void pinLowRelease(uint16_t low, uint16_t high) {
    pinLow();
    delayMicros(low);
    pinRelease();
    delayMicros(high);
}


// Initialize OneWire
uint8_t oneWireReset() {
    pinLowRelease(480, 70);
    uint8_t resetResult = pinRead();
    delayMicros(410);
    return (resetResult);
}


// Write bit in Master mode
void masterWriteBit(uint8_t data) {
uint16_t low, high;
    if (data == 0) {
        low = 65;
        high = 5;
    } else {
        low = 10;
        high = 55;
    }
    pinLowRelease(low, high);
}


// Write byte in Master mode
void masterWriteByte(uint8_t data) {
    for (uint8_t i = 0; i < 8; i++) {
        masterWriteBit(data & 1);
        data = data >> 1;
    }
}


// Read byte in Master mode
uint8_t masterReadByte() {
    uint8_t data = 0;
    for (uint8_t i = 0; i < 8; i++) {
        pinLowRelease(3, 10);
        data = data | (pinRead() << i);
        delayMicros(53);
    }
    return data;
}


// Read bytes into the buffer
void masterReadBytes(uint8_t bytes) {
    for (uint8_t i = 0; i < bytes; i++) {
        dataBytes[i] = masterReadByte();
    }
}


// Reset timer counter and flags
void resetTimer() {
    TCNT0 = 0;
    TIFR0 = 0xFF;
}


// Wait for high level in Slave mode
void waitForHigh() {
    while ((PINB & _BV(one_wire_slave_pin)) == 0);
    GIFR = _BV(INTF0);                          // Reset INT0 Interrupt Flag
}


// Wait for low level in Slave mode
void waitForLow() {
    while ((GIFR & _BV(INTF0)) == 0);           // Detect falling edge
}


// Write bit in Slave mode
bool slaveWriteBit(uint8_t data) {
    waitForLow();
    bool timeslotCheckResult = (TIFR0 & _BV(OCF0A));
    resetTimer();
    if ((data & 1) == 0) {
        pinLowRelease(30, 1);
    }
    waitForHigh();
    return(timeslotCheckResult);
}


// Read bit in Slave mode
uint8_t slaveReadBit() {
    waitForLow();
    uint8_t data = (TIFR0 & _BV(OCF0A));
    resetTimer(); 
    delayMicros(30);
    data |= pinRead();              // Merge data whith timeslot check result
    waitForHigh();
    return (data);
}


// Calculate CRC over the buffer - 0x00 is correct
uint8_t oneWireCRC(uint8_t len) {
    uint8_t crc = 0;
    for (uint8_t j = 0; j < len; j++) {
        crc = crc ^ dataBytes[j];
        for (uint8_t i = 0; i < 8; i++) crc = (crc >> 1) ^ ((crc & 1) ? 0x8c : 0);
    }
    return crc;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////
// EEPROM functions
///////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Write byte to EEPROM
void EEPROM_write(uint8_t ucAddress, uint8_t ucData) {
    while(EECR & (1<<EEPE));        // Wait for completion of previous write
    EECR = (0<<EEPM1)|(0>>EEPM0);   // Set Programming mode
    EEARL = ucAddress;              // Set up address and data registers
    EEDR = ucData;
    EECR |= (1<<EEMPE);             // Write logical one to EEMPE
    EECR |= (1<<EEPE);              // Start eeprom write by setting EEPE
}

// Read byte from EEPROM
unsigned char EEPROM_read(uint8_t ucAddress) {
    while(EECR & (1<<EEPE));        // Wait for completion of previous write
    EEARL = ucAddress;              // Set up address register
    EECR |= (1<<EERE);              // Start eeprom read by writing EERE
    return EEDR;                    // Return data from data register
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////
// Other functions
///////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Blink N times
void blink(uint8_t count) {
    while (count--) {
        PORTB = PORTB & ~(1 << led_pin);        // Turn LED off
        delay005s(10);                          // 0.5s
        PORTB = PORTB | (1 << led_pin);         // Turn LED on
        delay005s(10);                          // 0.5s
    }
}


// Pulse generation for oscillator calibration
void pulse(uint16_t duration) {
    pinLow();
    delayMicros(duration);
    pinRelease();
    delayMicros(duration);
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////
// Main loop
///////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//
int main() {

// Setup timer
//  TCCR0A = 0;                     // Normal mode (default value)
    TCCR0B = _BV(CS01) | _BV(CS00); // 64 - prescaler (clk / 64)

// Power options
    PRR = 0xFF;                     // Disable timer and ADC
    ACSR = _BV(ACD);                // Disable Analog Comparator

// Configure pins
    DDRB = _BV(led_pin);                                    // Configure all pins as inputs excepting the LED pin
    PORTB = 0xFF ^ (_BV(battery_pin) | _BV(led_pin));       // All up excepting battery and led pins

// Pulse generation for oscillator calibration
/*
    masterMode = true;
    PORTB = PORTB & ~_BV(one_wire_master_pin);              // Remove pull-up from iButton pin

 calibration:

    pulse(480);
    pulse(410);
    pulse(70);
    pulse(65);
    pulse(55);
    pulse(53);
    pulse(10);
    pulse(5);
    pulse(3);

    delay005s(20);

    goto calibration;
*/

    sei();                          // Enable interrupts  
    powerDown();                    // First sleep after powering on to stabilize the processes

// Another way to save energy is to dramatically reduce the clock frequency. But we will not use it, because it is less effective than power-down sleep mode.
//  CLKPR = _BV(CLKPCE);            // Enable changes
//  CLKPR = _BV(CLKPS3);            // 256 - prescaler (clk / 256)

    OSCCAL = 60;                    // 0-127 Oscillator Frequency calibration value: set the frequency to ~8 MHz (should be calibrated manually)
  
    bool function = ((PINB & _BV(function_pin)) == 0);      // Read function button state

    if (PINB & _BV(battery_pin)) {                          // Check if the emulator powered by battery

        PORTB |= _BV(led_pin);                              // LED on
        PORTB = PORTB & ~_BV(one_wire_master_pin);          // Remove pull-up from iButton pin

        if (function) {
            for (uint8_t i = 0; i < EEPROM_size; i++) EEPROM_write(i, 0xFF);    // Erase EEPROM before new programming
            blink(4);
        }

        while (true) {
            delay005s(20);

            if (oneWireReset() == 0) {                      // Reset OneWire

                masterWriteByte(read_ROM_command);          // Read iButton ROM
                masterReadBytes(8);
        
                if (dataBytes[7] && (oneWireCRC(8) == 0)) { // Check ROM CRC

                        uint8_t i = 0;
                        bool notFound;
                        do {                                // Search for the ROM in the saved list
                            notFound = false;
                            for (uint8_t j = 0; j < 8; j++) {
                                uint8_t val = EEPROM_read(i + j);
//                              if ((~val | j) == 0) {      // Beware of integer promotion when performing bitwise operations on integer types smaller than int:
                                                            //  https://wiki.sei.cmu.edu/confluence/display/c/EXP14-C.+Beware+of+integer+promotion+when+performing+bitwise+operations+on+integer+types+smaller+than+int
                                if (((val ^ 0xFF) | j) == 0) {                  // End of list reached
                                    notFound = true;
                                    goto writeEEPROM;
                                }

                                if (val != dataBytes[j]) {                      // Doesn't match. Go to the next one.
                                    notFound = true;
                                    break;
                                }
                            }
                        } while (notFound && ((i += 8) < EEPROM_size));

writeEEPROM:
              
                        if (notFound && (i < EEPROM_size)) {                    // If the ROM is not found and we have not reached the EEPROM limits
                            blink(1);
                            uint8_t j = 0;
                            for (j = 0; j < 8; j++) {
                                EEPROM_write(i + j, dataBytes[j]);              // Save new ROM to the list
                            }
                        } else {
                            blink(2);                       // ROM was found or EEPROM is full
                        }
                } else {
                        blink(3);                           // ROM CRC Error
                }
            }
        }

    }


    DDRB |= _BV(battery_pin);                   // Set battery pin as outut
    if (function) {                             // If function key pressed (pulled to ground)...
        DDRB |= _BV(function_pin);              // configure function pin as output
        rom_addr = alternate_key_position;      // and start reading ROMs from the alternate position
    }
    slaveMode = true;

Loop:

    PORTB = 0xFF ^ DDRB;                        // Invert pull-ups (1 for inputs, 0 for outputs - as far as I could measure, this configuration consumes the least energy)

    powerDown();                                // Sleep and accumulate energy
  
    PORTB = PORTB & ~_BV(one_wire_slave_pin);   // Remove pull-up from intercom pin

    uint8_t repeats = 2;            // The number of repeats for one key (current key ROM will be issued N+1 times)

// Wait for 1-Wire Reset pulse
WaitForResetPulse:

    PRR = 0xFF;                     // Disable timer and ADC

    GIFR = _BV(INTF0);
    GIMSK = _BV(INT0);              // Enable INT0 interrupts

    MCUCR = _BV(SE) | _BV(SM1);     // Sleep Enable, Power-down mode, INT0 Low level detection mode
    __asm__ __volatile__ ( "sleep  \n" :: );

    GIMSK = 0;                      // Disable INT0 interrupts
    MCUCR = _BV(ISC01);             // INT0 falling edge detection mode
    PRR = _BV(PRADC);               // Enable Timer

// Setup timer to control timeslots. To check whether the pulse fits into the timeslot, it is necessary that channel B triggers, but channel A does not trigger.
    OCR0A = 960 / timeslot_divider; // 960us is the maximum pulse duration, 240 is the minimum.
    OCR0B = 240 / timeslot_divider; // We would also need to subtract the ~22 clock cycles that are spent on waking up from sleep, but the one clock cycle of the timer is equal to 64 clock cycles of the CPU, so we will not do this unless absolutely necessary.

    resetTimer();                   // Reset timer counter and flags
  
    waitForHigh();                  // Wait for the end of the Reset pulse

    if (((TIFR0 & _BV(OCF0B)) == 0) || (TIFR0 & _BV(OCF0A))) {                  // Check timeslot error
        goto WaitForResetPulse;
    }

// Send Presence pulse
    delayMicros(30);
    pinLowRelease(120, 1);
    waitForHigh();

    uint8_t data = 0;
    OCR0A = 120 / timeslot_divider; // 120us is the maximum read/write pulse duration
    TCNT0 = 120 / timeslot_divider; // Skip first timeslot check (now we have 2048 microseconds until the next timeout)
  
// Read command from master
ReadCommand:
    for (uint8_t i = 0; i < 8; i++) {
        uint8_t read_data = slaveReadBit();
        data = data | (read_data << i);

        if (read_data > 1) {        // Check timeslot error
            goto Loop;
        }
    }

// Process commands
    if (data == search_ROM_command) {
        // Search ROM command

        uint8_t rom_byte_number = 0;
        uint8_t rom_bit, rom_byte_mask = 1;

        do {
            rom_bit = (EEPROM_read(rom_addr + rom_byte_number) & rom_byte_mask) > 0;    // Get the current ROM bit

            if (slaveWriteBit(rom_bit)) {               // Write the bit into the bus and check timeslot error
                goto Loop;
            }
     
            if (slaveWriteBit(rom_bit ^ 0xFF)) {        // Write the inverted bit into the bus and check timeslot error
                goto Loop;
            }

            data = slaveReadBit();                      // Read master's choice

            if (data > 1) {                             // Check timeslot error
                goto Loop;
            }

            if (data != rom_bit) {                      // Check if the master has chosen another slave
                goto WaitForResetPulse;
            }

            // Iterate through all the bits and bytes of the current ROM
            rom_byte_mask <<= 1;
            if (rom_byte_mask == 0) {
                rom_byte_number++;
                rom_byte_mask = 1;
            }

        } while (rom_byte_number < 8);

    } else {
        // Read ROM command
        // Answer with the current iButton ROM
        for (uint8_t i = 0; i < 8; i++) {
            data = EEPROM_read(rom_addr + i);
            TCNT0 = 120 / timeslot_divider;             // Skip timeslot check between bytes (now we have 2048 microseconds until the next timeout)
            for (uint8_t j = 0; j < 8; j++) {
                if (slaveWriteBit(data & 1)) {          // Check timeslot error
                    goto Loop;
                }
                data = data >> 1;
            }
        }
    }

    if (repeats) {
        repeats--;
        goto WaitForResetPulse;
    }

    // Switch to the next saved ROM
    rom_addr += 8;
    if ((rom_addr >= EEPROM_size) || (!function && ((rom_addr >= alternate_key_position) || (EEPROM_read(rom_addr) == 0xFF)))) {
        if (function) {
            rom_addr = alternate_key_position;
        } else {
            rom_addr = 0;
        }
    }

    goto Loop;

}

// END-OF-FILE
