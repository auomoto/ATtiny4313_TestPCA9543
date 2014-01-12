/*

	Check fuses:
	avrdude -c usbtiny -p ATtiny4313 -P usb -U lfuse:r:-:h
	Burned new fuses: 0xFF 0xDF 0xFF
	>avrdude -p ATtiny4313 -c usbtiny -U lfuse:w:0xff:m
	The 0xFF in lfuse gives the longest startup time with a crystal
	Using a 14.745600 MHz crystal.

	This program was used to test a PCA9543 I2C bus switch. The PCA9543
	can switch between two separate I2C channels or block both or select
	both. This is useful if you have different voltage requirements for
	I2C devices or if you don't have an I2C address conflict (the PN532
	NFID chip has only one I2C bus address option).






*/

#include <util/delay.h>
#include <avr/eeprom.h>
#include <avr/io.h>
#include <avr/wdt.h>
#include "usi_twi_master.h"
#include "mcp23008.h"
#include "pca9543.h"

// A2 A1 A0 on the MCP23008 port expander
#define MCP23008A						0b00000000
#define MCP23008B						0b00000111

#define LEDPIN PB2
#define brightness(X) (OCR0A = X)		// PWM on PB2
#define COMMANDSENT (UCSRA & _BV(RXC))	// Is there something in the RX buffer?
#define TXREADY (UCSRA & _BV(UDRE))		// Is the transmit buffer empty?

void init_ATtiny4313(void);
void sendByte(uint8_t);
void sendCRLF(void);
void sendPrompt(void);
void sendString(char*);

uint8_t EEMEM eeVer[]="ATtiny4313_TestPCA9543 20140112";

int main (void)
{

	uint8_t i, pause, cmd, addr;
	char strBuf[32];

	init_ATtiny4313();
	usi_twi_master_initialize();

	sendPrompt();

	for (;;) {

		if (COMMANDSENT) {

			cmd = UDR;
			sendByte(cmd);							// echo the command

			switch(cmd) {

				case ('\r'):
					break;

				case ('+'):							// On-board LED brighter
					i = OCR0A;
					brightness((i<<1) | 0x01);
					break;

				case ('-'):							// On-board LED fainter
					OCR0A = (OCR0A >> 1);
					break;

				case ('0'):
					pca9543SelectChannel(0);
					break;

				case ('1'):
					pca9543SelectChannel(1);
					break;

				case ('a'):							// Scan lights on board A connected to channel 0
					pca9543SelectChannel(0);
					addr = MCP23008A;
					pause = 25;
					mcp23008_Init(addr);
					for (i = 0; i < 7; i++) {
						mcp23008_Write(addr, GPIO, 1 << i);
						_delay_ms(pause);
					}
					for (i = 0; i < 7; i++) {
						mcp23008_Write(addr, GPIO, 128 >> i);
						_delay_ms(pause);
					}
					mcp23008_Write(addr, GPIO, 255);
					_delay_ms(500);
					mcp23008_Write(addr, GPIO, 0);
					break;

				case ('A'):							// Scan lights on board A (only if correct channel selected)
					addr = MCP23008A;
					pause = 25;
					mcp23008_Init(addr);
					for (i = 0; i < 7; i++) {
						mcp23008_Write(addr, GPIO, 1 << i);
						_delay_ms(pause);
					}
					for (i = 0; i < 7; i++) {
						mcp23008_Write(addr, GPIO, 128 >> i);
						_delay_ms(pause);
					}
					mcp23008_Write(addr, GPIO, 255);
					_delay_ms(500);
					mcp23008_Write(addr, GPIO, 0);
					break;

				case ('b'):							// Scan lights on board B if connected to channel 1
					pca9543SelectChannel(1);
					addr = MCP23008B;
					pause = 25;
					mcp23008_Init(addr);
					for (i = 0; i < 7; i++) {
						mcp23008_Write(addr, GPIO, 1 << i);
						_delay_ms(pause);
					}
					for (i = 0; i < 7; i++) {
						mcp23008_Write(addr, GPIO, 128 >> i);
						_delay_ms(pause);
					}
					mcp23008_Write(addr, GPIO, 255);
					_delay_ms(500);
					mcp23008_Write(addr, GPIO, 0);
					break;

				case ('B'):							// Scan lights on board B (only if correct channel selected)
					addr = MCP23008B;
					pause = 25;
					mcp23008_Init(addr);
					for (i = 0; i < 7; i++) {
						mcp23008_Write(addr, GPIO, 1 << i);
						_delay_ms(pause);
					}
					for (i = 0; i < 7; i++) {
						mcp23008_Write(addr, GPIO, 128 >> i);
						_delay_ms(pause);
					}
					mcp23008_Write(addr, GPIO, 255);
					_delay_ms(500);
					mcp23008_Write(addr, GPIO, 0);
					break;

				case ('r'):
					i = pca9543ControlRegister();
					pca9543SelectChannel(1);
					addr = MCP23008B;
					mcp23008_Write(addr, GPIO, i);
					break;

				case ('R'):							// reset in 500 ms
					WDTCR = 0x98 | 0b00000101;
					while (1) {};
					break;

				case ('V'):							// Print version from EEPROM
					eeprom_read_block((void*) &strBuf, (const void*) &eeVer, 32);
					sendCRLF();
					sendString(strBuf);
					break;

				default:
					sendString("?");
					break;
			}

			sendPrompt();

		}
	}
}


void sendByte(uint8_t x)
{

	while (!TXREADY) {
		asm("nop");
	}

	UDR = x;

}

void sendCRLF(void)
{

	sendByte('\n');
	sendByte('\r');

}

void sendPrompt(void)
{

	sendCRLF();
	sendByte('>');

}

void sendString(char str[])
{

	uint8_t i = 0;

	while (str[i]) {
		sendByte(str[i++]);
	}

}

void errMsg(uint8_t msgNum)
{

	sendByte('E');
	sendCRLF();

}


/*
===========================================================================

	UBRRL and UBRRH USART Baud Rate Registers. 12 bits, computed from:
	
	UBRR = (F_CPU / (16 * Baudrate)) - 1 where "Baudrate" is the common
	understanding.

===========================================================================

	UCSRB Control STatus Register B
	---------------------------------------------------------------------
	Bit	|	7	|	6	|	5	|	4	|	3	|	2	|	1	|	0	|
	Item| RXCIE	| TXCIE	| UDRIE	|  RXEN	|  TXEN	| UCSZ2	|  RXBB	|  TXBB	|
	Set	|	0	|	0	|	0	|	1	|	1	|	0	|	0	|	0	|
	---------------------------------------------------------------------

	RXCIE is RX Complete Interrupt Enable (we won't use this)
	TXCIE is the TX Complete Interrupt ENable
	UDRIE is USART Data Register Empty Interrupt Enable
	RXEN is Receiver Enable
	TXEN is Transmitter Enable
	UCSZ2, combined with UCSZ[1:0] in UCSRC sets the number of data bits
	RXB8 Receive Data Bit 8 (ninth bit of data, if present)
	TXB8 Transmit bit 8 (ninth bit of data)

	UCSRB = 0b00011000

===========================================================================

	UCSRC Control STatus Register C
	---------------------------------------------------------------------
	Bit	|	7	|	6	|	5	|	4	|	3	|	2	|	1	|	0	|
	Item|UMSEL1	|UMSEL0	| UPM1	| UPM0	| USBS	| UCSZ1	| UCSZ0	| UCPOL	|
	Set	|	0	|	0	|	0	|	0	|	0	|	1	|	1	|	0	|
	---------------------------------------------------------------------

	UMSEL[1:0] is the USART mode select. We want 0b00, Asynchronous UART
	UPM[1:0] is Parity mode. 0b00 is disabled, 0b01 reserved, 0b10 Enabled,
	even parity, 0b11 Enabled, odd parity.
	USBS is Stop Bit Select (how many stop bits). 0->1-bit, 1->2-bits.
	UCSZ[1:0] is the rest of the character size description. An 8-bit
	situation says both of these bits should be high.
	UCPOL is clock polarity. Changes fall/rising edges.

	UCSRC = 0b00000110

===========================================================================

	TCCR0A (Timer/Counter Control Register A for Timer0) settings
	---------------------------------------------------------------------
	Bit	|	7	|	6	|	5	|	4	|	3	|	2	|	1	|	0	|
	Item| COM0A1| COM0A0| COM0B1| COM0B0|	-	|	-	| WGM01	| WGM00	|
	Set	|	1	|	0	|	0	|	0	|	0	|	0	|	0	|	1	|
	---------------------------------------------------------------------

	For phase-correct PWM with TOP=OCR0A, WGM0[2:0] should be 0b001 (WGM0[2]
	is in TCCR0B). In phase correct PWM mode the timer counts up, then down.

	We're only using one compare register here so COM0B[1:0] is 0b00 and
	COM0A[1:0] is 0b10, which sets OC0A on the way up and clears it on the
	way down.

	TCCR0A = 0b10000001

===========================================================================

	TCCR0B (Timer/Counter Control Register B for Timer0) settings
	---------------------------------------------------------------------
	Bit	|	7	|	6	|	5	|	4	|	3	|	2	|	1	|	0	|
	Item| FOC0A	| FOC0B	|	-	|	-	| WGM02	|  CS02	|  CS01	|  CS00	|
	Set	|	0	|	0	|	0	|	0	|	0	|	0	|	1	|	0	|
	---------------------------------------------------------------------

	CS0[2:0] are the clock select bits:

		0b000	No clock source (timer stopped)
		0b001	clk/1 (no prescaling)
		0b010	clk/8
		0b011	clk/64
		0b100	clk/256
		0b101	clk/1024
		0b110	External clock on T0 pin, falling edge
		0b111	External clock on T0 pin, rising edge

	The PWM frequency is clk/(N*510) where N is the prescale factor. If
	our nominal CPU clock frequency is 14.745600 MHz, the frequency and
	period for the prescaled timer are are:

		======================================
		CS0[2:0]	Frequency		Period
		--------------------------------------
		0b001		28.913 kHz		 34.6 us
		0b010		 3.614 kHz		276.7 us
		0b011	   452.765 Hz		  2.214 ms
		0b100	   112.941 Hz		  8.854 ms
		0b101	    28.235 Hz		 35.417 ms
		======================================

	The Faulhaber 2622 B-SC motor takes a PWM frequency input in the range
	of 500 Hz to 18 kHz. Let's choose 3.614 kHz. The motor stops turning at
	duty cycle < 2.0% (5/255) and starts turning at duty cycle > 3.0% (8/255).

	For phase-correct PWM, WGM0[2:0] (Wavform Generation Mode)
	should be 0b001 if we're using OC0A (setting WGM0[2] to "1" means that
	OC0A becomes TOP and OC0B is the compared register). WGM0[1:0] are in
	TCCR0A.

	The Force Output Compare (FOC0A and FOC0B bits) must be set to zero (0)
	when using the timer in PWM mode.

	TCCR0B = 0b00000010

===========================================================================

	MCUSR MCU Status Register (what caused the MCU Reset?)
	---------------------------------------------------------------------
	Bit	|	7	|	6	|	5	|	4	|	3	|	2	|	1	|	0	|
	Item|   -	|   -	|	-	|	-	| WDRF	|  BORF	| EXTRF	|  PORF	|
	Set	|	0	|	0	|	0	|	0	|	0	|	0	|	1	|	0	|
	---------------------------------------------------------------------

	WDRF: Watchdog Reset Flag is set if a WD Reset occurs. Reset by power
	on or writing zero to the flag.
	
	BORF: Brown-out Reset flag is set if a brown-out Reset happens.
	
	EXTRF: External Reset Flag
	
	PORF: Power-on Reset Flag

	NB: THe WDRF *MUST* be reset or the system will immediately reset and
	loop forever.

===========================================================================

	WDTCR Watchdog Timer Control Register (aka WDTCSR on ATtiny2313)
	128kHz oscillator
	---------------------------------------------------------------------
	Bit	|	7	|	6	|	5	|	4	|	3	|	2	|	1	|	0	|
	Item| WDIF	| WDIE	| WDP3	| WDCE	|  WDE	| WDP2	| WDP1	|  WDP0	|
	Set	|	0	|	0	|	0	|	0	|	0	|	0	|	1	|	0	|
	---------------------------------------------------------------------

	WDIF: Watchdog Timeout Interrupt Flag is set when a timeout occurs
	and the WDT is configured for interrupt.
	WDIE: Watchdog Timeout Interrupt Enable.
	WDP3: One of three prescaler bits (see below)
	WDCE: Watchdog Change Enable (must be set to change WDE)
	WDE: Watchdog enable
	WDP[3:0]: Timer prescaler bits

		=========================================
		WDP3	WDP2	WDP1	WDP0	Appx time
		-----------------------------------------
		 0		 0		 0		 0		16 ms
		 0		 0		 0		 1		32 ms
		 0		 0		 1		 0		64 ms
		 0		 0		 1		 1		0.125 s
		 0		 1		 0		 0		0.25 s
		 0		 1		 0		 1		0.5 s
		 0		 1		 1		 0		1.0 s
		 0		 1		 1		 1		2.0 s
		 1		 0		 0		 0		4.0 s
		 1		 0		 0		 1		8.0 s
		=========================================

	Recovering from a WDT reset is a little tricky. The steps:
		1. Clear the MCUSR watchdog reset flag (WDRF).
		2. In one operation, write logic one to both WDCE and WDE.
		3. In the very next line, write a logic 0 to WDE.


===========================================================================

*/

#define BAUDRATE 19200
#define MYUBRR ((F_CPU / 16 / BAUDRATE) -1)

void init_ATtiny4313(void)
{

	uint8_t i, reset = 0;

	if (MCUSR & _BV(WDRF)) {	// If there was a watchdog reset...
		MCUSR = 0x00;			// Clear the WDRF flag (and everything else)
		WDTCR = 0x18;			// Both WDCE and WDE
		WDTCR = 0x00;			// Now clear WDE
		reset = 1;
	}

	UBRRH = (uint8_t) (MYUBRR >> 8);	// Set baud rate
	UBRRL = (uint8_t) MYUBRR;
	UCSRB = 0b00011000;			// Enable transmit & receive
	UCSRC = 0b00000110;			// 8 data bits, no parity, one stop bit

	TCCR0A = 0b10000001;		// Phase-correct PWM on OC0A (see above)
	TCCR0B = 0b00000010;		// Select clock prescaler /8 (see above)

	DDRB  |= _BV(LEDPIN);		// LEDPIN is an output

	while (COMMANDSENT) {		// Clear the serial port
		i = UDR;
	}

	for (i = 0; i < 3; i++) {
		brightness(255);
		_delay_ms(50);
		brightness(0);
		_delay_ms(200);
	}

	sendCRLF();
	if (reset) {
		sendString("Reset");
	} else {
		sendString("PwrUP");
	}

}
