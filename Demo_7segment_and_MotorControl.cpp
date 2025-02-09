/*
 * SPI_Main.cpp
 *
 * Created: 3/25/2018 10:06:32 AM
 * Author : Zhengyang Xiao
 
   This program sets up SPI in Main mode, mode 0, with SCK frequency of Fosc/16.
   Then the letter 'R' is transmitted.
   Whatever data that is received from Secondary is displayed on PortD
 ;
 
 */ 

#include <avr/io.h>

int send_to_MAX7221(unsigned char command, unsigned char data);
void wait(volatile int N);
void delayNms_timer0();
int pattern[6] = {0b01000000,0b00100000,0b00010000,0b00001000,0b00000100,0b00000010};
void complete();


int main(void) {
	// Set up ADC
	DDRC =  0b00111111;  // define all Port C bits as input
	PRR = 0x00;  // clear Power Reduction ADC bit (0) in PRR register
	ADCSRA = 0b10000111; //1<<ADEN | 1<<ADPS2 | 1<<ADPS1 | 1<<ADPS0;  // 0x87 // 0b10000111 // Set ADC Enable bit (7) in ADCSRA register, and set ADC prescaler to 128 (bits 2-0 of ADCSRA = ADPS2-ADPS0 = 111)
	ADMUX = 0b01100101; //0<<REFS1 | 1<<REFS0 | 1<<ADLAR; //0x60; // 0b01100000  // select Analog Reference voltage to be AVcc (bits 7-6 of ADMUX = 01),
	//left justification (bit 5 of ADMUX = ADLAR = 1);
	//and select channel 0 (bits 3-0 of ADMUX = MUX3-MUX0 = 000)
	
	DDRD = 0b11111000;// Make OC0A (PD6) and OC0B (PD5) output bits -- these are the PWM pins;
	OCR0A = 0x00;       // Load $00 into OCR0 to set initial duty cycle to 0 (motor off)
	TCCR0A = 0b10000011; //1<<COM0A1 | 0<<COM0A0 | 1<<WGM01 | 1<<WGM00;      // Set non-inverting mode on OC0A pin (COMA1:0 = 10; Fast PWM (WGM1:0 bits = bits 1:0 = 11) (Note that we are not affecting OC0B because COMB0:1 bits stay at default = 00)
	TCCR0B = 0b00000011; //0<<CS02 | 1<<CS01 | 1<<CS00; // Set base PWM frequency (CS02:0 - bits 2-0 = 011 for prescaler of 64, for approximately 1kHz base frequency)
	// PWM is now running on selected pin at selected base frequency.  Duty cycle is set by loading/changing value in OCR0A register.
	// Set up ADC
	PORTC = 0x00;
	
	
	PORTD = 0x00;
	DDRB = 0b00101100; //	DDRB = 1<<PORTB5 | 1<<PORTB3 | 1<<PORTB2;  // Set pins SCK, MOSI, and SS as output

	// Set up Main SPI
	SPCR = 0b01010001; // SPCR = 1<<SPE | 1<<MSTR | 1<<SPR0; // (SPIE = 0, SPE = 1, DORD = 0, MSTR = 1, CPOL = 0, CPHA = 0, SPR1 = 0, SPR0 = 1)
		// enable the SPI, set to Main mode 0, SCK = Fosc/16, lead with MSB
		
	// Transmit the data

	
	send_to_MAX7221(0b00001010,0x0A);
	send_to_MAX7221(0b00001011,0b00000010);
	while (1)
	{
		if (!(PIND & 0b00000001))
		{
			send_to_MAX7221(0b00001001,0b00001111);
			send_to_MAX7221(0b00001100,0x01);
			send_to_MAX7221(0b00000010,0b00000001);
			PORTC = PORTC | 0b00100000;
			for (int i=0;i<0xF0;i++)
			{
				OCR0A = i;
				wait(10);
			}
			wait(1000);
			for (int i=0xF0;i>0x00;i--)
			{
				OCR0A = i;
				wait(10);
			}
			wait(3000);
			PORTC = PORTC & 0b11011111;
			PORTC = PORTC | 0b00010000;
			for (int i=0;i<0xF0;i++)
			{
				OCR0A = i;
				wait(10);
			}
			wait(1000);
			for (int i=0xF0;i>0x00;i--)
			{
				OCR0A = i;
				wait(10);
			}
			complete();
		}
		if (!(PIND & 0b00000010))
		{
			send_to_MAX7221(0b00001001,0b00001111);
			send_to_MAX7221(0b00001100,0x01);
			send_to_MAX7221(0b00000010,0b00000010);
			wait(3000);
			complete();
		}
		if (!(PIND & 0b00000100))
		{
			send_to_MAX7221(0b00001001,0b00001111);
			send_to_MAX7221(0b00001100,0x01);
			send_to_MAX7221(0b00000010,0b00000011);
			wait(3000);
			complete();
		}

		
	}
	while ((SPSR & 0b10000000) == 0); //while ((SPSR & (0x1<<SPIF)) == 0) {} // Check the SPIF bit and wait for it to be set => transmit complete
		
	PORTB |= 0b00000100; //PORTB |= 1 << PORTB2;  // disable Secondary (this clears SBIF flag)

}

// This is a function that will send two bytes of a command (command and data) to
// the MAX7221 chip by SPI serial communication

// The code assumes that SPI is already set up (for the MAX7221 - SPI Mode 0, MSB first)

void complete()
{
	send_to_MAX7221(0b00001001,0b00000000);
	for (int j=0;j<3;j++)
	{
		for (int i=0;i<6;i++)
		{
			send_to_MAX7221(0b00000010,pattern[i]);
			wait(50);
		}
	}
	send_to_MAX7221(0b00001100,0x00);
	PORTC = 0x00;
}


int send_to_MAX7221(unsigned char command, unsigned char data)
{
	PORTB = PORTB & 0b11111011; // Clear PB2, which is the SS bit, so that transmission can begin
	SPDR = command; // Send command
	while(!(SPSR & (1<<SPIF))); // Wait for transmission to finish

	SPDR = data; // Send data
	while(!(SPSR & (1<<SPIF))); // Wait for transmission to finish

	PORTB = PORTB | 0b00000100; // Return PB2 to 1, which is the SS bit, to end transmission
	return 0;
}

void wait(volatile int N)
{
	// This subroutine creates a delay equal to N ms
	while (N > 0)
	{
		TCCR1A = 0x00; // clears WGM00 and WGM01 (bits 0 and 1) to ensure Timer/Counter is in normal mode.
		TCNT1 = 0;  // preload load TIMER1 with 5 if counting to 255 (count must reach 65,535-5 = 250)
		TCCR1B =  0b00000011; //1<<CS01 | 1<<CS00;	TCCR0B = 0x03;  // Start TIMER0, Normal mode, 16 MHz crystal clock, prescaler = 64
		while (TCNT1 < 0xFA); // exits when count = 250
		TCCR1B = 0x00; // stop TIMER0
		N--;
	}
}