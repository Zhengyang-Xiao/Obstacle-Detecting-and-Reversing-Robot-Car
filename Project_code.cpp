/*
 * SPI_Main.cpp
 *
 * Created: 3/25/2018 10:06:32 AM
 * Author : Zhengyang Xiao
 
   This program works for group 28 mechatronics project
 
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>

// declare functions
void delayNms_timer0();
int send_to_MAX7221(unsigned char command, unsigned char data);
void wait(int msec);
void wait_count(int msec);

void forward_accelerate();
void forward_slow_down();
void backward_accelerate();
void backward_slow_down();

void show1();
void show2();
void complete();
void showNA();
void NAblink();
void showBlank();

// declare variables
int pattern[6] = {0b01000000,0b00100000,0b00010000,0b00001000,0b00000100,0b00000010};
int i;
int count;



int main(void) {
	// Set up ADC 
	DDRC =  0b00111100;  // define all Port C bits as input
	PRR = 0x00;  // clear Power Reduction ADC bit (0) in PRR register
	ADCSRA = 0b10000111; //1<<ADEN | 1<<ADPS2 | 1<<ADPS1 | 1<<ADPS0;  // 0x87 // 0b10000111 // Set ADC Enable bit (7) in ADCSRA register, and set ADC prescaler to 128 (bits 2-0 of ADCSRA = ADPS2-ADPS0 = 111)
	ADMUX = 0b01100101; //0<<REFS1 | 1<<REFS0 | 1<<ADLAR; //0x60; // 0b01100000  // select Analog Reference voltage to be AVcc (bits 7-6 of ADMUX = 01),
	//left justification (bit 5 of ADMUX = ADLAR = 1);
	//and select channel 0 (bits 3-0 of ADMUX = MUX3-MUX0 = 000)
	
	// Set up Interrupts
	EICRA = 1<<ISC01 | 0<<ISC00 | 1<<ISC11 | 0<<ISC10;	// trigger INT0 and INT1 on falling edge (when the button is pressed)
	EIMSK = 1<<INT1 | 1<<INT0;	// enable INT1 and INT0
	sei();	// enable Global Interrupt
	
	// set up PWM
	DDRD = 0b11100000;// Make OC0A (PD6) and OC0B (PD5) output bits -- these are the PWM pins;
	OCR0A = 0x00;       // Load $00 into OCR0 to set initial duty cycle to 0 (motor off)
	TCCR0A = 0b10000011; //1<<COM0A1 | 0<<COM0A0 | 1<<WGM01 | 1<<WGM00;      // Set non-inverting mode on OC0A pin (COMA1:0 = 10; Fast PWM (WGM1:0 bits = bits 1:0 = 11) (Note that we are not affecting OC0B because COMB0:1 bits stay at default = 00)
	TCCR0B = 0b00000011; //0<<CS02 | 1<<CS01 | 1<<CS00; // Set base PWM frequency (CS02:0 - bits 2-0 = 011 for prescaler of 64, for approximately 1kHz base frequency)
	// PWM is now running on selected pin at selected base frequency.  Duty cycle is set by loading/changing value in OCR0A register.
	
	// Set up communication
	DDRB = 0b00101100; //	DDRB = 1<<PORTB5 | 1<<PORTB3 | 1<<PORTB2;  // Set pins SCK, MOSI, and SS as output
	// Set up Main SPI
	SPCR = 0b01010001; // SPCR = 1<<SPE | 1<<MSTR | 1<<SPR0; // (SPIE = 0, SPE = 1, DORD = 0, MSTR = 1, CPOL = 0, CPHA = 0, SPR1 = 0, SPR0 = 1)
	// enable the SPI, set to Main mode 0, SCK = Fosc/16, lead with MSB
	
	// initialization
	PORTC = 0x00; // initialize the motor settings, PC4 and PC5
	PORTD = 0b00001111; // Enable PD2 and PD3 pull-up resistors and switches
	
	while (1)
	{
		if (!(PIND & 0b00000001))
		{
			send_to_MAX7221(0b00001010,0x0A); // set intensity light
			send_to_MAX7221(0b00001011,0b00000010); // scan limit of 2
			send_to_MAX7221(0b00001100,0x01); // enable diplay
			show1();
			forward_accelerate();
			wait_count(2500);
			forward_slow_down();
			wait(2000);
			backward_accelerate();
			wait(2500);
			backward_slow_down();
			complete();
		}
		if (!(PIND & 0b00000010))
		{
			send_to_MAX7221(0b00001010,0x0A); // set intensity light
			send_to_MAX7221(0b00001011,0b00000010); // scan limit of 2
			send_to_MAX7221(0b00001100,0x01); // enable diplay
			show2();
			forward_accelerate();
			wait_count(5000);
			forward_slow_down();
			wait(2500);
			backward_accelerate();
			wait(5000);
			backward_slow_down();
			complete();
		}
	}
	while ((SPSR & 0b10000000) == 0); //while ((SPSR & (0x1<<SPIF)) == 0) {} // Check the SPIF bit and wait for it to be set => transmit complete
	PORTB |= 0b00000100; //PORTB |= 1 << PORTB2;  // disable Secondary (this clears SBIF flag)
}



//Interrupt Service Routine for INT0
ISR(INT0_vect) // This routine is entered if Switch 1 goes high (door 1 is opened)
{
	if (count > 0)
	{
		showNA();
		count = -count;
		forward_slow_down();
		wait(1000);
		backward_accelerate();
		while(count != 0)
		{
			count++;
			wait(1);
		}
		backward_slow_down();
		NAblink();
	}
}

// group functions
void forward_accelerate()
{
	PORTC = PORTC & 0b11101111;
	PORTC = PORTC | 0b00100000;
	for (i=0;i<150;i++)
	{
		OCR0A = i;
		wait(10);
	}
}

void forward_slow_down()
{
	PORTC = PORTC & 0b11101111;
	PORTC = PORTC | 0b00100000;
	for (i=150;i>0x00;i--)
	{
		OCR0A = i;
		wait(10);
	}
}

void backward_accelerate()
{
	PORTC = PORTC & 0b11011111;
	PORTC = PORTC | 0b00010000;
	for (i=0;i<150;i++)
	{
		OCR0A = i;
		wait(10);
	}
}

void backward_slow_down()
{
	PORTC = PORTC & 0b11011111;
	PORTC = PORTC | 0b00010000;
	for (i=150;i>0x00;i--)
	{
		OCR0A = i;
		wait(10);
	}
}
// the above functions controls the car's movement


void complete() // This function is to display a pattern indicating that the mission is complete
{
	send_to_MAX7221(0b00001001,0b00000000);
	for (int j=0;j<3;j++)
	{
		for (int i=0;i<6;i++)
		{
			send_to_MAX7221(0b00000010,pattern[i]);
			send_to_MAX7221(0b00000001,pattern[i]);
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

void wait_count(int msec) // this function counts time to record distance
{
	// This subroutine calls others to create a delay
	// function passes number of msec to delay
	while (msec > 0) {
		delayNms_timer0();
		msec = msec - 1;
		count++;
		
	}
} // end wait

void wait(int msec)
{
	// This subroutine calls others to create a delay
	// function passes number of msec to delay
	while (msec > 0) {
		delayNms_timer0();
		msec = msec - 1;
	}
} // end wait

void delayNms_timer0()
{
	// This subroutine creates a delay of N msec using TIMER0 with prescaler on clock, where, for a 16MHz clock:
	//		N = .0156 msec for no prescaler and count of 250 (preload counter with 5)
	//		N = 0.125 msec for prescaler set to 8 and count of 250 (preload counter with 5)
	//		N = 1 msec for prescaler set to 64 and count of 250 (preload counter with 5)
	//		N = 4 msec for prescaler set to 256 and count of 250 (preload counter with 5)
	//		N = 16 msec for prescaler set to 1,024 and count of 250 (preload counter with 5)

	TCCR1A = 0x00; // clears WGM00 and WGM01 (bits 0 and 1) to ensure Timer/Counter is in normal mode.
	TCNT1 = 0;  // preload load TIMER1 with 5 if counting to 255 (count must reach 65,535-5 = 250)
	// or preload with 0 and count to 250
	//TCCR0B = 0x01; // Start Timer0, Normal mode, crystal clock, no prescaler
	// TCCR0B = 0x02; // Start Timer0, Normal mode, crystal clock, prescaler = 8
	TCCR1B =  0x03;  // Start Timer0, Normal mode, crystal clock, prescaler = 64
	//TCCR0B = 0x04; // Start Timer0, Normal mode, crystal clock, prescaler = 256
	//TCCR0B = 0x05; // Start Timer0, Normal mode, crystal clock, prescaler = 1024

	while (TCNT1 < 0xfa); // exits when count = 250 (requires preload of 0 to make count = 250)

	TCCR1B = 0x00; // Stop TIMER1
	TIFR1 = 0x1<<TOV1;  // Clear TOV1 (note that this is an odd bit in that it
	//is cleared by writing a 1 to it)
}

void show1()
{
	send_to_MAX7221(0b00001001,0b00001111);
	send_to_MAX7221(0b00000001,0b00000000);
	send_to_MAX7221(0b00000010,0b00000001);
}

void show2()
{
	send_to_MAX7221(0b00001001,0b00001111);
	send_to_MAX7221(0b00000001,0b00000000);
	send_to_MAX7221(0b00000010,0b00000010);
}

void showNA()
{
	send_to_MAX7221(0b00001001,0b00000000);
	send_to_MAX7221(0b00000001,0b01110110);
	send_to_MAX7221(0b00000010,0b01110111);
}

void showBlank()
{
	send_to_MAX7221(0b00001001,0b00001111);
	send_to_MAX7221(0b00000001,0b00001111);
	send_to_MAX7221(0b00000010,0b00001111);
}

void NAblink()
{
	while(1)
	{
		showNA();
		wait(500);
		showBlank();
		wait(500);
	}
}