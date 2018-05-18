/*
 * AmplifierTest1.0.c
 *
 * Created: 4/24/2018 4:18:51 PM
 * Author : jbrown40
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h> // Routine for FLASH (program memory)
#include <util/delay.h>
#include <string.h>
#include <stdio.h>
 
#define F_CPU 8000000L // This should match the processor speed
#define BAUD_RATE 9600 // Baud rate: the rate at which information is transferred in a communication channel

// Variables and #define for the RX ring buffer.
#define RX_BUFFER_SIZE 64
unsigned char rx_buffer[RX_BUFFER_SIZE];
volatile unsigned char rx_buffer_head; //read position of buffer
volatile unsigned char rx_buffer_tail; //write position of buffer
 
int realNumbers[64];
int periodIndex; //will hold the index of zero cross point on sine wave

//CHANGE TO GUITAR FREQUENCIES
const float E2 = 82.41;
const float A2 = 110.00;
const float D3 = 146.80;
const float G3 = 196.00;
const float B3 = 246.940;
const float E4 = 329.60;

//all usart functions come from Anton Kruger's functions (see below)
unsigned char uart_buffer_empty(void);
void usart_prints(const char *ptr);
void usart_printf(const char *ptr);
void usart_init();
void usart_putc(const char c);
unsigned char usart_getc(void);
void metronome();
void readADC(int rn[], int periodIndex); //read analog to digital converter input
void findNote(float frequency); //table of frequency values crossed checked with calculated frequency
unsigned char command; //user start command

int main(void){
	int y = 0;
	while (y != 20) { //always running
		char str[25]; //holds the string which we want to print to SecureCRT
		int i,j; //counter
		sei(); // Enable interrupts
		io_init(); // configure I/O pins
		usart_init(); // Initialize the USART
		adc_init();	// Initialize ADC conversions (analog to digital converter, 10-bit mode)
		readADC(&realNumbers, periodIndex); //start ADC reading and frequency detection
		y++;
		_delay_ms(10000);
	}//end while

}

ISR(USART_RX_vect)
{
	// USART receive interrupt handler.
	// To do: check and warn if buffer overflows.
	char c = UDR0; //receive buffer UDRn
	rx_buffer[rx_buffer_head] = c; //first character of receive buffer
	if (rx_buffer_head == RX_BUFFER_SIZE - 1) //if buffer is at max
	rx_buffer_head = 0; //reset buffer
	else
	rx_buffer_head++; //count up in buffer
}

void io_init(void){
	DDRB = 0b11111111; //LED outputs
	DDRC = 0b00000000; //ADC set as input
	PORTB = 0b00000000; //turn off all LEDs, turn off pushbutton
}

void usart_init(void){
	// Configures the USART for serial 8N1 with
	// the Baud rate controlled by a #define.
	unsigned short s;
	// Set Baud rate, controlled with #define above.
	s = (double)F_CPU / (BAUD_RATE*16.0) - 1.0;
	UBRR0H = (s & 0xFF00); //UBRRnH and UBRRnL are baud rate registers
	UBRR0L = (s & 0x00FF); //for high and low bits of baud rate

	// Receive complete interrupt enable: RXCIE0
	// Receiver & Transmitter enable: RXEN0,TXEN0
	UCSR0B = (1<<RXCIE0)|(1<<RXEN0)|(1<<TXEN0); //USART control and status reg B

	// Along with UCSZ02 bit in UCSR0B, set 8 bits
	UCSR0C = (1<<UCSZ01)|(1<<UCSZ00);
	DDRD |= (1<< 1); // PD0 is output (Tx)
	DDRD &= ~(1<< 0); // PD1 is input (Rx)

	// Empty buffers
	rx_buffer_head = 0;
	rx_buffer_tail = 0;
	
}
void adc_init(void)
{
	// |= compound bitwise or --> 'set to 1'
	ADMUX |= (1 << REFS0); // Set ADC reference to AVCC (analog power supply cleaned)
	// configures ADC
	ADCSRB |= (0 << ADTS0) | (0 << ADTS1) | (0 << ADTS2);  //Free-Running Mode

	ADCSRA |= (1 << ADEN);  // Enable ADC (turn on)
	ADCSRA |= (1 << ADSC);  // Start A/D Conversions

}

void usart_printf(const char *ptr){
	// Send NULL-terminated data from FLASH.
	// Uses polling (and it blocks).

	char c;

	while(pgm_read_byte_near(ptr)) {
		c = pgm_read_byte_near(ptr++); //Read a byte from the program space with a 16-bit (near) address.
		usart_putc(c);
	}
}
void usart_putc(const char c){

	// Send "c" via the USART.  Uses poling
	// (and it blocks). Wait for UDRE0 to become
	// set (=1), which indicates the UDR0 is empty
	// and can accept the next character.

	while (!(UCSR0A & (1<<UDRE0)));
	UDR0 = c;
}

void USART_Transmit( unsigned char data ){
	/* Wait for empty transmit buffer */
	while ( !( UCSR0A & (1<<UDRE0)) )
	;

	UDR0 = data;
}
void usart_prints(const char *ptr){
	// Send NULL-terminated data from SRAM.
	// Uses polling (and it blocks).

	while(*ptr) {
		while (!( UCSR0A & (1<<UDRE0)))
		;
		UDR0 = *(ptr++);
	}
}

unsigned char usart_getc(void)
{
	// Get char from the receiver buffer.  This
	// function blocks until a character arrives.
	unsigned char c;
	// Wait for a character in the buffer.

	while (rx_buffer_tail == rx_buffer_head) //rx_buffer_head is the index of the location from which to write.
	;
	c = rx_buffer[rx_buffer_tail]; //rx_buffer_tail is the index of the location from which to read.
	if (rx_buffer_tail == RX_BUFFER_SIZE-1)
	rx_buffer_tail = 0;
	else
	rx_buffer_tail++; //move to next read location
	return c;
}

 

void readADC(int rn[], int p){
	float period = 0; //will hold period of wave (T)
	float frequency = 0; //will hold frequency of wave (f)
	//read from ADC for 3 seconds, taking 300 samples
	char str[25]; //holds the string which we want to print to SecureCRT
	//if(buttonIsPressed()){ //when the user pressed the button, they are ready to start taking readings
	for(int i = 0; i < 300; i++){ //take 300 samples of the sine wave
			ADCSRA |= (1<<ADSC); //trigger 
			while(ADCSRA & (1<<ADSC)){ //wait for ADC conversion to complete
			}
			rn[i] = ADC; //read in ADC value (0->1023 with 1023 =~ 5V)
			//sprintf(str, "ADC = %d\n\r", rn[i]);
			//usart_prints(str);
		_delay_ms(10);
	}
	//loop through array of values, find zero point, save index of 1-(zero point)...
	for (int k = 4; k < 300; k++){ //skip the first 4 values to allow for delay in tone
		if (rn[k] < 512){
			p = k-1; //save index value
			break;
		}
	}
	//}
	//calculate the frequency
	period = (p*2.0)*.001; //multiply period index by 2 to get full period
	sprintf(str, "period = %f\n\r", period);
	usart_prints(str);
	frequency = (1/period); //f=1/T
	sprintf(str, "frequency = %f\n\r", frequency);
	usart_prints(str);
	findNote(frequency); //match the frequency with note, if in range
} 

//all ranges created below give room for a reading that is slightly toward
//the head or tail end of a sine wave
void findNote(float frequency){
	if (frequency > 50.00 && frequency < 100.00){
			usart_prints("E2\n");
			if(frequency < 83){
				PORTB = 0b00000001; //flat
			}
			else if(frequency > 83){ //sharp
				PORTB = 0b00000010;
			}
			else{
				PORTB = 0b00000100; //in-tune
			}
			
	}
	else if (frequency > 100.00 && frequency < 130.00){
		usart_prints("A2\n");
		if(frequency < 110){
			PORTB = 0b00000001;
		}
		else if(frequency > 110){
			PORTB = 0b00000010;
		}
		else{
			PORTB = 0b00000100;
		}
	}
	else if (frequency > 130.00 && frequency < 180.00){
		usart_prints("D3\n");
		if(frequency < 145){
			PORTB = 0b00000001;
		}
		else if(frequency > 145){
			PORTB = 0b00000010;
		}
		else{
			PORTB = 0b00000100;
		}
	}
	else if (frequency > 180.00 && frequency < 250.00){
		usart_prints("G3\n");
		if(frequency < 195){
			PORTB = 0b00000001;
		}
		else if(frequency > 195){
			PORTB = 0b00000010;
		}
		else{
			PORTB = 0b00000100;
		}
	}
	else if (frequency > 250.00 && frequency < 300.00){
		usart_prints("B3\n");
		if(frequency < 245){
			PORTB = 0b00000001;
		}
		else if(frequency > 245){
			PORTB = 0b00000010;
		}
		else{
			PORTB = 0b00000100;
		}
	}
	else if (frequency > 300.00 && frequency < 500.00){
		usart_prints("E4\n");
		if(frequency < 328){
			PORTB = 0b00000001;
		}
		else if(frequency > 328){
			PORTB = 0b00000010;
		}
		else{
			PORTB = 0b00000100;
		}
	}
	else{ //if frequency not in range, shut off LED and alert user
		usart_prints("Frequency undetectable.\n");
		PORTB = 0b00000000;
	}

}
/****************************************************************************************
*    References:
*   (1) Author: Anton Kruger
*    Link: http://s-iihr64.iihr.uiowa.edu/MyWeb/Teaching/ECE3360_2016/Resources/SerialSample.c
*    Title: Sample code for serial communication
*	 Retrieved on: 4/2/2018
*
***************************************************************************************/
 