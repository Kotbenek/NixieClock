//4.096MHz crystal
#define F_CPU 4096000

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

//UART buffer
volatile unsigned char uart_pointer = 0;
volatile unsigned char uart_data[256];

//Time variables
volatile uint8_t hours = 23;
volatile uint8_t minutes = 59;
volatile uint8_t seconds = 50;
volatile uint16_t ms = 0;

//Timezone correction in respect to UTC
volatile int8_t utc_correction = 0;

//Timezone correction setting variables
volatile uint8_t utc_setting_flag = 0;
volatile uint16_t utc_setting_counter = 0;

//Slot machine variables
volatile uint8_t slot_flag = 0;
volatile uint8_t slot_counter = 0;
volatile uint8_t slot_number = 0;

//GPS module power on/off
#define GPS_ON PORTB &= ~(1<<PB0)
#define GPS_OFF PORTB |= (1<<PB0)

//Tubes driver board signals
#define STROBE_LATCH_CLOCK PORTD |= (1<<PD6); __asm__ __volatile__ ("nop"); PORTD &= ~(1<<PD6); __asm__ __volatile__ ("nop")
#define STROBE_SHIFT_CLOCK PORTD |= (1<<PD7); __asm__ __volatile__ ("nop"); PORTD &= ~(1<<PD7); __asm__ __volatile__ ("nop")
#define SERIAL_1 { PORTD |= (1<<PD4); __asm__ __volatile__ ("nop"); }
#define SERIAL_0 { PORTD &= ~(1<<PD4); __asm__ __volatile__ ("nop"); }


int main()
{
	//UART setup
	UCSRB |= (1<<RXCIE) | (1<<RXEN);
	//9600bps (1.29% error)
	UBRRL = 26;

	//Set PB0 as output - GPS module power on/off
	DDRB |= (1<<PB0);
	//Turn on GPS module after 1 second
	GPS_OFF;
	_delay_ms(1000);
	GPS_ON;

	//Set PD4, PD5, PD6 and PD7 as outputs - tubes driver board signals
	DDRD |= (1<<PD4) | (1<<PD5) | (1<<PD6) | (1<<PD7);

	//Enable pull-up for buttons (timezone setting)
	PORTC |= (1<<PC0) | (1<<PC1);
	
	//Timer0 prescaler 64
	TCCR0 |= (1<<CS01) | (1<<CS00);

	//Timer0 overflow interrupt enable
	TIMSK |= (1<<TOIE0); 

	//Timer1 fast PWM on pin OC1A, 8-bit, prescaler 8 (2kHz)
	TCCR1A |= (1<<COM1A1);
	TCCR1A |= (1<<WGM10);
	TCCR1B |= (1<<WGM12);
	TCCR1B |= (1<<CS11);

	//Turn on ADC, prescaler 32 (128kHz)
	ADCSRA = (1<<ADEN) | (1<<ADPS0) | (1<<ADPS2);
	//Reference voltage - internal 2.56V
	ADMUX = (1<<REFS1) | (1<<REFS0);
  	//Channel ADC0 (MUX 00000)
	//ADMUX &= 0b11100000; //Redundant - it is already 00000 on startup 	

	//Enable interrupts
	sei();

	//Main loop
	while(1)
	{		
		//Buttons handling
		if (!(PINC & (1<<PC0)))
		{
			//Timezone-		
			cli();
			utc_correction--;
			hours += 24;
			hours--;
			hours %= 24;
			utc_setting_flag = 1;
			utc_setting_counter = 0;
			sei();
			_delay_ms(500);
		}

		if (!(PINC & (1<<PC1)))
		{
			//Timezone+
			cli();
			utc_correction++;
			hours++;
			hours %= 24;
			utc_setting_flag = 1;
			utc_setting_counter = 0;
			sei();
			_delay_ms(500);
		}
	}
}

//Send time data to the tubes driver board
static inline void send_digits(uint8_t digit1, uint8_t digit2, uint8_t digit3, uint8_t digit4, uint8_t digit5, uint8_t digit6)
{
	//Digit1 - BCDA
	//Digit2 - BCDA
	//Digit3 - BCDA
	//Digit4 - BCDA
	//Digit5 - BCDA
	//Digit6 - CDAB

	//A mask = 0b00000001
	//B mask = 0b00000010
	//C mask = 0b00000100
	//D mask = 0b00001000

	//Digit1
	if (digit1 & 0b00000010) SERIAL_1
	else SERIAL_0
	STROBE_SHIFT_CLOCK;

	if (digit1 & 0b00000100) SERIAL_1
	else SERIAL_0
	STROBE_SHIFT_CLOCK;

	if (digit1 & 0b00001000) SERIAL_1
	else SERIAL_0
	STROBE_SHIFT_CLOCK;

	if (digit1 & 0b00000001) SERIAL_1
	else SERIAL_0
	STROBE_SHIFT_CLOCK;

	//Digit2
	if (digit2 & 0b00000010) SERIAL_1
	else SERIAL_0
	STROBE_SHIFT_CLOCK;

	if (digit2 & 0b00000100) SERIAL_1
	else SERIAL_0
	STROBE_SHIFT_CLOCK;

	if (digit2 & 0b00001000) SERIAL_1
	else SERIAL_0
	STROBE_SHIFT_CLOCK;

	if (digit2 & 0b00000001) SERIAL_1
	else SERIAL_0
	STROBE_SHIFT_CLOCK;

	//Digit3
	if (digit3 & 0b00000010) SERIAL_1
	else SERIAL_0
	STROBE_SHIFT_CLOCK;

	if (digit3 & 0b00000100) SERIAL_1
	else SERIAL_0
	STROBE_SHIFT_CLOCK;

	if (digit3 & 0b00001000) SERIAL_1
	else SERIAL_0
	STROBE_SHIFT_CLOCK;

	if (digit3 & 0b00000001) SERIAL_1
	else SERIAL_0
	STROBE_SHIFT_CLOCK;

	//Digit4
	if (digit4 & 0b00000010) SERIAL_1
	else SERIAL_0
	STROBE_SHIFT_CLOCK;

	if (digit4 & 0b00000100) SERIAL_1
	else SERIAL_0
	STROBE_SHIFT_CLOCK;

	if (digit4 & 0b00001000) SERIAL_1
	else SERIAL_0
	STROBE_SHIFT_CLOCK;

	if (digit4 & 0b00000001) SERIAL_1
	else SERIAL_0
	STROBE_SHIFT_CLOCK;

	//Digit5
	if (digit5 & 0b00000010) SERIAL_1
	else SERIAL_0
	STROBE_SHIFT_CLOCK;

	if (digit5 & 0b00000100) SERIAL_1
	else SERIAL_0
	STROBE_SHIFT_CLOCK;

	if (digit5 & 0b00001000) SERIAL_1
	else SERIAL_0
	STROBE_SHIFT_CLOCK;

	if (digit5 & 0b00000001) SERIAL_1
	else SERIAL_0
	STROBE_SHIFT_CLOCK;

	//Digit6
	if (digit6 & 0b00000100) SERIAL_1
	else SERIAL_0
	STROBE_SHIFT_CLOCK;

	if (digit6 & 0b00001000) SERIAL_1
	else SERIAL_0
	STROBE_SHIFT_CLOCK;

	if (digit6 & 0b00000001) SERIAL_1
	else SERIAL_0
	STROBE_SHIFT_CLOCK;

	if (digit6 & 0b00000010) SERIAL_1
	else SERIAL_0
	STROBE_SHIFT_CLOCK;

	//Additional wait
	__asm__ __volatile__ ("nop");

	//Latch data
	STROBE_LATCH_CLOCK;
}

//250Hz interrupt
ISR(TIMER0_OVF_vect)
{
	//Timekeeping
	ms += 4;
	if (ms == 1000)
	{
		ms = 0;
		seconds++;
		if (seconds == 60)
		{
			seconds = 0;
			minutes++;
			if (minutes == 60)
			{
				minutes = 0;
				hours++;

				//Activate slot machine each hour
				slot_flag = 1;
				slot_counter = 0;
				slot_number = 0;

				if (hours == 24)
				{
					hours = 0;		
					//Turn on GPS module each day
					GPS_ON;			
				}
			}
		}
		//Send time data to the tubes driver board if slot machine and timezone correction flags are not active
		if (!slot_flag && !utc_setting_flag)
		{
			send_digits(hours / 10, hours % 10, minutes / 10, minutes % 10, seconds / 10, seconds % 10);			
		}	
	}
	//If slot machine and timezone correction flags are not active
	if(!slot_flag && !utc_setting_flag)
	{
		//Set tube brightness
		//Measure voltage on the resistor-photoresistor voltage divider
		ADCSRA |= (1<<ADSC);
		//Wait for the measurement to complete
       	while(ADCSRA & (1<<ADSC));
		//Use the ADC conversion result to set PWM duty cycle
      	uint16_t adc_result = ADC;
		adc_result = adc_result >> 2;
		if (adc_result > 255) adc_result = 255;
		if (adc_result < 50) adc_result = 50;
		OCR1A = adc_result;
	}
	//If slot machine is active
	if (slot_flag)
	{			
		//PWM duty cycle = 100%
		OCR1A = 255;

		//If there is a digit to display
		if (slot_number < 10)
		{
			//4ms * 50 = 200ms per digit
			if (slot_counter < 50)
			{
				//Send the digit to the tubes driver board
				send_digits(slot_number, slot_number, slot_number, slot_number, slot_number, slot_number);
				//Increment the digit timer
				slot_counter++;
			}
			//Next digit
			else
			{
				slot_number++;
				slot_counter = 0;
			}
		}
		//Cycled through all digits, slot machine off
		else
		{
			slot_flag = 0;
			//Send time data to the tubes driver board
			send_digits(hours / 10, hours % 10, minutes / 10, minutes % 10, seconds / 10, seconds % 10);			
		}
	}	
	//Timezone correction menu
	if (utc_setting_flag)
	{
		//If timezone correction is negative, display 1___XX, where XX = timezone correction
		if (utc_correction < 0) send_digits(1, 15, 15, 15, (utc_correction * -1) / 10, (utc_correction * -1) % 10);
		//Else display 0___XX, where XX = timezone correction
		else send_digits(0, 15, 15, 15, utc_correction / 10, utc_correction % 10);

		//Increment the timezone correction menu counter
		utc_setting_counter++;

		//If 5 seconds passed
		if (utc_setting_counter == 1250)
		{
			//Turn off timezone correction menu
			utc_setting_flag = 0;
			utc_setting_counter = 0;
			//Send time data to the tubes driver board
			send_digits(hours / 10, hours % 10, minutes / 10, minutes % 10, seconds / 10, seconds % 10);
		}
	}
}

//Function checking the checksum of GPS module data frame
unsigned char is_checksum_ok()
{
	//Checksum variable
	unsigned char checksum = uart_data[0];
	//Buffer pointer
	unsigned char ptr = 1;

	//'*' - end of data
	while (uart_data[ptr] != '*')
	{
		//If there is no '*' after 127 characters, data frame is 100% corrupted
		//(NMEA frames are shorter than that)
		if (ptr > 127) return 0;

		checksum = checksum ^ uart_data[ptr];
		ptr++;
	}

	//Split the checksum into two 4-bit values
	unsigned char checksum_h = checksum;
	checksum_h = checksum_h >> 4;
	checksum = checksum & 0b00001111;

	//Get the checksum from the data frame
	unsigned char uart_h = uart_data[ptr + 1];
	unsigned char uart_l = uart_data[ptr + 2];

	if (uart_h >= 'A') uart_h -= 'A' - 10;
	else uart_h -= '0';

	if (uart_l >= 'A') uart_l -= 'A' - 10;
	else uart_l -= '0';

	//Check the checksum
	if (uart_h == checksum_h && uart_l == checksum) return 1;
	else return 0;
}

//UART RX interrupt
ISR(USART_RXC_vect)
{
	//Get the data
	unsigned char buffer;
	buffer = UDR;
	//If it's the beginning of the data frame
	if (buffer == '$')
	{
		//Set the buffer pointer to the beginning of the buffer
		uart_pointer = 0;

		//If the frame stored in buffer is the GPGGA one
		if (uart_data[0] == 'G' &&
			uart_data[1] == 'P' &&
			uart_data[2] == 'G' &&
			uart_data[3] == 'G' &&
			uart_data[4] == 'A')
		{
			//If the data frame contains actual data and the checksum is correct
			if ((uart_data[6]  != ',') && (is_checksum_ok()))
			{
				//Turn off GPS module
				GPS_OFF;
				//Disable interrupts
				cli();
				//Get hours from the data frame
				hours = (uart_data[6] - '0') * 10 + (uart_data[7] - '0');
				//Apply timezone correction
				hours += utc_correction + 24;
				hours %= 24;
				//Get minutes from the data frame
				minutes = (uart_data[8] - '0') * 10 + (uart_data[9] - '0');
				//Get seconds from the data frame
				seconds = (uart_data[10] - '0') * 10 + (uart_data[11] - '0');
				//Set milliseconds to zero
				ms = 0;
				//Clear the timekeeping timer counter
				TCNT0 = 0;
				//Enable interrupts
				sei();
				//Send time data to the tubes driver board
				send_digits(hours / 10, hours % 10, minutes / 10, minutes % 10, seconds / 10, seconds % 10);
				//Clear the buffer (set the first character to 0)
				uart_data[0] = 0;
			}	
		
		}
	}
	//If it's not the beginning of the data frame
	else 
	{
		//Save data to the buffer
		uart_data[uart_pointer] = buffer;	
		//Increment the buffer pointer
		uart_pointer++;
	}
}
