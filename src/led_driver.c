#define __AVR_ATmega328P__ 1
#define F_CPU 16000000UL

#define BAUD 9600

#define MPU_ADDR 0x68
#define MPU_WAKEUP 0x6b
#define MPU_XOUT_H 0x3b
#define MPU_TEMP_H 0x41

// THESE ARE ALL IN PORTB
// pin 8
#define DECADE_RESET 0x01
// pin 9
#define DECADE_CLOCK 0x02
// pin 10
#define SHIFT_LATCH 0x04
// pin 11
#define SHIFT_DATA 0x08
// pin 13
#define SHIFT_CLOCK 0x20

#include <avr/io.h>
#include <util/delay.h>

#include <util/setbaud.h>

void uart_init() {
	UBRR0H = UBRRH_VALUE;
	UBRR0L = UBRRL_VALUE;

#if USE_2X
	UCSR0A |= _BV(U2X0);
#else
	UCSR0A &= ~(_BV(U2X0));
#endif

	UCSR0C = _BV(UCSZ01) | _BV(UCSZ00); /* 8-bit data */
	UCSR0B = _BV(RXEN0) | _BV(TXEN0);   /* Enable RX and TX */
}


void uart_putchar(unsigned char c) {
	loop_until_bit_is_set(UCSR0A, UDRE0); /* Wait until data register empty. */
	UDR0 = c;
	loop_until_bit_is_set(UCSR0A, TXC0); /* Wait until transmission ready. */
}

void uart_write(char *data, unsigned int len)
{
//	PORTB |= 0x20;
	int i;
	for (i=0; i < len; i++)
	{
		uart_putchar(data[i]);
//		_delay_ms(10);
	}
//	PORTB &= ~0x20;
}

char uart_getchar()
{
	loop_until_bit_is_set(UCSR0A, RXC0); /* Wait until data exists. */
	return UDR0;
}

void i2c_init()
{
	//set SCL to 400kHz
	TWSR = 0x00;
	TWBR = 0x11;

	//enable TWI
	TWCR = (1<<TWEN);
}

void i2c_start()
{
	TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
	while ((TWCR & (1<<TWINT)) == 0);
}

//send stop signal
void i2c_stop()
{
	while (TWCR & (1<<TWIE)) ;
	TWCR = (1<<TWINT)|(1<<TWSTO)|(1<<TWEN);
//	while (TWCR & (1<<TWSTO)) ;
}

void i2c_write(uint8_t data)
{
	TWDR = data;
	TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWEA);
	while ((TWCR & (1<<TWINT)) == 0);
	_delay_ms(12);
}

uint8_t i2c_read_ack()
{
	TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWEA);
	while ((TWCR & (1<<TWINT)) == 0);
	return TWDR;
}

//read byte with NACK
uint8_t i2c_read_nack()
{
	TWCR = (1<<TWINT)|(1<<TWEN);
	while ((TWCR & (1<<TWINT)) == 0);
	return TWDR;
}

uint8_t i2c_get_status()
{
	return TWSR & 0xF8;
}

uint8_t i2c_get_status_()
{
	return TWSR & 0xf8;
}

static unsigned char inttohex[16] = "0123456789abcdef";

void print_stat(unsigned char c)
{
	unsigned char out[3];
	out[0] = inttohex[c >> 4];
	out[1] = inttohex[c & 0x0f];
	out[2] = '\n';

	uart_write(out, 3);
}

void i2c_scan()
{
	char i;
	for (i = 0; i < 0x7f; i++)
	{
		i2c_start();
		i2c_write((i << 1) | 0);
//		_delay_ms(12);
		char ack = i2c_get_status();
		i2c_stop();

		// 0x18 means ack
		// 0x20 means nack
		if (ack == 0x18)
		{
			uart_write("Found i2c device at: ", 21);
			uart_putchar(inttohex[i>>4]);
			uart_putchar(inttohex[i&0x0f]);
			uart_putchar('\n');
		}
	}
}

void spi_init()
{
/*	SPCR |= ( (1<<SPE) | (1<<MSTR) ); // enable SPI as master
	//SPCR |= ( (1<<SPR1) | (1<<SPR0) ); // set prescaler bits
	SPCR &= ~( (1<<SPR1) | (1<<SPR0) ); // clear prescaler bits
	SPSR = 0; // clear SPI status reg
	SPDR = 0; // clear SPI data reg
//	SPSR |= (1<<SPI2X); // set prescaler bits
	//SPSR &= ~(1<<SPI2X); // clear prescaler bits*/
}

void spi_transfer(unsigned char data)
{
	int i;
	for (i=0; i<8; i++)
	{
		if (data & 1)
			PORTB |= SHIFT_DATA;
		else
			PORTB &= ~SHIFT_DATA;

		data = data>>1;

		PORTB |= SHIFT_CLOCK;
		PORTB &= ~SHIFT_CLOCK;
	}
}
/*
char spi_transfer(char data)
{
	SPDR = data;					// Start the transmission
	while (!(SPSR & (1<<SPIF))) ;	// Wait the end of the transmission
	return SPDR;					// return the received byte, we don't need that
}*/

static char buffer[6][3] = {0};

void display_buffer()
{
	int i;

	_delay_ms(100);
	PORTB &= (~DECADE_RESET);

	for (i = 0; i < 6; i++)
	{
		PORTB |= SHIFT_LATCH;
		_delay_ms(100);
		spi_transfer(buffer[i][0]);
		_delay_ms(100);
		spi_transfer(buffer[i][1]);
		_delay_ms(100);
		spi_transfer(buffer[i][2]);
		_delay_ms(100);
		PORTB &= (~SHIFT_LATCH);

		_delay_ms(400);//waiting a bit

/*		PORTB |= SHIFT_LATCH;
		_delay_ms(100);
		spi_transfer(0xff);// clearing the data
		spi_transfer(0xff);
		spi_transfer(0xff);
		_delay_ms(100);
		PORTB &= (~SHIFT_LATCH);*/

		// next line
/*		if (uart_getchar() == ' ')
		{
			buffer[i][0] = 0x00;
			buffer[i][1] = 0x00;
			buffer[i][2] = 0x00;
		}
		else
		{
			buffer[i][0] = 0xff;
			buffer[i][1] = 0xff;
			buffer[i][2] = 0xff;
		}*/

//		PORTB &= (~SHIFT_LATCH);

		PORTB &= (~DECADE_CLOCK);
		_delay_ms(100);//waiting a bit
		PORTB |= DECADE_CLOCK;
//		_delay_ms(100);
//		PORTB |= DECADE_CLOCK;
//		_delay_ms(100);//waiting a bit
//		PORTB &= (~DECADE_CLOCK);
//		_delay_ms(100);

		uart_write(".", 1);
	}

	PORTB |= DECADE_RESET;
//	uart_write("\r\n", 2);
}

/*
void display_word(int loops, byte word_print[][6], int num_patterns, int delay_langth)
{
	// this function displays your symbols
	i = 0;// resets the counter fot the 4017
	for (int g = 0; g < 6; g++)//resets the the long int where your word goes
		scrolling_word[g] = 0;
	for (int x = 0; x < num_patterns; x++)
	{ //main loop, goes over your symbols
		// you will need to find a better way to make the symbols scroll my way is limited for 24 columns

		for (int r = 0; r < 6; r++)//puts the buildes the first symbol
			scrolling_word[r] |= word_print[x][r]; 
		for (int z = 0; z < 6; z++)
		{ //the sctolling action
			for (int p = 0; p < 6; p++)
				scrolling_word[p] = scrolling_word[p] << 1;
			// end of the scrolling funcion
			for (int t = 0; t < delay_langth; t++)
			{ // delay function, it just loops over the same display
				for (int y = 0; y < 6; y++)
				{// scaning the display
					if (i == 6)
					{// counting up to 6 with the 4017
						PORTB = PORTB | DECADE_RESET;
						PORTB = PORTB & (~DECADE_RESET);
						i = 0;
					}
					PORTB = PORTB & (~SHIFT_LATCH);
					spi_transfer(make_word(0x01000000, y));// sending the data
					spi_transfer(make_word(0x00010000, y));
					spi_transfer(make_word(0x00000100, y));
					PORTB = PORTB | SHIFT_LATCH;
					_delay_ms(800);//waiting a bit
					PORTB = PORTB & (~SHIFT_LATCH);
					spi_transfer(0);// clearing the data
					spi_transfer(0);
					spi_transfer(0);
					PORTB = PORTB | SHIFT_LATCH;
					
					PORTB = PORTB | DECADE_CLOCK;
					PORTB = PORTB & (~DECADE_CLOCK);
					i++;
				}
			}
		}
	}
	finish_scroll(delay_langth);
}*/

int main()
{
	unsigned int toggle = 0, i;
	unsigned char data[8];

	unsigned char out[] = "x: XXXX y: XXXX z: XXXX t: XXXX\n";

//	DDRB = 0x20; // built-in LED is output
	DDRB |= (DECADE_RESET | DECADE_CLOCK | SHIFT_CLOCK | SHIFT_DATA | SHIFT_LATCH); // PINS 8, 9, 10, 11, and 13 are all output

//	uart_init();
//	i2c_init();

	// Reset the decade counter
	PORTB |= DECADE_RESET;
	_delay_ms(10);
	PORTB &= ~DECADE_RESET;

	spi_init();

	//turn on i2c pullup resistors
	//no need, external resistors present
//	PORTC = 0x30;
//	PORTC = 0x00;

	_delay_ms(100);

	for (i = 0; i < 6; i++)
	{
		buffer[i][0] = 0xff;
		buffer[i][1] = 0xff;
		buffer[i][2] = 0xff;
	}

//	i2c_scan();

//	for (i=0; i<0xff; i++)
//		print_stat(i);

	while (1)
	{
//		PORTB = toggle;
//		for (i=0;i<250;i++)
//		_delay_ms(50);
//		if (PIND & 0x08)
/*
			out[2] = (data[0] & 0x80) ? '-' : '+';
			out[3] = inttohex[data[0]>>4 & 0x0f];
			out[4] = inttohex[data[0]>>0 & 0x0f];
			out[5] = inttohex[data[1]>>4 & 0x0f];
			out[6] = inttohex[data[1]>>0 & 0x0f];

			out[10] = (data[2] & 0x80) ? '-' : '+';
			out[11] = inttohex[data[2]>>4 & 0x0f];
			out[12] = inttohex[data[2]>>0 & 0x0f];
			out[13] = inttohex[data[3]>>4 & 0x0f];
			out[14] = inttohex[data[3]>>0 & 0x0f];

			out[18] = (data[4] & 0x80) ? '-' : '+';
			out[19] = inttohex[data[4]>>4 & 0x0f];
			out[20] = inttohex[data[4]>>0 & 0x0f];
			out[21] = inttohex[data[5]>>4 & 0x0f];
			out[22] = inttohex[data[5]>>0 & 0x0f];
*/
			display_buffer();

//			uart_write(out, 32);
	}
}
