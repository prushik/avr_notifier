#define __AVR_ATmega328P__ 1
#define F_CPU 16000000UL

#define BAUD 9600

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

// same thing, but don't block
void uart_send_char(unsigned char c) {
	UDR0 = c;
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

char uart_read_char(unsigned char *a)
{
	if (UCSR0A & _BV(RXC0))
	{
		*a = UDR0;
		return 1;
	}
	else
	{
		return 0;
	}
}

//char uart_read(unsigned char *buf, unsigned char len)
//{
//	
//}

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
	SPCR |= ( (1<<SPE) | (1<<MSTR) ); // enable SPI as master
	//SPCR |= ( (1<<SPR1) | (1<<SPR0) ); // set prescaler bits
	SPCR &= ~( (1<<SPR1) | (1<<SPR0) ); // clear prescaler bits
	SPSR = 0; // clear SPI status reg
	SPDR = 0; // clear SPI data reg
//	SPSR |= (1<<SPI2X); // set prescaler bits
	//SPSR &= ~(1<<SPI2X); // clear prescaler bits
}
/*
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
}*/

char spi_transfer(char data)
{
	SPDR = data;					// Start the transmission
	while (!(SPSR & (1<<SPIF))) ;	// Wait the end of the transmission
	return SPDR;					// return the received byte, we don't need that
}

unsigned char buffer[6][3] = {0};

void display_buffer()
{
	int i;

	PORTB |= DECADE_RESET;
//	_delay_ms(2);
	PORTB &= (~DECADE_RESET);

	for (i = 0; i < 6; i++)
	{
		PORTB |= SHIFT_LATCH;
//		_delay_ms(100);
		spi_transfer(buffer[i][0]);
//		_delay_ms(100);
		spi_transfer(buffer[i][1]);
//		_delay_ms(100);
		spi_transfer(buffer[i][2]);
//		_delay_ms(100);
		PORTB &= (~SHIFT_LATCH);
//		_delay_us(50);
		PORTB |= DECADE_CLOCK;
		PORTB &= (~DECADE_CLOCK);

//		_delay_ms(4);//waiting a bit

//		PORTB |= SHIFT_LATCH;
//		_delay_ms(100);
//		spi_transfer(0x00);// clearing the data
//		spi_transfer(0x00);
//		spi_transfer(0x00);
//		_delay_ms(100);
//		PORTB &= (~SHIFT_LATCH);

		// next line
/*		if (uart_getchar() != ' ')
		{
			buffer[i][0] = 0x7e;
			buffer[i][1] = 0x7e;
			buffer[i][2] = 0x7e;
		}
		else
		{
			buffer[i][0] = 0x66;
			buffer[i][1] = 0x66;
			buffer[i][2] = 0x66;
		}*/

//		PORTB &= (~SHIFT_LATCH);

//		_delay_ms(10);//waiting a bit
//		_delay_ms(100);
//		PORTB |= DECADE_CLOCK;
//		_delay_ms(100);//waiting a bit
//		PORTB &= (~DECADE_CLOCK);
//		_delay_ms(100);

//		uart_write(".", 1);
	}

//	PORTB |= DECADE_RESET;
//	uart_write("\r\n", 2);
}

void scroll()
{
	int i;
	for (i=0;i<6;i++)
	{
		char carry = (buffer[i][0]&0x80)?1:0;
		buffer[i][0] = (buffer[i][0]<<1) | ((buffer[i][1]&0x80)?1:0);
		buffer[i][1] = (buffer[i][1]<<1) | ((buffer[i][2]&0x80)?1:0);
		buffer[i][2] = (buffer[i][2]<<1) | carry;
	}
}

void _buffer(int i, unsigned char a, unsigned char b, unsigned char c)
{
	buffer[i][0] = a;
	buffer[i][1] = b;
	buffer[i][2] = c;
}

#define STATE_OPEN 0
#define STATE_DATA 1
#define STATE_CMD 2
static char cur_state = 0;
static unsigned char last_byte = 0;
static unsigned char state_count = 0;
static char scrolling = 1;

void handle_input()
{
	unsigned char input_byte;
	if (uart_read_char(&input_byte))
	{
		switch (cur_state)
		{
			case STATE_OPEN:
				if (input_byte == 'I')
				{
					cur_state = STATE_DATA;
					state_count = 0;
				}
				if (input_byte == 'S')
				{
					scroll();
				}
				if (input_byte == 'B')
				{
					scrolling = 0;
				}
				if (input_byte == 'C')
				{
					scrolling = 1;
				}
				break;
			case STATE_DATA:
				if (state_count < 18)
				{
					buffer[state_count/3][state_count%3] = input_byte;
					state_count += 1;
				}
				else
				{
					cur_state = STATE_OPEN;
				}
				break;
			default:
				break;
		}
	}
}

int main()
{
//	unsigned int toggle = 0, i;
//	unsigned char data[8];

//	unsigned char out[] = "x: XXXX y: XXXX z: XXXX t: XXXX\n";

//	DDRB = 0x20; // built-in LED is output
	DDRB |= (DECADE_RESET | DECADE_CLOCK | SHIFT_CLOCK | SHIFT_DATA | SHIFT_LATCH); // PINS 8, 9, 10, 11, and 13 are all output

//	uart_init();
//	i2c_init();

	// Reset the decade counter
	PORTB |= DECADE_RESET;
	_delay_ms(10);
	PORTB &= ~DECADE_RESET;

	spi_init();

	_delay_ms(100);

	_buffer(0, 0x11, 0x11, 0x11);
	_buffer(1, 0x22, 0x22, 0x22);
	_buffer(2, 0x44, 0x44, 0x44);
	_buffer(3, 0x44, 0x44, 0x44);
	_buffer(4, 0x22, 0x22, 0x22);
	_buffer(5, 0x11, 0x11, 0x11);

	unsigned char count_1=0,count_2=0;

	while (1)
	{
		count_1++;

		display_buffer();
		if (!count_1 & scrolling) {count_2++;}
		if ((count_2>50) & scrolling) {count_2=1; scroll();}
//		control();
		handle_input();
	}
}
