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

char spi_transfer(char data)
{
	SPDR = data;					// Start the transmission
	while (!(SPSR & (1<<SPIF))) ;	// Wait the end of the transmission
	return SPDR;					// return the received byte, we don't need that
}

int main()
{
	DDRB |= (DECADE_RESET | DECADE_CLOCK | SHIFT_CLOCK | SHIFT_DATA | SHIFT_LATCH); // PINS 8, 9, 10, 11, and 13 are all output

	uart_init();

	while (1)
	{
		char uc = uart_getchar();
		uart_putchar(uc);
		uart_putchar('\r');
		uart_putchar('\n');
	}
}
