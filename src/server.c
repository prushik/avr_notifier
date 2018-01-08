#include <stdio.h>
#include <errno.h>
#include <fcntl.h> 
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <time.h>

// font - numbers 0-9
unsigned char d[10][6] = {
	{0x00,0x02,0x05,0x05,0x05,0x02}, // 0
	{0x00,0x06,0x02,0x02,0x02,0x07}, // 1
	{0x06,0x03,0x01,0x01,0x02,0x07}, // 2
	{0x06,0x01,0x01,0x02,0x01,0x06}, // 3
	{0x00,0x05,0x05,0x07,0x01,0x01}, // 4
	{0x07,0x04,0x06,0x01,0x05,0x06}, // 5
	{0x03,0x04,0x04,0x06,0x05,0x02}, // 6
	{0x00,0x07,0x01,0x01,0x02,0x02}, // 7
	{0x02,0x05,0x02,0x05,0x05,0x02}, // 8
	{0x03,0x05,0x03,0x01,0x01,0x01}  // 9
};

static unsigned char buffer[6][3] = {0};

int set_interface_attribs(int fd, int speed, int parity)
{
	struct termios tty;
	memset(&tty, 0, sizeof tty);
	if (tcgetattr(fd, &tty) != 0)
	{
//                error_message ("error %d from tcgetattr", errno);
		return -1;
	}

	cfsetospeed(&tty, speed);
	cfsetispeed(&tty, speed);

	tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
	// disable IGNBRK for mismatched speed tests; otherwise receive break
	// as \000 chars
	tty.c_iflag &= ~IGNBRK;         // disable break processing
	tty.c_lflag = 0;                // no signaling chars, no echo,
									// no canonical processing
	tty.c_oflag = 0;                // no remapping, no delays
	tty.c_cc[VMIN]  = 0;            // read doesn't block
	tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

	tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

	tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
									// enable reading
	tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
	tty.c_cflag |= parity;
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag &= ~CRTSCTS;

	if (tcsetattr(fd, TCSANOW, &tty) != 0)
	{
//                error_message ("error %d from tcsetattr", errno);
		return -1;
	}
	return 0;
}

void set_blocking(int fd, int should_block)
{
	struct termios tty;
	memset (&tty, 0, sizeof tty);
	if (tcgetattr(fd, &tty) != 0)
	{
//                error_message ("error %d from tggetattr", errno);
		return;
	}

	tty.c_cc[VMIN]  = should_block;
	tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

	tcsetattr(fd, TCSANOW, &tty);
//                error_message ("error %d setting term attributes", errno);
}

void buffer_digits(int n)
{
	int i,j;
	for (i=0; i<6; i++)
	{
		for (j=0; j<3; j++)
		{
			buffer[i][j] = 0;
		}
	}
	while (n)
	{
		int digit = n%10;
		n = n/10;

		for (i=0;i<6;i++)
		{
			// copied the scrolling code from the avr firmware
//			unsigned char carry = (buffer[i][0]>>4)&0x0f;
			buffer[i][0] = (buffer[i][0]<<4); 
			buffer[i][0] |= ((buffer[i][1]>>4)&0x0f);
			buffer[i][1] = (buffer[i][1]<<4);
			buffer[i][1] |= ((buffer[i][2]>>4)&0x0f);
			buffer[i][2] = (buffer[i][2]<<4);
//			buffer[i][2] |= carry;
		}

		for (i=0; i<6; i++)
		{
			buffer[i][2] |= d[digit][i];
		}
	}
}

void buffer_digits_l(int n)
{
	int i,j;
	for (i=0; i<6; i++)
	{
		for (j=0; j<3; j++)
		{
			buffer[i][j] = 0;
		}
	}
	while (n)
	{
		int digit = n%10;
		n = n/10;

		for (i=0;i<6;i++)
		{
			// copied the scrolling code from the avr firmware
//			unsigned char carry = (buffer[i][0]>>4)&0x0f;
			buffer[i][2] = (buffer[i][2]>>4) | (buffer[i][1]&0x0f)<<4;
			buffer[i][1] = (buffer[i][1]>>4) | (buffer[i][0]&0x0f)<<4;
			buffer[i][0] = (buffer[i][0]>>4);
//			buffer[i][2] |= carry;
		}

		for (i=0; i<6; i++)
		{
			buffer[i][0] |= (d[digit][i]<<4);
		}
	}
}

int main()
{
	char *portname = "/dev/ttyUSB0";

	int fd = open(portname, O_RDWR);
	if (fd < 0)
	{
//			error_message ("error %d opening %s: %s", errno, portname, strerror (errno));
			return 1;
	}

	set_interface_attribs(fd, B9600, 0);  // set speed to 9600 bps, 8n1 (no parity)
	set_blocking(fd, 1);                // set blocking

	buffer_digits_l(00001);

	int i,j;
	for (i=0; i<50; i++)
		usleep(99000);             //We need to allow the avr to boot up

	write (fd, "BI", 2);
	for (i=0;i<6;i++)
	{
		for (j=0;j<3;j++)
		{
			usleep(20000);             // Why is this needed? I guess my firmware sucks...
			write(fd, &buffer[i][j], 1);
		}
	}
	usleep(2000);
	write(fd, "B", 1);
	write(fd, "BBBB", 4); // WTF? why does it not get there the first time?

	for (i=0; i<50; i++)
		usleep(99000);             //We need to allow the avr to boot up

	while (1)
	{
		long unsigned int xtime = (time(0)-(3600*5)) % 86400 ;
		buffer_digits_l((xtime*10000) / 86400);

		write(fd, "BI", 2);
		for (i=0;i<6;i++)
		{
			for (j=0;j<3;j++)
			{
				usleep(20000);             // Why is this needed? I guess my firmware sucks...
				write(fd, &buffer[i][j], 1);
			}
		}

		for (i=0; i<50; i++)
			usleep(99000);
	}

	close(fd);
}
