#include <errno.h>
#include <fcntl.h> 
#include <string.h>
#include <termios.h>
#include <unistd.h>

unsigned char d[10][6] = {
	{0x00,0x06,0x09,0x09,0x09,0x06},
	{0x00,0x06,0x02,0x02,0x02,0x07},
	{0x06,0x03,0x01,0x01,0x02,0x07},
	{0x06,0x01,0x01,0x02,0x01,0x06},
	{0x00,0x05,0x05,0x07,0x01,0x01},
	{0x07,0x04,0x07,0x02,0x05,0x06},
	{0x03,0x04,0x04,0x06,0x05,0x02},
	{0x00,0x07,0x01,0x01,0x02,0x02},
	{0x02,0x05,0x02,0x05,0x05,0x02},
	{0x03,0x05,0x03,0x01,0x01,0x01}
};

static unsigned char buffer[6][3] = {0};

int set_interface_attribs (int fd, int speed, int parity)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
//                error_message ("error %d from tcgetattr", errno);
                return -1;
        }

        cfsetospeed (&tty, speed);
        cfsetispeed (&tty, speed);

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

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
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
        if (tcgetattr (fd, &tty) != 0)
        {
//                error_message ("error %d from tggetattr", errno);
                return;
        }

        tty.c_cc[VMIN]  = should_block;
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

      tcsetattr (fd, TCSANOW, &tty);
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
			unsigned char carry = (buffer[i][2]>>4)&0x0f;
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

int main()
{
	char *portname = "/dev/ttyUSB0";

	int fd = open(portname, O_RDWR);
	if (fd < 0)
	{
//			error_message ("error %d opening %s: %s", errno, portname, strerror (errno));
			return 1;
	}

	set_interface_attribs (fd, B57600, 0);  // set speed to 115,200 bps, 8n1 (no parity)
//	set_blocking (fd, 1);                // set blocking

	buffer_digits(9614);

	int i,j;
	for (i=0; i<50; i++)
		usleep (99000);             //We need to allow the avr to boot up

	write (fd, "BI", 2);
	for (i=0;i<6;i++)
	{
		for (j=0;j<3;j++)
		{
			usleep (200000);             // sleep enough to transmit the 7 plus
			write (fd, &buffer[i][j], 1);
		}
	}
	usleep (20000);             // sleep enough to transmit the 7 plus
//	write (fd, "C", 1);
	write (fd, "CCCC", 4);

	usleep (99000);             // sleep enough to transmit the 7 plus
	usleep (99000);             // sleep enough to transmit the 7 plus
	usleep (99000);             // sleep enough to transmit the 7 plus
	usleep (99000);             // sleep enough to transmit the 7 plus
	usleep (99000);             // sleep enough to transmit the 7 plus
	usleep (99000);             // sleep enough to transmit the 7 plus
	usleep (99000);             // sleep enough to transmit the 7 plus
	usleep (99000);             // sleep enough to transmit the 7 plus
	usleep (99000);             // sleep enough to transmit the 7 plus
	usleep (99000);             // sleep enough to transmit the 7 plus
	usleep (99000);             // sleep enough to transmit the 7 plus
										 // receive 25:  approx 100 uS per char transmit
//	char buf [100];
//	int n = read (fd, buf, sizeof buf);  // read up to 100 characters if ready to read
	close(fd);
}
