#include "sonar.h"
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

static char m_buffer[16];
static int m_fd;
static bool m_initialized;

bool sonar_init(char *p_path)
{
	m_fd = open(p_path, (O_RDWR | O_NOCTTY | O_NDELAY));

	if (m_fd == -1) {
		m_initialized = false;
		return false;
	}

	fcntl(m_fd, F_SETFL, 0);

	struct termios options;
	tcgetattr(m_fd, &options);

	/* Inspired by https://github.com/mavlink/c_uart_interface_example/blob/master/serial_port.cpp: */
	options.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK |
			     ISTRIP | IXON);
	options.c_oflag &= ~(OCRNL | ONLCR | ONLRET | ONOCR | OFILL | OPOST);
	options.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
	options.c_cflag &= ~(CSIZE | PARENB);
	options.c_cflag |= CS8;

	options.c_cc[VMIN] = 1; /* minimum bytes to return from read() */
	options.c_cc[VTIME] = 10; /* inter-character timer */

	cfsetispeed(&options, B9600);
	cfsetospeed(&options, B9600);

	tcsetattr(m_fd, TCSANOW, &options);

	m_initialized = true;

	return true;
}

bool sonar_read(int *p_distance_mm)
{
	if (!m_initialized)
		return false;

	static char numbuff[16];
	static int numlen;
	static bool save_num;

	int nread = read(m_fd, m_buffer, sizeof(m_buffer));
	for (int i = 0; i < nread; i++) {
		switch (m_buffer[i]) {
		case 'R':
			save_num = true;
			numlen = 0;
			break;
		case 13:
			if (save_num && numlen > 0) {
				*p_distance_mm = atoi(numbuff);
				numlen = 0;
				return true;
			}
			break;
		default:
			if (save_num) {
				numbuff[numlen] = m_buffer[i];
				numlen++;

				if (numlen == 16)
					save_num = false;
				else
					numbuff[numlen] = '\0';
			}
			break;
		}
	}

	return false;
}

bool sonar_close(void)
{
	if (m_initialized && close(m_fd) == 0) {
		m_initialized = false;
		return true;
	}

	return false;
}
