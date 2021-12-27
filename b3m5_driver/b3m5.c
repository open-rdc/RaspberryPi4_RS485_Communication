/**
 * Kondo B3M 0.1 Library
 *
 * Copyright 2016 - Yasuo Hayashibara (yasuo@hayashibara.net)
 * Chiba Institute of Technology
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <linux/serial.h>
#include <wiringPi.h>

#include "b3m5.h"

int target_deg100[256];

#define BAUDRATE 1500000

// Convenience macros
#define b3m_error(ki, err) { \
  snprintf(ki->error, 128, "ERROR: %s: %s\n", __func__, err); \
  fprintf(stderr,"%s\n", ki->error); \
  return -1; }

/*!
 * @brief Iniitialize the B3M interface
 * 1500000 baud, 8 bits, even parity, 1 stop bit
 *
 * @return 0 if successful, < 0 otherwise
 */
int b3m_init(B3MData * r, const char* serial_port, int switch_txrx_port)
{
	int i;
	printf("b3m_init\n");
	assert(r);
	r->debug = 0;

	struct termios tio;

	r->fd = open(serial_port, O_RDWR | O_NOCTTY);
	if (ioctl(r->fd, TCGETS, &tio)){
		b3m_error(r, "Get serial port parameters");
	}

	tio.c_cflag &= ~CBAUD;          // clear mask for setting baud rate
	tio.c_cflag &= ~PARENB;         // set no parity
	tio.c_cflag &= ~CSTOPB;         // 1 stop bit
	tio.c_cflag &= ~CSIZE;          // clear mask for setting the data size
	tio.c_cflag |= B3M_BAUD;        // set B3M baud
	tio.c_cflag |= CS8;             // character size 8 bit
	tio.c_cflag |= CREAD;           // enable receiver
	tio.c_cflag |= CLOCAL;          // ignore modem status line
	tio.c_iflag = IGNBRK | IGNPAR;  // ignore break condition and characer with parity error
	tio.c_oflag = 0;                // raw mode
	tio.c_lflag = 0;                // noncanonical input
	tio.c_cc[VMIN] = 0;             // 0 return all else until n byte received
	tio.c_cc[VTIME] = 1;            // 0 block forever else until n tenth second
	tcflush(r->fd, TCIOFLUSH);      // flush current port setting

	if (ioctl(r->fd, TCSETS, &tio)){
		b3m_error(r, "Set serial port parameters");
	}

	struct serial_struct sstruct;

	if (ioctl(r->fd, TIOCGSERIAL, &sstruct) < 0) {
		printf("Error: could not get comm ioctl\n");
		exit(-1);
	}

	sstruct.custom_divisor = sstruct.baud_base / BAUDRATE;
	sstruct.flags |= ASYNC_SPD_CUST;

	if (ioctl(r->fd, TIOCSSERIAL, &sstruct) < 0) {
		printf("Error: could not set custom comm baud divisor\n");
		exit(-1);
	}
	for(i = 0; i < 256; i ++) target_deg100[i] = 100000;
	r->switch_txrx_port = switch_txrx_port;
	pinMode(r->switch_txrx_port, OUTPUT);
	digitalWrite(r->switch_txrx_port, LOW);

	return 0;
}

/*!
 * @brief Close / Deinitialize the B3M Interface
 * 
 * @return 0 if successful, < 0 otherwise
 */
int b3m_close(B3MData * r)
{
	printf("b3m_close\n");
	assert(r);

	close(r->fd);

	return 0;
}

/*!
 * @brief Write n bytes from the swap to the Kondo
 * 
 * @return >0 number of bytes written, < 0 if error
 */
int b3m_write(B3MData * r, int n)
{
	assert(r);

	int i;
	if ((i = write(r->fd, r->swap, n)) < 0)
		b3m_error(r, "Send data");

	return i;
}


/*!
 * @brief Read n bytes from the B3M, waiting for at most usecs for the n bytes
 * Performs this by continuously polling the serial buffer until either
 * all of the bytes are read or the timeout has been reached.
 *
 * @return < 0: error, >= 0: number of bytes read
 */
int b3m_read_timeout(B3MData * r, int n, long usecs)
{
	assert(r);

	static struct timeval tv, end;
	int i = 0, bytes_read = 0;
	gettimeofday(&tv, NULL);

	// determine end time
	end.tv_sec = tv.tv_sec + usecs / 1000000;
	end.tv_usec = tv.tv_usec + usecs % 1000000;
	if (end.tv_usec > 1000000) {
		end.tv_sec += 1;
		end.tv_usec -= 1000000;
	}
	// spam the read until data arrives
	do {
	    if ((i = read(r->fd, &(r->swap[bytes_read]), n - bytes_read)) < 0) {
			b3m_error(r, "Read data");
	    }
	    bytes_read += i;
	    gettimeofday(&tv, NULL);
	    if (bytes_read > 0) {
			if (r->swap[0] != n) break;
	    }
	} while (bytes_read < n && (tv.tv_sec < end.tv_sec || (tv.tv_sec == end.tv_sec && tv.tv_usec < end.tv_usec)));
	return bytes_read;
}

/*!
 * @brief Purge the serial buffers
 *
 * @return 0 if successful, < 0 if error
 */
int b3m_purge(B3MData * r)
{
	assert(r);
	tcflush(r->fd, TCIOFLUSH);

	return 0;
}


/*!
 * @brief Transaction template: Purge, then send out_bytes, then receive in_bytes
 * On RX, blocks for at most timeout microseconds before giving up.
 *
 * @return < 0: error, >= 0: number of bytes read
 */
int b3m_trx_timeout(B3MData * r, UINT bytes_out, UINT bytes_in, long timeout)
{
	assert(r);
	int i, j;

	if ((i = b3m_purge(r)) < 0)
		return i;

	digitalWrite(r->switch_txrx_port, HIGH);
	if ((i = b3m_write(r, bytes_out)) < 0) {
		digitalWrite(r->switch_txrx_port, LOW);
		return i;
	}

	if (bytes_out == 7) delayMicroseconds(70);
	else delayMicroseconds(90);
	digitalWrite(r->switch_txrx_port, LOW);

	// debug printing
	if (r->debug) {
		printf("send %d bytes: ", bytes_out);
		for (j = 0; j < bytes_out; j++)
			printf("%x ", r->swap[j]);
		printf("\n");
	}

	// clear swap
	for (i = 0; i < bytes_in; i++)
		r->swap[i] = 0;

	// read the return data
	i = b3m_read_timeout(r, bytes_in, timeout);

	// debug printing
	if (r->debug) {
		printf("recv %d bytes: ", i);
		for (j = 0; j < i; j++)
			printf("%x ", r->swap[j]);
		printf("\n");
	}

	return i;
}


/*!
 * @brief send command to servo motors
 *
 * @param[in] id servo id, 0-255 (255: broadcast)
 * @param[in] address servo address
 * @param[in] data servo data
 * @param[in] byte byte of data
 * @return error status
 */
int b3m_com_send(B3MData * r, UINT id, UINT address, UCHAR *data, int byte)
{
	assert(r);

	int i, n = 0;
	int sum = 0, time = 0;

	// build command
	r->swap[n++] = 7 + byte;				// length
	r->swap[n++] = B3M_CMD_WRITE;			// command
	r->swap[n++] = B3M_RETURN_ERROR_STATUS;	// option
	r->swap[n++] = id;						// id
	for(i = 0; i < byte; i ++){
		r->swap[n++] = data[i];
	}
	r->swap[n++] = address;
	r->swap[n++] = 0x01;					// number of ID
	for(i = 0; i < n; i ++){
		sum += r->swap[i];
	}
	r->swap[n] = sum & 0xff;

	// synchronize with servo
	if ((i = b3m_trx_timeout(r, 7 + byte, 5, B3M_POS_TIMEOUT)) < 0)
		return i;

	// return error status
	return r->swap[2];
}


/*!
 * @brief get status from servo motors
 *
 * @param[in] id servo id, 0-255 (255: broadcast)
 * @param[in] address servo address
 * @param[out] data servo data
 * @param[in] byte byte of data
 * @return error status
 */
int b3m_get_status(B3MData * r, UINT id, UINT address, UCHAR *data, int byte)
{
	assert(r);

	int i, n = 0;
	int sum = 0, time = 0;

	// build command
	r->swap[n++] = 0x07;					// length
	r->swap[n++] = B3M_CMD_READ;			// command
	r->swap[n++] = B3M_RETURN_ERROR_STATUS;	// option
	r->swap[n++] = id;						// id
	r->swap[n++] = address;
	r->swap[n++] = byte;					// number of ID
	for(i = 0; i < n; i ++){
		sum += r->swap[i];
	}
	r->swap[n] = sum & 0xff;

	// synchronize with servo
	if ((i = b3m_trx_timeout(r, 7, 5 + byte, B3M_POS_TIMEOUT)) < 0){
		return i;
	}

	if (r->swap[0] != 5 + byte) return -1;
	for(i = 0; i < byte; i ++){
		data[i] = r->swap[i + 4];
	}

	// return error status
	return 0;
}


/*!
 * @brief set servo position
 *
 * @param[in] id the servo id, 0-255 (255: broadcast)
 * @param[in] pos the position to set (angle * 100)
 * @return error status.
 */
int b3m_set_angle(B3MData * r, UINT id, int deg100)
{
	assert(r);

	UCHAR data[2];
	data[0] = deg100 & 0xff;
	data[1] = deg100 >> 8;

	return b3m_com_send(r, id, B3M_SERVO_DESIRED_POSITION, data, 2);
}


/*!
 * @brief Get servo position
 *
 * @param[in] id the servo id, 0-255 (255: broadcast)
 * @param[out] deg100 the angle * 100
 * @return error status.
 */
int b3m_get_angle(B3MData * r, UINT id, int *deg100)
{
	assert(r);

	UCHAR data[2];
	if (b3m_get_status(r, id, B3M_SERVO_CURRENT_POSITION, data, 2)){
		return -1;
	}

	*deg100 = (int)((short)((data[1] << 8) + data[0]));
	return 0;
}


/*!
 * @brief set servo mode
 *
 * @param[in] id the servo id, 0-255 (255: broadcast)
 * @param[in] option B3M_OPTIONS_*
 * @return error status
 */
int b3m_servo_mode(B3MData * r, UINT id, UCHAR option)
{
	assert(r);

	UCHAR data[2];
	data[0] = option;						// mode
	data[1] = 0x00;							// interpolation

	return b3m_com_send(r, id, B3M_SERVO_SERVO_MODE, data, 2);
}

