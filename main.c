#include <stdio.h>
#include <fcntl.h>  /* File Control Definitions          */
#include <termios.h>/* POSIX Terminal Control Definitions*/
#include <errno.h>  /* ERROR Number Definitions          */
#include <unistd.h> /* UNIX Standard Definitions         */
char* main()
//void main()
{
	int fd;
	fd = open("/dev/ttyAMA0",O_RDWR | O_NOCTTY);
	if(fd == 1)
	{
		printf("Error! in Opening ttyAMA0\n");
		printf("fd = [%d]\n",fd);
	}
	else
	{
		printf("ttyAMA0 Opened Successfully\n");
		printf("fd = [%i]\n",fd);
	}

	struct termios SerialPortSettings;

	tcgetattr(fd, &SerialPortSettings);

	// Set Baud rate
	cfsetispeed(&SerialPortSettings,B9600);
	cfsetospeed(&SerialPortSettings,B9600);

	SerialPortSettings.c_cflag &= ~PARENB;
	SerialPortSettings.c_cflag &= ~CSTOPB;
        SerialPortSettings.c_cflag &= ~CSIZE;    	// Clears the mask for setti
        SerialPortSettings.c_cflag |=  CS8;      	// Set the data bits = 8

        SerialPortSettings.c_cflag &= ~CRTSCTS;  	// No Hardware flow Con$
        SerialPortSettings.c_cflag |= CREAD | CLOCAL; 	// Enable receiver,Igno$

        SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY);          // Di$
        SerialPortSettings.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);  // No$

        SerialPortSettings.c_oflag &= ~OPOST;//No Output Processing
        // Setting Time outs
        SerialPortSettings.c_cc[VMIN] = 150; // Read at least 150 characters
        SerialPortSettings.c_cc[VTIME] = 0;  // Wait indefinetly

        if((tcsetattr(fd, TCSANOW, &SerialPortSettings)) != 0) // Set the attri
	{
        	printf("  ERROR ! in Setting attributes\n");
		return "Error setting attributes";
	}

        //------------------------------- Read data from serial port --------

        tcflush(fd, TCIFLUSH);  // Discards old data in the rx buffer

        char read_buffer[150];  // Buffer to store the data received
        int  bytes_read = 0;	// Number of bytes read by the read() system

	while(true)
	{
	        bytes_read = read(fd,&read_buffer,140); // Read the data
	        printf("  Bytes Rxed [%i]\n", bytes_read); // Print the number of bytes
		if (read_buffer[0] == '$' && read_buffer[1] == 'G')
		{
			printf("%s - break\n",read_buffer);
			break;
		}
	}

	close(fd);
	return read_buffer;
}
