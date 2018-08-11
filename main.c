#include <stdio.h>
#include <fcntl.h>  /* File Control Definitions          */
#include <termios.h>/* POSIX Terminal Control Definitions*/
#include <errno.h>  /* ERROR Number Definitions          */
#include <unistd.h> /* UNIX Standard Definitions         */
//char* main()
void main()
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
        SerialPortSettings.c_cc[VMIN] = 100; // Read at least 10 characters
        SerialPortSettings.c_cc[VTIME] = 0; // Wait indefinetly   

        if((tcsetattr(fd, TCSANOW, &SerialPortSettings)) != 0) // Set the attri$
        	printf("\n  ERROR ! in Setting attributes");
        else
		printf("\n  BaudRate = 9600 \n  StopBits = 1 \n  Parity   = none");

        //------------------------------- Read data from serial port --------

        tcflush(fd, TCIFLUSH);   // Discards old data in the rx buffer       

        char read_buffer[32];   // Buffer to store the data received         
        int  bytes_read = 0;    // Number of bytes read by the read() system 
        int i = 0;

        bytes_read = read(fd,&read_buffer,32); // Read the data

        printf("\n\n  Bytes Rxed -%d", bytes_read); // Print the number of by
        printf("\n\n  ");

	for(i=0;i<bytes_read;i++)        //printing only the received charact
	        printf("%c",read_buffer[i]);

	close(fd);
//	return "Hello";
}
