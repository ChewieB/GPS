#include <stdio.h>
#include <fcntl.h>  /* File Control Definitions          */
#include <termios.h>/* POSIX Terminal Control Definitions*/
#include <errno.h>  /* ERROR Number Definitions          */
#include <unistd.h> /* UNIX Standard Definitions         */
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>

int OpenFile(void)
{
	int fd;
	fd = open("/dev/ttyAMA0",O_RDWR | O_NOCTTY);
	if (fd == -1)
	{
		printf("Error! in Opening ttyAMA0\n");
		printf("fd = [%d]\n",fd);
		return -1;
	}
	return fd;
}

int main(int argc, char *argv[])
{
	if(argc > 2)
	{
		printf("Wrong number of arguments [%i]\n -h for help!\n", argc);
		return -1;
	}

	if (argc == 1)
	{
		int fd = OpenFile();
		if(fd == -1)
		{
			printf("File Open Error");
			return -1;
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
			return -1;
		}

	        //------------------------------- Read data from serial port --------

        	tcflush(fd, TCIFLUSH);  // Discards old data in the rx buffer

	        char read_buffer[150];  // Buffer to store the data received

		bool ValidNEMA = false;

		for (int i = 0; i < 30; i++)
		{
			read(fd,&read_buffer,140); // Read the data
			if (read_buffer[0] == '$' && 
                	    read_buffer[1] == 'G' &&
	                    read_buffer[2] == 'P' &&
        	            read_buffer[3] == 'G' &&
                	    read_buffer[4] == 'G' &&
	                    read_buffer[5] == 'A' )
			{
				ValidNEMA = true;
				break;
			}
		}

		if (!ValidNEMA)
		{
			printf("No Valid NEMA sentences read :(\n");
			return -1;
		}

		char* p;

		int comma2 = 0;
		int comma3 = 0;
		p = strchr(read_buffer,',');
		p = strchr(p + 1, ',');
		comma2 = p - read_buffer;
		p = strchr(p + 1, ',');
		comma3 = p - read_buffer;

		char sub[1000];
		int c = 0, length, position;
		length = comma3 - comma2;
		position = comma2;

		while (c < length - 1)
		{
			sub[c] = read_buffer[position + 1 + c];
	      		c++;
   		}
	   	sub[c] = '\0';

		float fred = atof(sub)/100.0F;

		p = strchr(p + 1, ',');
		int comma4 = p - read_buffer;
		p = strchr(p + 1, ',');
		int comma5 = p - read_buffer;
		char ns = read_buffer[comma3 + 1];

		length = comma5 - comma4;
		position = comma4 + 1;
		for (int i = 0; i < length - 1; i++)
		{
			sub[i] = read_buffer[position + i];
		}
		sub[c] = '\0';
		float East = atof(sub)/100.0F;

		char ew = read_buffer[comma5 + 1];

		printf("%f %c %f %c", fred, ns, East, ew);

		close(fd);
		return 1;
	}
	else
	{
		if (!strncasecmp("-h",argv[1],2))
		{
			printf("GPS Serial reader (c) M Baker 2018\n");
			printf("-h     Help            -t    GPS Time\n");
			printf("-s     Sats'visible    -d    direction & speed\n");
			printf("-v     version		nil  Lat & Long\n");
			printf("-p     Port ToDo\n");
			return -1;
		}
		else if (!strncasecmp("-t", argv[1], 2))
		{
			printf("Time is now !   ToDo\n");
			return -1;
		}
		else
		{
			printf("No argument handling Yet!\n");
			return -1; // for now
		}
	}
}
