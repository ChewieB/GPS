#include <stdio.h>
#include <fcntl.h>  /* File Control Definitions          */
#include <termios.h>/* POSIX Terminal Control Definitions*/
#include <errno.h>  /* ERROR Number Definitions          */
#include <unistd.h> /* UNIX Standard Definitions         */
char* main()
{
	int fd;
	fd = open("/dev/ttyAMA0",O_RDWR | O_NOCTTY);
	if(fd == 1)
	{
		printf("\n  Error! in Opening ttyAMA0\n");
	}
	else
		printf("\n  ttyAMA0 Opened Successfully\n");
	close(fd);
	return "Hello";
}

