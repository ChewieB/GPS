#include <stdio.h>
#include <fcntl.h>  /* File Control Definitions          */
#include <termios.h>/* POSIX Terminal Control Definitions*/
#include <errno.h>  /* ERROR Number Definitions          */
#include <unistd.h> /* UNIX Standard Definitions         */
void main()
{
  int fd;
  fd = open("/dev/ttyUSB0",O_RDWR | O_NOCTTY);
  if(fd == ­1)
     printf("\n  Error! in Opening ttyUSB0\n");
  else
     printf("\n  ttyUSB0 Opened Successfully\n");
  close(fd);
}

