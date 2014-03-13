/*
* The code is released under the GNU General Public License.
* Created 28th April 2013
*/

#define ARDUINO_ADDRESS 0x69

#include <unistd.h>
#include <signal.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <string.h>
#include "i2c-dev.h"
#include "vector.h"


int file;


void  INThandler(int sig)
{
        signal(sig, SIG_IGN);
        exit(0);
}


//servo range is 0-3
//position range is (2^14)-1
void writeServoPosition(int servo, int position)
{
	char a = 0x0;
	char b = 0x0;

	if(servo > 3 || servo < 0) //check servo range
		return;

	if(position > 180)
		position = 180;
	if(position < 0)
		position = 0;

	if(ioctl(file, I2C_SLAVE, ARDUINO_ADDRESS) < 0)
	{
		printf("Unable to open servo controller (Arduino) %s\n", strerror(errno));
	}
	a |= servo;
	b|= position;

	if(i2c_smbus_write_byte_data(file, a, b) < 0)
		printf("Failed to write to servo controller (Arduino) %s\n", strerror(errno));
}

void initServos()
{
	char filename[20];
	sprintf(filename, "/dev/i2c-%d", 1);
	file = open(filename, O_RDWR);
	if(file<0)
	{
		printf("Unable to open I2C bus!\n");
		exit(1);
	}
}

int main(int argc, char *argv[])
{
	int pos = 90;
	int servo = 0;
        signal(SIGINT, INThandler);
	initServos();
	if(argc == 3)
	{
		pos = atoi(argv[1]);
		servo = atoi(argv[2]);
	}
	printf("Setting to: %d\n", pos);
	writeServoPosition(servo,pos);
}

