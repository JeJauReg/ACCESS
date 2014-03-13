/*
* The code is released under the GNU General Public License.
* Developed by Mark Williams
* A guide to this code can be found here; http://marks-space.com/2013/04/22/845/
* Created 28th April 2013
*/


#include <unistd.h>
#include <math.h>
#include <signal.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <string.h>
#include <time.h>
#include <pigpio.h>
#include "L3G.h"
#include "LSM303.h"
#include "sensor.c"
#include "i2c-dev.h"
#include "vector.h"

#define X   0
#define Y   1
#define Z   2

#define DT 0.02         // [s/loop] loop period. 20ms
#define AA 0.98         // complementary filter constant

#define A_GAIN 0.0573      // [deg/LSB]
#define G_GAIN 0.070     // [deg/s/LSB]
#define RAD_TO_DEG 57.29578
#define M_PI 3.14159265358979323846

#define CALIBRATE_TIME 5000 //time to wait for stabilization before activating servos
#define MIDY 90
#define MIDX 90

#define ARDUINO_ADDRESS 0x69

int file;

void  INThandler(int sig)
{
        signal(sig, SIG_IGN);
        exit(0);
}

//servo range is 0-3
//position range is (2^14)-1
void writeServoPosition(int position1, int position2)
{
        char a = 0x0;
        char b = 0x0;

        if(position1 > 180)
                position1 = 90;
        if(position1 < 0)
                position1 = 90;


        if(position2 > 180)
                position2 = 90;
        if(position2 < 0)
                position2 = 90;


        if(ioctl(file, I2C_SLAVE, ARDUINO_ADDRESS) < 0)
        {
                printf("Unable to open servo controller (Arduino) %s\n", strerror(errno));
        }
        a |= position1;
        b |= position2;

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


int mymillis()
{
	struct timeval tv;
	gettimeofday(&tv, NULL);
	return (tv.tv_sec) * 1000 + (tv.tv_usec)/1000;
}

int timeval_subtract(struct timeval *result, struct timeval *t2, struct timeval *t1)
{
    long int diff = (t2->tv_usec + 1000000 * t2->tv_sec) - (t1->tv_usec + 1000000 * t1->tv_sec);
    result->tv_sec = diff / 1000000;
    result->tv_usec = diff % 1000000;
    return (diff<0);
}

/*void getMagRaw(vector3 *v)
{
	
}

void getAccRaw(vector3 *v)
{

}

void getGyrRaw(vector3 *v)
{

}*/

void getAccAngle(vector3 *v)
{
	int *Pacc_raw;
	int acc_raw[3];

	Pacc_raw = acc_raw;

	readACC(Pacc_raw);

	v->x = (float) (atan2(*(acc_raw+1),*(acc_raw+2))+M_PI)*RAD_TO_DEG;
	v->y = (float) (atan2(*(acc_raw+2),*acc_raw)+M_PI)*RAD_TO_DEG;
	//v->z?
}

void getGyrRate(vector3 *v)
{
	int *Pgyr_raw;
	int gyr_raw[3];

	Pgyr_raw = gyr_raw;

	readGYR(Pgyr_raw);

	v->x = (float) *gyr_raw * G_GAIN;
	v->y = (float) *(gyr_raw+1) * G_GAIN;
	v->z = (float) *(gyr_raw+2) * G_GAIN;

	//printf("GR[x] = %f \t GR[y] = %f \t GR[z] = %f\n",v->x, v->y, v->z);
}

void getGyrAngle(vector3 *v)
{
	getGyrRate(v);
	v->x += (v->x) * DT;
	v->y += (v->y) * DT;
	v->z += (v->z) * DT;
}

void getCFAngle(vector3 *v, vector3 *acc_angle, vector3 *gyr_rate)
{
	getAccAngle(acc_angle);
	getGyrRate(gyr_rate);

	v->x = AA*(v->x + gyr_rate->x*DT) + (1-AA) * acc_angle->x;
	v->y = AA*(v->y + gyr_rate->y*DT) + (1-AA) * acc_angle->y;
}

int main(int argc, char *argv[])
{
	int startInt  = mymillis();
	int t1;
	struct  timeval tvBegin, tvEnd,tvDiff;
	int currentXValue = MIDX;
	int currentYValue = MIDY;

	vector3 *acc, *cfangle, *gyro, *normal;

	initServos();

	acc = malloc(sizeof(vector3));
	cfangle = malloc(sizeof(vector3));
	gyro = malloc(sizeof(vector3));
	normal = malloc(sizeof(vector3));

        signal(SIGINT, INThandler);
	enableIMU();
	gettimeofday(&tvBegin, NULL);

	t1 = mymillis();
	while(mymillis() - t1 < CALIBRATE_TIME)
	{
		startInt = mymillis();
		getCFAngle(cfangle, acc, gyro);
		normal->x = cfangle->x - 180;
		normal->y = cfangle->y - 270;

		while(mymillis() - startInt < 20)
		{
			usleep(100);
		}
	}

	while(1)
	{
		startInt = mymillis();

		getCFAngle(cfangle, acc, gyro);
		normal->x = cfangle->x - 90;
		normal->y = cfangle->y - 180;

		currentYValue = (int)(normal->y);
		currentXValue = (int)(normal->x);

		printf(" CFangleX = %7.3f \t CFangleY = %7.3f \t pulsewidth = %d\n", normal->x, normal->y, currentYValue);

		writeServoPosition(currentXValue, currentYValue);

		//Each loop should be at least 20ms.
        	while(mymillis() - startInt < 20)
	        {
        	    usleep(10);
	        }

		printf("Loop Time %d\t", mymillis()- startInt);
	}

}

