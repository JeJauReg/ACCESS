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
#include "v1.h"

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


void  INThandler(int sig)
{
        signal(sig, SIG_IGN);
        exit(0);
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
	int GPIO = 4;
	int currentXValue = 1500;
	int currentYValue = 1500;

	vector3 *acc, *cfangle, *gyro, *normal;

	gpioInitialise();

	gpioServo(GPIO, currentYValue);

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
		gpioServo(GPIO, currentYValue);

		getCFAngle(cfangle, acc, gyro);
		normal->x = cfangle->x - 180;
		normal->y = cfangle->y - 270;

		printf(" CFangleX = %7.3f \t CFangleY = %7.3f \t pulsewidth = %d\n", normal->x, normal->y, currentYValue);
		
		if(normal->y > 1)
		{
			currentYValue = 1500 + ((int)(normal->y)/2.0)*10;
		}

		//Each loop should be at least 20ms.
        	while(mymillis() - startInt < 20)
	        {
        	    usleep(100);
	        }

		printf("Loop Time %d\t", mymillis()- startInt);
	}

}

