/*
* The code is released under the GNU General Public License.
* Created 28th April 2013
*/


#include <unistd.h>
#include <signal.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <string.h>
#include "i2c-dev.h"
#include "vector.h"





void  INThandler(int sig)
{
        signal(sig, SIG_IGN);
        exit(0);
}


int main(int argc, char *argv[])
{
        signal(SIGINT, INThandler);
	
	
}

