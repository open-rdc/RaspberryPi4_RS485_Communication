#include "b3m5.h"
#include <unistd.h>
#include <wiringPi.h>

int channel[19] = {0,1,2,3,4,5,0,0,0,0,0,0,0,0,0,0,0,0,0};
int switch_txrx_port[5] = {1, 2, 3, 24, 29};
UINT id[5] = {0, 1, 2, 3, 4};
int ang100[5] = {0, 0, 0, 0, 0};

int main(){
	wiringPiSetup();
	piHiPri(99);
	B3MData b3m[5];
	char serial_port[5][20] = {"/dev/ttyAMA0", "/dev/ttyAMA1", "/dev/ttyAMA2", "/dev/ttyAMA3", "/dev/ttyAMA4"};
	if (b3m_init(b3m, serial_port, switch_txrx_port) < 0){
		fprintf(stderr, "cannnot port open\r\n");
	}
	b3m_servo_mode(b3m, id, B3M_OPTIONS_RUN_NORMAL);
	b3m_set_angle(b3m, id, ang100);
	int no_error = 0;
	for(int i = 0; i < 10000; i ++){
		int deg100[5];
		ang100[0] = ang100[1] = ang100[2] = ang100[3] = ang100[4] = i;
		b3m_set_angle(b3m, id, ang100);
		usleep(0);
		if (b3m_get_angle(b3m, id, deg100) == 0) {
			//printf("%d	%d	%d	%d	%d\r\n", deg100[0], deg100[1], deg100[2], deg100[3], deg100[4]);
		} else {
			//printf("error\r\n");
			no_error ++;
		}
		usleep(0);
	}
	printf("error: %d\r\n", no_error);
	b3m_close(b3m);
}

