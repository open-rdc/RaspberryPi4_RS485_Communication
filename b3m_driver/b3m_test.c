#include "b3m.h"
#include <unistd.h>
#include <wiringPi.h>

int channel[19] = {0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
int switch_txrx_port[5] = {1, 2, 3, 24, 29};

int main(){
	int id = 1;
	wiringPiSetup();
	piHiPri(99);
	B3MData b3m[5];
	char serial_port[5][20] = {"/dev/ttyAMA0", "/dev/ttyAMA1", "/dev/ttyAMA2", "/dev/ttyAMA3", "/dev/ttyAMA4"};
	for(int i = 0; i < 5; i ++){
		if (b3m_init(&b3m[i], serial_port[i], switch_txrx_port[i]) < 0){
			fprintf(stderr, "cannnot port open %s\r\n", serial_port);
		}
	}
	b3m_servo_mode(&b3m[channel[0]], 0, B3M_OPTIONS_RUN_NORMAL);
	b3m_set_angle(&b3m[channel[0]], 0, 0.0);
	b3m_servo_mode(&b3m[channel[id]], id, B3M_OPTIONS_RUN_NORMAL);
	b3m_set_angle(&b3m[channel[id]], id, 0.0);
	int no_error = 0;
	for(int i = 0; i < 10000; i ++){
		int deg100;
		b3m_set_angle(&b3m[channel[0]], 0, i*1);
		b3m_set_angle(&b3m[channel[id]], id, i*1);
		usleep(0);
		if (b3m_get_angle(&b3m[channel[0]], 0, &deg100) == 0) {
			printf("%d	", deg100);
		} else {
			printf("error	");
			no_error ++;
		}
		if (b3m_get_angle(&b3m[channel[id]], id, &deg100) == 0) {
			printf("%d\r\n", deg100);
		} else {
			printf("error\r\n");
			no_error ++;
		}
		usleep(0);
	}
	printf("error: %d\r\n", no_error);
	for(int i = 0; i < 5; i ++) b3m_close(&b3m[i]);
}

