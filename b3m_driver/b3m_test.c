#include "b3m.h"
#include <unistd.h>

int main(){
	B3MData b3m;
	char *serial_port = "/dev/ttyAMA0";
	if (b3m_init(&b3m, serial_port) < 0){
		fprintf(stderr, "cannnot port open %s\r\n", serial_port);
	}
	b3m_servo_mode(&b3m, 0, B3M_OPTIONS_RUN_NORMAL);
	b3m_set_angle(&b3m, 0, 0.0);
	int no_error = 0;
	for(int i = 0; i < 1000; i ++){
		int deg100;
		if (b3m_get_angle(&b3m, 0, &deg100) == 0) {
			printf("%d\r\n", deg100);
		} else {
			printf("error\r\n");
			no_error ++;
		}
		usleep(1000);
	}
	printf("error: %d\r\n", no_error);
	b3m_close(&b3m);
}
