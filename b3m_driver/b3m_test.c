#include "b3m.h"

int main(){
	B3MData b3m;
	char *serial_port = "/dev/ttyAMA0";
	if (b3m_init(&b3m, serial_port) < 0){
		fprintf(stderr, "cannnot port open %s\r\n", serial_port);
	}
	b3m_servo_mode(&b3m, 0, B3M_OPTIONS_RUN_NORMAL);
	b3m_set_angle(&b3m, 0, 0.0);
	b3m_close(&b3m);
}
