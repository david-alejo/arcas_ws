/*
 * joystick.cpp
 *
 *  Created on: Mar 22, 2012
 *      Author: catec
 */
#include<iostream>

#include "joystick.h"


int CJoystick::open_joystick(char* joystick) {
	joystick_fd = open(joystick, O_RDONLY | O_NONBLOCK); /* read write for force feedback? */
	if (joystick_fd < 0)
		return joystick_fd;

	/* maybe ioctls to interrogate features here? */

	return joystick_fd;
}

int CJoystick::read_joystick_event(struct js_event *jse) {
	int bytes;

	bytes = read(joystick_fd, jse, sizeof(*jse));

	if (bytes == -1)
		return 0;

	if (bytes == sizeof(*jse))
		return 1;

	printf("Unexpected bytes from joystick:%d\n", bytes);

	return -1;
}

void CJoystick::close_joystick() {
	close(joystick_fd);
}

int CJoystick::get_joystick_status(struct wwvi_js_event* wjse) {
	int rc;
	struct js_event jse;
	if (joystick_fd < 0)
		return -1;

	// memset(wjse, 0, sizeof(*wjse));
	while ((rc = read_joystick_event(&jse) == 1)) {
		jse.type &= ~JS_EVENT_INIT; /* ignore synthetic events */
		if (jse.type == JS_EVENT_AXIS) {
			switch (jse.number) {

			case 5:
				wjse->stick3_x = (((jse.value+32767)*2048)/-65534) + 1024;
				break;
			case 4:
				wjse->stick3_y = (((jse.value+32767)*2048)/-65534) + 1024;
				break;
			case 3:
				wjse->stick_x = (((jse.value+32767)*2048)/-65534) + 1024;
				break;
			case 1:
				wjse->stick_y = (((jse.value+32767)*2048)/-65534) + 1024;
				break;
			case 2:
				wjse->stick2_x = (((jse.value+32767)*2048)/-65534) + 1024;
				break;
			case 0:
				wjse->stick2_y = (((jse.value+32767)*2048)/-65534) + 1024;
				break;
			default:
				break;
			}
		} else if (jse.type == JS_EVENT_BUTTON) {
			if (jse.number < 10) {
				switch (jse.value) {
				case 0:
				case 1:
					wjse->button[jse.number] = jse.value;
					break;
				default:
					break;
				}
			}
		}
	}
	// printf("%d\n", wjse->stick1_y);
	return 0;
}
