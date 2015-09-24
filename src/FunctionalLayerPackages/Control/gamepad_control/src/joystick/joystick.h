/*
 * joystick.h
 *
 *  Created on: Mar 22, 2012
 *      Author: catec
 */


#include <stdio.h>
//#include <ros/ros.h>
//#include <stdio.h>
//#include <sys/types.h>
//#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
//#include <unistd.h>
//#include <stdlib.h>
//#include <string.h>
//#include <catec_msgs/ControlReferenceRwStamped.h>
#include <string>
//#include <algorithm>
//#include <iostream>
//#include <math.h>
//#include <signal.h>

#ifndef __JOYSTICK_H__
#define __JOYSTICK_H__

#ifndef PI
#define PI 3.141592653589793
#endif

#define JOYSTICK_DEVNAME "/dev/input/js0"

#define JS_EVENT_BUTTON         0x01    /* button pressed/released */
#define JS_EVENT_AXIS           0x02    /* joystick moved */
#define JS_EVENT_INIT           0x80    /* initial state of device */

using namespace std;

struct js_event {
	unsigned int time;	/* event timestamp in milliseconds */
	short value;   /* value */
	unsigned char type;     /* event type */
	unsigned char number;   /* axis/button number */
};

struct wwvi_js_event {
	int button[11];
	int stick_x;
	int stick_y;
	int stick2_x;
	int stick2_y;
	int stick3_x;
	int stick3_y;
};

class CJoystick
{
public:
	int open_joystick(char* joystick);
	int read_joystick_event(struct js_event *jse);
	void close_joystick();
	int get_joystick_status(struct wwvi_js_event *wjse);
	CJoystick():joystick_fd(-1)
	{

	}
protected:
	int joystick_fd;
};

#endif
