/*!
 * @file DesiredStateCommand.h
 * @brief Logic to convert a joystick command into a desired trajectory for the robot
 *
 * This will generate a state trajectory which can easily be used for model predictive controllers
 */

/*========================= Gamepad Control ==========================*/
/**
 *
 */
#ifndef DESIRED_STATE_COMMAND_H
#define DESIRED_STATE_COMMAND_H

#include <iostream>
#include <stdio.h>
#include <cstdlib>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/joystick.h>
#define JOY_DEV "/dev/input/js0"
#include "Types/cppTypes.h"

template <typename T>
class DesiredStateCommand 
{

public:	
	Vec2<float> leftAnalogStick;
  	Vec2<float> rightAnalogStick;
  	bool trigger_pressed = false;

	DesiredStateCommand();

  	void convertToStateCommands();
	void getJoyCommands();
	void closeJoy();

private:
	bool joyPresent;
	
	const T filter = 0.1;

	//Joystick params
	struct js_event js; 
	int joy_fd;
	int *axis=NULL, num_of_axis=0, num_of_buttons=0, x;
	char *button=NULL, name_of_joystick[80];

};

#endif