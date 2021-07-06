#include "Controllers/DesiredStateCommand.h"

const Vec2<float> LEFT_STICK_ANALOG{0.0, 0.0};
const Vec2<float> RIGHT_STICK_ANALOG{0.0, 0.0};

template <typename T>
DesiredStateCommand<T>::DesiredStateCommand()
{
	joyPresent = true;

  if( ( joy_fd = open( JOY_DEV , O_RDONLY)) == -1 )
	{
		printf( "Couldn't open joystick\n" );
		joyPresent = false;
	}
  else
  {
    printf("Joystick detected: %s\n\t%d axis\n\t%d buttons\n\n"
    , name_of_joystick
    , num_of_axis
    , num_of_buttons );
  }
  

  ioctl( joy_fd, JSIOCGAXES, &num_of_axis );
  ioctl( joy_fd, JSIOCGBUTTONS, &num_of_buttons );
  ioctl( joy_fd, JSIOCGNAME(80), &name_of_joystick );

  axis = (int *) calloc( num_of_axis, sizeof( int ) );
  button = (char *) calloc( num_of_buttons, sizeof( char ) );

  fcntl( joy_fd, F_SETFL, O_NONBLOCK ); /* use non-blocking mode */
}

template <typename T>
void DesiredStateCommand<T>::convertToStateCommands() 
{
	Vec2<float> joystickLeft, joystickRight;

	if(joyPresent)
  {
    joystickLeft[0] = 0.5 * axis[0]/(32767.0);
    joystickLeft[1] = -0.5 * axis[1]/(32767.0);
    joystickRight[0] = 0.5* axis[2]/(32767.0);
  }
  else
  {
    joystickLeft = LEFT_STICK_ANALOG;
    joystickRight = RIGHT_STICK_ANALOG;
  }
  
  joystickLeft[0] *= -1.f;
	joystickRight[0] *= -1.f;

	leftAnalogStick = leftAnalogStick * (T(1) - filter) + joystickLeft * filter;
	rightAnalogStick = rightAnalogStick * (T(1) - filter) + joystickRight * filter;
}

template <typename T>
void DesiredStateCommand<T>::getJoyCommands()
{
	 /* read the joystick state */
    read(joy_fd, &js, sizeof(struct js_event));
    
      /* see what to do with the event */
    switch (js.type & ~JS_EVENT_INIT)
    {
      case JS_EVENT_AXIS:
        axis   [ js.number ] = js.value;
        break;
      case JS_EVENT_BUTTON:
        button [ js.number ] = js.value;
        break;
    }

	fflush(stdout); 
}

template <typename T>
void DesiredStateCommand<T>::closeJoy()
{
  close(joy_fd);
}

template class DesiredStateCommand<double>;
template class DesiredStateCommand<float>;