/*============================= Stand Up ==============================*/
/**
 * Transitionary state that is called for the robot to stand up into
 * balance control mode.
 */

#include "FSM_State_StandUp.h"
#include "Utilities/Utilities_print.h"

/**
 * Constructor for the FSM State that passes in state specific info to
 * the generic FSM State constructor.
 *
 * @param _controlFSMData holds all of the relevant control data
 */
template <typename T>
FSM_State_StandUp<T>::FSM_State_StandUp(ControlFSMData<T>* _controlFSMData)
    : FSM_State<T>(_controlFSMData, FSM_StateName::STAND_UP, "STAND_UP"),
_ini_foot_pos(4){
  // Do nothing
  // Set the pre controls safety checks
  this->checkSafeOrientation = false;

  // Post control safety checks
  this->checkPDesFoot = false;
  this->checkForceFeedForward = false;
}

template <typename T>
void FSM_State_StandUp<T>::onEnter() 
{
  iter = 0;

  for(size_t leg(0); leg<4; ++leg)
  {
    _ini_foot_pos[leg] = this->_data->_legController->datas[leg].p;
  }

}

/**
 * Calls the functions to be executed on each control loop iteration.
 */
template <typename T>
void FSM_State_StandUp<T>::run() 
{
  iter++;
  if(this->_data->_quadruped->_robotType == RobotType::ALIENGO) 
  {
    T hMax = 0.35;
    // T progress = 2 * iter * this->_data->controlParameters->controller_dt;
    T progress = 1 * iter * this->_data->controlParameters->controller_dt;

    if (progress > 1.){ progress = 1.; }

    for(int i = 0; i < 4; i++) {
      this->_data->_legController->commands[i].kpCartesian = Vec3<T>(400, 400, 400).asDiagonal();
      this->_data->_legController->commands[i].kdCartesian = Vec3<T>(3, 3, 3).asDiagonal();

      this->_data->_legController->commands[i].pDes = _ini_foot_pos[i];
      this->_data->_legController->commands[i].pDes[2] = 
        progress*(-hMax) + (1. - progress) * _ini_foot_pos[i][2];

   
    }
  }
}


/**
 * Cleans up the state information on exiting the state.
 */
template <typename T>
void FSM_State_StandUp<T>::onExit() {
  // Nothing to clean up when exiting
}

// template class FSM_State_StandUp<double>;
template class FSM_State_StandUp<float>;
