#ifndef MIT_CONTROLLER
#define MIT_CONTROLLER

#include <RobotController.h>
// #include "Controllers/GaitScheduler.h"
#include "Controllers/ContactEstimator.h"
#include "fsm_states/ControlFSM.h"
#include "MIT_UserParameters.h"
//#include <gui_main_control_settings_t.hpp>

class MIT_Controller: public RobotController{
public:
  MIT_Controller();
  virtual ~MIT_Controller(){}

  virtual void initializeController();
  virtual void runController();
  virtual ControlParameters* getUserControlParameters() {
      return &userParameters;
  }
  virtual void Estop(){ _controlFSM->initialize(); } //CHECK


protected:
  ControlFSM<float>* _controlFSM;
  GaitScheduler<float>* _gaitScheduler;
  MIT_UserParameters userParameters;
  
};

#endif
