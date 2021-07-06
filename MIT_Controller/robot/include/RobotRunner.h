#ifndef ROBOTRUNNER_H
#define ROBOTRUNNER_H

#include "hardware_interface.h"
#include "JPosInitializer.h"
#include "RobotController.h"
#include "Controllers/LegController.h"
#include "Controllers/DesiredStateCommand.h"
#include "Controllers/StateEstimatorContainer.h"
#include "Controllers/ContactEstimator.h"
#include "Controllers/OrientationEstimator.h"
#include "Controllers/PositionVelocityEstimator.h"
#include "Dynamics/Quadruped.h"
#include "ControlParameters/ControlParameters.h"
#include "ControlParameters/RobotParameters.h"
#include "Types/IMUTypes.h"
#include "Utilities/save_file.h"

class RobotRunner: public HardwareInterface
{
public:
    RobotRunner( const double &loop_rate, RobotController*, RobotType);
    void RobotControl();
    void init();

    ~RobotRunner();

    RobotController* _robot_ctrl;
    RobotType robotType;


private:
  JPosInitializer<float>* _jpos_initializer;
  Quadruped<float> _quadruped;
  LegController<float>* _legController = nullptr;
  VectorNavData _vectorNavData;
  StateEstimate<float> _stateEstimate;
  StateEstimatorContainer<float>* _stateEstimator;

  FloatingBaseModel<float> _model;

  RobotControlParameters* controlParameters = nullptr;
  ControlParameters* _userControlParameters = nullptr;
  SaveFileManager<float>* _saveFileManager = nullptr;

  DesiredStateCommand<float>* _desiredStateCommand;

  //DEBUG

  RobotControlParameters robotParams;


  static double targetPos[12];
  Mat3<float> kpMat;
  Mat3<float> kdMat;

  void setupStep();
  void run_jpos();
  void run_controller();
  void torque_standup();
  void finalizeStep();

  void updateVectorNav();
  void initializeStateEstimator();

  void logLegCmd();
  

  int iter;
  std::vector< Vec3<float> > _ini_foot_pos;

};

#endif