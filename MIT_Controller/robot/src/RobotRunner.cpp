#include "RobotRunner.h"
#include "Dynamics/Aliengo.h"

double RobotRunner::targetPos[12] = {
                        0.0, -0.67, 1.3, 
                     -0.0, -0.67, 1.3, 
                        0.0, -0.67, 1.3, 
                        -0.0, -0.67, 1.3
                                         };


RobotRunner::RobotRunner( const double &loop_rate, RobotController* robot_ctrl, RobotType rType)
        :HardwareInterface(LeggedType::A1, LOWLEVEL, loop_rate),
        _robot_ctrl(robot_ctrl),
        robotType(rType)
{
    kpMat << 50, 0, 0, 0, 50, 0, 0, 0, 50;
    kdMat << 2, 0, 0, 0, 2, 0, 0, 0, 2;
}

void RobotRunner::init()
{
    
    Init();
    
    _quadruped = buildAliengo<float>();
    _model = _quadruped.buildModel();

    controlParameters = new RobotControlParameters();

    _userControlParameters = _robot_ctrl->getUserControlParameters();

    //Load Robot Parameters
    try {
        controlParameters->initializeFromYamlFile(THIS_COM "/config/mini-cheetah-defaults.yaml");
    } catch(std::exception& e) {
        printf("Failed to initialize robot parameters from yaml file: %s\n", e.what());
        exit(1);
    }

    if(!controlParameters->isFullyInitialized()) {
        printf("Failed to initialize all robot parameters\n");
        exit(1);
    }

    printf("Robot parameters loaded sucessfully\n");

    //Load User Parameters
    try 
    {
        _userControlParameters->initializeFromYamlFile(THIS_COM "config/mc-mit-ctrl-user-parameters.yaml");
    } catch(std::exception& e) 
    {
        printf("Failed to initialize user parameters from yaml file: %s\n", e.what());
        exit(1);
    }

    if(!_userControlParameters->isFullyInitialized()) 
    {
        printf("Failed to initialize all user parameters\n");
        exit(1);
    }

    _saveFileManager = new SaveFileManager<float>("29June_3", "/log/");

    _jpos_initializer = new JPosInitializer<float>(10., 0.002);

    _legController = new LegController<float>(_quadruped);

    _stateEstimator = new StateEstimatorContainer<float>(&_vectorNavData, _legController->datas, &_stateEstimate, controlParameters, _saveFileManager);
    initializeStateEstimator();


    _desiredStateCommand = new DesiredStateCommand<float>();

    _robot_ctrl->_model = &_model;
	_robot_ctrl->_quadruped = &_quadruped;
	_robot_ctrl->_legController = _legController;
  	_robot_ctrl->_stateEstimator = _stateEstimator;
  	_robot_ctrl->_stateEstimate = &_stateEstimate;
	_robot_ctrl->_robotType = RobotType::ALIENGO;
	_robot_ctrl->_controlParameters = controlParameters;
	_robot_ctrl->_desiredStateCommand = _desiredStateCommand;

	 _robot_ctrl->initializeController();

    iter = 0;

}

void RobotRunner::RobotControl()
{
    ++iter;
    
    setupStep();
    if(iter > 10){
            // run_jpos();
        run_controller();
    }

    finalizeStep();
}

void RobotRunner::setupStep()
{
    updateVectorNav();
    _legController->updateData(&data);
    _legController->zeroCommand();
    _legController->setEnabled(true);

    _stateEstimator->run();

}

void RobotRunner::run_jpos()
{
    //Run JPos Initializer 
    if(!_jpos_initializer->IsInitialized(_legController))
	{
        for (int leg = 0; leg < 4; leg++) 
        {
          _legController->commands[leg].kpJoint = kpMat;
          _legController->commands[leg].kdJoint = kdMat;
        }
	}
    else
	{
        for (int leg = 0; leg < 4; leg++) 
        {
          _legController->commands[leg].kpJoint = kpMat;
          _legController->commands[leg].kdJoint = kdMat;
        }
    	for (int leg=0; leg<4; ++leg)
	    	for (int jidx=0; jidx<3; ++jidx)
		    {
		    	_legController->commands[leg].tauFeedForward[jidx] = 0.;
	        	_legController->commands[leg].qDes[jidx] = targetPos[3 * leg + jidx];
	        	_legController->commands[leg].qdDes[jidx] = 0.;
		    }
	}

}

void RobotRunner::torque_standup()
{
    if(robotType == RobotType::UNITREE_A1){
        float hMax = 0.25;
        float progress = 2 * iter * 0.002; //TODO replace 0.002 with controller-dt variable

        if(progress > 1.){ progress = 1.; }
        if(iter == 0)
        {
            for(size_t leg(0); leg<4; ++leg)
            {
                _ini_foot_pos[leg] = _legController->datas[leg].p;
            }
        }

        for(int i = 0; i < 4; i++) 
        {
            _legController->commands[i].kpCartesian = Vec3<float>(500, 500, 500).asDiagonal();
            _legController->commands[i].kdCartesian = Vec3<float>(8, 8, 8).asDiagonal();

            _legController->commands[i].pDes = _ini_foot_pos[i];
            _legController->commands[i].pDes[2] = 
        progress*(-hMax) + (1. - progress) * _ini_foot_pos[i][2];
        }
    }

    iter++;
}

void RobotRunner::run_controller()
{
       
    if(!_jpos_initializer->IsInitialized(_legController))
	{
        for (int leg = 0; leg < 4; leg++) 
        {
          _legController->commands[leg].kpJoint = kpMat;
          _legController->commands[leg].kdJoint = kdMat;
        }
	}
    else
	{
        if(iter < (1500 + 5000)) // jpos + wait time
        {
            for (int leg = 0; leg < 4; leg++) 
            {
                _legController->commands[leg].kpJoint = kpMat;
                _legController->commands[leg].kdJoint = kdMat;
            }
            for (int leg=0; leg<4; ++leg)
                for (int jidx=0; jidx<3; ++jidx)
                {
                    _legController->commands[leg].tauFeedForward[jidx] = 0.;
                    _legController->commands[leg].qDes[jidx] = targetPos[3 * leg + jidx];
                    _legController->commands[leg].qdDes[jidx] = 0.;
                }
            }
        else
        {
            // _saveFileManager->saveValue("running MPC Controller:");
            _robot_ctrl->runController();
        }
        
	}
}

void RobotRunner::finalizeStep()
{
    #ifdef DEBUG
    _stateEstimator->log_estimate();
    logLegCmd();
    #endif

    _legController->updateCommand(&cmd);
}

void RobotRunner::updateVectorNav()
{
    _vectorNavData.accelerometer[0] = data.imu.accelerometer[0];
    _vectorNavData.accelerometer[1] = data.imu.accelerometer[1];
    _vectorNavData.accelerometer[2] = data.imu.accelerometer[2];

    _vectorNavData.quat[0] = data.imu.quaternion[1];
    _vectorNavData.quat[1] = data.imu.quaternion[2];
    _vectorNavData.quat[2] = data.imu.quaternion[3];
    _vectorNavData.quat[3] = data.imu.quaternion[0];  //CHECK if the format is the same 

    _vectorNavData.gyro[0] = data.imu.gyroscope[0];
    _vectorNavData.gyro[1] = data.imu.gyroscope[1];
    _vectorNavData.gyro[2] = data.imu.gyroscope[2];
}

void RobotRunner::initializeStateEstimator() 
{
    _stateEstimator->removeAllEstimators();

    _stateEstimator->addEstimator<ContactEstimator<float>>();
    
    Vec4<float> contactDefault;
    contactDefault << 0.5, 0.5, 0.5, 0.5;
    _stateEstimator->setContactPhase(contactDefault);

    _stateEstimator->addEstimator<VectorNavOrientationEstimator<float>>();

    _stateEstimator->addEstimator<LinearKFPositionVelocityEstimator<float>>();

}

void RobotRunner::logLegCmd()
{
    _saveFileManager->saveValue("\n*******************Leg Controller Command*******************");
    _saveFileManager->saveValue("tauEstimate:");
    for(int leg = 0; leg < 4; leg++)
    {
        _saveFileManager->saveValue("Leg: " + std::to_string(leg));
        _saveFileManager->saveVector(_legController->datas[leg].tauEstimate);
    }

    _saveFileManager->saveValue("\nqDes:");
    for(int leg = 0; leg < 4; leg++)
    {
        _saveFileManager->saveValue("Leg: " + std::to_string(leg) );
        _saveFileManager->saveVector(_legController->commands[leg].qDes);
    }
    _saveFileManager->saveValue("\nq:");
    for(int leg = 0; leg < 4; leg++)
    {
        _saveFileManager->saveValue("Leg: " + std::to_string(leg) );
        _saveFileManager->saveVector(_legController->datas[leg].q);
    }

    _saveFileManager->saveValue("\nqdDes:");
    for(int leg = 0; leg < 4; leg++)
    {
        _saveFileManager->saveValue("Leg: " + std::to_string(leg) );
        _saveFileManager->saveVector(_legController->commands[leg].qdDes);
    }

    _saveFileManager->saveValue("\nqd:");
    for(int leg = 0; leg < 4; leg++)
    {
        _saveFileManager->saveValue("Leg: " + std::to_string(leg) );
        _saveFileManager->saveVector(_legController->datas[leg].qd);
    }

    _saveFileManager->saveValue("\npDes:");
    for(int leg = 0; leg < 4; leg++)
    {
        _saveFileManager->saveValue("Leg: " + std::to_string(leg) );
        _saveFileManager->saveVector(_legController->commands[leg].pDes);
    }

    _saveFileManager->saveValue("\nvDes:");
    for(int leg = 0; leg < 4; leg++)
    {
        _saveFileManager->saveValue("Leg: " + std::to_string(leg) );
        _saveFileManager->saveVector(_legController->commands[leg].vDes);
    }
}

RobotRunner::~RobotRunner()
{
    printf("Deleting RobotRunner \n");
    delete _legController;
    delete _jpos_initializer;
    delete _stateEstimator;
    delete _desiredStateCommand;
}