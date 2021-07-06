/*============================ Control FSM ============================*/
/**
 * The Finite State Machine that manages the robot's controls. Handles
 * calls to the FSM State functions and manages transitions between all
 * of the states.
 */

#include "ControlFSM.h"

/**
 * Constructor for the Control FSM. Passes in all of the necessary
 * data and stores it in a struct. Initializes the FSM with a starting
 * state and operating mode.
 *
 * @param _quadruped the quadruped information
 * @param _stateEstimator contains the estimated states
 * @param _legController interface to the leg controllers
 * @param _gaitScheduler controls scheduled foot contact modes
 * @param _desiredStateCommand gets the desired COM state trajectories
 * @param controlParameters passes in the control parameters from the GUI
 */
template <typename T>
ControlFSM<T>::ControlFSM(Quadruped<T>* _quadruped,
                          StateEstimatorContainer<T>* _stateEstimator,
                          LegController<T>* _legController,
                          GaitScheduler<T>* _gaitScheduler,
                          DesiredStateCommand<T>* _desiredStateCommand,
                          RobotControlParameters* controlParameters,
                          // VisualizationData* visualizationData,
                          MIT_UserParameters* userParameters)
{
  // Add the pointers to the ControlFSMData struct
  data._quadruped = _quadruped;
  data._stateEstimator = _stateEstimator;
  data._legController = _legController;
  data._gaitScheduler = _gaitScheduler;
  data._desiredStateCommand = _desiredStateCommand;
  data.controlParameters = controlParameters;
  // data.visualizationData = visualizationData;
  data.userParameters = userParameters;

  // Initialize and add all of the FSM States to the state list
  statesList.invalid = nullptr;
  statesList.passive = new FSM_State_Passive<T>(&data);
  statesList.jointPD = new FSM_State_JointPD<T>(&data);
  // statesList.impedanceControl = new FSM_State_ImpedanceControl<T>(&data);
  statesList.standUp = new FSM_State_StandUp<T>(&data);
  // statesList.balanceStand = new FSM_State_BalanceStand<T>(&data);
  statesList.locomotion = new FSM_State_Locomotion<T>(&data);
  // statesList.recoveryStand = new FSM_State_RecoveryStand<T>(&data);
  // statesList.vision = new FSM_State_Vision<T>(&data);
  // statesList.backflip = new FSM_State_BackFlip<T>(&data);
  // statesList.frontJump = new FSM_State_FrontJump<T>(&data);

  safetyChecker = new SafetyChecker<T>(&data);

  // Initialize the FSM with the Passive FSM State
  initialize();
}

/**
 * Initialize the Control FSM with the default settings. SHould be set to
 * Passive state and Normal operation mode.
 */
template <typename T>
void ControlFSM<T>::initialize() {

  // // Initialize a new FSM State with the control data
  // currentState = statesList.jointPD;

  // // Enter the new current state cleanly
  // currentState->onEnter();

  // Initialize to not be in transition
  // nextState = currentState;

  // Initialize FSM mode to normal operation
  operatingMode = FSM_OperatingMode::NORMAL;
}

/**
 * Called each control loop iteration. Decides if the robot is safe to
 * run controls and checks the current state for any transitions. Runs
 * the regular state behavior if all is normal.
 */
template <typename T>
void ControlFSM<T>::runFSM() 
{

  if(isfirst)
  {
    // Initialize a new FSM State with the control data
  currentState = statesList.locomotion;

  // Enter the new current state cleanly
  currentState->onEnter();

  isfirst = false;

  }


  // Check the robot state for safe operation
	operatingMode = safetyPreCheck();

	if (operatingMode == FSM_OperatingMode::NORMAL)
	{
		// Run the iteration for the current state normally
		currentState->run();
	}

	else
	{
		currentState = statesList.passive;
    	currentState->onEnter();
	}
	

	// Print the current state of the FSM
	printInfo(0);

	// Increase the iteration counter
	iter++;
}

template <typename T>
FSM_OperatingMode ControlFSM<T>::safetyPreCheck() {
  // Check for safe orientation if the current state requires it
  if (currentState->checkSafeOrientation) 
  {
    if (!safetyChecker->checkSafeOrientation()) 
    {
      operatingMode = FSM_OperatingMode::ESTOP;
      std::cout << "broken: Orientation Safety Check FAIL" << std::endl;
    }
  }

  // Default is to return the current operating mode
  return operatingMode;
}

template <typename T>
void ControlFSM<T>::printInfo(int opt) {
  switch (opt) {
    case 0:  // Normal printing case at regular intervals
      // Increment printing iteration
      printIter++;

      // Print at commanded frequency
      if (printIter == printNum) {
        std::cout << "[CONTROL FSM] Printing FSM Info...\n";
        std::cout
            << "---------------------------------------------------------\n";
        std::cout << "Iteration: " << iter << "\n";
        if (operatingMode == FSM_OperatingMode::NORMAL) {
          std::cout << "Operating Mode: NORMAL in " << currentState->stateString
                    << "\n";

        }else if (operatingMode == FSM_OperatingMode::ESTOP) {
          std::cout << "Operating Mode: ESTOP\n";
        }
        std::cout << std::endl;

        // Reset iteration counter
        printIter = 0;
      }
      break;
  }
}

// template class ControlFSM<double>; This should be fixed... need to make
// RobotRunner a template
template class ControlFSM<float>;
