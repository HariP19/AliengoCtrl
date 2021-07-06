#include "FSM_State_Locomotion.h"
#include "WBC_Ctrl/LocomotionCtrl/LocomotionCtrl.hpp"


template <typename T>
FSM_State_Locomotion<T>::FSM_State_Locomotion(ControlFSMData<T>* _controlFSMData)
    : FSM_State<T>(_controlFSMData, FSM_StateName::LOCOMOTION, "LOCOMOTION")
{
    printf("Inside FSM Locmotion constructor\n");
    cMPCOld = new ConvexMPCLocomotion(_controlFSMData->controlParameters->controller_dt,
       								   33 / (1000. * _controlFSMData->controlParameters->controller_dt),
        							   _controlFSMData->userParameters);

    printf("Initialized MPC\n");

    this->turnOnAllSafetyChecks();
    this->checkPDesFoot = false;

    this->footFeedForwardForces = Mat34<T>::Zero();
    this->footstepLocations = Mat34<T>::Zero();
    _wbc_ctrl = new LocomotionCtrl<T>(_controlFSMData->_quadruped->buildModel());
    printf("Initialized wbc ctrl\n");
    _wbc_data = new LocomotionCtrlData<T>();
}

template <typename T>
void FSM_State_Locomotion<T>::onEnter()
{
	
  cMPCOld->initialize();
	this->_data->_gaitScheduler->gaitData._nextGait = GaitType::TROT;
	printf("[FSM LOCOMOTION] on Enter\n");
}

template <typename T>
void FSM_State_Locomotion<T>::run() 
{
  LocomotionControlStep();
}

template <typename T>
void FSM_State_Locomotion<T>::LocomotionControlStep() 
{

  cMPCOld->run<T>(*this->_data);
  Vec3<T> pDes_backup[4];
  Vec3<T> vDes_backup[4];
  Mat3<T> Kp_backup[4];
  Mat3<T> Kd_backup[4];

  for(int leg(0); leg<4; ++leg)
  {
    pDes_backup[leg] = this->_data->_legController->commands[leg].pDes;
    vDes_backup[leg] = this->_data->_legController->commands[leg].vDes;
    Kp_backup[leg] = this->_data->_legController->commands[leg].kpCartesian;
    Kd_backup[leg] = this->_data->_legController->commands[leg].kdCartesian;


  }

  // Printing cMPC Output
  //
  // printf("\n -------------------------------------------- \n");
  // pretty_print(cMPCOld->pBody_des, std::cout, "pBody_des");
  // pretty_print(cMPCOld->vBody_des, std::cout, "vBody_des");
  // pretty_print(cMPCOld->aBody_des, std::cout, "aBody_des");

  // printf("\n");

  // pretty_print(cMPCOld->pBody_RPY_des, std::cout, "pBody_RPY_des");
  // pretty_print(cMPCOld->vBody_Ori_des, std::cout, "vBody_Ori_des");

  // printf("\n");

  // for(size_t i(0); i<4; ++i)
  // {
  //     printf("\nLeg: %zu\n", i);
  //     pretty_print(cMPCOld->pFoot_des[i], std::cout, "pFoot_des");
  //     pretty_print(cMPCOld->vFoot_des[i], std::cout, "vFoot_des");
  //     pretty_print(cMPCOld->aFoot_des[i], std::cout, "aFoot_des");
  //     pretty_print(cMPCOld->Fr_des[i], std::cout, "Fr_des");
  // }

  // printf("\n");

  // pretty_print(cMPCOld->contact_state, std::cout, "contact_state");


  // Writing cMPCold data to WBC

  if(this->_data->userParameters->use_wbc > 0.9){
    _wbc_data->pBody_des = cMPCOld->pBody_des;
    _wbc_data->vBody_des = cMPCOld->vBody_des;
    _wbc_data->aBody_des = cMPCOld->aBody_des;

    _wbc_data->pBody_RPY_des = cMPCOld->pBody_RPY_des;
    _wbc_data->vBody_Ori_des = cMPCOld->vBody_Ori_des;
    
    for(size_t i(0); i<4; ++i){
      _wbc_data->pFoot_des[i] = cMPCOld->pFoot_des[i];
      _wbc_data->vFoot_des[i] = cMPCOld->vFoot_des[i];
      _wbc_data->aFoot_des[i] = cMPCOld->aFoot_des[i];
      _wbc_data->Fr_des[i] = cMPCOld->Fr_des[i]; 
    }
    _wbc_data->contact_state = cMPCOld->contact_state;
    _wbc_ctrl->run(_wbc_data, *this->_data);
  }

  for(int leg(0); leg<4; ++leg)
  {
    this->_data->_legController->commands[leg].vDes = vDes_backup[leg];
    this->_data->_legController->commands[leg].kdCartesian = Kd_backup[leg];
  }

  //DEBUG
  // if(this->_data->userParameters->use_wbc > 0.9){
  //   _wbc_data->pBody_des =  pBody_des;
  //   _wbc_data->vBody_des =  vBody_des;
  //   _wbc_data->aBody_des =  aBody_des;

  //   _wbc_data->pBody_RPY_des =  pBody_RPY_des;
  //   _wbc_data->vBody_Ori_des =  vBody_Ori_des;
    
  //   for(size_t i(0); i<4; ++i){
  //     _wbc_data->pFoot_des[i] =  pFoot_des[i];
  //     _wbc_data->vFoot_des[i] =  vFoot_des[i];
  //     _wbc_data->aFoot_des[i] =  aFoot_des[i];
  //     _wbc_data->Fr_des[i] =  Fr_des[i]; 
  //   }
  //   _wbc_data->contact_state =  contact_state;
  //   _wbc_ctrl->run(_wbc_data, *this->_data);
  // }
  // for(int leg(0); leg<4; ++leg){
  //   //this->_data->_legController->commands[leg].pDes = pDes_backup[leg];
  //   this->_data->_legController->commands[leg].vDes = vDes_backup[leg];
  //   //this->_data->_legController->commands[leg].kpCartesian = Kp_backup[leg];
  //   this->_data->_legController->commands[leg].kdCartesian = Kd_backup[leg];
  // }  

}

template <typename T>
void FSM_State_Locomotion<T>::onExit()
{}

template class FSM_State_Locomotion<float>;