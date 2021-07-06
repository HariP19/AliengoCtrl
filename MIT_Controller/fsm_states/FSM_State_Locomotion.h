#ifndef FSM_STATE_LOCOMOTION_H
#define FSM_STATE_LOCOMOTION_H

#include <convexMPC/ConvexMPCLocomotion.h>
#include "FSM_State.h"

template<typename T> class WBC_Ctrl;
template<typename T> class LocomotionCtrlData;
/**
 *
 */
template <typename T>
class FSM_State_Locomotion : public FSM_State<T> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  FSM_State_Locomotion(ControlFSMData<T>* _controlFSMData);

  // Behavior to be carried out when entering a state
  void onEnter();

  // Run the normal behavior for the state
  void run();

  // Checks for any transition triggers
  FSM_StateName checkTransition();

  // Manages state specific transitions
  // TransitionData<T> transition();

  // Behavior to be carried out when exiting a state
  void onExit();

 private:
  // Keep track of the control iterations
  int iter = 0;
  ConvexMPCLocomotion* cMPCOld;
  WBC_Ctrl<T> * _wbc_ctrl;
  LocomotionCtrlData<T> * _wbc_data;

  // Parses contact specific controls to the leg controller
  void LocomotionControlStep();

  bool locomotionSafe();

  // Impedance control for the stance legs during locomotion
  void StanceLegImpedanceControl(int leg);

  //WBC test parameters

  // Standing
  Vec3<float> pBody_des = {0.042467,  -0.000670,   0.306346};  //Unitree A1
  Vec3<float> vBody_des= {0.000000,  0.000000,  0.000000};
  Vec3<float> aBody_des= {0.000000,  0.000000,  0.000000};

  //   Vec3<float> pBody_des = {0.000001,   0.001300,   0.20700}; //Mini-cheetah
  // Vec3<float> vBody_des= {0.000000,  0.000000,  0.000000};
  // Vec3<float> aBody_des= {0.000000,  0.000000,  0.000000};



  Vec3<float> pBody_RPY_des= {0.000000,  0.000000,  0.000000};
  Vec3<float> vBody_Ori_des= {0.000000,  0.000000,  0.000000};

  // Four legs on ground

  // Vec3<float> pFoot_des[4]= { {0.195784,  -0.114680,  -0.003000},
  //                           {0.187163,  0.108687,  -0.002828},
  //                           {-0.187688,  -0.127418,  -0.002826},
  //                           {-0.189955,   0.103456,  -0.003000}};

  Vec3<float> pFoot_des[4]= { {0.195784,  -0.114680,  -0.003000},
                          {0.187163,  0.108687,  -0.002828},
                          {-0.187688,  -0.127418,  -0.002826},
                          {-0.189955,   0.103456,  -0.003000}};

                              
  Vec3<float> vFoot_des[4]= { {0.000000, -0.000000, -0.000000},
                              {0.000000, -0.000000, -0.000000},
                              {0.000000, -0.000000, -0.000000},
                              {0.000000, -0.000000, -0.000000}};
  Vec3<float> aFoot_des[4]= { {0.000000, -0.000000, -0.000000},
                              {0.000000, -0.000000, -0.000000},
                              {0.000000, -0.000000, -0.000000},
                              {0.000000, -0.000000, -0.000000}};

  // Assumption: 
  //             Total weight = 122.0394
  //             Hind weight = 42.71379 (0.35xtotal weight)
  //             Front weight = 79.32561


  Vec3<float> Fr_des[4]= { {0.000000, -0.000000, 39.662805},  //Unitree A1
                            {0.000000, -0.000000, 39.662805},    
                            {0.000000, -0.000000, 21.356895},
                            {0.000000, -0.000000, 21.356895}};

  // Vec3<float> Fr_des[4]= { {0.000000, -0.000000, 20.05739},  //Mini-cheetah
  //                           {0.000000, -0.000000, 20.05739},
  //                           {0.000000, -0.000000, 25.752276},
  //                           {0.000000, -0.000000, 25.752276}};

  Vec4<float> contact_state= {1, 1, 1, 1};
};

#endif  // FSM_STATE_LOCOMOTION_H
