#include "FSM_State_JointPD.h"
#include "Utilities/Utilities_print.h"

template <typename T>
FSM_State_JointPD<T>::FSM_State_JointPD(ControlFSMData<T>* _controlFSMData)
	: FSM_State<T>(_controlFSMData, FSM_StateName::JOINT_PD, "JOINT_PD"),
	  _ini_jpos(cheetah::num_act_joint)
{}

template <typename T>
void FSM_State_JointPD<T>::onEnter()
{
	printf("Entered JointPD onEnter\n");

	iter = 0;

	
  for(size_t leg(0); leg<4; ++leg){
    for(size_t jidx(0); jidx <3; ++jidx){
      _ini_jpos[3*leg + jidx] = FSM_State<T>::_data->_legController->datas[leg].q[jidx];
    }
  }

	pretty_print(_ini_jpos, std::cout, "initial pose");
}

template <typename T>
void FSM_State_JointPD<T>::run()
{
	Vec3<T> qDes;
	// qDes << 0.0, 0.0, 0.0;
	qDes << 0.0, -0.67, 1.3;

	Vec3<T> qdDes;
	qdDes << 0, 0, 0;

	static double progress(0.);
	progress += this->_data->controlParameters->controller_dt;
	double movement_duration(5.0);
	double ratio = progress/movement_duration;

	if(ratio > 1.)
		ratio = 1.;


	this->jointPDControl(0, ratio*qDes + (1. - ratio)*_ini_jpos.head(3), qdDes);
  	this->jointPDControl(1, ratio*qDes + (1. - ratio)*_ini_jpos.segment(3, 3), qdDes);
  	this->jointPDControl(2, ratio*qDes + (1. - ratio)*_ini_jpos.segment(6, 3), qdDes);
  	this->jointPDControl(3, ratio*qDes + (1. - ratio)*_ini_jpos.segment(9, 3), qdDes);
}

template <typename T>
void FSM_State_JointPD<T>::onExit()
{}

template class FSM_State_JointPD<float>;
