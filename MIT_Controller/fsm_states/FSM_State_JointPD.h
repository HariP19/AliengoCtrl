#ifndef FSM_STATE_JOINTPD_H
#define FSM_STATE_JOINTPD_H

#include "FSM_State.h"

template <typename T>
class FSM_State_JointPD : public FSM_State<T> 
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	FSM_State_JointPD(ControlFSMData<T>* _controlFSMData);

	void onEnter();
	void run();
	void onExit();

private:

	int iter = 0;
	DVec<T> _ini_jpos;

};

#endif