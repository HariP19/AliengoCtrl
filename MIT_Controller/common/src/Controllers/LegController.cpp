#include "Controllers/LegController.h"
#include "Utilities/Utilities_print.h"

/*!
 * Zero the leg command so the leg will not output torque
 */
template <typename T>
void LegControllerCommand<T>::zero() {
  tauFeedForward = Vec3<T>::Zero();
  forceFeedForward = Vec3<T>::Zero();
  qDes = Vec3<T>::Zero();
  qdDes = Vec3<T>::Zero();
  pDes = Vec3<T>::Zero();
  vDes = Vec3<T>::Zero();
  kpCartesian = Mat3<T>::Zero();
  kdCartesian = Mat3<T>::Zero();
  kpJoint = Mat3<T>::Zero();
  kdJoint = Mat3<T>::Zero();
}

/*!
 * Zero the leg data
 */
template <typename T>
void LegControllerData<T>::zero() {
  q = Vec3<T>::Zero();
  qd = Vec3<T>::Zero();
  p = Vec3<T>::Zero();
  v = Vec3<T>::Zero();
  J = Mat3<T>::Zero();
  tauEstimate = Vec3<T>::Zero();
}

template <typename T>
void LegController<T>::zeroCommand() {
  for (auto& cmd : commands) {
    cmd.zero();
  }
  _legsEnabled = false;
}


template <typename T>
void LegController<T>::updateData(const UNITREE_LEGGED_SDK::LowState* lowState) {
  for (int leg = 0; leg < 4; leg++) {
    // q:
    datas[leg].q(0) = lowState->motorState[3*leg+0].q;
    datas[leg].q(1) = -lowState->motorState[3*leg+1].q;
    datas[leg].q(2) = -lowState->motorState[3*leg+2].q;

    // qd
    datas[leg].qd(0) = lowState->motorState[3*leg+0].dq;
    datas[leg].qd(1) = -lowState->motorState[3*leg+1].dq;
    datas[leg].qd(2) = -lowState->motorState[3*leg+2].dq;


    // J and p
    computeLegJacobianAndPosition<T>(_quadruped, datas[leg].q, &(datas[leg].J),
                                     &(datas[leg].p), leg);

    // v
    datas[leg].v = datas[leg].J * datas[leg].qd;

  }
}

template <typename T>
void LegController<T>::updateCommand(UNITREE_LEGGED_SDK::LowCmd* lowCmd) 
{

  for (int leg = 0; leg < 4; leg++) {

    // tauFF
    Vec3<T> legTorque = commands[leg].tauFeedForward;

    // forceFF
    Vec3<T> footForce = commands[leg].forceFeedForward;

    // cartesian PD
    footForce +=
        commands[leg].kpCartesian * (commands[leg].pDes - datas[leg].p);
    footForce +=
        commands[leg].kdCartesian * (commands[leg].vDes - datas[leg].v);

    // Torque
    legTorque += datas[leg].J.transpose() * footForce;

    // set command:
    lowCmd->motorCmd[leg*3+0].tau = legTorque(0);
    lowCmd->motorCmd[leg*3+1].tau = -legTorque(1);
    lowCmd->motorCmd[leg*3+2].tau = -legTorque(2);

    // jolegnt space pd
    // joint space PD
    lowCmd->motorCmd[leg*3+0].Kd = commands[leg].kdJoint(0, 0);
    lowCmd->motorCmd[leg*3+1].Kd = commands[leg].kdJoint(1, 1);
    lowCmd->motorCmd[leg*3+2].Kd = commands[leg].kdJoint(2, 2);

    lowCmd->motorCmd[leg*3+0].Kp = commands[leg].kpJoint(0, 0);
    lowCmd->motorCmd[leg*3+1].Kp = commands[leg].kpJoint(1, 1);
    lowCmd->motorCmd[leg*3+2].Kp = commands[leg].kpJoint(2, 2);

    lowCmd->motorCmd[leg*3+0].q = commands[leg].qDes(0);
    lowCmd->motorCmd[leg*3+1].q = -commands[leg].qDes(1);
    lowCmd->motorCmd[leg*3+2].q = -commands[leg].qDes(2);

    lowCmd->motorCmd[leg*3+0].dq = commands[leg].qdDes(0);
    lowCmd->motorCmd[leg*3+1].dq = -commands[leg].qdDes(1);
    lowCmd->motorCmd[leg*3+2].dq = -commands[leg].qdDes(2);

    // estimate torque
    datas[leg].tauEstimate =
        legTorque +
        commands[leg].kpJoint * (commands[leg].qDes - datas[leg].q) +
        commands[leg].kdJoint * (commands[leg].qdDes - datas[leg].qd);
  }
}

template <typename T>
void LegController<T>::print_legCmd()
{
  printf("\nFoot Position\n");
  for(int leg = 0; leg<4; leg++)
  {
    printf("\tleg %d , position: %f, %f, %f\n", leg, commands[leg].qDes[0], commands[leg].qDes[1], commands[leg].qDes[2]);
  }

  printf("\nFoot Position\n");
  for(int leg = 0; leg<4; leg++)
  {
    printf("\tleg %d , position: %f, %f, %f\n", leg, datas[leg].q[0], datas[leg].q[1], datas[leg].q[2]);
  }

  // printf("\nFoot Velocity\n");
  // for(int leg = 0; leg<4; leg++)
  // {
  //   printf("\tleg %d , velocity: %f, %f, %f\n", leg, datas[leg].v[0], datas[leg].v[1], datas[leg].v[2]);
  // }

  printf("\nEstimate Torque\n");
  for(int leg = 0; leg<4; leg++)
  {
    printf("\tleg %d , Torque: %f, %f, %f\n", leg, datas[leg].tauEstimate[0], datas[leg].tauEstimate[1], datas[leg].tauEstimate[2]);
  }


  // printf("\nKp Mat\n");
  //   pretty_print(commands[0].kpJoint, std::cout, "Kp");

  // printf("\nKd Mat\n");
  //   pretty_print(commands[0].kdJoint, std::cout, "Kd");

  // printf("\nkpCartesian Mat\n");
  //   pretty_print(commands[0].kpCartesian, std::cout, "KpCartesian");
  
  // printf("\nKdCartesian Mat\n");
  //   pretty_print(commands[0].kdCartesian, std::cout, "KdCartesian");
    
}

//CHECK Zero offset for the controller and Unitree robots

//TODO Remove unwanted templates
template struct LegControllerCommand<double>;
template struct LegControllerCommand<float>;

template struct LegControllerData<double>;
template struct LegControllerData<float>;

template class LegController<double>;
template class LegController<float>;


template <typename T>
void computeLegJacobianAndPosition(Quadruped<T>& quad, Vec3<T>& q, Mat3<T>* J,
                                   Vec3<T>* p, int leg) {

  //TODO Needs to be changed
  T l1 = quad._abadLinkLength;
  T l2 = quad._hipLinkLength;
  T l3 = quad._kneeLinkLength;
  T l4 = quad._kneeLinkY_offset;
  T sideSign = quad.getSideSign(leg);

  T s1 = std::sin(q(0));
  T s2 = std::sin(q(1));
  T s3 = std::sin(q(2));

  T c1 = std::cos(q(0));
  T c2 = std::cos(q(1));
  T c3 = std::cos(q(2));

  T c23 = c2 * c3 - s2 * s3;
  T s23 = s2 * c3 + c2 * s3;

  if (J) {
    J->operator()(0, 0) = 0;
    J->operator()(0, 1) = l3 * c23 + l2 * c2;
    J->operator()(0, 2) = l3 * c23;
    J->operator()(1, 0) = l3 * c1 * c23 + l2 * c1 * c2 - (l1+l4) * sideSign * s1;
    J->operator()(1, 1) = -l3 * s1 * s23 - l2 * s1 * s2;
    J->operator()(1, 2) = -l3 * s1 * s23;
    J->operator()(2, 0) = l3 * s1 * c23 + l2 * c2 * s1 + (l1+l4) * sideSign * c1;
    J->operator()(2, 1) = l3 * c1 * s23 + l2 * c1 * s2;
    J->operator()(2, 2) = l3 * c1 * s23;
  }

  if (p) {
    p->operator()(0) = l3 * s23 + l2 * s2;
    p->operator()(1) = (l1+l4) * sideSign * c1 + l3 * (s1 * c23) + l2 * c2 * s1;
    p->operator()(2) = (l1+l4) * sideSign * s1 - l3 * (c1 * c23) - l2 * c1 * c2;
  }
}

template void computeLegJacobianAndPosition<double>(Quadruped<double>& quad,
                                                    Vec3<double>& q,
                                                    Mat3<double>* J,
                                                    Vec3<double>* p, int leg);
template void computeLegJacobianAndPosition<float>(Quadruped<float>& quad,
                                                   Vec3<float>& q,
                                                   Mat3<float>* J,
                                                   Vec3<float>* p, int leg);
