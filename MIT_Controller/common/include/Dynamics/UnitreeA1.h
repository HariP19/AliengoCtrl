#ifndef PROJECT_MINICHEETAH_H
#define PROJECT_MINICHEETAH_H

#include "FloatingBaseModel.h"
#include "Quadruped.h"

/*!
 * Generate a Quadruped model of Mini Cheetah
 */
template <typename T>
Quadruped<T> buildA1() {
  Quadruped<T> quadruped;
  quadruped._robotType = RobotType::UNITREE_A1;

  quadruped._bodyMass = 4.713;
  quadruped._bodyLength = 0.183 * 2;
  quadruped._bodyWidth = 0.047 * 2;
  quadruped._bodyHeight = 0.114;
  quadruped._abadGearRatio = 6;
  quadruped._hipGearRatio = 6;
  quadruped._kneeGearRatio = 9.33;
  quadruped._abadLinkLength = 0.062;
  quadruped._hipLinkLength = 0.209;
  //quadruped._kneeLinkLength = 0.175;
  //quadruped._maxLegLength = 0.384;
  quadruped._kneeLinkY_offset = 0.004;
  //quadruped._kneeLinkLength = 0.20;
  quadruped._kneeLinkLength = 0.195;
  quadruped._maxLegLength = 0.409;


  quadruped._motorTauMax = 3.f;
  quadruped._batteryV = 24;
  quadruped._motorKT = .05;  // this is flux linkage * pole pairs
  quadruped._motorR = 0.173;
  quadruped._jointDamping = .01;
  quadruped._jointDryFriction = .2;
  //quadruped._jointDamping = .0;
  //quadruped._jointDryFriction = .0;


  // rotor inertia if the rotor is oriented so it spins around the z-axis
  Mat3<T> rotorRotationalInertiaZ;
  rotorRotationalInertiaZ << 33, 0, 0, 0, 33, 0, 0, 0, 63;
  rotorRotationalInertiaZ = 1e-6 * rotorRotationalInertiaZ;

  Mat3<T> RY = coordinateRotation<T>(CoordinateAxis::Y, M_PI / 2);
  Mat3<T> RX = coordinateRotation<T>(CoordinateAxis::X, M_PI / 2);
  Mat3<T> rotorRotationalInertiaX =
      RY * rotorRotationalInertiaZ * RY.transpose();
  Mat3<T> rotorRotationalInertiaY =
      RX * rotorRotationalInertiaZ * RX.transpose();

  // spatial inertias

  Mat3<T> abadRotationalInertia;
  abadRotationalInertia << 807, -0.466 , -9.409 , -0.466 , 552.929, -0.342 , -9.409 , -0.342 , 469.246;
  abadRotationalInertia = abadRotationalInertia * 1e-6;
  Vec3<T> abadCOM(0.000635 , 3.10E-05 , -0.003311);  // LEFT
  SpatialInertia<T> abadInertia(0.54, abadCOM, abadRotationalInertia);

  Mat3<T> hipRotationalInertia;
  hipRotationalInertia << -5529.065 ,-343.869,  -4.825,  -343.869 , 1367.788 , 22.448,  -4.825 , 22.448 , 5139.339;
  hipRotationalInertia = hipRotationalInertia * 1e-6;
  Vec3<T> hipCOM(0.003237, -0.027326, -0.022327);
  SpatialInertia<T> hipInertia(0.634, hipCOM, hipRotationalInertia);

  Mat3<T> kneeRotationalInertia, kneeRotationalInertiaRotated;
  kneeRotationalInertiaRotated << 32.426 , -141.163 , 0 ,-141.163,  2897.972 , 0 ,0 ,0 ,3014.022;
  kneeRotationalInertiaRotated = kneeRotationalInertiaRotated * 1e-6;
  kneeRotationalInertia = RY * kneeRotationalInertiaRotated * RY.transpose();
  Vec3<T> kneeCOM(-0.107388, 0.006435 , 0);
  SpatialInertia<T> kneeInertia(0.064, kneeCOM, kneeRotationalInertia);

  Vec3<T> rotorCOM(0, 0, 0);
  SpatialInertia<T> rotorInertiaX(0.055, rotorCOM, rotorRotationalInertiaX);
  SpatialInertia<T> rotorInertiaY(0.055, rotorCOM, rotorRotationalInertiaY);

  Mat3<T> bodyRotationalInertia;
  bodyRotationalInertia << 11253, 0, 0, 0, 36203, 0, 0, 0, 42673;
  bodyRotationalInertia = bodyRotationalInertia * 1e-6;
  Vec3<T> bodyCOM(0, 0, 0);
  SpatialInertia<T> bodyInertia(quadruped._bodyMass, bodyCOM,
                                bodyRotationalInertia);

  quadruped._abadInertia = abadInertia;
  quadruped._hipInertia = hipInertia;
  quadruped._kneeInertia = kneeInertia;
  quadruped._abadRotorInertia = rotorInertiaX;
  quadruped._hipRotorInertia = rotorInertiaY;
  quadruped._kneeRotorInertia = rotorInertiaY;
  quadruped._bodyInertia = bodyInertia;

  // locations
  quadruped._abadRotorLocation = Vec3<T>(0.125, 0.049, 0);
  quadruped._abadLocation =
      Vec3<T>(quadruped._bodyLength, quadruped._bodyWidth, 0) * 0.5;
  quadruped._hipLocation = Vec3<T>(0, quadruped._abadLinkLength, 0);
  quadruped._hipRotorLocation = Vec3<T>(0, 0.04, 0);
  quadruped._kneeLocation = Vec3<T>(0, 0, -quadruped._hipLinkLength);
  quadruped._kneeRotorLocation = Vec3<T>(0, 0, 0);

  return quadruped;
}

#endif  // PROJECT_MINICHEETAH_H