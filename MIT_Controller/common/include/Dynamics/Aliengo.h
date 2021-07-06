#ifndef PROJECT_MINICHEETAH_H
#define PROJECT_MINICHEETAH_H

#include "FloatingBaseModel.h"
#include "Quadruped.h"

/*!
 * Generate a Quadruped model of Mini Cheetah
 */
template <typename T>
Quadruped<T> buildAliengo() {
  Quadruped<T> quadruped;
  quadruped._robotType = RobotType::ALIENGO;

  quadruped._bodyMass = 9.041;
  quadruped._bodyLength = 0.647;
  quadruped._bodyWidth = 0.150;
  quadruped._bodyHeight = 0.112;
  quadruped._abadGearRatio = 6;  //
  quadruped._hipGearRatio = 6;   //
  quadruped._kneeGearRatio = 9.33;  //
  quadruped._abadLinkLength = 0.0418;   
  quadruped._hipLinkLength = 0.25; 
  //quadruped._kneeLinkLength = 0.175;
  //quadruped._maxLegLength = 0.384;
  quadruped._kneeLinkY_offset = 0.083;
  //quadruped._kneeLinkLength = 0.20;
  quadruped._kneeLinkLength = 0.25;
  quadruped._maxLegLength = 0.5;


  quadruped._motorTauMax = 44.0f;
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
  abadRotationalInertia << 2903.894, -71.85, -1.262, -71.85, 4907.517, -1.75, -1.262, -1.75, 5586.944;   //ixx,ixy,ixz, iyx,iyy,iyz,izx,izy,izz
  abadRotationalInertia = abadRotationalInertia * 1e-6;
  Vec3<T> abadCOM(-0.022191, 0.015144, -0.000015);  // LEFT
  SpatialInertia<T> abadInertia(1.993, abadCOM, abadRotationalInertia);

  Mat3<T> hipRotationalInertia;
  hipRotationalInertia << 5666.803, 3.597, 491.446, 3.597, 5847.229, 10.086, 491.446, 10.086, 369.811;     //ixx,ixy,ixz, iyx,iyy,iyz,izx,izy,izz
  hipRotationalInertia = hipRotationalInertia * 1e-6;
  Vec3<T> hipCOM(-0.005607, -0.003877, -0.048199);
  SpatialInertia<T> hipInertia(0.639, hipCOM, hipRotationalInertia);

  Mat3<T> kneeRotationalInertia, kneeRotationalInertiaRotated;
  kneeRotationalInertiaRotated << 6341.369, -0.003, -87.951, -0.003, 6355.157, -1.336, -87.951, -1.336, 39.188;  //ixx,ixy,ixz, iyx,iyy,iyz,izx,izy,izz
  kneeRotationalInertiaRotated = kneeRotationalInertiaRotated * 1e-6;
  kneeRotationalInertia = RY * kneeRotationalInertiaRotated * RY.transpose();
  Vec3<T> kneeCOM(0.002781, 0.000063, -0.142518);
  SpatialInertia<T> kneeInertia(0.207, kneeCOM, kneeRotationalInertia);

  Vec3<T> rotorCOM(0, 0, 0);
  SpatialInertia<T> rotorInertiaX(0.055, rotorCOM, rotorRotationalInertiaX);
  SpatialInertia<T> rotorInertiaY(0.055, rotorCOM, rotorRotationalInertiaY);

  Mat3<T> bodyRotationalInertia;
  bodyRotationalInertia << 33260.231, -451.628, 487.603, -451.628, 161172.11, 48.356, 487.603, 48.356, 174604.42;  //ixx,ixy,ixz, iyx,iyy,iyz,izx,izy,izz
  bodyRotationalInertia = bodyRotationalInertia * 1e-6;
  Vec3<T> bodyCOM(0.008465, 0.004045, -0.000763);
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