/**
 * Copyright (C) 2017 Chalmers Revere
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301,
 * USA.
 */

#ifndef KALMAN_HPP
#define KALMAN_HPP

#include <tuple>
#include <utility>
#include <thread>
#include <Eigen/Dense>
#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"

class Kalman {


private:
 Kalman(const Kalman &) = delete;
 Kalman(Kalman &&)      = delete;
 Kalman &operator=(const Kalman &) = delete;
 Kalman &operator=(Kalman &&) = delete;
public:
  Kalman(std::map<std::string, std::string> commandlineArguments,cluon::OD4Session &a_od4);
  ~Kalman();

void nextPose(cluon::data::Envelope data);
void nextSlamPose(cluon::data::Envelope data);
void nextEllipsePose(cluon::data::Envelope data);
void nextHeading(cluon::data::Envelope data);
void nextGroundSpeed(cluon::data::Envelope data);
void nextYawRate(cluon::data::Envelope data);
void nextAcceleration(cluon::data::Envelope data);
void setStateMachineStatus(cluon::data::Envelope data);
bool getStateMachineStatus();
void nextRack(cluon::data::Envelope data);
bool getModuleState();
void sendStates(uint32_t ukfStamp);
void initializeModule();
void UKFPrediction();
void UKFUpdate();
void checkVehicleState();
bool getFilterInitState();
void filterInitialization();
bool getEllipseState();

 private:
  void setUp(std::map<std::string, std::string> commandlineArguments);

  double rackTravelToFrontWheelSteering(float &rackTravel);
  double magicFormula(double &alpha, double &Fz, double const &mu);
  double calculateHeading(double x1, double y1,double x2,double y2);
  Eigen::MatrixXd UKFWeights();
  Eigen::MatrixXd sigmaPoints(Eigen::MatrixXd &x);
  Eigen::MatrixXd vehicleModel(Eigen::MatrixXd x);
  Eigen::MatrixXd measurementModel(Eigen::MatrixXd x);
  void tearDown();

  cluon::OD4Session &od4;
  Eigen::Vector3d m_odometryData;

  Eigen::Vector3d m_odometrySlamData;
  Eigen::Vector3d m_acceleration;
  double m_yawRate = 0;
  double m_delta = 0;
  double m_groundSpeed = 0;
  double m_lastHeadingMeasurement = 0;
  double m_lastEllipseHeading = 0;
  double m_lastSentHeading = 0;
  int m_laps = 0;
  cluon::data::TimeStamp m_yawReceivedTime = {};
  cluon::data::TimeStamp m_lastYawReceivedTime = {};
  cluon::data::TimeStamp m_groundSpeedReceivedTime = {};
  cluon::data::TimeStamp m_lastGroundSpeedReceivedTime = {};
  cluon::data::TimeStamp m_geolocationReceivedTime ={};
  cluon::data::TimeStamp m_lastGeolocationReceivedTime ={};
  cluon::data::TimeStamp m_accReceivedTime ={};
  cluon::data::TimeStamp m_lastAccReceivedTime ={};
  cluon::data::TimeStamp m_rackReceivedTime ={};
  cluon::data::TimeStamp m_wheelReceivedTime ={};
  std::mutex m_poseMutex;
  std::mutex m_yawMutex;
  std::mutex m_groundSpeedMutex;
  std::mutex m_stateMachineMutex;
  std::mutex m_accMutex;
  std::mutex m_deltaMutex;
  std::mutex m_wheelMutex;
  std::mutex m_initMutex;
  Eigen::MatrixXd m_states;
  Eigen::MatrixXd m_Q;
  Eigen::MatrixXd m_R;
  std::array<double,2> m_gpsReference;
  Eigen::MatrixXd m_vehicleModelParameters;
  Eigen::MatrixXd m_stateCovP;
  Eigen::Vector2f m_wheelSpeed;
  Eigen::Vector2d m_lastPos;
  double m_maxSpeed = 0;
  int m_validRackMeasurements = 0;
  double m_startHeading = 0;
  double m_headingInitDistance = 0;
  double m_startHeadingEkf = 0;
  double m_headingBiasFromSlam = 0;
  int m_headingEkfInitCounter = 0;
  //Ready Variables
  bool m_readyStateMachine = false; //Start false
  bool m_readyState = false; //Start false
  bool m_filterInit = false; //Start false
  bool m_zeroVelState = true;
  bool m_ellipseState = false;

  int m_validGroundSpeedMeasurements = 0;
  int m_validWheelLeftMeasurements = 0;
  int m_validWheelRightMeasurements = 0;
  int m_validHeadingMeasurements = 0;
  uint32_t m_wheelIdLeft;
  uint32_t m_wheelIdRight;
  std::vector<Eigen::Vector2d> m_positionVec = {};
  std::vector<double> m_filterInitYaw = {};
  double m_currentVelMean = 0;
  double m_currentAccMean = 0;
  double m_currentYawMean = 0;
  double m_currentVelCov = 0;  
  double m_currentAccCov = 0;
  double m_currentYawCov = 0;
  int m_velMeasurementCount = 0;  
  int m_accMeasurementCount = 0;
  int m_yawMeasurementCount = 0;
  bool m_gotLeft = false;
  bool m_gotRight = false;
  double const m_alphaConst = 20000;
  bool m_ekfStartHeadingInitiated = false;
  bool m_ekfStartHeadingCloser = false;
  bool m_recievedSlamPose = false;
  bool m_motionSlamUpdate = false;
  double m_slamHeadingOffset = 0;
  double m_slamXOffset = 0;
  double m_slamYOffset = 0;

  double m_rX = 0;
  double m_rY = 0;
  double m_rVelX = 0;
  double m_rAccX = 0;
  double m_rAccY = 0;
  double m_rYaw = 0;
  double m_rHeading = 0;  
    // Constants for degree transformation
  const double DEG2RAD = 0.017453292522222; // PI/180.0
  const double RAD2DEG = 57.295779513082325; // 1.0 / DEG2RAD;
  const double PI = 3.14159265f;
};

//Wheelspeed encode id 1504-left | 1505-right | CID 219
#endif
