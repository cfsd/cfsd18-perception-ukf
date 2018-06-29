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

#include <iostream>

#include "ukf.hpp"
#include "WGS84toCartesian.hpp"

Kalman::Kalman(std::map<std::string, std::string> commandlineArguments,cluon::OD4Session &a_od4) :
  od4(a_od4)
  , m_odometryData()
  , m_acceleration()
  , m_poseMutex()
  , m_yawMutex()
  , m_groundSpeedMutex()
  , m_stateMachineMutex()
  , m_accMutex()
  , m_deltaMutex()
  , m_states()
  , m_Q()
  , m_R()
  , m_gpsReference()
  , m_sampleTime()
  , m_vehicleModelParameters()
  , m_stateCovP()
{
  setUp(commandlineArguments);
  m_odometryData << 0,0,0;
  m_acceleration << 0,0,0;
  m_states = Eigen::MatrixXd::Zero(6,1);
  m_vehicleModelParameters = Eigen::MatrixXd::Zero(7,1);
  m_Q = Eigen::MatrixXd::Zero(6,6); //Six states
  m_R = Eigen::MatrixXd::Zero(7,7); //Seven Measurements
  m_stateCovP = Eigen::MatrixXd::Identity(6,6); //Initialize P

}


void Kalman::setUp(std::map<std::string, std::string> configuration)
{


	  m_gpsReference[0] = static_cast<double>(std::stod(configuration["refLatitude"]));
	  m_gpsReference[1] = static_cast<double>(std::stod(configuration["refLongitude"]));
	  m_sampleTime = static_cast<double>(std::stod(configuration["sampleTime"]));
	
	  double const qX = static_cast<double>(std::stod(configuration["Qx"]));
	  double const qY = static_cast<double>(std::stod(configuration["Qy"]));
	  double const qVelX = static_cast<double>(std::stod(configuration["QvelX"]));
	  double const qVelY = static_cast<double>(std::stod(configuration["QvelY"]));
	  double const qYaw = static_cast<double>(std::stod(configuration["Qyaw"]));
	  double const qHeading = static_cast<double>(std::stod(configuration["Qheading"]));

	  m_Q << qX,0,0,0,0,0,
	  		 0,qY,0,0,0,0,
	  		 0,0,qVelX,0,0,0,
	  		 0,0,0,qVelY,0,0,
	  		 0,0,0,0,qYaw,0,
	  		 0,0,0,0,0,qHeading;

	  //m_paramVecR << rX,rY,rVelX,rAccX,rAccY,rYaw,rHeading;

	  double const rX = static_cast<double>(std::stod(configuration["Rx"]));
	  double const rY = static_cast<double>(std::stod(configuration["Ry"]));
	  double const rVelX = static_cast<double>(std::stod(configuration["RvelX"]));
	  double const rAccX = static_cast<double>(std::stod(configuration["RaccX"]));
	  double const rAccY = static_cast<double>(std::stod(configuration["RaccY"]));
	  double const rYaw = static_cast<double>(std::stod(configuration["Ryaw"]));
	  double const rHeading = static_cast<double>(std::stod(configuration["Rheading"]));		 
	  m_R << rX,0,0,0,0,0,0,
	  		 0,rY,0,0,0,0,0,
	  		 0,0,rVelX,0,0,0,0,
	  		 0,0,0,rAccX,0,0,0,
	  		 0,0,0,0,rAccY,0,0,
	  		 0,0,0,0,0,rYaw,0,
	  		 0,0,0,0,0,0,rHeading;

	  double const vM = static_cast<double>(std::stod(configuration["m"]));
	  double const vIz = static_cast<double>(std::stod(configuration["Iz"]));
	  double const vG = static_cast<double>(std::stod(configuration["g"]));
	  double const vL = static_cast<double>(std::stod(configuration["l"]));
	  double const vLf = static_cast<double>(std::stod(configuration["lf"]));
	  double const vLr = static_cast<double>(std::stod(configuration["lr"]));
	  double const vMu = static_cast<double>(std::stod(configuration["mu"]));

	m_vehicleModelParameters << vM,vIz,vG,vL,vLf,vLr,vMu;
  //m_timeBetweenKeyframes = static_cast<double>(std::stod(configuration["timeBetweenKeyframes"]));

}

void Kalman::nextPose(cluon::data::Envelope data){
    //#########################Recieve Odometry##################################
  
  std::lock_guard<std::mutex> lockPose(m_poseMutex);
  m_geolocationReceivedTime = data.sampleTimeStamp();
  auto odometry = cluon::extractMessage<opendlv::logic::sensation::Geolocation>(std::move(data));

  double longitude = odometry.longitude();
  double latitude = odometry.latitude();

  //toCartesian(const std::array<double, 2> &WGS84Reference, const std::array<double, 2> &WGS84Position)

  std::array<double,2> WGS84ReadingTemp;

  WGS84ReadingTemp[0] = latitude;
  WGS84ReadingTemp[1] = longitude;

  std::array<double,2> WGS84Reading = wgs84::toCartesian(m_gpsReference, WGS84ReadingTemp); 
  //opendlv::data::environment::WGS84Coordinate gpsCurrent = opendlv::data::environment::WGS84Coordinate(latitude, longitude);
  //opendlv::data::environment::Point3 gpsTransform = m_gpsReference.transform(gpsCurrent);

  m_odometryData << WGS84Reading[0],
                    WGS84Reading[1],
                    odometry.heading();                 
}

void Kalman::nextGroundSpeed(cluon::data::Envelope data){

  std::lock_guard<std::mutex> lockGroundSpeed(m_groundSpeedMutex);
  auto groundSpeed = cluon::extractMessage<opendlv::proxy::GroundSpeedReading>(std::move(data));
  m_groundSpeed = groundSpeed.groundSpeed();
  m_groundSpeedReceivedTime = data.sampleTimeStamp();
   //std::cout << "Yaw in message: " << m_yawRate << std::endl;
}
void Kalman::nextYawRate(cluon::data::Envelope data){

  std::lock_guard<std::mutex> lockYaw(m_yawMutex);
  auto yawRate = cluon::extractMessage<opendlv::proxy::AngularVelocityReading>(std::move(data));
  m_yawRate = static_cast<double>(yawRate.angularVelocityZ());
   m_yawReceivedTime = data.sampleTimeStamp();
   //std::cout << "Yaw in message: " << m_yawRate << std::endl;
}

void Kalman::nextAcceleration(cluon::data::Envelope data){
  std::lock_guard<std::mutex> lockAcc(m_accMutex);
  auto acceleration = cluon::extractMessage<opendlv::proxy::AccelerationReading>(std::move(data));
  m_acceleration(0) = acceleration.accelerationX();
  m_acceleration(1) = acceleration.accelerationY();
  m_acceleration(2) = acceleration.accelerationZ();
  m_accReceivedTime = data.sampleTimeStamp();
}


void Kalman::nextRack(cluon::data::Envelope data){	
	std::lock_guard<std::mutex> lockDelta(m_deltaMutex);

  auto rackTravel = cluon::extractMessage<opendlv::proxy::GroundSteeringReading>(std::move(data));
  float rT = rackTravel.groundSteering();
  m_rackReceivedTime = data.sampleTimeStamp();
  m_delta = rackTravelToFrontWheelSteering(rT);
}

void Kalman::setStateMachineStatus(cluon::data::Envelope data){
  std::lock_guard<std::mutex> lockStateMachine(m_stateMachineMutex);
  auto machineStatus = cluon::extractMessage<opendlv::proxy::SwitchStateReading>(std::move(data));
  int state = machineStatus.state();
  if(state == 2){
    m_readyStateMachine = true;
  }
  
}

bool Kalman::getStateMachineStatus(){

	return m_readyStateMachine;
}

bool Kalman::getModuleState(){

  return m_readyState;

}

double Kalman::rackTravelToFrontWheelSteering(float &rackTravel)
{

	float const rackTravelToSteeringAngleLineSlope = -1.225f;
	rackTravel = (rackTravel-100.3f)/5.0f;

	double delta = static_cast<double>(rackTravelToSteeringAngleLineSlope*rackTravel*3.14159/180); 

	return delta;
}
double Kalman::magicFormula(double &alpha, double &Fz, double const &mu)
{

	double const C = 1;
	double const c_alpha = 25229;
	double const B = c_alpha/C/mu/Fz;
	double const E = -2;
	double Fy = mu*Fz*std::sin(C*std::atan(B*alpha - E*(B*alpha - std::atan(B*alpha))));

	return Fy;
}

Eigen::MatrixXd Kalman::UKFWeights()
{

 	int const n = 6; //Amount of states
	double const alpha = 0.5;
	double const beta = 2;
	double const kappa = 3 - n;
	double const lambda = std::pow(alpha,2)*(n + kappa) - n;

	//Calculate weights to sigma points
	double Wm0 = lambda/(n+lambda);
	double Wc0 = lambda / (n + lambda) + (1 - std::pow(alpha,2) + beta);;

	Eigen::MatrixXd Wmc = Eigen::MatrixXd::Zero(2,2*n+1);

	Wmc(0,0) = Wm0;
	Wmc(1,0) = Wc0;

	for(int i = 0; i < 2*n; i++){
		Wmc(0,i+1) = 1 / (2 * (n + lambda));
		Wmc(1,i+1) = Wmc(0,i+1);
	}

	return Wmc;
}

Eigen::MatrixXd Kalman::sigmaPoints(Eigen::MatrixXd &x)
{

	int const n = x.rows(); //Amount of states
	double const alpha = 0.5;
	double const kappa = 3 - n;
	double const lambda = std::pow(alpha,2)*(n + kappa) - n;
	double c = std::sqrt(n + lambda);

	Eigen::MatrixXd Pchol(m_stateCovP.llt().matrixL());
	Eigen::MatrixXd A = c*Pchol;
	Eigen::MatrixXd SP = Eigen::MatrixXd::Zero(n,2*n+1);
	Eigen::MatrixXd Ap = Eigen::MatrixXd::Zero(n,n);
	Eigen::MatrixXd An = Eigen::MatrixXd::Zero(n,n);

	for(int j = 0; j < A.cols(); j++){

		Ap.col(j) = x+A.col(j);
		An.col(j) = x-A.col(j);

	}

	SP << x, Ap, An;
	return SP;
}

void Kalman::UKFPrediction()
{	

	int const n = 6;
	Eigen::MatrixXd x = m_states;
	Eigen::MatrixXd Wmc = UKFWeights();
	Eigen::MatrixXd SP = sigmaPoints(x);
	Eigen::MatrixXd x_hat = Eigen::MatrixXd::Zero(n,1);
	Eigen::MatrixXd sigmaPoint, sigmaStates;
	//Calculate Mean

	for(int i = 0; i < 2*n+1; i++){

		sigmaPoint = SP.col(i);
		sigmaStates = vehicleModel(sigmaPoint);
		x_hat = x_hat + sigmaStates*Wmc(0,i);

	}

	//Calculate Covariance
	Eigen::MatrixXd P_temp = Eigen::MatrixXd::Zero(n,n); 
	for(int i = 0; i < 2*n+1; i++){

		sigmaPoint = SP.col(i);
		sigmaStates = vehicleModel(sigmaPoint);
		P_temp = P_temp + (sigmaStates-x_hat)*(sigmaStates-x_hat).transpose()*Wmc(1,i);

	}

	P_temp = P_temp + m_Q;
	//Dirty trick to keep numerical stability
	m_stateCovP = (P_temp + P_temp.transpose())/2;

	m_states = x_hat;
}

void Kalman::UKFUpdate()
{

		Eigen::MatrixXd x = m_states;
		Eigen::MatrixXd Wmc = UKFWeights();
		Eigen::MatrixXd SP = sigmaPoints(x);
		int const n = x.rows();

		Eigen::MatrixXd y_hat = Eigen::MatrixXd::Zero(7,1);
		Eigen::MatrixXd sigmaPoint, sigmaStates;
		Eigen::MatrixXd y = Eigen::MatrixXd::Zero(7,1);
		//Calculate Mean
	for(int i = 0; i < 2*n+1;i++){

		sigmaPoint = SP.col(i);
		sigmaStates = measurementModel(sigmaPoint);
		y_hat = y_hat + sigmaStates*Wmc(0,i);

	}

	Eigen::MatrixXd Pxy = Eigen::MatrixXd::Zero(6,7);
	//Innovation covariance

	Eigen::MatrixXd S = m_R;
	for(int i = 0; i < 2*n+1; i++){

		sigmaPoint = SP.col(i);
		sigmaStates = measurementModel(sigmaPoint);

		Pxy = Pxy + (sigmaPoint-x)*(sigmaStates-y_hat).transpose()*Wmc(1,i);
		S = S + (sigmaStates-y_hat)*(sigmaStates-y_hat).transpose()*Wmc(1,i);

	}
	//Collect measurements from sensor
	{
  		std::lock_guard<std::mutex> lockPose(m_poseMutex);
  		std::lock_guard<std::mutex> lockGroundSpeed(m_groundSpeedMutex);
  		std::lock_guard<std::mutex> lockYaw(m_yawMutex);
  		std::lock_guard<std::mutex> lockAcc(m_accMutex);

		y << m_odometryData(0), 
		 	m_odometryData(1),
		 	m_groundSpeed,
		 	m_acceleration(0),
		 	m_acceleration(1),
		 	m_yawRate,
		 	m_odometryData(2);
	}

	//State update
	x = x + Pxy*S.inverse()*(y-y_hat);
	m_stateCovP = m_stateCovP - Pxy*S.inverse()*Pxy.transpose();
	//Dirty trick to keep numerical stability
	m_stateCovP = (m_stateCovP + m_stateCovP.transpose())/2;
	
	m_states =  x;
}


Eigen::MatrixXd Kalman::vehicleModel(Eigen::MatrixXd x)
{
	std::lock_guard<std::mutex> lockDelta(m_deltaMutex);
	
	if(x(2) < 0.0001){

		x(2) = 0.01;
	}

	Eigen::MatrixXd xdot = Eigen::MatrixXd::Zero(x.rows(),1);
	double alphaF = std::atan((m_vehicleModelParameters(3)*x(4)) + x(3)/x(2)) - m_delta;
	double alphaR = std::atan((x(3)-m_vehicleModelParameters(4)*x(4))/x(2));

	//Non linear Tire Model

	double Fzf = m_vehicleModelParameters(0)*m_vehicleModelParameters(2)*(m_vehicleModelParameters(4)/(m_vehicleModelParameters(4)+m_vehicleModelParameters(3)));
	double Fzr = m_vehicleModelParameters(0)*m_vehicleModelParameters(2)*(m_vehicleModelParameters(3)/(m_vehicleModelParameters(4)+m_vehicleModelParameters(3)));

	double Fyf = magicFormula(alphaF,Fzf,m_vehicleModelParameters(5));
	double Fyr = magicFormula(alphaR,Fzr,m_vehicleModelParameters(5));


	xdot << x(2),
			x(3),
			-Fyf*std::sin(m_delta)/m_vehicleModelParameters(0) + x(4)*x(3),
			(Fyf*std::cos(m_delta)+Fyr)/m_vehicleModelParameters(0) - x(4)*x(3),
			(m_vehicleModelParameters(3)*Fyf*std::cos(m_delta)-m_vehicleModelParameters(4)*Fyr)/m_vehicleModelParameters(1),
			x(4);

	Eigen::MatrixXd fx = x + xdot*m_sampleTime;

	return fx;
}

Eigen::MatrixXd Kalman::measurementModel(Eigen::MatrixXd x)
{

	std::lock_guard<std::mutex> lockDelta(m_deltaMutex);
	
	Eigen::MatrixXd hx = Eigen::MatrixXd::Zero(7,1);
	double alphaF = std::atan((m_vehicleModelParameters(3)*x(4)) + x(3)/x(2)) - m_delta;
	double alphaR = std::atan((x(3)-m_vehicleModelParameters(4)*x(4))/x(2));

	//Non linear Tire Model

	double Fzf = m_vehicleModelParameters(0)*m_vehicleModelParameters(2)*(m_vehicleModelParameters(4)/(m_vehicleModelParameters(4)+m_vehicleModelParameters(3)));
	double Fzr = m_vehicleModelParameters(0)*m_vehicleModelParameters(2)*(m_vehicleModelParameters(3)/(m_vehicleModelParameters(4)+m_vehicleModelParameters(3)));

	double Fyf = magicFormula(alphaF,Fzf,m_vehicleModelParameters(5));
	double Fyr = magicFormula(alphaR,Fzr,m_vehicleModelParameters(5));

	hx << x(0),
		  x(1),
		  x(2),
		  -Fyf*std::sin(m_delta)/m_vehicleModelParameters(0),
		  (Fyf*std::cos(m_delta)+Fyr)/m_vehicleModelParameters(0),
		  x(4),
		  x(5);

	return hx;
}

void Kalman::sendStates(uint32_t ukfStamp){

	//Pose
	opendlv::logic::sensation::Geolocation poseMessage;
  	std::lock_guard<std::mutex> lockSend(m_poseMutex); 
  	std::array<double,2> cartesianPos;
  	cartesianPos[0] = m_states(0);
  	cartesianPos[1] = m_states(1);
  	std::array<double,2> sendGPS = wgs84::fromCartesian(m_gpsReference, cartesianPos);
  	poseMessage.longitude(sendGPS[0]);
    poseMessage.latitude(sendGPS[1]);
    poseMessage.heading(static_cast<float>(m_states(5)));
    //std::chrono::system_clock::time_point tp = std::chrono::system_clock::now();
    cluon::data::TimeStamp sampleTime = m_geolocationReceivedTime;
    od4.send(poseMessage, sampleTime ,ukfStamp);

    //groundspeed
    opendlv::proxy::GroundSpeedReading gsMessage;
	std::lock_guard<std::mutex> lockGroundSpeed(m_groundSpeedMutex);
	gsMessage.groundSpeed(static_cast<float>(m_states(2)));
	sampleTime = m_groundSpeedReceivedTime;
    od4.send(gsMessage, sampleTime ,ukfStamp);

    //Yaw
    opendlv::proxy::AngularVelocityReading yawMessage;    
  	std::lock_guard<std::mutex> lockYaw(m_yawMutex);
  	yawMessage.angularVelocityZ(static_cast<float>(m_states(4)));
  	sampleTime = m_yawReceivedTime;
    od4.send(yawMessage, sampleTime ,ukfStamp);
}

void Kalman::initializeModule(){
	int validGpsMeasurements = 0;
	int validGroundspeedMeasurements = 0;
	int validYawMeasurements = 0;
	int validAccMeasurements = 0;
	int validRackMeasurements = 0;
	bool gpsReadyState = false;
	bool groundSpeedReadyState = false;
	bool yawReadyState = false;
	bool accReadyState = false;
	bool rackReadyState = false;

	double lastOdoX = 10000;
	double lastOdoY = 10000;
	float lastGroundspeed = 10000;
	double lastYaw = 10000;
	float lastAccX = 10000;
	float lastAccY = 10000;
	double lastRack = 10000;
	while(!m_readyState){
		bool sleep = true;
    	auto start = std::chrono::system_clock::now();

    	while(sleep)
    	{
      		auto now = std::chrono::system_clock::now();
      		auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(now - start);

      		if(elapsed.count() > 50*1000){
        		//std::cout << "Timed out" << std::endl;
        		sleep = false;
      		}
		}

		//Check pose

      	if( std::fabs(m_odometryData(0) - lastOdoX) > 0.001 && std::fabs(m_odometryData(1) - lastOdoY) > 0.001){
        	if(m_odometryData(0) < 1000 && m_odometryData(1) < 1000){
          		lastOdoX = m_odometryData(0);
          		lastOdoY = m_odometryData(1);
          		validGpsMeasurements++;
        	}
      	}

      	if(validGpsMeasurements > 30){
        	gpsReadyState = true;
        	std::cout << "GPS Ready .." << std::endl;
      	}


		//Check groundspeed
      	if(std::fabs(m_groundSpeed -lastGroundspeed) > 0.001){
      		lastGroundspeed = m_groundSpeed;
      		validGroundspeedMeasurements++;
      	}

      	if(validGroundspeedMeasurements > 30){
      		groundSpeedReadyState = true;
      		std::cout << "Groundspeed measurement Ready .." << std::endl;
      	}
		//Check Yaw
      	if(std::fabs(m_yawRate - lastYaw) > 0.001){
      		lastYaw = m_yawRate;
      		validYawMeasurements++;
      	}
      	if(validYawMeasurements > 30){
      		yawReadyState = true;
      		std::cout << "Yaw measurement Ready .." << std::endl;

      	}
		//Check Acc
      	if(std::fabs(m_acceleration(0) - lastAccX) > 0.001 && std::fabs(m_acceleration(1) - lastAccY) > 0.001){
      		lastAccX = m_acceleration(0);
      		lastAccY = m_acceleration(1);
      		validAccMeasurements++;
      	}
      	if(validAccMeasurements > 30){
      		accReadyState = true;
      		std::cout << "Acc measurement Ready .." << std::endl;
      	}
      	//Rack
      	if(std::fabs(m_delta - lastRack) > 0.001){

      		lastRack = m_delta;
      		validRackMeasurements++;
      	}
      	if(validRackMeasurements > 30){

      		rackReadyState = true;

      		std::cout << "Rack measurement Ready .." << std::endl;
      	}

      	if(gpsReadyState && groundSpeedReadyState && yawReadyState && accReadyState && rackReadyState){

      		m_readyState = true;

      		std::cout << "UKF Ready .." << std::endl;
      	}
	}
}

void Kalman::tearDown()
{
}
Kalman::~Kalman()
{

}
