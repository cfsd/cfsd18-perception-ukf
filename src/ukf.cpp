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
  , m_wheelMutex()
  , m_states()
  , m_Q()
  , m_R()
  , m_gpsReference()
  , m_vehicleModelParameters()
  , m_stateCovP()
  , m_wheelSpeed()
  , m_wheelIdLeft()
  , m_wheelIdRight()
{
  m_odometryData << 0,0,0;
  m_acceleration << 0,0,0;
  m_states = Eigen::MatrixXd::Zero(6,1);
  m_states << 0,0,0,0,0,0;
  m_vehicleModelParameters = Eigen::MatrixXd::Zero(7,1);
  m_Q = Eigen::MatrixXd::Zero(6,6); //Six states
  m_R = Eigen::MatrixXd::Zero(7,7); //Seven Measurements
  m_stateCovP = Eigen::MatrixXd::Zero(6,6); //Initialize P
  m_stateCovP << 1,0,0,0,0,0,
				 0,1,0,0,0,0,
				 0,0,0.1,0,0,0,
				 0,0,0,0.000001,0,0,
				 0,0,0,0,0.05,0,
				 0,0,0,0,0,0.1;
  m_wheelSpeed << 0,0;	

	//std::cout << m_states.transpose() << std::endl;
  setUp(commandlineArguments);
}


void Kalman::setUp(std::map<std::string, std::string> configuration)
{
	
	  m_gpsReference[0] = static_cast<double>(std::stod(configuration["refLatitude"]));
	  m_gpsReference[1] = static_cast<double>(std::stod(configuration["refLongitude"]));


      m_wheelIdLeft = static_cast<uint32_t>(std::stoi(configuration["wheelEncoderIdLeft"]));
      m_wheelIdRight = static_cast<uint32_t>(std::stoi(configuration["wheelEncoderIdRight"]));

	  double qX = static_cast<double>(std::stod(configuration["Qx"]));
	  double qY = static_cast<double>(std::stod(configuration["Qy"]));
	  double qVelX = static_cast<double>(std::stod(configuration["QvelX"]));
	  double qVelY = static_cast<double>(std::stod(configuration["QvelY"]));
	  double qYaw = static_cast<double>(std::stod(configuration["Qyaw"]));
	  double qHeading = static_cast<double>(std::stod(configuration["Qheading"]));

		m_Q << qX,0,0,0,0,0,
				0,qY,0,0,0,0,
				0,0,qVelX,0,0,0,
				0,0,0,qVelY,0,0,
				0,0,0,0,qYaw,0,
				0,0,0,0,0,qHeading;
	  

	//std::cout << "Q: " << m_Q << std::endl;
	  m_rX = static_cast<double>(std::stod(configuration["Rx"]));
	  m_rY = static_cast<double>(std::stod(configuration["Ry"]));
	  m_rVelX = static_cast<double>(std::stod(configuration["RvelX"]));
	  m_rAccX = static_cast<double>(std::stod(configuration["RaccX"]));
	  m_rAccY = static_cast<double>(std::stod(configuration["RaccY"]));
	  m_rYaw = static_cast<double>(std::stod(configuration["Ryaw"]));
	  m_rHeading = static_cast<double>(std::stod(configuration["Rheading"]));	 
	  m_R << m_rX,0,0,0,0,0,0,
	  		 0,m_rY,0,0,0,0,0,
	  		 0,0,m_rVelX,0,0,0,0,
	  		 0,0,0,m_rAccX,0,0,0,
	  		 0,0,0,0,m_rAccY,0,0,
	  		 0,0,0,0,0,m_rYaw,0,
	  		 0,0,0,0,0,0,m_rHeading;
	  
	//std::cout << "R: " << m_R << std::endl;
	  double const vM = static_cast<double>(std::stod(configuration["m"]));
	  double const vIz = static_cast<double>(std::stod(configuration["Iz"]));
	  double const vG = static_cast<double>(std::stod(configuration["g"]));
	  double const vL = static_cast<double>(std::stod(configuration["l"]));
	  double const vLf = static_cast<double>(std::stod(configuration["lf"]));
	  double const vLr = static_cast<double>(std::stod(configuration["lr"]));
	  double const vMu = static_cast<double>(std::stod(configuration["mu"]));

	m_vehicleModelParameters << vM,vIz,vG,vL,vLf,vLr,vMu;


	std::cout << "vmp: " << m_vehicleModelParameters.transpose() << std::endl;
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
   if(!m_filterInit && !m_zeroVelState){
  	Eigen::Vector2d tempPos;
  	tempPos << WGS84ReadingTemp[0],WGS84ReadingTemp[1];
  	m_positionVec.push_back(tempPos);
  }
  //opendlv::data::environment::WGS84Coordinate gpsCurrent = opendlv::data::environment::WGS84Coordinate(latitude, longitude);
  //opendlv::data::environment::Point3 gpsTransform = m_gpsReference.transform(gpsCurrent);
  double heading = calculateHeading(WGS84Reading[0],WGS84Reading[0]);
  m_odometryData << WGS84Reading[0],
                    WGS84Reading[1],
                    heading;            
  
  //std::cout << std::fixed << std::setprecision(9) << m_odometryData(0) << " | " << m_odometryData(1) << std::endl;					     
}

void Kalman::nextGroundSpeed(cluon::data::Envelope data){

  std::lock_guard<std::mutex> lockGroundSpeed(m_groundSpeedMutex);
  
  	auto groundSpeed = cluon::extractMessage<opendlv::proxy::GroundSpeedReading>(std::move(data));
  	uint32_t stamp = data.senderStamp();
  	if(stamp == m_wheelIdLeft){
  		m_gotLeft = true;
  		m_wheelSpeed(0) = groundSpeed.groundSpeed();
  	}else if(stamp == m_wheelIdRight){
  		m_gotRight = true;
  		m_wheelSpeed(1) = groundSpeed.groundSpeed();
  	}

  	if(m_gotLeft && m_gotRight){
  		m_gotLeft = false;
  		m_gotRight = false;
  		m_groundSpeed = static_cast<double>((m_wheelSpeed(0) + m_wheelSpeed(1))/2);
  		m_currentVelMean += m_groundSpeed;
  		m_velMeasurementCount++;
		if(!m_readyState){
			m_validGroundSpeedMeasurements++;
		}
		if(!m_filterInit && !m_zeroVelState){

			if(m_groundSpeed > m_maxSpeed){ m_maxSpeed = m_groundSpeed;}
		}
  	}	
   //std::cout << "Yaw in message: " << m_yawRate << std::endl;
}
void Kalman::nextYawRate(cluon::data::Envelope data){

  std::lock_guard<std::mutex> lockYaw(m_yawMutex);
  auto yawRate = cluon::extractMessage<opendlv::proxy::AngularVelocityReading>(std::move(data));
  m_yawRate = static_cast<double>(yawRate.angularVelocityZ());
   m_yawReceivedTime = data.sampleTimeStamp();
   m_currentYawMean += m_yawRate;
   m_yawMeasurementCount++;
   //std::cout << "Yaw in message: " << m_yawRate << std::endl;
}

void Kalman::nextAcceleration(cluon::data::Envelope data){
  std::lock_guard<std::mutex> lockAcc(m_accMutex);
  auto acceleration = cluon::extractMessage<opendlv::proxy::AccelerationReading>(std::move(data));
  m_acceleration(0) = static_cast<double>(acceleration.accelerationX());
  m_acceleration(1) = static_cast<double>(acceleration.accelerationY());
  m_acceleration(2) = static_cast<double>(acceleration.accelerationZ());
  m_accReceivedTime = data.sampleTimeStamp();

  m_currentAccMean += m_acceleration(0);
  m_accMeasurementCount++;
}


void Kalman::nextRack(cluon::data::Envelope data){	
	std::lock_guard<std::mutex> lockDelta(m_deltaMutex);

  auto rackTravel = cluon::extractMessage<opendlv::proxy::GroundSteeringReading>(std::move(data));
  float rT = rackTravel.groundSteering();
  m_rackReceivedTime = data.sampleTimeStamp();
  m_delta = rackTravelToFrontWheelSteering(rT);
  m_validRackMeasurements++;
}

void Kalman::setStateMachineStatus(cluon::data::Envelope data){
  std::lock_guard<std::mutex> lockStateMachine(m_stateMachineMutex);
  auto machineStatus = cluon::extractMessage<opendlv::proxy::SwitchStateReading>(std::move(data));
  int state = machineStatus.state();
  if(state == 2){
    m_readyStateMachine = true;
  }
  
}

double Kalman::calculateHeading(double x, double y){

	double deltaX = m_odometryData(0)-x;
	double deltaY = m_odometryData(1)-y;
	double heading = std::atan2(deltaY,deltaX);
    heading = heading-PI;
    heading = (heading > PI)?(heading-2*PI):(heading);
	heading = (heading < -PI)?(heading+2*PI):(heading);

	return heading;

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

 	int const n = 6; //Amount of states  alpha = 0.5, beta = 2 kappa = 3-n
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
	//std::cout << "weights: " << Wmc << std::endl;
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

	//std::cout << "before prediction: " << m_states.transpose() << std::endl;
	int const n = 6;
	Eigen::MatrixXd x = m_states;
	//std::cout << "before prediction: " << x.transpose() << std::endl;
	Eigen::MatrixXd Wmc = UKFWeights();
	Eigen::MatrixXd SP = sigmaPoints(x);
	//std::cout << "SP: " << SP << std::endl;
	Eigen::MatrixXd x_hat = Eigen::MatrixXd::Zero(n,1);
	Eigen::MatrixXd sigmaPoint, sigmaStates;
	//Calculate Mean

	for(int i = 0; i < 2*n+1; i++){

		sigmaPoint = SP.col(i);
		sigmaStates = vehicleModel(sigmaPoint);
		x_hat = x_hat + sigmaStates*Wmc(0,i);

	}
	//std::cout << "pred x: " << x_hat.transpose() << std::endl;
	//Check Heading inside pi/2 to -pi/2

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

	//std::cout << "after prediction: " << m_states.transpose() << std::endl;
}

void Kalman::UKFUpdate()
{


	//std::cout << "before update: " << m_states.transpose() << std::endl;

		Eigen::MatrixXd x = m_states;
		Eigen::MatrixXd Wmc = UKFWeights();
		Eigen::MatrixXd SP = sigmaPoints(x);

	//std::cout << "SP: " << SP << std::endl;
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

	//std::cout << "measurements" << y.transpose() << std::endl;
  		if(m_zeroVelState){
  			y << m_odometryData(0), 
		 		 m_odometryData(1),
		 		 0.0,
		 		 0.0,
		 		 0.0,
		 		 0.0,
		 		 m_odometryData(2);
  		}else{
			y << m_odometryData(0), 
		 		 m_odometryData(1),
		 		 m_groundSpeed,
		 		 m_acceleration(0),
		 		 m_acceleration(1),
		 		 m_yawRate,
		 		 m_odometryData(2);
		}	
	}

	//State update
	x = x + Pxy*S.inverse()*(y-y_hat);

	double heading = x(5);
	heading = (heading > PI)?(heading-2*PI):(heading);
	heading = (heading < -PI)?(heading+2*PI):(heading);
	x(5) = heading;
	//Check heading so its between pi/2 and -pi/2
	m_stateCovP = m_stateCovP - Pxy*S.inverse()*Pxy.transpose();
	//Dirty trick to keep numerical stability
	m_stateCovP = (m_stateCovP + m_stateCovP.transpose())/2;
	
	m_states =  x;

	//std::cout << "after update: " << m_states.transpose() << std::endl;
}


Eigen::MatrixXd Kalman::vehicleModel(Eigen::MatrixXd x)
{
	std::lock_guard<std::mutex> lockDelta(m_deltaMutex);
	
	if(x(2) < 0.0001){

		x(2) = 0.01;
	}

	Eigen::MatrixXd xdot = Eigen::MatrixXd::Zero(x.rows(),1);

	double alphaF = 0.0;
	double alphaR = 0.0;
	if(!m_zeroVelState){
		alphaF = std::atan( (m_vehicleModelParameters(4)*x(4) + x(3) )/x(2)) - m_delta;
		alphaR = std::atan( (x(3)-m_vehicleModelParameters(5)*x(4))/x(2));
		//double ce = m_vehicleModelParameters(4)*x(4) + x(3);

		//Non linear Tire Model

		//double Fzf = m_vehicleModelParameters(0)*m_vehicleModelParameters(2)*(m_vehicleModelParameters(4)/(m_vehicleModelParameters(4)+m_vehicleModelParameters(3)));
		//double Fzr = m_vehicleModelParameters(0)*m_vehicleModelParameters(2)*(m_vehicleModelParameters(3)/(m_vehicleModelParameters(4)+m_vehicleModelParameters(3)));

    	alphaF = (std::fabs(alphaF) > 0.002)?(0):(alphaF);
    	alphaR = (std::fabs(alphaR) > 0.002)?(0):(alphaR);
	}
	
	//std::cout << "from vehicle model: " << alphaF << " | " << alphaR << " | " << ce << std::endl;
	double Fyf = 20000*alphaF; //magicFormula(alphaF,Fzf,m_vehicleModelParameters(5));
	double Fyr =  20000*alphaR;//magicFormula(alphaR,Fzr,m_vehicleModelParameters(5));


	xdot << x(2),
			x(3),
			-Fyf*std::sin(m_delta)/m_vehicleModelParameters(0) + x(4)*x(3),
			(Fyf*std::cos(m_delta)+Fyr)/m_vehicleModelParameters(0) - x(4)*x(2),
			(m_vehicleModelParameters(4)*Fyf*std::cos(m_delta)-m_vehicleModelParameters(5)*Fyr)/m_vehicleModelParameters(1),
			x(4);

	//Update xdot with timedifference
	double timeElapsed;
	cluon::data::TimeStamp currentTime = cluon::time::now();
	double tm = 0.1;
	//Position
    timeElapsed = fabs(static_cast<double>(cluon::time::deltaInMicroseconds(m_groundSpeedReceivedTime,currentTime)));
    xdot(0) = xdot(0)*tm;
    xdot(1) = xdot(1)*tm;

    //Velocity    
    timeElapsed = fabs(static_cast<double>(cluon::time::deltaInMicroseconds(m_accReceivedTime,currentTime)));
    xdot(2) = xdot(2)*tm;
    xdot(3) = xdot(3)*tm;

    //Yaw   
    timeElapsed = fabs(static_cast<double>(cluon::time::deltaInMicroseconds(m_yawReceivedTime,currentTime)));
    xdot(4) = xdot(4)*tm;

    //Heading
    timeElapsed = fabs(static_cast<double>(cluon::time::deltaInMicroseconds(m_geolocationReceivedTime,currentTime)));
    xdot(5) = xdot(5)*tm;
	//std::cout << "TM: " << timeElapsed << std::endl;
	timeElapsed = timeElapsed;
    //Update states
	Eigen::MatrixXd fx = x + xdot;

	return fx;
}

Eigen::MatrixXd Kalman::measurementModel(Eigen::MatrixXd x)
{

	std::lock_guard<std::mutex> lockDelta(m_deltaMutex);
	
	Eigen::MatrixXd hx = Eigen::MatrixXd::Zero(7,1);

	double alphaF = 0;
	double alphaR = 0;
	if(!m_zeroVelState){
		alphaF = std::atan( (m_vehicleModelParameters(4)*x(4) + x(3) )/x(2)) - m_delta;
		alphaR = std::atan( (x(3)-m_vehicleModelParameters(5)*x(4))/x(2));
		//double ce = m_vehicleModelParameters(4)*x(4) + x(3);

		//Non linear Tire Model

		//double Fzf = m_vehicleModelParameters(0)*m_vehicleModelParameters(2)*(m_vehicleModelParameters(4)/(m_vehicleModelParameters(4)+m_vehicleModelParameters(3)));
		//double Fzr = m_vehicleModelParameters(0)*m_vehicleModelParameters(2)*(m_vehicleModelParameters(3)/(m_vehicleModelParameters(4)+m_vehicleModelParameters(3)));

    	alphaF = (std::fabs(alphaF) > 0.002)?(0):(alphaF);
    	alphaR = (std::fabs(alphaR) > 0.002)?(0):(alphaR);
	}
	//std::cout << "from measurement model: " << alphaF << " | " << alphaR <<" | " << ce << std::endl;
	double Fyf = 20000*alphaF; //magicFormula(alphaF,Fzf,m_vehicleModelParameters(5));
	double Fyr =  20000*alphaR;//magicFormula(alphaR,Fzr,m_vehicleModelParameters(5));
	
	hx << x(0),
		  x(1),
		  x(2),
		  0 , //-Fyf*std::sin(m_delta)/m_vehicleModelParameters(0)
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

	//std::cout << m_states << std::endl;
}

void Kalman::initializeModule(){
	int validGpsMeasurements = 0;
	//int validGroundspeedMeasurements = 0;
	int validYawMeasurements = 0;
	int validAccMeasurements = 0;
	//int validRackMeasurements = 0;
	bool gpsReadyState = false;
	bool groundSpeedReadyState = false;
	bool yawReadyState = false;
	bool accReadyState = false;
	bool rackReadyState = false;

	double lastOdoX = 10000;
	double lastOdoY = 10000;
	//double lastGroundspeed = 10000;
	double lastYaw = 10000;
	double lastAccX = 10000;
	double lastAccY = 10000;
	//double lastRack = 10000;
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

      	if(!gpsReadyState && validGpsMeasurements > 30){
        	gpsReadyState = true;
        	std::cout << "GPS Ready .." << std::endl;
      	}


		//Check groundspeed
      	/*if(std::fabs(m_groundSpeed - lastGroundspeed) > 0.001){
      		lastGroundspeed = m_groundSpeed;
      		validGroundspeedMeasurements++;
      	}*/

      	if(!groundSpeedReadyState && m_validGroundSpeedMeasurements > 30){
      		groundSpeedReadyState = true;
      		std::cout << "Groundspeed measurement Ready .." << std::endl;
      	}
		//Check Yaw
      	if(std::fabs(m_yawRate - lastYaw) > 0.000001){
      		lastYaw = m_yawRate;
      		validYawMeasurements++;
      	}
      	if(!yawReadyState && validYawMeasurements > 30){
      		yawReadyState = true;
      		std::cout << "Yaw measurement Ready .." << std::endl;

      	}
		//Check Acc
      	if(std::fabs(m_acceleration(0) - lastAccX) > 0.001 && std::fabs(m_acceleration(1) - lastAccY) > 0.001){
      		lastAccX = m_acceleration(0);
      		lastAccY = m_acceleration(1);
      		validAccMeasurements++;
      	}
      	if(!accReadyState && validAccMeasurements > 30){
      		accReadyState = true;
      		std::cout << "Acc measurement Ready .." << std::endl;
      	}
      	//Rack
      	/*if(std::fabs(m_delta - lastRack) > 0.001){

      		lastRack = m_delta;
      		validRackMeasurements++;
      	}*/
      	if(!rackReadyState && m_validRackMeasurements > 30){

      		rackReadyState = true;
      		std::cout << "Rack measurement Ready .." << std::endl;
      	}

      	if(gpsReadyState && groundSpeedReadyState && yawReadyState && accReadyState && rackReadyState){

      		m_readyState = true;

      		std::cout << "UKF Ready .." << std::endl;
      	}
	}
}

void Kalman::checkVehicleState(){

	//Check if the filter is gonna update with no velocity

	bool velCheck = false;
	bool yawCheck = false;
	bool accCheck = false;
	bool reset = false;

  	std::lock_guard<std::mutex> lockGroundSpeed(m_groundSpeedMutex);

  	if(!reset){
  		if(m_velMeasurementCount > 25 && m_accMeasurementCount > 25 && m_yawMeasurementCount > 25){

  			reset = true;
  		}	
  	}
	if(m_velMeasurementCount > 12 && m_accMeasurementCount > 12 && m_yawMeasurementCount > 12 && !reset){

		if(m_currentVelMean/m_velMeasurementCount < 0.1){
			velCheck = true;
		}

		if(m_currentYawMean/m_yawMeasurementCount < 0.05){

			yawCheck = true;
		}

		if(m_currentAccMean/m_accMeasurementCount < 0.03){
			accCheck = true;
		}

		if(velCheck && yawCheck && accCheck){
			m_zeroVelState = true;
			std::cout << "Zero Velocity Update Mode ..." << std::endl;
		}else{
			m_zeroVelState = false;

			std::cout << "Motion Update Mode ..." << std::endl;
		}
	}
	

	if(m_zeroVelState){
		m_velMeasurementCount = 0;
		m_currentVelMean = 0;
		m_accMeasurementCount = 0;
		m_currentAccMean = 0;
		m_yawMeasurementCount= 0;
		m_currentYawMean = 0;

		m_R <<  10,0,0,0,0,0,0,
	  			0,10,0,0,0,0,0,
	  			0,0,0.01,0,0,0,0,
	  			0,0,0,0.01,0,0,0,
	  		 	0,0,0,0,0.01,0,0,
	  		 	0,0,0,0,0,0.01,0,
	  		 	0,0,0,0,0,0,0.01;

		
	}else{

		m_velMeasurementCount = 0;
		m_currentVelMean = 0;
		m_accMeasurementCount = 0;
		m_currentAccMean = 0;
		m_yawMeasurementCount= 0;
		m_currentYawMean = 0;
	  	m_R << m_rX,0,0,0,0,0,0,
	  	   	 0,m_rY,0,0,0,0,0,
	  		 0,0,m_rVelX,0,0,0,0,
	  		 0,0,0,m_rAccX,0,0,0,
	  		 0,0,0,0,m_rAccY,0,0,
	  		 0,0,0,0,0,m_rYaw,0,
	  		 0,0,0,0,0,0,m_rHeading;
	}
	//Check last 1 seconds of acceleration measurements so there is a small or negative acceleration

	//Check last 1 seconds of velocities that it is close to zero



}

bool Kalman::getFilterInitState(){

	return m_filterInit;

}
void Kalman::filterInitialization(){

	if(!m_zeroVelState && !m_filterInit){
		uint32_t maxIndex = m_positionVec.size()-1;
		double distance = std::sqrt( (m_positionVec[0](1)-m_positionVec[maxIndex](1))*(m_positionVec[0](1)-m_positionVec[maxIndex](1)) + (m_positionVec[0](1) - m_positionVec[maxIndex](1)) * ( m_positionVec[0](1)-m_positionVec[maxIndex](1)) );
		if(m_maxSpeed > 1 && distance > 5){			
			m_filterInit = true;
			std::cout << "UKF Filtering Initialized ..." << std::endl;
			m_positionVec.clear();
		}
	}
	
}

void Kalman::tearDown()
{
}
Kalman::~Kalman()
{

}
