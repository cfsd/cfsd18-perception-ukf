/*
 * Copyright (C) 2018  Christian Berger
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"
#include "ukf.hpp"
#include <Eigen/Dense>

#include <cstdint>
#include <tuple>
#include <utility>
#include <iostream>
#include <string>
#include <thread>

int32_t main(int32_t argc, char **argv) {
  int32_t retCode{0};
  std::map<std::string, std::string> commandlineArguments = cluon::getCommandlineArguments(argc, argv);
  if (commandlineArguments.size()<10) {
    std::cerr << argv[0] << " is a slam implementation for the CFSD18 project." << std::endl;
    std::cerr << "Usage:   " << argv[0] << " --cid=<OpenDaVINCI session> [--id=<Identifier in case of simulated units>] [--verbose] [Module specific parameters....]" << std::endl;
    std::cerr << "Example: " << argv[0] << "--cid=111 --id=120 --detectConeId=118 --estimationId=114 --gatheringTimeMs=10 --sameConeThreshold=1.2 --refLatitude=48.123141 --refLongitude=12.34534 --timeBetweenKeyframes=0.5 --coneMappingThreshold=50 --conesPerPacket=20" <<  std::endl;
    retCode = 1;
  } else {
    //uint32_t const ID{(commandlineArguments["id"].size() != 0) ? static_cast<uint32_t>(std::stoi(commandlineArguments["id"])) : 0};
    bool const VERBOSE{commandlineArguments.count("verbose") != 0};
    (void)VERBOSE;
    // Interface to a running OpenDaVINCI session (ignoring any incoming Envelopes).
    cluon::data::Envelope data;
    //std::shared_ptr<Slam> slammer = std::shared_ptr<Slam>(new Slam(10));
    cluon::OD4Session od4{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};
    cluon::OD4Session od4Dan{static_cast<uint16_t>(std::stoi(commandlineArguments["cidDan"]))};

    Kalman kalman(commandlineArguments,od4);

    uint32_t estimationStamp = static_cast<uint32_t>(std::stoi(commandlineArguments["estimationId"]));
    uint32_t estimationStampRaw = static_cast<uint32_t>(std::stoi(commandlineArguments["estimationIdRaw"]));
    uint32_t ukfStamp = static_cast<uint32_t>(std::stoi(commandlineArguments["id"])); 
    uint32_t stateMachineStamp = static_cast<uint32_t>(std::stoi(commandlineArguments["stateMachineId"]));
    uint32_t rackStamp = static_cast<uint32_t>(std::stoi(commandlineArguments["rackId"]));
    uint32_t wheelIdLeft = static_cast<uint32_t>(std::stoi(commandlineArguments["wheelEncoderIdLeft"]));
    uint32_t wheelIdRight = static_cast<uint32_t>(std::stoi(commandlineArguments["wheelEncoderIdRight"]));
    uint32_t slamStamp = static_cast<uint32_t>(std::stoi(commandlineArguments["slamId"]));

    auto poseEnvelope{[&ukf = kalman,senderStamp = estimationStampRaw, senderEkf = estimationStamp, senderSlam = slamStamp](cluon::data::Envelope &&envelope)
      {
        
        if(envelope.senderStamp() == senderStamp) {
          ukf.nextPose(envelope);
        }else if(envelope.senderStamp() == senderEkf){
          
            ukf.nextHeading(envelope);
          if(!ukf.getEllipseState()){  
            ukf.nextEllipsePose(envelope);
          }
            
        }else if(envelope.senderStamp() == senderSlam){
          ukf.nextSlamPose(envelope);
        }
      } 
    };

    auto yawRateEnvelope{[&ukf = kalman, senderStamp = estimationStamp](cluon::data::Envelope &&envelope)
      {
        if(envelope.senderStamp() == senderStamp){
          ukf.nextYawRate(envelope);
        }
      }
    };

    auto groundSpeedEnvelope{[&ukf = kalman, senderStampSBG = estimationStamp, senderStampWheelL = wheelIdLeft,senderStampWheelR = wheelIdRight](cluon::data::Envelope &&envelope)
      {
        if(envelope.senderStamp() == senderStampSBG || envelope.senderStamp() == senderStampWheelR || envelope.senderStamp() == senderStampWheelL){
          ukf.nextGroundSpeed(envelope);
        }
      }
    };

    auto accelerationEnvelope{[&ukf = kalman, senderStamp = estimationStamp](cluon::data::Envelope &&envelope)
      {
        if(envelope.senderStamp() == senderStamp){
          ukf.nextAcceleration(envelope);
        }
      }
    };
    auto rackEnvelope{[&ukf = kalman, senderStamp = rackStamp](cluon::data::Envelope &&envelope)
      {
        if(envelope.senderStamp() == senderStamp){
          ukf.nextRack(envelope);
        }
      }
    };
    auto stateMachineStatusEnvelope{[&ukf = kalman, senderStamp = stateMachineStamp](cluon::data::Envelope &&envelope)
      {
        if(envelope.senderStamp() == senderStamp){
          
          ukf.setStateMachineStatus(envelope);
        }
      }
    };
    od4.dataTrigger(opendlv::logic::sensation::Geolocation::ID(),poseEnvelope);
    od4.dataTrigger(opendlv::proxy::AngularVelocityReading::ID(),yawRateEnvelope);
    od4.dataTrigger(opendlv::proxy::GroundSpeedReading::ID(),groundSpeedEnvelope);
    od4Dan.dataTrigger(opendlv::proxy::GroundSpeedReading::ID(),groundSpeedEnvelope);
    od4.dataTrigger(opendlv::proxy::AccelerationReading::ID(),accelerationEnvelope);
    od4Dan.dataTrigger(opendlv::proxy::GroundSteeringReading::ID(),rackEnvelope);
    od4.dataTrigger(opendlv::proxy::SwitchStateReading::ID(),stateMachineStatusEnvelope);
    

    // Just sleep as this microservice is data driven.
    using namespace std::literals::chrono_literals;
    bool readyState = false;
    int checkZeroVelocityUpdate = 0;
    while (od4.isRunning() && od4Dan.isRunning()) {

      if(readyState){
        opendlv::system::SignalStatusMessage ssm;
        ssm.code(1);
        cluon::data::TimeStamp sampleTime = cluon::time::now();
        od4.send(ssm, sampleTime ,ukfStamp);
        if(kalman.getStateMachineStatus()){
          
          if(checkZeroVelocityUpdate > 20){
            kalman.checkVehicleState();
            checkZeroVelocityUpdate = 0;
          }
            kalman.UKFPrediction();          
            kalman.UKFUpdate();
            
          if(kalman.getFilterInitState()){
            kalman.sendStates(ukfStamp);
          }else{
            kalman.filterInitialization();
          }
        }
      }else{
        kalman.initializeModule();
        readyState = kalman.getModuleState();
      }

      checkZeroVelocityUpdate++;
      std::this_thread::sleep_for(0.05s);
      std::chrono::system_clock::time_point tp;
    }
  }
  return retCode;
}


