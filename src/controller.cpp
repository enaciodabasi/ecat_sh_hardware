/**
 * @file controller.cpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2024-10-25
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "ecat_sh_hardware/controller.hpp"
#include "ecat_sh_hardware/shared_memory_handler.hpp"

using namespace std::chrono_literals;

VelocityLimiter::VelocityLimiter()
{
}

void VelocityLimiter::limit(double& command, double current_value, double dt)
{
  // Limit jerk
  // Limit acceleration

  const double requestedAcc = (current_value - command);
  const double possibleAcceleration = std::clamp(requestedAcc, min_acc * dt, max_acc * dt);
  command += possibleAcceleration;
  // Limit velocity

  command = std::clamp(command, min_vel, max_vel);
}

CIA402_State deriveState(uint16_t status_word)
{
  if ((status_word & 0b01001111) == 0b00000000)
  {
    return CIA402_State::NOT_READY_TO_SWITCH_ON;
  }
  else if ((status_word & 0b01001111) == 0b01000000)
  {
    return CIA402_State::SWITCH_ON_DISABLED;
  }
  else if ((status_word & 0b01101111) == 0b00100001)
  {
    return CIA402_State::READY_TO_SWITCH_ON;
  }
  else if ((status_word & 0b01101111) == 0b00100011)
  {
    return CIA402_State::SWITCH_ON;
  }
  else if ((status_word & 0b01101111) == 0b00100111)
  {
    return CIA402_State::OPERATION_ENABLED;
  }
  else if ((status_word & 0b01101111) == 0b00000111)
  {
    return CIA402_State::QUICK_STOP_ACTIVE;
  }
  else if ((status_word & 0b01001111) == 0b00001111)
  {
    return CIA402_State::FAULT_REACTION_ACTIVE;
  }
  else if ((status_word & 0b01001111) == 0b00001000)
  {
    return CIA402_State::FAULT;
  }
  return CIA402_State::UNKNOWN_STATE;
}

bool run = true;
void sigInHandler(int signal)
{
  run = false;
}

uint16_t transitionToState(CIA402_State state, uint16_t control_word)
{
  switch (state)
  {
    case CIA402_State::START:
      return control_word;
    case CIA402_State::NOT_READY_TO_SWITCH_ON:
      return control_word;
    case CIA402_State::SWITCH_ON_DISABLED:
      return (control_word & 0b01111110) | 0b00000110;
    case CIA402_State::READY_TO_SWITCH_ON:
      return (control_word & 0b01110111) | 0b00000111;
    case CIA402_State::SWITCH_ON:
      return (control_word & 0b01111111) | 0b00001111;
    case CIA402_State::OPERATION_ENABLED:
      return control_word;
    case CIA402_State::QUICK_STOP_ACTIVE:
      return (control_word & 0b01111111) | 0b00001111;
    case CIA402_State::FAULT_REACTION_ACTIVE:
      return control_word;
    case CIA402_State::FAULT:
      return (control_word & 0b11111111) | 0b10000000;
    default:
      break;
  }
  return control_word;
}

int main(int argc, char** argv)
{
  SharedMemoryHandler shMemHandler;

  if (shMemHandler.init(ShMode::User) != ecat_sh_hardware::Error::NoError)
  {
    return 1;
  }


  shared_obj_info::EthercatDataObject rightWheelData;
  DriverStateHandler rightWheelStateHandler;
  shared_obj_info::EthercatDataObject leftWheelData;
  DriverStateHandler leftWheelStateHandler;

  std::shared_ptr<VelocityCommand> velCommandPtr = std::make_shared<VelocityCommand>();
  std::shared_ptr<RosData> rosDataPtr = std::make_shared<RosData>();
  std::mutex rosSyncMutex;
  std::atomic<bool> shutdownRequested = false;

  Odometry odomHandler;

  std::thread rosThread(&ros_communication, std::ref(shutdownRequested), std::ref(rosSyncMutex),
                        std::ref(velCommandPtr), std::ref(rosDataPtr));
  rosThread.detach();

  const sched_param schedParam{ .sched_priority = 60 };
  if (sched_setscheduler(0, SCHED_FIFO, &schedParam) != 0)
  {
    std::cout << "Could not set scheduler policy." << std::endl;
    return 1;
  }

  signal(SIGINT, sigInHandler);

  while (run)
  {
    std::future<Odometry> odomFuture;

    //timepoint currentTime = std::chrono::high_resolution_clock::now();
    if(shMemHandler.tryLockSem() == 0)
    {
    auto dataPackageOpt = shMemHandler.getEcDataObject();
    shMemHandler.freeSem();
    if (dataPackageOpt)
    {
      const std::vector<shared_obj_info::EthercatDataObject> dataPackage = dataPackageOpt.value();
      if (!dataPackage.empty())
      {
        // dataPackage[0] := right wheel
        rightWheelData = dataPackage[0];
        rightWheelData.operation_mode = 0x09;
        if (CIA402_State rightWheelCurrentState = deriveState(rightWheelData.status_word);
            rightWheelCurrentState != CIA402_State::OPERATION_ENABLED)
        {
          uint16_t newControlWord =
              transitionToState(rightWheelCurrentState, rightWheelStateHandler.previousControlWord);
          rightWheelData.control_word = newControlWord;
          std::cout << "Right new control word: " << newControlWord << std::endl;
          rightWheelStateHandler.previousControlWord = newControlWord;
          rightWheelStateHandler.previousState = rightWheelCurrentState;
          rightWheelStateHandler.isOperational = false;
        }
        else
        {
          rightWheelStateHandler.isOperational = true;
        }

        // dataPackage[1] := left wheel
        leftWheelData = dataPackage[1];
        leftWheelData.operation_mode = 0x09;
        if (CIA402_State leftWheelCurrentState = deriveState(leftWheelData.status_word);
            leftWheelCurrentState != CIA402_State::OPERATION_ENABLED)
        {
          uint16_t newControlWord = transitionToState(leftWheelCurrentState, leftWheelStateHandler.previousControlWord);
          leftWheelData.control_word = newControlWord;
          std::cout << "Left new control word: " << newControlWord << std::endl;
          leftWheelStateHandler.previousControlWord = newControlWord;
          leftWheelStateHandler.previousState = leftWheelCurrentState;
          leftWheelStateHandler.isOperational = false;
        }
        else if (CIA402_State leftWheelCurrentState = deriveState(leftWheelData.status_word);
                 leftWheelCurrentState == CIA402_State::OPERATION_ENABLED)
        {
          leftWheelStateHandler.isOperational = true;
        }

/*         odomFuture =
            std::async(std::launch::async, &Odometry::update, odomHandler, (double)rightWheelData.current_velocity,
                       (double)leftWheelData.current_velocity, currentTime); */
      }
    }

    bool driversEnabled = (rightWheelStateHandler.isOperational && leftWheelStateHandler.isOperational);

/*     rosSyncMutex.lock();
    VelocityCommand velCmd = *velCommandPtr;
    rosSyncMutex.unlock(); */

    // If all slaves are operational, write commands:
    if (driversEnabled)
    {
      // Get data from ROS
      
/*       auto wheelVelocities = getWheelVelocityFromRobotCmd(velCmd.linear, velCmd.angular);     
 *//*       rightWheelData.target_velocity = jointVelocityToMotorVelocity(wheelVelocities.first);
      leftWheelData.target_velocity = jointVelocityToMotorVelocity(wheelVelocities.second); */
            rightWheelData.target_velocity = 75;
      leftWheelData.target_velocity = 75;
      //std::cout << "Target: " << leftWheelData.target_velocity << std::endl;
    }
    else
    {
      rightWheelData.target_velocity = 0;
      leftWheelData.target_velocity = 0;
    }
    
    shMemHandler.lockSem();
    shMemHandler.sendEcDataObject({ rightWheelData, leftWheelData });
    bool couldFreeSem = shMemHandler.freeSem();
    }
    else
    {
      std::cout << "COuld not get lock on sem " << std::endl;
      printf("%s\n", strerror(errno));
    }
/*     odomFuture.wait();
    rosDataPtr->odometry = odomFuture.get(); */


    std::this_thread::sleep_for(2ms);
  }
  shutdownRequested = true;
}