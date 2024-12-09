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
  std::expected<shm_handler::SharedMemoryHandler<shared_obj_info::EthercatDataObject, 2>, shm_handler::Error>
      sharedMemoryHandlerInit = shm_handler::SharedMemoryHandler<shared_obj_info::EthercatDataObject, 2>::create(
          shared_obj_info::SHARED_MEMORY_SEG_NAME, shared_obj_info::ETHERCAT_DATA_SEM_NAME, shm_handler::Mode::OPEN);

  if (!sharedMemoryHandlerInit.has_value())
  {
    std::cout << "Could not create Shared Memory Handler: " << shm_handler::ErrorMap.at(sharedMemoryHandlerInit.error())
              << std::endl;
    return 10;
  }

  auto sharedMemoryHandler = std::move(sharedMemoryHandlerInit.value());

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

    timepoint currentTime = std::chrono::time_point_cast<std::chrono::duration<double, std::ratio<1>>>(std::chrono::system_clock::now());
    if (sharedMemoryHandler.lock())
    {

      // dataPackage[0] := right wheel
      auto& rightWheelShData = sharedMemoryHandler.getDataPtr()[0];

      rightWheelShData.operation_mode = 0x09;
      if (CIA402_State rightWheelCurrentState = deriveState(rightWheelShData.status_word);
          rightWheelCurrentState != CIA402_State::OPERATION_ENABLED)
      {
        
        uint16_t newControlWord = transitionToState(rightWheelCurrentState, rightWheelStateHandler.previousControlWord);
        if((rightWheelShData.status_word & 0x0008) && !(rightWheelShData.status_word & 0x0007)) {
          newControlWord = 0x0080;
        }

        
        rightWheelShData.control_word = newControlWord;
        rightWheelStateHandler.previousControlWord = newControlWord;
        rightWheelStateHandler.previousState = rightWheelCurrentState;
        rightWheelStateHandler.isOperational = false;
      }
      else
      {
        rightWheelStateHandler.isOperational = true;
      }

      // dataPackage[1] := left wheel
      auto& leftWheelShData = sharedMemoryHandler.getDataPtr()[1];
      leftWheelShData.operation_mode = 0x09;
      if (CIA402_State leftWheelCurrentState = deriveState(leftWheelShData.status_word);
          leftWheelCurrentState != CIA402_State::OPERATION_ENABLED)
      {
        uint16_t newControlWord = transitionToState(leftWheelCurrentState, leftWheelStateHandler.previousControlWord);
        if((leftWheelShData.status_word & 0x0008) && !(leftWheelShData.status_word & 0x0007)) {
          newControlWord = 0x0080;
        }
        leftWheelShData.control_word = newControlWord;
        leftWheelStateHandler.previousControlWord = newControlWord;
        leftWheelStateHandler.previousState = leftWheelCurrentState;
        leftWheelStateHandler.isOperational = false;
      }
      else if (CIA402_State leftWheelCurrentState = deriveState(leftWheelShData.status_word);
               leftWheelCurrentState == CIA402_State::OPERATION_ENABLED)
      {
        leftWheelStateHandler.isOperational = true;
      }

              /* odomFuture =
                  std::async(std::launch::async, &Odometry::update, odomHandler,
         motorVelToLinearVel_BHFF((double)rightWheelShData.current_velocity / 0.1), motorVelToLinearVel_BHFF((double)leftWheelShData.current_velocity / 0.1), currentTime);
 */
      bool driversEnabled = (rightWheelStateHandler.isOperational && leftWheelStateHandler.isOperational);

          rosSyncMutex.lock();
          VelocityCommand velCmd = *velCommandPtr;  
          auto od = odomHandler.update(motorVelToLinearVel_BHFF((double)rightWheelShData.current_velocity / 0.1), motorVelToLinearVel_BHFF((double)leftWheelShData.current_velocity / 0.1), currentTime);
          rosDataPtr->odometry = od;
          rosSyncMutex.unlock();

      // If all slaves are operational, write commands:
      if (driversEnabled)
      {
        // Get data from ROS

      
      auto wheelVelocities = getWheelVelocityFromRobotCmd(velCmd.linear, velCmd.angular);

      rightWheelData.target_velocity = jointLinearVelToMotorVel_BHF(wheelVelocities.first * 0.1);
      leftWheelData.target_velocity = jointLinearVelToMotorVel_BHF(wheelVelocities.second * 0.1) * -1;
      rightWheelShData.target_velocity = rightWheelData.target_velocity;
      leftWheelShData.target_velocity = leftWheelData.target_velocity;
      }
      else
      {
        rightWheelShData.target_velocity = 0;
        leftWheelShData.target_velocity = 0;
      }
      sharedMemoryHandler.unlock();
    }
    
    
    std::this_thread::sleep_for(2ms);
  }
  shutdownRequested = true;
}