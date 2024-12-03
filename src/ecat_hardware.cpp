
#include "ecat_sh_hardware/ecat_hardware.hpp"
#include "ecat_sh_hardware/shared_obj.hpp"
#include "ecrt.h"

#include <unordered_map>
#include <signal.h>
#include <array>
#include <iostream>

constexpr auto SLAVE_START_POSITION = 0;
constexpr auto SLAVE_ALIAS = 0;
constexpr auto SLAVE_VENDOR_ID = 0x000000fb;
constexpr auto SLAVE_PRODUCT_ID = 0x65520000;
struct
{
  uint control_word;
  uint operation_mode;
  uint target_position;
  uint target_velocity;
  uint status_word;
  uint current_position;
  uint current_velocity;
} LeftMotorEthercatDataOffsets;

struct
{
  uint control_word;
  uint operation_mode;
  uint target_position;
  uint target_velocity;
  uint status_word;
  uint current_position;
  uint current_velocity;
} RightMotorEthercatDataOffsets;

/* struct IoChannelOffsets
{
  uint channel1;
  uint channel2;
  uint channel3;
  uint channel4;
  uint channel5;
  uint channel6;
  uint channel7;
  uint channel8;
  uint channel9;
  uint channel10;
  uint channel11;
  uint channel12;
  uint channel13;
  uint channel14;
  uint channel15;
  uint channel16;
}; */

std::array<uint, 16> digitalOutputOffsets;
std::array<uint, 16> digitalOutputBitPosition;
std::array<uint, 16> digitalInputOffsets;
std::array<uint, 16> digitalInputBitPosition;

/*
const ec_pdo_entry_reg_t lifterDomainRegistries[] = {
    {SLAVE_START_POSITION, SLAVE_ALIAS, SLAVE_VENDOR_ID, SLAVE_PRODUCT_ID,
     0x6040, 0x00, &EthercatDataOffsets.control_word},
    {SLAVE_START_POSITION, SLAVE_ALIAS, SLAVE_VENDOR_ID, SLAVE_PRODUCT_ID,
     0x6060, 0x00, &EthercatDataOffsets.operation_mode},
    {SLAVE_START_POSITION, SLAVE_ALIAS, SLAVE_VENDOR_ID, SLAVE_PRODUCT_ID,
     0x607a, 0x00, &EthercatDataOffsets.target_position},
    {SLAVE_START_POSITION, SLAVE_ALIAS, SLAVE_VENDOR_ID, SLAVE_PRODUCT_ID,
     0x60ff, 0x0, &EthercatDataOffsets.target_velocity},
    {SLAVE_START_POSITION, SLAVE_ALIAS, SLAVE_VENDOR_ID, SLAVE_PRODUCT_ID,
     0x6041, 0x0, &EthercatDataOffsets.status_word},
    {SLAVE_START_POSITION, SLAVE_ALIAS, SLAVE_VENDOR_ID, SLAVE_PRODUCT_ID,
     0x6064, 0x0, &EthercatDataOffsets.current_position},
    {SLAVE_START_POSITION, SLAVE_ALIAS, SLAVE_VENDOR_ID, SLAVE_PRODUCT_ID,
     0x606C, 0x0, &EthercatDataOffsets.current_velocity},
    {}};*/

ec_pdo_entry_info_t right_motor_pdo_entries[] = { { 0x6040, 0x00, 16 }, { 0x6060, 0x00, 8 },  { 0x607a, 0x00, 32 },
                                                  { 0x60ff, 0x00, 32 }, { 0x6041, 0x00, 16 }, { 0x6064, 0x00, 32 },
                                                  { 0x606c, 0x00, 32 } };

ec_pdo_info_t right_motor_pdo_info[] = { { 0x1603, 4, right_motor_pdo_entries + 0 },
                                         { 0x1A03, 3, right_motor_pdo_entries + 4 } };

ec_sync_info_t right_motor_slave_syncs[] = { { 0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE },
                                             { 1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE },
                                             { 2, EC_DIR_OUTPUT, 1, right_motor_pdo_info + 0, EC_WD_ENABLE },
                                             { 3, EC_DIR_INPUT, 1, right_motor_pdo_info + 1, EC_WD_DISABLE },
                                             { 0xff } };

ec_pdo_entry_info_t left_motor_pdo_entries[] = { { 0x6040, 0x00, 16 }, { 0x6060, 0x00, 8 },  { 0x607a, 0x00, 32 },
                                                 { 0x60ff, 0x00, 32 }, { 0x6041, 0x00, 16 }, { 0x6064, 0x00, 32 },
                                                 { 0x606c, 0x00, 32 } };

ec_pdo_info_t left_motor_pdo_info[] = { { 0x1603, 4, left_motor_pdo_entries + 0 },
                                        { 0x1A03, 3, left_motor_pdo_entries + 4 } };

ec_sync_info_t left_motor_slave_syncs[] = { { 0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE },
                                            { 1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE },
                                            { 2, EC_DIR_OUTPUT, 1, left_motor_pdo_info + 0, EC_WD_ENABLE },
                                            { 3, EC_DIR_INPUT, 1, left_motor_pdo_info + 1, EC_WD_DISABLE },
                                            { 0xff } };

constexpr auto DIGITAL_INPUT_START_POSITION = 3;
constexpr auto DIGITAL_INPUT_ALIAS = 0;
constexpr auto DIGITAL_INPUT_VENDOR_ID = 0x00000002;
constexpr auto DIGITAL_INPUT_PRODUCT_ID = 0x07113052;
ec_pdo_entry_info_t digital_input_pdo_entries[] = {
  { 0x6000, 0x01, 1 }, { 0x6010, 0x01, 1 }, { 0x6020, 0x01, 1 }, { 0x6030, 0x01, 1 },
  { 0x6040, 0x01, 1 }, { 0x6050, 0x01, 1 }, { 0x6060, 0x01, 1 }, { 0x6070, 0x01, 1 },
  { 0x6080, 0x01, 1 }, { 0x6090, 0x01, 1 }, { 0x60a0, 0x01, 1 }, { 0x60b0, 0x01, 1 },
  { 0x60c0, 0x01, 1 }, { 0x60d0, 0x01, 1 }, { 0x60e0, 0x01, 1 }, { 0x60f0, 0x01, 1 },
};

ec_pdo_info_t digital_input_pdos[] = {
  { 0x1a00, 1, digital_input_pdo_entries + 0 },  /* Channel 1 */
  { 0x1a01, 1, digital_input_pdo_entries + 1 },  /* Channel 2 */
  { 0x1a02, 1, digital_input_pdo_entries + 2 },  /* Channel 3 */
  { 0x1a03, 1, digital_input_pdo_entries + 3 },  /* Channel 4 */
  { 0x1a04, 1, digital_input_pdo_entries + 4 },  /* Channel 5 */
  { 0x1a05, 1, digital_input_pdo_entries + 5 },  /* Channel 6 */
  { 0x1a06, 1, digital_input_pdo_entries + 6 },  /* Channel 7 */
  { 0x1a07, 1, digital_input_pdo_entries + 7 },  /* Channel 8 */
  { 0x1a08, 1, digital_input_pdo_entries + 8 },  /* Channel 9 */
  { 0x1a09, 1, digital_input_pdo_entries + 9 },  /* Channel 10 */
  { 0x1a0a, 1, digital_input_pdo_entries + 10 }, /* Channel 11 */
  { 0x1a0b, 1, digital_input_pdo_entries + 11 }, /* Channel 12 */
  { 0x1a0c, 1, digital_input_pdo_entries + 12 }, /* Channel 13 */
  { 0x1a0d, 1, digital_input_pdo_entries + 13 }, /* Channel 14 */
  { 0x1a0e, 1, digital_input_pdo_entries + 14 }, /* Channel 15 */
  { 0x1a0f, 1, digital_input_pdo_entries + 15 }, /* Channel 16 */
};

ec_sync_info_t digital_input_syncs[] = { { 0, EC_DIR_INPUT, 16, digital_input_pdos + 0, EC_WD_DISABLE }, { 0xff } };

constexpr auto DIGITAL_OUTPUT_START_POSITION = 4;
constexpr auto DIGITAL_OUTPUT_ALIAS = 0;
constexpr auto DIGITAL_OUTPUT_VENDOR_ID = 0x00000002;
constexpr auto DIGITAL_OUTPUT_PRODUCT_ID = 0x0af93052;
ec_pdo_entry_info_t digital_output_pdo_entries[] = {
  { 0x7000, 0x01, 1 }, { 0x7010, 0x01, 1 }, { 0x7020, 0x01, 1 }, { 0x7030, 0x01, 1 },
  { 0x7040, 0x01, 1 }, { 0x7050, 0x01, 1 }, { 0x7060, 0x01, 1 }, { 0x7070, 0x01, 1 },
  { 0x7080, 0x01, 1 }, { 0x7090, 0x01, 1 }, { 0x70a0, 0x01, 1 }, { 0x70b0, 0x01, 1 },
  { 0x70c0, 0x01, 1 }, { 0x70d0, 0x01, 1 }, { 0x70e0, 0x01, 1 }, { 0x70f0, 0x01, 1 },

};

ec_pdo_info_t digital_output_pdos[] = {
  { 0x1600, 1, digital_output_pdo_entries + 0 },  /* Channel 1 */
  { 0x1601, 1, digital_output_pdo_entries + 1 },  /* Channel 2 */
  { 0x1602, 1, digital_output_pdo_entries + 2 },  /* Channel 3 */
  { 0x1603, 1, digital_output_pdo_entries + 3 },  /* Channel 4 */
  { 0x1604, 1, digital_output_pdo_entries + 4 },  /* Channel 5 */
  { 0x1605, 1, digital_output_pdo_entries + 5 },  /* Channel 6 */
  { 0x1606, 1, digital_output_pdo_entries + 6 },  /* Channel 7 */
  { 0x1607, 1, digital_output_pdo_entries + 7 },  /* Channel 8 */
  { 0x1608, 1, digital_output_pdo_entries + 8 },  /* Channel 9 */
  { 0x1609, 1, digital_output_pdo_entries + 9 },  /* Channel 10 */
  { 0x160a, 1, digital_output_pdo_entries + 10 }, /* Channel 11 */
  { 0x160b, 1, digital_output_pdo_entries + 11 }, /* Channel 12 */
  { 0x160c, 1, digital_output_pdo_entries + 12 }, /* Channel 13 */
  { 0x160d, 1, digital_output_pdo_entries + 13 }, /* Channel 14 */
  { 0x160e, 1, digital_output_pdo_entries + 14 }, /* Channel 15 */
  { 0x160f, 1, digital_output_pdo_entries + 15 }, /* Channel 16 */

};

ec_sync_info_t digital_output_syncs[] = { { 0, EC_DIR_OUTPUT, 8, digital_output_pdos + 0, EC_WD_ENABLE },
                                          { 1, EC_DIR_OUTPUT, 8, digital_output_pdos + 8, EC_WD_ENABLE },
                                          { 0xff } };

/* ec_sync_info_t digital_output_syncs[] = { { 0, EC_DIR_OUTPUT, 16, digital_output_pdos + 0, EC_WD_ENABLE }, { 0xff }
 * };
 */
const ec_pdo_entry_reg_t domainRegistries[] = {
  { SLAVE_ALIAS, SLAVE_START_POSITION, SLAVE_VENDOR_ID, SLAVE_PRODUCT_ID, 0x6040, 0x00,
    &RightMotorEthercatDataOffsets.control_word },
  { SLAVE_ALIAS, SLAVE_START_POSITION, SLAVE_VENDOR_ID, SLAVE_PRODUCT_ID, 0x6060, 0x00,
    &RightMotorEthercatDataOffsets.operation_mode },
  { SLAVE_ALIAS, SLAVE_START_POSITION, SLAVE_VENDOR_ID, SLAVE_PRODUCT_ID, 0x607a, 0x00,
    &RightMotorEthercatDataOffsets.target_position },
  { SLAVE_ALIAS, SLAVE_START_POSITION, SLAVE_VENDOR_ID, SLAVE_PRODUCT_ID, 0x60ff, 0x00,
    &RightMotorEthercatDataOffsets.target_velocity },
  { SLAVE_ALIAS, SLAVE_START_POSITION, SLAVE_VENDOR_ID, SLAVE_PRODUCT_ID, 0x6041, 0x00,
    &RightMotorEthercatDataOffsets.status_word },
  { SLAVE_ALIAS, SLAVE_START_POSITION, SLAVE_VENDOR_ID, SLAVE_PRODUCT_ID, 0x6064, 0x00,
    &RightMotorEthercatDataOffsets.current_position },
  { SLAVE_ALIAS, SLAVE_START_POSITION, SLAVE_VENDOR_ID, SLAVE_PRODUCT_ID, 0x606C, 0x00,
    &RightMotorEthercatDataOffsets.current_velocity },
  { SLAVE_ALIAS, SLAVE_START_POSITION + 1, SLAVE_VENDOR_ID, SLAVE_PRODUCT_ID, 0x6040, 0x00,
    &LeftMotorEthercatDataOffsets.control_word },
  { SLAVE_ALIAS, SLAVE_START_POSITION + 1, SLAVE_VENDOR_ID, SLAVE_PRODUCT_ID, 0x6060, 0x00,
    &LeftMotorEthercatDataOffsets.operation_mode },
  { SLAVE_ALIAS, SLAVE_START_POSITION + 1, SLAVE_VENDOR_ID, SLAVE_PRODUCT_ID, 0x607a, 0x00,
    &LeftMotorEthercatDataOffsets.target_position },
  { SLAVE_ALIAS, SLAVE_START_POSITION + 1, SLAVE_VENDOR_ID, SLAVE_PRODUCT_ID, 0x60ff, 0x00,
    &LeftMotorEthercatDataOffsets.target_velocity },
  { SLAVE_ALIAS, SLAVE_START_POSITION + 1, SLAVE_VENDOR_ID, SLAVE_PRODUCT_ID, 0x6041, 0x00,
    &LeftMotorEthercatDataOffsets.status_word },
  { SLAVE_ALIAS, SLAVE_START_POSITION + 1, SLAVE_VENDOR_ID, SLAVE_PRODUCT_ID, 0x6064, 0x00,
    &LeftMotorEthercatDataOffsets.current_position },
  { SLAVE_ALIAS, SLAVE_START_POSITION + 1, SLAVE_VENDOR_ID, SLAVE_PRODUCT_ID, 0x606C, 0x00,
    &LeftMotorEthercatDataOffsets.current_velocity },
  {}
};

const ec_pdo_entry_reg_t digitalIoDomainRegistries[] = {
  { DIGITAL_INPUT_ALIAS, DIGITAL_INPUT_START_POSITION, DIGITAL_INPUT_VENDOR_ID, DIGITAL_INPUT_PRODUCT_ID, 0x6000, 0x01,
    &digitalInputOffsets[0], &digitalInputBitPosition[0] },
  { DIGITAL_INPUT_ALIAS, DIGITAL_INPUT_START_POSITION, DIGITAL_INPUT_VENDOR_ID, DIGITAL_INPUT_PRODUCT_ID, 0x6010, 0x01,
    &digitalInputOffsets[1], &digitalInputBitPosition[1] },
  { DIGITAL_INPUT_ALIAS, DIGITAL_INPUT_START_POSITION, DIGITAL_INPUT_VENDOR_ID, DIGITAL_INPUT_PRODUCT_ID, 0x6020, 0x01,
    &digitalInputOffsets[2], &digitalInputBitPosition[2] },
  { DIGITAL_INPUT_ALIAS, DIGITAL_INPUT_START_POSITION, DIGITAL_INPUT_VENDOR_ID, DIGITAL_INPUT_PRODUCT_ID, 0x6030, 0x01,
    &digitalInputOffsets[3], &digitalInputBitPosition[3] },
  { DIGITAL_INPUT_ALIAS, DIGITAL_INPUT_START_POSITION, DIGITAL_INPUT_VENDOR_ID, DIGITAL_INPUT_PRODUCT_ID, 0x6040, 0x01,
    &digitalInputOffsets[4], &digitalInputBitPosition[4] },
  { DIGITAL_INPUT_ALIAS, DIGITAL_INPUT_START_POSITION, DIGITAL_INPUT_VENDOR_ID, DIGITAL_INPUT_PRODUCT_ID, 0x6050, 0x01,
    &digitalInputOffsets[5], &digitalInputBitPosition[5] },
  { DIGITAL_INPUT_ALIAS, DIGITAL_INPUT_START_POSITION, DIGITAL_INPUT_VENDOR_ID, DIGITAL_INPUT_PRODUCT_ID, 0x6060, 0x01,
    &digitalInputOffsets[6], &digitalInputBitPosition[6] },
  { DIGITAL_INPUT_ALIAS, DIGITAL_INPUT_START_POSITION, DIGITAL_INPUT_VENDOR_ID, DIGITAL_INPUT_PRODUCT_ID, 0x6070, 0x01,
    &digitalInputOffsets[7], &digitalInputBitPosition[7] },
  { DIGITAL_INPUT_ALIAS, DIGITAL_INPUT_START_POSITION, DIGITAL_INPUT_VENDOR_ID, DIGITAL_INPUT_PRODUCT_ID, 0x6080, 0x01,
    &digitalInputOffsets[8], &digitalInputBitPosition[8] },
  { DIGITAL_INPUT_ALIAS, DIGITAL_INPUT_START_POSITION, DIGITAL_INPUT_VENDOR_ID, DIGITAL_INPUT_PRODUCT_ID, 0x6090, 0x01,
    &digitalInputOffsets[9], &digitalInputBitPosition[9] },
  { DIGITAL_INPUT_ALIAS, DIGITAL_INPUT_START_POSITION, DIGITAL_INPUT_VENDOR_ID, DIGITAL_INPUT_PRODUCT_ID, 0x60a0, 0x01,
    &digitalInputOffsets[10], &digitalInputBitPosition[10] },
  { DIGITAL_INPUT_ALIAS, DIGITAL_INPUT_START_POSITION, DIGITAL_INPUT_VENDOR_ID, DIGITAL_INPUT_PRODUCT_ID, 0x60b0, 0x01,
    &digitalInputOffsets[11], &digitalInputBitPosition[11] },
  { DIGITAL_INPUT_ALIAS, DIGITAL_INPUT_START_POSITION, DIGITAL_INPUT_VENDOR_ID, DIGITAL_INPUT_PRODUCT_ID, 0x60c0, 0x01,
    &digitalInputOffsets[12], &digitalInputBitPosition[12] },
  { DIGITAL_INPUT_ALIAS, DIGITAL_INPUT_START_POSITION, DIGITAL_INPUT_VENDOR_ID, DIGITAL_INPUT_PRODUCT_ID, 0x60d0, 0x01,
    &digitalInputOffsets[13], &digitalInputBitPosition[13] },
  { DIGITAL_INPUT_ALIAS, DIGITAL_INPUT_START_POSITION, DIGITAL_INPUT_VENDOR_ID, DIGITAL_INPUT_PRODUCT_ID, 0x60e0, 0x01,
    &digitalInputOffsets[14], &digitalInputBitPosition[14] },
  { DIGITAL_INPUT_ALIAS, DIGITAL_INPUT_START_POSITION, DIGITAL_INPUT_VENDOR_ID, DIGITAL_INPUT_PRODUCT_ID, 0x60f0, 0x01,
    &digitalInputOffsets[15], &digitalInputBitPosition[15] },
  { DIGITAL_OUTPUT_ALIAS, DIGITAL_OUTPUT_START_POSITION, DIGITAL_OUTPUT_VENDOR_ID, DIGITAL_OUTPUT_PRODUCT_ID, 0x7000,
    0x01, &digitalOutputOffsets[0], &digitalOutputBitPosition[0] },
  { DIGITAL_OUTPUT_ALIAS, DIGITAL_OUTPUT_START_POSITION, DIGITAL_OUTPUT_VENDOR_ID, DIGITAL_OUTPUT_PRODUCT_ID, 0x7010,
    0x01, &digitalOutputOffsets[1], &digitalOutputBitPosition[1] },
  { DIGITAL_OUTPUT_ALIAS, DIGITAL_OUTPUT_START_POSITION, DIGITAL_OUTPUT_VENDOR_ID, DIGITAL_OUTPUT_PRODUCT_ID, 0x7020,
    0x01, &digitalOutputOffsets[2], &digitalOutputBitPosition[2] },
  { DIGITAL_OUTPUT_ALIAS, DIGITAL_OUTPUT_START_POSITION, DIGITAL_OUTPUT_VENDOR_ID, DIGITAL_OUTPUT_PRODUCT_ID, 0x7030,
    0x01, &digitalOutputOffsets[3], &digitalOutputBitPosition[3] },
  { DIGITAL_OUTPUT_ALIAS, DIGITAL_OUTPUT_START_POSITION, DIGITAL_OUTPUT_VENDOR_ID, DIGITAL_OUTPUT_PRODUCT_ID, 0x7040,
    0x01, &digitalOutputOffsets[4], &digitalOutputBitPosition[4] },
  { DIGITAL_OUTPUT_ALIAS, DIGITAL_OUTPUT_START_POSITION, DIGITAL_OUTPUT_VENDOR_ID, DIGITAL_OUTPUT_PRODUCT_ID, 0x7050,
    0x01, &digitalOutputOffsets[5], &digitalOutputBitPosition[5] },
  { DIGITAL_OUTPUT_ALIAS, DIGITAL_OUTPUT_START_POSITION, DIGITAL_OUTPUT_VENDOR_ID, DIGITAL_OUTPUT_PRODUCT_ID, 0x7060,
    0x01, &digitalOutputOffsets[6], &digitalOutputBitPosition[6] },
  { DIGITAL_OUTPUT_ALIAS, DIGITAL_OUTPUT_START_POSITION, DIGITAL_OUTPUT_VENDOR_ID, DIGITAL_OUTPUT_PRODUCT_ID, 0x7070,
    0x01, &digitalOutputOffsets[7], &digitalOutputBitPosition[7] },
  { DIGITAL_OUTPUT_ALIAS, DIGITAL_OUTPUT_START_POSITION, DIGITAL_OUTPUT_VENDOR_ID, DIGITAL_OUTPUT_PRODUCT_ID, 0x7080,
    0x01, &digitalOutputOffsets[8], &digitalOutputBitPosition[8] },
  { DIGITAL_OUTPUT_ALIAS, DIGITAL_OUTPUT_START_POSITION, DIGITAL_OUTPUT_VENDOR_ID, DIGITAL_OUTPUT_PRODUCT_ID, 0x7090,
    0x01, &digitalOutputOffsets[9], &digitalOutputBitPosition[9] },
  { DIGITAL_OUTPUT_ALIAS, DIGITAL_OUTPUT_START_POSITION, DIGITAL_OUTPUT_VENDOR_ID, DIGITAL_OUTPUT_PRODUCT_ID, 0x70a0,
    0x01, &digitalOutputOffsets[10], &digitalOutputBitPosition[10] },
  { DIGITAL_OUTPUT_ALIAS, DIGITAL_OUTPUT_START_POSITION, DIGITAL_OUTPUT_VENDOR_ID, DIGITAL_OUTPUT_PRODUCT_ID, 0x70b0,
    0x01, &digitalOutputOffsets[11], &digitalOutputBitPosition[11] },
  { DIGITAL_OUTPUT_ALIAS, DIGITAL_OUTPUT_START_POSITION, DIGITAL_OUTPUT_VENDOR_ID, DIGITAL_OUTPUT_PRODUCT_ID, 0x70c0,
    0x01, &digitalOutputOffsets[12], &digitalOutputBitPosition[12] },
  { DIGITAL_OUTPUT_ALIAS, DIGITAL_OUTPUT_START_POSITION, DIGITAL_OUTPUT_VENDOR_ID, DIGITAL_OUTPUT_PRODUCT_ID, 0x70d0,
    0x01, &digitalOutputOffsets[13], &digitalOutputBitPosition[13] },
  { DIGITAL_OUTPUT_ALIAS, DIGITAL_OUTPUT_START_POSITION, DIGITAL_OUTPUT_VENDOR_ID, DIGITAL_OUTPUT_PRODUCT_ID, 0x70e0,
    0x01, &digitalOutputOffsets[14], &digitalOutputBitPosition[14] },
  { DIGITAL_OUTPUT_ALIAS, DIGITAL_OUTPUT_START_POSITION, DIGITAL_OUTPUT_VENDOR_ID, DIGITAL_OUTPUT_PRODUCT_ID, 0x70f0,
    0x01, &digitalOutputOffsets[15], &digitalOutputBitPosition[15] },
  {}
};

bool runHardwareLoop = true;

void sigInHandler(int signal)
{
  runHardwareLoop = false;
}

int main(int argc, char** argv)
{
  ec_master_t* masterPtr = nullptr;
  ec_domain_t* domainPtr = nullptr;
  ec_domain_t* digitalIoDomainPtr = nullptr;
  ec_slave_config_t* slaveConfigPtr = nullptr;
  uint8_t* domainProcessData = nullptr;
  uint8_t* digitalIoDomainProcessData = nullptr;

  masterPtr = ecrt_request_master(0);
  if (masterPtr == nullptr)
  {
    return 1;
  }

  domainPtr = ecrt_master_create_domain(masterPtr);
  if (domainPtr == nullptr)
  {
    return 1;
  }

  digitalIoDomainPtr = ecrt_master_create_domain(masterPtr);
  if (digitalIoDomainPtr == nullptr)
  {
    return 1;
  }

  // Configure first slave:
  slaveConfigPtr =
      ecrt_master_slave_config(masterPtr, SLAVE_ALIAS, SLAVE_START_POSITION, SLAVE_VENDOR_ID, SLAVE_PRODUCT_ID);
  if (slaveConfigPtr == nullptr)
  {
    return 1;
  }

  int configurePdosRes = ecrt_slave_config_pdos(slaveConfigPtr, EC_END, right_motor_slave_syncs);
  if (configurePdosRes != 0)
  {
    std::cout << "Could not configure right motor driver PDOs." << std::endl;
    return 1;
  }

  ecrt_slave_config_dc(slaveConfigPtr, 0x0300, 2000000, 1000000, 2000000, 0);

  // Configure second slave:

  slaveConfigPtr =
      ecrt_master_slave_config(masterPtr, SLAVE_ALIAS, SLAVE_START_POSITION + 1, SLAVE_VENDOR_ID, SLAVE_PRODUCT_ID);
  configurePdosRes = ecrt_slave_config_pdos(slaveConfigPtr, EC_END, left_motor_slave_syncs);
  if (configurePdosRes != 0)
  {
    std::cout << "Could not configure left motor driver PDOs." << std::endl;
    return 1;
  }

  ecrt_slave_config_dc(slaveConfigPtr, 0x0300, 2000000, 1000000, 2000000, 0);

  slaveConfigPtr = ecrt_master_slave_config(masterPtr, 0, 2, DIGITAL_INPUT_VENDOR_ID, 0x044c2c52);

  slaveConfigPtr = ecrt_master_slave_config(masterPtr, DIGITAL_INPUT_ALIAS, DIGITAL_INPUT_START_POSITION,
                                            DIGITAL_INPUT_VENDOR_ID, DIGITAL_INPUT_PRODUCT_ID);
  configurePdosRes = ecrt_slave_config_pdos(slaveConfigPtr, EC_END, digital_input_syncs);
  if (configurePdosRes != 0)
  {
    std::cout << "Could not configure digital input PDOs." << std::endl;
    return 1;
  }

  slaveConfigPtr = ecrt_master_slave_config(masterPtr, DIGITAL_OUTPUT_ALIAS, DIGITAL_OUTPUT_START_POSITION,
                                            DIGITAL_OUTPUT_VENDOR_ID, DIGITAL_OUTPUT_PRODUCT_ID);
  configurePdosRes = ecrt_slave_config_pdos(slaveConfigPtr, EC_END, digital_output_syncs);
  
  if (configurePdosRes != 0)
  {
    std::cout << "Could not configure digital input PDOs." << std::endl;
    return 1;
  }

  int registerDomainEntriesRes = ecrt_domain_reg_pdo_entry_list(domainPtr, domainRegistries);

  if (registerDomainEntriesRes != 0)
  {
    std::cout << "Could not register PDO list." << std::endl;
    return 1;
  }

  registerDomainEntriesRes = ecrt_domain_reg_pdo_entry_list(digitalIoDomainPtr, digitalIoDomainRegistries);
  if(registerDomainEntriesRes != 0)
  {
    std::cout << "Could not register digital IO PDO list." << std::endl;
    return 1;
  }

  // Set current thread scheduler and priority:

  const sched_param schedParam{ .sched_priority = 80 };
  if (sched_setscheduler(0, SCHED_FIFO, &schedParam) != 0)
  {
    std::cout << "Could not set scheduler policy." << std::endl;
    return 1;
  }

  // Initiliaze shared memory:

  std::expected<shm_handler::SharedMemoryHandler<shared_obj_info::EthercatDataObject, 2>, shm_handler::Error>
      sharedMemoryHandlerInit = shm_handler::SharedMemoryHandler<shared_obj_info::EthercatDataObject, 2>::create(
          shared_obj_info::SHARED_MEMORY_SEG_NAME, shared_obj_info::ETHERCAT_DATA_SEM_NAME, shm_handler::Mode::CREATE);

  if (!sharedMemoryHandlerInit.has_value())
  {
    std::cout << "Could not create Shared Memory Handler: " << shm_handler::ErrorMap.at(sharedMemoryHandlerInit.error())
              << std::endl;
    return 10;
  }

  auto sharedMemoryHandler = std::move(sharedMemoryHandlerInit.value());

  // Activate EtherCAT:

  int activateMasterRes = ecrt_master_activate(masterPtr);
  if (activateMasterRes != 0)
  {
    std::cout << "Could not activate master" << std::endl;
    return 1;
  }

  if (!(domainProcessData = ecrt_domain_data(domainPtr)))
  {
    std::cout << "Could not create domain data" << std::endl;
    return 1;
  }

  if(!(digitalIoDomainProcessData = ecrt_domain_data(digitalIoDomainPtr)))
  {
    std::cout << "Could not create digital IO domain data" << std::endl;
    return 1;
  }

  // Connect shutdown signal to SIGINT:
  signal(SIGINT, sigInHandler);

  ecat_sh_hardware::DistributedClockHelper distributedClockHelper;

  clock_gettime(CLOCK_MONOTONIC, &distributedClockHelper.wakeupTime);

  shared_obj_info::EthercatDataObject rightWheelData;
  shared_obj_info::EthercatDataObject leftWheelData;

  auto periodNs = ecat_sh_hardware::NANOSEC_PER_SEC / 500;
  distributedClockHelper.cycleTime = { 0, periodNs };
  distributedClockHelper.referenceClockCounter = 0;
  /*
    struct sched_param processParam = {};
    processParam.sched_priority = sched_get_priority_max(SCHED_FIFO);
    if(sched_setscheduler(0, SCHED_FIFO, &processParam) != 0)
    {
      std::cout << "Could not change process priority" << std::endl;
      return 1;
    } */
  timespec startTime;
  timespec lastStartTime = startTime;
  clock_gettime(CLOCK_MONOTONIC, &startTime);
  bool bit_test_counter = true;
  int dig_out_count = 0;
  while (runHardwareLoop)
  {
    // DC sync
    distributedClockHelper.wakeupTime =
        ecat_sh_hardware::addTimespec(distributedClockHelper.wakeupTime, distributedClockHelper.cycleTime);
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &distributedClockHelper.wakeupTime, NULL);

    clock_gettime(CLOCK_MONOTONIC, &startTime);
    timespec periodTs = { .tv_sec = startTime.tv_sec - lastStartTime.tv_sec,
                          .tv_nsec = startTime.tv_nsec - lastStartTime.tv_nsec };
    lastStartTime = startTime;

    std::chrono::duration<double> periodAsSecs = timespecToChronoDuration<double, std::ratio<1>>(periodTs);

    std::cout << "Period: " << periodAsSecs.count() << std::endl;
    ecrt_master_application_time(masterPtr, ecat_sh_hardware::timespecToNanoSec(distributedClockHelper.wakeupTime));

    ecrt_master_receive(masterPtr);
    ecrt_domain_process(domainPtr);

    /**
     * Read || Write Logic
     *
     */
    auto rightMotorSW = readFromSlave<uint16_t>(domainProcessData, RightMotorEthercatDataOffsets.status_word);
    if (rightMotorSW)
    {
      rightWheelData.status_word = rightMotorSW.value();
      std::cout << "Right SW: " << rightMotorSW.value() << std::endl;
      // std::cout << "Written control word: " << readFromSlave<uint16_t>(domainProcessData,
      // RightMotorEthercatDataOffsets.control_word).value() << std::endl;
    }
    auto rightMotorCurrentPosition =
        readFromSlave<int32_t>(domainProcessData, RightMotorEthercatDataOffsets.current_position);
    if (rightMotorCurrentPosition)
    {
      rightWheelData.current_position = rightMotorCurrentPosition.value();
    }
    auto rightMotorCurrentVelocity =
        readFromSlave<int32_t>(domainProcessData, RightMotorEthercatDataOffsets.current_position);
    if (rightMotorCurrentPosition)
    {
      rightWheelData.current_position = rightMotorCurrentVelocity.value();
    }

    auto leftMotorSW = readFromSlave<uint16_t>(domainProcessData, LeftMotorEthercatDataOffsets.status_word);
    if (leftMotorSW)
    {
      leftWheelData.status_word = leftMotorSW.value();
      std::cout << "Left SW: " << leftMotorSW.value() << std::endl;
    }
    auto leftMotorCurrentPosition =
        readFromSlave<int32_t>(domainProcessData, LeftMotorEthercatDataOffsets.current_position);
    if (leftMotorCurrentPosition)
    {
      leftWheelData.current_position = leftMotorCurrentPosition.value();
    }
    auto leftMotorCurrentVelocity =
        readFromSlave<int32_t>(domainProcessData, LeftMotorEthercatDataOffsets.current_position);
    if (leftMotorCurrentVelocity)
    {
      leftWheelData.current_position = leftMotorCurrentVelocity.value();
    }

    // If we get a lock on the semaphore, read/write from/to EtherCAT:

    if (sharedMemoryHandler.tryLock())
    {
      std::cout << "Got lock\n";
      auto& rightWheelShData = sharedMemoryHandler.getDataPtr()[0];
      rightWheelShData.status_word = rightWheelData.status_word;
      rightWheelShData.current_position = rightWheelData.current_position;
      rightWheelShData.current_velocity = rightWheelData.current_velocity;

      rightWheelData.control_word = rightWheelShData.control_word;
      rightWheelData.target_position = rightWheelShData.target_position;
      rightWheelData.target_velocity = rightWheelShData.target_velocity;

      auto& leftWheelShData = sharedMemoryHandler.getDataPtr()[1];
      leftWheelShData.status_word = leftWheelData.status_word;
      leftWheelShData.current_position = leftWheelData.current_position;
      leftWheelShData.current_velocity = leftWheelData.current_velocity;
      // leftWheelData.status_word = leftWheelShData.status_word;
      // leftWheelData.current_position = leftWheelShData.current_position;
      // leftWheelData.current_velocity = leftWheelShData.current_velocity;

      leftWheelData.control_word = leftWheelShData.control_word;
      leftWheelData.target_position = leftWheelShData.target_position;
      leftWheelData.target_velocity = leftWheelShData.target_velocity;

      sharedMemoryHandler.unlock();
    }

    writeToSlave(domainProcessData, RightMotorEthercatDataOffsets.control_word, rightWheelData.control_word);
    writeToSlave(domainProcessData, RightMotorEthercatDataOffsets.operation_mode, rightWheelData.operation_mode);
    writeToSlave(domainProcessData, RightMotorEthercatDataOffsets.target_velocity, rightWheelData.target_velocity);

    writeToSlave(domainProcessData, LeftMotorEthercatDataOffsets.control_word, leftWheelData.control_word);
    writeToSlave(domainProcessData, LeftMotorEthercatDataOffsets.operation_mode, leftWheelData.operation_mode);
    writeToSlave(domainProcessData, LeftMotorEthercatDataOffsets.target_velocity, leftWheelData.target_velocity);

    clock_gettime(CLOCK_MONOTONIC, &distributedClockHelper.currentTime);
    if (distributedClockHelper.referenceClockCounter)
    {
      distributedClockHelper.referenceClockCounter -= 1;
      ecrt_master_sync_reference_clock_to(masterPtr,
                                          ecat_sh_hardware::timespecToNanoSec(distributedClockHelper.currentTime));
    }
    else
    {
      distributedClockHelper.referenceClockCounter = 1;
    }

    ecrt_master_sync_slave_clocks(masterPtr);

    ecrt_domain_queue(domainPtr);
    ecrt_master_send(masterPtr);
  }

  ecrt_release_master(masterPtr);

  return 0;
}
