#include "ecat_master.hpp"

using namespace EthercatCommunication;

/*****************************************************************************************/
/// Extern global variable declaration.
uint32_t g_sync_ref_counter = 0;
/*****************************************************************************************/

EthercatMaster::EthercatMaster()
{
  //request_sdos_.resize(g_kNumberOfServoDrivers);
}

EthercatMaster::~EthercatMaster()
{
}

int EthercatMaster::OpenEthercatMaster()
{
  std::cout << "[EthercatMaster] Trying to open EtherCAT master...\n";
  fd = std::system("ls /dev | grep EtherCAT* > /dev/null");
  if (fd)
  {
    std::cout << "[EthercatMaster] Opening EtherCAT master...\n";
    std::system("sudo ethercatctl start");
    usleep(2e6);
    fd = std::system("ls /dev | grep EtherCAT* > /dev/null");
    if (fd)
    {
      std::cerr << "[EthercatMaster] Error : EtherCAT device not found.\n";
      return -1;
    }
    else
    {
      return 0;
    }
  }
  std::cout << "[EthercatMaster] EtherCAT master opened...\n";
  return 0;
}

int EthercatMaster::ConfigureMaster(int idx)
{
  std::cout << "[EthercatMaster] Requesting EtherCAT master...\n";
  if (!m_master)
  {
    m_master = ecrt_request_master(idx);
    if (!m_master)
    {
      std::cerr << "[EthercatMaster] Requesting master instance failed ! ";
      return -1;
    }
  }

  m_master_domain = ecrt_master_create_domain(m_master);
  if (!m_master_domain)
  {
    std::cerr << "[EthercatMaster] Failed to create master domain ! ";
    return -1;
  }
  return 0;
}

void EthercatMaster::ReleaseMaster()
{
  ecrt_release_master(m_master);
  m_master = NULL;
  m_master_domain = NULL;
  m_master_state = {};
}

int EthercatMaster::ShutDownEthercatMaster()
{
  fd = std::system("ls /dev | grep EtherCAT* > /dev/null\n");
  if (!fd)
  {
    std::cout << "[EthercatMaster] Shutting down EtherCAT master...";
    std::system("cd ~; sudo ethercatctl stop\n");
    usleep(1e6);
    fd = std::system("ls /dev | grep EtherCAT* > /dev/null\n");
    if (fd)
    {
      std::cout << "[EthercatMaster] EtherCAT shut down succesfull.";
      return 0;
    }
    else
    {
      std::cerr << "[EthercatMaster] Error : EtherCAT shutdown error.";
      return -1;
    }
  }
  return 0;
}

int EthercatMaster::RestartEthercatMaster()
{
  std::cout << "[EthercatMaster] Restarting EtherCAT master...";
  std::system("cd ~; sudo ethercatctl restart\n");
  usleep(2e6);
  fd = std::system("ls /dev | grep EtherCAT* > /dev/null\n");
  if (!fd)
  {
    std::cout << "[EthercatMaster] EtherCAT restart succesfull.";
    return 0;
  }
  else
  {
    std::cerr << "[EthercatMaster] Error : EtherCAT restart error.";
    return -1;
  }
}

int EthercatMaster::CheckMasterState(ec_master_state_t& ms)
{
  ecrt_master_state(m_master, &ms);
  if (ms.slaves_responding != m_master_state.slaves_responding)
  {
    std::cout << "[EthercatMaster] " << ms.slaves_responding <<" slave(s).\n";
    if (ms.slaves_responding < 1)
    {
      std::cout <<  "[EthercatMaster] Connection error,no response from slaves.";
      return -1;
    }
  }
  
  if (ms.al_states != m_master_state.al_states)
  {
    std::printf("[EthercatMaster] AL states: 0x%02X.\n", ms.al_states);
    //std::cout << "[EthercatMaster] AL states: 0x" << std::hex << std::setfill('0') <<std::setw(2) << ms.al_states << "\n";
  }
  if (ms.link_up != m_master_state.link_up)
  {
    std::printf("[EthercatMaster] Link is %s.\n", ms.link_up ? "up" : "down");
    if (!ms.link_up)
    {
      std::cerr << "[EthercatMaster] Master state link down";
      return -1;
    }
  }
  m_master_state = ms;
  return 0;
}

void EthercatMaster::CheckMasterDomainState()
{
  ec_domain_state_t ds;  // Domain instance
  ecrt_domain_state(m_master_domain, &ds);
  if (ds.working_counter != m_master_domain_state.working_counter)
    std::printf("[EthercatMaster] masterDomain: WC %u.\n", ds.working_counter);
  if (ds.wc_state != m_master_domain_state.wc_state)
    std::printf("[EthercatMaster] masterDomain: State %u.\n", ds.wc_state);
  if (m_master_domain_state.wc_state == EC_WC_COMPLETE)
  {
    m_master_domain_state = ds;
  }
  m_master_domain_state = ds;
}

int EthercatMaster::ActivateMaster()
{
  if (ecrt_master_activate(m_master))
  {
    std::cerr << "[EthercatMaster] Master activation error ! ";
    return -1;
  }
  return 0;
}

int EthercatMaster::RegisterDomain()
{
  for (int i = 0; i < NUM_OF_SLAVES; i++)
  {
    slaves_[i].slave_pdo_domain_ = ecrt_domain_data(m_master_domain);
    if(!(slaves_[i].slave_pdo_domain_) )
    {
        std::cerr << "[EthercatMaster] Domain PDO registration error";
        return -1;
    }
  }
  return 0;
}

void EthercatMaster::DeactivateCommunication()
{
  std::cout << "[EthercatMaster] Deactivating EtherCAT master : Cyclic PDO ended...\n";
  ecrt_master_deactivate(m_master);
  ReleaseMaster();
}

int EthercatMaster::GetNumberOfConnectedSlaves()
{
  unsigned int number_of_slaves;
  usleep(1e6);
  ecrt_master_state(m_master, &m_master_state);
  number_of_slaves = m_master_state.slaves_responding;
  if (NUM_OF_SLAVES != number_of_slaves)
  {
    std::cerr << "[EthercatMaster] Please enter correct number of slaves... \n"
              << "Entered number of slave : " << NUM_OF_SLAVES << "\n"
              << "Connected slaves        : " << number_of_slaves << "\n"; 
    return -1;
  }
  return 0;
}

void EthercatMaster::GetSlaveInformation(int position, ec_slave_info_t& info)
{
    ecrt_master_get_slave(m_master, position, &info);
}

int EthercatMaster::InitSlave(int i)
{
  ecrt_master_get_slave(m_master, i, &slaves_[i].slave_info_);
  slaves_[i].slave_config_ = ecrt_master_slave_config(m_master,slaves_[i].slave_info_.alias,
                                                                    slaves_[i].slave_info_.position,
                                                                    slaves_[i].slave_info_.vendor_id,
                                                                    slaves_[i].slave_info_.product_code); 
  if(!slaves_[i].slave_config_) {
      std::cerr << "[EthercatMaster] Failed to  configure slave ! ";
      return -1;
  }
  return 0 ;
}

void EthercatMaster::CheckSlaveConfigurationState()
{
  for (int i = 0; i < NUM_OF_SLAVES; i++)
  {
    //slaves_[i].CheckSlaveConfigState();
    ecrt_slave_config_state(slaves_[i].slave_config_, &slaves_[i].slave_config_state_);
    if (slaves_[i].slave_config_state_.al_state != 0x08)
    {
      std::cout << "[EthercatMaster] Slave "<<i<< " is not operational. AL state is :" << std::hex << slaves_[i].slave_config_state_.al_state << std::endl;
    }
  }
}

void EthercatMaster::ResetSlave(int i)
{
    slaves_[i].slave_pdo_domain_ = NULL;
    slaves_[i].slave_config_ = NULL;
    slaves_[i].slave_pdo_entry_info_ = NULL;
    slaves_[i].slave_pdo_entry_reg_ = NULL;
    slaves_[i].slave_pdo_info_ = NULL;
    slaves_[i].slave_sync_info_ = NULL;    
}

void EthercatMaster::ReceiveAndProcess()
{
  ecrt_master_receive(m_master);
  ecrt_domain_process(m_master_domain);
}

void EthercatMaster::QueueAndSend()
{
  ecrt_domain_queue(m_master_domain);
  ecrt_master_send(m_master);
}

void EthercatMaster::SetMasterApplicationTime(const timespec& time)
{
  ecrt_master_application_time(m_master, TIMESPEC2NS(time));
}

void EthercatMaster::SyncMasterReferenceClock(const timespec& time)
{
  ecrt_master_sync_reference_clock_to(m_master, TIMESPEC2NS(time));
}

void EthercatMaster::SyncSlaveClock()
{
  ecrt_master_sync_slave_clocks(m_master);
}

int EthercatMaster::WaitForOperationalMode()
{
  ec_master_state_t ms;
  int try_counter=0;
  int check_state_count=0;
  int time_out = 20e3;
  while (m_master_state.al_states != EC_AL_STATE_OP )
  {
    if(try_counter < time_out)
    {
      clock_gettime(CLOCK_MONOTONIC, &m_sync_timer);
      ecrt_master_application_time(m_master, TIMESPEC2NS(m_sync_timer));
      ecrt_master_receive(m_master);
      ecrt_domain_process(m_master_domain);
      usleep(PERIOD_US);
      if(!check_state_count)
      {
        CheckMasterState(ms);
        CheckMasterDomainState();
        CheckSlaveConfigurationState();
        check_state_count = PERIOD_US ;
      }
      ecrt_domain_queue(m_master_domain);                
      ecrt_master_sync_slave_clocks(m_master);
      ecrt_master_sync_reference_clock_to(m_master, TIMESPEC2NS(m_sync_timer));
      ecrt_master_send(m_master);

      try_counter++;
      check_state_count--;
    }
    else 
    {
      std::cerr << "[EthercatMaster] Error : Time out occurred while waiting for OP mode.!";
      return -1;
    }
  }
  return 0;
}

int8_t EthercatMaster::SdoRead(SDO_data& pack)
{
  if (ecrt_master_sdo_upload(m_master, pack.slave_position, pack.index, pack.sub_index, (uint8_t*)(&pack.data),
                             pack.data_sz, &pack.result_sz, &pack.err_code))
  {
    std::printf("[EthercatMaster] SDO read error, code: %d \n", &pack.err_code);
    return -1;
  }
  return 0;
}

int8_t EthercatMaster::SdoWrite(SDO_data& pack)
{
  if (ecrt_master_sdo_download(m_master, pack.slave_position, pack.index, pack.sub_index, (uint8_t*)(&pack.data),
                               pack.data_sz, &pack.err_code))
  {
    std::printf("[EthercatMaster] SDO write error, code : %d \n", &pack.err_code);
    return -1;
  }
  return 0;
}

// ---------------------------------------------------------------
// Functions belows are application specific

int EthercatMaster::MapDefaultPdos()
{
  // CKim - Code here is specific for MISS-Robot V3 which has 5 EPOS4 as a slaves, 
  // with absolute encoder enabled for motor 2 and 3

  /* Master 0, Slave 0, "EPOS4"
   * Vendor ID:       0x000000fb
   * Product code:    0x61500000
   * Revision number: 0x01600000
   */

  // CKim - First 5 entries will be read by slave (master sends command). RxPDO
  // The maximal allowed length for all mapped objects for PDO is 40 bytes for EtherCAT. See EPOS4 firmware manual
  ec_pdo_entry_info_t maxon_epos_RxPdo_entries[5] = { 
    { OD_CONTROL_WORD, 16 },  { OD_TARGET_VELOCITY, 32 }, { OD_TARGET_POSITION, 32 },
    { OD_TARGET_TORQUE, 16 }, { OD_TORQUE_OFFSET, 16 } };
    
  // CKim - next 8 entries will be transmitted by slave (master receives the data). TxPDO
  ec_pdo_entry_info_t maxon_epos_TxPdo_entries[9] = { 
    { OD_STATUS_WORD, 16 },             { OD_POSITION_ACTUAL_VAL, 32 },   { OD_VELOCITY_ACTUAL_VALUE, 32 },
    { OD_TORQUE_ACTUAL_VALUE, 16 },     { OD_ERROR_CODE, 16 },            {OD_ANALOG_INPUT_PROPERTIES_1, 16}, 
    {OD_ANALOG_INPUT_PROPERTIES_2, 16}, { OD_OPERATION_MODE_DISPLAY, 8 }, { OD_DIGITAL_INPUTS, 32 } };
  
  for (int i = 0; i < g_kNumberOfServoDrivers; i++)
  {
    // CKim - { PDO Index,  Number of item, Array of entry info }
    // 0x1600 : RxPDO1 / 0x1a00 : TxPDO 1
    ec_pdo_info_t maxon_RxPdos;    
    maxon_RxPdos.index = 0x1600;   maxon_RxPdos.n_entries = 5;  maxon_RxPdos.entries = maxon_epos_RxPdo_entries;

    ec_pdo_info_t maxon_TxPdos;
    maxon_TxPdos.index = 0x1a00;   maxon_TxPdos.n_entries = 9;  maxon_TxPdos.entries = maxon_epos_TxPdo_entries; 
    
    // CKim - Sync manager configuration of the EPOS4. 0,1 is reserved for SDO communications
    // EC_WD_ENABLE means that the sync manager of the slave will throw error
    // if it does not synchronize within certain interval
    ec_sync_info_t maxon_syncs[5] = { { 0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE },
                                      { 1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE },
                                      { 2, EC_DIR_OUTPUT, 1, &maxon_RxPdos, EC_WD_ENABLE },
                                      { 3, EC_DIR_INPUT, 1, &maxon_TxPdos, EC_WD_DISABLE },
                                      { 0xff } };

    if (ecrt_slave_config_pdos(slaves_[i].slave_config_, EC_END, maxon_syncs))
    {
      std::cerr << "[EthercatMaster] Slave " << i << " PDO configuration failed... \n";
      return -1;
    }

    // CKim - Registers a PDO entry for process data exchange in a domain. Obtain offsets
    this->slaves_[i].offset_.control_word =
        ecrt_slave_config_reg_pdo_entry(this->slaves_[i].slave_config_, OD_CONTROL_WORD, m_master_domain, NULL);
    this->slaves_[i].offset_.target_vel =
        ecrt_slave_config_reg_pdo_entry(this->slaves_[i].slave_config_, OD_TARGET_VELOCITY, m_master_domain, NULL);
    this->slaves_[i].offset_.target_pos =
        ecrt_slave_config_reg_pdo_entry(this->slaves_[i].slave_config_, OD_TARGET_POSITION, m_master_domain, NULL);
      this->slaves_[i].offset_.target_tor =
        ecrt_slave_config_reg_pdo_entry(this->slaves_[i].slave_config_, OD_TARGET_TORQUE, m_master_domain, NULL);
    this->slaves_[i].offset_.torque_offset =
        ecrt_slave_config_reg_pdo_entry(this->slaves_[i].slave_config_, OD_TORQUE_OFFSET, m_master_domain, NULL);


    this->slaves_[i].offset_.status_word = 
        ecrt_slave_config_reg_pdo_entry(this->slaves_[i].slave_config_, OD_STATUS_WORD, m_master_domain, NULL);
    this->slaves_[i].offset_.actual_pos =
        ecrt_slave_config_reg_pdo_entry(this->slaves_[i].slave_config_, OD_POSITION_ACTUAL_VAL, m_master_domain, NULL);
    this->slaves_[i].offset_.actual_vel = 
        ecrt_slave_config_reg_pdo_entry(this->slaves_[i].slave_config_, OD_VELOCITY_ACTUAL_VALUE, m_master_domain, NULL);
    this->slaves_[i].offset_.actual_tor =
        ecrt_slave_config_reg_pdo_entry(this->slaves_[i].slave_config_, OD_TORQUE_ACTUAL_VALUE, m_master_domain, NULL);
    this->slaves_[i].offset_.error_code =
        ecrt_slave_config_reg_pdo_entry(this->slaves_[i].slave_config_, OD_ERROR_CODE, m_master_domain, NULL);
    this->slaves_[i].offset_.op_mode_display = 
        ecrt_slave_config_reg_pdo_entry(this->slaves_[i].slave_config_, OD_OPERATION_MODE_DISPLAY, m_master_domain, NULL);
    this->slaves_[i].offset_.digital_input = 
      ecrt_slave_config_reg_pdo_entry(this->slaves_[i].slave_config_, OD_DIGITAL_INPUTS, m_master_domain, NULL);
    this->slaves_[i].offset_.analog_input_1 = 
      ecrt_slave_config_reg_pdo_entry(this->slaves_[i].slave_config_, OD_ANALOG_INPUT_PROPERTIES_1, m_master_domain, NULL);
    this->slaves_[i].offset_.analog_input_2 = 
      ecrt_slave_config_reg_pdo_entry(this->slaves_[i].slave_config_, OD_ANALOG_INPUT_PROPERTIES_2, m_master_domain, NULL);

    if ((slaves_[i].offset_.actual_pos < 0) || (slaves_[i].offset_.status_word < 0) ||
        (slaves_[i].offset_.actual_vel < 0) || (slaves_[i].offset_.target_vel < 0) ||
        (slaves_[i].offset_.target_pos < 0) || (slaves_[i].offset_.control_word < 0) ||
        (slaves_[i].offset_.target_tor < 0) || (slaves_[i].offset_.actual_tor < 0) ||
        (slaves_[i].offset_.torque_offset < 0) || (slaves_[i].offset_.error_code < 0) ||
        (slaves_[i].offset_.op_mode_display < 0) || (slaves_[i].offset_.digital_input < 0) || 
        (slaves_[i].offset_.analog_input_1 < 0) || (slaves_[i].offset_.analog_input_2 < 0) )
    {
      std::cerr << "[EthercatMaster] Failed to configure  PDOs for motors.!";
      return -1;
    }
  }

  return 0;
}

void EthercatMaster::ConfigDcSyncDefault()
{
  // CKim - Activating DC synchronization for slave.. 
  // Code here is specific for EPOS4. Assign activate parameter for  EPOS4 is 0x300
  // 0x300 for Elmo | and same for EasyCAT
  // Assign activate parameters specified in slaves ESI file

  for (int i = 0; i < g_kNumberOfServoDrivers; i++)
  {
    ecrt_slave_config_dc(slaves_[i].slave_config_, 0X0300, PERIOD_NS, slaves_[i].kSync0_shift_, 0, 0);
  }
#if CUSTOM_SLAVE
  ecrt_slave_config_dc(slaves_[FINAL_SLAVE].slave_config_, 0X0300, PERIOD_NS, 2000200000, 0, 0);
#endif
}

// ---------------------------------------------------------------

uint16_t EthercatMaster::ReadStatusWordViaSDO(int index)
{
  SDO_data pack;                    uint16_t status_word;
  pack.slave_position = index;      pack.index = OD_STATUS_WORD;
  pack.sub_index = 0;               pack.data_sz = sizeof(uint16_t);
  if (SdoRead(pack))
  {
    std::cerr << "Error while reading Status Word\n";
    return -1;
  }
  status_word = (uint16_t)(pack.data);
  return status_word;
}

int16_t EthercatMaster::WriteControlWordViaSDO(int index, uint16_t control_word)
{
  SDO_data pack;    
  pack.index = OD_CONTROL_WORD;     pack.slave_position = index;
  pack.sub_index = 0;               pack.data_sz = sizeof(uint16_t);
  pack.data = uint32_t(control_word);
  if (SdoWrite(pack))
  {
     std::cerr << "Error while writing Control Word\n ";
    return -1;
  }
  return 0;
}

uint8_t EthercatMaster::ReadOpModeViaSDO(int index)
{
  SDO_data pack;                    uint8_t op_mode;
  pack.slave_position = index;      pack.index = OD_OPERATION_MODE;
  pack.sub_index = 0;               pack.data_sz = sizeof(uint8_t);
  if (SdoRead(pack))
  {
    std::cerr << "Error while reading Operation Mode\n";
    return -1;
  }
  op_mode = (uint8_t)(pack.data);
  return op_mode;
}

int16_t EthercatMaster::WriteOpModeViaSDO(int index, uint8_t op_mode)
{
  SDO_data pack;                  pack.index = OD_OPERATION_MODE;
  pack.slave_position = index;    pack.sub_index = 0;
  pack.data_sz = sizeof(uint8_t); pack.data = uint32_t(op_mode);
  if (SdoWrite(pack))
  {
    std::cerr << "Error while writing Op Mode \n";
    return -1;
  }
  return 0;
}

int32_t EthercatMaster::ReadActualVelocityViaSDO(int index)
{
  SDO_data pack;                    int32_t actual_vel;
  pack.slave_position = index;      pack.index = OD_VELOCITY_ACTUAL_VALUE;
  pack.sub_index = 0;               pack.data_sz = sizeof(int32_t);
  if (SdoRead(pack))
  {
    std::cerr << "Error while reading Actual Velocity \n";
    return -1;
  }
  actual_vel = (int32_t)(pack.data);
  return actual_vel;
}

int16_t EthercatMaster::WriteTargetVelocityViaSDO(int index, int32_t target_vel)
{
  SDO_data pack;                    pack.index = OD_TARGET_VELOCITY;
  pack.slave_position = index;      pack.sub_index = 0;
  pack.data_sz = sizeof(int32_t);   pack.data = int32_t(target_vel);
  if (SdoWrite(pack))
  {
    std::cerr << "Error while writing Target Velocity\n";
    return -1;
  }
  if (WriteControlWordViaSDO(index, SM_RUN))
  {
    return -1;
  }
  return 0;
}

int32_t EthercatMaster::ReadActualPositionViaSDO(int index)
{
  SDO_data pack;                  int32_t actual_pos;
  pack.slave_position = index;    pack.index = OD_POSITION_ACTUAL_VAL;; //OD_MOTOR_RATED_TORQUE;
  pack.sub_index = 0;             pack.data_sz = sizeof(int32_t);
  if (SdoRead(pack))
  {
    std::cerr << "Error while reading Actual Position \n";
    return -1;
  }
  actual_pos = (int32_t)(pack.data);
  return actual_pos;
}

int16_t EthercatMaster::WriteTargetPositionViaSDO(int index, int32_t target_pos)
{
  SDO_data pack;                      pack.index = OD_TARGET_POSITION;
  pack.slave_position = index;        pack.sub_index = 0;
  pack.data_sz = sizeof(int32_t);     pack.data = int32_t(target_pos);
  if (SdoWrite(pack))
  {
    std::cerr << "Error while writing Target Position ";
    return -1;
  }
  if (TEST_BIT(ReadStatusWordViaSDO(index), 10) == 1)
  {
    WriteControlWordViaSDO(index, SM_RELATIVE_POS);
    WriteControlWordViaSDO(index, SM_GO_ENABLE);
  }
  else
  {
    if (WriteControlWordViaSDO(index, SM_GO_ENABLE))
    {
      return -1;
    }
  }
  return 0;
}

int16_t EthercatMaster::ReadActualTorqueViaSDO(int index)
{
  SDO_data pack;                int16_t actual_tor;
  pack.slave_position = index;  pack.index = OD_TORQUE_ACTUAL_VALUE;
  pack.sub_index = 0;           pack.data_sz = sizeof(int16_t);
  if (SdoRead(pack))
  {
    std::cerr << "Error while reading SDO \n";
    return -1;
  }
  actual_tor = (int16_t)(pack.data);
  return actual_tor;
}

int16_t EthercatMaster::WriteTargetTorqueViaSDO(int index, uint16_t target_tor)
{
  SDO_data pack;            
  pack.index = OD_TARGET_TORQUE;      pack.slave_position = index;
  pack.sub_index = 0;                 pack.data_sz = sizeof(int16_t);
  pack.data = uint32_t(target_tor); 
  if (SdoWrite(pack))
  {
    std::cerr << "Error while writing SDO \n";
    return -1;
  }
  if (WriteControlWordViaSDO(index, SM_RUN))
  {
    return -1;
  }
  return 0;
}

uint16_t EthercatMaster::ReadErrorCodeViaSDO(int index)
{
  SDO_data pack;
  uint16_t error_code;              pack.slave_position = index;
  pack.index = OD_ERROR_CODE;       pack.sub_index = 0;
  pack.data_sz = sizeof(uint16_t);
  if (SdoRead(pack))
  {
    std::cerr << "Error while reading Error Code \n";
    return -1;
  }
  error_code = (uint16_t)(pack.data);
  return error_code;
}

int32_t EthercatMaster::ReadAbsEncViaSDO(int index)
{
  SDO_data pack;                    int32_t abs_pos;
  pack.slave_position = index;      pack.index = OD_ADDITIONAL_POSITION_ACTUAL2;
  pack.sub_index = 2;               pack.data_sz = sizeof(int32_t);
  if (SdoRead(pack))
  {
    std::cerr << "Error while reading Absolute Encoder Position \n";
    return -1;
  }
  abs_pos = (int32_t)(pack.data);
  return abs_pos;
}

int16_t EthercatMaster::ReadAnalogInput1ViaSDO(int index)
{
    SDO_data pack ; 
    int16_t analog_input_1;  
    pack.slave_position = index;
    pack.index = OD_ANALOG_INPUT_PROPERTIES_1 ; 
    pack.sub_index = 1 ;
    pack.data_sz = sizeof(int16_t);
    if(SdoRead(pack)){
       std::cout << "Error while reading Analog Input #1 " << std::endl;
        return -1; 
    }
    analog_input_1 = (int16_t)(pack.data);
    //if(index==6)  { std::cout << "Sparta : " << analog_input_1 << std::endl;  }
    return analog_input_1 ; 
}

int16_t EthercatMaster::ReadAnalogInput2ViaSDO(int index)
{
    SDO_data pack ; 
    int16_t analog_input_2;  
    pack.slave_position = index;
    pack.index = OD_ANALOG_INPUT_PROPERTIES_2 ; 
    pack.sub_index = 2 ;
    pack.data_sz = sizeof(int16_t);
    if(SdoRead(pack)){
       std::cout << "Error while reading Analog Input #2 " << std::endl;
        return -1; 
    }
    analog_input_2 = (int16_t)(pack.data);
    return analog_input_2 ; 
}

uint32_t EthercatMaster::ReadDigitalInputViaSDO(int index)
{
    SDO_data pack ; 
    uint32_t digital_input;  
    pack.slave_position = index;
    pack.index = OD_DIGITAL_INPUTS;
    pack.sub_index = 0 ;
    pack.data_sz = sizeof(uint32_t);
    if(SdoRead(pack)){
       std::cout << "Error while reading Digital Input " << std::endl;
        return -1; 
    }
    digital_input = (uint32_t)(pack.data);
    return digital_input;
}

uint16_t EthercatMaster::ClearFaultsViaSDO(int index)
{
  std::printf("Clearing faults for motor %d ...  ", index + 1);
  WriteControlWordViaSDO(index, SM_START);
  WriteControlWordViaSDO(index, SM_FULL_RESET);
}

int EthercatMaster::SetProfilePositionParameters(int position, ProfilePosParam& P)
{
  // Operation mode to ProfilePositionMode for slave on that position.
  if (ecrt_slave_config_sdo8(slaves_[position].slave_config_, OD_OPERATION_MODE, kProfilePosition))
  {
    std::cerr << "Set operation mode config error ! ";
    return -1;
  }
  // profile velocity
  if (ecrt_slave_config_sdo32(slaves_[position].slave_config_, OD_PROFILE_VELOCITY, P.profile_vel) < 0)
  {
    std::cerr << "Set profile velocity config error ! ";
    return -1;
  }
  // max profile velocity
  if (ecrt_slave_config_sdo32(slaves_[position].slave_config_, OD_MAX_PROFILE_VELOCITY, P.max_profile_vel) < 0)
  {
    std::cerr << "Set max profile velocity config error !";
    return -1;
  }
  // profile acceleration
  if (ecrt_slave_config_sdo32(slaves_[position].slave_config_, OD_PROFILE_ACCELERATION, P.profile_acc) < 0)
  {
    std::cerr << "Set profile acceleration failed ! ";
    return -1;
  }
  // profile deceleration
  if (ecrt_slave_config_sdo32(slaves_[position].slave_config_, OD_PROFILE_DECELERATION, P.profile_dec) < 0)
  {
    std::cerr << "Set profile deceleration failed ! ";
    return -1;
  }
  // quick stop deceleration
  if (ecrt_slave_config_sdo32(slaves_[position].slave_config_, OD_QUICK_STOP_DECELERATION, P.quick_stop_dec) < 0)
  {
    std::cerr << "Set quick stop deceleration failed !";
    return -1;
  }
  // max following error
  if (ecrt_slave_config_sdo32(slaves_[position].slave_config_, OD_MAX_FOLLOWING_ERROR, P.max_fol_err) < 0)
  {
    std::cerr << "Set max following error failed ! ";
    return -1;
  }
  return 0;
}

int EthercatMaster::SetProfileVelocityParameters(int position, ProfileVelocityParam& P)
{
  // Set operation mode to ProfileVelocityMode for slave on that position.
  if (ecrt_slave_config_sdo8(slaves_[position].slave_config_, OD_OPERATION_MODE, kProfileVelocity))
  {
    std::cerr << "Set operation mode config error ! ";
    return -1;
  }
  // motionProfileType
  if (ecrt_slave_config_sdo16(slaves_[position].slave_config_, OD_MOTION_PROFILE_TYPE, P.motion_profile_type) < 0)
  {
    std::cerr << "Set profile velocity config error ! ";
    return -1;
  }
  // max profile velocity
  if (ecrt_slave_config_sdo32(slaves_[position].slave_config_, OD_MAX_PROFILE_VELOCITY, P.max_profile_vel) < 0)
  {
    std::cerr << "Set max profile  velocity config error ! ";
    return -1;
  }
  // profile acceleration
  if (ecrt_slave_config_sdo32(slaves_[position].slave_config_, OD_PROFILE_DECELERATION, P.profile_dec) < 0)
  {
    std::cerr << "Set profile deceleration failed !";
    return -1;
  }
  // profile deceleration
  if (ecrt_slave_config_sdo32(slaves_[position].slave_config_, OD_PROFILE_ACCELERATION, P.profile_acc) < 0)
  {
    std::cerr << "Set profile acceleration failed ! ";
    return -1;
  }
  // quick stop deceleration
  if (ecrt_slave_config_sdo32(slaves_[position].slave_config_, OD_QUICK_STOP_DECELERATION, P.quick_stop_dec) < 0)
  {
    std::cerr << "Set quick stop deceleration failed ! ";
    return -1;
  }
  return 0;
}

int EthercatMaster::SetCyclicSyncPositionModeParameters(int position, CSPositionModeParam& P)
{
  // Set operation mode to Cyclic Synchronous Position mode for motor in specified physical position w.r.t master.
  if (ecrt_slave_config_sdo8(slaves_[position].slave_config_, OD_OPERATION_MODE, kCSPosition))
  {
    std::cerr << "Set operation mode config error ! ";
    return -1;
  }
  // quick stop deceleration
  if (ecrt_slave_config_sdo32(slaves_[position].slave_config_, OD_QUICK_STOP_DECELERATION, P.quick_stop_dec) < 0)
  {
    std::cerr << "Set quick stop deceleration failed !";
    return -1;
  }
  // Following Error
  if (ecrt_slave_config_sdo32(slaves_[position].slave_config_, OD_MAX_FOLLOWING_ERROR, P.max_fol_err) < 0)
  {
    std::cerr << "Set max following error failed !";
    return -1;
  }
  // Interpolation time period is 1ms by default. Default unit is milliseconds (ms)
  if (ecrt_slave_config_sdo8(slaves_[position].slave_config_, OD_INTERPOLATION_TIME_PERIOD,
                             P.interpolation_time_period) < 0)
  {
    std::cerr << "Set interpolation time period failed !";
    return -1;
  }
  return 0;
}

int EthercatMaster::SetCyclicSyncVelocityModeParameters(int position, CSVelocityModeParam& P)
{
  // Set operation mode to Cyclic Synchronous Velocity mode for motor in specified physical position w.r.t master.
  if (ecrt_slave_config_sdo8(slaves_[position].slave_config_, OD_OPERATION_MODE, kCSVelocity))
  {
    std::cerr << "Set operation mode config error ! ";
    return -1;
  }
  // // Velocity control parameter set, P, I gain only
  // if (ecrt_slave_config_sdo32(slaves_[position].slave_config_, OD_VELOCITY_CONTROLLER_PGAIN,
  //                             P.velocity_controller_gain.Pgain) < 0)
  // {
  //   std::cerr << "Set velocity Pgain failed ! ");
  //   return -1;
  // }
  // if (ecrt_slave_config_sdo32(slaves_[position].slave_config_, OD_VELOCITY_CONTROLLER_IGAIN,
  //                             P.velocity_controller_gain.Igain) < 0)
  // {
  //   std::cerr << "Set velocity Igain failed ! ");
  //   return -1;
  // }
  // profile deceleration
  if (ecrt_slave_config_sdo32(slaves_[position].slave_config_, OD_PROFILE_DECELERATION, P.profile_dec) < 0)
  {
    std::cerr << "Set profile deceleration failed ! ";
    return -1;
  }
  // quick stop deceleration
  if (ecrt_slave_config_sdo32(slaves_[position].slave_config_, OD_QUICK_STOP_DECELERATION, P.quick_stop_dec) < 0)
  {
    std::cerr << "Set quick stop deceleration failed !";
    return -1;
  }
  // Interpolation time period is 1ms by default.Default unit is milliseconds (ms)
  if (ecrt_slave_config_sdo8(slaves_[position].slave_config_, OD_INTERPOLATION_TIME_PERIOD,
                             P.interpolation_time_period) < 0)
  {
    std::cerr << "Set interpolation time period failed !";
    return -1;
  }
  return 0;
}

int EthercatMaster::SetCyclicSyncTorqueModeParameters(int position, CSTorqueModeParam& P)
{
  // Set operation mode to Cyclic Synchronous Velocity mode for motor in specified physical position w.r.t master.
  if (ecrt_slave_config_sdo8(slaves_[position].slave_config_, OD_OPERATION_MODE, kCSTorque))
  {
    std::cerr << "Set operation mode config error ! ";
    return -1;
  }
  // profile deceleration
  if (ecrt_slave_config_sdo32(slaves_[position].slave_config_, OD_PROFILE_DECELERATION, P.profile_dec) < 0)
  {
    std::cerr << "Set profile deceleration failed ! ";
    return -1;
  }
  // quick stop deceleration
  if (ecrt_slave_config_sdo32(slaves_[position].slave_config_, OD_QUICK_STOP_DECELERATION, P.quick_stop_dec) < 0)
  {
    std::cerr << "Set quick stop deceleration failed !";
    return -1;
  }
  
  // if (ecrt_slave_config_sdo16(slaves_[position].slave_config_, OD_MAX_TORQUE, P.max_torque) < 0)
  // {
  //   std::cerr << "Set max torque failed !";
  //   return -1;
  // }
  // if (ecrt_slave_config_sdo32(slaves_[position].slave_config_, OD_MAX_MOTOR_SPEED, P.max_profile_vel) < 0)
  // {
  //   std::cerr << "Set max motor speed failed !";
  //   return -1;
  // }
  return 0;
}

int EthercatMaster::HomeMotor(int position, HomingParam& H)
{
  // Enable Homing Mode
  if (WriteOpModeViaSDO(position, kHoming) < 0)
  {
    std::cerr << "Changing to homing mode failed ! ";
    return -1;
  };

  // Homing Method
  {
    SDO_data pack;
    pack.index = OD_HOMING_METHOD;              pack.slave_position = position;
    pack.sub_index = 0;                         pack.data_sz = sizeof(uint8_t);
    pack.data = uint8_t(H.homing_method);
    if (SdoWrite(pack))
    {
      std::cerr << "Error while writing homing method\n ";
      return -1;
    }
  }

  // Switch search speed
  {
    SDO_data pack;
    pack.index = 0x6099;                pack.slave_position = position;
    pack.sub_index = 0x01;              pack.data_sz = sizeof(uint32_t);
    pack.data = uint32_t(H.speed_for_switch_search);
    if (SdoWrite(pack))
    {
      std::cerr << "Error while writing homing switch search speed\n ";
      return -1;
    }
  }

  // // Switch search speed
  // if (ecrt_slave_config_sdo32(slaves_[position].slave_config_,OD_HOMING_SWITCH_SEARCH_SPEED, H.speed_for_switch_search) < 0)
  // {
  //   std::cerr << "Set homing switch search speed failed ! ");
  //   return -1;
  // }



  // Zero search speed
  {
    SDO_data pack;
    pack.index = 0x6099;                pack.slave_position = position;
    pack.sub_index = 0x02;              pack.data_sz = sizeof(uint32_t);
    pack.data = uint32_t(H.speed_for_zero_search);
    if (SdoWrite(pack))
    {
      std::cerr << "Error while writing homing zero search speed\n ";
      return -1;
    }
  }

  // Offset move distance
  {
    SDO_data pack;
    pack.index = 0x30B1;                pack.slave_position = position;
    pack.sub_index = 0;                 pack.data_sz = sizeof(uint32_t);
    pack.data = uint32_t(H.home_offset);
    if (SdoWrite(pack))
    {
      std::cerr << "Error while writing homing zero search speed\n ";
      return -1;
    }
  }

  //  CKim - Start homing by changing control word
  if (WriteControlWordViaSDO(position, SM_RUN) < 0)
  {
    std::cerr << "Failed to start homing procedure ! ";
    return -1;
  }
  std::cout << "Homing Started";
  return 0;
}

