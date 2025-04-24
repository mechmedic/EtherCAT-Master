#include "EposNtwk.hpp"

using namespace EthercatCommunication;

EposNtwk::EposNtwk()
{
    /* Master 0, Slave 0, "EPOS4"
     * Vendor ID:       0x000000fb
     * Product code:    0x61500000
     * Revision number: 0x01600000
     */
}

EposNtwk::~EposNtwk()
{
}

int EposNtwk::MapPdos()
{
  // To map PDOs
  // 1. Set 'ec_pdo_entry_info_t' which specifies index/subindex/size of
  //    an object (PDO Entry) that will be mapped to PDO
  // 2. Set 'ec_pdo_info_t' which specifies index in a slave's object dictionary
  //    (PDO index) that the entry information will be stored.
  //    {PDOidx, number of PDO entry, pointer to pdo entries}
  // 3. Set 'ec_sync_info_t', sync manager configuration information
  //    index / direction / number of PDOs to be managed / PDO info / watchdog mode
  //    SyncManagers ensure a consistent and secure data exchange between the
  //    EtherCAT master and the local application of a slave device.
  //    The EtherCAT master configures the SyncManager of each slave device;
  //    it determines the direction and method of communication.
  //    https://infosys.beckhoff.com/english.php?content=../content/1033/el5101/10715752587.html&id=
  //    https://infosys.beckhoff.com/english.php?content=../content/1033/tc3_io_intro/4981170059.html&id=
  // 4. Registers a PDO entry for process data exchange in a domain.
  //    This gives an offset from the Process Data Domain which we can use to access PDO data

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
    // {0xff} at the end is used tell the end of 'ec_sync_info_t' list
    ec_sync_info_t maxon_syncs[5] = { { 0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE },
                                      { 1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE },
                                      { 2, EC_DIR_OUTPUT, 1, &maxon_RxPdos, EC_WD_ENABLE },
                                      { 3, EC_DIR_INPUT, 1, &maxon_TxPdos, EC_WD_DISABLE },
                                      { 0xff } };

    // CKim - Connect the configured Sync manager to corresponding slaves
    // ecrt_slave_config_pdos (slave configuration, number of sync manager,
    // array of sync manager configuration):
    // EC_END tells to read until {oxff} in 'ec_sync_info_t' array
    if (ecrt_slave_config_pdos(slaves_[i].slave_config_, EC_END, maxon_syncs))
    {
      std::cerr << "[EposNtwk] Slave " << i << " PDO configuration failed... \n";
      return -1;
    }

    // CKim - Registers a PDO entry for process data exchange in a domain. Obtain offsets
    // of each slave to Process Data Domain. Only the registered entry will be communicated by master.
    // offset = ecrt_slave_config_reg_pdo_entry
    // (slave configuration, index, subindex, domain)
    // Returns offset (in bytes) of the PDO entry's process data from the beginning of the
    // domain data, which is used for read/write

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
      std::cerr << "[EposNtwk] Failed to configure  PDOs for motors.!";
      return -1;
    }
  }

  return 0;
}

void EposNtwk::ConfigDcSyncDefault()
{
  // CKim - Activating DC synchronization for slave..
  // Code here is specific for EPOS4. Assign activate parameter for  EPOS4 is 0x300
  // 0x300 for Elmo | and same for EasyCAT
  // Assign activate parameters specified in slaves ESI file
  // https://infosys.beckhoff.com/english.php?content=../content/1033/ethercatsystem/2469118347.html&id=
  // https://infosys.beckhoff.com/english.php?content=../content/1033/ethercatsystem/2469122443.html&id=
  for (int i = 0; i < g_kNumberOfServoDrivers; i++)
  {
    ecrt_slave_config_dc(slaves_[i].slave_config_, 0X0300, PERIOD_NS, slaves_[i].kSync0_shift_, 0, 0);
  }
#if CUSTOM_SLAVE
  ecrt_slave_config_dc(slaves_[FINAL_SLAVE].slave_config_, 0X0300, PERIOD_NS, 2000200000, 0, 0);
#endif
}

void EposNtwk::ReadFromSlaves()
{
    for (int i = 0; i < g_kNumberOfServoDrivers; i++)
    {
        m_PdoData[i].actual_pos =
                EC_READ_S32(slaves_[i].slave_pdo_domain_ + slaves_[i].offset_.actual_pos);
        m_PdoData[i].actual_vel =
                EC_READ_S32(slaves_[i].slave_pdo_domain_ + slaves_[i].offset_.actual_vel);
        m_PdoData[i].status_word =
                EC_READ_U16(slaves_[i].slave_pdo_domain_ + slaves_[i].offset_.status_word);
        m_PdoData[i].actual_tor =
                EC_READ_S16(slaves_[i].slave_pdo_domain_ + slaves_[i].offset_.actual_tor);
        m_PdoData[i].error_code =
                EC_READ_U16(slaves_[i].slave_pdo_domain_ + slaves_[i].offset_.error_code);
        m_PdoData[i].digital_input =
                EC_READ_U32(slaves_[i].slave_pdo_domain_ + slaves_[i].offset_.digital_input);
        m_PdoData[i].op_mode_display =
                EC_READ_U8(slaves_[i].slave_pdo_domain_ + slaves_[i].offset_.op_mode_display);

        // DY
        m_PdoData[i].analog_input_1 =
                EC_READ_S16(slaves_[i].slave_pdo_domain_ + slaves_[i].offset_.analog_input_1);
        m_PdoData[i].analog_input_2 =
                EC_READ_S16(slaves_[i].slave_pdo_domain_ + slaves_[i].offset_.analog_input_2);
    }
}

void EposNtwk::WriteToSlaves()
{
    for (int i = 0; i < g_kNumberOfServoDrivers; i++)
    {
        EC_WRITE_U16(slaves_[i].slave_pdo_domain_ + slaves_[i].offset_.control_word,
                     m_PdoData[i].control_word);
        EC_WRITE_S32(slaves_[i].slave_pdo_domain_ + slaves_[i].offset_.target_vel, m_PdoData[i].target_vel);
    }
}


// ---------------------------------------------------------------

uint16_t EposNtwk::ReadStatusWordViaSDO(int index)
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

int16_t EposNtwk::WriteControlWordViaSDO(int index, uint16_t control_word)
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

uint8_t EposNtwk::ReadOpModeViaSDO(int index)
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

int16_t EposNtwk::WriteOpModeViaSDO(int index, uint8_t op_mode)
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

int32_t EposNtwk::ReadActualVelocityViaSDO(int index)
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

int16_t EposNtwk::WriteTargetVelocityViaSDO(int index, int32_t target_vel)
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

int32_t EposNtwk::ReadActualPositionViaSDO(int index)
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

int16_t EposNtwk::WriteTargetPositionViaSDO(int index, int32_t target_pos)
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

int16_t EposNtwk::ReadActualTorqueViaSDO(int index)
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

int16_t EposNtwk::WriteTargetTorqueViaSDO(int index, uint16_t target_tor)
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

uint16_t EposNtwk::ReadErrorCodeViaSDO(int index)
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

int32_t EposNtwk::ReadAbsEncViaSDO(int index)
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

int16_t EposNtwk::ReadAnalogInput1ViaSDO(int index)
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

int16_t EposNtwk::ReadAnalogInput2ViaSDO(int index)
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

uint32_t EposNtwk::ReadDigitalInputViaSDO(int index)
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

uint16_t EposNtwk::ClearFaultsViaSDO(int index)
{
  std::printf("Clearing faults for motor %d ...  ", index + 1);
  WriteControlWordViaSDO(index, SM_START);
  WriteControlWordViaSDO(index, SM_FULL_RESET);
}

int EposNtwk::SetProfilePositionParameters(int position, ProfilePosParam& P)
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

int EposNtwk::SetProfileVelocityParameters(int position, ProfileVelocityParam& P)
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

int EposNtwk::SetCyclicSyncPositionModeParameters(int position, CSPositionModeParam& P)
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

int EposNtwk::SetCyclicSyncVelocityModeParameters(int position, CSVelocityModeParam& P)
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

int EposNtwk::SetCyclicSyncTorqueModeParameters(int position, CSTorqueModeParam& P)
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

int EposNtwk::HomeMotor(int position, HomingParam& H)
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

