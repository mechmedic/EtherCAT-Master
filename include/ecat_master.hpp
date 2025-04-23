/******************************************************************************
 *
 *  $Id$
 *
 *  Copyright (C) 2021 Veysi ADIN, UST KIST
 *
 *  This file is part of the IgH EtherCAT master userspace program in the ROS2 environment.
 *
 *  The IgH EtherCAT master userspace program in the ROS2 environment is free software; you can
 *  redistribute it and/or modify it under the terms of the GNU General
 *  Public License as published by the Free Software Foundation; version 2
 *  of the License.
 *
 *  The IgH EtherCAT master userspace program in the ROS2 environment is distributed in the hope that
 *  it will be useful, but WITHOUT ANY WARRANTY; without even the implied
 *  warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with the IgH EtherCAT master userspace program in the ROS environment. If not, see
 *  <http://www.gnu.org/licenses/>.
 *
 *  ---
 *
 *  The license mentioned above concerns the source code only. Using the
 *  EtherCAT technology and brand is only permitted in compliance with the
 *  industrial property and similar rights of Beckhoff Automation GmbH.
 *
 *  Contact information: veysi.adin@kist.re.kr
 *****************************************************************************/
/******************************************************************************
 *  \file   ecat_master.hpp
 *  \brief  IgH EtherCAT library functionality wrapper class header file.
 *
 *   This header file contains blueprint of EthercatMaster class which will be
 *   responsible for encapsulating IgH EtherCAT library's functionality.
 *******************************************************************************/
#pragma once
/******************************************************************************/
#include "ecat_globals.hpp"
/******************************************************************************/
/// Forward declaration of EthercatSlave class.
class EthercatSlave;
#include "ecat_slave.hpp"
/******************************************************************************/
/// C++ Headers
#include <iostream>
#include <cstdio>
#include <vector>
/*****************************************************************************/
namespace EthercatCommunication
{
class EthercatMaster
{
public:
  EthercatMaster();
  ~EthercatMaster();
  EthercatSlave slaves_[NUM_OF_SLAVES];
  //std::vector<SdoRequest> request_sdos_;

  /**
   * @brief Opens EtherCAT master via command line tool if it's not already on.
   * @return 0 if successfull, otherwise -1.
   */
  int OpenEthercatMaster();

  /**
   * @brief Requests master instance and creates a domain for a master.
   * @note  Keep in mind that created master and domain are global variables.
   * @param idx Index of the master in case of multiple masters
   * @return 0 if succesful otherwise -1.
   */
  int ConfigureMaster(int idx);
  
  /**
   * @brief Deactivates and releases master shouldn't be called in real-time.
   */
  void ReleaseMaster();

  /**
   * @brief Shutdowns EtherCAT master via command line tool if it's not already off.
   * @return 0 if successfull, otherwise -1.
   */
  int ShutDownEthercatMaster();

  /**
   * @brief Restarts Ethercat master via command line tool.
   * @return 0 if successfull, otherwise -1.
   */
  int RestartEthercatMaster();

  /**
   * @brief This function will check master's state, in terms of number of responding slaves and 
   * their application layer states   *
   * @return 0 if succesful, otherwise -1
   * \see ec_master_state_t structure.
   **/
  int CheckMasterState(ec_master_state_t& ms);
  
  /**
   * @brief  Reads the state of a domain.
   * Stores the domain state in the given state structure.
   * Using this method, the process data exchange can be monitored in realtime.
   */
  void CheckMasterDomainState();

  /**
   * @brief Activates master, after this function call realtime operation can start.
   * \warning Before activating master all configuration should be done
   * \warning After calling this function you have to register domain(s) and start realtime task.
   * @return 0 if succesful, otherwise -1.
   */
  int ActivateMaster();

  /**
   * @brief Registers domain for each slave.
   *  This method has to be called after ecrt_master_activate() to get the mapped domain process data memory.
   * @return 0 if succeful , otherwise -1
   */
  int RegisterDomain();

  /**
   * @brief Deactivates master, and it is used to stop cyclic PDO exchange.
   * @note That calling this function means that your PDO exchange will stop.
   * @note Additionally all pointers created by requesting master are freed with this function./
   * If you want to resume your communication, you'll have to do the configuration again.
   */
  void DeactivateCommunication();
  
  /**
   * @brief Get the Number Of physically Connected Slaves to the bus.
   *        And checks if specified NUM_OF_SLAVES is correct.
   * @return 0 if NUM_OF_SLAVES setting is correct, otherwise -1.
   */
  int GetNumberOfConnectedSlaves();

  /**
   * @brief Get the information of physically connected slaves to the master.
   *        This function will return connected slave's vendor id, product code.
   */
  void GetSlaveInformation(int position, ec_slave_info_t& info);
  
  /**
   * @brief Configures i th slave.
   * @return 0 if successfull, otherwise -1. 
   */
  int InitSlave(int position);
  
  /**
   * @brief This function will check slave's application layer states. (INIT/PREOP/SAFEOP/OP)
   */
  void CheckSlaveConfigurationState();
  
  /**
   * @brief Reset i'th slaves
   */
  void ResetSlave(int position);

  /**
   * @brief This function will receive data from slaves and update data domain.
   *        Call this before reading data from slaves
   */
  void ReceiveAndProcess();

  /**
   * @brief This function will update data domain and send data to the slaves.
   *        Call this to after updating the data to be sent to the slaves
   */
  void QueueAndSend();

  /**
   * @brief Set Application time
  */
  void SetMasterApplicationTime(const timespec& time);

  /**
   * @brief Set Master Reference Clock
  */
  void SyncMasterReferenceClock(const timespec& time);

  /**
   * @brief Set slave clock
   */
  void SyncSlaveClock();

  /**
   * @brief Puts all slave to operational mode. User must call this before entering real-time operation.
   *        Reason for this function is that, master and slave has to do several exchange before becoming operational.
   *        So this function does exchange between master and slaves for up to 10 sec, could finish earlier.
   *        If timeout occurs it will return -1.
   * @return 0 if successfull, otherwise -1.
   */
  int WaitForOperationalMode();

  /**
   * @brief This function reads data from specified index and subindex via SDO, returned data will be stored in
   * pack.data which needs to be casted to correct data type afterwards.
   * \see @SDO_data struct
   * \note This is a blocking function, until response has been received. Don't use it in real-time context!!.
   *
   * @param pack SDO_data struct which contains; index,subindex,data size, and data that will be used to store read
   * data.
   * @return 0 if successfull, otherwise -1.
   */
  int8_t SdoRead(SDO_data& pack);

  /**
   * @brief This function writes data to specified index and subindex via SDO.
   * \see @SDO_data struct
   * \note This is a blocking function, until response has been received.Don't use it in real-time context!!.
   *
   * @param pack SDO_data struct which contains; index,subindex,data size and data to be written.
   * @return 0 if successfull, otherwise -1.
   */
  int8_t SdoWrite(SDO_data& pack);

  // ---------------------------------------------------------------
  // Functions belows are application specific

  /**
   * @brief Maps default PDOs for the application.
   * @note This method is specific to our implementation. However you can modify it to suit your needs.
   * If you have different topology or different servo drives use MapCustomPdos()
   * function or modify this function based on your needs.
   * @return 0 if successfull, otherwise -1.
   */
  int MapDefaultPdos();

  /**
   * @brief Configures DC sync for our default configuration
   *
   */
  void ConfigDcSyncDefault();

  // ---------------------------------------------------------------

  /**
   * @brief Get the Status Word from CiA402 slaves in specified index via SDO communication.
   * @param index slave index
   * @return status word
   */
  uint16_t ReadStatusWordViaSDO(int index);

  /**
   * @brief Writes control word to slave in specified index via SDO.
   *
   * @param index slave index, @param control_word control word to be written
   * @return 0 if successfull otherwise -1.
   */
  int16_t WriteControlWordViaSDO(int index, uint16_t control_word);
  
  /**
   * @brief Writes desired operational mode, to slave in specified index via SDO.
   *
   * @param index slave index
   * @param op_mode desired operational mode. @see OpMode enum defined in @file ecat_definitions.hpp
   * @return 0 if successfull, otherwise -1.
   */
    int16_t WriteOpModeViaSDO(int index, uint8_t op_mode);
  
  /**
   * @brief Reads current operational mode from slave in specified index via SDO.
   *
   * @param index slave index
   * @return status word of selected slave.
   */
  uint8_t ReadOpModeViaSDO(int index);
  
  /**
   * @brief Reads actual velocity value from slave in specified index via SDO.
   *
   * @param index slave index
   * @return actual velocity value of selected slave.
   */
  int32_t ReadActualVelocityViaSDO(int index);
  /**
   * @brief Writes target velocity value via SDO to slave in specified index.
   *
   * @param index slave index
   * @param target_vel desired target velocity val.
   * @return 0 if successfull, othewise -1.
   */
  int16_t WriteTargetVelocityViaSDO(int index, int32_t target_vel);
  /**
   * @brief Read actual position from slave in specified index via SDO.
   *
   * @param index slave index
   * @return actual position of selected slave.
   */
  int32_t ReadActualPositionViaSDO(int index);
  /**
   * @brief Writes target position value via SDO to slave in specified index.
   *
   * @param index slave index.
   * @param target_pos desired target position value.
   * @return 0 if successfull, otherwise -1.
   */
  int16_t WriteTargetPositionViaSDO(int index, int32_t target_pos);
  /**
   * @brief Read actual torque value from slave in specified index via SDO.
   *
   * @param index slave index
   * @return actual torque value of selected slave.
   */
  int16_t ReadActualTorqueViaSDO(int index);
  /**
   * @brief Writes target torque value to slave in specified index via SDO.
   *
   * @param index slave index
   * @param target_tor desired target torque value.
   * @return 0 if successfull, otherwise -1.
   */
  int16_t WriteTargetTorqueViaSDO(int index, uint16_t target_tor);
  /**
   * @brief Enable CiA402 supported motor drives in specified index via SDO.
   *
   * @param index slave index
   * @return 0 if successfull, otherwise -1.
   */
  int16_t EnableDrivesViaSDO(int index);
  /**
   * @brief Disable CiA402 supported motor drives in specified index via SDO.
   *
   * @param index slave index
   * @return 0 if successfull, otherwise -1.
   */
  int16_t DisableDrivesViaSDO(int index);
  /**
   * @brief Reads error register value from slave in specified index via SDO.
   * 
   * @param index slave index
   * @return error register value of the selected slave.
   */
  uint16_t ReadErrorCodeViaSDO(int index);
  /**
   * @brief Read absolute Encoder Via SDO
   * 
   * @param index slave index
   * @return 0 if successfull, otherwise -1.
   */
  int32_t ReadAbsEncViaSDO(int index);
  /**
   * @brief Read analog input via SDO
   * 
   * @param index slave index
   * @return 0 if successfull, otherwise -1.
   */
  int16_t ReadAnalogInput1ViaSDO(int index);
  /**
   * @brief Read analog input via SDO
   * 
   * @param index slave index
   * @return 0 if successfull, otherwise -1.
   */
  int16_t ReadAnalogInput2ViaSDO(int index);
  /**
   * @brief Read Digital Input via SDO
   * 
   * @param index slave index
   * @return 0 if successfull, otherwise -1.
   */
  uint32_t ReadDigitalInputViaSDO(int index);
  /**
   * @brief Clears all faults by sending control word clear fault command.
   * 
   * @param index slave index
   * @return 0 if successfull, otherwise -1.
   */
  uint16_t ClearFaultsViaSDO(int index);

  /**
   * @brief Set mode to ProfilePositionMode with specified parameters for servo drive on that position.
   *
   * @param position Slave position
   * @param P Profile position parameter structure specified by user.
   * @return 0 if successfull, otherwise -1.
   */
  int SetProfilePositionParameters(int position, ProfilePosParam& P);
  
  /**
   * @brief Set mode to ProfileVelocityMode with specified parameters for servo drive on that position.
   *
   * @param position Slave position
   * @param P Profile velocity parameter structure specified by user.
   * @return 0 if succesful, -1 otherwise.
   */
  int SetProfileVelocityParameters(int position, ProfileVelocityParam& P);
 
  /**
   * @brief Set the Cyclic Sync Position Mode Parameters for slave in specified physical position w.r.t. master.
   *
   * @param position Physical position of slave to be configured
   * @param P Cyclic Sync. Position Mode Parameters.
   * @return 0 if sucessfull, otherwise -1.
   */
  int SetCyclicSyncPositionModeParameters(int position, CSPositionModeParam& P);
  
  /**
   * @brief Set the Cyclic Sync Velocity Mode Parameters for slave in specified physical position w.r.t. master.
   *
   * @param position Physical position of slave to be configured
   * @param P Cyclic Sync. Velocity Mode Parameters.
   * @return 0 if sucessfull, otherwise -1.
   */
  int SetCyclicSyncVelocityModeParameters(int position, CSVelocityModeParam& P);
  
  /**
   * @brief Set the Cyclic Sync Torque Mode Parameters for slave in specified physical position w.r.t. master.
   *
   * @param position Physical position of slave to be configured
   * @param P Cyclic Sync. Torque Mode Parameters.
   * @return 0 if sucessfull, otherwise -1.
   */
  int SetCyclicSyncTorqueModeParameters(int position, CSTorqueModeParam& P);

  /**
   * @brief Set the Homing Mode and Start Homing.
   *
   * @param H Homing Mode Parameters.
   * @param position Physical position of slave to be configured
   * @return 0 if sucessfull, otherwise -1.
   */
  int HomeMotor(int position, HomingParam& H);

  
private:
  /// File descriptor to open and wake  master from CLI.
  int fd;

  ec_master_t* m_master = NULL;                  // EtherCAT master instance
  ec_master_state_t m_master_state = {};         // EtherCAT master state
  ec_domain_t* m_master_domain = NULL;           // Ethercat data passing master domain
  ec_domain_state_t m_master_domain_state = {};  // EtherCAT master domain state
  struct timespec m_sync_timer;
};
}  // namespace EthercatCommunication


   /**
   * @brief Maps default SDOs which can be found @see SdoRequest struct.
   * @return 0 if successfull, otherwise -1.
   */
  // int MapDefaultSdos();

  // /**
  //  * @brief Writes SDO in real-time context.
  //  *
  //  * @param req
  //  * @param data
  //  */
  // void WriteSDO(ec_sdo_request_t* req, int32_t data, int size);
  // /**
  //  * @brief Reads SDO in real-time context by creating read request.
  //  *
  //  * @param req
  //  */
  // uint16_t ReadSDO(ec_sdo_request_t* req, uint16_t& status_word);

