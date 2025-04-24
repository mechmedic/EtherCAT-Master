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
 *  \file   EthercatNtwk.hpp
 *  \brief  IgH EtherCAT library functionality wrapper class header file.
 *
 *   This header file contains base class of EthercatNtwk that consists of
 *   EtherCAT master using IgH EtherLab library and connected slaves.
 *   Inherit this class and override MapPdos(), ConfigDcSyncDefault()
 *   ReadFromSlaves() and WriteToSlaves(), and also add additional functions
 *   for configuring slaves, depending on the specific connected slaves
 *******************************************************************************/
#pragma once
/******************************************************************************/
#include "ecat_globals.hpp"
#include "EthercatSlave.hpp"
#include <ecrt.h>
/******************************************************************************/
/// C++ Headers
#include <iostream>
#include <cstdio>
#include <vector>
/*****************************************************************************/
namespace EthercatCommunication
{

/// CKim - SDO_data Structure holding all data needed to send/receive an SDO object.
typedef struct {
    uint16_t slave_position;    // Position based addressing.
    uint16_t index;		        // Index in Object dictionary
    uint8_t  sub_index;	        // Subindex in Object dictionary
    uint32_t data ;             // Actual data to sent/receive
    size_t   data_sz;	        // Size
    size_t   result_sz;         // Resulted data size
    uint32_t err_code;	        // Error code
} SDO_data;

class EthercatNtwk
{
public:
  EthercatNtwk();
  ~EthercatNtwk();

  /**
   * @brief Opens EtherCAT master via command line tool if it's not already on.
   * @return 0 if successfull, otherwise -1.
   */
  int OpenEthercatMaster();

  /**
   * @brief Requests master instance and creates a domain for a master.
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
  int CheckMasterState();
  
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
  void DeactivateMaster();
  
  /**
   * @brief Get the Number Of physically Connected Slaves to the bus.
   * @return 0 if the Number is equal to m_NumberOfSlaves, -1 otherwise.
   */
  int CheckNumberOfConnectedSlaves();

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

protected:
  /// File descriptor to open and wake  master from CLI.
  int fd;

  ec_master_t* m_master = NULL;                  // EtherCAT master instance
  ec_master_state_t m_master_state = {};         // EtherCAT master state
  ec_domain_t* m_master_domain = NULL;           // Ethercat data passing master domain
  ec_domain_state_t m_master_domain_state = {};  // EtherCAT master domain state
  struct timespec m_sync_timer;

  std::vector<EthercatSlave> m_slaves;
  int m_NumberOfSlaves = 0;

// -------------------------------------------------------
// Virtual functions to be overridden
public:
  /**
   * @brief Maps PDOs for the application.
   * @note One must override this method for their own applications
   * @return 0 if successfull, otherwise -1.
   */
  virtual int MapPdos() = 0;

  /**
   * @brief Configures DC sync for our default configuration
   */
  virtual void ConfigDcSyncDefault() = 0;

//  /**
//   * @brief Configures DC sync for our default configuration
//   */
//  void ReceiveConfigDcSyncDefault();

  /**
   * @brief Updates internal data from the data received from PDO
   * @note One must override this method for their own applications
   */
  virtual void ReadFromSlaves() = 0;

  /**
   * @brief Updates internal data to be sent via PDO
   * @note One must override this method for their own applications
   */
  virtual void WriteToSlaves() = 0;
};
}  // namespace EthercatCommunication
