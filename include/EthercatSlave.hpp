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
 *  \file   ecat_slave.hpp
 *  \brief  Ethercat slave class implementation header file.
 *
 *   This header file contains blueprint of EtherCAT slave's which will be
 *   used to  communicate with EtherCAT master.
 *******************************************************************************/
#pragma once

#include "ecat_globals.hpp"
#include <ecrt.h>

/******************************************************************************
 *  \class   EthercatSlave
 *  \brief   Contains EtherCAT slave parameters for configuration.
 *******************************************************************************/
namespace EthercatCommunication
{

/// offset for PDO entries to register PDOs.
typedef struct
{
    uint32_t target_pos ;
    uint32_t target_vel ;
    uint32_t target_tor ;
    uint32_t torque_offset;
    uint32_t max_tor  ;
    uint32_t control_word ;
    uint32_t op_mode ;
    uint32_t profile_acc ;
    uint32_t profile_dec ;
    uint32_t quick_stop_dec ;
    uint32_t profile_vel ;

    uint32_t actual_pos ;
    uint32_t pos_fol_err ;
    uint32_t actual_vel ;
    uint32_t actual_cur ;
    uint32_t actual_tor ;
    uint32_t status_word ;
    uint32_t op_mode_display ;
    uint32_t error_code ;
    uint32_t extra_status_reg ;
    uint32_t digital_input;
    uint32_t abs_encoder_pos;

    // DY
    uint32_t analog_input_1;
    uint32_t analog_input_2;
    uint32_t analog_output_1;
    uint32_t analog_output_2;

    uint32_t r_limit_switch;
    uint32_t l_limit_switch;
    uint32_t emergency_switch;
    uint32_t pressure_sensor;
} OffsetPDO ;


class EthercatSlave
{
public:
  EthercatSlave();
  ~EthercatSlave();
  /**
   * @brief This function will check slave's application layer states.
   *        (INIT/PREOP/SAFEOP/OP)
   * @note This function shouldn't be called in real time context.For diagnosis
   *       you can use CheckDomainState() encapsulation in ecat_node.
   * @return 0 if succesful.
   */
  int CheckSlaveConfigState();

  /**
   * @brief Slave information data structure.
   *      This structure contains all information related to slave.
   *      It will be used to get slave's information from master.
   */
  ec_slave_info_t slave_info_;

  /// Slave configuration parameters, assinged to each slave.
  ec_slave_config_t* slave_config_;

  /// Slave state handle to check if slave is online and slaves state machine status(INIT/PREOP/SAFEOP/0P)
  ec_slave_config_state_t slave_config_state_;

  /// For custom PDO configuration, check \see MapDefaultPdos() function @file ecat_master.cpp as an example
  ec_pdo_info_t* slave_pdo_info_;

  /// For custom PDO configuration, check \see MapDefaultPdos() function @file ecat_master.cpp as an example
  ec_pdo_entry_info_t* slave_pdo_entry_info_;

  /// For custom PDO configuration, check \see MapDefaultPdos() function @file ecat_master.cpp as an example
  ec_sync_info_t* slave_sync_info_;

  /// For custom PDO configuration, check \see MapDefaultPdos() function @file ecat_master.cpp as an example
  ec_pdo_entry_reg_t* slave_pdo_entry_reg_;

  /// PDO domain for data exchange
  uint8_t* slave_pdo_domain_;

  /// DC sync shift setting, zero will give best synchronization.
  const static uint32_t kSync0_shift_ = 0;


  /// Variable for checking motor state
  int32_t motor_state_;

  /// Offset for PDO entries to assign pdo registers.
  OffsetPDO offset_;

};  // EthercatSlave class
}
