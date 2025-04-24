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
 *  \file   EposNtwk.hpp
 *  \brief  Class implementing EPOS4 connected by EtherCAT network
 *******************************************************************************/
#pragma once
/******************************************************************************/
#include <EthercatNtwk.hpp>
#include <epos_definitions.h>
/******************************************************************************/
namespace EthercatCommunication
{


// CKim - Struct for received feedback data from slaves
typedef struct
{
    int32_t   target_pos ;
    int32_t   target_vel ;
    int16_t   target_tor ;
    int16_t   max_tor ;
    uint16_t  control_word ;
    OpMode    op_mode ;
    int32_t   vel_offset ;
    int16_t   tor_offset ;

    int32_t  actual_pos ;
    int32_t  actual_vel ;
    int16_t  actual_cur ;
    int16_t  actual_tor ;
    uint16_t status_word ;
    uint16_t error_code;

    int8_t   op_mode_display ;
    uint8_t  left_limit_switch_val ;
    uint8_t  right_limit_switch_val ;
    uint8_t  s_emergency_switch_val;
    uint32_t digital_input;

    // DY
    uint8_t  p_emergency_switch_val;
    int32_t  right_x_axis;
    int32_t  left_x_axis;
    uint8_t  com_status;
    int16_t  analog_input_1;
    int16_t  analog_input_2;

}EposPdoData;


class EposNtwk : public EthercatNtwk
{
public:
  EposNtwk();
  ~EposNtwk();

public:
  EposPdoData m_PdoData[g_kNumberOfServoDrivers];

  // ---------------------------------------------------------------
  // CKim - Virtual functions.

public:
  /**
   * @brief Maps PDOs for the application.
   * @note  These functions are specific for MISS-Robot V3 which has 5 EPOS4 as a slaves,
   * with absolute encoder enabled for motor 2 and 3
   * @return 0 if successfull, otherwise -1.
   */
  virtual int MapPdos();

  /**
   * @brief Configures DC sync for our default configuration
   */
  virtual void ConfigDcSyncDefault();

//  /**
//   * @brief Configures DC sync for our default configuration
//   */
//  void ReceiveConfigDcSyncDefault();

  /**
   * @brief Updates internal data from the data received from PDO
   * @note  Will update internal robot joint states
   */
  virtual void ReadFromSlaves();

  /**
   * @brief Updates internal data to be sent via PDO
   * @note  Will send commands to each joints
   */
  virtual void WriteToSlaves();


  // ---------------------------------------------------------------
  // CKim - Additional functions specific for EPOS4

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


};
}  // namespace EthercatCommunication
