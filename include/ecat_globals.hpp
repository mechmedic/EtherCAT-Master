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
/*****************************************************************************
 * \file  ecat_globals.hpp
 * \brief Header file for all include statements and global variables for EtherCAT
 *        communication.
 * 
 * This header file contains required include statements for IgH EtherCAT library,
 * global variables (e.g. ethercat master,master_state, domain,domain_state), 
 * structs for PDO offset and recieved data from slaves,
 * Communication period and number of slaves can be specified in here.
 *******************************************************************************/
#pragma once
#include "ecat_definitions.hpp"

/****************************************************************************/
                /// USER SHOULD DEFINE THIS AREAS ///
/// Number of connected servo drives.                
const uint32_t  g_kNumberOfServoDrivers = 7 ; 
const uint32_t  g_kNumberOfArms = 1;
/// Select operation mode for motors, default: Profile Velocity.
//static int8_t   g_kOperationMode = kProfileVelocity ; 
// static int8_t   g_kOperationMode = kCSVelocity ;  // Velocity mode - slave
static int8_t   g_kOperationMode = kCSTorque ;  // Torque mode - master 
// static int8_t   g_kOperationMode = kProfilePosition ;
#define NUM_OF_SLAVES     7  /// Total number of connected slave to the bus.
/// Set this to 1 if you have custom EtherCAT slave other than servo drive.
/// @note  That if you have different custom slave than EasyCAT you have to modify PDO mapping by yourself.
#define CUSTOM_SLAVE      0  
#define FREQUENCY       250  /// Ethercat PDO exchange loop frequency in Hz
//#define FREQUENCY       1000  /// Ethercat PDO exchange loop frequency in Hz
#define MEASURE_TIMING    0    /// If you want to measure timings leave it as one, otherwise make it 0.
#define DISTRIBUTED_CLOCK 0   /// If you want to use distributed clock make it one, otherwise leave it zero.


/*****************************************************************************/
#define PERIOD_NS       (g_kNsPerSec/FREQUENCY)  /// EtherCAT communication period in nanoseconds.
#define PERIOD_US       (PERIOD_NS / 1000)
#define PERIOD_MS       (PERIOD_US / 1000)

const struct timespec       g_cycle_time = {0, PERIOD_NS} ;       // cycletime settings in ns. 
const struct timespec       g_half_cycle_time = {0, PERIOD_NS/2} ;       // cycletime settings in ns. 

// CKim - Left or Right hand 1 if right hand


// CKim - Gear ratio of the master devices
#define M1_HEAD 3.0/13.0                    // Motor 1 output/input    
#define M2_HEAD 3.0/13.0                    // Motor 2 output/input    
#define M3_HEAD 3.0/13.0                    // Motor 3 output/input    
#define M4_HEAD 247.0/5175.0                // Motor 4 output/input    
#define M5_HEAD 247.0/5175.0                // Motor 5 output/input    
#define M6_HEAD 247.0/5175.0                // Motor 6 output/input    
#define M7_HEAD 247.0/5175.0                // Motor 7 output/input    

#define M1_CPR  2000                        // Counts per revolution for motor 1
#define M2_CPR  2000                        // Counts per revolution for motor 2
#define M3_CPR  2000                        // Counts per revolution for motor 3
#define M4_CPR  4096                        // Counts per revolution for motor 4
#define M5_CPR  4096                        // Counts per revolution for motor 5
#define M6_CPR  4096                        // Counts per revolution for motor 6
#define M7_CPR  4096                        // Counts per revolution for motor 7

#define M1_PULLEY   26/72                   // out/in
#define M2_PULLEY   14/60                   // out/in
#define M3_PULLEY   14/60                   // out/in
#define M4_BEVEL    1/2                     // out/in
#define M5_BEVEL    1/2                     // out/in
#define M6_BEVEL    1/2                     // out/in
#define M7_BEVEL    1/2                     // out/in

// Motor to Joint reduction ratio
#define m1InputToOutput double(M1_HEAD*M1_PULLEY*-1.0)
#define m2InputToOutput double(M2_HEAD*M2_PULLEY)
#define m3InputToOutput double(M3_HEAD*M3_PULLEY*-1.0)
#define m4InputToOutput double(M4_HEAD*M4_BEVEL)
#define m5InputToOutput double(M5_HEAD*M5_BEVEL*-1.0)
#define m6InputToOutput double(M6_HEAD*M6_BEVEL)
#define m7InputToOutput double(M7_HEAD*M7_BEVEL*-1.0)

// Motor position unit is counts
#define m1InputCntToOutputRad double(2.0*M_PI/M1_CPR*m1InputToOutput)
#define m2InputCntToOutputRad double(2.0*M_PI/M2_CPR*m2InputToOutput)
#define m3InputCntToOutputRad double(2.0*M_PI/M3_CPR*m3InputToOutput)
#define m4InputCntToOutputRad double(2.0*M_PI/M4_CPR*m4InputToOutput)
#define m5InputCntToOutputRad double(2.0*M_PI/M5_CPR*m5InputToOutput)
#define m6InputCntToOutputRad double(2.0*M_PI/M6_CPR*m6InputToOutput)
#define m7InputCntToOutputRad double(2.0*M_PI/M7_CPR*m7InputToOutput)

// Motor velocity unit is in rpm
#define RadToRpm double(60.0/(2*M_PI))
#define RpmToRad double(2*M_PI/60.0)
#define m1InputRpmToOutputRad double(RpmToRad*m1InputToOutput)
#define m2InputRpmToOutputRad double(RpmToRad*m2InputToOutput)
#define m3InputRpmToOutputRad double(RpmToRad*m3InputToOutput)
#define m4InputRpmToOutputRad double(RpmToRad*m4InputToOutput)
#define m5InputRpmToOutputRad double(RpmToRad*m5InputToOutput)
#define m6InputRpmToOutputRad double(RpmToRad*m6InputToOutput)
#define m7InputRpmToOutputRad double(RpmToRad*m7InputToOutput)

// Motor Rated Torque (0x6076, read from EPOS4, micro Newton-Meter unit)
#define m1RatedTorque 457380.0
#define m2RatedTorque 457380.0
#define m3RatedTorque 457380.0
#define m4RatedTorque 15990.0
#define m5RatedTorque 15990.0
#define m6RatedTorque 15990.0
#define m7RatedTorque 15990.0

// Converts calculated joint torque (in N.mm) to target torque command (1/1000 of the motor rated torque)
#define m1JointToTargetTorque double(m1InputToOutput/m1RatedTorque*1000.0*1000.0)
#define m2JointToTargetTorque double(m2InputToOutput/m2RatedTorque*1000.0*1000.0)
#define m3JointToTargetTorque double(m3InputToOutput/m3RatedTorque*1000.0*1000.0)
#define m4JointToTargetTorque double(m4InputToOutput/m4RatedTorque*1000.0*1000.0)
#define m5JointToTargetTorque double(m5InputToOutput/m5RatedTorque*1000.0*1000.0)
#define m6JointToTargetTorque double(m6InputToOutput/m6RatedTorque*1000.0*1000.0)
#define m7JointToTargetTorque double(m7InputToOutput/m7RatedTorque*1000.0*1000.0)

