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

#include <iostream>
#include <cstring>
#include <limits.h>
#include <stdlib.h>
#include <pthread.h>
#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <time.h>
#include <sys/mman.h>
#include <malloc.h>
#include <sched.h>
#include <chrono>
#include <memory>

#define TEST_BIT(NUM,N)    ((NUM &  (1 << N))>>N)  /// Check specific bit in the data. 0 or 1.
#define SET_BIT(NUM,N)      (NUM |  (1 << N))  /// Set(1) specific bit in the data.
#define RESET_BIT(NUM,N)    (NUM & ~(1 << N))  /// Reset(0) specific bit in the data

/// Convert timespec struct to nanoseconds */
#define TIMESPEC2NS(T)      ((uint64_t) (T).tv_sec * g_kNsPerSec + (T).tv_nsec)
#define DIFF_NS(A, B) (((B).tv_sec - (A).tv_sec) * g_kNsPerSec + (B).tv_nsec - (A).tv_nsec)

/// Using Monotonic system-wide clock.  */
#define CLOCK_TO_USE        CLOCK_MONOTONIC

/****************************************************************************/
// Global variable declarations
const uint32_t           g_kNsPerSec = 1000000000;     /// Nanoseconds per second.

/// Number of connected servo drives.
const uint32_t  g_kNumberOfServoDrivers = 7 ;
#define NUM_OF_SLAVES     7  /// Total number of connected slave to the bus.
#define FREQUENCY       1000  /// Ethercat PDO exchange loop frequency in Hz
#define DISTRIBUTED_CLOCK 0   /// If you want to use distributed clock make it one, otherwise leave it zero.
//#define MEASURE_TIMING    0    /// If you want to measure timings leave it as one, otherwise make it 0.

/*****************************************************************************/
#define PERIOD_NS       (g_kNsPerSec/FREQUENCY)  /// EtherCAT communication period in nanoseconds.
#define PERIOD_US       (PERIOD_NS / 1000)
#define PERIOD_MS       (PERIOD_US / 1000)

const struct timespec       g_cycle_time = {0, PERIOD_NS} ;       // cycletime settings in ns.
//const struct timespec       g_half_cycle_time = {0, PERIOD_NS/2} ;       // cycletime settings in ns.


//static volatile sig_atomic_t sig = 1;
//extern struct timespec      g_sync_timer ;                       // timer for DC sync .
//extern uint32_t             g_sync_ref_counter;                  // To sync every cycle.

/// Select operation mode for motors, default: Profile Velocity.
//static int8_t   g_kOperationMode = kProfileVelocity ;
// static int8_t   g_kOperationMode = kCSVelocity ;  // Velocity mode - slave
//static int8_t   g_kOperationMode = kCSTorque ;  // Torque mode - master
// static int8_t   g_kOperationMode = kProfilePosition ;



/**
 * @brief Add two timespec struct.
 *
 * @param time1 Timespec struct 1
 * @param time2 Timespec struct 2
 * @return Addition result
 */

inline struct timespec timespec_add(struct timespec time1, struct timespec time2)
{
    struct timespec result;

    if ((time1.tv_nsec + time2.tv_nsec) >= g_kNsPerSec)
    {
        result.tv_sec = time1.tv_sec + time2.tv_sec + 1;
        result.tv_nsec = time1.tv_nsec + time2.tv_nsec - g_kNsPerSec;
    }
    else
    {
        result.tv_sec = time1.tv_sec + time2.tv_sec;
        result.tv_nsec = time1.tv_nsec + time2.tv_nsec;
    }

    return result;
}



