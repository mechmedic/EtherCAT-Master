/*****************************************************************************
 * \file  epos_definitions.hpp
 * \brief Header file for all structures for EPOS slaves
 *
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

/// Object dictionary paramaters PDO index and default values in here.
#include "object_dictionary.hpp"

/// Motor operation modes
typedef enum
{
   kProfilePosition = 1,
   kProfileVelocity = 3,
   kProfileTorque   = 4,
   kHoming = 6,
   kInterpolatedPosition = 7,
   kCSPosition = 8,
   kCSVelocity = 9,
   kCSTorque = 10,
} OpMode ;

/// Homing Methods - from EPOS4 Firmware manual p213
typedef enum
{
 kActualPosition = 37,
 kIndexPositiveSpeed = 34,
 kIndexNegativeSpeed = 33,
 kHomeSwitchNegativeSpeed = 27,
 kHomeSwitchPositiveSpeed = 23,
 kPositiveLimitSwitch = 18,
 kNegativeLimitSwitch = 17,
 kHomeSwitchNegativeSpeedIndex = 11,
 kHomeSwitchPositiveSpeedIndex = 7,
 kPositiveLimitSwitchIndex = 2,
 kNegativeLimitSwitchIndex = 1,
 kCurrentPositiveIndex = -1,
 kCurrentNegativeIndex = -2,
 kCurrentPositiveSpeed = -3,
 kCurrentNegativeSpeed = -4
} HomingMethod;

// constant declarations
static constexpr uint8_t PRIMARY_STATE_UNKNOWN = 0u;
static constexpr uint8_t PRIMARY_STATE_UNCONFIGURED = 1u;
static constexpr uint8_t PRIMARY_STATE_INACTIVE = 2u;
static constexpr uint8_t PRIMARY_STATE_ACTIVE = 3u;
static constexpr uint8_t PRIMARY_STATE_FINALIZED = 4u;
static constexpr uint8_t TRANSITION_STATE_CONFIGURING = 10u;
static constexpr uint8_t TRANSITION_STATE_CLEANINGUP = 11u;
static constexpr uint8_t TRANSITION_STATE_SHUTTINGDOWN = 12u;
static constexpr uint8_t TRANSITION_STATE_ACTIVATING = 13u;
static constexpr uint8_t TRANSITION_STATE_DEACTIVATING = 14u;
static constexpr uint8_t TRANSITION_STATE_ERRORPROCESSING = 15u;

/// CIA 402 state machine motor states
enum MotorStates{
   kReadyToSwitchOn = 1,
   kSwitchedOn,
   kOperationEnabled,
   kFault,
   kVoltageEnabled,
   kQuickStop,
   kSwitchOnDisabled,
   kWarning,
   kRemote,
   kTargetReached,
   kInternalLimitActivate
};

enum ErrorRegisterBits{
   kGenericError = 0,
   kCurrentError,
   kVoltageError,
   kTemperatureError,
   kCommunicationError,
   kDeviceProfileSpecificError,
   kReserved,
   kMotionError
};

/// Sensor Configuration for motor for more information \see EPOS4-Firmware-Specification Pg.138
enum SensorConfig{
   kSensor1TypeNone=0,
   kSensor1TypeDigitalIncrementalEncoder1=1,
   kSensor2TypeNone=0,
   kSensor2TypeDigitalIncrementalEncoder2=256,
   kSensor2TypeAnalogIncrementalEncoderSinCos=512,
   kSensor2TypeSSIAbsoluteEncoder=768,
   kSensor3TypeNone=0,
   kSensor3TypeDigitalHallSensor=131072  //EC motors only

};

/// Control structure configuration for control mechanism to select sensor structure specific to hardware. \see EPOS4-Firmware-Specification pg. 140
enum ControlStructureBits{
   /// These are bit locations not values for values  \see EPOS4-Firmware-Specification pg. 140 !!!
   kCurrentControlStructure  = 0,    // 0-3 , 4 bits. Val : 1 - PI current controller
   kVelocityControlStructure = 4,   // 4-7,   4bits.  Val : 0 - None | 1 - PI Vecolity controller (low pass filter) | 2 - PI velocity controller (observer)
   kPositionControlStructure = 8,   // 8-11 , 4bits.  Val : 0 - None | 1 - PID position controller
   kGearLocation             = 12,  // 1 bit          Val : 0 - None | 1 - Gear Mounted on system
   kProcessValueReference    = 14,  // 14-15 2 bits.  Val : 0 - On motor (or undefined) | 1 - On gear
   kMainSensor               = 16,  // 16-19 4 bits.  Val : 0 - None | 1 - Sensor 1  | 2 - Sensor 2 | 3 - Sensor 3
   kAuxiliarySensor          = 20,  // 20-23 4 bits.  Val : 0 - None | 1 - Sensor 1  | 2 - Sensor 2 | 3 - Sensor 3
   kMountingPositionSensor1  = 24,  // 24-25 2 bits.  Val : 0 - On motor (or undefined) | 1 - On gear
   kMountingPositionSensor2  = 26,  // 26-27 2 bits.  Val : 0 - On motor (or undefined) | 1 - On gear
   kMountingPositionSensor3  = 28,  // 28-29 2 bits.  Val : 0 - On motor
};

/// Struct contains configuration parameters for Profile Position mode.
typedef struct
{
    uint32_t profile_vel ;
    uint32_t profile_acc ;
    uint32_t profile_dec ;
    uint32_t max_fol_err ;
    uint32_t max_profile_vel ;
    uint32_t quick_stop_dec ;
    uint16_t motion_profile_type ;
    uint32_t p_gain;
    uint32_t i_gain;
    uint32_t d_gain;
} ProfilePosParam ;

/// Struct contains configuration parameters for Profile Velocity mode.
typedef struct
{
    uint32_t	max_profile_vel;
    uint32_t	quick_stop_dec;
    uint32_t	profile_acc;
    uint32_t	profile_dec;
    uint16_t    motion_profile_type;
} ProfileVelocityParam ;

/// Struct contains configuration parameters for cyclic sync. position mode.
typedef struct
{
    uint32_t nominal_current ;
    uint16_t torque_constant ;
    uint32_t current_controller_gain ;
    uint32_t position_control_parameter_set ;
    uint32_t software_position_limit ;
    uint16_t motor_rated_torque ;
    uint32_t max_gear_input_speed ;
    uint32_t profile_vel ;
    uint32_t profile_acc ;
    uint32_t profile_dec ;
    uint32_t max_fol_err ;
    uint32_t max_profile_vel ;
    uint32_t quick_stop_dec ;
    uint32_t interpolation_time_period ;
} CSPositionModeParam ;

/// Struct containing 'velocity control parameter set' 0x30A2
/// Has 4 sub index. Default values are from EPOS4 firmware manual
typedef struct
 {
    uint32_t Pgain = 20000;     // micro amp sec per radian
    uint32_t Igain = 500000;    // micro amp per radian
    uint32_t FFVelgain = 0;
    uint32_t FFAccgain = 0;
 } VelControlParam;

/// Struct contains configuration parameters for cyclic sync. velocity mode.
typedef struct
{
    VelControlParam velocity_controller_gain ;
    uint32_t max_profile_vel ;
    uint32_t quick_stop_dec ;
    uint32_t profile_dec ;
    uint32_t software_position_limit ;
    uint32_t interpolation_time_period ;
} CSVelocityModeParam ;

/// Struct contains configuration parameters for cyclic sync. torque mode.
typedef struct
{
   uint32_t nominal_current ;
   uint16_t torque_constant ;
   uint32_t software_position_limit ;
   uint16_t motor_rated_torque ;
   uint32_t max_gear_input_speed ;
   uint32_t profile_vel ;
   uint32_t profile_acc ;
   uint32_t profile_dec ;
   uint32_t max_profile_vel ;
   uint32_t quick_stop_dec ;
//    uint32_t interpolation_time_period ;
   uint16_t max_torque;
} CSTorqueModeParam ;

/// Struct contains configuration parameters for homing mode.
typedef struct
{
    uint32_t	max_fol_err;
    uint32_t	max_profile_vel;
    uint32_t	quick_stop_dec;
    uint32_t	speed_for_switch_search;
    uint32_t	speed_for_zero_search;
    uint32_t	homing_acc;
    // Used when homing by touching mechanical limit and sensing current
    uint16_t	curr_threshold_homing;
    // Amount to move away from the sensed limit
    int32_t		home_offset;
    int8_t		homing_method;
} HomingParam;

/// EPOS Error number
enum ErrorType
{
    NO_ERROR = 0,
    GENERIC_ERROR = 0x1000,
    GENERIC_INIT_ERROR = 0x1080,
    GENERIC_INIT_ERROR_1 = 0x1081,
    GENERIC_INIT_ERROR_2 = 0x1082,
    GENERIC_INIT_ERROR_3 = 0x1083,
    GENERIC_INIT_ERROR_4 = 0x1084,
    GENERIC_INIT_ERROR_5 = 0x1085,
    GENERIC_INIT_ERROR_6 = 0x1086,
    GENERIC_INIT_ERROR_7 = 0x1087,
    GENERIC_INIT_ERROR_8 = 0x1088,
    FIRMWARE_INCOMPATIBLITY_ERROR = 0x1090,
    OVER_CURRENT_ERROR = 0x2310,
    POWER_STAGE_PROTECTION_ERROR = 0x2320,
    OVER_VOLTAGE_ERROR = 0x3210,
    UNDER_VOLTAGE_ERROR = 0x3220,
    THERMAL_OVERLOAD_ERROR = 0x4210,
    THERMAL_MOTOR_OVERLOAD_ERRROR = 0x4380,
    LOGIC_SUPPLY_TOO_LOW_ERROR = 0x5113,
    HARDWARE_DEFECT_ERROR = 0x5280,
    HARDWARE_INCOMPATIBLITY_ERROR = 0x5281,
    HARDWARE_ERROR = 0x5480,
    HARDWARE_ERROR_1 = 0x5481,
    HARDWARE_ERROR_2 = 0x5482,
    HARDWARE_ERROR_3 = 0x5483,
    SIGN_OF_LIFE_ERROR = 0x6080,
    EXTENSION_1_WATCHDOG_ERROR = 0x6081,
    INTERNAL_SOFTWARE_ERROR = 0x6180,
    SOFTWARE_PARAMETER_ERROR = 0x6320,
    PERSISTENT_PARAMETER_CORRUPT_ERROR = 0x6380,
    POSITION_SENSOR_ERROR = 0x7320,
    POSITION_SENSOR_BREACH_ERROR = 0x7380,
    POSITION_SENSOR_RESOLUTION_ERROR  = 0x7381,
    POSITION_SENSOR_INDEX_ERROR = 0x7382,
    HALL_SENSOR_ERROR = 0x7388,
    HALL_SENSOR_NOT_FOUND_ERROR =  0x7389,
    HALL_ANGLE_DETECTION_ERROR = 0x738A,
    SSI_SENSOR_ERROR = 0x738C,
    SSI_SENSOR_FRAME_ERROR = 0x738D,
    MISSING_MAIN_SENSOR_ERROR = 0x7390,
    MISSING_COMMUTATION_SENSOR_ERROR = 0x7391,
    MAIN_SENSOR_DIRECTION_ERROR = 0x7392,
    ETHERCAT_COMMUNCATION_ERROR = 0x8180,
    ETHERCAT_INITIALIZATION_ERROR = 0x8181,
    ETHERCAT_RX_QUEUE_OVERFLOW_ERROR = 0x8182,
    ETHERCAT_COMMUNICATION_ERROR_INTERNAL  = 0x8183,
    ETHERCAT_COMMUNICATION_CYCLE_TIME_ERROR = 0x8184,
    ETHERCAT_PDO_COMMUNICATION_ERROR = 0x8280,
    ETHERCAT_SDO_COMMUNICATION_ERROR = 0x8281,
    FOLLOWING_ERROR = 0x8611,
    NEGATIVE_LIMIT_SWITCH_ERROR = 0x8A80,
    POSITIVE_LIMIT_SWITCH_ERROR = 0x8A81,
    SOFTWARE_POSITION_LIMIT_ERROR = 0x8A82,
    STO_ERROR = 0x8A88,
    SYSTEM_OVERLOADED_ERROR = 0xFF01,
    WATCHDOG_ERROR = 0xFF02,
    SYSTEM_PEAK_OVERLOADED_ERROR = 0XFF0B,
    CONTROLLER_GAIN_ERROR = 0xFF10,
    AUTO_TUNING_INDENTIFICATION_ERROR = 0xFF11,
    AUTO_TUNING_CURRENT_LIMIT_ERROR = 0xFF12,
    AUTO_TUNING_IDENTIFICATION_CURRENT_ERROR = 0xFF13,
    AUTO_TUNING_DATA_SAMPLING_ERROR  = 0xFF14,
    AUTO_TUNING_SAMPLE_MISMATCH_ERROR = 0xFF15,
    AUTO_TUNING_PARAMETER_ERROR = 0xFF16,
    AUTO_TUNING_AMPLITUDE_MISMATCH_ERROR = 0xFF17,
    AUTO_TUNING_TIMEOUT_ERROR = 0xFF19,
    AUTO_TUNING_STAND_STILL_ERROR = 0xFF20,
    AUTO_TUNING_TORQUE_INVALID_ERROR = 0xFF21,
    AUTO_TUNING_MAX_SYSTEM_SPEED_ERROR = 0XFF22,
    AUTO_TUNING_MOTOR_CONNECTION_ERROR = 0xFF23,
    AUTO_TUNING_SENSOR_SIGNAL_ERROR = 0XFF24
};

/// Returns EPOS Error string
static std::string GetErrorMessage(const int& err_code)
{
    switch (err_code)
    {
        case NO_ERROR:
            return "No error";
        case GENERIC_ERROR:
            return "Generic error";
        case GENERIC_INIT_ERROR:
            return "Generic initialization error";
        case GENERIC_INIT_ERROR_1:
            return "Generic initialization error 1";
        case GENERIC_INIT_ERROR_2:
            return "Generic initialization error 2";
        case GENERIC_INIT_ERROR_3:
            return "Generic initialization error 3";
        case GENERIC_INIT_ERROR_4:
            return "Generic initialization error 4";
        case GENERIC_INIT_ERROR_5:
            return "Generic initialization error 5";
        case GENERIC_INIT_ERROR_6:
            return "Generic initialization error 6";
        case GENERIC_INIT_ERROR_7:
            return "Generic initialization error 7";
        case GENERIC_INIT_ERROR_8:
            return "Generic initialization error 8";
        case FIRMWARE_INCOMPATIBLITY_ERROR:
            return "Firmware incompatibility error";
        case OVER_CURRENT_ERROR:
            return "Over current error";
        case POWER_STAGE_PROTECTION_ERROR:
            return "Power stage protection error";
        case OVER_VOLTAGE_ERROR:
            return "Over voltage error";
        case UNDER_VOLTAGE_ERROR:
            return "Under voltage error";
        case THERMAL_OVERLOAD_ERROR:
            return "Thermal overload error";
        case THERMAL_MOTOR_OVERLOAD_ERRROR:
            return "Thermal motor overload error";
        case LOGIC_SUPPLY_TOO_LOW_ERROR:
            return "Logic supply too low error";
        case HARDWARE_DEFECT_ERROR:
            return "Hardware defect error";
        case HARDWARE_INCOMPATIBLITY_ERROR:
            return "Hardware incompatibility error";
        case HARDWARE_ERROR:
            return "Hardware error";
        case HARDWARE_ERROR_1:
            return "Hardware error 1";
        case HARDWARE_ERROR_2:
            return "Hardware error 2";
        case HARDWARE_ERROR_3:
            return "Hardware error 3";
        case SIGN_OF_LIFE_ERROR:
            return "Sign of life error";
        case EXTENSION_1_WATCHDOG_ERROR:
            return "Extension 1 watchdog error";
        case INTERNAL_SOFTWARE_ERROR:
            return "Internal software error";
        case SOFTWARE_PARAMETER_ERROR:
            return "Software parameter error";
        case PERSISTENT_PARAMETER_CORRUPT_ERROR:
            return "Persistent parameter corrupt error";
        case POSITION_SENSOR_ERROR:
            return "Position sensor error";
        case POSITION_SENSOR_BREACH_ERROR:
            return "Position sensor breach error";
        case POSITION_SENSOR_RESOLUTION_ERROR:
            return "Position sensor resolution error";
        case POSITION_SENSOR_INDEX_ERROR:
            return "Position sensor index error";
        case HALL_SENSOR_ERROR:
            return "Hall sensor error";
        case HALL_SENSOR_NOT_FOUND_ERROR:
            return "Hall sensor not found error";
        case HALL_ANGLE_DETECTION_ERROR:
            return "Hall angle detection error";
        case SSI_SENSOR_ERROR:
            return "SSI sensor error";
        case SSI_SENSOR_FRAME_ERROR:
            return "SSI sensor frame error";
        case MISSING_MAIN_SENSOR_ERROR:
            return "Missing main sensor error";
        case MISSING_COMMUTATION_SENSOR_ERROR:
            return "Missing commutation sensor error";
        case MAIN_SENSOR_DIRECTION_ERROR:
            return "Main sensor direction error";
        case ETHERCAT_COMMUNCATION_ERROR:
            return "Ethercat communication error";
        case ETHERCAT_INITIALIZATION_ERROR:
            return "Ethercat initialization error";
        case ETHERCAT_RX_QUEUE_OVERFLOW_ERROR:
            return "Ethercat RX queue overflow error";
        case ETHERCAT_COMMUNICATION_ERROR_INTERNAL:
            return "Ethercat communication error internal";
        case ETHERCAT_COMMUNICATION_CYCLE_TIME_ERROR:
            return "Ethercat communication cycle time error";
        case ETHERCAT_PDO_COMMUNICATION_ERROR:
            return "Ethercat PDO communication error";
        case ETHERCAT_SDO_COMMUNICATION_ERROR:
            return "Ethercat SDO communication error";
        case FOLLOWING_ERROR:
            return "Following error";
        case NEGATIVE_LIMIT_SWITCH_ERROR :
            return "Negative limit switch error";
        case POSITIVE_LIMIT_SWITCH_ERROR :
            return "Positive limit switch error";
        case SOFTWARE_POSITION_LIMIT_ERROR:
            return "Software position limit error";
        case STO_ERROR :
            return "STO error";
        case SYSTEM_OVERLOADED_ERROR:
            return "System overloaded error";
        case WATCHDOG_ERROR:
            return "Watchdog error";
        case SYSTEM_PEAK_OVERLOADED_ERROR:
            return "System peak overloaded error";
        case CONTROLLER_GAIN_ERROR:
            return "Controller gain error";
        case AUTO_TUNING_INDENTIFICATION_ERROR:
            return "Auto tuning identification error";
        case AUTO_TUNING_CURRENT_LIMIT_ERROR:
            return "Auto tuning current limit error";
        case AUTO_TUNING_IDENTIFICATION_CURRENT_ERROR:
            return "Auto tuning identification current error";
        case AUTO_TUNING_DATA_SAMPLING_ERROR:
            return "Auto tuning data sampling error";
        case AUTO_TUNING_SAMPLE_MISMATCH_ERROR:
            return "Auto tuning sample mismatch error";
        case AUTO_TUNING_PARAMETER_ERROR:
            return "Auto tuning parameter error";
        case AUTO_TUNING_AMPLITUDE_MISMATCH_ERROR:
            return "Auto tuning amplitude mismatch error";
        case AUTO_TUNING_TIMEOUT_ERROR:
            return "Auto tuning timeout error";
        case AUTO_TUNING_STAND_STILL_ERROR:
            return "Auto tuning stand still error";
        case AUTO_TUNING_TORQUE_INVALID_ERROR:
            return "Auto tuning torque invalid error";
        case AUTO_TUNING_MAX_SYSTEM_SPEED_ERROR:
            return "Auto tuning max system speed error";
        case AUTO_TUNING_MOTOR_CONNECTION_ERROR:
            return "Auto tuning motor connection error";
        case AUTO_TUNING_SENSOR_SIGNAL_ERROR:
            return "Auto tuning sensor signal error";
        default:
            return "Unknown error";
    }
}

