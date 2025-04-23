/*****************************************************************************
 * This is an example code for using 'ecat_master' and 'ecat_slave' class to
 * connect to EPOS4
 *******************************************************************************/

#define MAX_SAFE_STACK (8 * 1024) /* The maximum stack size which is
                                     guranteed safe to access without
                                     faulting */
#define NSEC_PER_SEC (1000000000)

#include <ecat_master.hpp>
#include <ecat_slave.hpp>

#include <iostream>
#include <memory>

using namespace std;
using namespace EthercatCommunication;

// 1. Request master, 2. Create process data domain,
// 3. Configure Slaves, 4. Setup PDO
// 5. Activate master, 6. Register domain data
// 7. Run cyclic exchange (receive and process, update data, queue and send)

void cyclic_task()
{

}

int main()
{
    // ------------------------------------------------------------- //
    // CKim - Initialize EtherCAT master
    // ------------------------------------------------------------- //

    // CKim - Create unique pointer to EthercatMaster
    auto ecat_master = std::make_unique<EthercatMaster>();

    // CKim - Open EtherCAT master via command line
    cout << "Opening EtherCAT device...\n";
    if (ecat_master->OpenEthercatMaster())      {   return -1;      }

    // CKim - Request master and create process data domain
    // Pass index in case of multiple master
    cout << "Configuring EtherCAT master...\n";
    if (ecat_master->ConfigureMaster(0))
    {
      return -1;
    }

    // ------------------------------------------------------------- //
    // CKim - Read the connected slave info and configure them
    // ------------------------------------------------------------- //

    // CKim - Check if the number of connected slaves is equal to the
    // NUM_OF_SLAVES defined in ecat_globals.hpp. In case of error, close master
    cout << "Getting connected slave informations...\n";
    if (ecat_master->GetNumberOfConnectedSlaves())
    {
      ecat_master->ReleaseMaster();
      for (int i = 0; i < NUM_OF_SLAVES; i++)   {
        ecat_master->ResetSlave(i);
      }
      return -1;
    }

    // CKim - Read and print out the connected slave info
    ec_slave_info_t info;
    for (int i = 0; i < NUM_OF_SLAVES; i++)
    {
      ecat_master->GetSlaveInformation(i, info);
      std::printf("\n");
      std::printf("--------------------Slave Info -------------------------\n");
      std::printf("Slave alias         = %d\n ", info.alias);
      std::printf("Slave position      = %d\n ", info.position);
      std::printf("Slave vendor_id     = 0x%08x\n ", info.vendor_id);
      std::printf("Slave product_code  = 0x%08x\n ", info.product_code);
      std::printf("Slave name          = %s\n ", info.name);
      std::printf("--------------------EOF %d'th Slave Info ----------------\n ", i);
    }

    // CKim - Initialize slaves
    cout << "Configuring  slaves...\n";
    for (int i = 0; i < NUM_OF_SLAVES; i++)
    {
      if (ecat_master->InitSlave(i))
      {
        return -1;
      }
    }

    // ------------------------------------------------------------- //
    // CKim - Setup parameters of the EPOS via SDO,
    // configure PDO mappings, and Distributed Clock Sync
    // This part will be application specific
    // ------------------------------------------------------------- //
//    cout << "Configuration EPOS parameters...\n";
//    for (int i = 0; i < g_kNumberOfServoDrivers; i++)
//    {
//             motor_opmode_[i] = kCSTorque;
//          //motor_opmode_[i] = kCSVelocity;
//          if(SetConfigurationParameters(i, motor_opmode_[i]))
//          {
//            return -1;
//          }
//     }

    cout << "Mapping default PDOs...\n";
    if (ecat_master->MapDefaultPdos())  {   return -1;  }

    cout << "Configuring DC synchronization...\n";
    if (DISTRIBUTED_CLOCK)
        ecat_master->ConfigDcSyncDefault();



    // ------------------------------------------------------------- //
    // CKim - Activate master, obtain pointer to process data
    // domain's memory
    // ------------------------------------------------------------- //
    cout << "Activating master...\n";
    if (ecat_master->ActivateMaster())  {   return -1;  }

    cout << "Registering master domain...\n";
    if (ecat_master->RegisterDomain())  {   return -1;  }


    // ------------------------------------------------------------- //
    // CKim - Prepare and start cyclic data exchange
    // This part will be application specific
    // ------------------------------------------------------------- //
    cout << "Starting cyclic data exchange...\n";

    // CKim - Configure realtime thread priority
    /* Set priority */
    struct sched_param param = {};
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);

    printf("Using priority %i.", param.sched_priority);
    if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
        perror("sched_setscheduler failed");
    }

    /* Lock memory */
    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
        fprintf(stderr, "Warning: Failed to lock memory: %s\n",
                strerror(errno));
    }

    /* Stack prefault */
    unsigned char dummy[MAX_SAFE_STACK];
    memset(dummy, 0, MAX_SAFE_STACK);


    // CKim - Start cyclic data exchange
    struct timespec wakeup_time;
    int ret = 0;

    printf("Starting RT task with dt=%u ns.\n", PERIOD_NS);

    clock_gettime(CLOCK_MONOTONIC, &wakeup_time);
    wakeup_time.tv_sec += 1; /* start in future */
    wakeup_time.tv_nsec = 0;

    while (1) {
        ret = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME,
                              &wakeup_time, NULL);
        if (ret) {
            fprintf(stderr, "clock_nanosleep(): %s\n", strerror(ret));
            break;
        }

        cyclic_task();

        wakeup_time.tv_nsec += PERIOD_NS;
        while (wakeup_time.tv_nsec >= NSEC_PER_SEC) {
            wakeup_time.tv_nsec -= NSEC_PER_SEC;
            wakeup_time.tv_sec++;
        }
    }


//    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Waiting for operational mode...\n");
//     if (ecat_node_->WaitForOperationalMode())
//     {
//       RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Error : Time out occurred while waiting for OP mode.!  ");
//       on_cleanup(this->get_current_state());
//       if(received_data_.current_lifecycle_state==PRIMARY_STATE_UNCONFIGURED)
//       on_configure(this->get_current_state());
//       return -1;
//     }
//     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Initialization succesful...\n");

//     if (SetComThreadPriorities())
//     {
//       return -1;
//     }

//   void EthercatLifeCycle::StartPdoExchange(void* instance)



    // ------------------------------------------------------------- //
    // CKim - Stop cyclic data exchange, cleanup and close
    // ------------------------------------------------------------- //

    // CKim - Release master and shutdown master via command line
    cout << "Ending EtherCAT master...\n";
    ecat_master->DeactivateCommunication();
    ecat_master->ReleaseMaster();
    if (ecat_master->ShutDownEthercatMaster())
    {
      return -1;
    }

    cout << "Hello World!" << endl;
    return 0;
}


//int EthercatLifeCycle::SetConfigurationParameters(int index, int mode)
//{
//  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Setting configuration parameter for motor %d ...", index + 1);
//  int error = 0;
//  if (mode == kProfilePosition)
//  {
//    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Setting drives to Position mode...\n");
//    ProfilePosParam P;
//    uint32_t max_fol_err;
//    P.profile_vel = 450;
//    P.profile_acc = 1e4;
//    P.profile_dec = 1e4;
//    P.max_profile_vel = 1e3;
//    P.quick_stop_dec = 3e4;
//    P.motion_profile_type = 0;
//    error = ecat_node_->SetProfilePositionParameters(index, P);
//  }
//  else if (mode == kCSPosition)
//  {
//    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Setting drives to CSP mode...\n");
//    CSPositionModeParam P;
//    uint32_t max_fol_err;
//    P.profile_vel = 50;
//    P.profile_acc = 3e4;
//    P.profile_dec = 3e4;
//    P.max_profile_vel = 1e3;
//    P.quick_stop_dec = 3e4;
//    P.interpolation_time_period = 0;
//    P.max_fol_err = 200000;
//    error = ecat_node_->SetCyclicSyncPositionModeParameters(index, P);
//  }
//  else if (mode == kCSVelocity)
//  {
//    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Setting drives to CSV mode...\n");
//    CSVelocityModeParam P;
//    P.profile_dec = 3e4;
//    P.max_profile_vel = 1e3;
//    P.quick_stop_dec = 3e4;
//    P.interpolation_time_period = 2;
//    error = ecat_node_->SetCyclicSyncVelocityModeParameters(index, P);
//  }
//  else if (mode == kCSTorque)
//  {
//    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Setting drives to CST mode...\n");
//    CSTorqueModeParam P;
//    //P.max_gear_input_speed = ;
//    // P.max_torque = 50;
//    // P.max_profile_vel = 40000;
//    P.profile_dec = 3e4;
//    P.quick_stop_dec = 3e4;
//    error = ecat_node_->SetCyclicSyncTorqueModeParameters(index, P);
//  }
//  else
//  {
//    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Setting drives to Velocity mode...\n");
//    ProfileVelocityParam P;
//    P.profile_acc = 3e4;
//    P.profile_dec = 3e4;
//    P.max_profile_vel = 1e3;
//    P.quick_stop_dec = 3e4;
//    P.motion_profile_type = 0;
//    error = ecat_node_->SetProfileVelocityParameters(index, P);
//  }
//  return error;
//}


void EthercatLifeCycle::StartPdoExchange(void* instance)
{
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting PDO exchange....\n");

  // Measurement time in minutes, e.g.
  uint32_t print_max_min = measurement_time * 60000;
  uint32_t print_val = 1e4;
  int error_check = 0;
  int32_t status_var = 0;
  struct timespec wake_up_time, time;
  struct timespec publish_time_start = {}, publish_time_end = {};

  // get current time
  int status_check_counter = 1000;
  int enabled_counter = 0;
  int try_enable_counter = g_kNumberOfServoDrivers * 5;

  ec_master_state_t ms;

  for(int i=0; i<g_kNumberOfServoDrivers; i++)
  {
    sent_data_.target_pos[i] = received_data_.actual_pos[i];
    sent_data_.target_vel[i] = 0;
    sent_data_.target_tor[i] = 0;
    sent_data_.tor_offset[i] = 0;
  }

  // ------------------------------------------------------- //
  // CKim - Initialization loop before entring control loop.
  // Switch On and Enable Driver
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Enabling motors...");
  clock_gettime(CLOCK_TO_USE, &wake_up_time);
  while (sig && !gui_buttons_status_.b_stop_cyclic_pdo && (enabled_counter < g_kNumberOfServoDrivers))
  {
    // CKim - Sleep for 1 ms
    wake_up_time = timespec_add(wake_up_time, g_cycle_time);
    clock_nanosleep(CLOCK_TO_USE, TIMER_ABSTIME, &wake_up_time, NULL);
    ecat_node_->SetMasterApplicationTime(wake_up_time);

    // CKim - Receive process data
    ecat_node_->ReceiveAndProcess();
    ReadFromSlaves();

    // CKim - Initialize target pos and vel
    for (int i = enabled_counter; i < g_kNumberOfServoDrivers; i++)
    {
      // CKim - Check status and update control words to enable drivers
      // Returns number of enabled drivers
      EnableDrivers(i);
      if (motor_state_[i] == kOperationEnabled)
      {
        enabled_counter += 1;
      }
      else
      {
        enabled_counter = 0;
      }
    }

    // CKim - Periodic printout
    if (status_check_counter)
    {
      status_check_counter--;
    }
    else
    {
      // Checking master/domain/slaves state every 1sec.
      if (ecat_node_->CheckMasterState(ms) < 0)
      {
        RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Connection error, check your physical connection.");
        al_state_ = ms.al_states;
        received_data_.emergency_switch_val = 0;
        emergency_status_ = 0;
        // PublishAllData();
        error_check++;
        if (error_check == 5)
          return;
      }
      else
      {
        ecat_node_->CheckMasterDomainState();
        error_check = 0;
        al_state_ = ms.al_states;
        status_check_counter = 1000;

        for (int i = 0; i < g_kNumberOfServoDrivers; i++)
        {
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "State of Drive %d : %d\n", i, motor_state_[i]);
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Trying to enable motors");
        }
        // PublishAllData();
      }
    }

    // CKim - Queue data
    RobotControlLaw();
    WriteToSlaves();

    // CKim - Sync Timer
    if (g_sync_ref_counter)
    {
      g_sync_ref_counter--;
    }
    else
    {
      clock_gettime(CLOCK_TO_USE, &time);
      ecat_node_->SyncMasterReferenceClock(time);
      g_sync_ref_counter = 1;
    }
    ecat_node_->SyncSlaveClock();
    ecat_node_->QueueAndSend();
    // PublishAllData();

  }  // while(sig)
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "All motors enabled, entering control loop");

  // ------------------------------------------------------- //
  // CKim - All motors enabled. Start control loop
  clock_gettime(CLOCK_TO_USE, &m_begin_time);
  while (sig && !gui_buttons_status_.b_stop_cyclic_pdo)
  {
    wake_up_time = timespec_add(wake_up_time, g_cycle_time);
    clock_nanosleep(CLOCK_TO_USE, TIMER_ABSTIME, &wake_up_time, NULL);
    ecat_node_->SetMasterApplicationTime(wake_up_time);

    // receive process data
    ecat_node_->ReceiveAndProcess();

    if (status_check_counter) {   status_check_counter--; }
    else
    {
      // Checking master/domain/slaves state every 1sec.
      if (ecat_node_->CheckMasterState(ms) < 0)
      {
        RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Connection error, check your physical connection.");
        al_state_ = ms.al_states;
        received_data_.emergency_switch_val = 0;
        emergency_status_ = 0;
        PublishAllData();
        error_check++;
        if (error_check == 5)
          return;
      }
      else
      {
        // ecat_node_->CheckMasterDomainState();
        error_check = 0;
        al_state_ = ms.al_states;
        status_check_counter = 1000;
      }
    }
    ReadFromSlaves();
    RobotControlLaw();
    WriteToSlaves();

    if (g_sync_ref_counter) {  g_sync_ref_counter--;  }
    else
    {
      clock_gettime(CLOCK_TO_USE, &time);
      ecat_node_->SyncMasterReferenceClock(time);
      g_sync_ref_counter = 1;
    }

    ecat_node_->SyncSlaveClock();
    ecat_node_->QueueAndSend();

    // m_Counter++;
    // if(m_Counter % 4 == 0)
    // {
      PublishAllData();
    // }
  }  // while(1/sig) //Ctrl+C signal

  // ------------------------------------------------------- //
  // CKim - Disable drivers before exiting
  wake_up_time = timespec_add(wake_up_time, g_cycle_time);
  clock_nanosleep(CLOCK_TO_USE, TIMER_ABSTIME, &wake_up_time, NULL);
  ecat_node_->SetMasterApplicationTime(wake_up_time);
  ecat_node_->ReceiveAndProcess();
  ReadFromSlaves();

  for (int i = 0; i < g_kNumberOfServoDrivers; i++)
  {
    sent_data_.control_word[i] = SM_GO_SWITCH_ON_DISABLE;
  }
  RobotControlLaw();
  WriteToSlaves();
  ecat_node_->QueueAndSend();
  // ------------------------------------------------------- //

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Leaving control thread.");
  pthread_exit(NULL);
}  // StartPdoExchange end

// --------------------------------------------------------------------------------------------

void EthercatLifeCycle::ReadFromSlaves()
{
  for (int i = 0; i < g_kNumberOfServoDrivers; i++)
  {
    received_data_.actual_pos[i] =
        EC_READ_S32(ecat_node_->slaves_[i].slave_pdo_domain_ + ecat_node_->slaves_[i].offset_.actual_pos);
    received_data_.actual_vel[i] =
        EC_READ_S32(ecat_node_->slaves_[i].slave_pdo_domain_ + ecat_node_->slaves_[i].offset_.actual_vel);
    received_data_.status_word[i] =
        EC_READ_U16(ecat_node_->slaves_[i].slave_pdo_domain_ + ecat_node_->slaves_[i].offset_.status_word);
    received_data_.actual_tor[i] =
        EC_READ_S16(ecat_node_->slaves_[i].slave_pdo_domain_ + ecat_node_->slaves_[i].offset_.actual_tor);
    received_data_.error_code[i] =
        EC_READ_U16(ecat_node_->slaves_[i].slave_pdo_domain_ + ecat_node_->slaves_[i].offset_.error_code);
    received_data_.digital_input[i] =
        EC_READ_U32(ecat_node_->slaves_[i].slave_pdo_domain_ + ecat_node_->slaves_[i].offset_.digital_input);
    received_data_.op_mode_display[i] =
        EC_READ_U8(ecat_node_->slaves_[i].slave_pdo_domain_ + ecat_node_->slaves_[i].offset_.op_mode_display);

   // DY
    received_data_.analog_input_1[i] =
        EC_READ_S16(ecat_node_->slaves_[i].slave_pdo_domain_ + ecat_node_->slaves_[i].offset_.analog_input_1);
    received_data_.analog_input_2[i] =
        EC_READ_S16(ecat_node_->slaves_[i].slave_pdo_domain_ + ecat_node_->slaves_[i].offset_.analog_input_2);
  }
  received_data_.com_status = al_state_;
  emergency_status_ = 1;
  gui_node_data_ = 1;
  received_data_.emergency_switch_val = 1;
}  // ReadFromSlaves end

// // CKim - For experiment
// void EthercatLifeCycle::RobotControlLaw()
// {
//   // -----------------------------------------------------------------------------------------------
//   /// WRITE YOUR CUSTOM CONTROL ALGORITHM, VARIABLES DECLARATAION HERE, LIKE IN EXAMPLE BELOW.

//   // 1. Generate sinusoidal motion for each joint for dynamic parameter identification
//   struct timespec curr_time;
//   clock_gettime(CLOCK_TO_USE, &curr_time);
//   uint64_t dt = DIFF_NS(curr_time, m_begin_time);
//   dt /= 1000;   // to us.

//   for(int i=0; i<7; i++)  {
//     JointVelCmd[i] = 0;
//   }

//   //JointVelCmd[3] = M_PI/4.0*2*M_PI/3.0*cos(2*M_PI/3000000.0*dt);
//   //JointVelCmd[4] = M_PI/4.0*2*M_PI/3.0*cos(2*M_PI/3000000.0*dt);
//   //JointVelCmd[5] = M_PI/6.0*2*M_PI/3.0*cos(2*M_PI/3000000.0*dt);
//   JointVelCmd[6] = M_PI/2.0*2*M_PI/3.0*cos(2*M_PI/3000000.0*dt);

//   // 2. Convert to motor rpm
//   for(int i=0; i<7; i++ )  {
//     sent_data_.target_vel[i] = JointVelCmd[i]/MotorInputRpmToOutputRad[i];
//   }

//   // 3. Update robot joint pos, vel and kinematics
//   // Kinematically, master device is modeled as a 7 dof serial manipulator by treating
//   // 5 bar linkage at the base is as a 2R manipulator
//   Eigen::VectorXd jPos(7);      Eigen::VectorXd jVel(7);    Eigen::VectorXd jTor(7);
//   for(int i=0; i<7; i++ )  {
//     jPos[i] = received_data_.actual_pos[i]*MotorInputCntToOutputRad[i];         // rad
//     jVel[i] = received_data_.actual_vel[i]*MotorInputRpmToOutputRad[i];         // rad/s
//     jTor[i] = received_data_.actual_tor[i]/JointToTargetTorque[i];              // N.mm
//   }
//   jPos[2] = jPos[2] - jPos[1];    // CKim - Due to 5 bar linkage structure.
//   jVel[2] = jVel[2] - jVel[1];    // CKim - Due to 5 bar linkage structure.

//   SE3 T = m_pRobotMdl->ForwardKinematics(jPos);
//   Jacobian Jb = m_pRobotMdl->JacobianBody(jPos);
//   se3 Vb(0);
//   for(int i=0; i<7; i++ )  {  Vb += (Jb[i]*jVel[i]);  }
//   Vec3 w = Rotate(T,Vec3(Vb[0], Vb[1], Vb[2]));
//   Vec3 v = Rotate(T,Vec3(Vb[3], Vb[4], Vb[5]));
//   for(int i=0; i<12; i++) { hapticInput_.tf_mat[i] = T[i];  }
//   for(int i=0; i<3; i++)  { hapticInput_.ang_vel[i] = w[i];   hapticInput_.lin_vel[i] = v[i];            }
//   for(int i=0; i<7; i++)  {
//     hapticInput_.joint_ang[i] = jPos[i];   hapticInput_.joint_vel[i] = jVel[i];  hapticInput_.joint_tor[i] = jTor[i];   }
//   // -----------------------------------------------------------------------------------------------

// }

// CKim - For robot control
void EthercatLifeCycle::RobotControlLaw()
{
  // -----------------------------------------------------------------------------------------------
  /// WRITE YOUR CUSTOM CONTROL ALGORITHM, VARIABLES DECLARATAION HERE, LIKE IN EXAMPLE BELOW.

  // 1. Convert motor pos, vel to joint pos and vels.
  // Kinematically, master device is modeled as a 7 dof serial manipulator by treating
  // 5 bar linkage at the base is as a 2R manipulator
  Eigen::VectorXd jPos(7);      Eigen::VectorXd jVel(7);    Eigen::VectorXd jTor(7);
  for(int i=0; i<7; i++ )  {
    jPos[i] = received_data_.actual_pos[i]*MotorInputCntToOutputRad[i];         // rad
    jVel[i] = received_data_.actual_vel[i]*MotorInputRpmToOutputRad[i];         // rad/s
    jTor[i] = received_data_.actual_tor[i]/JointToTargetTorque[i];              // N.mm
  }
  jPos[2] = jPos[2] - jPos[1];    // CKim - Due to 5 bar linkage structure.
  jVel[2] = jVel[2] - jVel[1];    // CKim - Due to 5 bar linkage structure.

  // 2. Solve forward kinematics and also find linear and  angular velocity in space.
  SE3 T = m_pRobotMdl->ForwardKinematics(jPos);
  Jacobian Jb = m_pRobotMdl->JacobianBody(jPos);
  se3 Vb(0);
  for(int i=0; i<7; i++ )  {  Vb += (Jb[i]*jVel[i]);  }
  Vec3 w = Rotate(T,Vec3(Vb[0], Vb[1], Vb[2]));
  Vec3 v = Rotate(T,Vec3(Vb[3], Vb[4], Vb[5]));
  for(int i=0; i<12; i++) { hapticInput_.tf_mat[i] = T[i];  }
  for(int i=0; i<3; i++)  { hapticInput_.ang_vel[i] = w[i];   hapticInput_.lin_vel[i] = v[i];            }
  for(int i=0; i<7; i++)  {
    hapticInput_.joint_ang[i] = jPos[i];   hapticInput_.joint_vel[i] = jVel[i];  hapticInput_.joint_tor[i] = jTor[i];   }

  // 3. Process Gripper. analog_input_1 of the motor 6 is the ADC value (in mV) of gripper
  // It is around 3350 when fully open and 930 when fully closed
  // analog_input_2 of the motor 6 is the ADC value (in mV) button.
  // It is 4.9 when pressed and 0 when released
  hapticInput_.grasp_val = 100.0 - (received_data_.analog_input_1[6] - 930.0)/(3350.0-930.0)*100.0;
  hapticInput_.button_pressed = ( (received_data_.analog_input_2[6] > 2500) ? 1 : 0);
  for(int i=0; i<3; i++)  { hapticInput_.pedal_button[i] = m_pedalInput[i];  }

  // // 3-1. Write message to send to Microsurgery robot
  // char robotStr[1024];
  // sprintf(robotStr, ",DEV%d %lf %lf %lf %lf %lf %lf %lf %d %d %d %lf", m_isRightHand,
  //         jPos[0], jPos[1], jPos[2], jPos[3], jPos[4], jPos[5], jPos[6],
  //         m_pedalInput[0], m_pedalInput[2], 0,
  //         hapticInput_.grasp_val/100.0);
  // microsurgery_robot_cmd_.data = robotStr;

  // 4. Calculate gravity
  Eigen::VectorXd gTq(7);
  gTq = m_pRobotMdl->GravityTorque(jPos);

  //double gTqWeight[7] = { 0, 0, 0, 0, 0, 0, 0};
  //double gTqWeight[7] = { 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 0.0};
  double gTqWeight[7] = { 1.0, 0.9, 0.9, 1.0, 1.0, 1.0, 0.0};
  for(int i=0; i<7; i++)  { gTq[i]*=gTqWeight[i];  }

  // 5. Calculate Coriolis Torque kg.mm*mm/s^2 = 1000 N.mm
  Eigen::MatrixXd C = Eigen::MatrixXd::Zero(7,7);
  Eigen::VectorXd cTq(7);
  C = m_pRobotMdl->CoriolisMatrix(jPos,jVel);
  cTq = C*jVel;

  double cTqWeight[7] = { 0.0, 0.0, 0.0, 0.9, 0.9, 0.9, 0.0};
  for(int i=0; i<7; i++)  { cTq[i]/=1000.0; cTq[i]*=cTqWeight[i];  }

  // 6. Add some damping for stability
  Eigen::VectorXd dampingTq(7);
  double dampingCoeff[7] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  for(int i=0; i<7; i++)  {
    dampingTq[i] = dampingCoeff[i]*jVel[i];
  }

  // CKim - Same needs to be done for the damping torque
  dampingTq[1] -= dampingTq[2];

  // 7. Convert to motor torque
  for(int i=0; i<7; i++ )  {
    sent_data_.target_tor[i] = (gTq[i] + dampingTq[i] + cTq[i])*JointToTargetTorque[i];
  }

  // CKim - Memo.
  // Axis 6 static friction compensation.
  // Axis 4, 5 viscous friction compensation and inertia compensation -> Done
  // Axis 3 : Coriolis torque? compensation? -> Done
  // Axis 2,1 : Vibration in downward motion. Since this only happens when we are moving downward, direction which
  // the joint is heavily loaded, this is probably due to the stiction friction in motor.
  // Reduce the gain of the motor torque control to make user feelthe friction less.
  // Axis 0 : Too heavy
  // Need Friction compensation for 6, and 2 especially.
  // -----------------------------------------------------------------------------------------------
}

void EthercatLifeCycle::WriteToSlaves()
{
  //  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Writing to slaves....\n");
  if (!emergency_status_ || !gui_node_data_)
  {
    // CKim - In case of error
    for (int i = 0; i < g_kNumberOfServoDrivers; i++)
    {
      if(motor_opmode_[i] == kCSTorque)
      {
        // CKim - Torque
        EC_WRITE_U16(ecat_node_->slaves_[i].slave_pdo_domain_ + ecat_node_->slaves_[i].offset_.control_word,
                    sent_data_.control_word[i]);
        EC_WRITE_S16(ecat_node_->slaves_[i].slave_pdo_domain_ + ecat_node_->slaves_[i].offset_.target_tor,
                    sent_data_.target_tor[i]);
        // EC_WRITE_S16(ecat_node_->slaves_[i].slave_pdo_domain_ + ecat_node_->slaves_[i].offset_.torque_offset,
        //              sent_data_.tor_offset[i]);
      }
      if(motor_opmode_[i] == kCSVelocity)
      {
        EC_WRITE_U16(ecat_node_->slaves_[i].slave_pdo_domain_ + ecat_node_->slaves_[i].offset_.control_word,
                    sent_data_.control_word[i]);
        EC_WRITE_S32(ecat_node_->slaves_[i].slave_pdo_domain_ + ecat_node_->slaves_[i].offset_.target_vel, 0);
      }
    }
  }
  else
  {
    for (int i = 0; i < g_kNumberOfServoDrivers; i++)
    {
      if(motor_opmode_[i] == kCSTorque)
      {
        EC_WRITE_U16(ecat_node_->slaves_[i].slave_pdo_domain_ + ecat_node_->slaves_[i].offset_.control_word,
                    sent_data_.control_word[i]);
        EC_WRITE_S16(ecat_node_->slaves_[i].slave_pdo_domain_ + ecat_node_->slaves_[i].offset_.target_tor,
                    sent_data_.target_tor[i]);
        EC_WRITE_S16(ecat_node_->slaves_[i].slave_pdo_domain_ + ecat_node_->slaves_[i].offset_.torque_offset,
                    sent_data_.tor_offset[i]);
      }
      if(motor_opmode_[i] == kCSVelocity)
      {
        EC_WRITE_U16(ecat_node_->slaves_[i].slave_pdo_domain_ + ecat_node_->slaves_[i].offset_.control_word,
                    sent_data_.control_word[i]);
        EC_WRITE_S32(ecat_node_->slaves_[i].slave_pdo_domain_ + ecat_node_->slaves_[i].offset_.target_vel, sent_data_.target_vel[i]);
      }
    }
  }
}

