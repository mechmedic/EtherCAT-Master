/*****************************************************************************
 * This is an example code for using 'epos_ntwk' and 'ecat_slave' class to
 * connect EPOS4 slaves to EtherCAT master implemented by IgH EtherLab library
 *
 * Typical process of setting up the EtherCAT master and connecting slaves
 * are as below.
 *
 * 1. Open and request master, create process data domain
 * 2. Find connected slaves and initialize them
 * 3. Configure slaves (PDO mapping, distributed clock, parameters ...)
 * 4. Activate master and register domain data
 * 5. Run cyclic data exchange (receive and process, update data,
 *    queue and send, sychronize distributed clock if slave supports it)
 * 6. Stop cyclic data exchange, cleanup and close
 *
 * Steps 1,2,4 and 6 are common for applications. On the other hand, the
 * steps 3 and 5 will be application specific because each application will
 * have different slaveswith different configurations, data for
 * PDO communications, and processing of the data during cyclic exchange.
 *
 * One needs to inherit EthercatNtwk class and override MapPdos(),
 * ReadFromSlaves() and WriteToSlaves() function according to the
 * slave connection configuration of your fieldbus.
 *******************************************************************************/

#define MAX_SAFE_STACK (8 * 1024) /* The maximum stack size which is
                                     guranteed safe to access without
                                     faulting */
#define NSEC_PER_SEC (1000000000)

#include <EposNtwk.hpp>
#include <EthercatSlave.hpp>

#include <iostream>
#include <memory>
#include <math.h>

using namespace std;
using namespace EthercatCommunication;

void SetComThreadPriorities()
{
     /* Lock memory */
    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
        fprintf(stderr, "Warning: Failed to lock memory: %s\n",
                strerror(errno));
    }


    ////    /* Turn off malloc trimming.*/
    ////    mallopt(M_TRIM_THRESHOLD, -1);

    ////    /* Turn off mmap usage. */
    ////    mallopt(M_MMAP_MAX, 0);

    /* Stack prefault */
    unsigned char dummy[MAX_SAFE_STACK];
    memset(dummy, 0, MAX_SAFE_STACK);

    /* Set priority */
    struct sched_param param = {};
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);

    printf("Using priority %i.", param.sched_priority);
    if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
        perror("sched_setscheduler failed");
    }

//    err_ = pthread_attr_init(&ethercat_thread_attr_);
//    if (err_)
//    {
//      RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Error initializing thread attribute  ! ");
//      return -1;
//    }
//    /**********************************************************************************************/
//    // This part is for CPU isolation to dedicate one core for EtherCAT communication.
//    // for this feature to be active fist you have to modify GRUB_CMDLINE_LINUX_DEFAULT in /etc/default/grub
//    // add isolcpus=3 so after editing it will be ; GRUB_CMDLINE_LINUX_DEFAULT = "quiet splash isolcpus=3"
//    // save and exit, and type sudo update-grub and reboot.
//    //  cpu_set_t mask;
//    // CPU_ZERO(&mask);
//    // CPU_SET(3,&mask);

//    // int result = sched_setaffinity(0,sizeof(mask),&mask);
//    /**********************************************************************************************/

//    /* Set a specific stack size  */
//    err_ = pthread_attr_setstacksize(&ethercat_thread_attr_, 4096 * 4096);
//    if (err_)
//    {
//      RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Error setting thread stack size  ! ");
//      return -1;
//    }

//    err_ = pthread_attr_setschedpolicy(&ethercat_thread_attr_, SCHED_FIFO);
//    if (err_)
//    {
//      RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Pthread setschedpolicy failed ! ");
//      return -1;
//    }
//    err_ = pthread_attr_setschedparam(&ethercat_thread_attr_, &ethercat_sched_param_);
//    if (err_)
//    {
//      RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Pthread setschedparam failed ! ");
//      return -1;
//    }
//    /* Use scheduling parameters of attr */
//    err_ = pthread_attr_setinheritsched(&ethercat_thread_attr_, PTHREAD_EXPLICIT_SCHED);
//    if (err_)
//    {
//      RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Pthread setinheritsched failed ! ");
//      return -1;
//    }

}

int main()
{
    // ------------------------------------------------------------- //
    // CKim - 1. Open and request master, create process data domain
    // ------------------------------------------------------------- //
    // CKim - Create unique pointer to EthercatMaster
    auto epos_ntwk = std::make_unique<EposNtwk>();

    // CKim - Open EtherCAT master via command line
    cout << "Opening EtherCAT device...\n";
    if (epos_ntwk->OpenEthercatMaster())      {   return -1;      }

    // CKim - Request master and create process data domain
    // Pass index in case of multiple master
    cout << "Configuring EtherCAT master...\n";
    if (epos_ntwk->ConfigureMaster(0))
    {
      return -1;
    }

    // ------------------------------------------------------------- //
    // CKim - 2. Find connected slaves and initialize them
    // ------------------------------------------------------------- //
    // CKim - Scan for the connected slaves, check if the number of
    // connected slaves is equal to the g_kNumberOfSlaves defined in EposNtwk.hpp.
    // In case of error, close master
    cout << "Getting connected slave informations...\n";
    if (epos_ntwk->CheckNumberOfConnectedSlaves())
    {
      epos_ntwk->ReleaseMaster();
      for (int i = 0; i < g_kNumberOfSlaves; i++)   {
        epos_ntwk->ResetSlave(i);
      }
      return -1;
    }

    // CKim - Read and print out the connected slave info
    ec_slave_info_t info;
    for (int i = 0; i < g_kNumberOfSlaves; i++)
    {
      epos_ntwk->GetSlaveInformation(i, info);
      std::printf("\n");
      std::printf("--------------------Slave Info -------------------------\n");
      std::printf("Slave alias         = %d\n ", info.alias);
      std::printf("Slave position      = %d\n ", info.position);
      std::printf("Slave vendor_id     = 0x%08x\n ", info.vendor_id);
      std::printf("Slave product_code  = 0x%08x\n ", info.product_code);
      std::printf("Slave name          = %s\n ", info.name);
      std::printf("--------------------EOF %d'th Slave Info ----------------\n ", i);
    }

    // CKim - Initialize the connected slaves
    cout << "Configuring  slaves...\n";
    for (int i = 0; i < g_kNumberOfSlaves; i++)
    {
      if (epos_ntwk->InitSlave(i))
      {
        return -1;
      }
    }

    // ------------------------------------------------------------- //
    // CKim - 3. Configure slaves
    // 3-1. Configure PDO mappings, and Distributed Clock Sync
    // 3-2. Setup parameters of the EPOS via SDO,
    // This part will be application specific. Update MapPdos()
    // and implement and add additional parameter configurations.
    // ------------------------------------------------------------- //
    cout << "Mapping default PDOs...\n";
    if (epos_ntwk->MapPdos())  {   return -1;  }

    cout << "Configuring DC synchronization...\n";
    if (DISTRIBUTED_CLOCK)
        epos_ntwk->ConfigDcSyncDefault();

    cout << "Configuration EPOS parameters via SDO...\n";
    for (int i = 0; i < g_kNumberOfServoDrivers; i++)
    {
        cout << "Setting drives to CSV mode...\n";
        CSVelocityModeParam P;
        P.profile_dec = 3e4;
        P.max_profile_vel = 1e3;
        P.quick_stop_dec = 3e4;
        P.interpolation_time_period = 2;
        if(epos_ntwk->SetCyclicSyncVelocityModeParameters(i, P))
        {
            return -1;
        }
    }

    cout << "Enabling Drive via SDO...\n";
    for (int i = 0; i < g_kNumberOfServoDrivers; i++)
    {
        epos_ntwk->EnableDrivesViaSDO(i);
    }

    // ------------------------------------------------------------- //
    // CKim - 4. Activate master and register domain data.
    // This gives us pointer to process data in domain's memory
    // ------------------------------------------------------------- //
    cout << "Activating master...\n";
    if (epos_ntwk->ActivateMaster())  {   return -1;  }

    cout << "Registering master domain...\n";
    if (epos_ntwk->RegisterDomain())  {   return -1;  }

    // May need to wait for operational mode
    //if (ecat_node_->WaitForOperationalMode())

    // ------------------------------------------------------------- //
    // CKim - 5. Run cyclic exchange
    // (receive and process, update data, queue and send)
    // This part will be application specific.
    // Update ReadFromSlaves and WriteToSlaves for your application
    // ------------------------------------------------------------- //
    cout << "Starting cyclic data exchange...\n";

    // CKim - Configure realtime thread priority
    SetComThreadPriorities();

    // CKim - Start cyclic data exchange
    struct timespec wakeup_time, ref_time;
    struct timespec curr_time, begin_time;
    int ret = 0;
    int status_check_counter = 1000;
    int sync_ref_counter = 0;

    printf("Starting RT task with dt=%u ns.\n", PERIOD_NS);
    clock_gettime(CLOCK_TO_USE, &begin_time);
    while (1)
    {
        // -----------------------
        // CKim - Sleep for control period
        clock_gettime(CLOCK_TO_USE, &wakeup_time);
        wakeup_time = timespec_add(wakeup_time, g_cycle_time);
        ret = clock_nanosleep(CLOCK_TO_USE, TIMER_ABSTIME, &wakeup_time, NULL);
        if (ret) {
            fprintf(stderr, "clock_nanosleep(): %s\n", strerror(ret));
            break;
        }

        // -----------------------
        // CKim - Write application time to master (using distributed clock feature)
        // The master has to know the application's time when operating slaves with distributed clocks.
        // This time is used as a common base for all the clocks in the network.
        // The time is not incremented by the master itself, so this method has to be called cyclically
        if(DISTRIBUTED_CLOCK)   {
            epos_ntwk->SetMasterApplicationTime(wakeup_time);   }

        // -----------------------
        // CKim - Periodically check master/domain/slave state
        if (status_check_counter) {   status_check_counter--; }
        else
        {
          ec_master_state_t ms;
          if (epos_ntwk->CheckMasterState(ms) < 0)
          {
              // Do some error handling??
              break;
          }
          else
          {
            epos_ntwk->CheckMasterDomainState();
            status_check_counter = 1000;
          }
        }

        // -----------------------
        // CKim - Update PDO data
        epos_ntwk->ReceiveAndProcess();

        // -----------------------
        // CKim - Read the updated PDO data, do your own processing, and
        // prepare data to send by PDO
        epos_ntwk->ReadFromSlaves();

        // CKim - Generate sinusoidal motion for each joint for dynamic parameter identification
        clock_gettime(CLOCK_TO_USE, &curr_time);
        uint64_t dt = DIFF_NS(curr_time, begin_time);
        dt /= 1000;   // to us.
        float VelCmd = 1.0/3.0*3.141592*cos(2*3.141592/3000000.0*dt);
        epos_ntwk->m_EposData[0].target_vel = VelCmd;

        epos_ntwk->WriteToSlaves();

        // -----------------------
        // CKim - Synchronize reference clock and then synchronize slave clock
        // to the reference clock. . (using distributed clock feature)
        // The reference clock will by synchronized to the time passed in the sync_time parameter.
        // All slave clocks synchronized to the reference clock.
        if(DISTRIBUTED_CLOCK)   {
            if (sync_ref_counter) {  sync_ref_counter--;  }
            else
            {
              clock_gettime(CLOCK_TO_USE, &ref_time);
              epos_ntwk->SyncMasterReferenceClock(ref_time);
              sync_ref_counter = 1;
            }
            epos_ntwk->SyncSlaveClock();
        }

        // -----------------------
        // CKim - Send updated data via PDO
        epos_ntwk->QueueAndSend();
    }


    // ------------------------------------------------------------- //
    // CKim - 6. Stop cyclic data exchange, cleanup and close
    // ------------------------------------------------------------- //

    // CKim - Release master and shutdown master via command line
    cout << "Ending EtherCAT master...\n";
    epos_ntwk->DeactivateMaster();
    epos_ntwk->ReleaseMaster();
    if (epos_ntwk->ShutDownEthercatMaster())
    {
      return -1;
    }

    return 0;
}


//void EthercatLifeCycle::StartPdoExchange(void* instance)
//{
//  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting PDO exchange....\n");

//  // Measurement time in minutes, e.g.
//  uint32_t print_max_min = measurement_time * 60000;
//  uint32_t print_val = 1e4;
//  int error_check = 0;
//  int32_t status_var = 0;
//  struct timespec wake_up_time, time;
//  struct timespec publish_time_start = {}, publish_time_end = {};

//  // get current time
//  int status_check_counter = 1000;
//  int enabled_counter = 0;
//  int try_enable_counter = g_kNumberOfServoDrivers * 5;

//  ec_master_state_t ms;

//  for(int i=0; i<g_kNumberOfServoDrivers; i++)
//  {
//    sent_data_.target_pos[i] = received_data_.actual_pos[i];
//    sent_data_.target_vel[i] = 0;
//    sent_data_.target_tor[i] = 0;
//    sent_data_.tor_offset[i] = 0;
//  }

//  // ------------------------------------------------------- //
//  // CKim - Initialization loop before entring control loop.
//  // Switch On and Enable Driver
//  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Enabling motors...");
//  clock_gettime(CLOCK_TO_USE, &wake_up_time);
//  while (sig && !gui_buttons_status_.b_stop_cyclic_pdo && (enabled_counter < g_kNumberOfServoDrivers))
//  {
//    // CKim - Sleep for 1 ms
//    wake_up_time = timespec_add(wake_up_time, g_cycle_time);
//    clock_nanosleep(CLOCK_TO_USE, TIMER_ABSTIME, &wake_up_time, NULL);
//    ecat_node_->SetMasterApplicationTime(wake_up_time);

//    // CKim - Receive process data
//    ecat_node_->ReceiveAndProcess();
//    ReadFromSlaves();

//    // CKim - Initialize target pos and vel
//    for (int i = enabled_counter; i < g_kNumberOfServoDrivers; i++)
//    {
//      // CKim - Check status and update control words to enable drivers
//      // Returns number of enabled drivers
//      EnableDrivers(i);
//      if (motor_state_[i] == kOperationEnabled)
//      {
//        enabled_counter += 1;
//      }
//      else
//      {
//        enabled_counter = 0;
//      }
//    }

//    // CKim - Periodic printout
//    if (status_check_counter)
//    {
//      status_check_counter--;
//    }
//    else
//    {
//      // Checking master/domain/slaves state every 1sec.
//      if (ecat_node_->CheckMasterState(ms) < 0)
//      {
//        RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Connection error, check your physical connection.");
//        al_state_ = ms.al_states;
//        received_data_.emergency_switch_val = 0;
//        emergency_status_ = 0;
//        // PublishAllData();
//        error_check++;
//        if (error_check == 5)
//          return;
//      }
//      else
//      {
//        ecat_node_->CheckMasterDomainState();
//        error_check = 0;
//        al_state_ = ms.al_states;
//        status_check_counter = 1000;

//        for (int i = 0; i < g_kNumberOfServoDrivers; i++)
//        {
//          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "State of Drive %d : %d\n", i, motor_state_[i]);
//          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Trying to enable motors");
//        }
//        // PublishAllData();
//      }
//    }

//    // CKim - Queue data
//    RobotControlLaw();
//    WriteToSlaves();

//    // CKim - Sync Timer
//    if (g_sync_ref_counter)
//    {
//      g_sync_ref_counter--;
//    }
//    else
//    {
//      clock_gettime(CLOCK_TO_USE, &time);
//      ecat_node_->SyncMasterReferenceClock(time);
//      g_sync_ref_counter = 1;
//    }
//    ecat_node_->SyncSlaveClock();
//    ecat_node_->QueueAndSend();
//    // PublishAllData();

//  }  // while(sig)
//  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "All motors enabled, entering control loop");

//  // ------------------------------------------------------- //
//  // CKim - All motors enabled. Start control loop
//  clock_gettime(CLOCK_TO_USE, &m_begin_time);
//  while (sig && !gui_buttons_status_.b_stop_cyclic_pdo)
//  {
//    wake_up_time = timespec_add(wake_up_time, g_cycle_time);
//    clock_nanosleep(CLOCK_TO_USE, TIMER_ABSTIME, &wake_up_time, NULL);
//    ecat_node_->SetMasterApplicationTime(wake_up_time);

//    // receive process data
//    ecat_node_->ReceiveAndProcess();

//    if (status_check_counter) {   status_check_counter--; }
//    else
//    {
//      // Checking master/domain/slaves state every 1sec.
//      if (ecat_node_->CheckMasterState(ms) < 0)
//      {
//        RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Connection error, check your physical connection.");
//        al_state_ = ms.al_states;
//        received_data_.emergency_switch_val = 0;
//        emergency_status_ = 0;
//        PublishAllData();
//        error_check++;
//        if (error_check == 5)
//          return;
//      }
//      else
//      {
//        // ecat_node_->CheckMasterDomainState();
//        error_check = 0;
//        al_state_ = ms.al_states;
//        status_check_counter = 1000;
//      }
//    }
//    ReadFromSlaves();
//    RobotControlLaw();
//    WriteToSlaves();

//    if (g_sync_ref_counter) {  g_sync_ref_counter--;  }
//    else
//    {
//      clock_gettime(CLOCK_TO_USE, &time);
//      ecat_node_->SyncMasterReferenceClock(time);
//      g_sync_ref_counter = 1;
//    }

//    ecat_node_->SyncSlaveClock();
//    ecat_node_->QueueAndSend();

//    // m_Counter++;
//    // if(m_Counter % 4 == 0)
//    // {
//      PublishAllData();
//    // }
//  }  // while(1/sig) //Ctrl+C signal

//  // ------------------------------------------------------- //
//  // CKim - Disable drivers before exiting
//  wake_up_time = timespec_add(wake_up_time, g_cycle_time);
//  clock_nanosleep(CLOCK_TO_USE, TIMER_ABSTIME, &wake_up_time, NULL);
//  ecat_node_->SetMasterApplicationTime(wake_up_time);
//  ecat_node_->ReceiveAndProcess();
//  ReadFromSlaves();

//  for (int i = 0; i < g_kNumberOfServoDrivers; i++)
//  {
//    sent_data_.control_word[i] = SM_GO_SWITCH_ON_DISABLE;
//  }
//  RobotControlLaw();
//  WriteToSlaves();
//  ecat_node_->QueueAndSend();
//  // ------------------------------------------------------- //

//  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Leaving control thread.");
//  pthread_exit(NULL);
//}  // StartPdoExchange end

//// --------------------------------------------------------------------------------------------










////    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Waiting for operational mode...\n");
////     if (ecat_node_->WaitForOperationalMode())
////     {
////       RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Error : Time out occurred while waiting for OP mode.!  ");
////       on_cleanup(this->get_current_state());
////       if(received_data_.current_lifecycle_state==PRIMARY_STATE_UNCONFIGURED)
////       on_configure(this->get_current_state());
////       return -1;
////     }
////     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Initialization succesful...\n");

////     if (SetComThreadPriorities())
////     {
////       return -1;
////     }

////   void EthercatLifeCycle::StartPdoExchange(void* instance)




