/////****************************************************************************/
////#include "rclcpp/rclcpp.hpp"
/////****************************************************************************/
////#include "ecat_master.hpp"
/////****************************************************************************/
////#include "ecat_lifecycle.hpp"
////std::shared_ptr<EthercatLifeCycleNode::EthercatLifeCycle> ecat_lifecycle_node;

////void signalHandler(int /*signum*/)
////{
////    //sig = 0;
////    usleep(1e6);
////    ecat_lifecycle_node->shutdown();
////}

////int main(int argc, char **argv)
////{
////    // CKim - Configure stdout sream buffer. _IONBF means no buffering. Each I/O operation is written as soon as possible.
////    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    
////    // CKim - Associate 'signalHandler' function with interrupt signal (Ctrl+C key)
////    signal(SIGINT,signalHandler);

////    // CKim - Init ROS2 node. rclcpp stands for ROS Client Library for CPP.
////    // rclcpp provides the standard C++ API for interacting with ROS 2.
////    rclcpp::init(argc, argv);

////    // -----------------------------------------------------------------------------
////    // CKim - Prepare memory for real time performance
////    // https://design.ros2.org/articles/realtime_background.html
    
////    // CKim - Lock this processe's memory. Necessary for real time performance....
////    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
////        RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Mlockall failed, check if you have sudo authority.");
////        return -1;
////    }

    
////    /* Turn off malloc trimming.*/
////    mallopt(M_TRIM_THRESHOLD, -1);

////    /* Turn off mmap usage. */
////    mallopt(M_MMAP_MAX, 0);
////    // -----------------------------------------------------------------------------

////    // CKim - Initialize and launch EthercatLifeCycleNode
////    ecat_lifecycle_node = std::make_unique<EthercatLifeCycleNode::EthercatLifeCycle>();
////    rclcpp::spin(ecat_lifecycle_node->get_node_base_interface());
    
////    // rclcpp::executors::MultiThreadedExecutor exe;
////    // exe.add_node(ecat_lifecycle_node->get_node_base_interface());

////    // exe.spin();

////    // CKim - Terminate node
////    rclcpp::shutdown();
////    return 0;
////}










///*****************************************************************************
// *
// *  $Id$
// *
// *  Copyright (C) 2007-2009  Florian Pose, Ingenieurgemeinschaft IgH
// *
// *  This file is part of the IgH EtherCAT Master.
// *
// *  The IgH EtherCAT Master is free software; you can redistribute it and/or
// *  modify it under the terms of the GNU General Public License version 2, as
// *  published by the Free Software Foundation.
// *
// *  The IgH EtherCAT Master is distributed in the hope that it will be useful,
// *  but WITHOUT ANY WARRANTY; without even the implied warranty of
// *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General
// *  Public License for more details.
// *
// *  You should have received a copy of the GNU General Public License along
// *  with the IgH EtherCAT Master; if not, write to the Free Software
// *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
// *
// *  ---
// *
// *  The license mentioned above concerns the source code only. Using the
// *  EtherCAT technology and brand is only permitted in compliance with the
// *  industrial property and similar rights of Beckhoff Automation GmbH.
// *
// ****************************************************************************/

//#include <errno.h>
//#include <signal.h>
//#include <stdio.h>
//#include <string.h>
//#include <sys/resource.h>
//#include <sys/time.h>
//#include <sys/types.h>
//#include <unistd.h>
//#include <time.h> /* clock_gettime() */
//#include <sys/mman.h> /* mlockall() */
//#include <sched.h> /* sched_setscheduler() */

///****************************************************************************/

//#include "ecrt.h"

///****************************************************************************/

///** Task period in ns. */
//#define PERIOD_NS   (1000000)

//#define MAX_SAFE_STACK (8 * 1024) /* The maximum stack size which is
//                                     guranteed safe to access without
//                                     faulting */

///****************************************************************************/

///* Constants */
//#define NSEC_PER_SEC (1000000000)
//#define FREQUENCY (NSEC_PER_SEC / PERIOD_NS)

///****************************************************************************/

//// EtherCAT
//static ec_master_t *master = NULL;
//static ec_master_state_t master_state = {};

//static ec_domain_t *domain1 = NULL;
//static ec_domain_state_t domain1_state = {};

//static ec_slave_config_t *sc_ana_in = NULL;
//static ec_slave_config_state_t sc_ana_in_state = {};

///****************************************************************************/

//// process data
//static uint8_t *domain1_pd = NULL;

//static unsigned int counter = 0;
//static uint8_t buzz = 0;
//static uint8_t SegData = 0;

//static uint32_t offsetAlarm;
//static uint32_t offsetTemperature;
//static uint32_t offsetSegment;
//static uint32_t offsetPot;
//static uint32_t offsetSwitch;

//float TempData;
//uint16_t PotData;
//uint8_t SwitchData;

//void check_domain1_state(void)
//{
//    ec_domain_state_t ds;

//    ecrt_domain_state(domain1, &ds);

//    if (ds.working_counter != domain1_state.working_counter) {
//        printf("Domain1: WC %u.\n", ds.working_counter);
//    }
//    if (ds.wc_state != domain1_state.wc_state) {
//        printf("Domain1: State %u.\n", ds.wc_state);
//    }

//    domain1_state = ds;
//}

///*****************************************************************************/

//void check_master_state(void)
//{
//    ec_master_state_t ms;

//    ecrt_master_state(master, &ms);

//    if (ms.slaves_responding != master_state.slaves_responding) {
//        printf("%u slave(s).\n", ms.slaves_responding);
//    }
//    if (ms.al_states != master_state.al_states) {
//        printf("AL states: 0x%02X.\n", ms.al_states);
//    }
//    if (ms.link_up != master_state.link_up) {
//        printf("Link is %s.\n", ms.link_up ? "up" : "down");
//    }

//    master_state = ms;
//}

///*****************************************************************************/

//void check_slave_config_states(void)
//{
//    ec_slave_config_state_t s;

//    ecrt_slave_config_state(sc_ana_in, &s);

//    if (s.al_state != sc_ana_in_state.al_state) {
//        printf("AnaIn: State 0x%02X.\n", s.al_state);
//    }
//    if (s.online != sc_ana_in_state.online) {
//        printf("AnaIn: %s.\n", s.online ? "online" : "offline");
//    }
//    if (s.operational != sc_ana_in_state.operational) {
//        printf("AnaIn: %soperational.\n", s.operational ? "" : "Not ");
//    }

//    sc_ana_in_state = s;
//}

///*****************************************************************************/

//void cyclic_task()
//{

//}

///****************************************************************************/

//void stack_prefault(void)
//{
//    unsigned char dummy[MAX_SAFE_STACK];

//    memset(dummy, 0, MAX_SAFE_STACK);
//}

///****************************************************************************/

//int main(int argc, char **argv)
//{

//    // CKim - 3. Configure slaves
//    // Tell master about the topology of the slave networks.
//    // This can be done by creating “slave configurations” that will provide
//    // bus position, vendor id and product code.
//    // ecrt_master_slave_config(master, alias, position, vendor_id, product code)
//    // If alias is 0, slaves are addressed by absolute position,
//    // otherwise, position 'p 'means p'th slave after the slave with the 'alias' name.

//    // alias, position, vendor_id and product code can be found by connecting the slaves
//    // and running command line program 'ethercat cstruct'
//    // In this eample, two EasyCAT slaves are connected.
//    ec_slave_config_t *sc1;
//    ec_slave_config_t *sc2;
//    uint32_t VendorID_EasyCAT = 0x0000079a;
//    uint32_t ProductCode_LAB1 = 0xababa001;
//    uint32_t ProductCode_LAB2 = 0xababa002;

//    if (!(sc1 = ecrt_master_slave_config(master,0,0,VendorID_EasyCAT,ProductCode_LAB1))) {
//        fprintf(stderr, "Failed to get slave configuration.\n");
//        return -1;
//    }
//    if (!(sc2 = ecrt_master_slave_config(master,0,1,VendorID_EasyCAT,ProductCode_LAB2))) {
//        fprintf(stderr, "Failed to get slave configuration.\n");
//        return -1;
//    }
//    printf("Configured Slaves!!\n");






//    // CKim - Configure realtime thread priority
//    /* Set priority */
//    struct sched_param param = {};
//    param.sched_priority = sched_get_priority_max(SCHED_FIFO);

//    printf("Using priority %i.", param.sched_priority);
//    if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
//        perror("sched_setscheduler failed");
//    }

//    /* Lock memory */
//    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
//        fprintf(stderr, "Warning: Failed to lock memory: %s\n",
//                strerror(errno));
//    }

//    stack_prefault();

//    // CKim 8. - Start cyclic data exchange
//    struct timespec wakeup_time;
//    int ret = 0;

//    printf("Starting RT task with dt=%u ns.\n", PERIOD_NS);

//    clock_gettime(CLOCK_MONOTONIC, &wakeup_time);
//    wakeup_time.tv_sec += 1; /* start in future */
//    wakeup_time.tv_nsec = 0;

//    while (1) {
//        ret = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME,
//                &wakeup_time, NULL);
//        if (ret) {
//            fprintf(stderr, "clock_nanosleep(): %s\n", strerror(ret));
//            break;
//        }

//        cyclic_task();

//        wakeup_time.tv_nsec += PERIOD_NS;
//        while (wakeup_time.tv_nsec >= NSEC_PER_SEC) {
//            wakeup_time.tv_nsec -= NSEC_PER_SEC;
//            wakeup_time.tv_sec++;
//        }
//    }

//    return ret;
//}

///****************************************************************************/
