#include "EthercatNtwk.hpp"

using namespace EthercatCommunication;

//****************************************************************************************/
///// Extern global variable declaration.
//uint32_t g_sync_ref_counter = 0;
///****************************************************************************************

EthercatNtwk::EthercatNtwk()
{
}

EthercatNtwk::~EthercatNtwk()
{
}

int EthercatNtwk::OpenEthercatMaster()
{
  std::cout << "[EthercatNtwk] Trying to open EtherCAT master...\n";
  fd = std::system("ls /dev | grep EtherCAT > /dev/null");
  if (fd)
  {
    std::cout << "[EthercatNtwk] Opening EtherCAT master...\n";
    std::system("sudo ethercatctl start");
    usleep(2e6);
    fd = std::system("ls /dev | grep EtherCAT > /dev/null");
    if (fd)
    {
      std::cerr << "[EthercatNtwk] Error : EtherCAT device not found.\n";
      return -1;
    }
    else
    {
      return 0;
    }
  }
  std::cout << "[EthercatNtwk] EtherCAT master opened...\n";
  return 0;
}

int EthercatNtwk::ConfigureMaster(int idx)
{
  // CKim - 1. Request master
  // To access IgH EtherCAT master inside your application,
  // one must first request the master.
  // 'ecrt_request_master(index)'. Returns pointer to 'ec_master_t'
  std::cout << "[EthercatNtwk] Requesting EtherCAT master...\n";
  if (!m_master)
  {
    m_master = ecrt_request_master(idx);
    if (!m_master)
    {
      std::cerr << "[EthercatNtwk] Requesting master instance failed ! ";
      return -1;
    }
  }

  // CKim - 2. Create a Process Data Domain
  // Image of the process data objects (PDO) that will be
  // exchanged through EtherCAT communication are managed by
  // 'Process Data Domain'.
  // 'ecrt_master_create_domain'. Returns pointer to ec_domain_t
  m_master_domain = ecrt_master_create_domain(m_master);
  if (!m_master_domain)
  {
    std::cerr << "[EthercatNtwk] Failed to create master domain ! ";
    return -1;
  }
  return 0;
}

void EthercatNtwk::ReleaseMaster()
{
  ecrt_release_master(m_master);
  m_master = NULL;
  m_master_domain = NULL;
  m_master_state = {};
}

int EthercatNtwk::ShutDownEthercatMaster()
{
  fd = std::system("ls /dev | grep EtherCAT* > /dev/null\n");
  if (!fd)
  {
    std::cout << "[EthercatNtwk] Shutting down EtherCAT master...";
    std::system("cd ~; sudo ethercatctl stop\n");
    usleep(1e6);
    fd = std::system("ls /dev | grep EtherCAT* > /dev/null\n");
    if (fd)
    {
      std::cout << "[EthercatNtwk] EtherCAT shut down succesfull.";
      return 0;
    }
    else
    {
      std::cerr << "[EthercatNtwk] Error : EtherCAT shutdown error.";
      return -1;
    }
  }
  return 0;
}

int EthercatNtwk::RestartEthercatMaster()
{
  std::cout << "[EthercatNtwk] Restarting EtherCAT master...";
  std::system("cd ~; sudo ethercatctl restart\n");
  usleep(2e6);
  fd = std::system("ls /dev | grep EtherCAT* > /dev/null\n");
  if (!fd)
  {
    std::cout << "[EthercatNtwk] EtherCAT restart succesfull.";
    return 0;
  }
  else
  {
    std::cerr << "[EthercatNtwk] Error : EtherCAT restart error.";
    return -1;
  }
}

int EthercatNtwk::CheckMasterState()
{
  ecrt_master_state(m_master, &m_master_state);
  if (m_master_state.slaves_responding != m_master_state.slaves_responding)
  {
    std::cout << "[EthercatNtwk] " << m_master_state.slaves_responding <<" slave(s).\n";
    if (m_master_state.slaves_responding < 1)
    {
      std::cout <<  "[EthercatNtwk] Connection error,no response from slaves.";
      return -1;
    }
  }
  
  if (m_master_state.al_states != m_master_state.al_states)
  {
    std::printf("[EthercatNtwk] AL states: 0x%02X.\n", m_master_state.al_states);
    //std::cout << "[EthercatNtwk] AL states: 0x" << std::hex << std::setfill('0') <<std::setw(2) << m_master_state.al_states << "\n";
  }
  if (m_master_state.link_up != m_master_state.link_up)
  {
    std::printf("[EthercatNtwk] Link is %s.\n", m_master_state.link_up ? "up" : "down");
    if (!m_master_state.link_up)
    {
      std::cerr << "[EthercatNtwk] Master state link down";
      return -1;
    }
  }
  return 0;
}

void EthercatNtwk::CheckMasterDomainState()
{
  ec_domain_state_t ds;  // Domain instance
  ecrt_domain_state(m_master_domain, &ds);
  if (ds.working_counter != m_master_domain_state.working_counter)
    std::printf("[EthercatNtwk] masterDomain: WC %u.\n", ds.working_counter);
  if (ds.wc_state != m_master_domain_state.wc_state)
    std::printf("[EthercatNtwk] masterDomain: State %u.\n", ds.wc_state);
  if (m_master_domain_state.wc_state == EC_WC_COMPLETE)
  {
    m_master_domain_state = ds;
  }
  m_master_domain_state = ds;
}

int EthercatNtwk::ActivateMaster()
{
  if (ecrt_master_activate(m_master))
  {
    std::cerr << "[EthercatNtwk] Master activation error ! ";
    return -1;
  }
  return 0;
}

int EthercatNtwk::RegisterDomain()
{
  for (int i = 0; i < NUM_OF_SLAVES; i++)
  {
    slaves_[i].slave_pdo_domain_ = ecrt_domain_data(m_master_domain);
    if(!(slaves_[i].slave_pdo_domain_) )
    {
        std::cerr << "[EthercatNtwk] Domain PDO registration error";
        return -1;
    }
  }
  return 0;
}

void EthercatNtwk::DeactivateMaster()
{
  std::cout << "[EthercatNtwk] Deactivating EtherCAT master : Cyclic PDO ended...\n";
  ecrt_master_deactivate(m_master);
}

int EthercatNtwk::GetNumberOfConnectedSlaves()
{
  unsigned int number_of_slaves;
  usleep(1e6);
  ecrt_master_state(m_master, &m_master_state);
  number_of_slaves = m_master_state.slaves_responding;
  if (NUM_OF_SLAVES != number_of_slaves)
  {
    std::cerr << "[EthercatNtwk] Please enter correct number of slaves... \n"
              << "Entered number of slave : " << NUM_OF_SLAVES << "\n"
              << "Connected slaves        : " << number_of_slaves << "\n"; 
    return -1;
  }
  return 0;
}

void EthercatNtwk::GetSlaveInformation(int position, ec_slave_info_t& info)
{
    ecrt_master_get_slave(m_master, position, &info);
}

int EthercatNtwk::InitSlave(int i)
{
  ecrt_master_get_slave(m_master, i, &slaves_[i].slave_info_);
  slaves_[i].slave_config_ = ecrt_master_slave_config(m_master,slaves_[i].slave_info_.alias,
                                                                    slaves_[i].slave_info_.position,
                                                                    slaves_[i].slave_info_.vendor_id,
                                                                    slaves_[i].slave_info_.product_code); 
  if(!slaves_[i].slave_config_) {
      std::cerr << "[EthercatNtwk] Failed to  configure slave ! ";
      return -1;
  }
  return 0 ;
}

void EthercatNtwk::CheckSlaveConfigurationState()
{
  for (int i = 0; i < NUM_OF_SLAVES; i++)
  {
    //slaves_[i].CheckSlaveConfigState();
    ecrt_slave_config_state(slaves_[i].slave_config_, &slaves_[i].slave_config_state_);
    if (slaves_[i].slave_config_state_.al_state != 0x08)
    {
      std::cout << "[EthercatNtwk] Slave "<<i<< " is not operational. AL state is :" << std::hex << slaves_[i].slave_config_state_.al_state << std::endl;
    }
  }
}

void EthercatNtwk::ResetSlave(int i)
{
    slaves_[i].slave_pdo_domain_ = NULL;
    slaves_[i].slave_config_ = NULL;
    slaves_[i].slave_pdo_entry_info_ = NULL;
    slaves_[i].slave_pdo_entry_reg_ = NULL;
    slaves_[i].slave_pdo_info_ = NULL;
    slaves_[i].slave_sync_info_ = NULL;    
}

void EthercatNtwk::ReceiveAndProcess()
{
  ecrt_master_receive(m_master);
  ecrt_domain_process(m_master_domain);
}

void EthercatNtwk::QueueAndSend()
{
  ecrt_domain_queue(m_master_domain);
  ecrt_master_send(m_master);
}

void EthercatNtwk::SetMasterApplicationTime(const timespec& time)
{
  ecrt_master_application_time(m_master, TIMESPEC2NS(time));
}

void EthercatNtwk::SyncMasterReferenceClock(const timespec& time)
{
  ecrt_master_sync_reference_clock_to(m_master, TIMESPEC2NS(time));
}

void EthercatNtwk::SyncSlaveClock()
{
  ecrt_master_sync_slave_clocks(m_master);
}

int EthercatNtwk::WaitForOperationalMode()
{
  //ec_master_state_t ms;
  int try_counter=0;
  int check_state_count=0;
  int time_out = 20e3;
  while (m_master_state.al_states != EC_AL_STATE_OP )
  {
    if(try_counter < time_out)
    {
      clock_gettime(CLOCK_MONOTONIC, &m_sync_timer);
      ecrt_master_application_time(m_master, TIMESPEC2NS(m_sync_timer));
      ecrt_master_receive(m_master);
      ecrt_domain_process(m_master_domain);
      usleep(PERIOD_US);
      if(!check_state_count)
      {
        CheckMasterState();
        CheckMasterDomainState();
        CheckSlaveConfigurationState();
        check_state_count = PERIOD_US ;
      }
      ecrt_domain_queue(m_master_domain);                
      ecrt_master_sync_slave_clocks(m_master);
      ecrt_master_sync_reference_clock_to(m_master, TIMESPEC2NS(m_sync_timer));
      ecrt_master_send(m_master);

      try_counter++;
      check_state_count--;
    }
    else 
    {
      std::cerr << "[EthercatNtwk] Error : Time out occurred while waiting for OP mode.!";
      return -1;
    }
  }
  return 0;
}

int8_t EthercatNtwk::SdoRead(SDO_data& pack)
{
  if (ecrt_master_sdo_upload(m_master, pack.slave_position, pack.index, pack.sub_index, (uint8_t*)(&pack.data),
                             pack.data_sz, &pack.result_sz, &pack.err_code))
  {
    std::printf("[EthercatNtwk] SDO read error, code: %d \n", &pack.err_code);
    return -1;
  }
  return 0;
}

int8_t EthercatNtwk::SdoWrite(SDO_data& pack)
{
  if (ecrt_master_sdo_download(m_master, pack.slave_position, pack.index, pack.sub_index, (uint8_t*)(&pack.data),
                               pack.data_sz, &pack.err_code))
  {
    std::printf("[EthercatNtwk] SDO write error, code : %d \n", &pack.err_code);
    return -1;
  }
  return 0;
}


