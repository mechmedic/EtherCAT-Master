# EtherCAT-Master
C++ Class implementing EtherCAT Master using IgH EtherCAT Library
## Basic Usage
1. Class EthercatNtwk wraps basic functions of the EtherCAT library. Inherit this, define your own data structure to store data and PDO offset, override MapPdo, ReadFromSlaves, WriteToSlaves to implement specific PDO data exchange of your application.
2. See main.cpp for typical steps of establishing connection, configuring master and slaves, and running cyclid PDO data exchange.
3. Class EposNtwk has additional functionality for configuring Maxon EPOS vis SDO. 
