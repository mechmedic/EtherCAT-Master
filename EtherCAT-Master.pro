TEMPLATE = app
CONFIG += console c++17
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += \
        main.cpp \
        src/EposNtwk.cpp \
        src/EthercatNtwk.cpp \
        src/EthercatSlave.cpp \
        src/timing.cpp

DISTFILES += \
    README.md

HEADERS += \
    include/EposNtwk.hpp \
    include/EthercatNtwk.hpp \
    include/EthercatSlave.hpp \
    include/ecat_definitions.hpp \
    include/ecat_globals.hpp \
    include/object_dictionary.hpp \
    include/timing.hpp

INCLUDEPATH += include

# CKim - Additional headers and libraries for EtherCAT library
INCLUDEPATH += /opt/etherlab/include
DEPENDPATH += /opt/etherlab/include
unix:!macx: LIBS += -L/opt/etherlab/lib/ -lethercat
