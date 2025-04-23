TEMPLATE = app
CONFIG += console c++17
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += \
        main.cpp \
        src/asmain.cpp \
        src/ecat_master.cpp \
        src/ecat_slave.cpp \
        src/timing.cpp

DISTFILES += \
    README.md

HEADERS += \
    include/ecat_definitions.hpp \
    include/ecat_globals.hpp \
    include/ecat_master.hpp \
    include/ecat_slave.hpp \
    include/object_dictionary.hpp \
    include/timing.hpp

INCLUDEPATH += include

# CKim - Additional headers and libraries for EtherCAT library
INCLUDEPATH += /opt/etherlab/include
DEPENDPATH += /opt/etherlab/include
unix:!macx: LIBS += -L/opt/etherlab/lib/ -lethercat
