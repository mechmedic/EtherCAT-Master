TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += \
        main.cpp \
        src/ecat_master.cpp \
        src/ecat_slave.cpp \
        src/main.cpp \
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
