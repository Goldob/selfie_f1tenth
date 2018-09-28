TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.cpp \
    include/process.cpp

HEADERS += \
    include/def.hpp \
    include/process.hpp
