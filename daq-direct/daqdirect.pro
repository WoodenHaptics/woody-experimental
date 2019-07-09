TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt
CONFIG += c++11;

SOURCES += main.cpp
QMAKE_CXXFLAGS += -std=c++11

unix:!macx: LIBS += -l826_64
INCLUDEPATH += $$PWD/../../fs_polhem/haptikfabrikenapi/external/sensoray
DEPENDPATH += $$PWD/../../fs_polhem/haptikfabrikenapi/external/sensoray
