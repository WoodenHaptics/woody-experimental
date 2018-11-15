TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt
CONFIG += c++11;

SOURCES += main.cpp
QMAKE_CXXFLAGS += -std=c++11

unix:!macx: LIBS += -L$$PWD/../chai3d-3.0.0-woody/external/s826/lib/ -l826_64
INCLUDEPATH += $$PWD/../chai3d-3.0.0-woody/external/s826/include
DEPENDPATH += $$PWD/../chai3d-3.0.0-woody/external/s826/include
unix:!macx: PRE_TARGETDEPS += $$PWD/../../chai3d-3.0.0-woody/external/s826/lib/lib826_64.a
