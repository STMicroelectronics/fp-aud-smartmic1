
TEMPLATE = app

CONFIG += console

# ----------------------------------------------------
#### Uncomment the following lines for non Qt version
CONFIG -= qt
DEFINES += CCOMPORT_NO_QT
HEADERS += ASTSerialLib/Include/CComPort_noQt.h
SOURCES += ASTSerialLib/Source/CComPort_noQt.cpp

#### Qt version: Uncomment the following lines
#QT += core serialport
#HEADERS += ASTSerialLib/Include/CComPort.h
#SOURCES += ASTSerialLib/Source/CComPort.cpp
# ----------------------------------------------------

TARGET = AudioSerialLib_Example

DEPENDPATH += . \
              src \
              ASTSerialLib/Include \
              ASTSerialLib/Source

INCLUDEPATH += src \
               ASTSerialLib/Include \
               AudioSerialLib \


# Input
HEADERS += ASTSerialLib/Include/Serial_CMD.h \
           ASTSerialLib/Include/ASTSerialLib.h \
           ASTSerialLib/Include/CCronometer.h \
           ASTSerialLib/Include/serial_protocol.h \
           ASTSerialLib/Include/SerialComm.h \
           AudioSerialLib/AudioModuleSerialLib.h

SOURCES += src/AudioSerialLib_Example.cpp \
           ASTSerialLib/Source/ASTSerialLib.cpp \
           ASTSerialLib/Source/CCronometer.cpp \
           ASTSerialLib/Source/serial_protocol.c \
           ASTSerialLib/Source/SerialComm.cpp \
           AudioSerialLib/AudioModuleSerialLib.cpp \


!win32 {
    HEADERS += src/kbhit.h
    SOURCES += src/kbhit.c
}
