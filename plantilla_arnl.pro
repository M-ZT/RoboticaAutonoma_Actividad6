#-------------------------------------------------
#
# Project created by QtCreator 2013-11-16T19:42:26
#
#-------------------------------------------------

QT       += core

QT       -= gui

TARGET = plantilla_arnl
CONFIG   += console
CONFIG   -= app_bundle

TEMPLATE = app
INCLUDEPATH += /usr/local/Aria/include \
                /usr/local/Aria/ArNetworking/include \
                /usr/local/Arnl/include
QMAKE_LIBDIR = /usr/local/Aria/lib/ \
                /usr/local/Arnl/lib
LIBS += -lAria -lArNetworking -lBaseArnl -lArnl

SOURCES += main.cpp \
    control.cpp

HEADERS += \
    control.h
