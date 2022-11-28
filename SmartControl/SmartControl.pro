#-------------------------------------------------
#
# Project created by QtCreator 2022-11-19T14:09:53
#
#-------------------------------------------------

QT       += core gui
QT +=network
QT +=multimedia

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = SmartControl
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp

HEADERS  += mainwindow.h

FORMS    += mainwindow.ui

DISTFILES += \
    sound/1.wav \


RESOURCES += \
#    images/res.qrc \
#    good.qrc
    sourse.qrc
