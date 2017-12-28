#-------------------------------------------------
#
# Project created by QtCreator 2017-12-21T14:30:09
#
#-------------------------------------------------

QT       += core gui serialport

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = CameraTracking
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    serialportforservomotor.cpp

HEADERS  += mainwindow.h \
    serialportforservomotor.h

FORMS    += mainwindow.ui

win32: LIBS += -L$$PWD/libs/ -llibopencv_core.dll
win32: LIBS += -L$$PWD/libs/ -llibopencv_calib3d.dll
win32: LIBS += -L$$PWD/libs/ -llibopencv_features2d.dll
win32: LIBS += -L$$PWD/libs/ -llibopencv_highgui.dll
win32: LIBS += -L$$PWD/libs/ -llibopencv_imgcodecs.dll
win32: LIBS += -L$$PWD/libs/ -llibopencv_imgproc.dll
win32: LIBS += -L$$PWD/libs/ -llibopencv_video.dll
win32: LIBS += -L$$PWD/libs/ -llibopencv_videoio.dll
win32: LIBS += -L$$PWD/libs/ -llibopencv_videostab.dll

INCLUDEPATH += $$PWD/opencv2
DEPENDPATH += $$PWD/opencv2
