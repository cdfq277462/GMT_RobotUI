QT       += core gui
QT       += network

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++11

# The following define makes your compiler emit warnings if you use
# any Qt feature that has been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    main.cpp \
    mainwindow.cpp \
    robodk_api.cpp \
    toolbox.cpp

HEADERS += \
    GenaricStructure.h \
    mainwindow.h \
    robodk_api.h \
    toolbox.h

FORMS += \
    mainwindow.ui

INCLUDEPATH += /usr/local/include/opencv4 \
             /usr/local/include/
LIBS += -L/usr/local/lib -lopencv_core -lopencv_imgproc -lopencv_imgcodecs -lopencv_ximgproc -lopencv_highgui \
        -L/home/draw/Desktop/RoboDK_EC612/RoboDK-API/C++/Example/include -limagemomentequalellipse -ladativethreshold

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
