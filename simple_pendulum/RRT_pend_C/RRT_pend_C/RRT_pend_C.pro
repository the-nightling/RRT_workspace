TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.c \
    RRT.c \
    kdtree.c

HEADERS += \
    RRT.h \
    kdtree.h

