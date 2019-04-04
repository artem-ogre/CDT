QT += core gui widgets opengl

TEMPLATE = app

SOURCES += \
        main.cpp \

INCLUDEPATH += \
    include \
    utils \

HEADERS += \
    include/predicates.h \
    include/CDT.h \
    utils/VerifyTopology.h \
    include/PointRTree.h \
    include/CDTUtils.h \

win32: LIBS += opengl32.lib
