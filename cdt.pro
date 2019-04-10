QT += core gui widgets opengl

TEMPLATE = app


SOURCES += \
    main.cpp \

# Uncomment to not use boost::rtree
DEFINES += CDT_DONT_USE_BOOST_RTREE

# Uncomment to use as compiled library
DEFINES += CDT_USE_AS_COMPILED_LIBRARY
SOURCES += include/CDT.cpp

INCLUDEPATH += \
    include \
    utils \

HEADERS += \
    include/predicates.h \
    include/CDT.h \
    include/CDT.hpp \
    utils/VerifyTopology.h \
    include/PointRTree.h \
    include/CDTUtils.h \
    include/CDTUtils.hpp \

win32: LIBS += opengl32.lib
