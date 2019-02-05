QT += core gui widgets

TEMPLATE = app

SOURCES += \
        main.cpp \

HEADERS += \
    predicates.h \
    CDT.h \
    VerifyTopology.h

win32
{
    INCLUDEPATH += C:/Data/DevTools/boost_1_64_0
    INCLUDEPATH += C:/Projects/GlobalVendor/Polaris/include/Boost/boost_1_61_0
    LIBS += -LC:/Data/DevTools/boost_1_64_0/lib64-msvc-12.0
}
