QT += core widgets opengl openglwidgets

CONFIG += c++17
CONFIG -= app_bundle

TARGET = ikd_tree_qt_demo
TEMPLATE = app

# 定义项目类型
DEFINES += QT_DEPRECATED_WARNINGS

# 包含ikd-Tree库头文件
INCLUDEPATH += .

# 源文件
SOURCES += \
    ikd_tree_qt_demo.cpp \
    qt_point_cloud_viewer.cpp

# 头文件  
HEADERS += \
    ikd_Tree_qt.hpp \
    qt_point_cloud_viewer.h

# Windows特定设置
win32 {
    CONFIG += console
    CONFIG -= app_bundle
    LIBS += -lopengl32 -lglu32
}

# Unix/Linux特定设置
unix {
    LIBS += -lGL -lGLU
}

# 编译器标志
QMAKE_CXXFLAGS += -std=c++17

# 删除无效的预编译依赖
