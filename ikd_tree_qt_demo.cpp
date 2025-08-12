#include <QApplication>
#include <QStyleFactory>
#include <QDir>
#include <QStandardPaths>
#include <QMessageBox>
#include <QSplashScreen>
#include <QPixmap>
#include <QPainter>
#include <QFont>
#include <QTimer>
#include "qt_point_cloud_viewer.h"

/**
 * @brief 创建启动画面
 * @param app 应用程序实例
 * @return 启动画面指针
 */
QSplashScreen* createSplashScreen(QApplication* app)
{
    // 创建启动画面
    QPixmap splashPixmap(400, 300);
    splashPixmap.fill(QColor(45, 52, 54));
    
    QPainter painter(&splashPixmap);
    painter.setRenderHint(QPainter::Antialiasing);
    
    // 绘制标题
    QFont titleFont("Arial", 24, QFont::Bold);
    painter.setFont(titleFont);
    painter.setPen(QColor(116, 185, 255));
    painter.drawText(splashPixmap.rect(), Qt::AlignCenter, "ikd-Tree Qt Demo");
    
    // 绘制副标题
    QFont subFont("Arial", 12);
    painter.setFont(subFont);
    painter.setPen(QColor(223, 230, 233));
    QRect subRect = splashPixmap.rect();
    subRect.moveTop(subRect.center().y() + 30);
    painter.drawText(subRect, Qt::AlignCenter, u8"高性能空间索引可视化演示");

    // 绘制版本信息
    QFont versionFont("Arial", 10);
    painter.setFont(versionFont);
    painter.setPen(QColor(149, 165, 166));
    QRect versionRect = splashPixmap.rect();
    versionRect.moveTop(versionRect.bottom() - 40);
    painter.drawText(versionRect, Qt::AlignCenter, "Based on Qt6 & OpenGL\nVersion 1.0");
    
    auto* splash = new QSplashScreen(splashPixmap);
    splash->setWindowFlags(splash->windowFlags() | Qt::WindowStaysOnTopHint);
    
    return splash;
}

/**
 * @brief 设置应用程序样式
 * @param app 应用程序实例
 */
void setupApplicationStyle(QApplication* app)
{
    // 设置应用程序样式
    app->setStyle(QStyleFactory::create("Fusion"));
    
    // 设置深色主题
    QPalette darkPalette;
    darkPalette.setColor(QPalette::Window, QColor(53, 53, 53));
    darkPalette.setColor(QPalette::WindowText, QColor(255, 255, 255));
    darkPalette.setColor(QPalette::Base, QColor(25, 25, 25));
    darkPalette.setColor(QPalette::AlternateBase, QColor(53, 53, 53));
    darkPalette.setColor(QPalette::ToolTipBase, QColor(0, 0, 0));
    darkPalette.setColor(QPalette::ToolTipText, QColor(255, 255, 255));
    darkPalette.setColor(QPalette::Text, QColor(255, 255, 255));
    darkPalette.setColor(QPalette::Button, QColor(53, 53, 53));
    darkPalette.setColor(QPalette::ButtonText, QColor(255, 255, 255));
    darkPalette.setColor(QPalette::BrightText, QColor(255, 0, 0));
    darkPalette.setColor(QPalette::Link, QColor(42, 130, 218));
    darkPalette.setColor(QPalette::Highlight, QColor(42, 130, 218));
    darkPalette.setColor(QPalette::HighlightedText, QColor(0, 0, 0));
    
    app->setPalette(darkPalette);
    
    // 设置样式表
    QString styleSheet = R"(
        QGroupBox {
            font-weight: bold;
            border: 2px solid #3498db;
            border-radius: 5px;
            margin: 10px 0px;
            padding-top: 10px;
        }
        
        QGroupBox::title {
            subcontrol-origin: margin;
            left: 10px;
            padding: 0 5px 0 5px;
            color: #3498db;
        }
        
        QPushButton {
            background-color: #3498db;
            border: none;
            padding: 8px 16px;
            border-radius: 4px;
            font-weight: bold;
            color: white;
        }
        
        QPushButton:hover {
            background-color: #2980b9;
        }
        
        QPushButton:pressed {
            background-color: #1c5985;
        }
        
        QPushButton:disabled {
            background-color: #7f8c8d;
        }
        
        QPushButton:checked {
            background-color: #27ae60;
        }
        
        QDoubleSpinBox, QSpinBox {
            padding: 4px;
            border: 1px solid #555;
            border-radius: 3px;
            background-color: #2b2b2b;
        }
        
        QSlider::groove:horizontal {
            border: 1px solid #555;
            height: 8px;
            background: #2b2b2b;
            border-radius: 4px;
        }
        
        QSlider::handle:horizontal {
            background: #3498db;
            border: 1px solid #2980b9;
            width: 16px;
            margin: -4px 0;
            border-radius: 8px;
        }
        
        QTextEdit {
            border: 1px solid #555;
            border-radius: 4px;
            background-color: #1e1e1e;
            color: #ffffff;
        }
        
        QProgressBar {
            border: 1px solid #555;
            border-radius: 4px;
            text-align: center;
            background-color: #2b2b2b;
        }
        
        QProgressBar::chunk {
            background-color: #3498db;
            border-radius: 3px;
        }
    )";
    
    app->setStyleSheet(styleSheet);
}

/**
 * @brief 检查系统要求
 * @return true表示系统满足要求
 */
bool checkSystemRequirements()
{
    // 检查OpenGL支持
    QOpenGLContext context;
    if (!context.create()) {
        QMessageBox::critical(nullptr, u8"系统要求检查", u8"❌ 无法创建OpenGL上下文！\n" "请确保您的系统支持OpenGL。");
        return false;
    }
    
    return true;
}

/**
 * @brief 主函数 - ikd-Tree Qt Demo 程序入口点
 * @param argc 命令行参数数量
 * @param argv 命令行参数数组
 * @return 程序退出状态码
 */
int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    
    // 设置应用程序信息
    app.setApplicationName("ikd-Tree Qt Demo");
    app.setApplicationVersion("1.0");
    app.setOrganizationName("ikd-Tree Team");
    app.setApplicationDisplayName(u8"ikd-Tree Qt 可视化演示");

    // 设置高DPI支持
    app.setAttribute(Qt::AA_EnableHighDpiScaling);
    app.setAttribute(Qt::AA_UseHighDpiPixmaps);
    
    // 检查系统要求
    if (!checkSystemRequirements()) {
        return -1;
    }
    
    // 创建启动画面
    auto* splash = createSplashScreen(&app);
    splash->show();
    app.processEvents();
    
    // 设置应用程序样式
    setupApplicationStyle(&app);

    splash->showMessage(u8"正在初始化组件...", Qt::AlignBottom | Qt::AlignCenter, Qt::white);
    app.processEvents();
    
    // 模拟初始化过程
    QTimer::singleShot(1000, [&]() { splash->showMessage(u8"正在加载ikd-Tree库...", Qt::AlignBottom | Qt::AlignCenter, Qt::white); });

    QTimer::singleShot(1500, [&]() { splash->showMessage(u8"正在准备OpenGL渲染器...", Qt::AlignBottom | Qt::AlignCenter, Qt::white); });

    // 创建主窗口
    QtIkdTreeDemo window;
    QTimer::singleShot(2000, [&]() {
        // 隐藏启动画面并显示主窗口
        splash->finish(&window);
        window.show();
        
        // 清理启动画面
        splash->deleteLater();
    });

    return app.exec();
}

// 不再需要显式实例化 - 使用header-only模板实现
