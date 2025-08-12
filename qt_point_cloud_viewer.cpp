#include "qt_point_cloud_viewer.h"
#include <QApplication>
#include <QRandomGenerator>
#include <QElapsedTimer>
#include <QMessageBox>
#include <QtMath>
#include <QDebug>
#include <QTime>
#ifdef _WIN32
#include <Windows.h>
#include <gl/GL.h>
#else
#include <GL/gl.h>
#endif

// ==================== QtPointCloudViewer 实现 ====================

QtPointCloudViewer::QtPointCloudViewer(QWidget* parent)
    : QOpenGLWidget(parent)
    , m_cameraPos(0.0f, 0.0f, 50.0f)
    , m_cameraTarget(0.0f, 0.0f, 0.0f)
    , m_cameraUp(0.0f, 1.0f, 0.0f)
    , m_cameraDistance(50.0f)
    , m_cameraYaw(0.0f)
    , m_cameraPitch(0.0f)
    , m_mousePressed(false)
    , m_searchRadius(5.0)
    , m_pointSize(2.0f)
    , m_showOriginal(true)
    , m_showSearched(true)
    , m_showRadius(true)
    , m_showSearchBox(true)
    , m_showRadiusSphere(true)
{
    setFocusPolicy(Qt::StrongFocus);
    
    // 初始化搜索参数
    m_radiusCenter = Vector3D(0.0, 0.0, 0.0);
    
    m_searchBox.vertex_min[0] = -5.0;
    m_searchBox.vertex_min[1] = -5.0;
    m_searchBox.vertex_min[2] = -5.0;
    m_searchBox.vertex_max[0] = 5.0;
    m_searchBox.vertex_max[1] = 5.0;
    m_searchBox.vertex_max[2] = 5.0;
}

void QtPointCloudViewer::initializeGL()
{
    initializeOpenGLFunctions();
    
    // 设置OpenGL状态
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
    glEnable(GL_PROGRAM_POINT_SIZE);
    glPointSize(m_pointSize);
    
    // 启用点平滑
    glEnable(GL_POINT_SMOOTH);
    glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);
    
    // 启用混合以支持透明度
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    
    // 设置背景色
    glClearColor(0.1f, 0.1f, 0.15f, 1.0f);
    
    updateCameraPosition();
    
    qDebug() << "OpenGL初始化完成 - 版本:" << (char*)glGetString(GL_VERSION);
}

void QtPointCloudViewer::resizeGL(int width, int height)
{
    glViewport(0, 0, width, height);
    
    // 设置投影矩阵
    m_projection.setToIdentity();
    m_projection.perspective(45.0f, float(width) / float(height), 0.1f, 1000.0f);
}

void QtPointCloudViewer::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    // 设置投影和视图矩阵
    glMatrixMode(GL_PROJECTION);
    glLoadMatrixf(m_projection.constData());
    
    glMatrixMode(GL_MODELVIEW);
    m_view.setToIdentity();
    m_view.lookAt(m_cameraPos, m_cameraTarget, m_cameraUp);
    glLoadMatrixf(m_view.constData());
    
    // 先渲染原始点云（作为背景）
    if (m_showOriginal && !m_originalCloud.isEmpty()) {
        glPointSize(qMax(m_pointSize - 1.0f, 2.0f));  // 原始点云稍小
        renderPointCloud(m_originalCloud, QVector3D(0.5f, 0.5f, 0.5f));  // 更暗的颜色
        qDebug() << "渲染原始点云，点数:" << m_originalCloud.size();
    }
    
    // 然后渲染搜索结果（较大的点，覆盖原始点）
    if (m_showSearched && !m_searchedCloud.isEmpty()) {
        glPointSize(qMax(m_pointSize + 2.0f, 5.0f));  // 搜索结果点更大
        glDisable(GL_DEPTH_TEST);  // 临时禁用深度测试，确保搜索结果显示在前面
        renderPointCloud(m_searchedCloud, QVector3D(1.0f, 0.0f, 0.0f));
        glEnable(GL_DEPTH_TEST);   // 重新启用深度测试
        qDebug() << "渲染搜索结果，点数:" << m_searchedCloud.size();
    }
    
    // 渲染半径搜索结果
    if (m_showRadius && !m_radiusCloud.isEmpty()) {
        glPointSize(qMax(m_pointSize + 2.0f, 5.0f));  // 半径搜索结果也更大
        glDisable(GL_DEPTH_TEST);  // 临时禁用深度测试
        renderPointCloud(m_radiusCloud, QVector3D(0.0f, 1.0f, 0.0f));
        glEnable(GL_DEPTH_TEST);   // 重新启用深度测试
        qDebug() << "渲染半径搜索结果，点数:" << m_radiusCloud.size();
    }
    
    // 渲染搜索包围盒
    if (m_showSearchBox) {
        renderBoundingBox(m_searchBox, QVector3D(1.0f, 1.0f, 0.0f));
    }
    
    // 渲染搜索球体
    if (m_showRadiusSphere) {
        renderSphere(m_radiusCenter, m_searchRadius, QVector3D(0.0f, 1.0f, 1.0f));
    }
}

void QtPointCloudViewer::renderPointCloud(const ColoredPointCloud& cloud, const QVector3D& defaultColor)
{
    // 点大小现在在外部设置，这里不再设置
    glBegin(GL_POINTS);
    
    int pointCount = 0;
    for (const auto& point : cloud) {
        // 使用点的颜色，如果没有设置颜色则使用默认颜色
        QVector3D color = (point.color.x() >= 0.0f && point.color.y() >= 0.0f && point.color.z() >= 0.0f) 
                         ? point.color : defaultColor;
        glColor3f(color.x(), color.y(), color.z());
        glVertex3d(point.position.x(), point.position.y(), point.position.z());
        pointCount++;
    }
    
    glEnd();
    
    // 只在第一次渲染时输出调试信息
    static bool firstRender = true;
    if (firstRender) {
        qDebug() << "实际渲染点数:" << pointCount << "点大小:" << qMax(m_pointSize, 3.0f) 
                 << "默认颜色:" << defaultColor.x() << defaultColor.y() << defaultColor.z();
        firstRender = false;
    }
}

void QtPointCloudViewer::renderBoundingBox(const BoxPointType& box, const QVector3D& color)
{
    glColor3f(color.x(), color.y(), color.z());
    glLineWidth(2.0f);
    
    // 绘制包围盒的12条边
    glBegin(GL_LINES);
    
    // 底面4条边
    glVertex3d(box.vertex_min[0], box.vertex_min[1], box.vertex_min[2]);
    glVertex3d(box.vertex_max[0], box.vertex_min[1], box.vertex_min[2]);
    
    glVertex3d(box.vertex_max[0], box.vertex_min[1], box.vertex_min[2]);
    glVertex3d(box.vertex_max[0], box.vertex_max[1], box.vertex_min[2]);
    
    glVertex3d(box.vertex_max[0], box.vertex_max[1], box.vertex_min[2]);
    glVertex3d(box.vertex_min[0], box.vertex_max[1], box.vertex_min[2]);
    
    glVertex3d(box.vertex_min[0], box.vertex_max[1], box.vertex_min[2]);
    glVertex3d(box.vertex_min[0], box.vertex_min[1], box.vertex_min[2]);
    
    // 顶面4条边
    glVertex3d(box.vertex_min[0], box.vertex_min[1], box.vertex_max[2]);
    glVertex3d(box.vertex_max[0], box.vertex_min[1], box.vertex_max[2]);
    
    glVertex3d(box.vertex_max[0], box.vertex_min[1], box.vertex_max[2]);
    glVertex3d(box.vertex_max[0], box.vertex_max[1], box.vertex_max[2]);
    
    glVertex3d(box.vertex_max[0], box.vertex_max[1], box.vertex_max[2]);
    glVertex3d(box.vertex_min[0], box.vertex_max[1], box.vertex_max[2]);
    
    glVertex3d(box.vertex_min[0], box.vertex_max[1], box.vertex_max[2]);
    glVertex3d(box.vertex_min[0], box.vertex_min[1], box.vertex_max[2]);
    
    // 4条竖直边
    glVertex3d(box.vertex_min[0], box.vertex_min[1], box.vertex_min[2]);
    glVertex3d(box.vertex_min[0], box.vertex_min[1], box.vertex_max[2]);
    
    glVertex3d(box.vertex_max[0], box.vertex_min[1], box.vertex_min[2]);
    glVertex3d(box.vertex_max[0], box.vertex_min[1], box.vertex_max[2]);
    
    glVertex3d(box.vertex_max[0], box.vertex_max[1], box.vertex_min[2]);
    glVertex3d(box.vertex_max[0], box.vertex_max[1], box.vertex_max[2]);
    
    glVertex3d(box.vertex_min[0], box.vertex_max[1], box.vertex_min[2]);
    glVertex3d(box.vertex_min[0], box.vertex_max[1], box.vertex_max[2]);
    
    glEnd();
    
    glLineWidth(1.0f);
}

void QtPointCloudViewer::renderSphere(const Vector3D& center, double radius, const QVector3D& color)
{
    glColor3f(color.x(), color.y(), color.z());
    glLineWidth(1.5f);
    
    const int segments = 32;
    const double angleStep = 2.0 * M_PI / segments;
    
    // 绘制三个圆环表示球体
    glBegin(GL_LINE_LOOP);
    // XY平面圆环
    for (int i = 0; i < segments; ++i) {
        double angle = i * angleStep;
        double x = center.x() + radius * qCos(angle);
        double y = center.y() + radius * qSin(angle);
        glVertex3d(x, y, center.z());
    }
    glEnd();
    
    glBegin(GL_LINE_LOOP);
    // XZ平面圆环
    for (int i = 0; i < segments; ++i) {
        double angle = i * angleStep;
        double x = center.x() + radius * qCos(angle);
        double z = center.z() + radius * qSin(angle);
        glVertex3d(x, center.y(), z);
    }
    glEnd();
    
    glBegin(GL_LINE_LOOP);
    // YZ平面圆环
    for (int i = 0; i < segments; ++i) {
        double angle = i * angleStep;
        double y = center.y() + radius * qCos(angle);
        double z = center.z() + radius * qSin(angle);
        glVertex3d(center.x(), y, z);
    }
    glEnd();
    
    glLineWidth(1.0f);
}

void QtPointCloudViewer::updateCameraPosition()
{
    // 从球坐标转换为笛卡尔坐标
    float yawRad = qDegreesToRadians(m_cameraYaw);
    float pitchRad = qDegreesToRadians(m_cameraPitch);
    
    m_cameraPos.setX(m_cameraDistance * qCos(pitchRad) * qCos(yawRad));
    m_cameraPos.setY(m_cameraDistance * qSin(pitchRad));
    m_cameraPos.setZ(m_cameraDistance * qCos(pitchRad) * qSin(yawRad));
    
    m_cameraPos += m_cameraTarget;
    
    update();
}

void QtPointCloudViewer::mousePressEvent(QMouseEvent* event)
{
    if (event->button() == Qt::LeftButton) {
        m_mousePressed = true;
        m_lastMousePos = event->pos();
    }
}

void QtPointCloudViewer::mouseMoveEvent(QMouseEvent* event)
{
    if (m_mousePressed) {
        QPoint delta = event->pos() - m_lastMousePos;
        
        // 更新相机角度
        m_cameraYaw += delta.x() * 0.5f;
        m_cameraPitch += delta.y() * 0.5f;
        
        // 限制俯仰角
        m_cameraPitch = qBound(-89.0f, m_cameraPitch, 89.0f);
        
        updateCameraPosition();
        m_lastMousePos = event->pos();
    }
}

void QtPointCloudViewer::wheelEvent(QWheelEvent* event)
{
    // 缩放
    float delta = event->angleDelta().y() / 120.0f;
    m_cameraDistance *= (1.0f - delta * 0.1f);
    m_cameraDistance = qBound(1.0f, m_cameraDistance, 200.0f);
    
    updateCameraPosition();
}

void QtPointCloudViewer::setOriginalCloud(const ColoredPointCloud& cloud)
{
    m_originalCloud = cloud;
    update();
}

void QtPointCloudViewer::setSearchedCloud(const ColoredPointCloud& cloud)
{
    m_searchedCloud = cloud;
    update();
}

void QtPointCloudViewer::setRadiusCloud(const ColoredPointCloud& cloud)
{
    m_radiusCloud = cloud;
    update();
}

void QtPointCloudViewer::setSearchBox(const BoxPointType& box)
{
    m_searchBox = box;
    update();
}

void QtPointCloudViewer::setRadiusSearch(const Vector3D& center, double radius)
{
    m_radiusCenter = center;
    m_searchRadius = radius;
    update();
}

void QtPointCloudViewer::setPointSize(float size)
{
    m_pointSize = size;
    update();
}

void QtPointCloudViewer::resetCamera()
{
    m_cameraDistance = 50.0f;
    m_cameraYaw = 0.0f;
    m_cameraPitch = 0.0f;
    m_cameraTarget = QVector3D(0.0f, 0.0f, 0.0f);
    updateCameraPosition();
}

// ==================== QtIkdTreeDemo 实现 ====================

QtIkdTreeDemo::QtIkdTreeDemo(
    QWidget *parent)
    : QWidget(parent)
    , m_kdTree(new KD_TREE<ikdTree_PointType<double>>(0.5, 0.6, 0.2))
    , m_updateTimer(new QTimer(this))
{
    resize(1400, 900);
    setWindowTitle(QString::fromUtf8("ikd-Tree Qt Demo - 高性能空间索引可视化演示"));
    
    initializeUI();
    connectSignals();
    
    // 启动更新定时器
    m_updateTimer->start(50); // 20 FPS
    
    addLog(QString::fromUtf8("🚀 ikd-Tree Qt Demo 启动成功！"));
    addLog(QString::fromUtf8("📝 使用鼠标左键拖拽旋转视角，滚轮缩放"));
}

void QtIkdTreeDemo::initializeUI()
{
    // 创建主分割器
    m_mainSplitter = new QSplitter(Qt::Horizontal, this);
    
    // 创建3D可视化器
    m_viewer = new QtPointCloudViewer(this);
    m_viewer->setMinimumSize(800, 600);
    
    // 创建控制面板
    m_controlPanel = new QWidget(this);
    m_controlPanel->setMaximumWidth(350);
    m_controlPanel->setMinimumWidth(300);
    
    // 数据控制组
    m_dataGroup = new QGroupBox(QString::fromUtf8("数据生成与构建"), m_controlPanel);
    auto* dataLayout = new QVBoxLayout(m_dataGroup);
    
    auto* pointCountLayout = new QHBoxLayout();
    pointCountLayout->addWidget(new QLabel(QString::fromUtf8("点数量:")));
    m_pointCountSpin = new QSpinBox();
    m_pointCountSpin->setRange(100, 100000);
    m_pointCountSpin->setValue(5000);
    m_pointCountSpin->setSuffix(QString::fromUtf8(" 个"));
    pointCountLayout->addWidget(m_pointCountSpin);
    dataLayout->addLayout(pointCountLayout);
    
    m_generateBtn = new QPushButton(QString::fromUtf8("🎲 生成随机点云"));
    m_buildTreeBtn = new QPushButton(QString::fromUtf8("🌲 构建ikd-Tree"));
    m_buildTreeBtn->setEnabled(false);
    
    dataLayout->addWidget(m_generateBtn);
    dataLayout->addWidget(m_buildTreeBtn);
    
    m_progressBar = new QProgressBar();
    m_progressBar->setVisible(false);
    dataLayout->addWidget(m_progressBar);
    
    // 搜索控制组
    m_searchGroup = new QGroupBox(QString::fromUtf8("空间搜索参数"), m_controlPanel);
    auto* searchLayout = new QVBoxLayout(m_searchGroup);
    
    // 包围盒搜索
    auto* boxLabel = new QLabel(QString::fromUtf8("📦 包围盒搜索"));
    boxLabel->setStyleSheet("font-weight: bold; color: #2980b9;");
    searchLayout->addWidget(boxLabel);
    
    auto* boxCenterLayout = new QHBoxLayout();
    boxCenterLayout->addWidget(new QLabel(QString::fromUtf8("中心:")));
    m_boxCenterX = new QDoubleSpinBox();
    m_boxCenterY = new QDoubleSpinBox();
    m_boxCenterZ = new QDoubleSpinBox();
    for (auto* spin : {m_boxCenterX, m_boxCenterY, m_boxCenterZ}) {
        spin->setRange(-50.0, 50.0);
        spin->setValue(0.0);
        spin->setDecimals(1);
        boxCenterLayout->addWidget(spin);
    }
    searchLayout->addLayout(boxCenterLayout);
    
    auto* boxSizeLayout = new QHBoxLayout();
    boxSizeLayout->addWidget(new QLabel(QString::fromUtf8("尺寸:")));
    m_boxSizeX = new QDoubleSpinBox();
    m_boxSizeY = new QDoubleSpinBox();
    m_boxSizeZ = new QDoubleSpinBox();
    for (auto* spin : {m_boxSizeX, m_boxSizeY, m_boxSizeZ}) {
        spin->setRange(0.1, 50.0);
        spin->setValue(5.0);
        spin->setDecimals(1);
        boxSizeLayout->addWidget(spin);
    }
    searchLayout->addLayout(boxSizeLayout);
    
    m_boxSearchBtn = new QPushButton(QString::fromUtf8("🔍 执行包围盒搜索"));
    m_boxSearchBtn->setEnabled(false);
    searchLayout->addWidget(m_boxSearchBtn);
    
    m_boxDeleteBtn = new QPushButton(QString::fromUtf8("🗑️ 删除包围盒搜索结果"));
    m_boxDeleteBtn->setEnabled(false);
    m_boxDeleteBtn->setStyleSheet("QPushButton { color: #c0392b; }");
    searchLayout->addWidget(m_boxDeleteBtn);
    
    // 半径搜索
    auto* radiusLabel = new QLabel(QString::fromUtf8("🎯 半径搜索"));
    radiusLabel->setStyleSheet("font-weight: bold; color: #27ae60;");
    searchLayout->addWidget(radiusLabel);
    
    auto* radiusCenterLayout = new QHBoxLayout();
    radiusCenterLayout->addWidget(new QLabel(QString::fromUtf8("中心:")));
    m_radiusCenterX = new QDoubleSpinBox();
    m_radiusCenterY = new QDoubleSpinBox();
    m_radiusCenterZ = new QDoubleSpinBox();
    for (auto* spin : {m_radiusCenterX, m_radiusCenterY, m_radiusCenterZ}) {
        spin->setRange(-50.0, 50.0);
        spin->setValue(0.0);
        spin->setDecimals(1);
        radiusCenterLayout->addWidget(spin);
    }
    searchLayout->addLayout(radiusCenterLayout);
    
    auto* radiusSizeLayout = new QHBoxLayout();
    radiusSizeLayout->addWidget(new QLabel(QString::fromUtf8("半径:")));
    m_radiusSize = new QDoubleSpinBox();
    m_radiusSize->setRange(0.1, 50.0);
    m_radiusSize->setValue(8.0);
    m_radiusSize->setDecimals(1);
    radiusSizeLayout->addWidget(m_radiusSize);
    searchLayout->addLayout(radiusSizeLayout);
    
    m_radiusSearchBtn = new QPushButton(QString::fromUtf8("🎯 执行半径搜索"));
    m_radiusSearchBtn->setEnabled(false);
    searchLayout->addWidget(m_radiusSearchBtn);
    
    m_radiusDeleteBtn = new QPushButton(QString::fromUtf8("🗑️ 删除半径搜索结果"));
    m_radiusDeleteBtn->setEnabled(false);
    m_radiusDeleteBtn->setStyleSheet("QPushButton { color: #c0392b; }");
    searchLayout->addWidget(m_radiusDeleteBtn);
    
    // 可视化控制组
    m_visualGroup = new QGroupBox(QString::fromUtf8("可视化设置"), m_controlPanel);
    auto* visualLayout = new QVBoxLayout(m_visualGroup);
    
    auto* pointSizeLayout = new QHBoxLayout();
    pointSizeLayout->addWidget(new QLabel(QString::fromUtf8("点大小:")));
    m_pointSizeSlider = new QSlider(Qt::Horizontal);
    m_pointSizeSlider->setRange(1, 10);
    m_pointSizeSlider->setValue(2);
    pointSizeLayout->addWidget(m_pointSizeSlider);
    visualLayout->addLayout(pointSizeLayout);
    
    m_showOriginalBtn = new QPushButton(QString::fromUtf8("原始点云"));
    m_showSearchedBtn = new QPushButton(QString::fromUtf8("包围盒结果"));
    m_showRadiusBtn = new QPushButton(QString::fromUtf8("半径结果"));
    m_showBoxBtn = new QPushButton(QString::fromUtf8("搜索包围盒"));
    m_showSphereBtn = new QPushButton(QString::fromUtf8("搜索球体"));
    
    for (auto* btn : {m_showOriginalBtn, m_showSearchedBtn, m_showRadiusBtn, m_showBoxBtn, m_showSphereBtn}) {
        btn->setCheckable(true);
        btn->setChecked(true);
        visualLayout->addWidget(btn);
    }
    
    m_resetCameraBtn = new QPushButton(QString::fromUtf8("🔄 重置视角"));
    visualLayout->addWidget(m_resetCameraBtn);
    
    // 日志文本
    m_logText = new QTextEdit(m_controlPanel);
    m_logText->setMaximumHeight(150);
    m_logText->setReadOnly(true);
    m_logText->setStyleSheet("font-family: 'Consolas', monospace; font-size: 9pt;");
    
    // 布局控制面板
    auto* controlLayout = new QVBoxLayout(m_controlPanel);
    controlLayout->addWidget(m_dataGroup);
    controlLayout->addWidget(m_searchGroup);
    controlLayout->addWidget(m_visualGroup);
    controlLayout->addWidget(new QLabel(QString::fromUtf8("📊 操作日志")));
    controlLayout->addWidget(m_logText);
    controlLayout->addStretch();
    
    // 布局主界面
    m_mainSplitter->addWidget(m_viewer);
    m_mainSplitter->addWidget(m_controlPanel);
    m_mainSplitter->setStretchFactor(0, 1);
    m_mainSplitter->setStretchFactor(1, 0);
    
    auto* mainLayout = new QHBoxLayout(this);
    mainLayout->addWidget(m_mainSplitter);
}

void QtIkdTreeDemo::connectSignals()
{
    connect(m_generateBtn, &QPushButton::clicked, this, &QtIkdTreeDemo::generateRandomPoints);
    connect(m_buildTreeBtn, &QPushButton::clicked, this, &QtIkdTreeDemo::buildTree);
    connect(m_boxSearchBtn, &QPushButton::clicked, this, &QtIkdTreeDemo::performBoxSearch);
    connect(m_radiusSearchBtn, &QPushButton::clicked, this, &QtIkdTreeDemo::performRadiusSearch);
    connect(m_boxDeleteBtn, &QPushButton::clicked, this, &QtIkdTreeDemo::deleteBoxSearchResults);
    connect(m_radiusDeleteBtn, &QPushButton::clicked, this, &QtIkdTreeDemo::deleteRadiusSearchResults);
    
    connect(m_pointSizeSlider, &QSlider::valueChanged, this, [this](int value) {
        m_viewer->setPointSize(float(value));
    });
    
    connect(m_showOriginalBtn, &QPushButton::toggled, m_viewer, &QtPointCloudViewer::setShowOriginal);
    connect(m_showSearchedBtn, &QPushButton::toggled, m_viewer, &QtPointCloudViewer::setShowSearched);
    connect(m_showRadiusBtn, &QPushButton::toggled, m_viewer, &QtPointCloudViewer::setShowRadius);
    connect(m_showBoxBtn, &QPushButton::toggled, m_viewer, &QtPointCloudViewer::setShowSearchBox);
    connect(m_showSphereBtn, &QPushButton::toggled, m_viewer, &QtPointCloudViewer::setShowRadiusSphere);
    connect(m_resetCameraBtn, &QPushButton::clicked, m_viewer, &QtPointCloudViewer::resetCamera);
    
    // 实时更新搜索参数
    for (auto* spin : {m_boxCenterX, m_boxCenterY, m_boxCenterZ, m_boxSizeX, m_boxSizeY, m_boxSizeZ}) {
        connect(spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &QtIkdTreeDemo::updateVisualization);
    }
    
    for (auto* spin : {m_radiusCenterX, m_radiusCenterY, m_radiusCenterZ, m_radiusSize}) {
        connect(spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &QtIkdTreeDemo::updateVisualization);
    }
}

void QtIkdTreeDemo::generateRandomPoints()
{
    const int pointCount = m_pointCountSpin->value();
    m_originalPoints.clear();
    m_originalPoints.reserve(pointCount);
    
    m_progressBar->setVisible(true);
    m_progressBar->setRange(0, pointCount);
    
    QElapsedTimer timer;
    timer.start();
    
    auto* rng = QRandomGenerator::global();
    
    addLog(QString(QString::fromUtf8("🎲 开始生成 %1 个随机点...")).arg(pointCount));
    
    for (int i = 0; i < pointCount; ++i) {
        ikdTree_PointType<double> point;
        point.x = rng->bounded(qint64(-25.0), qint64(25.0));
        point.y = rng->bounded(qint64(-25.0), qint64(25.0));
        point.z = rng->bounded(qint64(-25.0), qint64(25.0));
        m_originalPoints.append(point);
        
        if (i % 1000 == 0) {
            m_progressBar->setValue(i);
            QApplication::processEvents();
        }
    }
    
    m_progressBar->setValue(pointCount);
    m_progressBar->setVisible(false);
    
    // 更新可视化
    auto coloredCloud = convertToColoredCloud(m_originalPoints, QVector3D(0.7f, 0.7f, 0.7f));
    m_viewer->setOriginalCloud(coloredCloud);
    
    m_buildTreeBtn->setEnabled(true);
    
    addLog(QString(QString::fromUtf8("✅ 点云生成完成，耗时 %1 ms")).arg(timer.elapsed()));
}

void QtIkdTreeDemo::buildTree()
{
    if (m_originalPoints.isEmpty()) {
        QMessageBox::warning(this, QString::fromUtf8("警告"), QString::fromUtf8("请先生成点云数据！"));
        return;
    }
    
    QElapsedTimer timer;
    timer.start();
    
    addLog(QString::fromUtf8("🌲 开始构建ikd-Tree..."));
    
    // 创建新的ikd-Tree实例
    m_kdTree.reset(new KD_TREE<ikdTree_PointType<double>>(0.5, 0.6, 0.2));

    // 构建树
    m_kdTree->build(m_originalPoints);
    
    const auto elapsed = timer.elapsed();
    
    addLog(QString(QString::fromUtf8("✅ ikd-Tree构建完成！")));
    addLog(QString(QString::fromUtf8("   • 耗时: %1 ms")).arg(elapsed));
    addLog(QString(QString::fromUtf8("   • 有效节点: %1")).arg(m_kdTree->validnum()));
    addLog(QString(QString::fromUtf8("   • 树大小: %1")).arg(m_kdTree->size()));
    
    // 启用搜索功能
    m_boxSearchBtn->setEnabled(true);
    m_radiusSearchBtn->setEnabled(true);
    
    updateVisualization();
}

void QtIkdTreeDemo::performBoxSearch()
{
    if (!m_kdTree || m_kdTree->size() == 0) {
        QMessageBox::warning(this, QString::fromUtf8("警告"), QString::fromUtf8("请先构建ikd-Tree！"));
        return;
    }
    
    // 设置搜索包围盒
    BoxPointType box;
    Vector3D center(m_boxCenterX->value(), m_boxCenterY->value(), m_boxCenterZ->value());
    Vector3D size(m_boxSizeX->value(), m_boxSizeY->value(), m_boxSizeZ->value());
    
    box.vertex_min[0] = center.x() - size.x();
    box.vertex_max[0] = center.x() + size.x();
    box.vertex_min[1] = center.y() - size.y();
    box.vertex_max[1] = center.y() + size.y();
    box.vertex_min[2] = center.z() - size.z();
    box.vertex_max[2] = center.z() + size.z();
    
    QElapsedTimer timer;
    timer.start();
    
    // 执行搜索
    QVector<ikdTree_PointType<double>> searchResults;
    m_kdTree->boxSearch(box, searchResults);
    
    const auto elapsed = timer.elapsed();
    
    // 保存搜索结果用于删除
    m_lastBoxSearchResults = searchResults;
    
    // 更新可视化
    auto coloredResults = convertToColoredCloud(searchResults, QVector3D(1.0f, 0.0f, 0.0f));
    m_viewer->setSearchedCloud(coloredResults);
    m_viewer->setSearchBox(box);
    
    addLog(QString(QString::fromUtf8("🔍 包围盒搜索完成：")));
    addLog(QString(QString::fromUtf8("   • 找到 %1 个点")).arg(searchResults.size()));
    addLog(QString(QString::fromUtf8("   • 耗时 %1 ms")).arg(elapsed));
    if (searchResults.size() > 0) {
        addLog(QString::fromUtf8("💡 提示：红色较大点为搜索结果，可关闭'原始点云'显示更清楚"));
        m_boxDeleteBtn->setEnabled(true);  // 启用删除按钮
    } else {
        m_boxDeleteBtn->setEnabled(false); // 没有结果时禁用删除按钮
    }
}

void QtIkdTreeDemo::performRadiusSearch()
{
    if (!m_kdTree || m_kdTree->size() == 0) {
        QMessageBox::warning(this, QString::fromUtf8("警告"), QString::fromUtf8("请先构建ikd-Tree！"));
        return;
    }
    
    // 设置搜索参数
    ikdTree_PointType<double> center;
    center.x = m_radiusCenterX->value();
    center.y = m_radiusCenterY->value();
    center.z = m_radiusCenterZ->value();
    double radius = m_radiusSize->value();
    
    QElapsedTimer timer;
    timer.start();
    
    // 执行搜索
    QVector<ikdTree_PointType<double>> searchResults;
    m_kdTree->radiusSearch(center, static_cast<float>(radius), searchResults);
    
    const auto elapsed = timer.elapsed();
    
    // 保存搜索结果用于删除
    m_lastRadiusSearchResults = searchResults;
    
    // 更新可视化
    auto coloredResults = convertToColoredCloud(searchResults, QVector3D(0.0f, 1.0f, 0.0f));
    m_viewer->setRadiusCloud(coloredResults);
    m_viewer->setRadiusSearch(Vector3D(center.x, center.y, center.z), radius);
    
    addLog(QString(QString::fromUtf8("🎯 半径搜索完成：")));
    addLog(QString(QString::fromUtf8("   • 找到 %1 个点")).arg(searchResults.size()));
    addLog(QString(QString::fromUtf8("   • 耗时 %1 ms")).arg(elapsed));
    if (searchResults.size() > 0) {
        addLog(QString::fromUtf8("💡 提示：绿色较大点为搜索结果，可关闭'原始点云'显示更清楚"));
        m_radiusDeleteBtn->setEnabled(true);  // 启用删除按钮
    } else {
        m_radiusDeleteBtn->setEnabled(false); // 没有结果时禁用删除按钮
    }
}

void QtIkdTreeDemo::updateVisualization()
{
    // 更新搜索包围盒显示
    BoxPointType box;
    Vector3D center(m_boxCenterX->value(), m_boxCenterY->value(), m_boxCenterZ->value());
    Vector3D size(m_boxSizeX->value(), m_boxSizeY->value(), m_boxSizeZ->value());
    
    box.vertex_min[0] = center.x() - size.x();
    box.vertex_max[0] = center.x() + size.x();
    box.vertex_min[1] = center.y() - size.y();
    box.vertex_max[1] = center.y() + size.y();
    box.vertex_min[2] = center.z() - size.z();
    box.vertex_max[2] = center.z() + size.z();
    
    m_viewer->setSearchBox(box);
    
    // 更新半径搜索显示
    Vector3D radiusCenter(m_radiusCenterX->value(), m_radiusCenterY->value(), m_radiusCenterZ->value());
    m_viewer->setRadiusSearch(radiusCenter, m_radiusSize->value());
}

void QtIkdTreeDemo::addLog(const QString& message)
{
    QTime currentTime = QTime::currentTime();
    QString timeStr = currentTime.toString("hh:mm:ss.zzz");
    m_logText->append(QString("[%1] %2").arg(timeStr, message));
    
    // 自动滚动到底部
    QTextCursor cursor = m_logText->textCursor();
    cursor.movePosition(QTextCursor::End);
    m_logText->setTextCursor(cursor);
}

QtPointCloudViewer::ColoredPointCloud QtIkdTreeDemo::convertToColoredCloud(
    const QVector<ikdTree_PointType<double>> &points, const QVector3D &color)
{
    QtPointCloudViewer::ColoredPointCloud result;
    result.reserve(points.size());
    
    for (const auto& point : points) {
        Vector3D pos(point.x, point.y, point.z);
        result.append(QtPointCloudViewer::ColoredPoint(pos, color));
    }
    
    return result;
}

void QtIkdTreeDemo::deleteBoxSearchResults()
{
    if (m_lastBoxSearchResults.isEmpty()) {
        QMessageBox::information(this, QString::fromUtf8("提示"), QString::fromUtf8("没有包围盒搜索结果可删除！"));
        return;
    }
    
    QElapsedTimer timer;
    timer.start();
    
    // 从ikd-Tree中批量删除搜索到的点
    QVector<ikdTree_PointType<double>> pointsToDelete = m_lastBoxSearchResults;
    m_kdTree->deletePoints(pointsToDelete);
    int deletedCount = pointsToDelete.size();
    
    const auto elapsed = timer.elapsed();
    
    // 更新原始点云数据（从中移除已删除的点）
    for (const auto& pointToDelete : m_lastBoxSearchResults) {
        for (int i = m_originalPoints.size() - 1; i >= 0; --i) {
            const auto& originalPoint = m_originalPoints[i];
            if (qAbs(originalPoint.x - pointToDelete.x) < 1e-6 &&
                qAbs(originalPoint.y - pointToDelete.y) < 1e-6 &&
                qAbs(originalPoint.z - pointToDelete.z) < 1e-6) {
                m_originalPoints.removeAt(i);
                break;
            }
        }
    }
    
    // 更新可视化
    auto updatedCloud = convertToColoredCloud(m_originalPoints, QVector3D(0.5f, 0.5f, 0.5f));
    m_viewer->setOriginalCloud(updatedCloud);
    
    // 清空搜索结果显示
    m_viewer->setSearchedCloud(QtPointCloudViewer::ColoredPointCloud());
    
    // 清空保存的搜索结果
    m_lastBoxSearchResults.clear();
    
    // 禁用删除按钮
    m_boxDeleteBtn->setEnabled(false);
    
    addLog(QString(QString::fromUtf8("🗑️ 包围盒搜索结果删除完成：")));
    addLog(QString(QString::fromUtf8("   • 删除了 %1 个点")).arg(deletedCount));
    addLog(QString(QString::fromUtf8("   • 耗时 %1 ms")).arg(elapsed));
    addLog(QString(QString::fromUtf8("   • 剩余点数: %1")).arg(m_kdTree->size()));
}

void QtIkdTreeDemo::deleteRadiusSearchResults()
{
    if (m_lastRadiusSearchResults.isEmpty()) {
        QMessageBox::information(this, QString::fromUtf8("提示"), QString::fromUtf8("没有半径搜索结果可删除！"));
        return;
    }
    
    QElapsedTimer timer;
    timer.start();
    
    // 从ikd-Tree中批量删除搜索到的点
    QVector<ikdTree_PointType<double>> pointsToDelete = m_lastRadiusSearchResults;
    m_kdTree->deletePoints(pointsToDelete);
    int deletedCount = pointsToDelete.size();
    
    const auto elapsed = timer.elapsed();
    
    // 更新原始点云数据（从中移除已删除的点）
    for (const auto& pointToDelete : m_lastRadiusSearchResults) {
        for (int i = m_originalPoints.size() - 1; i >= 0; --i) {
            const auto& originalPoint = m_originalPoints[i];
            if (qAbs(originalPoint.x - pointToDelete.x) < 1e-6 &&
                qAbs(originalPoint.y - pointToDelete.y) < 1e-6 &&
                qAbs(originalPoint.z - pointToDelete.z) < 1e-6) {
                m_originalPoints.removeAt(i);
                break;
            }
        }
    }
    
    // 更新可视化
    auto updatedCloud = convertToColoredCloud(m_originalPoints, QVector3D(0.5f, 0.5f, 0.5f));
    m_viewer->setOriginalCloud(updatedCloud);
    
    // 清空搜索结果显示
    m_viewer->setRadiusCloud(QtPointCloudViewer::ColoredPointCloud());
    
    // 清空保存的搜索结果
    m_lastRadiusSearchResults.clear();
    
    // 禁用删除按钮
    m_radiusDeleteBtn->setEnabled(false);
    
    addLog(QString(QString::fromUtf8("🗑️ 半径搜索结果删除完成：")));
    addLog(QString(QString::fromUtf8("   • 删除了 %1 个点")).arg(deletedCount));
    addLog(QString(QString::fromUtf8("   • 耗时 %1 ms")).arg(elapsed));
    addLog(QString(QString::fromUtf8("   • 剩余点数: %1")).arg(m_kdTree->size()));
}
