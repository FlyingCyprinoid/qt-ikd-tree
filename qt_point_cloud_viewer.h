#pragma once

#include <QWidget>
#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QMatrix4x4>
#include <QVector3D>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QTimer>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QLabel>
#include <QSlider>
#include <QGroupBox>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QProgressBar>
#include <QTextEdit>
#include <QSplitter>
#include <QVector>
#include <QtMath>
#include "ikd_Tree_qt.hpp"

/**
 * @brief Qt风格的3D点云可视化器 - 使用OpenGL渲染点云数据
 */
class QtPointCloudViewer : public QOpenGLWidget, protected QOpenGLFunctions
{
    Q_OBJECT

public:
    /**
     * @brief 点云数据结构 - 包含位置和颜色信息
     */
    struct ColoredPoint {
        Vector3D position;          ///< 点位置
        QVector3D color;           ///< 点颜色 (RGB 0-1)
        
        ColoredPoint(const Vector3D& pos = Vector3D(), const QVector3D& col = QVector3D(-1.0f, -1.0f, -1.0f))
            : position(pos), color(col) {}
    };
    
    using ColoredPointCloud = QVector<ColoredPoint>;

private:
    // 渲染相关
    QMatrix4x4 m_projection;        ///< 投影矩阵
    QMatrix4x4 m_view;              ///< 视图矩阵
    QMatrix4x4 m_model;             ///< 模型矩阵
    
    // 相机控制
    QVector3D m_cameraPos;          ///< 相机位置
    QVector3D m_cameraTarget;       ///< 相机目标点
    QVector3D m_cameraUp;           ///< 相机上方向
    float m_cameraDistance;         ///< 相机到目标的距离
    float m_cameraYaw;              ///< 相机水平角度
    float m_cameraPitch;            ///< 相机俯仰角度
    
    // 鼠标控制
    QPoint m_lastMousePos;          ///< 上次鼠标位置
    bool m_mousePressed;            ///< 鼠标按下状态
    
    // 点云数据
    ColoredPointCloud m_originalCloud;     ///< 原始点云
    ColoredPointCloud m_searchedCloud;     ///< 搜索结果点云
    ColoredPointCloud m_radiusCloud;       ///< 半径搜索结果
    
    // 搜索参数
    BoxPointType m_searchBox;              ///< 搜索包围盒
    Vector3D m_radiusCenter;               ///< 半径搜索中心
    double m_searchRadius;                 ///< 搜索半径
    
    // 渲染设置
    float m_pointSize;              ///< 点大小
    bool m_showOriginal;            ///< 显示原始点云
    bool m_showSearched;            ///< 显示搜索结果
    bool m_showRadius;              ///< 显示半径搜索结果
    bool m_showSearchBox;           ///< 显示搜索包围盒
    bool m_showRadiusSphere;        ///< 显示搜索球体

protected:
    void initializeGL() override;
    void resizeGL(int width, int height) override;
    void paintGL() override;
    
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;

private:
    /**
     * @brief 渲染点云 - 绘制指定颜色的点云数据
     * @param cloud 点云数据
     * @param defaultColor 默认颜色
     */
    void renderPointCloud(const ColoredPointCloud& cloud, const QVector3D& defaultColor = QVector3D(1.0f, 1.0f, 1.0f));
    
    /**
     * @brief 渲染包围盒 - 绘制搜索区域的线框
     * @param box 包围盒
     * @param color 线框颜色
     */
    void renderBoundingBox(const BoxPointType& box, const QVector3D& color);
    
    /**
     * @brief 渲染球体 - 绘制半径搜索的球体线框
     * @param center 球心
     * @param radius 半径
     * @param color 颜色
     */
    void renderSphere(const Vector3D& center, double radius, const QVector3D& color);
    
    /**
     * @brief 更新相机位置 - 根据球坐标计算相机位置
     */
    void updateCameraPosition();

public:
    explicit QtPointCloudViewer(QWidget* parent = nullptr);
    ~QtPointCloudViewer() override = default;

public slots:
    /**
     * @brief 设置原始点云数据
     * @param cloud 点云数据
     */
    void setOriginalCloud(const ColoredPointCloud& cloud);
    
    /**
     * @brief 设置搜索结果点云
     * @param cloud 搜索结果
     */
    void setSearchedCloud(const ColoredPointCloud& cloud);
    
    /**
     * @brief 设置半径搜索结果
     * @param cloud 半径搜索结果
     */
    void setRadiusCloud(const ColoredPointCloud& cloud);
    
    /**
     * @brief 设置搜索包围盒
     * @param box 包围盒
     */
    void setSearchBox(const BoxPointType& box);
    
    /**
     * @brief 设置半径搜索参数
     * @param center 搜索中心
     * @param radius 搜索半径
     */
    void setRadiusSearch(const Vector3D& center, double radius);
    
    /**
     * @brief 设置点大小
     * @param size 点大小
     */
    void setPointSize(float size);
    
    /**
     * @brief 设置显示选项
     */
    void setShowOriginal(bool show) { m_showOriginal = show; update(); }
    void setShowSearched(bool show) { m_showSearched = show; update(); }
    void setShowRadius(bool show) { m_showRadius = show; update(); }
    void setShowSearchBox(bool show) { m_showSearchBox = show; update(); }
    void setShowRadiusSphere(bool show) { m_showRadiusSphere = show; update(); }
    
    /**
     * @brief 重置相机视角
     */
    void resetCamera();
};

/**
 * @brief 主控制面板 - 包含参数设置和可视化控制
 */
class QtIkdTreeDemo : public QWidget
{
    Q_OBJECT

private:
    // UI组件
    QtPointCloudViewer* m_viewer;           ///< 3D可视化器
    QSplitter* m_mainSplitter;              ///< 主分割器
    QWidget* m_controlPanel;                ///< 控制面板
    
    // 控制组件
    QGroupBox* m_dataGroup;                 ///< 数据控制组
    QPushButton* m_generateBtn;             ///< 生成数据按钮
    QPushButton* m_buildTreeBtn;            ///< 构建树按钮
    QSpinBox* m_pointCountSpin;             ///< 点数量选择
    QProgressBar* m_progressBar;            ///< 进度条
    
    QGroupBox* m_searchGroup;               ///< 搜索控制组
    QDoubleSpinBox* m_boxCenterX;           ///< 包围盒中心X
    QDoubleSpinBox* m_boxCenterY;           ///< 包围盒中心Y
    QDoubleSpinBox* m_boxCenterZ;           ///< 包围盒中心Z
    QDoubleSpinBox* m_boxSizeX;             ///< 包围盒大小X
    QDoubleSpinBox* m_boxSizeY;             ///< 包围盒大小Y
    QDoubleSpinBox* m_boxSizeZ;             ///< 包围盒大小Z
    QPushButton* m_boxSearchBtn;            ///< 包围盒搜索按钮
    QPushButton* m_boxDeleteBtn;            ///< 删除包围盒搜索结果按钮
    
    QDoubleSpinBox* m_radiusCenterX;        ///< 半径搜索中心X
    QDoubleSpinBox* m_radiusCenterY;        ///< 半径搜索中心Y
    QDoubleSpinBox* m_radiusCenterZ;        ///< 半径搜索中心Z
    QDoubleSpinBox* m_radiusSize;           ///< 搜索半径
    QPushButton* m_radiusSearchBtn;         ///< 半径搜索按钮
    QPushButton* m_radiusDeleteBtn;         ///< 删除半径搜索结果按钮
    
    QGroupBox* m_visualGroup;               ///< 可视化控制组
    QSlider* m_pointSizeSlider;             ///< 点大小滑块
    QPushButton* m_showOriginalBtn;         ///< 显示原始点云
    QPushButton* m_showSearchedBtn;         ///< 显示搜索结果
    QPushButton* m_showRadiusBtn;           ///< 显示半径搜索
    QPushButton* m_showBoxBtn;              ///< 显示搜索包围盒
    QPushButton* m_showSphereBtn;           ///< 显示搜索球体
    QPushButton* m_resetCameraBtn;          ///< 重置相机
    
    QTextEdit* m_logText;                   ///< 日志文本
    
    // ikd-Tree相关
    QScopedPointer<KD_TREE<ikdTree_PointType<double>>> m_kdTree;  ///< ikd-Tree实例
    QVector<ikdTree_PointType<double>> m_originalPoints;          ///< 原始点数据
    QVector<ikdTree_PointType<double>> m_lastBoxSearchResults;    ///< 最后一次包围盒搜索结果
    QVector<ikdTree_PointType<double>> m_lastRadiusSearchResults; ///< 最后一次半径搜索结果

    QTimer* m_updateTimer;                  ///< 更新定时器

private slots:
    /**
     * @brief 生成随机点云数据
     */
    void generateRandomPoints();
    
    /**
     * @brief 构建ikd-Tree
     */
    void buildTree();
    
    /**
     * @brief 执行包围盒搜索
     */
    void performBoxSearch();
    
    /**
     * @brief 执行半径搜索
     */
    void performRadiusSearch();
    
    /**
     * @brief 删除包围盒搜索到的点
     */
    void deleteBoxSearchResults();
    
    /**
     * @brief 删除半径搜索到的点
     */
    void deleteRadiusSearchResults();
    
    /**
     * @brief 更新可视化设置
     */
    void updateVisualization();
    
    /**
     * @brief 添加日志信息
     * @param message 日志消息
     */
    void addLog(const QString& message);

public:
    explicit QtIkdTreeDemo(QWidget* parent = nullptr);
    ~QtIkdTreeDemo() override = default;
    
private:
    /**
     * @brief 初始化UI界面
     */
    void initializeUI();
    
    /**
     * @brief 连接信号槽
     */
    void connectSignals();
    
    /**
     * @brief 转换点云数据格式
     * @param points ikd-Tree点数据
     * @param color 点颜色
     * @return Qt可视化点云数据
     */
    QtPointCloudViewer::ColoredPointCloud convertToColoredCloud(const QVector<ikdTree_PointType<double>> &points,
                                                                const QVector3D &color = QVector3D(1.0f, 1.0f, 1.0f));
};

// MOC文件将自动包含
