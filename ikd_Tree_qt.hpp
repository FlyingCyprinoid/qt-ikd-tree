#pragma once
#include <QObject>
#include <QVector>
#include <QQueue>
#include <QThread>
#include <QMutex>
#include <QMutexLocker>
#include <QSharedPointer>
#include <QScopedPointer>
#include <QPointer>
// 移除QVector3D依赖，使用自定义3D向量类
#include <QList>
#include <QtMath>
#include <QDebug>
#include <QAtomicInt>
#include <QRandomGenerator>
#include <QElapsedTimer>
#include <limits>
#include <cmath>
#include <algorithm>
#include <cstring>

#define EPSS 1e-6
#define MULTI_THREAD_REBUILD_POINT_NUM 1500
#define Minimal_Unbalanced_Tree_Size 10
#define DOWNSAMPLE_SWITCH true
#define ForceRebuildPercentage 0.2
#define Q_LEN 1000000

/*
Description: ikd-Tree: an incremental k-d tree for robotic applications - Qt版本头文件实现
Author: Yixi Cai (原作者) + Qt移植版本
email: yixicai@connect.hku.hk
Note: 这是ikd-Tree的纯Qt header-only实现版本，解决模板前向声明警告问题
*/

/**
 * @brief 自定义3D向量类 - 基于double类型，不依赖GUI模块
 * 
 * 替代QVector3D，避免对Qt GUI模块的依赖
 */
class Vector3D
{
public:
    double m_x, m_y, m_z;
    
    /**
     * @brief 默认构造函数 - 初始化为零向量
     */
    Vector3D() : m_x(0.0), m_y(0.0), m_z(0.0) {}
    
    /**
     * @brief 构造函数 - 使用三个分量初始化
     * @param x X分量
     * @param y Y分量
     * @param z Z分量
     */
    Vector3D(double x, double y, double z) : m_x(x), m_y(y), m_z(z) {}
    
    /**
     * @brief 拷贝构造函数
     */
    Vector3D(const Vector3D& other) : m_x(other.m_x), m_y(other.m_y), m_z(other.m_z) {}
    
    /**
     * @brief 赋值运算符
     */
    Vector3D& operator=(const Vector3D& other)
    {
        if (this != &other) {
            m_x = other.m_x;
            m_y = other.m_y;
            m_z = other.m_z;
        }
        return *this;
    }
    
    /**
     * @brief 获取X分量
     */
    double x() const { return m_x; }
    
    /**
     * @brief 获取Y分量
     */
    double y() const { return m_y; }
    
    /**
     * @brief 获取Z分量
     */
    double z() const { return m_z; }
    
    /**
     * @brief 设置X分量
     */
    void setX(double x) { m_x = x; }
    
    /**
     * @brief 设置Y分量
     */
    void setY(double y) { m_y = y; }
    
    /**
     * @brief 设置Z分量
     */
    void setZ(double z) { m_z = z; }
    
    /**
     * @brief 向量加法
     */
    Vector3D operator+(const Vector3D& other) const
    {
        return Vector3D(m_x + other.m_x, m_y + other.m_y, m_z + other.m_z);
    }
    
    /**
     * @brief 向量减法
     */
    Vector3D operator-(const Vector3D& other) const
    {
        return Vector3D(m_x - other.m_x, m_y - other.m_y, m_z - other.m_z);
    }
    
    /**
     * @brief 标量乘法
     */
    Vector3D operator*(double scalar) const
    {
        return Vector3D(m_x * scalar, m_y * scalar, m_z * scalar);
    }
    
    /**
     * @brief 计算向量长度
     */
    double length() const
    {
        return qSqrt(m_x * m_x + m_y * m_y + m_z * m_z);
    }
    
    /**
     * @brief 计算向量长度的平方
     */
    double lengthSquared() const
    {
        return m_x * m_x + m_y * m_y + m_z * m_z;
    }
    
    /**
     * @brief 向量归一化
     */
    Vector3D normalized() const
    {
        double len = length();
        if (len > 0.0) {
            return Vector3D(m_x / len, m_y / len, m_z / len);
        }
        return Vector3D();
    }
    
    /**
     * @brief 点积
     */
    double dotProduct(const Vector3D& other) const
    {
        return m_x * other.m_x + m_y * other.m_y + m_z * other.m_z;
    }
    
    /**
     * @brief 叉积
     */
    Vector3D crossProduct(const Vector3D& other) const
    {
        return Vector3D(
            m_y * other.m_z - m_z * other.m_y,
            m_z * other.m_x - m_x * other.m_z,
            m_x * other.m_y - m_y * other.m_x
        );
    }
};

/**
 * @brief 定义ikd-Tree使用的点类型模板结构
 * 
 * 使用Qt风格的模板化3D点数据结构，支持自定义数据类型
 * @tparam DataType 附加数据成员的类型，默认为int
 */
template<typename DataType = int>
struct ikdTree_PointType
{
    double x, y, z;      ///< 3D坐标，使用double类型提高精度
    DataType data;       ///< 用户自定义的附加数据成员
    
    /**
     * @brief 默认构造函数 - 初始化所有成员为默认值
     */
    ikdTree_PointType() : x(0.0), y(0.0), z(0.0), data(DataType{}) 
    {
    }
    
    /**
     * @brief 构造函数 - 初始化3D点坐标
     * @param px X坐标值
     * @param py Y坐标值  
     * @param pz Z坐标值
     * @param d 附加数据值，默认为DataType的默认值
     */
    ikdTree_PointType(double px, double py, double pz, const DataType& d = DataType{})
        : x(px), y(py), z(pz), data(d)
    {
    }
    
    /**
     * @brief 从Vector3D构造点
     * @param vec 3D向量
     * @param d 附加数据值，默认为DataType的默认值
     */
    explicit ikdTree_PointType(const Vector3D& vec, const DataType& d = DataType{})
        : x(vec.x()), y(vec.y()), z(vec.z()), data(d)
    {
    }
    
    /**
     * @brief 转换为Vector3D
     * @return 对应的3D向量
     */
    Vector3D toVector3D() const 
    {
        return Vector3D(x, y, z);
    }
    
    /**
     * @brief 从float坐标构造点 - 兼容性函数
     * @param px X坐标值(float)
     * @param py Y坐标值(float)
     * @param pz Z坐标值(float)
     * @param d 附加数据值，默认为DataType的默认值
     */
    ikdTree_PointType(float px, float py, float pz, const DataType& d = DataType{})
        : x(static_cast<double>(px)), y(static_cast<double>(py)), z(static_cast<double>(pz)), data(d)
    {
    }
};

// 为了兼容性，提供默认的点类型别名
using DefaultPointType = ikdTree_PointType<int>;

/**
 * @brief 定义包围盒类型结构
 * 
 * 用于表示3D空间中的轴对齐包围盒
 */
struct BoxPointType
{
    double vertex_min[3];  ///< 包围盒最小顶点坐标 [x_min, y_min, z_min] (改为double)
    double vertex_max[3];  ///< 包围盒最大顶点坐标 [x_max, y_max, z_max] (改为double)
    
    /**
     * @brief 默认构造函数 - 初始化包围盒
     */
    BoxPointType() 
    {
        for(int i = 0; i < 3; ++i) {
            vertex_min[i] = 0.0;
            vertex_max[i] = 0.0;
        }
    }
    
    /**
     * @brief 构造函数 - 使用最小和最大点初始化包围盒
     * @param min_point 最小点坐标
     * @param max_point 最大点坐标
     */
    template<typename DataType>
    BoxPointType(const ikdTree_PointType<DataType>& min_point, const ikdTree_PointType<DataType>& max_point)
    {
        vertex_min[0] = min_point.x; vertex_min[1] = min_point.y; vertex_min[2] = min_point.z;
        vertex_max[0] = max_point.x; vertex_max[1] = max_point.y; vertex_max[2] = max_point.z;
    }
    
    /**
     * @brief 从float数组构造包围盒 - 兼容性函数
     * @param min_vals 最小值数组[3]
     * @param max_vals 最大值数组[3]
     */
    BoxPointType(const float min_vals[3], const float max_vals[3])
    {
        for(int i = 0; i < 3; ++i) {
            vertex_min[i] = static_cast<double>(min_vals[i]);
            vertex_max[i] = static_cast<double>(max_vals[i]);
        }
    }
};

enum operation_set {ADD_POINT, DELETE_POINT, DELETE_BOX, ADD_BOX, DOWNSAMPLE_DELETE, PUSH_DOWN};

enum delete_point_storage_set {NOT_RECORD, DELETE_POINTS_REC, MULTI_THREAD_REC};

/**
 * @brief 手动队列类模板 - 使用Qt容器重新实现
 * 
 * 基于QVector实现的高性能循环队列，用于操作日志记录
 * @tparam T 队列元素类型
 */
template <typename T>
class MANUAL_Q
{
private:
    int m_head;        ///< 队列头指针
    int m_tail;        ///< 队列尾指针  
    int m_counter;     ///< 元素计数器
    QVector<T> m_queue; ///< 基于QVector的队列存储
    bool m_isEmpty;    ///< 空队列标志
    mutable QMutex m_mutex; ///< 线程安全互斥锁

public:
    /**
     * @brief 构造函数 - 初始化队列
     */
    explicit MANUAL_Q() 
        : m_head(0), m_tail(0), m_counter(0), m_isEmpty(true)
    {
        m_queue.resize(Q_LEN);
    }
    
    /**
     * @brief 弹出队首元素
     */
    void pop()
    {
        QMutexLocker locker(&m_mutex);
        if (m_counter == 0) return;
        m_head++;
        m_head %= Q_LEN;
        m_counter--;
        if (m_counter == 0) m_isEmpty = true;
    }
    
    /**
     * @brief 获取队首元素
     * @return 队首元素的引用
     */
    T front() const
    {
        QMutexLocker locker(&m_mutex);
        return m_queue[m_head];
    }
    
    /**
     * @brief 获取队尾元素  
     * @return 队尾元素的引用
     */
    T back() const
    {
        QMutexLocker locker(&m_mutex);
        return m_queue[m_tail];
    }
    
    /**
     * @brief 清空队列
     */
    void clear()
    {
        QMutexLocker locker(&m_mutex);
        m_head = 0;
        m_tail = 0;
        m_counter = 0;
        m_isEmpty = true;
    }
    
    /**
     * @brief 向队尾添加元素
     * @param op 要添加的元素
     */
    void push(const T& op)
    {
        QMutexLocker locker(&m_mutex);
        m_queue[m_tail] = op;
        m_counter++;
        if (m_isEmpty) m_isEmpty = false;
        m_tail++;
        m_tail %= Q_LEN;
    }
    
    /**
     * @brief 检查队列是否为空
     * @return true表示队列为空
     */
    bool empty() const
    {
        QMutexLocker locker(&m_mutex);
        return m_isEmpty;
    }
    
    /**
     * @brief 获取队列大小
     * @return 队列中元素的数量
     */
    int size() const
    {
        QMutexLocker locker(&m_mutex);
        return m_counter;
    }
};

/**
 * @brief 增量式K-D树类模板 - Qt版本实现
 * 
 * 支持动态插入、删除和搜索的高效3D点云数据结构
 * @tparam PointType 点类型，通常为ikdTree_PointType<DataType>
 */
template<typename PointType>
class KD_TREE
{
public:
    using PointVector = QVector<PointType>;          ///< 点向量类型定义
    using Ptr = QSharedPointer<KD_TREE<PointType>>; ///< 智能指针类型定义
    
    /**
     * @brief K-D树节点结构体 - 使用Qt内存管理
     */
    struct KD_TREE_NODE
    {
        PointType point;                        ///< 节点存储的点数据
        quint8 division_axis;                   ///< 分割轴 (0=x, 1=y, 2=z)
        int TreeSize = 1;                       ///< 子树大小
        int invalid_point_num = 0;              ///< 无效点数量
        int down_del_num = 0;                   ///< 下采样删除点数量
        bool point_deleted = false;             ///< 点删除标志
        bool tree_deleted = false;              ///< 子树删除标志
        bool point_downsample_deleted = false;  ///< 点下采样删除标志
        bool tree_downsample_deleted = false;   ///< 子树下采样删除标志
        bool need_push_down_to_left = false;    ///< 需要向左子树推送标志
        bool need_push_down_to_right = false;   ///< 需要向右子树推送标志  
        bool working_flag = false;              ///< 工作状态标志
        double radius_sq;                       ///< 节点包围球半径平方 (改为double)
        QMutex push_down_mutex;                 ///< 推送操作互斥锁
        double node_range_x[2], node_range_y[2], node_range_z[2]; ///< 节点包围盒范围 (改为double)
        KD_TREE_NODE* left_son_ptr = nullptr;   ///< 左子节点指针
        KD_TREE_NODE* right_son_ptr = nullptr;  ///< 右子节点指针
        KD_TREE_NODE* father_ptr = nullptr;     ///< 父节点指针
        // 用于论文数据记录
        double alpha_del;                       ///< 删除平衡因子 (改为double)
        double alpha_bal;                       ///< 结构平衡因子 (改为double)
        
        /**
         * @brief 默认构造函数 - 初始化节点
         */
        KD_TREE_NODE() = default;
        
        /**
         * @brief 析构函数 - 清理资源
         */
        ~KD_TREE_NODE() = default;
    };

    /**
     * @brief 操作日志类型结构体
     */
    struct Operation_Logger_Type
    {
        PointType point;
        BoxPointType boxpoint;
        bool tree_deleted = false;
        bool tree_downsample_deleted = false;
        operation_set op;
    };

    /**
     * @brief 点类型比较结构体 - 用于优先队列
     */
    struct PointType_CMP
    {
        PointType point;
        double dist = 0.0;  ///< 改为double类型
        
        PointType_CMP(const PointType& p = PointType(), double d = std::numeric_limits<double>::infinity())
            : point(p), dist(d)
        {
        }
        
        bool operator < (const PointType_CMP& a) const
        {
            if (qAbs(dist - a.dist) < 1e-10) return point.x < a.point.x;
            else return dist < a.dist;
        }    
    };

    /**
     * @brief 手动堆类 - 用于K近邻搜索的优先队列
     */
    class MANUAL_HEAP
    {
    public:
        MANUAL_HEAP(int max_capacity = 100)
            : m_cap(max_capacity), m_heapSize(0)
        {
            m_heap.resize(max_capacity);
        }

        ~MANUAL_HEAP() = default;

        void pop()
        {
            if (m_heapSize == 0) return;
            m_heap[0] = m_heap[m_heapSize-1];
            m_heapSize--;
            moveDown(0);
        }

        PointType_CMP top() const { return m_heap[0]; }

        void push(const PointType_CMP& point)
        {
            if (m_heapSize >= m_cap) return;
            m_heap[m_heapSize] = point;
            floatUp(m_heapSize);
            m_heapSize++;
        }

        int size() const { return m_heapSize; }

        void clear() { m_heapSize = 0; }
        
    private:
        int m_heapSize = 0;
        int m_cap = 0;        
        QVector<PointType_CMP> m_heap;
        
        void moveDown(int heap_index)
        {
            int l = heap_index * 2 + 1;
            PointType_CMP tmp = m_heap[heap_index];
            while (l < m_heapSize) {
                if (l + 1 < m_heapSize && m_heap[l] < m_heap[l+1]) l++;
                if (tmp < m_heap[l]) {
                    m_heap[heap_index] = m_heap[l];
                    heap_index = l;
                    l = heap_index * 2 + 1;
                } else break;
            }
            m_heap[heap_index] = tmp;
        }
        
        void floatUp(int heap_index)
        {
            int ancestor = (heap_index-1)/2;
            PointType_CMP tmp = m_heap[heap_index];
            while (heap_index > 0) {
                if (m_heap[ancestor] < tmp) {
                    m_heap[heap_index] = m_heap[ancestor];
                    heap_index = ancestor;
                    ancestor = (heap_index-1)/2;
                } else break;
            }
            m_heap[heap_index] = tmp;
        }
    };    

private:
    // 多线程重建相关 - 使用Qt线程机制
    QAtomicInt m_terminationFlag;               ///< 终止标志（原子操作）
    QAtomicInt m_rebuildFlag;                   ///< 重建标志（原子操作）
    QScopedPointer<QThread> m_rebuildThread;    ///< 重建线程智能指针
    mutable QMutex m_terminationFlagMutex;      ///< 终止标志互斥锁
    mutable QMutex m_rebuildPtrMutex;           ///< 重建指针互斥锁
    mutable QMutex m_workingFlagMutex;          ///< 工作标志互斥锁
    mutable QMutex m_searchFlagMutex;           ///< 搜索标志互斥锁
    mutable QMutex m_rebuildLoggerMutex;        ///< 重建日志互斥锁
    mutable QMutex m_pointsDeletedRebuildMutex; ///< 删除点重建互斥锁
    MANUAL_Q<Operation_Logger_Type> m_rebuildLogger; ///< 重建操作日志队列
    PointVector m_rebuildPclStorage;            ///< 重建点云存储
    KD_TREE_NODE** m_rebuildPtr = nullptr;      ///< 重建指针
    QAtomicInt m_searchMutexCounter;            ///< 搜索互斥计数器
    
    // K-D树函数和增强变量 - Qt风格命名
    int m_treesizeTemp = 0;                     ///< 临时树大小
    int m_validnumTemp = 0;                     ///< 临时有效节点数
    double m_alphaBalTemp = 0.5;                ///< 临时平衡因子 (改为double)
    double m_alphaDelTemp = 0.0;                ///< 临时删除因子 (改为double)
    double m_deleteCriterionParam = 0.5;        ///< 删除判据参数 (改为double)
    double m_balanceCriterionParam = 0.7;       ///< 平衡判据参数 (改为double)
    double m_downsampleSize = 0.2;              ///< 下采样尺寸 (改为double)
    bool m_deleteStorageDisabled = false;       ///< 删除存储禁用标志
    KD_TREE_NODE* m_staticRootNode = nullptr;   ///< 静态根节点指针
    PointVector m_pointsDeleted;                ///< 已删除点集合
    PointVector m_downsampleStorage;            ///< 下采样存储
    PointVector m_multithreadPointsDeleted;     ///< 多线程删除点集合

    // 私有方法实现（header-only模板设计，无需声明）
    
    /**
     * @brief 多线程重建处理函数实现 - Qt版本
     */
    void multiThreadRebuild() {
        bool terminated = false;
        KD_TREE_NODE* fatherPtr;
        KD_TREE_NODE** newNodePtr;
        Q_UNUSED(newNodePtr);
        
        while (!terminated) {
            {
                QMutexLocker terminationLocker(&m_terminationFlagMutex);
                terminated = m_terminationFlag.loadRelaxed();
            }
            
            if (terminated) break;
            
            {
                QMutexLocker rebuildLocker(&m_rebuildPtrMutex);
                QMutexLocker workingLocker(&m_workingFlagMutex);
                
                if (m_rebuildPtr != nullptr) {
                    // 执行重建逻辑
                    if (!m_rebuildLogger.empty()) {
                        qDebug() << u8"警告: 重建日志队列非空!";
                    }
                    
                    m_rebuildFlag.storeRelaxed(1);
                    if (*m_rebuildPtr == m_rootNode) {
                        m_treesizeTemp = m_rootNode->TreeSize;
                        m_validnumTemp = m_rootNode->TreeSize - m_rootNode->invalid_point_num;
                        m_alphaBalTemp = m_rootNode->alpha_bal;
                        m_alphaDelTemp = m_rootNode->alpha_del;
                    }
                    
                    KD_TREE_NODE* oldRootNode = (*m_rebuildPtr);
                    fatherPtr = (*m_rebuildPtr)->father_ptr;
                    m_rebuildPclStorage.clear();
                    
                    // 锁定搜索
                    {
                        QMutexLocker searchLocker(&m_searchFlagMutex);
                        while (m_searchMutexCounter.loadRelaxed() != 0) {
                            searchLocker.unlock();
                            QThread::usleep(1);
                            searchLocker.relock();
                        }
                        m_searchMutexCounter.storeRelaxed(-1);
                    }
                    
                    // 锁定删除点缓存并展平树
                    {
                        QMutexLocker pointsLocker(&m_pointsDeletedRebuildMutex);
                        flatten(*m_rebuildPtr, m_rebuildPclStorage, MULTI_THREAD_REC);
                    }
                    
                    // 解锁搜索
                    {
                        QMutexLocker searchLocker(&m_searchFlagMutex);
                        m_searchMutexCounter.storeRelaxed(0);
                    }
                    
                    workingLocker.unlock();
                    
                    // 重建和更新遗漏的操作
                    Operation_Logger_Type operation;
                    KD_TREE_NODE* newRootNode = nullptr;
                    
                    if (m_rebuildPclStorage.size() > 0) {
                        buildTree(&newRootNode, 0, m_rebuildPclStorage.size()-1, m_rebuildPclStorage);
                        
                        // 处理重建期间积累的操作
                        workingLocker.relock();
                        {
                            QMutexLocker loggerLocker(&m_rebuildLoggerMutex);
                            int tmpCounter = 0;
                            while (!m_rebuildLogger.empty()) {
                                operation = m_rebuildLogger.front();
                                m_maxQueueSize = qMax(m_maxQueueSize, m_rebuildLogger.size());
                                m_rebuildLogger.pop();
                                loggerLocker.unlock();
                                workingLocker.unlock();
                                runOperation(&newRootNode, operation);
                                tmpCounter++;
                                if (tmpCounter % 10 == 0) QThread::usleep(1);
                                workingLocker.relock();
                                loggerLocker.relock();
                            }
                        }
                    }
                    
                    // 替换到原始树
                    {
                        QMutexLocker searchLocker(&m_searchFlagMutex);
                        while (m_searchMutexCounter.loadRelaxed() != 0) {
                            searchLocker.unlock();
                            QThread::usleep(1);
                            searchLocker.relock();
                        }
                        m_searchMutexCounter.storeRelaxed(-1);
                    }
                    
                    if (fatherPtr->left_son_ptr == *m_rebuildPtr) {
                        fatherPtr->left_son_ptr = newRootNode;
                    } else if (fatherPtr->right_son_ptr == *m_rebuildPtr) {
                        fatherPtr->right_son_ptr = newRootNode;
                    } else {
                        qCritical() << u8"错误: 父指针与当前节点不兼容";
                    }
                    
                    if (newRootNode != nullptr) newRootNode->father_ptr = fatherPtr;
                    (*m_rebuildPtr) = newRootNode;
                    
                    if (fatherPtr == m_staticRootNode) m_rootNode = m_staticRootNode->left_son_ptr;
                    
                    KD_TREE_NODE* updateRoot = *m_rebuildPtr;
                    while (updateRoot != nullptr && updateRoot != m_rootNode) {
                        updateRoot = updateRoot->father_ptr;
                        if (updateRoot->working_flag) break;
                        if (updateRoot == updateRoot->father_ptr->left_son_ptr && updateRoot->father_ptr->need_push_down_to_left) break;
                        if (updateRoot == updateRoot->father_ptr->right_son_ptr && updateRoot->father_ptr->need_push_down_to_right) break;
                        update(updateRoot);
                    }
                    
                    {
                        QMutexLocker searchLocker(&m_searchFlagMutex);
                        m_searchMutexCounter.storeRelaxed(0);
                    }
                    
                    m_rebuildPtr = nullptr;
                    workingLocker.unlock();
                    m_rebuildFlag.storeRelaxed(0);
                    
                    // 删除废弃的树节点
                    deleteTreeNodes(&oldRootNode);
                }
            }
            
            QThread::usleep(100);
        }
        
        qDebug() << u8"重建线程正常终止";
    }

public:
    // 公有成员变量
    PointVector m_pclStorage;                  ///< 点云存储向量
    KD_TREE_NODE* m_rootNode = nullptr;        ///< 根节点指针
    int m_maxQueueSize = 0;                    ///< 最大队列大小记录
    
    /**
     * @brief 构造函数 - 初始化ikd-Tree
     * @param deleteParam 删除判据参数
     * @param balanceParam 平衡判据参数  
     * @param boxLength 下采样包围盒边长
     */
    explicit KD_TREE(double deleteParam = 0.5, double balanceParam = 0.6, double boxLength = 0.2)
        : m_terminationFlag(0), m_rebuildFlag(0), m_searchMutexCounter(0)
    {
        /**
         * @brief 构造函数实现 - 初始化ikd-Tree所有参数
         */
        m_deleteCriterionParam = deleteParam;
        m_balanceCriterionParam = balanceParam;
        m_downsampleSize = boxLength;
        m_rebuildLogger.clear();
        startThread();
        
        qDebug() << u8"ikd-Tree Qt版本初始化完成" 
                 << u8"删除参数:" << deleteParam 
                 << u8"平衡参数:" << balanceParam;
    }
    
    /**
     * @brief 析构函数实现 - 清理所有资源
     */
    ~KD_TREE()
    {
        stopThread();
        m_deleteStorageDisabled = true;
        deleteTreeNodes(&m_rootNode);
        m_pclStorage.clear();
        m_rebuildLogger.clear();
        
        qDebug() << u8"ikd-Tree Qt版本资源清理完成";
    }
    
    /**
     * @brief 设置删除判据参数
     */
    void setDeleteCriterionParam(double deleteParam)
    {
        /**
         * @brief 设置删除判据参数实现
         */
        QMutexLocker locker(&m_workingFlagMutex);
        m_deleteCriterionParam = deleteParam;
    }
    
    /**
     * @brief 设置平衡判据参数
     */
    void setBalanceCriterionParam(double balanceParam)
    {
        /**
         * @brief 设置平衡判据参数实现
         */
        QMutexLocker locker(&m_workingFlagMutex);
        m_balanceCriterionParam = balanceParam;
    }
    
    /**
     * @brief 设置下采样参数
     */
    void setDownsampleParam(double downsampleParam)
    {
        /**
         * @brief 设置下采样参数实现
         */
        QMutexLocker locker(&m_workingFlagMutex);
        m_downsampleSize = downsampleParam;
    }
    
    /**
     * @brief 重新初始化K-D树参数
     */
    void initializeKdTree(double deleteParam = 0.5, double balanceParam = 0.7, double boxLength = 0.2)
    {
        /**
         * @brief 重新初始化K-D树参数实现
         */
        setDeleteCriterionParam(deleteParam);
        setBalanceCriterionParam(balanceParam);
        setDownsampleParam(boxLength);
    }
    
    /**
     * @brief 获取树大小 - 线程安全版本
     */
    int size() const
    {
        /**
         * @brief 获取树大小实现 - 线程安全版本
         */
        if (m_rebuildPtr == nullptr || *m_rebuildPtr != m_rootNode) {
            if (m_rootNode != nullptr) {
                return m_rootNode->TreeSize;
            } else {
                return 0;
            }
        } else {
            QMutexLocker locker(&m_workingFlagMutex);
            if (locker.mutex()->tryLock()) {
                int s = m_rootNode->TreeSize;
                locker.unlock();
                return s;
            } else {
                return m_treesizeTemp;
            }
        }
    }
    
    /**
     * @brief 获取有效节点数
     */
    int validnum() const
    {
        /**
         * @brief 获取有效节点数实现
         */
        if (m_rebuildPtr == nullptr || *m_rebuildPtr != m_rootNode) {
            if (m_rootNode != nullptr)
                return (m_rootNode->TreeSize - m_rootNode->invalid_point_num);
            else 
                return 0;
        } else {
            QMutexLocker locker(&m_workingFlagMutex);
            if (locker.mutex()->tryLock()) {
                int s = m_rootNode->TreeSize - m_rootNode->invalid_point_num;
                locker.unlock();
                return s;
            } else {
                return -1;
            }
        }
    }
    
    /**
     * @brief 获取树范围 - 返回整个树的包围盒
     */
    BoxPointType treeRange() const
    {
        /**
         * @brief 获取树范围实现 - 返回整个树的包围盒
         */
        BoxPointType range;
        if (m_rebuildPtr == nullptr || *m_rebuildPtr != m_rootNode) {
            if (m_rootNode != nullptr) {
                range.vertex_min[0] = m_rootNode->node_range_x[0];
                range.vertex_min[1] = m_rootNode->node_range_y[0];
                range.vertex_min[2] = m_rootNode->node_range_z[0];
                range.vertex_max[0] = m_rootNode->node_range_x[1];
                range.vertex_max[1] = m_rootNode->node_range_y[1];
                range.vertex_max[2] = m_rootNode->node_range_z[1];
            } else {
                range = BoxPointType();
            }
        } else {
            QMutexLocker locker(&m_workingFlagMutex);
            if (locker.mutex()->tryLock()) {
                range.vertex_min[0] = m_rootNode->node_range_x[0];
                range.vertex_min[1] = m_rootNode->node_range_y[0];
                range.vertex_min[2] = m_rootNode->node_range_z[0];
                range.vertex_max[0] = m_rootNode->node_range_x[1];
                range.vertex_max[1] = m_rootNode->node_range_y[1];
                range.vertex_max[2] = m_rootNode->node_range_z[1];
                locker.unlock();
            } else {
                range = BoxPointType();
            }
        }
        return range;
    }
    
    /**
     * @brief 获取平衡因子和删除因子
     */
    void rootAlpha(double& alphaBal, double& alphaDel) const
    {
        /**
         * @brief 获取根节点平衡因子实现
         */
        if (m_rebuildPtr == nullptr || *m_rebuildPtr != m_rootNode) {
            alphaBal = m_rootNode->alpha_bal;
            alphaDel = m_rootNode->alpha_del;
            return;
        } else {
            QMutexLocker locker(&m_workingFlagMutex);
            if (locker.mutex()->tryLock()) {
                alphaBal = m_rootNode->alpha_bal;
                alphaDel = m_rootNode->alpha_del;
                locker.unlock();
                return;
            } else {
                alphaBal = m_alphaBalTemp;
                alphaDel = m_alphaDelTemp;      
                return;
            }
        }    
    }
    
    /**
     * @brief 构建K-D树
     */
    void build(const PointVector& pointCloud)
    {
        /**
         * @brief 构建K-D树实现 - 从点云数据构建完整的树
         */
        if (m_rootNode != nullptr) {
            deleteTreeNodes(&m_rootNode);
        }
        if (pointCloud.size() == 0) return;
        
        m_staticRootNode = new KD_TREE_NODE;
        initTreeNode(m_staticRootNode);
        
        // 复制点云数据到临时存储
        PointVector tempStorage = pointCloud;
        buildTree(&m_staticRootNode->left_son_ptr, 0, tempStorage.size()-1, tempStorage);
        update(m_staticRootNode);
        m_staticRootNode->TreeSize = 0;
        m_rootNode = m_staticRootNode->left_son_ptr;
        
        qDebug() << u8"K-D树构建完成，节点数:" << pointCloud.size();
    }
    
    /**
     * @brief 包围盒搜索
     */
    void boxSearch(const BoxPointType& boxOfPoint, PointVector& storage)
    {
        /**
         * @brief 包围盒搜索实现
         */
        storage.clear();
        searchByRange(m_rootNode, boxOfPoint, storage);
    }
    
    /**
     * @brief 半径搜索
     */
    void radiusSearch(const PointType& point, double radius, PointVector& storage)
    {
        /**
         * @brief 半径搜索实现
         */
        storage.clear();
        searchByRadius(m_rootNode, point, radius, storage);
    }
    

    
    /**
     * @brief 添加点集合
     */
    int addPoints(PointVector& pointToAdd, bool downsampleOn)
    {
        /**
         * @brief 批量添加点实现 - 支持下采样功能
         */
        int newPointSize = pointToAdd.size();
        int treeSize = size();
        BoxPointType boxOfPoint;
        PointType downsampleResult, midPoint;
        bool downsampleSwitch = downsampleOn && DOWNSAMPLE_SWITCH;
        double minDist, tmpDist;
        int tmpCounter = 0;

        Q_UNUSED(newPointSize);
        Q_UNUSED(treeSize);
        
        for (int i = 0; i < pointToAdd.size(); i++) {
            if (downsampleSwitch) {
                boxOfPoint.vertex_min[0] = qFloor(pointToAdd[i].x / m_downsampleSize) * m_downsampleSize;
                boxOfPoint.vertex_max[0] = boxOfPoint.vertex_min[0] + m_downsampleSize;
                boxOfPoint.vertex_min[1] = qFloor(pointToAdd[i].y / m_downsampleSize) * m_downsampleSize;
                boxOfPoint.vertex_max[1] = boxOfPoint.vertex_min[1] + m_downsampleSize;
                boxOfPoint.vertex_min[2] = qFloor(pointToAdd[i].z / m_downsampleSize) * m_downsampleSize;
                boxOfPoint.vertex_max[2] = boxOfPoint.vertex_min[2] + m_downsampleSize;
                
                midPoint.x = boxOfPoint.vertex_min[0] + (boxOfPoint.vertex_max[0] - boxOfPoint.vertex_min[0]) / 2.0;
                midPoint.y = boxOfPoint.vertex_min[1] + (boxOfPoint.vertex_max[1] - boxOfPoint.vertex_min[1]) / 2.0;
                midPoint.z = boxOfPoint.vertex_min[2] + (boxOfPoint.vertex_max[2] - boxOfPoint.vertex_min[2]) / 2.0;
                
                m_downsampleStorage.clear();
                searchByRange(m_rootNode, boxOfPoint, m_downsampleStorage);
                minDist = calcDist(pointToAdd[i], midPoint);
                downsampleResult = pointToAdd[i];
                
                for (int index = 0; index < m_downsampleStorage.size(); index++) {
                    tmpDist = calcDist(m_downsampleStorage[index], midPoint);
                    if (tmpDist < minDist) {
                        minDist = tmpDist;
                        downsampleResult = m_downsampleStorage[index];
                    }
                }
                
                if (m_rebuildPtr == nullptr || *m_rebuildPtr != m_rootNode) {
                    if (m_downsampleStorage.size() > 1 || samePoint(pointToAdd[i], downsampleResult)) {
                        if (m_downsampleStorage.size() > 0) {
                            deleteByRange(&m_rootNode, boxOfPoint, true, true);
                        }
                        addByPoint(&m_rootNode, downsampleResult, true, m_rootNode->division_axis);
                        tmpCounter++;
                    }
                } else {
                    if (m_downsampleStorage.size() > 1 || samePoint(pointToAdd[i], downsampleResult)) {
                        Operation_Logger_Type operationDelete, operation;
                        operationDelete.boxpoint = boxOfPoint;
                        operationDelete.op = DOWNSAMPLE_DELETE;
                        operation.point = downsampleResult;
                        operation.op = ADD_POINT;
                        
                        QMutexLocker workingLocker(&m_workingFlagMutex);
                        if (m_downsampleStorage.size() > 0) {
                            deleteByRange(&m_rootNode, boxOfPoint, false, true);
                        }
                        addByPoint(&m_rootNode, downsampleResult, false, m_rootNode->division_axis);
                        tmpCounter++;
                        
                        if (m_rebuildFlag.loadRelaxed()) {
                            QMutexLocker loggerLocker(&m_rebuildLoggerMutex);
                            if (m_downsampleStorage.size() > 0) {
                                m_rebuildLogger.push(operationDelete);
                            }
                            m_rebuildLogger.push(operation);
                        }
                    }
                }
            } else {
                if (m_rebuildPtr == nullptr || *m_rebuildPtr != m_rootNode) {
                    addByPoint(&m_rootNode, pointToAdd[i], true, m_rootNode->division_axis);
                } else {
                    Operation_Logger_Type operation;
                    operation.point = pointToAdd[i];
                    operation.op = ADD_POINT;
                    
                    QMutexLocker workingLocker(&m_workingFlagMutex);
                    addByPoint(&m_rootNode, pointToAdd[i], false, m_rootNode->division_axis);
                    
                    if (m_rebuildFlag.loadRelaxed()) {
                        QMutexLocker loggerLocker(&m_rebuildLoggerMutex);
                        m_rebuildLogger.push(operation);
                    }
                }
            }
        }
        return tmpCounter;
    }
    
    /**
     * @brief 删除点集合
     */
    void deletePoints(PointVector& pointToDel)
    {
        /**
         * @brief 批量删除点实现
         */
        for (int i = 0; i < pointToDel.size(); i++) {
            if (m_rebuildPtr == nullptr || *m_rebuildPtr != m_rootNode) {
                deleteByPoint(&m_rootNode, pointToDel[i], true);
            } else {
                Operation_Logger_Type operation;
                operation.point = pointToDel[i];
                operation.op = DELETE_POINT;
                
                QMutexLocker workingLocker(&m_workingFlagMutex);
                deleteByPoint(&m_rootNode, pointToDel[i], false);
                
                if (m_rebuildFlag.loadRelaxed()) {
                    QMutexLocker loggerLocker(&m_rebuildLoggerMutex);
                    m_rebuildLogger.push(operation);
                }
            }
        }
    }
    
    /**
     * @brief 添加包围盒集合
     */
    void addPointBoxes(QVector<BoxPointType>& boxPoints)
    {
        /**
         * @brief 批量添加包围盒实现 - 恢复指定区域内的点
         */
        for (int i = 0; i < boxPoints.size(); i++) {
            if (m_rebuildPtr == nullptr || *m_rebuildPtr != m_rootNode) {
                addByRange(&m_rootNode, boxPoints[i], true);
            } else {
                Operation_Logger_Type operation;
                operation.boxpoint = boxPoints[i];
                operation.op = ADD_BOX;
                
                QMutexLocker workingLocker(&m_workingFlagMutex);
                addByRange(&m_rootNode, boxPoints[i], false);
                
                if (m_rebuildFlag.loadRelaxed()) {
                    QMutexLocker loggerLocker(&m_rebuildLoggerMutex);
                    m_rebuildLogger.push(operation);
                }
            }
        }
    }
    
    /**
     * @brief 删除包围盒集合
     */
    int deletePointBoxes(QVector<BoxPointType>& boxPoints)
    {
        /**
         * @brief 批量删除包围盒实现
         */
        int tmpCounter = 0;
        for (int i = 0; i < boxPoints.size(); i++) {
            if (m_rebuildPtr == nullptr || *m_rebuildPtr != m_rootNode) {
                tmpCounter += deleteByRange(&m_rootNode, boxPoints[i], true, false);
            } else {
                Operation_Logger_Type operation;
                operation.boxpoint = boxPoints[i];
                operation.op = DELETE_BOX;
                
                QMutexLocker workingLocker(&m_workingFlagMutex);
                tmpCounter += deleteByRange(&m_rootNode, boxPoints[i], false, false);
                
                if (m_rebuildFlag.loadRelaxed()) {
                    QMutexLocker loggerLocker(&m_rebuildLoggerMutex);
                    m_rebuildLogger.push(operation);
                }
            }
        }
        return tmpCounter;
    }
    
    /**
     * @brief 平铺树节点
     */
    void flatten(KD_TREE_NODE* root, PointVector& storage, delete_point_storage_set storageType)
    {
        /**
         * @brief 展平树结构实现 - 将子树所有点提取到向量中
         */
        if (root == nullptr) return;
        
        pushDown(root);
        if (!root->point_deleted) {
            storage.append(root->point);
        }
        
        flatten(root->left_son_ptr, storage, storageType);
        flatten(root->right_son_ptr, storage, storageType);
        
        switch (storageType) {
        case NOT_RECORD:
            break;
        case DELETE_POINTS_REC:
            if (root->point_deleted && !root->point_downsample_deleted) {
                m_pointsDeleted.append(root->point);
            }
            break;
        case MULTI_THREAD_REC:
            if (root->point_deleted && !root->point_downsample_deleted) {
                m_multithreadPointsDeleted.append(root->point);
            }
            break;
        default:
            break;
        }
    }
    
    /**
     * @brief 获取已删除的点
     */
    void acquireRemovedPoints(PointVector& removedPoints)
    {
        /**
         * @brief 获取已删除点实现
         */
        QMutexLocker pointsLocker(&m_pointsDeletedRebuildMutex);
        for (int i = 0; i < m_pointsDeleted.size(); i++) {
            removedPoints.append(m_pointsDeleted[i]);
        }
        for (int i = 0; i < m_multithreadPointsDeleted.size(); i++) {
            removedPoints.append(m_multithreadPointsDeleted[i]);
        }
        m_pointsDeleted.clear();
        m_multithreadPointsDeleted.clear();
    }

    void startThread() {
        /**
         * @brief 启动重建线程实现 - 使用Qt线程机制
         */
        m_terminationFlag.storeRelaxed(0);
        
        // 创建重建线程
        m_rebuildThread.reset(QThread::create([this]() {
            this->multiThreadRebuild();
        }));
        
        m_rebuildThread->start();
        qDebug() << u8"Qt多线程重建已启动";
    }

    void stopThread() {
        /**
         * @brief 停止重建线程实现
         */
        {
            QMutexLocker locker(&m_terminationFlagMutex);
            m_terminationFlag.storeRelaxed(1);
        }
        
        if (m_rebuildThread && m_rebuildThread->isRunning()) {
            m_rebuildThread->quit();
            m_rebuildThread->wait();
        }
        
        qDebug() << u8"Qt多线程重建已停止";
    }

    void initTreeNode(KD_TREE_NODE* root) {
        /**
         * @brief 初始化树节点实现 - 设置节点默认值
         */
        if (!root) return;
        
        root->point = PointType(0.0, 0.0, 0.0);
        root->node_range_x[0] = root->node_range_x[1] = 0.0;
        root->node_range_y[0] = root->node_range_y[1] = 0.0;
        root->node_range_z[0] = root->node_range_z[1] = 0.0;
        root->division_axis = 0;
        root->father_ptr = nullptr;
        root->left_son_ptr = nullptr;
        root->right_son_ptr = nullptr;
        root->TreeSize = 0;
        root->invalid_point_num = 0;
        root->down_del_num = 0;
        root->point_deleted = false;
        root->tree_deleted = false;
        root->need_push_down_to_left = false;
        root->need_push_down_to_right = false;
        root->point_downsample_deleted = false;
        root->working_flag = false;
        // Qt的QMutex已经在构造函数中自动初始化
    }

    void buildTree(KD_TREE_NODE** root, int l, int r, PointVector& storage) {
        /**
         * @brief 递归构建K-D树实现 - 核心构建算法
         */
        if (l > r) return;
        
        *root = new KD_TREE_NODE;
        initTreeNode(*root);
        int mid = (l + r) >> 1;
        int divAxis = 0;
        
        // 找到最佳分割轴
        double minValue[3] = {std::numeric_limits<double>::infinity(), 
                             std::numeric_limits<double>::infinity(), 
                             std::numeric_limits<double>::infinity()};
        double maxValue[3] = {-std::numeric_limits<double>::infinity(), 
                             -std::numeric_limits<double>::infinity(), 
                             -std::numeric_limits<double>::infinity()};
        double dimRange[3] = {0, 0, 0};
        
        for (int i = l; i <= r; i++) {
            minValue[0] = qMin(minValue[0], storage[i].x);
            minValue[1] = qMin(minValue[1], storage[i].y);
            minValue[2] = qMin(minValue[2], storage[i].z);
            maxValue[0] = qMax(maxValue[0], storage[i].x);
            maxValue[1] = qMax(maxValue[1], storage[i].y);
            maxValue[2] = qMax(maxValue[2], storage[i].z);
        }
        
        // 选择最长维度作为分割轴
        for (int i = 0; i < 3; i++) dimRange[i] = maxValue[i] - minValue[i];
        for (int i = 1; i < 3; i++) if (dimRange[i] > dimRange[divAxis]) divAxis = i;
        
        (*root)->division_axis = divAxis;
        
        // 按分割轴排序并递归构建
        switch (divAxis) {
        case 0:
            std::nth_element(storage.begin() + l, storage.begin() + mid, storage.begin() + r + 1, pointCmpX);
            break;
        case 1:
            std::nth_element(storage.begin() + l, storage.begin() + mid, storage.begin() + r + 1, pointCmpY);
            break;
        case 2:
            std::nth_element(storage.begin() + l, storage.begin() + mid, storage.begin() + r + 1, pointCmpZ);
            break;
        default:
            std::nth_element(storage.begin() + l, storage.begin() + mid, storage.begin() + r + 1, pointCmpX);
            break;
        }
        
        (*root)->point = storage[mid];
        KD_TREE_NODE* leftSon = nullptr;
        KD_TREE_NODE* rightSon = nullptr;
        buildTree(&leftSon, l, mid-1, storage);
        buildTree(&rightSon, mid+1, r, storage);
        (*root)->left_son_ptr = leftSon;
        (*root)->right_son_ptr = rightSon;
        update(*root);
    }

    void searchByRange(KD_TREE_NODE* root, const BoxPointType& boxpoint, PointVector& storage) {
        /**
         * @brief 按范围搜索实现
         */
        if (root == nullptr) return;
        
        pushDown(root);
        
        if (boxpoint.vertex_max[0] <= root->node_range_x[0] || boxpoint.vertex_min[0] > root->node_range_x[1]) return;
        if (boxpoint.vertex_max[1] <= root->node_range_y[0] || boxpoint.vertex_min[1] > root->node_range_y[1]) return;
        if (boxpoint.vertex_max[2] <= root->node_range_z[0] || boxpoint.vertex_min[2] > root->node_range_z[1]) return;
        
        if (boxpoint.vertex_min[0] <= root->node_range_x[0] && boxpoint.vertex_max[0] > root->node_range_x[1] && 
            boxpoint.vertex_min[1] <= root->node_range_y[0] && boxpoint.vertex_max[1] > root->node_range_y[1] && 
            boxpoint.vertex_min[2] <= root->node_range_z[0] && boxpoint.vertex_max[2] > root->node_range_z[1]) {
            flatten(root, storage, NOT_RECORD);
            return;
        }
        
        if (boxpoint.vertex_min[0] <= root->point.x && boxpoint.vertex_max[0] > root->point.x && 
            boxpoint.vertex_min[1] <= root->point.y && boxpoint.vertex_max[1] > root->point.y && 
            boxpoint.vertex_min[2] <= root->point.z && boxpoint.vertex_max[2] > root->point.z) {
            if (!root->point_deleted) storage.append(root->point);
        }
        
        if ((m_rebuildPtr == nullptr) || root->left_son_ptr != *m_rebuildPtr) {
            searchByRange(root->left_son_ptr, boxpoint, storage);
        } else {
            QMutexLocker searchLocker(&m_searchFlagMutex);
            searchByRange(root->left_son_ptr, boxpoint, storage);
        }
        
        if ((m_rebuildPtr == nullptr) || root->right_son_ptr != *m_rebuildPtr) {
            searchByRange(root->right_son_ptr, boxpoint, storage);
        } else {
            QMutexLocker searchLocker(&m_searchFlagMutex);
            searchByRange(root->right_son_ptr, boxpoint, storage);
        }
    }

    void searchByRadius(KD_TREE_NODE* root, const PointType& point, double radius, PointVector& storage) {
        /**
         * @brief 按半径搜索实现
         */
        if (root == nullptr) return;
        
        pushDown(root);
        PointType rangeCenter;
        rangeCenter.x = (root->node_range_x[0] + root->node_range_x[1]) * 0.5;
        rangeCenter.y = (root->node_range_y[0] + root->node_range_y[1]) * 0.5;
        rangeCenter.z = (root->node_range_z[0] + root->node_range_z[1]) * 0.5;
        
        double dist = qSqrt(calcDist(rangeCenter, point));
        if (dist > radius + qSqrt(root->radius_sq)) return;
        
        if (dist <= radius - qSqrt(root->radius_sq)) {
            flatten(root, storage, NOT_RECORD);
            return;
        }
        
        if (!root->point_deleted && calcDist(root->point, point) <= radius * radius) {
            storage.append(root->point);
        }
        
        if ((m_rebuildPtr == nullptr) || root->left_son_ptr != *m_rebuildPtr) {
            searchByRadius(root->left_son_ptr, point, radius, storage);
        } else {
            QMutexLocker searchLocker(&m_searchFlagMutex);
            searchByRadius(root->left_son_ptr, point, radius, storage);
        }
        
        if ((m_rebuildPtr == nullptr) || root->right_son_ptr != *m_rebuildPtr) {
            searchByRadius(root->right_son_ptr, point, radius, storage);
        } else {
            QMutexLocker searchLocker(&m_searchFlagMutex);
            searchByRadius(root->right_son_ptr, point, radius, storage);
        }
    }

    void nearestSearch(const PointType& point, int kNearest, PointVector& nearestPoints, 
                      QVector<double>& pointDistance, double maxDist = std::numeric_limits<double>::infinity()) {
        /**
         * @brief K近邻搜索实现 - 搜索K个最近邻点
         */
        MANUAL_HEAP q(2 * kNearest);
        q.clear();
        pointDistance.clear();
        
        if (m_rebuildPtr == nullptr || *m_rebuildPtr != m_rootNode) {
            search(m_rootNode, kNearest, point, q, maxDist);
        } else {
            QMutexLocker searchLocker(&m_searchFlagMutex);
            while (m_searchMutexCounter.loadRelaxed() == -1) {
                searchLocker.unlock();
                QThread::usleep(1);
                searchLocker.relock();
            }
            m_searchMutexCounter.fetchAndAddRelaxed(1);
            searchLocker.unlock();
            
            search(m_rootNode, kNearest, point, q, maxDist);
            
            searchLocker.relock();
            m_searchMutexCounter.fetchAndSubRelaxed(1);
        }
        
        int kFound = qMin(kNearest, q.size());
        nearestPoints.clear();
        pointDistance.clear();
        
        for (int i = 0; i < kFound; i++) {
            nearestPoints.prepend(q.top().point);
            pointDistance.prepend(q.top().dist);
            q.pop();
        }
    }

    void search(KD_TREE_NODE* root, int kNearest, const PointType& point, MANUAL_HEAP& q, double maxDist) {
        /**
         * @brief K近邻搜索核心算法实现
         */
        if (root == nullptr || root->tree_deleted) return;
        
        double curDist = calcBoxDist(root, point);
        double maxDistSqr = maxDist * maxDist;
        if (curDist > maxDistSqr) return;
        
        if (root->need_push_down_to_left || root->need_push_down_to_right) {
            QMutexLocker pushLocker(&root->push_down_mutex);
            if (pushLocker.mutex()->tryLock()) {
                pushDown(root);
                pushLocker.unlock();
            } else {
                pushLocker.unlock();
                QMutexLocker waitLocker(&root->push_down_mutex);
            }
        }
        
        if (!root->point_deleted) {
            double dist = calcDist(point, root->point);
            if (dist <= maxDistSqr && (q.size() < kNearest || dist < q.top().dist)) {
                if (q.size() >= kNearest) q.pop();
                PointType_CMP currentPoint{root->point, dist};
                q.push(currentPoint);
            }
        }
        
        double distLeftNode = calcBoxDist(root->left_son_ptr, point);
        double distRightNode = calcBoxDist(root->right_son_ptr, point);
        
        if (q.size() < kNearest || distLeftNode < q.top().dist && distRightNode < q.top().dist) {
            if (distLeftNode <= distRightNode) {
                if (m_rebuildPtr == nullptr || *m_rebuildPtr != root->left_son_ptr) {
                    search(root->left_son_ptr, kNearest, point, q, maxDist);
                } else {
                    QMutexLocker searchLocker(&m_searchFlagMutex);
                    while (m_searchMutexCounter.loadRelaxed() == -1) {
                        searchLocker.unlock();
                        QThread::usleep(1);
                        searchLocker.relock();
                    }
                    m_searchMutexCounter.fetchAndAddRelaxed(1);
                    searchLocker.unlock();
                    search(root->left_son_ptr, kNearest, point, q, maxDist);
                    searchLocker.relock();
                    m_searchMutexCounter.fetchAndSubRelaxed(1);
                }
                
                if (q.size() < kNearest || distRightNode < q.top().dist) {
                    if (m_rebuildPtr == nullptr || *m_rebuildPtr != root->right_son_ptr) {
                        search(root->right_son_ptr, kNearest, point, q, maxDist);
                    } else {
                        QMutexLocker searchLocker(&m_searchFlagMutex);
                        while (m_searchMutexCounter.loadRelaxed() == -1) {
                            searchLocker.unlock();
                            QThread::usleep(1);
                            searchLocker.relock();
                        }
                        m_searchMutexCounter.fetchAndAddRelaxed(1);
                        searchLocker.unlock();
                        search(root->right_son_ptr, kNearest, point, q, maxDist);
                        searchLocker.relock();
                        m_searchMutexCounter.fetchAndSubRelaxed(1);
                    }
                }
            } else {
                if (m_rebuildPtr == nullptr || *m_rebuildPtr != root->right_son_ptr) {
                    search(root->right_son_ptr, kNearest, point, q, maxDist);
                } else {
                    QMutexLocker searchLocker(&m_searchFlagMutex);
                    while (m_searchMutexCounter.loadRelaxed() == -1) {
                        searchLocker.unlock();
                        QThread::usleep(1);
                        searchLocker.relock();
                    }
                    m_searchMutexCounter.fetchAndAddRelaxed(1);
                    searchLocker.unlock();
                    search(root->right_son_ptr, kNearest, point, q, maxDist);
                    searchLocker.relock();
                    m_searchMutexCounter.fetchAndSubRelaxed(1);
                }
                
                if (q.size() < kNearest || distLeftNode < q.top().dist) {
                    if (m_rebuildPtr == nullptr || *m_rebuildPtr != root->left_son_ptr) {
                        search(root->left_son_ptr, kNearest, point, q, maxDist);
                    } else {
                        QMutexLocker searchLocker(&m_searchFlagMutex);
                        while (m_searchMutexCounter.loadRelaxed() == -1) {
                            searchLocker.unlock();
                            QThread::usleep(1);
                            searchLocker.relock();
                        }
                        m_searchMutexCounter.fetchAndAddRelaxed(1);
                        searchLocker.unlock();
                        search(root->left_son_ptr, kNearest, point, q, maxDist);
                        searchLocker.relock();
                        m_searchMutexCounter.fetchAndSubRelaxed(1);
                    }
                }
            }
        } else {
            if (distLeftNode < q.top().dist) {
                if (m_rebuildPtr == nullptr || *m_rebuildPtr != root->left_son_ptr) {
                    search(root->left_son_ptr, kNearest, point, q, maxDist);
                } else {
                    QMutexLocker searchLocker(&m_searchFlagMutex);
                    while (m_searchMutexCounter.loadRelaxed() == -1) {
                        searchLocker.unlock();
                        QThread::usleep(1);
                        searchLocker.relock();
                    }
                    m_searchMutexCounter.fetchAndAddRelaxed(1);
                    searchLocker.unlock();
                    search(root->left_son_ptr, kNearest, point, q, maxDist);
                    searchLocker.relock();
                    m_searchMutexCounter.fetchAndSubRelaxed(1);
                }
            }
            if (distRightNode < q.top().dist) {
                if (m_rebuildPtr == nullptr || *m_rebuildPtr != root->right_son_ptr) {
                    search(root->right_son_ptr, kNearest, point, q, maxDist);
                } else {
                    QMutexLocker searchLocker(&m_searchFlagMutex);
                    while (m_searchMutexCounter.loadRelaxed() == -1) {
                        searchLocker.unlock();
                        QThread::usleep(1);
                        searchLocker.relock();
                    }
                    m_searchMutexCounter.fetchAndAddRelaxed(1);
                    searchLocker.unlock();
                    search(root->right_son_ptr, kNearest, point, q, maxDist);
                    searchLocker.relock();
                    m_searchMutexCounter.fetchAndSubRelaxed(1);
                }
            }
        }
    }

    void addByPoint(KD_TREE_NODE** root, const PointType& point, bool allowRebuild, int fatherAxis) {
        /**
         * @brief 按点添加实现 - 添加单个点到树中
         */
        if (*root == nullptr) {
            *root = new KD_TREE_NODE;
            initTreeNode(*root);
            (*root)->point = point;
            (*root)->division_axis = (fatherAxis + 1) % 3;
            update(*root);
            return;
        }
        
        (*root)->working_flag = true;
        Operation_Logger_Type addLog;
        addLog.op = ADD_POINT;
        addLog.point = point;
        pushDown(*root);
        
        // 根据分割轴决定添加方向
        bool goLeft = false;
        if ((*root)->division_axis == 0) {
            goLeft = (point.x < (*root)->point.x);
        } else if ((*root)->division_axis == 1) {
            goLeft = (point.y < (*root)->point.y);
        } else {
            goLeft = (point.z < (*root)->point.z);
        }
        
        if (goLeft) {
            if ((m_rebuildPtr == nullptr) || (*root)->left_son_ptr != *m_rebuildPtr) {
                addByPoint(&(*root)->left_son_ptr, point, allowRebuild, (*root)->division_axis);
            } else {
                QMutexLocker workingLocker(&m_workingFlagMutex);
                addByPoint(&(*root)->left_son_ptr, point, false, (*root)->division_axis);
                if (m_rebuildFlag.loadRelaxed()) {
                    QMutexLocker loggerLocker(&m_rebuildLoggerMutex);
                    m_rebuildLogger.push(addLog);
                }
            }
        } else {
            if ((m_rebuildPtr == nullptr) || (*root)->right_son_ptr != *m_rebuildPtr) {
                addByPoint(&(*root)->right_son_ptr, point, allowRebuild, (*root)->division_axis);
            } else {
                QMutexLocker workingLocker(&m_workingFlagMutex);
                addByPoint(&(*root)->right_son_ptr, point, false, (*root)->division_axis);
                if (m_rebuildFlag.loadRelaxed()) {
                    QMutexLocker loggerLocker(&m_rebuildLoggerMutex);
                    m_rebuildLogger.push(addLog);
                }
            }
        }
        
        update(*root);
        
        if (m_rebuildPtr != nullptr && *m_rebuildPtr == *root && 
            (*root)->TreeSize < MULTI_THREAD_REBUILD_POINT_NUM) {
            m_rebuildPtr = nullptr;
        }
        
        bool needRebuild = allowRebuild && criterionCheck(*root);
        if (needRebuild) {
            rebuild(root);
        }
        
        if ((*root) != nullptr) {
            (*root)->working_flag = false;
        }
    }

    void deleteByPoint(KD_TREE_NODE** root, const PointType& point, bool allowRebuild) {
        /**
         * @brief 按点删除实现 - 删除指定点
         */
        if ((*root) == nullptr || (*root)->tree_deleted) return;
        
        (*root)->working_flag = true;
        pushDown(*root);
        
        if (samePoint((*root)->point, point) && !(*root)->point_deleted) {
            (*root)->point_deleted = true;
            (*root)->invalid_point_num += 1;
            if ((*root)->invalid_point_num == (*root)->TreeSize) {
                (*root)->tree_deleted = true;
            }
            return;
        }
        
        Operation_Logger_Type deleteLog;
        deleteLog.op = DELETE_POINT;
        deleteLog.point = point;
        
        // 根据分割轴决定搜索方向
        bool goLeft = false;
        if ((*root)->division_axis == 0) {
            goLeft = (point.x < (*root)->point.x);
        } else if ((*root)->division_axis == 1) {
            goLeft = (point.y < (*root)->point.y);
        } else {
            goLeft = (point.z < (*root)->point.z);
        }
        
        if (goLeft) {
            if ((m_rebuildPtr == nullptr) || (*root)->left_son_ptr != *m_rebuildPtr) {
                deleteByPoint(&(*root)->left_son_ptr, point, allowRebuild);
            } else {
                QMutexLocker workingLocker(&m_workingFlagMutex);
                deleteByPoint(&(*root)->left_son_ptr, point, false);
                if (m_rebuildFlag.loadRelaxed()) {
                    QMutexLocker loggerLocker(&m_rebuildLoggerMutex);
                    m_rebuildLogger.push(deleteLog);
                }
            }
        } else {
            if ((m_rebuildPtr == nullptr) || (*root)->right_son_ptr != *m_rebuildPtr) {
                deleteByPoint(&(*root)->right_son_ptr, point, allowRebuild);
            } else {
                QMutexLocker workingLocker(&m_workingFlagMutex);
                deleteByPoint(&(*root)->right_son_ptr, point, false);
                if (m_rebuildFlag.loadRelaxed()) {
                    QMutexLocker loggerLocker(&m_rebuildLoggerMutex);
                    m_rebuildLogger.push(deleteLog);
                }
            }
        }
        
        update(*root);
        
        if (m_rebuildPtr != nullptr && *m_rebuildPtr == *root && 
            (*root)->TreeSize < MULTI_THREAD_REBUILD_POINT_NUM) {
            m_rebuildPtr = nullptr;
        }
        
        bool needRebuild = allowRebuild && criterionCheck(*root);
        if (needRebuild) {
            rebuild(root);
        }
        
        if ((*root) != nullptr) {
            (*root)->working_flag = false;
        }
    }

    int deleteByRange(KD_TREE_NODE** root, const BoxPointType& boxpoint, bool allowRebuild, bool isDownsample) {
        /**
         * @brief 按包围盒删除实现 - 删除指定范围内的点
         */
        if ((*root) == nullptr || (*root)->tree_deleted) return 0;
        
        (*root)->working_flag = true;
        pushDown(*root);
        
        int tmpCounter = 0;
        
        // 检查包围盒是否与节点范围相交
        if (boxpoint.vertex_max[0] <= (*root)->node_range_x[0] || boxpoint.vertex_min[0] > (*root)->node_range_x[1]) return 0;
        if (boxpoint.vertex_max[1] <= (*root)->node_range_y[0] || boxpoint.vertex_min[1] > (*root)->node_range_y[1]) return 0;
        if (boxpoint.vertex_max[2] <= (*root)->node_range_z[0] || boxpoint.vertex_min[2] > (*root)->node_range_z[1]) return 0;
        
        // 如果包围盒完全包含节点范围，删除整个子树
        if (boxpoint.vertex_min[0] <= (*root)->node_range_x[0] && boxpoint.vertex_max[0] > (*root)->node_range_x[1] && 
            boxpoint.vertex_min[1] <= (*root)->node_range_y[0] && boxpoint.vertex_max[1] > (*root)->node_range_y[1] && 
            boxpoint.vertex_min[2] <= (*root)->node_range_z[0] && boxpoint.vertex_max[2] > (*root)->node_range_z[1]) {
            
            (*root)->tree_deleted = true;
            (*root)->point_deleted = true;
            (*root)->need_push_down_to_left = true;
            (*root)->need_push_down_to_right = true;
            tmpCounter = (*root)->TreeSize - (*root)->invalid_point_num;
            (*root)->invalid_point_num = (*root)->TreeSize;
            
            if (isDownsample) {
                (*root)->tree_downsample_deleted = true;
                (*root)->point_downsample_deleted = true;
                (*root)->down_del_num = (*root)->TreeSize;
            }
            return tmpCounter;
        }
        
        // 检查当前节点的点是否在包围盒内
        if (!(*root)->point_deleted && 
            boxpoint.vertex_min[0] <= (*root)->point.x && boxpoint.vertex_max[0] > (*root)->point.x && 
            boxpoint.vertex_min[1] <= (*root)->point.y && boxpoint.vertex_max[1] > (*root)->point.y && 
            boxpoint.vertex_min[2] <= (*root)->point.z && boxpoint.vertex_max[2] > (*root)->point.z) {
            
            (*root)->point_deleted = true;
            tmpCounter += 1;
            if (isDownsample) (*root)->point_downsample_deleted = true;
        }
        
        Operation_Logger_Type deleteBoxLog;
        if (isDownsample) {
            deleteBoxLog.op = DOWNSAMPLE_DELETE;
        } else {
            deleteBoxLog.op = DELETE_BOX;
        }
        deleteBoxLog.boxpoint = boxpoint;
        
        // 递归处理左子树
        if ((m_rebuildPtr == nullptr) || (*root)->left_son_ptr != *m_rebuildPtr) {
            tmpCounter += deleteByRange(&((*root)->left_son_ptr), boxpoint, allowRebuild, isDownsample);
        } else {
            QMutexLocker workingLocker(&m_workingFlagMutex);
            tmpCounter += deleteByRange(&((*root)->left_son_ptr), boxpoint, false, isDownsample);
            if (m_rebuildFlag.loadRelaxed()) {
                QMutexLocker loggerLocker(&m_rebuildLoggerMutex);
                m_rebuildLogger.push(deleteBoxLog);
            }
        }
        
        // 递归处理右子树
        if ((m_rebuildPtr == nullptr) || (*root)->right_son_ptr != *m_rebuildPtr) {
            tmpCounter += deleteByRange(&((*root)->right_son_ptr), boxpoint, allowRebuild, isDownsample);
        } else {
            QMutexLocker workingLocker(&m_workingFlagMutex);
            tmpCounter += deleteByRange(&((*root)->right_son_ptr), boxpoint, false, isDownsample);
            if (m_rebuildFlag.loadRelaxed()) {
                QMutexLocker loggerLocker(&m_rebuildLoggerMutex);
                m_rebuildLogger.push(deleteBoxLog);
            }
        }
        
        update(*root);
        
        if (m_rebuildPtr != nullptr && *m_rebuildPtr == *root && 
            (*root)->TreeSize < MULTI_THREAD_REBUILD_POINT_NUM) {
            m_rebuildPtr = nullptr;
        }
        
        bool needRebuild = allowRebuild && criterionCheck(*root);
        if (needRebuild) {
            rebuild(root);
        }
        
        if ((*root) != nullptr) {
            (*root)->working_flag = false;
        }
        
        return tmpCounter;
    }

    void addByRange(KD_TREE_NODE** root, const BoxPointType& boxpoint, bool allowRebuild) {
        /**
         * @brief 按包围盒添加实现 - 恢复指定范围内的点
         */
        if ((*root) == nullptr) return;
        
        (*root)->working_flag = true;
        pushDown(*root);
        
        // 检查包围盒是否与节点范围相交
        if (boxpoint.vertex_max[0] <= (*root)->node_range_x[0] || boxpoint.vertex_min[0] > (*root)->node_range_x[1]) return;
        if (boxpoint.vertex_max[1] <= (*root)->node_range_y[0] || boxpoint.vertex_min[1] > (*root)->node_range_y[1]) return;
        if (boxpoint.vertex_max[2] <= (*root)->node_range_z[0] || boxpoint.vertex_min[2] > (*root)->node_range_z[1]) return;
        
        // 如果包围盒完全包含节点范围，恢复整个子树
        if (boxpoint.vertex_min[0] <= (*root)->node_range_x[0] && boxpoint.vertex_max[0] > (*root)->node_range_x[1] && 
            boxpoint.vertex_min[1] <= (*root)->node_range_y[0] && boxpoint.vertex_max[1] > (*root)->node_range_y[1] && 
            boxpoint.vertex_min[2] <= (*root)->node_range_z[0] && boxpoint.vertex_max[2] > (*root)->node_range_z[1]) {
            
            (*root)->tree_deleted = false || (*root)->tree_downsample_deleted;
            (*root)->point_deleted = false || (*root)->point_downsample_deleted;
            (*root)->need_push_down_to_left = true;
            (*root)->need_push_down_to_right = true;
            (*root)->invalid_point_num = (*root)->down_del_num;
            return;
        }
        
        // 检查当前节点的点是否在包围盒内
        if (boxpoint.vertex_min[0] <= (*root)->point.x && boxpoint.vertex_max[0] > (*root)->point.x && 
            boxpoint.vertex_min[1] <= (*root)->point.y && boxpoint.vertex_max[1] > (*root)->point.y && 
            boxpoint.vertex_min[2] <= (*root)->point.z && boxpoint.vertex_max[2] > (*root)->point.z) {
            
            (*root)->point_deleted = (*root)->point_downsample_deleted;
        }
        
        Operation_Logger_Type addBoxLog;
        addBoxLog.op = ADD_BOX;
        addBoxLog.boxpoint = boxpoint;
        
        // 递归处理左子树
        if ((m_rebuildPtr == nullptr) || (*root)->left_son_ptr != *m_rebuildPtr) {
            addByRange(&((*root)->left_son_ptr), boxpoint, allowRebuild);
        } else {
            QMutexLocker workingLocker(&m_workingFlagMutex);
            addByRange(&((*root)->left_son_ptr), boxpoint, false);
            if (m_rebuildFlag.loadRelaxed()) {
                QMutexLocker loggerLocker(&m_rebuildLoggerMutex);
                m_rebuildLogger.push(addBoxLog);
            }
        }
        
        // 递归处理右子树
        if ((m_rebuildPtr == nullptr) || (*root)->right_son_ptr != *m_rebuildPtr) {
            addByRange(&((*root)->right_son_ptr), boxpoint, allowRebuild);
        } else {
            QMutexLocker workingLocker(&m_workingFlagMutex);
            addByRange(&((*root)->right_son_ptr), boxpoint, false);
            if (m_rebuildFlag.loadRelaxed()) {
                QMutexLocker loggerLocker(&m_rebuildLoggerMutex);
                m_rebuildLogger.push(addBoxLog);
            }
        }
        
        update(*root);
        
        if (m_rebuildPtr != nullptr && *m_rebuildPtr == *root && 
            (*root)->TreeSize < MULTI_THREAD_REBUILD_POINT_NUM) {
            m_rebuildPtr = nullptr;
        }
        
        bool needRebuild = allowRebuild && criterionCheck(*root);
        if (needRebuild) {
            rebuild(root);
        }
        
        if ((*root) != nullptr) {
            (*root)->working_flag = false;
        }
    }

    void rebuild(KD_TREE_NODE** root) {
        /**
         * @brief 重建子树实现 - 重新平衡树结构以提高性能
         */
        if (!root || !*root) return;
        
        // 如果子树太小，不需要重建
        if ((*root)->TreeSize < Minimal_Unbalanced_Tree_Size) return;
        
        // 收集所有有效点
        PointVector storage;
        flatten(*root, storage, NOT_RECORD);
        
        // 删除原有节点
        deleteTreeNodes(root);
        
        // 重新构建平衡的树
        if (!storage.empty()) {
            buildTree(root, 0, storage.size() - 1, storage);
            if (*root) {
                update(*root);
            }
        }
    }

    void runOperation(KD_TREE_NODE** root, const Operation_Logger_Type& operation) {
        switch (operation.op) {
        case ADD_POINT:      
            addByPoint(root, operation.point, false, (*root) ? (*root)->division_axis : 0);          
            break;
        case ADD_BOX:
            addByRange(root, operation.boxpoint, false);
            break;
        case DELETE_POINT:
            deleteByPoint(root, operation.point, false);
            break;
        case DELETE_BOX:
            deleteByRange(root, operation.boxpoint, false, false);
            break;
        case DOWNSAMPLE_DELETE:
            deleteByRange(root, operation.boxpoint, false, true);
            break;
        case PUSH_DOWN:
            (*root)->tree_downsample_deleted |= operation.tree_downsample_deleted;
            (*root)->point_downsample_deleted |= operation.tree_downsample_deleted;
            (*root)->tree_deleted = operation.tree_deleted || (*root)->tree_downsample_deleted;
            (*root)->point_deleted = (*root)->tree_deleted || (*root)->point_downsample_deleted;
            if (operation.tree_downsample_deleted) (*root)->down_del_num = (*root)->TreeSize;
            if (operation.tree_deleted) (*root)->invalid_point_num = (*root)->TreeSize;
                else (*root)->invalid_point_num = (*root)->down_del_num;
            (*root)->need_push_down_to_left = true;
            (*root)->need_push_down_to_right = true;
            break;
        default:
            break;
        }
    }

    bool criterionCheck(KD_TREE_NODE* root) {
        /**
         * @brief 检查是否需要重建判据实现 - 判断树是否需要重建以保持平衡
         */
        if (root->TreeSize <= Minimal_Unbalanced_Tree_Size) {
            return false;
        }
        
        double balanceEvaluation = 0.0;
        double deleteEvaluation = 0.0;
        KD_TREE_NODE* sonPtr = root->left_son_ptr;
        if (sonPtr == nullptr) sonPtr = root->right_son_ptr;
        
        deleteEvaluation = double(root->invalid_point_num) / root->TreeSize;
        balanceEvaluation = double(sonPtr->TreeSize) / (root->TreeSize - 1);
        
        if (deleteEvaluation > m_deleteCriterionParam) {
            return true;
        }
        
        if (balanceEvaluation > m_balanceCriterionParam || balanceEvaluation < 1 - m_balanceCriterionParam) {
            return true;
        }
        
        return false;
    }

    void pushDown(KD_TREE_NODE* root) {
        /**
         * @brief 下推操作实现 - 将删除标记下推到子节点
         */
        if (root == nullptr) return;
        
        Operation_Logger_Type operation;
        operation.op = PUSH_DOWN;
        operation.tree_deleted = root->tree_deleted;
        operation.tree_downsample_deleted = root->tree_downsample_deleted;
        
        if (root->need_push_down_to_left && root->left_son_ptr != nullptr) {
            if (m_rebuildPtr == nullptr || *m_rebuildPtr != root->left_son_ptr) {
                root->left_son_ptr->tree_downsample_deleted |= root->tree_downsample_deleted;
                root->left_son_ptr->point_downsample_deleted |= root->tree_downsample_deleted;
                root->left_son_ptr->tree_deleted = root->tree_deleted || root->left_son_ptr->tree_downsample_deleted;
                root->left_son_ptr->point_deleted = root->left_son_ptr->tree_deleted || root->left_son_ptr->point_downsample_deleted;
                if (root->tree_downsample_deleted) root->left_son_ptr->down_del_num = root->left_son_ptr->TreeSize;
                if (root->tree_deleted) root->left_son_ptr->invalid_point_num = root->left_son_ptr->TreeSize;
                    else root->left_son_ptr->invalid_point_num = root->left_son_ptr->down_del_num;
                root->left_son_ptr->need_push_down_to_left = true;
                root->left_son_ptr->need_push_down_to_right = true;
                root->need_push_down_to_left = false;
            } else {
                QMutexLocker locker(&m_workingFlagMutex);
                root->left_son_ptr->tree_downsample_deleted |= root->tree_downsample_deleted;
                root->left_son_ptr->point_downsample_deleted |= root->tree_downsample_deleted;
                root->left_son_ptr->tree_deleted = root->tree_deleted || root->left_son_ptr->tree_downsample_deleted;
                root->left_son_ptr->point_deleted = root->left_son_ptr->tree_deleted || root->left_son_ptr->point_downsample_deleted;
                if (root->tree_downsample_deleted) root->left_son_ptr->down_del_num = root->left_son_ptr->TreeSize;
                if (root->tree_deleted) root->left_son_ptr->invalid_point_num = root->left_son_ptr->TreeSize;
                    else root->left_son_ptr->invalid_point_num = root->left_son_ptr->down_del_num;
                root->left_son_ptr->need_push_down_to_left = true;
                root->left_son_ptr->need_push_down_to_right = true;
                if (m_rebuildFlag.loadRelaxed()) {
                    QMutexLocker loggerLocker(&m_rebuildLoggerMutex);
                    m_rebuildLogger.push(operation);
                }
                root->need_push_down_to_left = false;
            }
        }
        
        if (root->need_push_down_to_right && root->right_son_ptr != nullptr) {
            if (m_rebuildPtr == nullptr || *m_rebuildPtr != root->right_son_ptr) {
                root->right_son_ptr->tree_downsample_deleted |= root->tree_downsample_deleted;
                root->right_son_ptr->point_downsample_deleted |= root->tree_downsample_deleted;
                root->right_son_ptr->tree_deleted = root->tree_deleted || root->right_son_ptr->tree_downsample_deleted;
                root->right_son_ptr->point_deleted = root->right_son_ptr->tree_deleted || root->right_son_ptr->point_downsample_deleted;
                if (root->tree_downsample_deleted) root->right_son_ptr->down_del_num = root->right_son_ptr->TreeSize;
                if (root->tree_deleted) root->right_son_ptr->invalid_point_num = root->right_son_ptr->TreeSize;
                    else root->right_son_ptr->invalid_point_num = root->right_son_ptr->down_del_num;
                root->right_son_ptr->need_push_down_to_left = true;
                root->right_son_ptr->need_push_down_to_right = true;
                root->need_push_down_to_right = false;
            } else {
                QMutexLocker locker(&m_workingFlagMutex);
                root->right_son_ptr->tree_downsample_deleted |= root->tree_downsample_deleted;
                root->right_son_ptr->point_downsample_deleted |= root->tree_downsample_deleted;
                root->right_son_ptr->tree_deleted = root->tree_deleted || root->right_son_ptr->tree_downsample_deleted;
                root->right_son_ptr->point_deleted = root->right_son_ptr->tree_deleted || root->right_son_ptr->point_downsample_deleted;
                if (root->tree_downsample_deleted) root->right_son_ptr->down_del_num = root->right_son_ptr->TreeSize;
                if (root->tree_deleted) root->right_son_ptr->invalid_point_num = root->right_son_ptr->TreeSize;
                    else root->right_son_ptr->invalid_point_num = root->right_son_ptr->down_del_num;
                root->right_son_ptr->need_push_down_to_left = true;
                root->right_son_ptr->need_push_down_to_right = true;
                if (m_rebuildFlag.loadRelaxed()) {
                    QMutexLocker loggerLocker(&m_rebuildLoggerMutex);
                    m_rebuildLogger.push(operation);
                }
                root->need_push_down_to_right = false;
            }
        }
    }

    void update(KD_TREE_NODE* root) {
        /**
         * @brief 更新节点信息实现 - 更新统计信息和包围盒
         */
        if (!root) return;
        
        KD_TREE_NODE* leftSonPtr = root->left_son_ptr;
        KD_TREE_NODE* rightSonPtr = root->right_son_ptr;
        double tmpRangeX[2] = {std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()};
        double tmpRangeY[2] = {std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()};
        double tmpRangeZ[2] = {std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()};
        
        // 更新树大小
        if (leftSonPtr != nullptr && rightSonPtr != nullptr) {
            root->TreeSize = leftSonPtr->TreeSize + rightSonPtr->TreeSize + 1;
            root->invalid_point_num = leftSonPtr->invalid_point_num + rightSonPtr->invalid_point_num + (root->point_deleted ? 1 : 0);
            root->down_del_num = leftSonPtr->down_del_num + rightSonPtr->down_del_num + (root->point_downsample_deleted ? 1 : 0);
            root->tree_downsample_deleted = leftSonPtr->tree_downsample_deleted && rightSonPtr->tree_downsample_deleted && root->point_downsample_deleted;
            root->tree_deleted = leftSonPtr->tree_deleted && rightSonPtr->tree_deleted && root->point_deleted;
            
            if (root->tree_deleted || (!leftSonPtr->tree_deleted && !rightSonPtr->tree_deleted && !root->point_deleted)) {
                tmpRangeX[0] = qMin(qMin(leftSonPtr->node_range_x[0], rightSonPtr->node_range_x[0]), root->point.x);
                tmpRangeX[1] = qMax(qMax(leftSonPtr->node_range_x[1], rightSonPtr->node_range_x[1]), root->point.x);
                tmpRangeY[0] = qMin(qMin(leftSonPtr->node_range_y[0], rightSonPtr->node_range_y[0]), root->point.y);
                tmpRangeY[1] = qMax(qMax(leftSonPtr->node_range_y[1], rightSonPtr->node_range_y[1]), root->point.y);
                tmpRangeZ[0] = qMin(qMin(leftSonPtr->node_range_z[0], rightSonPtr->node_range_z[0]), root->point.z);
                tmpRangeZ[1] = qMax(qMax(leftSonPtr->node_range_z[1], rightSonPtr->node_range_z[1]), root->point.z);
            } else {
                if (!leftSonPtr->tree_deleted) {
                    tmpRangeX[0] = qMin(tmpRangeX[0], leftSonPtr->node_range_x[0]);
                    tmpRangeX[1] = qMax(tmpRangeX[1], leftSonPtr->node_range_x[1]);
                    tmpRangeY[0] = qMin(tmpRangeY[0], leftSonPtr->node_range_y[0]);
                    tmpRangeY[1] = qMax(tmpRangeY[1], leftSonPtr->node_range_y[1]);
                    tmpRangeZ[0] = qMin(tmpRangeZ[0], leftSonPtr->node_range_z[0]);
                    tmpRangeZ[1] = qMax(tmpRangeZ[1], leftSonPtr->node_range_z[1]);
                }
                if (!rightSonPtr->tree_deleted) {
                    tmpRangeX[0] = qMin(tmpRangeX[0], rightSonPtr->node_range_x[0]);
                    tmpRangeX[1] = qMax(tmpRangeX[1], rightSonPtr->node_range_x[1]);
                    tmpRangeY[0] = qMin(tmpRangeY[0], rightSonPtr->node_range_y[0]);
                    tmpRangeY[1] = qMax(tmpRangeY[1], rightSonPtr->node_range_y[1]);
                    tmpRangeZ[0] = qMin(tmpRangeZ[0], rightSonPtr->node_range_z[0]);
                    tmpRangeZ[1] = qMax(tmpRangeZ[1], rightSonPtr->node_range_z[1]);
                }
                if (!root->point_deleted) {
                    tmpRangeX[0] = qMin(tmpRangeX[0], root->point.x);
                    tmpRangeX[1] = qMax(tmpRangeX[1], root->point.x);
                    tmpRangeY[0] = qMin(tmpRangeY[0], root->point.y);
                    tmpRangeY[1] = qMax(tmpRangeY[1], root->point.y);
                    tmpRangeZ[0] = qMin(tmpRangeZ[0], root->point.z);
                    tmpRangeZ[1] = qMax(tmpRangeZ[1], root->point.z);
                }
            }
        } else if (leftSonPtr != nullptr) {
            root->TreeSize = leftSonPtr->TreeSize + 1;
            root->invalid_point_num = leftSonPtr->invalid_point_num + (root->point_deleted ? 1 : 0);
            root->down_del_num = leftSonPtr->down_del_num + (root->point_downsample_deleted ? 1 : 0);
            root->tree_downsample_deleted = leftSonPtr->tree_downsample_deleted && root->point_downsample_deleted;
            root->tree_deleted = leftSonPtr->tree_deleted && root->point_deleted;
            
            if (root->tree_deleted || (!leftSonPtr->tree_deleted && !root->point_deleted)) {
                tmpRangeX[0] = qMin(leftSonPtr->node_range_x[0], root->point.x);
                tmpRangeX[1] = qMax(leftSonPtr->node_range_x[1], root->point.x);
                tmpRangeY[0] = qMin(leftSonPtr->node_range_y[0], root->point.y);
                tmpRangeY[1] = qMax(leftSonPtr->node_range_y[1], root->point.y);
                tmpRangeZ[0] = qMin(leftSonPtr->node_range_z[0], root->point.z);
                tmpRangeZ[1] = qMax(leftSonPtr->node_range_z[1], root->point.z);
            } else {
                if (!leftSonPtr->tree_deleted) {
                    tmpRangeX[0] = qMin(tmpRangeX[0], leftSonPtr->node_range_x[0]);
                    tmpRangeX[1] = qMax(tmpRangeX[1], leftSonPtr->node_range_x[1]);
                    tmpRangeY[0] = qMin(tmpRangeY[0], leftSonPtr->node_range_y[0]);
                    tmpRangeY[1] = qMax(tmpRangeY[1], leftSonPtr->node_range_y[1]);
                    tmpRangeZ[0] = qMin(tmpRangeZ[0], leftSonPtr->node_range_z[0]);
                    tmpRangeZ[1] = qMax(tmpRangeZ[1], leftSonPtr->node_range_z[1]);
                }
                if (!root->point_deleted) {
                    tmpRangeX[0] = qMin(tmpRangeX[0], root->point.x);
                    tmpRangeX[1] = qMax(tmpRangeX[1], root->point.x);
                    tmpRangeY[0] = qMin(tmpRangeY[0], root->point.y);
                    tmpRangeY[1] = qMax(tmpRangeY[1], root->point.y);
                    tmpRangeZ[0] = qMin(tmpRangeZ[0], root->point.z);
                    tmpRangeZ[1] = qMax(tmpRangeZ[1], root->point.z);
                }
            }
        } else if (rightSonPtr != nullptr) {
            root->TreeSize = rightSonPtr->TreeSize + 1;
            root->invalid_point_num = rightSonPtr->invalid_point_num + (root->point_deleted ? 1 : 0);
            root->down_del_num = rightSonPtr->down_del_num + (root->point_downsample_deleted ? 1 : 0);
            root->tree_downsample_deleted = rightSonPtr->tree_downsample_deleted && root->point_downsample_deleted;
            root->tree_deleted = rightSonPtr->tree_deleted && root->point_deleted;
            
            if (root->tree_deleted || (!rightSonPtr->tree_deleted && !root->point_deleted)) {
                tmpRangeX[0] = qMin(rightSonPtr->node_range_x[0], root->point.x);
                tmpRangeX[1] = qMax(rightSonPtr->node_range_x[1], root->point.x);
                tmpRangeY[0] = qMin(rightSonPtr->node_range_y[0], root->point.y);
                tmpRangeY[1] = qMax(rightSonPtr->node_range_y[1], root->point.y);
                tmpRangeZ[0] = qMin(rightSonPtr->node_range_z[0], root->point.z);
                tmpRangeZ[1] = qMax(rightSonPtr->node_range_z[1], root->point.z);
            } else {
                if (!rightSonPtr->tree_deleted) {
                    tmpRangeX[0] = qMin(tmpRangeX[0], rightSonPtr->node_range_x[0]);
                    tmpRangeX[1] = qMax(tmpRangeX[1], rightSonPtr->node_range_x[1]);
                    tmpRangeY[0] = qMin(tmpRangeY[0], rightSonPtr->node_range_y[0]);
                    tmpRangeY[1] = qMax(tmpRangeY[1], rightSonPtr->node_range_y[1]);
                    tmpRangeZ[0] = qMin(tmpRangeZ[0], rightSonPtr->node_range_z[0]);
                    tmpRangeZ[1] = qMax(tmpRangeZ[1], rightSonPtr->node_range_z[1]);
                }
                if (!root->point_deleted) {
                    tmpRangeX[0] = qMin(tmpRangeX[0], root->point.x);
                    tmpRangeX[1] = qMax(tmpRangeX[1], root->point.x);
                    tmpRangeY[0] = qMin(tmpRangeY[0], root->point.y);
                    tmpRangeY[1] = qMax(tmpRangeY[1], root->point.y);
                    tmpRangeZ[0] = qMin(tmpRangeZ[0], root->point.z);
                    tmpRangeZ[1] = qMax(tmpRangeZ[1], root->point.z);
                }
            }
        } else {
            root->TreeSize = 1;
            root->invalid_point_num = (root->point_deleted ? 1 : 0);
            root->down_del_num = (root->point_downsample_deleted ? 1 : 0);
            root->tree_downsample_deleted = root->point_downsample_deleted;
            root->tree_deleted = root->point_deleted;
            tmpRangeX[0] = root->point.x;
            tmpRangeX[1] = root->point.x;        
            tmpRangeY[0] = root->point.y;
            tmpRangeY[1] = root->point.y; 
            tmpRangeZ[0] = root->point.z;
            tmpRangeZ[1] = root->point.z;                 
        }
        
        // 复制范围数据
        memcpy(root->node_range_x, tmpRangeX, sizeof(tmpRangeX));
        memcpy(root->node_range_y, tmpRangeY, sizeof(tmpRangeY));
        memcpy(root->node_range_z, tmpRangeZ, sizeof(tmpRangeZ));
        
        double xL = (root->node_range_x[1] - root->node_range_x[0]) * 0.5;
        double yL = (root->node_range_y[1] - root->node_range_y[0]) * 0.5;
        double zL = (root->node_range_z[1] - root->node_range_z[0]) * 0.5;
        root->radius_sq = xL*xL + yL*yL + zL*zL;
        
        if (leftSonPtr != nullptr) leftSonPtr->father_ptr = root;
        if (rightSonPtr != nullptr) rightSonPtr->father_ptr = root;
        
        if (root == m_rootNode && root->TreeSize > 3) {
            KD_TREE_NODE* sonPtr = root->left_son_ptr;
            if (sonPtr == nullptr) sonPtr = root->right_son_ptr;
            double tmpBal = double(sonPtr->TreeSize) / (root->TreeSize - 1);
            root->alpha_del = double(root->invalid_point_num) / root->TreeSize;
            root->alpha_bal = (tmpBal >= 0.5 - EPSS) ? tmpBal : 1 - tmpBal;
        }
    }

    void deleteTreeNodes(KD_TREE_NODE** root) {
        /**
         * @brief 删除树节点实现 - 递归删除整个子树
         */
        if (*root == nullptr) return;
        
        pushDown(*root);
        deleteTreeNodes(&(*root)->left_son_ptr);
        deleteTreeNodes(&(*root)->right_son_ptr);
        
        delete *root;
        *root = nullptr;
    }

    bool samePoint(const PointType& a, const PointType& b) const {
        /**
         * @brief 判断点是否相同实现
         */
        return (qAbs(a.x - b.x) < EPSS && qAbs(a.y - b.y) < EPSS && qAbs(a.z - b.z) < EPSS);
    }

    double calcDist(const PointType& a, const PointType& b) const {
        /**
         * @brief 计算点间距离实现
         */
        double dist = (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) + (a.z - b.z) * (a.z - b.z);
        return dist;
    }

    double calcBoxDist(const KD_TREE_NODE* node, const PointType& point) const {
        /**
         * @brief 计算点到包围盒距离实现
         */
        if (node == nullptr) return std::numeric_limits<double>::infinity();
        
        double minDist = 0.0;
        if (point.x < node->node_range_x[0]) minDist += (point.x - node->node_range_x[0]) * (point.x - node->node_range_x[0]);
        if (point.x > node->node_range_x[1]) minDist += (point.x - node->node_range_x[1]) * (point.x - node->node_range_x[1]);
        if (point.y < node->node_range_y[0]) minDist += (point.y - node->node_range_y[0]) * (point.y - node->node_range_y[0]);
        if (point.y > node->node_range_y[1]) minDist += (point.y - node->node_range_y[1]) * (point.y - node->node_range_y[1]);
        if (point.z < node->node_range_z[0]) minDist += (point.z - node->node_range_z[0]) * (point.z - node->node_range_z[0]);
        if (point.z > node->node_range_z[1]) minDist += (point.z - node->node_range_z[1]) * (point.z - node->node_range_z[1]);
        return minDist;
    }

    static bool pointCmpX(const PointType& a, const PointType& b) {
        return a.x < b.x;
    }

    static bool pointCmpY(const PointType& a, const PointType& b) {
        return a.y < b.y;
    }

    static bool pointCmpZ(const PointType& a, const PointType& b) {
        return a.z < b.z;
    }
};

// 为兼容性提供的类型别名
using IkdTree = KD_TREE<DefaultPointType>;
using IkdTreePtr = QSharedPointer<IkdTree>;
