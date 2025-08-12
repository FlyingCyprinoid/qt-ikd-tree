# ikd-Tree Qt版本

这是ikd-Tree（增量式K-D树）的纯Qt实现版本，专为机器人应用和点云处理设计。

## 主要特性

- ✅ **完全Qt实现** - 移除所有pthread、STL、PCL依赖
- ✅ **高性能** - 保持原始ikd-Tree算法的所有性能优化
- ✅ **线程安全** - 使用Qt的QThread、QMutex、QAtomicInt
- ✅ **内存安全** - 使用Qt智能指针和RAII模式
- ✅ **跨平台** - 基于Qt6，支持Windows、Linux、macOS
- ✅ **中文注释** - 完整的中文函数级注释

## 文件结构

```
ikd-Tree/
├── ikd_Tree_qt.h        # Qt版本头文件
├── ikd_Tree_qt.cpp      # Qt版本实现文件
├── ikd-Tree.pro         # 库工程文件
├── example_qt.cpp       # 使用示例
├── example.pro          # 示例工程文件
└── README_Qt.md         # 本说明文档
```

## 环境要求

- Qt 6.0 或更高版本
- C++17 编译器
- qmake 构建系统

## 编译方法

### 编译库文件

```bash
# 进入项目目录
cd ikd-Tree

# 生成Makefile
qmake ikd-Tree.pro

# 编译
make
```

### 编译和运行示例

```bash
# 编译示例
qmake example.pro
make

# 运行示例（Windows）
./debug/ikd-tree-example.exe

# 运行示例（Linux/macOS）
./ikd-tree-example
```

## 使用方法

### 基础用法

```cpp
#include "ikd_Tree_qt.h"
#include <QVector>

int main()
{
    // 创建ikd-Tree实例
    auto kdTree = new KD_TREE<ikdTree_PointType>();
    
    // 准备点云数据
    QVector<ikdTree_PointType> pointCloud;
    pointCloud.append(ikdTree_PointType(1.0f, 2.0f, 3.0f));
    pointCloud.append(ikdTree_PointType(4.0f, 5.0f, 6.0f));
    // ... 添加更多点
    
    // 构建K-D树
    kdTree->build(pointCloud);
    
    // K近邻搜索
    ikdTree_PointType queryPoint(0.0f, 0.0f, 0.0f);
    QVector<ikdTree_PointType> nearestPoints;
    QVector<float> distances;
    kdTree->nearestSearch(queryPoint, 5, nearestPoints, distances);
    
    // 清理资源
    delete kdTree;
    return 0;
}
```

### 高级功能

#### 1. 半径搜索

```cpp
QVector<ikdTree_PointType> radiusPoints;
float radius = 2.0f;
kdTree->radiusSearch(queryPoint, radius, radiusPoints);
```

#### 2. 包围盒搜索

```cpp
BoxPointType searchBox;
searchBox.vertex_min[0] = -1.0f; searchBox.vertex_max[0] = 1.0f;
searchBox.vertex_min[1] = -1.0f; searchBox.vertex_max[1] = 1.0f;
searchBox.vertex_min[2] = -1.0f; searchBox.vertex_max[2] = 1.0f;

QVector<ikdTree_PointType> boxPoints;
kdTree->boxSearch(searchBox, boxPoints);
```

#### 3. 动态添加和删除点

```cpp
// 添加点（支持下采样）
QVector<ikdTree_PointType> newPoints;
// ... 填充新点
int addedCount = kdTree->addPoints(newPoints, true);

// 删除点
QVector<ikdTree_PointType> pointsToDelete;
// ... 填充要删除的点
kdTree->deletePoints(pointsToDelete);
```

## API参考

### 主要类

#### `KD_TREE<PointType>`
增量式K-D树的主要类模板。

**构造函数：**
```cpp
KD_TREE(float deleteParam = 0.5f, float balanceParam = 0.6f, float boxLength = 0.2f)
```

**主要方法：**

- `void build(const PointVector& pointCloud)` - 构建树
- `void nearestSearch(...)` - K近邻搜索  
- `void radiusSearch(...)` - 半径搜索
- `void boxSearch(...)` - 包围盒搜索
- `int addPoints(...)` - 添加点集
- `void deletePoints(...)` - 删除点集
- `int size() const` - 获取树大小
- `int validnum() const` - 获取有效节点数

#### `ikdTree_PointType`
3D点数据结构。

```cpp
struct ikdTree_PointType {
    float x, y, z;
    ikdTree_PointType(float px = 0.0f, float py = 0.0f, float pz = 0.0f);
    explicit ikdTree_PointType(const QVector3D& vec);
    QVector3D toVector3D() const;
};
```

#### `BoxPointType`
3D包围盒数据结构。

```cpp
struct BoxPointType {
    float vertex_min[3];  // 最小顶点坐标
    float vertex_max[3];  // 最大顶点坐标
    BoxPointType();
    BoxPointType(const ikdTree_PointType& min_point, const ikdTree_PointType& max_point);
};
```

## 性能特性

- **增量式更新** - 支持动态添加/删除点而不需要重建整个树
- **自动重建** - 当树不平衡时自动触发重建保持性能
- **多线程重建** - 使用Qt线程在后台进行树重建
- **下采样支持** - 内置下采样功能减少冗余点
- **线程安全** - 所有操作都是线程安全的

## 与原版差异

| 特性 | 原版 | Qt版本 |
|------|------|--------|
| 线程库 | pthread | QThread |
| 容器 | std::vector | QVector |
| 智能指针 | std::shared_ptr | QSharedPointer |
| 互斥锁 | pthread_mutex_t | QMutex |
| 原子操作 | 手动实现 | QAtomicInt |
| 点云库 | PCL | 自定义结构 |
| 注释语言 | 英文 | 中文 |

## 许可证

本项目基于原始ikd-Tree的开源许可证，为Qt移植版本。

## 致谢

- 原作者：Yixi Cai (yixicai@connect.hku.hk)
- Qt移植：基于原始ikd-Tree算法实现
