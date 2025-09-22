# MeshLib 技术文档

## 目录

1. [架构概述](#架构概述)
2. [核心数据结构](#核心数据结构)
3. [半边数据结构详解](#半边数据结构详解)
4. [空间索引与AABB树](#空间索引与aabb树)
5. [网格处理算法](#网格处理算法)
6. [几何计算核心](#几何计算核心)
7. [内存管理策略](#内存管理策略)
8. [并行处理机制](#并行处理机制)
9. [API接口说明](#api接口说明)
10. [性能优化技术](#性能优化技术)

---

## 架构概述

MeshLib 是一个高性能的三维网格处理库，采用现代C++设计，提供了完整的三维网格建模、处理和分析功能。整个库围绕半边数据结构（Half-Edge Data Structure）构建，这是计算几何领域的核心数据结构之一。

### 设计理念

1. **模块化架构**：将功能分解为独立的组件，通过清晰的接口进行交互
2. **性能优先**：采用缓存友好的数据布局和并行处理技术
3. **类型安全**：使用强类型ID系统（VertId、EdgeId、FaceId）避免误用
4. **零开销抽象**：模板元编程和内联优化确保抽象不带来性能损失

### 核心组件层次

```
┌─────────────────────────────────────────┐
│         应用层 API                       │
├─────────────────────────────────────────┤
│    算法层（Decimate/Smooth/Boolean）     │
├─────────────────────────────────────────┤
│    几何计算层（Distance/Intersection）    │
├─────────────────────────────────────────┤
│    空间索引层（AABBTree/Dipoles）        │
├─────────────────────────────────────────┤
│    拓扑层（MeshTopology）                │
├─────────────────────────────────────────┤
│    基础数据结构（Vector/Matrix/Id）       │
└─────────────────────────────────────────┘
```

---

## 核心数据结构

### Mesh 类

`Mesh` 类是整个库的核心，它将拓扑信息和几何信息结合在一起：

```cpp
struct Mesh {
    MeshTopology topology;  // 拓扑连接关系
    VertCoords points;      // 顶点坐标
    
    // 缓存的空间索引结构
    mutable SharedThreadSafeOwner<AABBTree> AABBTreeOwner_;
    mutable SharedThreadSafeOwner<AABBTreePoints> AABBTreePointsOwner_;
    mutable SharedThreadSafeOwner<Dipoles> dipolesOwner_;
};
```

#### 设计特点

1. **分离关注点**：拓扑和几何分开存储，便于独立处理
2. **惰性计算**：空间索引结构按需构建，使用 `mutable` 和线程安全包装
3. **缓存机制**：自动管理缓存的有效性，在几何或拓扑变化时失效

### ID 系统

MeshLib 使用强类型ID系统，每种实体都有独特的ID类型：

```cpp
template<typename T>
class Id {
    int id_;
public:
    constexpr explicit Id(int i) noexcept : id_(i) {}
    constexpr int get() const noexcept { return id_; }
    constexpr bool valid() const noexcept { return id_ >= 0; }
};

using VertId = Id<VertTag>;
using EdgeId = Id<EdgeTag>;
using FaceId = Id<FaceTag>;
```

这种设计避免了不同类型ID的误用，在编译期就能捕获类型错误。

### Vector 容器

自定义的 `Vector` 容器专门为ID索引优化：

```cpp
template<typename T, typename I>
class Vector {
    std::vector<T> data_;
public:
    T& operator[](I id) { return data_[id.get()]; }
    const T& operator[](I id) const { return data_[id.get()]; }
};
```

这提供了类型安全的索引访问，同时保持了原生数组的性能。

---

## 半边数据结构详解

### 半边表示法

MeshLib 采用半边（Half-Edge）数据结构，这是表示多边形网格的经典方法。每条边被分解为两个有向半边，分别属于相邻的两个面。

```cpp
struct HalfEdgeRecord {
    EdgeId next;  // 下一条半边（逆时针）
    EdgeId prev;  // 上一条半边（顺时针）
    VertId org;   // 起点顶点
    FaceId left;  // 左侧面片
};
```

### 拓扑存储

```cpp
class MeshTopology {
private:
    Vector<HalfEdgeRecord, EdgeId> edges_;      // 所有半边记录
    Vector<EdgeId, VertId> edgePerVertex_;      // 每个顶点的一条出边
    Vector<EdgeId, FaceId> edgePerFace_;        // 每个面的一条边
    
    VertBitSet validVerts_;  // 有效顶点集合
    FaceBitSet validFaces_;  // 有效面集合
    int numValidVerts_;      // 有效顶点计数
    int numValidFaces_;      // 有效面计数
};
```

### 拓扑操作原理

#### 1. 边遍历（Ring Traversal）

```cpp
// 遍历顶点的所有出边
EdgeId e = edgeWithOrg(v);
EdgeId start = e;
do {
    // 处理边 e
    e = next(e);
} while (e != start);
```

#### 2. 拓扑修改操作

**Splice 操作**：这是半边结构的核心操作，用于连接或断开边环。

```cpp
void splice(EdgeId a, EdgeId b) {
    // 交换 a 和 b 之后的边链接
    // 如果 a 和 b 在同一环中，则分裂成两个环
    // 如果在不同环中，则合并成一个环
}
```

**Edge Collapse**：边坍缩是网格简化的基本操作。

```cpp
EdgeId collapseEdge(EdgeId e, callback) {
    // 1. 删除相邻的两个三角形
    // 2. 合并两个顶点
    // 3. 更新周围的拓扑连接
    // 4. 调用回调处理属性迁移
}
```

**Edge Flip**：边翻转用于改善网格质量。

```cpp
void flipEdge(EdgeId e) {
    // 将共享边的两个三角形重新三角化
    // 改变对角线的连接方式
}
```

### 边界处理

半边结构优雅地处理网格边界：

- 边界边只有一个相邻面（另一侧是"洞"）
- 通过检查 `left(e)` 和 `right(e)` 的有效性判断边界
- 提供专门的边界遍历API

---

## 空间索引与AABB树

### AABB树结构

轴对齐包围盒树（Axis-Aligned Bounding Box Tree）是MeshLib的核心空间索引结构。

```cpp
template<typename Traits>
class AABBTreeBase {
protected:
    Vector<AABBTreeNode<Traits>, NodeId> nodes_;
    
public:
    struct Node {
        Box3f box;           // 包围盒
        NodeId leftOrFirst;  // 左子节点或第一个叶子
        NodeId rightOrLast;  // 右子节点或最后一个叶子
    };
};
```

### 树构建算法

采用自顶向下的SAH（Surface Area Heuristic）构建策略：

1. **分割策略**：选择使子节点表面积最小的分割平面
2. **平衡性**：确保树的深度为 O(log n)
3. **内存布局**：节点按深度优先顺序存储，提高缓存命中率

### 查询算法

#### 最近点查询

```cpp
MeshProjectionResult projectPoint(const Vector3f& point) {
    // 1. 从根节点开始递归搜索
    // 2. 使用包围盒剪枝不可能的分支
    // 3. 维护当前最近距离进行早期终止
    // 4. 返回最近的面片和重心坐标
}
```

#### 射线相交

```cpp
struct RayIntersection {
    float t;        // 射线参数
    FaceId face;    // 相交面片
    TriPoint bary;  // 重心坐标
};
```

### Dipoles 加速结构

Dipoles 是用于快速计算缠绕数（Winding Number）的辅助结构：

```cpp
struct Dipole {
    Vector3f center;     // 偶极子中心
    Vector3f moment;     // 偶极矩
    float area;          // 覆盖面积
};
```

这允许在远距离使用偶极子近似，大幅加速内外判断。

---

## 网格处理算法

### 网格简化（Decimation）

基于二次误差度量（QEM）的边坍缩算法：

```cpp
struct DecimateSettings {
    DecimateStrategy strategy;      // 简化策略
    float maxError;                 // 最大误差阈值
    float maxEdgeLen;               // 最大边长
    float maxTriangleAspectRatio;  // 最大长宽比
    bool optimizeVertexPos;         // 是否优化顶点位置
};
```

#### 算法流程

1. **初始化**：计算每个顶点的二次误差矩阵
2. **优先队列**：根据坍缩代价排序所有边
3. **迭代坍缩**：
   - 选择代价最小的边
   - 计算最优坍缩位置
   - 更新周围顶点的误差矩阵
   - 更新优先队列

#### 二次误差度量

每个顶点关联一个4×4对称矩阵Q，表示到相邻平面的距离平方和：

```cpp
class QuadraticForm3f {
    SymMatrix4f Q;  // 二次型矩阵
    
    float evaluate(const Vector3f& p) const {
        // 计算点p的误差: p^T * Q * p
    }
    
    Vector3f findMinimum() const {
        // 求解使误差最小的点位置
    }
};
```

### 网格平滑

提供多种平滑算法：

1. **拉普拉斯平滑**：移动顶点到邻居的质心
2. **双边滤波**：保持特征的平滑
3. **Taubin平滑**：避免体积收缩的平滑

### 网格重建

从点云或三角汤（Triangle Soup）重建拓扑一致的网格：

```cpp
Mesh fromTriangles(const Triangulation& triangles) {
    // 1. 构建顶点-面片关联
    // 2. 识别并处理非流形顶点
    // 3. 构建半边结构
    // 4. 缝合边界
}
```

处理非流形情况的策略：
- 复制非流形顶点
- 删除退化三角形
- 修复T型连接

---

## 几何计算核心

### 向量运算

```cpp
template<typename T>
struct Vector3 {
    T x, y, z;
    
    T length() const { return sqrt(lengthSq()); }
    T lengthSq() const { return x*x + y*y + z*z; }
    Vector3 normalized() const;
    Vector3 cross(const Vector3& b) const;
    T dot(const Vector3& b) const;
};
```

优化技巧：
- 尽可能使用 `lengthSq()` 避免开方
- 向量归一化检查零长度
- SIMD 指令加速（通过编译器自动向量化）

### 三角形运算

```cpp
// 重心坐标计算
TriPoint toTriPoint(FaceId f, const Vector3f& p) {
    // 使用Möller-Trumbore算法
    // 返回 (u, v, 1-u-v) 坐标
}

// 面积计算
float area(FaceId f) {
    auto [v0, v1, v2] = getTriVerts(f);
    return 0.5f * cross(v1-v0, v2-v0).length();
}

// 法向量计算
Vector3f normal(FaceId f) {
    auto [v0, v1, v2] = getTriVerts(f);
    return cross(v1-v0, v2-v0).normalized();
}
```

### 距离计算

点到三角形的距离计算，考虑所有情况：

1. 投影在三角形内部
2. 投影在边上
3. 投影在顶点上

```cpp
struct MeshProjectionResult {
    MeshTriPoint mtp;    // 三角形上的点
    PointOnFace pof;     // 3D坐标
    float distSq;        // 距离平方
};
```

### 相交测试

提供多种相交测试：

- 射线-三角形相交（Möller-Trumbore算法）
- 线段-三角形相交
- 三角形-三角形相交
- 包围盒相交（SAT算法）

---

## 内存管理策略

### 内存池设计

MeshLib 使用连续内存存储提高缓存效率：

```cpp
template<typename T, typename Id>
class Vector {
    std::vector<T> data_;
    
    void reserve(size_t n) { 
        data_.reserve(n);  // 预分配避免重分配
    }
    
    void shrink_to_fit() {
        data_.shrink_to_fit();  // 释放多余内存
    }
};
```

### 延迟删除策略

删除操作不立即压缩数组，而是标记为无效：

```cpp
void deleteFace(FaceId f) {
    // 1. 标记面为无效
    validFaces_.reset(f);
    numValidFaces_--;
    
    // 2. 清除相关边的引用
    // 3. 不移动内存，保持ID稳定性
}
```

### Pack 操作

定期执行 pack 操作来压缩内存：

```cpp
void pack() {
    // 1. 创建新的紧凑数组
    // 2. 建立旧ID到新ID的映射
    // 3. 更新所有引用
    // 4. 交换数组
}
```

### 共享所有权

使用 `SharedThreadSafeOwner` 管理大型缓存结构：

```cpp
template<typename T>
class SharedThreadSafeOwner {
    mutable std::shared_ptr<T> ptr_;
    mutable std::mutex mutex_;
    
    const T& get() const {
        std::lock_guard lock(mutex_);
        if (!ptr_) {
            ptr_ = std::make_shared<T>(/*构建*/);
        }
        return *ptr_;
    }
};
```

---

## 并行处理机制

### 任务分解

大型网格操作支持并行处理：

```cpp
struct DecimateSettings {
    int subdivideParts = 1;  // 分区数量
};
```

算法自动将网格分割为独立的子区域并行处理。

### 线程安全设计

1. **只读操作**：大部分查询操作是线程安全的
2. **缓存构建**：使用互斥锁保护惰性构建的缓存
3. **批量操作**：提供批量API减少锁竞争

### 并行算法示例

```cpp
// 并行构建AABB树
void buildAABBTreeParallel(const Mesh& mesh) {
    auto subtrees = divideIntoSubtrees(mesh);
    
    #pragma omp parallel for
    for (auto& subtree : subtrees) {
        buildSubtree(subtree);
    }
    
    mergeSubtrees(subtrees);
}
```

### 工作窃取队列

对于不平衡的任务，使用工作窃取策略：

```cpp
class WorkStealingQueue {
    std::deque<Task> tasks_;
    std::mutex mutex_;
    
    bool steal(Task& task) {
        std::lock_guard lock(mutex_);
        if (!tasks_.empty()) {
            task = tasks_.back();
            tasks_.pop_back();
            return true;
        }
        return false;
    }
};
```

---

## API接口说明

### 网格创建

```cpp
// 从三角形列表创建
Mesh mesh = Mesh::fromTriangles(vertices, triangles);

// 从点云三角化
Mesh mesh = Mesh::fromPointTriples(trianglePoints);

// 从面汤创建
Mesh mesh = Mesh::fromFaceSoup(vertices, faces);
```

### 网格查询

```cpp
// 获取包围盒
Box3f bbox = mesh.getBoundingBox();

// 计算面积和体积
double area = mesh.area();
double volume = mesh.volume();

// 最近点查询
auto result = mesh.projectPoint(point);

// 射线相交
auto hit = mesh.rayIntersect(ray);
```

### 网格修改

```cpp
// 网格简化
decimateMesh(mesh, DecimateSettings{
    .maxError = 0.01f,
    .targetFaces = 1000
});

// 网格平滑
smoothMesh(mesh, SmoothSettings{
    .iterations = 5,
    .force = 0.5f
});

// 网格细分
subdivideMesh(mesh, SubdivideSettings{
    .maxEdgeLen = 0.1f
});
```

### 拓扑操作

```cpp
// 边坍缩
mesh.topology.collapseEdge(edge);

// 边翻转
mesh.topology.flipEdge(edge);

// 边分裂
mesh.splitEdge(edge, newVertexPos);

// 面分裂
mesh.splitFace(face, newVertexPos);
```

---

## 性能优化技术

### 数据局部性优化

1. **结构体数组（SoA）vs 数组结构体（AoS）**
   - 顶点坐标使用 SoA 布局
   - 半边记录使用 AoS 布局
   
2. **缓存行对齐**
   ```cpp
   alignas(64) struct HalfEdgeRecord { ... };
   ```

3. **预取优化**
   ```cpp
   // 手动预取下一个要访问的数据
   __builtin_prefetch(&edges_[next_edge]);
   ```

### 算法优化

1. **早期终止**
   - 在搜索算法中使用边界检查
   - 距离查询中的球形剪枝

2. **增量更新**
   - AABB树的局部更新而非重建
   - 二次误差矩阵的增量更新

3. **空间换时间**
   - 缓存常用计算结果（法向量、面积）
   - 预计算查找表

### 编译器优化

1. **模板内联**
   ```cpp
   template<typename T>
   inline T dot(const Vector3<T>& a, const Vector3<T>& b) {
       return a.x*b.x + a.y*b.y + a.z*b.z;
   }
   ```

2. **分支预测提示**
   ```cpp
   if (likely(edge.valid())) { ... }
   if (unlikely(error_condition)) { ... }
   ```

3. **循环向量化**
   ```cpp
   #pragma omp simd
   for (int i = 0; i < n; ++i) {
       result[i] = a[i] + b[i];
   }
   ```

### 内存优化

1. **小对象优化**
   - ID类型只占4字节
   - 使用位集合而非哈希集

2. **延迟分配**
   - 空间索引按需构建
   - 大型缓冲区延迟分配

3. **内存池**
   - 固定大小对象使用对象池
   - 减少动态分配开销

---

## 总结

MeshLib 是一个精心设计的高性能网格处理库，其核心优势在于：

1. **半边数据结构**：提供了强大而灵活的拓扑表示
2. **高效的空间索引**：AABB树和Dipoles加速几何查询
3. **现代C++设计**：类型安全、零开销抽象、RAII
4. **并行处理**：充分利用多核处理器
5. **缓存友好**：优化的内存布局和访问模式
6. **模块化架构**：清晰的层次结构和接口设计

这些设计选择使得MeshLib能够高效处理包含数百万三角形的大型网格，同时保持代码的可维护性和扩展性。无论是用于科学计算、工程仿真还是计算机图形学应用，MeshLib都提供了坚实的基础设施。