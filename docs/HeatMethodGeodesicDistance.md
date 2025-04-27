# 基于Heat Method的测地距离计算技术方案

## 1. 概述

本文档描述了在MeshLib中实现基于Heat Method的测地距离计算方法的技术方案。Heat Method算法具有高效、可并行化程度高、数值稳定等优势，特别适合大规模三维网格的测地距离计算，可广泛应用于CAM刀具路径生成、网格参数化、形状分析等场景。

## 2. Heat Method原理

Heat Method算法由Keenan Crane等人在2013年提出（论文：《Geodesics in Heat: A New Approach to Computing Distance Based on Heat Flow》），其核心思想是利用热传导方程与测地距离之间的数学联系，将复杂的测地距离计算转化为三个简单步骤：

1. **热扩散（Heat Flow）**：求解短时间热传导方程
2. **梯度计算（Gradient Extraction）**：计算归一化梯度场
3. **泊松重建（Poisson Reconstruction）**：求解泊松方程重建距离场

与传统的Fast Marching Method相比，Heat Method有以下优势：
- **高并行性**：三个步骤均可高度并行化，适合GPU加速
- **快速计算**：对大规模网格有显著的速度优势
- **数值稳定**：不依赖于网格质量和拓扑结构
- **多源点支持**：可同时计算来自多个源点的距离场

## 3. 算法实现

### 3.1 数学原理

Heat Method的核心是利用热传导与测地距离的关系。当t→0时，热量扩散梯度与测地线方向的负梯度（指向热源）一致：

$$\nabla u(x, t) \approx -\frac{d(x)}{2t} e^{-\frac{d(x)^2}{4t}} \nabla d(x)$$

其中，u是热分布函数，d是测地距离函数。

### 3.2 算法步骤

#### 步骤1：求解热方程（短时间热扩散）

求解热传导方程：
$$\frac{\partial u}{\partial t} = \Delta u$$

离散形式：
$$(I - t\Delta)u = u_0$$

其中，$\Delta$是Laplace-Beltrami算子，$u_0$是初始热分布（源点为1，其他点为0）。

#### 步骤2：计算归一化梯度场

计算热分布梯度并归一化：
$$X = -\frac{\nabla u}{|\nabla u|}$$

#### 步骤3：求解泊松方程

求解：
$$\Delta \phi = \nabla \cdot X$$

最终测地距离为$\phi$的解。

### 3.3 离散化实现

在三角网格上，主要涉及以下离散化操作：

1. **离散Laplace-Beltrami算子**：使用Cotan权重
2. **离散梯度算子**：在三角面上定义
3. **离散散度算子**：三角面到顶点的映射

### 3.4 实现流程图

```
输入：三角网格，源点集
│
├── 步骤1：热扩散
│   ├── 构建Laplace矩阵(L)
│   ├── 设置t值（时间步长）
│   ├── 求解线性系统(I-tL)u=u0
│   └── 输出热分布u
│
├── 步骤2：梯度计算与归一化
│   ├── 计算每个三角面上的热梯度
│   ├── 归一化梯度向量
│   └── 输出单位向量场X
│
├── 步骤3：泊松重建
│   ├── 计算向量场X的散度
│   ├── 求解泊松方程Lφ=∇·X
│   └── 输出距离场φ
│
└── 输出：测地距离场
```

## 4. 与MeshLib集成方案

### 4.1 代码结构

在MeshLib中实现Heat Method，建议添加以下文件：

```
source/MRMesh/
├── MRHeatMethod.h             // Heat Method类定义和接口
├── MRHeatMethod.cpp           // 核心算法实现
└── MRHeatGeodesicDistance.h/cpp  // 基于Heat Method的测地距离计算
```

### 4.2 核心类设计

```cpp
class HeatMethod
{
public:
    // 构造函数
    HeatMethod(const Mesh& mesh);
    
    // 设置时间步长参数
    void setTimeStep(float t);
    
    // 计算从单个源点出发的测地距离
    VertScalars computeDistance(const MeshTriPoint& source);
    
    // 计算从多个源点出发的测地距离
    VertScalars computeDistance(const std::vector<MeshTriPoint>& sources);
    
    // 计算从顶点集出发的测地距离
    VertScalars computeDistance(const VertBitSet& sourceVerts);

private:
    // 预计算Laplace矩阵（使用cotan权重）
    void buildLaplacian();
    
    // 求解热方程
    void solveHeatFlow(VertScalars& u0, float t);
    
    // 计算梯度场
    void computeNormalizedGradient(const VertScalars& u, std::vector<Vector3f>& X);
    
    // 计算散度
    void computeDivergence(const std::vector<Vector3f>& X, VertScalars& divX);
    
    // 求解泊松方程
    void solvePoissonEquation(const VertScalars& rhs, VertScalars& phi);
    
    // 成员变量
    const Mesh& mesh_;
    SparseMatrix laplacian_;  // Laplace-Beltrami算子
    float timeStep_ = 1.0f;   // 时间步长
    
    // 线性系统求解器
    std::unique_ptr<Solver> heatSolver_;
    std::unique_ptr<Solver> poissonSolver_;
};
```

### 4.3 与现有接口集成

为了与MeshLib现有的Surface Distance API兼容，建议扩展`MRSurfaceDistance.h`，添加基于Heat Method的函数：

```cpp
// 使用Heat Method计算测地距离
MRMESH_API VertScalars computeHeatGeodesicDistance(
    const Mesh& mesh, 
    const MeshTriPoint& start, 
    float timeStep = 1.0f,
    const VertBitSet* region = nullptr
);

// 使用Heat Method计算多源点测地距离
MRMESH_API VertScalars computeHeatGeodesicDistance(
    const Mesh& mesh, 
    const std::vector<MeshTriPoint>& starts,
    float timeStep = 1.0f,
    const VertBitSet* region = nullptr
);

// 使用Heat Method计算从顶点集出发的测地距离
MRMESH_API VertScalars computeHeatGeodesicDistance(
    const Mesh& mesh, 
    const VertBitSet& startVertices,
    float timeStep = 1.0f,
    const VertBitSet* region = nullptr
);
```

### 4.4 内部依赖项

- **Eigen库**：线性代数运算和稀疏矩阵求解
- **MRLaplacian**：复用现有的Laplace矩阵构建功能
- **MRTimer**：性能计时
- **MRParallelFor**：并行计算

## 5. 有序加工路径生成

Heat Method计算出的是距离场，为了生成有序的加工路径，需要以下步骤：

### 5.1 提取等距离线

从距离场提取等距离线（等值线）:

```cpp
// 从测地距离场提取等值线
MRMESH_API Contours3f extractGeodesicContours(
    const Mesh& mesh,
    const VertScalars& distanceField,
    const std::vector<float>& levels,
    bool sortContours = true
);
```

### 5.2 路径排序算法

为确保加工路径有序，需要对提取的等距离线进行排序：

1. **层级排序**：首先按照距离值从小到大排序
2. **层内排序**：同一距离值的多个封闭轮廓，采用以下策略排序：
   - 内轮廓优先于外轮廓
   - 相邻轮廓连续加工
   - 最小化空行程距离

```cpp
// 对等值线进行排序，生成有序加工路径
MRMESH_API Contours3f sortGeodesicContours(
    const Contours3f& contours,
    const std::vector<float>& contourLevels,
    SortStrategy strategy = SortStrategy::MinimizeTransitions
);
```

### 5.3 路径连接与平滑

为减少刀具空行程，需要对相邻轮廓线进行连接和平滑：

```cpp
// 优化加工路径，减少空行程，添加连接路径
MRMESH_API Contours3f optimizeToolPath(
    const Contours3f& sortedContours,
    float maxTransitionHeight,
    bool addSmoothTransitions = true
);
```

### 5.4 完整路径生成流程

```
输入：网格和起点
│
├── 计算测地距离场
│   └── 使用Heat Method
│
├── 提取等距离线
│   ├── 确定等距离值
│   └── 提取对应轮廓线
│
├── 路径排序
│   ├── 层间排序（距离值）
│   └── 层内排序（减少空行程）
│
├── 路径连接与平滑
│   ├── 添加过渡路径
│   └── 平滑路径转角
│
└── 输出：有序加工路径
```

## 6. 性能优化方案

### 6.1 并行计算

Heat Method的三个主要步骤均适合并行化：

1. **热扩散**：使用并行线性系统求解器
2. **梯度计算**：三角面的梯度计算可并行
3. **泊松求解**：使用并行求解器

### 6.2 GPU加速

对于大规模网格，可考虑GPU加速：

```cpp
// GPU加速的Heat Method计算
#ifdef MR_USE_CUDA
MRMESH_API VertScalars computeHeatGeodesicDistanceCuda(
    const Mesh& mesh, 
    const MeshTriPoint& start,
    float timeStep = 1.0f
);
#endif
```

### 6.3 预计算与缓存

对于静态网格，可以预计算并缓存：

1. **Laplace矩阵**：只与网格拓扑和几何相关
2. **线性系统分解**：Cholesky分解可重用
3. **多分辨率策略**：对大型网格使用层次化策略

## 7. 实现示例

### 7.1 基础实现

```cpp
VertScalars computeHeatGeodesicDistance(const Mesh& mesh, const MeshTriPoint& start, float timeStep, const VertBitSet* region)
{
    MR_TIMER;
    
    // 构建Heat Method求解器
    HeatMethod heatMethod(mesh);
    heatMethod.setTimeStep(timeStep);
    
    // 计算测地距离
    return heatMethod.computeDistance(start);
}
```

### 7.2 等值线提取示例

```cpp
Contours3f extractGeodesicContours(const Mesh& mesh, const VertScalars& distField, const std::vector<float>& levels)
{
    MR_TIMER;
    
    Contours3f contours;
    contours.reserve(levels.size());
    
    for (float level : levels)
    {
        auto isoContour = extractIsolines(mesh, distField, level);
        contours.insert(contours.end(), isoContour.begin(), isoContour.end());
    }
    
    return contours;
}
```

## 8. 测试与验证方案

### 8.1 单元测试

1. **精度测试**：与解析解比较（球面、平面等）
2. **鲁棒性测试**：各种复杂形状和退化网格
3. **性能测试**：与Fast Marching方法对比

### 8.2 回归测试

1. **结果稳定性**：确保算法在不同网格分辨率下结果一致
2. **内存使用**：监控大型网格的内存消耗
3. **加工路径验证**：生成G代码并模拟验证

## 9. 结论与未来工作

Heat Method提供了一种高效计算测地距离的方法，特别适合CAM领域的加工路径生成。该方法在MeshLib中的实现将为现有的路径生成功能提供更高效的算法选择。

### 9.1 未来扩展方向

1. **各向异性距离**：支持考虑方向的测地距离计算
2. **热核签名**：扩展支持形状分析
3. **增量更新**：对网格局部修改后的距离场快速更新

## 10. 参考资料

1. Crane, K., Weischedel, C., & Wardetzky, M. (2013). Geodesics in heat: A new approach to computing distance based on heat flow. ACM Transactions on Graphics (TOG), 32(5), 152.
2. Crane, K., Weischedel, C., & Wardetzky, M. (2017). The heat method for distance computation. Communications of the ACM, 60(11), 90-99.
3. Sharp, N., & Crane, K. (2020). A Laplacian for nonmanifold triangle meshes. Computer Graphics Forum (SGP), 39(5). 