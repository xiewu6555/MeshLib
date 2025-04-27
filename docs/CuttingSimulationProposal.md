
# 基于MeshLib的切削仿真技术方案

## 1. 概述

本文档旨在提出一种在MeshLib框架内实现CNC切削仿真的技术方案。切削仿真的目标是可视化刀具沿预定路径移除工件材料的过程，验证刀具路径的正确性，检测碰撞，并分析最终加工结果。该功能将增强MeshLib的CAM能力，为用户提供更直观的路径验证工具。

## 2. 核心挑战

实现切削仿真需要解决以下关键问题：

1.  **工件表示**：如何高效表示初始毛坯（Stock）和动态变化的工件几何？
2.  **材料移除**：如何模拟刀具切割并更新工件几何？
3.  **刀具表示**：如何表示刀具几何及其运动（扫掠体）？
4.  **性能**：仿真过程，尤其是几何更新，计算量可能很大。
5.  **碰撞检测**：如何检测刀具/刀柄与工件/夹具之间的碰撞？
6.  **可视化**：如何实时或分步展示材料移除过程？

## 3. 技术选型：基于体素的方法

考虑到重复布尔运算在网格上的稳定性和性能挑战，以及MeshLib在体素处理（`MRVoxels`模块，特别是OpenVDB集成）方面的能力，**推荐采用基于体素的方法**进行切削仿真。

**优势**:

*   **鲁棒性**：体素布尔运算（尤其是基于SDF的）通常比网格布尔运算更稳定、快速。
*   **拓扑变化**：能自然处理复杂的拓扑变化（如孔洞、薄壁）。
*   **集成性**：可复用MeshLib现有的体素表示（`MRVDBFloatGrid`）、网格/体素转换（`MRMeshToDistanceVolume`, `MRMarchingCubes`）及OpenVDB工具。
*   **可视化**：体素数据易于进行体积渲染或快速提取表面进行可视化。

**主要思路**:

1.  将初始毛坯表示为一个**有符号距离场 (Signed Distance Field, SDF)** 的体素网格（如`MRVDBFloatGrid`）。
2.  模拟刀具沿路径段的运动，计算其**扫掠体**的SDF。
3.  通过**体素布尔运算 (CSG - Constructive Solid Geometry)**，从工件SDF中减去刀具扫掠体的SDF，模拟材料移除。
4.  动态更新体素网格，并从中提取表面网格进行可视化。

## 4. 核心组件与实现细节

### 4.1 工件表示 (Workpiece Representation)

*   **数据结构**: 使用`MR::VDBFloatGrid`存储工件的SDF。网格内部为负，外部为正，表面为0。
*   **初始化**:
    *   从输入的毛坯网格模型（`MR::Mesh`）转换得到初始SDF体素网格（使用`MR::meshToDistanceVolume`或类似功能）。
    *   支持基本形状（立方体、圆柱体）作为初始毛坯。
*   **精度**: 体素分辨率直接影响仿真精度和内存占用。需要提供可调参数。

### 4.2 刀具表示 (Tool Representation)

*   **几何定义**: 支持标准刀具类型（球头刀`BallEndMill`, 平底刀`FlatEndMill`, 圆环刀`ToroidalMill`）。
*   **表示方法**:
    *   **解析SDF**: 对简单刀具形状（球、圆柱、圆环）直接计算其SDF函数。
    *   **网格SDF**: 对复杂刀具（或刀柄），可将其表示为网格，再转换为局部SDF场。
*   **扫掠体SDF**:
    *   计算刀具沿一小段直线/圆弧路径运动形成的扫掠体。
    *   该扫掠体的SDF是仿真的核心。可采用保守近似（如用包络形状SDF）或更精确的计算方法。

### 4.3 材料移除算法 (Material Removal Algorithm)

*   **核心操作**: **CSG 差集 (A - B = A ∩ ¬B)**。在SDF场中，这对应于 `max` 运算：
    \[ SDF_{result}(p) = \max(SDF_{workpiece}(p), -SDF_{tool\_swept\_volume}(p)) \]
*   **实现**: 利用OpenVDB提供的`tools::csgUnion`, `tools::csgIntersection`, `tools::csgDifference` 或直接基于`transform`和`map`操作SDF网格。
*   **流程**:
    1.  获取当前工具路径段 (`MRToolPath`中的`GCommand`)。
    2.  确定刀具类型和参数。
    3.  计算该路径段对应的刀具扫掠体SDF (`SDF_{tool\_swept\_volume}`).
    4.  更新工件SDF: `workpieceSDF = max(workpieceSDF, -SDF_{tool\_swept\_volume})`.
    5.  重复处理下一路径段。

```mermaid
flowchart TD
    A[开始仿真] --> B(获取当前路径段 P)
    B --> C{路径段处理完毕?}
    C -- 否 --> D(计算刀具Tool的几何)
    D --> E(计算P对应的Tool扫掠体SDF: SDF_sweep)
    E --> F(计算-SDF_sweep)
    F --> G(更新工件SDF: SDF_workpiece = max(SDF_workpiece, -SDF_sweep))
    G --> B
    C -- 是 --> H[仿真结束]
    
    subgraph 数据
        I[工件SDF: SDF_workpiece]
        J[刀具路径]
    end
    
    J --> B
    I --> G
    G --> I
```

### 4.4 碰撞检测 (Collision Detection) - [可选/扩展]

*   **刀具/刀柄 vs. 更新后工件**:
    *   在每步更新后，计算刀具/刀柄（表示为SDF或查询点集）在当前`SDF_workpiece`中的值。
    *   如果刀具/刀柄的任何部分对应SDF值为负（或小于安全距离），则发生碰撞。
*   **刀具/刀柄 vs. 夹具**:
    *   将夹具表示为网格(`MR::Mesh`)或SDF体素。
    *   使用MeshLib的AABBTree进行网格碰撞检测，或进行SDF间的碰撞查询。
*   **实现**: 可基于`MR::AABBTree`或引入专门的碰撞检测库。

### 4.5 可视化 (Visualization)

*   **实时/分步更新**:
    1.  从当前`SDF_workpiece`体素网格提取等值面（使用`MR::MarchingCubes`或OpenVDB的`meshToVolume`工具）。
    2.  将提取的网格在`MRViewer`中渲染。
    3.  为提高性能，可以降低可视化网格的分辨率，或只在关键步骤/用户请求时更新。
*   **刀具可视化**: 渲染当前位置的刀具模型。
*   **结果分析**:
    *   渲染最终仿真结果的网格。
    *   与目标模型进行比较（如使用`MRMeshMeshDistance`），高亮显示过切/欠切区域。

## 5. 与MeshLib集成

### 5.1 新模块/类

*   **模块**: 建议在`MRVoxels`或创建一个新的`MRSimulation`模块。
*   **核心类**:
    *   `CuttingSimulator`: 管理仿真状态，执行仿真循环。
    *   `ToolDefinition`: 定义刀具几何和参数。
    *   `VoxelWorkpiece`: 封装基于`VDBFloatGrid`的工件SDF表示和更新操作。
    *   `CollisionDetector` (可选): 封装碰撞检测逻辑。

### 5.2 扩展现有模块

*   **MRToolPathPlugin**: 增加仿真控制UI（开始/暂停/步进/重置）、可视化选项、碰撞报告。
*   **MRViewer**: 集成仿真过程的可视化更新。
*   **MRToolPath**: 可能需要扩展以支持更精细的路径段信息（如姿态）。

### 5.3 依赖关系

```mermaid
graph TD
    A[CuttingSimulator] --> B[VoxelWorkpiece]
    A --> C[ToolDefinition]
    A --> D[MRToolPath]
    A --> E[MRViewer]
    A --> F[CollisionDetector]
    
    B --> G[MRVDBFloatGrid]
    B --> H[MRMeshToDistanceVolume]
    B --> I[MRMarchingCubes]
    B --> J[OpenVDB Tools]
    
    E --> I # 用于可视化
    F --> K[MRAABBTree] # 如果使用网格检测
    F --> G # 如果使用SDF检测
    
    L[MRToolPathPlugin] --> A
```

## 6. 性能优化

*   **体素分辨率**: 关键参数，平衡精度和性能。提供自适应分辨率（基于OpenVDB特性）或多级分辨率方案。
*   **并行计算**:
    *   体素更新操作（`max`运算）可在OpenVDB层面并行。
    *   Marching Cubes提取网格可并行（`MRMarchingCubes`可能已支持）。
    *   利用TBB (`MRTbbTaskArenaAndGroup`)。
*   **GPU加速**:
    *   SDF计算、CSG操作、Marching Cubes等是适合GPU加速的任务。
    *   利用`MRCuda`模块或直接使用CUDA/OpenCL/Vulkan Compute。
*   **稀疏性**: 充分利用OpenVDB的稀疏数据结构，只计算和存储靠近表面的体素。
*   **更新频率**: 优化可视化更新频率，避免每步都提取高精度网格。

## 7. API 设计 (初步)

```cpp
namespace MR {

// 刀具定义
struct ToolParams {
    ToolType type; // Ball, Flat, Toroidal
    float radius;
    float cornerRadius; // for Toroidal
    // ... 其他参数 (长度, 刀柄信息)
};

// 仿真设置
struct SimulationSettings {
    float voxelResolution; // 体素大小或每单位长度体素数
    // ... 其他设置 (碰撞检测开关, 安全距离)
};

class CuttingSimulator {
public:
    // 初始化仿真环境
    bool initialize(const Mesh& initialStock, 
                    const SimulationSettings& settings,
                    const std::vector<Mesh>& fixtures = {}); // 可选夹具

    // 设置当前使用的刀具
    void setTool(const ToolParams& tool);

    // 执行单步仿真 (处理一个路径段)
    bool stepSimulation(const GCommand& command); 
    
    // 执行整个工具路径
    bool runSimulation(const ToolPathResult& toolPath);
    
    // 获取当前工件的网格表示 (用于可视化)
    std::shared_ptr<Mesh> getCurrentWorkpieceMesh(float isoValue = 0.0f);
    
    // 获取碰撞信息 (如果开启)
    // CollisionReport getCollisions();

    // 重置仿真
    void reset();

private:
    VDBFloatGrid workpieceSDF_;
    ToolParams currentTool_;
    SimulationSettings settings_;
    // ... 内部状态
};

} // namespace MR
```

## 8. 测试与验证

*   **单元测试**: 测试SDF生成、CSG操作、扫掠体计算的正确性。
*   **集成测试**: 使用简单几何（如立方体、平面）和简单路径（直线、圆弧）进行仿真，与预期结果对比。
*   **复杂场景测试**: 使用真实零件模型和多轴路径进行测试。
*   **性能基准测试**: 测试不同分辨率、不同模型复杂度下的仿真速度和内存占用。
*   **视觉验证**: 对比仿真结果和实际加工件（如果可能）。

## 9. 未来工作

*   **刀柄/夹具仿真与碰撞检测**：更全面的碰撞检查。
*   **多轴仿真**：支持5轴等复杂运动的扫掠体计算。
*   **切削力/扭矩估算**：结合材料属性和切削参数进行力学分析。
*   **表面精度分析**：评估仿真结果的表面粗糙度。
*   **自适应体素分辨率**：根据几何复杂度动态调整分辨率。

## 10. 结论

采用基于体素（特别是OpenVDB SDF）的方法，结合MeshLib现有的网格处理、体素操作和可视化能力，是实现高效、鲁棒切削仿真的可行方案。该方案可以很好地集成到现有框架中，并为未来的功能扩展打下基础。