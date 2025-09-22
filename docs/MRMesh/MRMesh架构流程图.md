# MRMesh 架构流程图详解

## 1. 系统整体架构图

```mermaid
graph TB
    subgraph "应用层"
        APP[应用程序]
        PLUGIN[插件系统]
        SCRIPT[脚本接口]
    end
    
    subgraph "接口层"
        API[C++ API]
        BINDING[Python绑定]
        CLI[命令行接口]
    end
    
    subgraph "算法层"
        BOOL[布尔运算]
        DEC[网格简化]
        FILL[孔洞填充]
        FIX[网格修复]
        SMOOTH[平滑算法]
        PARAM[参数化]
        COLL[碰撞检测]
        ICP[配准算法]
    end
    
    subgraph "数据结构层"
        MESH[Mesh网格]
        TOPO[拓扑结构]
        GEOM[几何数据]
        INDEX[空间索引]
    end
    
    subgraph "基础设施层"
        MEM[内存管理]
        THREAD[线程池]
        IO[文件IO]
        MATH[数学库]
    end
    
    APP --> API
    PLUGIN --> API
    SCRIPT --> BINDING
    
    API --> BOOL
    API --> DEC
    API --> FILL
    API --> FIX
    API --> SMOOTH
    API --> PARAM
    API --> COLL
    API --> ICP
    
    BOOL --> MESH
    DEC --> MESH
    FILL --> MESH
    FIX --> MESH
    SMOOTH --> MESH
    PARAM --> MESH
    COLL --> INDEX
    ICP --> INDEX
    
    MESH --> TOPO
    MESH --> GEOM
    MESH --> INDEX
    
    TOPO --> MEM
    GEOM --> MEM
    INDEX --> THREAD
    IO --> MEM
```

## 2. 半边数据结构详细流程图

```mermaid
graph LR
    subgraph "半边数据结构"
        subgraph "顶点环"
            V[顶点v] --> E1[出边e1]
            E1 --> E2[e1.next.sym]
            E2 --> E3[e2.next.sym]
            E3 --> E4[...]
            E4 --> E1
        end
        
        subgraph "面环"
            F[面f] --> FE1[边fe1]
            FE1 --> FE2[fe1.next]
            FE2 --> FE3[fe2.next]
            FE3 --> FE1
        end
        
        subgraph "边对"
            HE1[半边he] -.-> HE2[对称边he.sym]
            HE1 --> VO[起点org]
            HE2 --> VD[起点dest]
            HE1 --> FL[左面left]
            HE2 --> FR[左面right]
        end
    end
```

## 3. 网格构建流程

```mermaid
flowchart TD
    Start[开始] --> Input{输入类型?}
    
    Input -->|三角列表| TriList[Triangulation]
    Input -->|面片汤| FaceSoup[Face Soup]
    Input -->|点云| PointCloud[Point Cloud]
    
    TriList --> CheckManifold{检查流形性?}
    CheckManifold -->|非流形| DupVerts[复制非流形顶点]
    CheckManifold -->|流形| BuildTopo[构建拓扑]
    DupVerts --> BuildTopo
    
    FaceSoup --> Triangulate[三角化]
    Triangulate --> CheckManifold
    
    PointCloud --> Reconstruct[表面重建]
    Reconstruct --> TriList
    
    BuildTopo --> CreateHalfEdges[创建半边]
    CreateHalfEdges --> LinkEdges[连接边]
    LinkEdges --> SetVerts[设置顶点]
    SetVerts --> SetFaces[设置面]
    SetFaces --> Validate[验证拓扑]
    
    Validate --> Success{成功?}
    Success -->|是| Output[输出Mesh]
    Success -->|否| Report[报告错误]
    Report --> Repair[尝试修复]
    Repair --> Validate
```

## 4. AABB树构建与查询流程

```mermaid
flowchart TD
    subgraph "构建流程"
        InputFaces[输入三角形] --> ComputeBBox[计算包围盒]
        ComputeBBox --> MortonCode[计算Morton码]
        
        MortonCode --> Sort[径向排序]
        Sort --> BuildLeaves[构建叶子节点]
        
        BuildLeaves --> FindSplit[寻找最佳分割]
        FindSplit --> CreateInternal[创建内部节点]
        CreateInternal --> UpdateBBox[更新包围盒]
        
        UpdateBBox --> MoreNodes{还有节点?}
        MoreNodes -->|是| FindSplit
        MoreNodes -->|否| TreeComplete[树构建完成]
    end
    
    subgraph "查询流程"
        QueryPoint[查询点] --> RootNode[根节点]
        RootNode --> TestBBox{包围盒测试}
        
        TestBBox -->|相交| IsLeaf{是叶子?}
        TestBBox -->|不相交| Prune[剪枝]
        
        IsLeaf -->|是| ExactTest[精确测试]
        IsLeaf -->|否| PushChildren[子节点入队]
        
        ExactTest --> UpdateBest[更新最佳结果]
        PushChildren --> PriorityQueue[优先队列]
        
        PriorityQueue --> NextNode[取下一节点]
        NextNode --> TestBBox
        
        UpdateBest --> CheckQueue{队列空?}
        CheckQueue -->|否| NextNode
        CheckQueue -->|是| ReturnResult[返回结果]
    end
```

## 5. 网格布尔运算详细流程

```mermaid
stateDiagram-v2
    [*] --> 预处理
    
    state 预处理 {
        输入验证 --> 构建AABB树
        构建AABB树 --> 计算包围盒
    }
    
    预处理 --> 相交检测
    
    state 相交检测 {
        [*] --> 宽相检测
        宽相检测 --> 窄相检测
        
        state 宽相检测 {
            AABB重叠测试 --> 候选对生成
        }
        
        state 窄相检测 {
            三角形相交 --> 交线计算
            交线计算 --> 交点排序
        }
    }
    
    相交检测 --> 网格分割
    
    state 网格分割 {
        [*] --> 边分割
        边分割 --> 面分割
        面分割 --> 重新三角化
        
        state 重新三角化 {
            约束Delaunay --> 质量优化
        }
    }
    
    网格分割 --> 分类标记
    
    state 分类标记 {
        [*] --> 种子选择
        种子选择 --> 区域生长
        区域生长 --> 内外判定
        
        state 内外判定 {
            射线法 --> 卷绕数法
            卷绕数法 --> 标记验证
        }
    }
    
    分类标记 --> 结果组装
    
    state 结果组装 {
        选择保留面 --> 边界识别
        边界识别 --> 缝合边界
        缝合边界 --> 拓扑清理
    }
    
    结果组装 --> 后处理
    
    state 后处理 {
        移除孤立 --> 合并重复
        合并重复 --> 优化网格
    }
    
    后处理 --> [*]
```

## 6. 网格简化算法流程

```mermaid
flowchart TD
    subgraph "初始化阶段"
        Start[开始] --> InitQEM[初始化QEM]
        InitQEM --> ComputeQ[计算顶点二次型]
        
        ComputeQ --> ForEachFace[遍历每个面]
        ForEachFace --> FacePlane[计算面平面]
        FacePlane --> VertexQ[累加到顶点Q]
        VertexQ --> MoreFaces{更多面?}
        MoreFaces -->|是| ForEachFace
        MoreFaces -->|否| InitHeap[初始化优先队列]
    end
    
    subgraph "边评估阶段"
        InitHeap --> ForEachEdge[遍历每条边]
        ForEachEdge --> ComputeCost[计算折叠代价]
        
        ComputeCost --> QSum[Q = Q1 + Q2]
        QSum --> OptimalPos[求最优位置]
        OptimalPos --> Error[计算误差]
        Error --> CheckConstraints{满足约束?}
        
        CheckConstraints -->|是| AddToHeap[加入优先队列]
        CheckConstraints -->|否| SkipEdge[跳过]
        
        AddToHeap --> MoreEdges{更多边?}
        SkipEdge --> MoreEdges
        MoreEdges -->|是| ForEachEdge
        MoreEdges -->|否| SimplifyLoop[简化循环]
    end
    
    subgraph "简化循环"
        SimplifyLoop --> HeapEmpty{队列空?}
        HeapEmpty -->|否| PopMin[取最小代价边]
        HeapEmpty -->|是| Finish[完成]
        
        PopMin --> ValidEdge{边有效?}
        ValidEdge -->|否| SimplifyLoop
        ValidEdge -->|是| CheckError{误差<阈值?}
        
        CheckError -->|是| Collapse[执行折叠]
        CheckError -->|否| Finish
        
        Collapse --> UpdateTopo[更新拓扑]
        UpdateTopo --> UpdateNeighbors[更新邻域]
        UpdateNeighbors --> RecomputeCosts[重算邻边代价]
        RecomputeCosts --> UpdateHeap[更新队列]
        UpdateHeap --> SimplifyLoop
    end
```

## 7. 孔洞填充算法流程

```mermaid
flowchart LR
    subgraph "孔洞检测"
        FindBoundary[查找边界边] --> GroupLoops[分组边界环]
        GroupLoops --> ValidateHoles[验证孔洞]
    end
    
    subgraph "填充策略选择"
        ValidateHoles --> HoleSize{孔洞大小?}
        HoleSize -->|小| MinWeight[最小权重法]
        HoleSize -->|中| Advancing[前沿推进法]
        HoleSize -->|大| Volumetric[体积法]
    end
    
    subgraph "最小权重三角化"
        MinWeight --> ExtractVerts[提取边界顶点]
        ExtractVerts --> DPTable[构建DP表]
        DPTable --> FillTable[填充DP表]
        FillTable --> Backtrack[回溯构建三角形]
    end
    
    subgraph "前沿推进法"
        Advancing --> InitFront[初始化前沿]
        InitFront --> SelectEar[选择耳朵]
        SelectEar --> CreateTri[创建三角形]
        CreateTri --> UpdateFront[更新前沿]
        UpdateFront --> FrontEmpty{前沿空?}
        FrontEmpty -->|否| SelectEar
        FrontEmpty -->|是| FillComplete[填充完成]
    end
    
    subgraph "后处理"
        Backtrack --> Smooth[平滑新顶点]
        FillComplete --> Smooth
        Smooth --> Optimize[优化三角形]
        Optimize --> Result[输出结果]
    end
```

## 8. 内存管理架构

```mermaid
graph TD
    subgraph "内存分配层次"
        App[应用请求] --> HighLevel[高级分配器]
        
        HighLevel --> PoolAlloc[内存池]
        HighLevel --> ArenaAlloc[竞技场分配器]
        HighLevel --> StackAlloc[栈分配器]
        
        PoolAlloc --> BlockMgr[块管理器]
        ArenaAlloc --> ChunkMgr[块管理器]
        StackAlloc --> LinearMgr[线性管理器]
        
        BlockMgr --> System[系统分配]
        ChunkMgr --> System
        LinearMgr --> System
    end
    
    subgraph "缓存管理"
        Cache[缓存系统] --> LRU[LRU策略]
        Cache --> ARC[ARC策略]
        
        LRU --> Evict[驱逐策略]
        ARC --> Evict
        
        Evict --> Free[释放内存]
        Evict --> Persist[持久化]
    end
    
    subgraph "垃圾回收"
        GC[垃圾回收器] --> RefCount[引用计数]
        GC --> MarkSweep[标记清除]
        
        RefCount --> Immediate[立即释放]
        MarkSweep --> Deferred[延迟释放]
        
        Immediate --> Free
        Deferred --> Free
    end
```

## 9. 并行处理架构

```mermaid
flowchart TB
    subgraph "任务分解"
        Input[输入任务] --> Analyze[分析依赖]
        Analyze --> Partition[空间分区]
        
        Partition --> Independent[独立任务]
        Partition --> Dependent[依赖任务]
    end
    
    subgraph "任务调度"
        Independent --> TaskQueue[任务队列]
        Dependent --> DAG[依赖图]
        
        TaskQueue --> Scheduler[调度器]
        DAG --> Scheduler
        
        Scheduler --> Distribute[分发任务]
    end
    
    subgraph "执行层"
        Distribute --> T1[线程1]
        Distribute --> T2[线程2]
        Distribute --> T3[线程3]
        Distribute --> T4[线程4]
        
        T1 --> LocalCache1[本地缓存]
        T2 --> LocalCache2[本地缓存]
        T3 --> LocalCache3[本地缓存]
        T4 --> LocalCache4[本地缓存]
    end
    
    subgraph "同步与合并"
        T1 --> Barrier[屏障同步]
        T2 --> Barrier
        T3 --> Barrier
        T4 --> Barrier
        
        Barrier --> Merge[结果合并]
        Merge --> Boundary[边界处理]
        Boundary --> Output[输出结果]
    end
```

## 10. 网格修复流程

```mermaid
stateDiagram-v2
    [*] --> 诊断
    
    state 诊断 {
        [*] --> 拓扑检查
        拓扑检查 --> 几何检查
        
        state 拓扑检查 {
            非流形顶点检测
            非流形边检测
            孤立元素检测
        }
        
        state 几何检查 {
            自相交检测
            退化三角形检测
            法向一致性检测
        }
    }
    
    诊断 --> 修复策略
    
    state 修复策略 {
        [*] --> 优先级排序
        优先级排序 --> 策略选择
        
        state 策略选择 {
            自动修复
            交互修复
            保守修复
        }
    }
    
    修复策略 --> 执行修复
    
    state 执行修复 {
        [*] --> 拓扑修复
        
        state 拓扑修复 {
            复制非流形顶点 --> 分离非流形边
            分离非流形边 --> 移除孤立元素
        }
        
        拓扑修复 --> 几何修复
        
        state 几何修复 {
            分割自相交 --> 合并重复顶点
            合并重复顶点 --> 翻转反向面
            翻转反向面 --> 填充孔洞
        }
    }
    
    执行修复 --> 验证
    
    state 验证 {
        完整性检查 --> 质量检查
        质量检查 --> 性能评估
    }
    
    验证 --> 结果
    结果 --> [*]
```

## 11. 碰撞检测流程

```mermaid
flowchart TD
    subgraph "宽相检测"
        Objects[对象列表] --> BVH[构建BVH]
        BVH --> Pairs[潜在碰撞对]
        
        Pairs --> SAP[扫描剪枝]
        SAP --> Candidates[候选对]
    end
    
    subgraph "窄相检测"
        Candidates --> TriPairs[三角形对]
        
        TriPairs --> SeparatingAxis[分离轴测试]
        SeparatingAxis --> NoCollision1[无碰撞]
        
        TriPairs --> GJK[GJK算法]
        GJK --> Distance[距离计算]
        
        Distance --> Threshold{<阈值?}
        Threshold -->|是| Collision[碰撞]
        Threshold -->|否| NoCollision2[无碰撞]
    end
    
    subgraph "碰撞响应"
        Collision --> Contact[接触点计算]
        Contact --> Normal[法向计算]
        Normal --> Depth[穿透深度]
        
        Depth --> Response[响应策略]
        Response --> Separate[分离]
        Response --> Deform[变形]
    end
    
    subgraph "持续检测"
        Moving[运动对象] --> CCD[连续碰撞检测]
        CCD --> Interpolate[插值轨迹]
        Interpolate --> TOI[碰撞时间]
        TOI --> Resolve[解决碰撞]
    end
```

## 12. 表面参数化流程

```mermaid
flowchart LR
    subgraph "预处理"
        Mesh[输入网格] --> CheckGenus[检查亏格]
        CheckGenus --> Cut{需要切割?}
        Cut -->|是| Seam[创建缝合线]
        Cut -->|否| Boundary[使用边界]
        Seam --> Boundary
    end
    
    subgraph "参数化方法"
        Boundary --> Method{选择方法}
        Method --> Conformal[共形映射]
        Method --> Harmonic[调和映射]
        Method --> LSCM[最小二乘共形]
        Method --> ABF[角度保持]
    end
    
    subgraph "优化"
        Conformal --> Energy[能量函数]
        Harmonic --> Energy
        LSCM --> Energy
        ABF --> Energy
        
        Energy --> Minimize[最小化]
        Minimize --> Iterate[迭代求解]
        Iterate --> Converge{收敛?}
        Converge -->|否| Iterate
        Converge -->|是| Result[参数化结果]
    end
    
    subgraph "后处理"
        Result --> Distortion[失真分析]
        Distortion --> Adjust[调整权重]
        Adjust --> Smooth[平滑UV]
        Smooth --> Pack[UV打包]
    end
```

## 13. 网格平滑算法流程

```mermaid
stateDiagram-v2
    [*] --> 选择算法
    
    state 选择算法 {
        [*] --> 拉普拉斯平滑
        [*] --> 双边滤波
        [*] --> 均值曲率流
        [*] --> Taubin平滑
    }
    
    选择算法 --> 拉普拉斯平滑实现
    
    state 拉普拉斯平滑实现 {
        计算拉普拉斯算子 --> 更新顶点位置
        更新顶点位置 --> 迭代检查
        迭代检查 --> 计算拉普拉斯算子
    }
    
    选择算法 --> 双边滤波实现
    
    state 双边滤波实现 {
        计算空间权重 --> 计算范围权重
        计算范围权重 --> 加权平均
        加权平均 --> 更新位置
    }
    
    选择算法 --> 特征保持
    
    state 特征保持 {
        检测特征边 --> 标记锐边
        标记锐边 --> 约束平滑
        约束平滑 --> 保持特征
    }
    
    拉普拉斯平滑实现 --> 结果评估
    双边滤波实现 --> 结果评估
    特征保持 --> 结果评估
    
    state 结果评估 {
        体积保持检查
        质量度量
        收敛判断
    }
    
    结果评估 --> [*]
```

## 14. 文件IO处理流程

```mermaid
flowchart TD
    subgraph "加载流程"
        File[文件路径] --> Detect[检测格式]
        Detect --> Parser{选择解析器}
        
        Parser --> STL[STL解析器]
        Parser --> OBJ[OBJ解析器]
        Parser --> PLY[PLY解析器]
        Parser --> GLTF[GLTF解析器]
        
        STL --> ReadHeader[读取头部]
        OBJ --> ReadHeader
        PLY --> ReadHeader
        GLTF --> ReadHeader
        
        ReadHeader --> Stream[流式读取]
        Stream --> Parse[解析数据]
        Parse --> Validate[验证数据]
        Validate --> Build[构建网格]
    end
    
    subgraph "保存流程"
        MeshData[网格数据] --> Format{选择格式}
        
        Format --> BinaryFormat[二进制格式]
        Format --> TextFormat[文本格式]
        
        BinaryFormat --> Compress[压缩]
        TextFormat --> Encode[编码]
        
        Compress --> WriteHeader[写入头部]
        Encode --> WriteHeader
        
        WriteHeader --> WriteData[写入数据]
        WriteData --> Flush[刷新缓冲]
        Flush --> Close[关闭文件]
    end
    
    subgraph "错误处理"
        Parse --> Error{错误?}
        Error -->|是| HandleError[错误处理]
        HandleError --> Recover[恢复策略]
        Recover --> Fallback[降级处理]
        Error -->|否| Build
    end
```

## 15. 性能分析工具集成

```mermaid
graph TB
    subgraph "性能采集"
        Code[代码执行] --> Timer[计时器]
        Code --> Counter[计数器]
        Code --> Memory[内存追踪]
        
        Timer --> Profile[性能剖析]
        Counter --> Profile
        Memory --> Profile
    end
    
    subgraph "数据聚合"
        Profile --> Aggregate[聚合统计]
        Aggregate --> CallGraph[调用图]
        Aggregate --> HotPath[热点路径]
        Aggregate --> MemLeak[内存泄漏]
    end
    
    subgraph "可视化"
        CallGraph --> FlameGraph[火焰图]
        HotPath --> Timeline[时间线]
        MemLeak --> HeapMap[堆栈图]
        
        FlameGraph --> Report[报告生成]
        Timeline --> Report
        HeapMap --> Report
    end
    
    subgraph "优化建议"
        Report --> Analysis[分析瓶颈]
        Analysis --> Suggest[优化建议]
        Suggest --> Vectorize[向量化]
        Suggest --> Parallelize[并行化]
        Suggest --> CacheOpt[缓存优化]
    end
```

---

## 总结

这些流程图详细展示了MRMesh库的核心架构和算法实现流程。每个流程图都反映了实际代码中的设计思路和执行逻辑：

1. **模块化设计**：各组件职责明确，相互独立
2. **分层架构**：从应用层到基础设施层的清晰分层
3. **算法优化**：每个算法都经过精心设计和优化
4. **并行处理**：充分利用多核处理器能力
5. **错误处理**：完善的错误检测和恢复机制

这些流程图可以帮助开发者：
- 理解系统整体架构
- 掌握核心算法流程
- 进行性能优化
- 排查和解决问题
- 扩展新功能

---

*本文档使用Mermaid语法编写，可在支持Mermaid的Markdown查看器中直接渲染为图表。*