**标题**
- 在 MeshLib 中用 SDF +（双）Marching Cubes 稳定 PushCutter 刀路

**范围**
- 说明如何在 `source/MRVoxels` 以有符号距离场（SDF）与等值面重建（Marching Cubes/DualMC）提升 PushCutter 类刀路规划的数值稳定性。
- 将概念映射到 MeshLib 现有组件，并提出最小改造与扩展集成两种路径。
- 给出算法流程、伪代码、参数建议、精度/性能分析与验证方案。

**背景**
- PushCutter（推刀）在三轴 CAM 中通过移动刀心，使刀具恰好“贴住”工件而不碰撞；传统做法是在三角网格上做刀具-三角面/边/顶接触求解，并在每个 (x, y) 求安全高度 Z，接触特征在边界切换时易产生数值抖动。
- SDF φ(p) 提供到工件的连续符号距离；对球刀/球角刀，配置空间障碍边界可视为 φ(p) = R（R 为球半径）。在该等值面上或其外侧规划刀心路径，可避免离散接触特征切换带来的不连续。
- MeshLib 已具备体素化、SDF 与 MC/DualMC 重建能力，并内置等高/蕾丝/恒纹理等刀路模块，可直接复用。

**现有能力（MeshLib）**
- 体素/SDF 与转换
  - `MRMeshToDistanceVolume`、`MRPointsToDistanceVolume`、`MRFloatGrid`、OpenVDB（`MROpenVDB*`、`MRVDBFloatGrid.h`）。
  - 体积插值工具（`MRVolumeInterpolation.h`）。
- 偏置/等值面重建
  - `offsetMesh`（DualMC，闭合输出）、`mcOffsetMesh`（标准MC）、`sharpOffsetMesh`（锐化）。
  - 倒扣处理：`FixUndercuts::fix`（`MRToolPath.cpp` 的 `preprocessMesh()` 已使用）。
- 刀路与后处理
  - `constantZToolPath`、`lacingToolPath`、`constantCuspToolPath`。
  - 抽切与插补：`extractPlaneSections`、`interpolateLines`、`interpolateArcs`。
  - G-code 导出：`exportToolPathToGCode`。

**总体策略**
- 两条可落地、改动最小的稳定化路径：
- 路径A（不新增模块）：先偏置后规划
  - 以 R（球/球角半径）对原始网格做正向偏置（DualMC），得到闭合、稳定的配置空间边界。
  - 在该偏置曲面上用现有 `constantZ/ lacing / constantCusp` 规划刀心。
  - 本质上等价于在 φ(p)=R 上规划 PushCutter（球形刀具）。
- 路径B（新增模块）：直接基于 SDF 的高度求解
  - 构建工件 SDF（OpenVDB 稀疏或致密网格），对每个 (x, y) 解 φ(x, y, z) = R，取满足约束的最高 z，得到 z_safe(x, y)。
  - 由 z_safe 生成扫描线/等高线，复用 `interpolateLines/Arcs`。

**算法A — 偏置曲面 + 现有刀路**
- 输入
  - `MeshPart mp`，`ToolPathParams params`。
  - R = `params.millRadius`（球/球角刀）。平底/复杂刀具见“刀具建模”。
- 步骤
  - 偏置
    - 调用 `offsetMesh(mp, +R, OffsetParameters{ voxelSize, signDetectionMode, ... })` 得 `offsetMeshOut`。
    - 优先 DualMC（`offsetMesh`）以获得更稳的三角质量；`mcOffsetMesh` 为备选。
  - 预处理
    - 可选倒扣清理：`FixUndercuts::fix`（`preprocessMesh` 已集成）。
    - 设置 `params.offsetMesh = &MeshPart{ offsetMeshOut, region? }`。
  - 规划
    - 任选：
      - `constantZToolPath(mp, params)`（自动使用 `params.offsetMesh`）。
      - `lacingToolPath(mp, params, Axis::X/Y)`。
      - `constantCuspToolPath(mp, ConstantCuspParams{...})`。
- 输出
  - `ToolPathResult`（含 `commands`、`modifiedMesh`、`modifiedRegion`）。
- 说明
  - `params.flatTool = false` 表示按球形偏置处理。平底/圆角刀见“刀具建模”。
  - `voxelSize` 决定几何公差；稳定性通常需 `voxelSize ≈ tol/3 .. tol/4`。
  - 恒纹理（constant cusp）建议 `sectionStep ≥ 1.2 * voxelSize`（头文件已有提示）。

**算法B — 直接 SDF PushCutter（建议新建 `MRPushCutter`）**
- 目标
  - 用 SDF 根求解获得 z_safe(x, y)，再生成刀路。
- 体积构建
  - 在稀疏（OpenVDB）或致密网格上构建 φ：
    - 空间范围：模型包围盒外扩 ≥ R 与安全高度裕量。
    - 分辨率：按目标公差选 `voxelSize`；构建 φ 近 R 的窄带以省内存。
- 高度求解
  - 固定 y = y0 的扫描线上，按步距 `dx` 采样 x：
    - 定义 z ∈ [z_min, z_max]（例如包围盒范围）。
    - 求满足 φ(x, y0, z) ≥ R 的最高 z；对球刀等价解 φ(x, y0, z) = R 的单调根。
    - 采用二分（单调、鲁棒）或配合梯度的牛顿/割线沿 +z 加速；每次查询用三线性插值（`MRVolumeInterpolation`）。
- 刀路生成
  - 由 z_safe 序列构造折线：
    - 蕾丝（lacing）：交替扫描方向；当跨越距离超过 `critTransitionLength` 时走安全高度过渡。
    - 等高（constant-Z）：可直接切 `φ=R` 表面获得等高线，也可对 z_safe 做阈值等值提取。
  - 用 `interpolateLines` / `interpolateArcs` 紧凑为 G1/G2/G3。
- 伪代码（二分示意）
  - 对每个扫描点 (x, y)：
    - `lo = z_min; hi = z_max;`
    - 迭代 N ≈ log2((z_max−z_min)/epsZ)：
      - `mid = 0.5*(lo+hi); v = φ(x,y,mid);`
      - 若 `v >= R` 则 `lo = mid`，否则 `hi = mid`。
    - `z_safe = lo`。
  - 按 `lacingToolPath` 思路组装过渡与进给。

**刀具建模**
- 球刀 / 球角刀
  - 取 R 为球半径；算法A严格等价于在 `φ=R` 上规划（即推刀）。
- 平底立铣刀（圆柱侧 + 平底）
  - 严格做法：与完整刀具几何 S_tool 做 Minkowski 和（SDF 形式为形态学差 φ ⊖ S_tool）。
    - 底面约束：φ 与底面圆盘的组合，保证 z ≥ z_bottom(x, y)。
    - 侧面约束：对半径 Rc 的圆柱侧保持径向间隙，可通过 `φ ≥ Rc` 并配合坡度限额近似。
  - 最小实现：用刀具圆角半径作球形近似（保守），遇陡直壁时减小步距/步深提高保守性。
- 圆角刀（Bull-nose/Filleted）
  - 以圆角半径进行 `φ=R` 近似；若要求严格，再叠加圆柱侧约束或使用完整刀具 SDF 合成。

**参数与默认**
- `voxelSize`
  - 依据目标公差 `tol`：建议 `voxelSize = tol / 3` 起步。
- `signDetectionMode`
  - 闭合网格优先非 Unsigned（OpenVDB 或 HoleWindingRule）。开口网格需要 Unsigned 以得到开口结果。
- 重建选择
  - 首选 `offsetMesh`（DualMC）；必要时调 `SharpOffsetParameters` 相关参数减少锯齿。
- 采样/切片
  - 建议 `sectionStep ≥ 1.2 × voxelSize`（恒纹理）。
  - `critTransitionLength` 控制何时走 `safeZ` 过渡，经验值为步距的 5–10 倍。
- 进给
  - `plungeLength`/`retractLength` 限定靠近工件的慢速段；配合 `plungeFeed`/`retractFeed`。

**数值稳定性**
- SDF 的优势
  - φ 连续可插值，接触化为标量方程根，避免面/边/顶组合切换导致的高度突变。
- MC/DualMC 选择
  - DualMC 三角质量更好、拓扑更稳，切片等高时更少锯齿。
- 平滑（可选）
  - 对 z_safe 做窗口 ≤ 1 体素的小幅平滑可降噪，但需保证不超公差；谨慎使用。

**性能考量**
- 体积代价
  - OpenVDB 稀疏+窄带可显著降内存。
- 高度求解
  - 每点约 O(log(range/epsZ)) 次 SDF 采样；可按扫描线并行、向量化。
- 偏置曲面规划
  - 一次偏置 + 等值切片通常比传统 PushCutter 的反复三角距离求解更高效。

**与 MeshLib 的集成（代码指引）**
- 预处理与偏置
  - `MRToolPath.cpp` 的 `preprocessMesh()` 在 `params.flatTool == false` 时已调用 `offsetMesh` 与 `FixUndercuts`。
  - 也可外部先算偏置并设置 `params.offsetMesh`，强制在给定曲面上规划。
- 切片与等值线
  - `extractAllSections()` 封装 `extractPlaneSections()` 并支持 `BypassDirection`。
- 刀路生成
  - `constantZToolPath`、`lacingToolPath`、`constantCuspToolPath` 输出 `GCommand`。
- 后处理
  - `interpolateLines`、`interpolateArcs` 将折线压缩为 G1/G2/G3。

**最小改造落地**
- 外部偏置
  - 调用 `offset = offsetMesh(mp, params.millRadius, { voxelSize = tol/3, ... })`。
  - 传入 `params.offsetMesh` 后再调 `constantZToolPath/lacingToolPath/constantCuspToolPath`。
- 验证
  - 对比在原网格 vs. 偏置网格上的规划：观察 Z 抖动与段数；预期偏置版本更平滑稳定。

**扩展落地（新增模块 `MRPushCutter`）**
- 文件位置
  - 头文件：`source/MRVoxels/MRPushCutter.h`
  - 源文件：`source/MRVoxels/MRPushCutter.cpp`
  - 由 `source/MRVoxels/CMakeLists.txt` 中的 `file(GLOB ...)` 自动纳入编译。
- API（已实现）
  - `MRVOXELS_API Expected<ToolPathResult> pushCutterToolPath(const MeshPart& mp, const PushCutterParams& params);`
  - `struct PushCutterParams : ToolPathParams`（新增字段）：
    - `PushCutterSolver solver { OffsetSurface | DirectSDF }`（默认 `OffsetSurface`）
    - `PushCutterPattern pattern { Lacing | ConstantZ | ConstantCusp }`（默认 `Lacing`）
    - `Axis lacingDirection`（用于 Lacing，默认 `Axis::X`）
    - `bool constantCuspFromCenterToBoundary`（恒纹理方向，默认 `true`）
    - 预留 SDF 参数：`sdfVoxelSize`、`sdfBandHalfWidth`、`zSolveEps`
- 行为
  - 当 `solver == OffsetSurface`：内部直接调用现有 `lacingToolPath / constantZToolPath / constantCuspToolPath` 在偏置曲面（或 `params.offsetMesh`）上规划。
  - 当 `solver == DirectSDF`：当前返回未实现错误（占位）。
- 使用示例
  - 蕾丝粗加工（球刀）：
    - `PushCutterParams p; p.millRadius=5; p.voxelSize=0.25; p.sectionStep=0.8; p.safeZ=100;`
    - `p.pattern=PushCutterPattern::Lacing; p.lacingDirection=Axis::X;`
    - `auto r = pushCutterToolPath(mp, p);`
  - 等高精加工：
    - `p.pattern=PushCutterPattern::ConstantZ;` 其余同上。
  - 恒纹理：
    - `p.pattern=PushCutterPattern::ConstantCusp; p.constantCuspFromCenterToBoundary=true;`
- 后续计划（DirectSDF）
  - 在 `DirectSDF` 下接入：SDF 构建（OpenVDB/致密栅格）、z_safe 二分/牛顿求解、扫描线生成，复用 G-command 插补与过渡。

**精度与公差**
- 误差来源
  - 体素化/SDF 离散（≈ O(voxelSize)）、MC/DualMC 网格化误差、插补近似误差。
- 建议
  - 设定 `voxelSize ≤ tol/3`；优先 DualMC；`interpolateLines/Arcs` 的 eps ≤ tol；
  - 平底/圆角刀若用球形近似，建议留少量余量供精加工。

**边界情形**
- 薄壁或特征厚度 < 2×`voxelSize` 可能在 SDF/MC 中消失或粘连：减小 `voxelSize` 或标记禁切区。
- 近垂直壁与平底刀：若未采用完整刀具 SDF，需减小步距并增加验证路径。
- 开口网格 + Unsigned：偏置表面为开口；运动规划需特别处理安全抬刀过渡。

**验证方案**
- 单元级
  - 解析几何体（平面/球/圆柱）上的 z 求解单调性与收敛；
  - 等值线抽取对 `voxelSize` 的稳定性。
- CAM 对比指标
  - 平滑度：相邻点 ΔZ 方差；
  - 段数与圆弧比；
  - 碰撞安全：沿路径抽样验证 φ(刀心) − R ≥ 0。
- 回归
  - 固定随机源；对比 G-code 差异受控在公差带内。

**操作建议**
- 球刀粗加工
  - `voxelSize = 0.25 mm`、`millRadius = 5 mm`、`sectionStep = 0.5–1.0 mm`；在偏置曲面规划。
- 恒纹理精加工
  - `voxelSize = cusp_tol/3`、`sectionStep` 依据纹理模型；在偏置曲面运行 `constantCuspToolPath`。

**参考**
- Anders Wallin CAM/PushCutter 笔记：https://www.anderswallin.net/cam/
- 代码位置：`source/MRVoxels/MRToolPath.cpp`、`MROffset.h`、`MRMarchingCubes.*`、`MRFloatGrid.*`。

**变更记录**
- v1.0（本文件）：首版集成指南与算法说明。
