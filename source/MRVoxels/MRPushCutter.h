#pragma once

#include "MRVoxelsFwd.h"
#include "MRToolPath.h"

#include "MRMesh/MRMeshPart.h"
#include "MRMesh/MRExpected.h"
#include "MRMesh/MRProgressCallback.h"

namespace MR
{

// 推刀（PushCutter）求解器类型
enum class PushCutterSolver
{
    // 通过在偏置曲面（φ = R 等值面）上规划刀心路径实现（依赖 offset + 现有刀路）
    OffsetSurface = 0,
    // 直接在 SDF 上按 (x,y) 求解 z_safe（预留：尚未实现）
    DirectSDF = 1
};

// 刀路样式（与现有生成器一致）
enum class PushCutterPattern
{
    Lacing = 0,
    ConstantZ = 1,
    ConstantCusp = 2
};

// PushCutter 参数（在 ToolPathParams 基础上扩展）
struct PushCutterParams : ToolPathParams
{
    // 选择求解器
    PushCutterSolver solver = PushCutterSolver::OffsetSurface;
    // 刀路样式
    PushCutterPattern pattern = PushCutterPattern::Lacing;
    // 用于 Lacing 的切向方向
    Axis lacingDirection = Axis::X;
    // 恒纹理方向：从中心到边界（仅 ConstantCusp 使用）
    bool constantCuspFromCenterToBoundary = true;

    // 直接 SDF 求解相关（预留）
    float sdfVoxelSize = 0.0f;      // 体素尺寸（世界单位）
    float sdfBandHalfWidth = 0.0f;  // 窄带半宽（世界单位）
    float zSolveEps = 0.0f;         // z 求解容差
};

// 生成推刀风格刀路
// - OffsetSurface：调用现有刀路生成器在偏置曲面上规划
// - DirectSDF：预留，未来直接基于 SDF 的 z_safe 求解
MRVOXELS_API Expected<ToolPathResult> pushCutterToolPath( const MeshPart& mp, const PushCutterParams& params );

}

