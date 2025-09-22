#include "MRPushCutter.h"

#include "MRToolPath.h"
#include "MROffset.h"

namespace MR
{

static ConstantCuspParams makeCuspParamsFrom( const PushCutterParams& p )
{
    ConstantCuspParams cp;
    // 继承 ToolPathParams 字段
    cp.millRadius = p.millRadius;
    cp.voxelSize = p.voxelSize;
    cp.sectionStep = p.sectionStep;
    cp.critTransitionLength = p.critTransitionLength;
    cp.plungeLength = p.plungeLength;
    cp.retractLength = p.retractLength;
    cp.plungeFeed = p.plungeFeed;
    cp.retractFeed = p.retractFeed;
    cp.baseFeed = p.baseFeed;
    cp.safeZ = p.safeZ;
    cp.bypassDir = p.bypassDir;
    cp.xf = p.xf;
    cp.flatTool = p.flatTool;
    cp.cb = p.cb;
    cp.toolpathExpansion = p.toolpathExpansion;
    cp.isolines = p.isolines;
    cp.startContours = p.startContours;
    cp.startVertices = p.startVertices;
    cp.offsetMesh = p.offsetMesh;
    // ConstantCusp 专有
    cp.fromCenterToBoundary = p.constantCuspFromCenterToBoundary;
    return cp;
}

Expected<ToolPathResult> pushCutterToolPath( const MeshPart& mp, const PushCutterParams& params )
{
    // 方案一：在偏置曲面上规划（可用）
    if ( params.solver == PushCutterSolver::OffsetSurface )
    {
        switch ( params.pattern )
        {
        case PushCutterPattern::Lacing:
            return lacingToolPath( mp, params, params.lacingDirection );
        case PushCutterPattern::ConstantZ:
            return constantZToolPath( mp, params );
        case PushCutterPattern::ConstantCusp:
        {
            auto cp = makeCuspParamsFrom( params );
            return constantCuspToolPath( mp, cp );
        }
        default:
            break;
        }
    }

    // 方案二：直接 SDF 求解（占位）
    return unexpected( std::string( "DirectSDF solver is not implemented yet" ) );
}

}

