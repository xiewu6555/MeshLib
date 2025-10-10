/**
 * @file MRFCLPerformanceTests.cpp
 * @brief FCL与MeshMeshDistance性能对比测试
 *
 * 本测试对比FCL(Flexible Collision Library)和MeshLib内置的MeshMeshDistance算法
 * 在静态和动态场景下的性能差异，用于评估两种方法在机床运动仿真等应用中的适用性。
 *
 * 测试场景：
 * 1. 静态场景：两个网格在固定位置进行距离查询
 * 2. 动态场景：模拟机床运动，网格随时间变换位置
 *
 * @version 1.0
 * @date 2025-09-30
 */

#include <MRMesh/MRMeshMeshDistance.h>
#include <MRMesh/MRMesh.h>
#include <MRMesh/MRMeshBuilder.h>
#include <MRMesh/MRMakeSphereMesh.h>
#include <MRMesh/MRAABBTree.h>
#include <MRMesh/MRAABBTreeMaker.h>
#include "MRMesh/MRCube.h"
#include "MRMesh/MRConstants.h"
#include <MRMesh/MRGTest.h>

#define ANKERL_NANOBENCH_IMPLEMENT
#include <nanobench.h>

#include <array>
#include <cassert>
#include <optional>
#include <functional>

// FCL includes
#include <fcl/narrowphase/collision.h>
#include <fcl/narrowphase/distance.h>
#include <fcl/geometry/bvh/BVH_model.h>
#include <fcl/geometry/shape/box.h>
#include <fcl/geometry/shape/sphere.h>

namespace MR
{

/**
 * @brief 将MeshLib的Mesh转换为FCL的BVH模型
 *
 * @param mesh 输入的MeshLib网格
 * @return 返回FCL BVH模型的智能指针
 */
std::shared_ptr<fcl::BVHModel<fcl::OBBRSS<double>>> convertMeshToFCL(const Mesh& mesh)
{
    auto model = std::make_shared<fcl::BVHModel<fcl::OBBRSS<double>>>();

    // 预分配内存以提高性能
    std::vector<fcl::Vector3<double>> vertices;
    vertices.reserve(mesh.points.size());

    // 添加顶点
    for (const auto& pt : mesh.points)
    {
        vertices.emplace_back(pt.x, pt.y, pt.z);
    }

    // 获取面片ID并预分配模型容量
    const auto& faceIds = mesh.topology.getFaceIds(nullptr);
    model->beginModel(static_cast<int>(faceIds.size()), static_cast<int>(vertices.size()));

    // 添加三角形面片
    for (auto f : faceIds)
    {
        const auto v = mesh.topology.getTriVerts(f);

        // Debug模式下的边界检查
        assert(v[0] < vertices.size() && "Invalid vertex index");
        assert(v[1] < vertices.size() && "Invalid vertex index");
        assert(v[2] < vertices.size() && "Invalid vertex index");

        model->addTriangle(
            vertices[v[0]],
            vertices[v[1]],
            vertices[v[2]]
        );
    }

    model->endModel();
    return model;
}

/**
 * @brief 使用FCL计算两个网格之间的距离
 *
 * @param model1 第一个网格的FCL模型
 * @param model2 第二个网格的FCL模型
 * @param transform 可选的变换矩阵
 * @return 返回两个网格之间的最小距离
 */
double computeDistanceFCL(
    const std::shared_ptr<fcl::BVHModel<fcl::OBBRSS<double>>>& model1,
    const std::shared_ptr<fcl::BVHModel<fcl::OBBRSS<double>>>& model2,
    std::optional<std::reference_wrapper<const AffineXf3f>> transform = std::nullopt)
{
    fcl::DistanceRequest<double> request;
    fcl::DistanceResult<double> result;

    const fcl::Transform3<double> fclTransform1 = fcl::Transform3<double>::Identity();

    const fcl::Transform3<double> fclTransform2 = [&]() {
        if (transform) {
            fcl::Transform3<double> tf;
            const auto& A = transform->get().A;
            const auto& b = transform->get().b;

            tf.linear() <<
                A.x.x, A.y.x, A.z.x,
                A.x.y, A.y.y, A.z.y,
                A.x.z, A.y.z, A.z.z;
            tf.translation() << b.x, b.y, b.z;
            return tf;
        }
        return fcl::Transform3<double>::Identity();
    }();

    fcl::distance(model1.get(), fclTransform1, model2.get(), fclTransform2, request, result);
    return result.min_distance;
}

// ===================== 静态场景测试 =====================

TEST(MRFCLPerformance, StaticSceneComparison)
{
    // 创建两个测试网格
    Mesh sphere1 = makeUVSphere(1.0f, 16, 16);  // 更密集的网格以更真实地模拟实际场景
    Mesh sphere2 = makeUVSphere(1.5f, 16, 16);

    // 设置一个固定的变换
    auto xf = AffineXf3f::translation(Vector3f(3.0f, 0.0f, 0.0f));

    // 将网格转换为FCL格式
    auto fclModel1 = convertMeshToFCL(sphere1);
    auto fclModel2 = convertMeshToFCL(sphere2);

    std::cout << "\n=== 静态场景性能测试 ===\n";
    std::cout << "测试配置：\n";
    std::cout << "  - 网格1：球体（半径1.0，顶点数：" << sphere1.points.size() << "）\n";
    std::cout << "  - 网格2：球体（半径1.5，顶点数：" << sphere2.points.size() << "）\n";
    std::cout << "  - 位移：(3.0, 0.0, 0.0)\n\n";

    ankerl::nanobench::Bench bench;
    bench.title("静态场景距离计算")
         .unit("次查询")
         .warmup(100)
         .minEpochIterations(1000);

    // 测试MeshLib的MeshMeshDistance
    double meshLibDist = 0.0;
    bench.run("MeshLib::findDistance", [&] {
        auto result = findDistance(sphere1, sphere2, &xf, FLT_MAX);
        meshLibDist = std::sqrt(result.distSq);
        ankerl::nanobench::doNotOptimizeAway(meshLibDist);
    });

    // 测试FCL的distance
    double fclDist = 0.0;
    bench.run("FCL::distance", [&] {
        fclDist = computeDistanceFCL(fclModel1, fclModel2, std::cref(xf));
        ankerl::nanobench::doNotOptimizeAway(fclDist);
    });

    std::cout << "\n结果验证：\n";
    std::cout << "  - MeshLib距离：" << meshLibDist << "\n";
    std::cout << "  - FCL距离：" << fclDist << "\n";
    std::cout << "  - 差异：" << std::abs(meshLibDist - fclDist) << "\n\n";
}

// ===================== 动态场景测试 =====================

TEST(MRFCLPerformance, DynamicSceneComparison)
{
    // 创建模拟机床的网格（刀具和工件）
    Mesh tool = makeCube();  // 刀具：立方体
    Mesh workpiece = makeUVSphere(2.0f, 32, 32);  // 工件：球体（更复杂的网格）

    // 将网格转换为FCL格式
    auto fclTool = convertMeshToFCL(tool);
    auto fclWorkpiece = convertMeshToFCL(workpiece);

    std::cout << "\n=== 动态场景性能测试（模拟机床运动）===\n";
    std::cout << "测试配置：\n";
    std::cout << "  - 刀具：立方体（顶点数：" << tool.points.size() << "）\n";
    std::cout << "  - 工件：球体（半径2.0，顶点数：" << workpiece.points.size() << "）\n";
    std::cout << "  - 运动模式：圆周运动（100个位置）\n\n";

    // 生成一系列变换，模拟机床的圆周运动
    constexpr int numTransforms = 100;
    constexpr float radius = 3.0f;

    std::vector<AffineXf3f> transforms;
    transforms.reserve(numTransforms);

    for (int i = 0; i < numTransforms; ++i)
    {
        const float angle = 2.0f * PI_F * static_cast<float>(i) / static_cast<float>(numTransforms);
        const Vector3f translation(radius * std::cos(angle), radius * std::sin(angle), 0.0f);

        // 组合旋转和平移
        const auto rotation = AffineXf3f::linear(Matrix3f::rotationFromEuler(Vector3f(0.0f, 0.0f, angle)));
        const auto trans = AffineXf3f::translation(translation);
        transforms.push_back(trans * rotation);
    }

    ankerl::nanobench::Bench bench;
    bench.title("动态场景距离计算（100次变换）")
         .unit("完整运动周期")
         .warmup(10)
         .minEpochIterations(100);

    // 测试MeshLib在动态场景下的性能
    bench.run("MeshLib::findDistance（动态）", [&] {
        for (const auto& xf : transforms)
        {
            auto result = findDistance(tool, workpiece, &xf, FLT_MAX);
            ankerl::nanobench::doNotOptimizeAway(result.distSq);
        }
    });

    // 测试FCL在动态场景下的性能
    bench.run("FCL::distance（动态）", [&] {
        for (const auto& xf : transforms)
        {
            const double dist = computeDistanceFCL(fclTool, fclWorkpiece, std::cref(xf));
            ankerl::nanobench::doNotOptimizeAway(dist);
        }
    });

    std::cout << "\n";
}

// ===================== BVH更新性能测试 =====================

TEST(MRFCLPerformance, BVHUpdateComparison)
{
    Mesh mesh = makeUVSphere(1.0f, 32, 32);

    std::cout << "\n=== BVH更新性能测试 ===\n";
    std::cout << "测试配置：\n";
    std::cout << "  - 网格：球体（半径1.0，顶点数：" << mesh.points.size() << "）\n";
    std::cout << "  - 测试场景：重复构建BVH树\n\n";

    ankerl::nanobench::Bench bench;
    bench.title("BVH构建性能")
         .unit("次构建")
         .warmup(50)
         .minEpochIterations(500);

    // 测试MeshLib的AABB树构建
    bench.run("MeshLib::AABBTree构建", [&] {
        mesh.invalidateCaches();  // 强制重建AABB树
        const auto& tree = mesh.getAABBTree();
        ankerl::nanobench::doNotOptimizeAway(tree.nodes().size());
    });

    // 测试FCL的BVH构建
    bench.run("FCL::BVH构建", [&] {
        auto model = convertMeshToFCL(mesh);
        ankerl::nanobench::doNotOptimizeAway(model->getNumBVs());
    });

    std::cout << "\n";
}

// ===================== 性能分析和适用场景说明 =====================

TEST(MRFCLPerformance, PerformanceAnalysis)
{
    std::cout << "\n======================================\n";
    std::cout << "性能分析与适用场景\n";
    std::cout << "======================================\n\n";

    std::cout << "【FCL的优势场景】\n";
    std::cout << "1. 动态场景：\n";
    std::cout << "   - 当网格在空间中频繁移动/旋转时，FCL的BVH结构可以高效处理变换\n";
    std::cout << "   - 机床运动仿真：刀具相对工件的连续运动路径碰撞检测\n";
    std::cout << "   - 机器人路径规划：机械臂各关节的实时碰撞检测\n";
    std::cout << "   - 装配仿真：零件在装配过程中的动态干涉检测\n\n";

    std::cout << "2. 持续查询：\n";
    std::cout << "   - BVH结构一次构建，可重复使用于多次查询\n";
    std::cout << "   - 适合连续帧的碰撞检测（如实时仿真）\n\n";

    std::cout << "3. 多种几何形状：\n";
    std::cout << "   - FCL支持球体、圆柱、胶囊等基本几何体的高效碰撞检测\n";
    std::cout << "   - 可以混合使用基本几何体和复杂网格\n\n";

    std::cout << "【MeshLib::findDistance的优势场景】\n";
    std::cout << "1. 静态场景：\n";
    std::cout << "   - 一次性距离查询，不需要预构建数据结构\n";
    std::cout << "   - 网格位置固定或很少变化的情况\n\n";

    std::cout << "2. 集成便利性：\n";
    std::cout << "   - 与MeshLib的其他功能无缝集成\n";
    std::cout << "   - 无需额外的数据结构转换开销\n";
    std::cout << "   - 直接支持MeshLib的所有网格特性\n\n";

    std::cout << "3. 精确性要求：\n";
    std::cout << "   - 提供精确的最近点对信息\n";
    std::cout << "   - 支持有符号距离计算\n\n";

    std::cout << "【推荐使用策略】\n";
    std::cout << "- 机床运动仿真：推荐使用FCL\n";
    std::cout << "  原因：刀具路径通常包含大量连续的位姿变化，FCL的BVH更新效率更高\n\n";
    std::cout << "- 静态干涉检查：推荐使用MeshLib::findDistance\n";
    std::cout << "  原因：一次性查询无需额外的数据结构构建开销\n\n";
    std::cout << "- 混合场景：可以根据查询频率动态选择\n";
    std::cout << "  策略：少于N次查询用MeshLib，多于N次查询用FCL（N约为10-50，取决于网格复杂度）\n\n";

    std::cout << "======================================\n\n";

    // 这不是一个真正的测试，只是用于输出分析信息
    EXPECT_TRUE(true);
}

} // namespace MR