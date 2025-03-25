#include "MRToolPathPlugin.h"
#include <array> // 显式包含array头文件
#include "../../build/MRViewer/MRViewerPluginsList.h"  // 使用正确的相对路径
#include "MRViewer.h"
#include "MRViewer/MRRibbonConstants.h"
#include "MRViewer/MRViewer.h"
#include "MRViewer/MRViewport.h"
#include "MRViewer/ImGuiHelpers.h"
#include "MRViewer/MRFileDialog.h"
#include "MRMesh/MRPolyline.h"
#include "MRMesh/MRObject.h"
#include "MRMesh/MRObjectsAccess.h"
#include "MRMesh/MRSceneRoot.h"
#include "MRMesh/MRBox.h"
#include "MRMesh/MRMesh.h"
#include "MRMesh/MRMeshBuilder.h"
#include "MRMesh/MRMeshNormals.h"
#include "MRMesh/MRTimer.h"
#include "MRMesh/MRFixSelfIntersections.h" // 添加自相交修复头文件
#include "MRIOExtras/MRStep.h"
#include "MRPch/MRSpdlog.h"
#include "MRPch/MRFmt.h"
#include "MRMesh/MRMeshDecimate.h"  // 添加网格简化头文件
#include "MRViewer/MRViewer.h"
#include "MRViewer/MRViewerSettingsManager.h"
#include "MRViewer/MRAppendHistory.h"
#include "MRMesh/MRObjectMesh.h"
#include "MRMesh/MRBitSet.h"
#include "MRMesh/MRMeshFixer.h" // 添加网格修复头文件
#include "MRMesh/MRMeshBuilder.h"
#include "MRMesh/MRVector3.h"
#include "MRMesh/MRMatrix3.h"
#include "MRMesh/MRMakeSphereMesh.h"
#include "MRMesh/MRCylinder.h"
#include "MRMesh/MRTorus.h"
#include "MRMesh/MRMeshFwd.h" // 包含ThreeVertIds的定义
#include "MRMesh/MRDisk.h" // 添加对新创建的MRDisk.h的引用

#include <chrono>
#include <fstream>
#include <iomanip>
#include <imgui.h>
#include <misc/cpp/imgui_stdlib.h>
#include <iostream>
#include <sstream>
#include <filesystem>
#include <cmath>
#include <fmt/format.h>
#include <algorithm>
#include <ctime>
#include <unordered_map>

// 如果M_PI_2未定义，添加定义
#ifndef M_PI_2
#define M_PI_2 1.57079632679489661923f
#endif

// 使用MR命名空间中的类型
using MR::Vector3f;    // 使用MR::Vector3f类型
using MR::VertId;      // 使用MR::VertId类型

// 如果有问题，我们可以创建一个本地的ThreeVertIds类型作为替代
// 但是要注意，这只是临时解决方案，应该使用真正的MR::ThreeVertIds
#ifndef USE_MR_THREE_VERT_IDS
#define USE_MR_THREE_VERT_IDS 1
#endif

#if !USE_MR_THREE_VERT_IDS
// 本地定义，用于替代MR::ThreeVertIds
struct LocalThreeVertIds {
    VertId v[3];
    
    VertId& operator[](int i) { return v[i]; }
    const VertId& operator[](int i) const { return v[i]; }
    
    // 添加其他必要的操作符和方法...
};
using ThreeVertIds = LocalThreeVertIds;
#else
using MR::ThreeVertIds; // 使用MR::ThreeVertIds类型
#endif

// 临时解决方案 - 直接在此定义makeDisk函数
namespace MR {
// 如果没有导入正确的头文件，这里提供内联实现
inline Mesh makeDisk(const Vector3f& center, const Vector3f& normal, float radius, int resolution) {
    Mesh result;
    
    // 计算垂直于法线的两个向量
    auto normalNorm = normal.normalized();
    auto perpVectors = normalNorm.perpendicular();
    Vector3f dirX = perpVectors.first;
    Vector3f dirY = perpVectors.second;
    
    // 创建点
    std::vector<Vector3f> points;
    points.reserve(resolution + 1);
    
    // 添加中心点
    points.push_back(center);
    
    // 添加周围的点
    for (int i = 0; i < resolution; ++i) {
        float angle = 2.0f * static_cast<float>(MR::PI) * static_cast<float>(i) / static_cast<float>(resolution);
        Vector3f pos = center + radius * (static_cast<float>(std::cos(angle)) * dirX + static_cast<float>(std::sin(angle)) * dirY);
        points.push_back(pos);
    }
    
    // 设置点
    result.points = std::move(points);
    
    // 创建三角形
    Triangulation triangulation;
    for (int i = 0; i < resolution; ++i) {
        int next = (i + 1) % resolution;
        ThreeVertIds triangle;
        triangle[0] = VertId(0);  // 中心点
        triangle[1] = VertId(i + 1);
        triangle[2] = VertId(next + 1);
        triangulation.push_back(triangle);
    }
    
    result.topology = MeshBuilder::fromTriangles(triangulation);
    return result;
}
}

// 简易网格构建器类定义
class MeshBuilder {
private:
    // 使用三角形索引而不是Triangle3f，避免类型转换问题
    std::vector<ThreeVertIds> triangles;
    std::vector<Vector3f> vertices;

public:
    MeshBuilder() = default;

    void addSphere(const Vector3f& center, float radius, int hRes, int vRes, float /*startAngle*/, float /*endAngle*/) {
        // 使用MR::makeUVSphere创建球体并合并到当前网格
        auto sphereMesh = MR::makeUVSphere(radius, hRes, vRes);
        // 移动球体到指定位置
        for (auto& p : sphereMesh.points) {
            p += center;
        }
        // 合并到当前网格
        mergeMesh(sphereMesh);
    }

    void addCylinder(const Vector3f& base, const Vector3f& top, float radius, int resolution) {
        // 使用MR::makeCylinder创建圆柱体
        float length = (top - base).length();
        auto dir = (top - base).normalized();
        
        // 创建圆柱体
        auto cylinderMesh = MR::makeCylinder(radius, length, resolution);
        
        // 旋转和移动到指定位置
        // 简化实现，假设方向是z轴
        for (auto& p : cylinderMesh.points) {
            p += base;
        }
        
        // 合并到当前网格
        mergeMesh(cylinderMesh);
    }

    void addDisk(const Vector3f& center, const Vector3f& normal, float radius, int resolution) {
        // 使用我们新实现的makeDisk函数
        auto diskMesh = MR::makeDisk(center, normal, radius, resolution);
        // 合并到当前网格
        mergeMesh(diskMesh);
    }

    void addTorus(const Vector3f& center, const Vector3f& /*normal*/, float primaryRadius, float secondaryRadius, 
                 int primaryRes, int secondaryRes) {
        // 创建圆环
        auto torusMesh = MR::makeTorus(primaryRadius, secondaryRadius, primaryRes, secondaryRes);
        // 移动到指定位置
        for (auto& p : torusMesh.points) {
            p += center;
        }
        // 合并到当前网格
        mergeMesh(torusMesh);
    }

    MR::Mesh makeMesh() {
        // 创建包含所有几何体的最终网格
        MR::Mesh result;
        result.points = std::move(vertices);
        
        // 将所有三角形添加到网格
        MR::Triangulation triangulation;
        for (const auto& tri : triangles) {
            // 添加三角形
            triangulation.push_back(tri);
        }
        
        result.topology = MR::MeshBuilder::fromTriangles(triangulation);
        return result;
    }

private:
    void mergeMesh(const MR::Mesh& mesh) {
        // 合并一个网格到当前网格
        size_t baseVertex = vertices.size();
        
        // 添加顶点
        for (const auto& p : mesh.points) {
            vertices.push_back(p);
        }
        
        // 添加三角形
        for (auto faceId : mesh.topology.getValidFaces()) {
            auto tri = mesh.topology.getTriVerts(faceId);
            // 转换索引以适应合并后的点云
            ThreeVertIds newTri;
            for (int i = 0; i < 3; ++i) {
                newTri[i] = VertId(baseVertex + tri[i].get());
            }
            triangles.push_back(newTri);
        }
    }
};

namespace MR
{

// 注册插件
MRVIEWER_PLUGIN_REGISTRATION(ToolPathPlugin)

// ImGui帮助标记函数实现（在参数旁边显示问号图标，鼠标悬停时显示帮助文本）
void ToolPathPlugin::HelpMarker(const char* desc)
{
    ImGui::TextDisabled("(?)");
    if (ImGui::IsItemHovered())
    {
        ImGui::BeginTooltip();
        ImGui::PushTextWrapPos(ImGui::GetFontSize() * 35.0f);
        ImGui::TextUnformatted(desc);
        ImGui::PopTextWrapPos();
        ImGui::EndTooltip();
    }
}

// 工具路径插件初始化
ToolPathPlugin::ToolPathPlugin() : StatePlugin("ToolPathPlugin")
{
    // 初始化工具路径参数
    toolPathParams_.millRadius = 3.0f;
    toolPathParams_.voxelSize = 0.5f;
    toolPathParams_.sectionStep = 1.0f;
    toolPathParams_.critTransitionLength = 10.0f;
    toolPathParams_.safeZ = 10.0f;
    toolPathParams_.baseFeed = 1000.0f;
    toolPathParams_.plungeFeed = 500.0f;
    toolPathParams_.retractFeed = 1000.0f;
    toolPathParams_.bypassDir = BypassDirection::Clockwise;

    // 初始化等余量路径参数
    constantCuspParams_.millRadius = 3.0f;
    constantCuspParams_.voxelSize = 0.5f;
    constantCuspParams_.sectionStep = 1.0f;
    constantCuspParams_.safeZ = 10.0f;
    constantCuspParams_.baseFeed = 1000.0f;
    constantCuspParams_.plungeFeed = 500.0f;
    constantCuspParams_.retractFeed = 1000.0f;
    constantCuspParams_.fromCenterToBoundary = true;
    constantCuspParams_.bypassDir = BypassDirection::Clockwise;
    
    // 缓存命令结果
    lacingCommands_ = std::make_shared<std::vector<PluginGCommand>>();
    constantZCommands_ = std::make_shared<std::vector<PluginGCommand>>();
    constantCuspCommands_ = std::make_shared<std::vector<PluginGCommand>>();
    
    // 初始化网格简化参数
    meshSimplificationRatio = 0.5f;
    maxVertexCount = 50000;
    enableAutoSimplification = true;

    // 注册事件处理函数
    auto* viewerInstance = Viewer::instance();
    
    // 设置键盘快捷键
    viewerInstance->keyDownSignal.connect([this](int key, int) {
        if (key == 'T' || key == 't') {
            generateCurrentToolPath();
            return true;
        }
        if (key == 'A' || key == 'a') {
            if (animating_) {
                stopAnimation();
            } else {
                startAnimation();
            }
            return true;
        }
        return false;
    });
}

ToolPathPlugin::~ToolPathPlugin()
{
    // 清理资源
}

// 显示插件主界面
void ToolPathPlugin::drawDialog(float menuScaling, ImGuiContext* /*ctx*/)
{
    // 绘制主面板
    drawMainPanel(menuScaling);
    
    // 绘制状态消息
    drawStatusMessages(menuScaling);
}

// 拖放文件处理函数
bool ToolPathPlugin::onDropFiles(int count, const char* const* paths)
{
    if (count <= 0 || !paths)
        return false;
    
    const std::string filename = paths[0];
    std::string ext = std::filesystem::path(filename).extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c) { return std::tolower(c); });
    
    if (ext == ".step" || ext == ".stp")
    {
        importSTEPModel(filename);
        return true;
    }
    
    return false;
}

// 辅助函数：显示工具路径信息
void ToolPathPlugin::showToolPathInfo(const std::vector<PluginGCommand>* commands, const std::string& name)
{
    if (!commands || commands->empty())
    {
        ImGui::Text("无路径数据");
        return;
    }

    // 计算路径长度和点数
    float pathLength = calculatePathLength(commands);
    int pointCount = static_cast<int>(commands->size());

    ImGui::Text("%s", name.c_str());
    ImGui::Text("路径长度: %.2f mm", pathLength);
    ImGui::Text("点数: %d", pointCount);
    ImGui::Text("平均间距: %.2f mm", pathLength / std::max(1, pointCount - 1));
}

// 绘制主面板
void ToolPathPlugin::drawMainPanel(float menuScaling)
{
    if (ImGui::Begin("工具路径生成", nullptr, ImGuiWindowFlags_AlwaysAutoResize))
    {
        ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.6f);
        
        if (ImGui::CollapsingHeader("模型选择", ImGuiTreeNodeFlags_DefaultOpen))
        {
            drawModelSelectionPanel(menuScaling);
        }
        
        if (ImGui::CollapsingHeader("刀具设置", ImGuiTreeNodeFlags_DefaultOpen))
        {
            drawToolPanel(menuScaling);
        }
        
        if (ImGui::CollapsingHeader("算法设置", ImGuiTreeNodeFlags_DefaultOpen))
        {
            drawAlgorithmPanel(menuScaling);
        }
        
        if (ImGui::CollapsingHeader("路径生成", ImGuiTreeNodeFlags_DefaultOpen))
        {
            drawPathGenerationPanel(menuScaling);
        }
        
        if (ImGui::CollapsingHeader("动画控制", ImGuiTreeNodeFlags_DefaultOpen))
        {
            drawAnimationPanel(menuScaling);
        }
        
        if (ImGui::CollapsingHeader("多视图设置", ImGuiTreeNodeFlags_DefaultOpen))
        {
            drawMultiViewPanel(menuScaling);
        }
        
        if (ImGui::CollapsingHeader("路径分析", ImGuiTreeNodeFlags_DefaultOpen))
        {
            drawAnalysisPanel(menuScaling);
        }
        
        ImGui::PopItemWidth();
    }
    ImGui::End();
}

// 绘制模型选择面板
void ToolPathPlugin::drawModelSelectionPanel(float /*menuScaling*/)
{
    // 实现代码...
}

// 绘制工具设置面板
void ToolPathPlugin::drawToolPanel(float /*menuScaling*/)
{
    // 刀具类型
    int toolType = static_cast<int>(toolType_);
    const char* toolTypeItems[] = { "球头刀", "平底刀", "圆角刀" };
    if (ImGui::Combo("刀具类型", &toolType, toolTypeItems, IM_ARRAYSIZE(toolTypeItems)))
    {
        toolType_ = static_cast<ToolType>(toolType);
        if (toolObject_)
        {
            SceneRoot::get().removeChild(toolObject_);
            toolObject_ = createToolModel(toolType_, toolPathParams_.millRadius);
            SceneRoot::get().addChild(toolObject_);
        }
    }
    
    // 刀具半径
    float radius = toolPathParams_.millRadius;
    if (ImGui::SliderFloat("刀具半径 (mm)", &radius, 0.5f, 10.0f))
    {
        toolPathParams_.millRadius = radius;
        constantCuspParams_.millRadius = radius;
        
        if (toolObject_)
        {
            SceneRoot::get().removeChild(toolObject_);
            toolObject_ = createToolModel(toolType_, radius);
            SceneRoot::get().addChild(toolObject_);
        }
    }
    ImGui::SameLine();
    ToolPathPlugin::HelpMarker("刀具半径影响刀具的实际尺寸和加工精度");
    
    // 体素大小（用于偏移网格）
    float voxelSize = toolPathParams_.voxelSize;
    if (ImGui::SliderFloat("体素大小 (mm)", &voxelSize, 0.1f, 5.0f))
    {
        toolPathParams_.voxelSize = voxelSize;
        constantCuspParams_.voxelSize = voxelSize;
    }
    ImGui::SameLine();
    ToolPathPlugin::HelpMarker("体素大小影响刀具路径的计算精度，较小的值提供更高的精度但需要更多计算时间");
    
    // 安全高度
    float safeZ = toolPathParams_.safeZ;
    if (ImGui::SliderFloat("安全高度 (mm)", &safeZ, 10.0f, 100.0f))
    {
        toolPathParams_.safeZ = safeZ;
        constantCuspParams_.safeZ = safeZ;
    }
    ImGui::SameLine();
    ToolPathPlugin::HelpMarker("安全高度是刀具在不加工时移动的高度，应确保不会与工件碰撞");
    
    // 进给率
    float baseFeed = toolPathParams_.baseFeed;
    if (ImGui::SliderFloat("加工进给率 (mm/min)", &baseFeed, 100.0f, 1000.0f))
    {
        toolPathParams_.baseFeed = baseFeed;
        constantCuspParams_.baseFeed = baseFeed;
    }
    ImGui::SameLine();
    ToolPathPlugin::HelpMarker("加工时刀具移动的速度");
    
    float plungeFeed = toolPathParams_.plungeFeed;
    if (ImGui::SliderFloat("下刀进给率 (mm/min)", &plungeFeed, 50.0f, 500.0f))
    {
        toolPathParams_.plungeFeed = plungeFeed;
        constantCuspParams_.plungeFeed = plungeFeed;
    }
    ImGui::SameLine();
    ToolPathPlugin::HelpMarker("刀具向下切入时的速度，通常低于普通进给率");
    
    float retractFeed = toolPathParams_.retractFeed;
    if (ImGui::SliderFloat("抬刀进给率 (mm/min)", &retractFeed, 100.0f, 1000.0f))
    {
        toolPathParams_.retractFeed = retractFeed;
        constantCuspParams_.retractFeed = retractFeed;
    }
    ImGui::SameLine();
    ToolPathPlugin::HelpMarker("刀具抬起时的速度，通常高于普通进给率");
    
    // 网格简化设置
    ImGui::Separator();
    ImGui::Text("网格简化设置");
    
    bool enableSimplify = enableAutoSimplification;
    if (ImGui::Checkbox("自动简化大型网格", &enableSimplify))
    {
        enableAutoSimplification = enableSimplify;
    }
    ImGui::SameLine();
    ToolPathPlugin::HelpMarker("对大型网格自动进行简化以提高性能");
    
    int maxVerts = maxVertexCount;
    if (ImGui::SliderInt("最大顶点数", &maxVerts, 10000, 200000))
    {
        maxVertexCount = maxVerts;
    }
    ImGui::SameLine();
    ToolPathPlugin::HelpMarker("超过此顶点数的网格将被自动简化");
    
    float simplifyRatio = meshSimplificationRatio;
    if (ImGui::SliderFloat("简化比例", &simplifyRatio, 0.1f, 0.9f))
    {
        meshSimplificationRatio = simplifyRatio;
    }
    ImGui::SameLine();
    ToolPathPlugin::HelpMarker("简化后保留的网格比例，越小简化越多，速度越快");
}

// 绘制算法设置面板
void ToolPathPlugin::drawAlgorithmPanel(float /*menuScaling*/)
{
    // 实现代码...
}

// 绘制路径生成面板
void ToolPathPlugin::drawPathGenerationPanel(float /*menuScaling*/)
{
    // 实现代码...
}

// 绘制动画控制面板
void ToolPathPlugin::drawAnimationPanel(float /*menuScaling*/)
{
    // 实现代码...
}

// 绘制多视图设置面板
void ToolPathPlugin::drawMultiViewPanel(float /*menuScaling*/)
{
    // 实现代码...
}

// 绘制分析面板
void ToolPathPlugin::drawAnalysisPanel(float /*menuScaling*/)
{
    // 实现代码...
}

// 绘制状态消息
void ToolPathPlugin::drawStatusMessages(float /*menuScaling*/)
{
    if (statusMessages_.empty())
        return;
    
    ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(300, 0), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowBgAlpha(0.7f);
    
    if (ImGui::Begin("状态消息", nullptr, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_AlwaysAutoResize))
    {
        auto currentTime = std::chrono::steady_clock::now();
        // 删除未使用的变量
        // float deltaTime = ImGui::GetIO().DeltaTime;
        
        for (auto it = statusMessages_.begin(); it != statusMessages_.end();)
        {
            // 计算剩余时间
            auto elapsedTime = std::chrono::duration_cast<std::chrono::seconds>(currentTime - it->timestamp).count();
            if (elapsedTime >= 5) // 消息显示5秒
            {
                it = statusMessages_.erase(it);
                continue;
            }
            
            // 设置颜色
            switch (it->type)
            {
            case StatusMessage::Type::Info:
                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.0f, 0.8f, 0.0f, 1.0f)); // 绿色
                break;
            case StatusMessage::Type::Warning:
                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.8f, 0.0f, 1.0f)); // 黄色
                break;
            case StatusMessage::Type::Error:
                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.3f, 0.3f, 1.0f)); // 红色
                break;
            }
            
            ImGui::TextWrapped("%s", it->message.c_str());
            ImGui::PopStyleColor();
            
            ++it;
        }
    }
    ImGui::End();
}

// 添加状态消息
void ToolPathPlugin::addStatusMessage(const std::string& message, StatusMessage::Type type)
{
    StatusMessage msg;
    msg.message = message;
    msg.type = type;
    msg.timestamp = std::chrono::steady_clock::now();
    
    statusMessages_.push_back(msg);
    
    // 限制消息数量
    if (statusMessages_.size() > 10)
    {
        statusMessages_.erase(statusMessages_.begin());
    }
}

// 分层刀路路径生成
std::vector<PluginGCommand> lacingToolPath(const Mesh& mesh, Axis direction, const ToolPathParams& params)
{
    // 创建MeshPart对象
    MeshPart meshPart(mesh);
    
    // 调用MRVoxels中的工具路径生成函数
    auto result = MR::lacingToolPath(meshPart, params, direction);
    
    // 处理可能的错误
    if (!result.has_value())
    {
        // 出错时返回空数组
        return {};
    }
    
    // 将工具路径结果转换为PluginGCommand
    std::vector<PluginGCommand> commands;
    const auto& cmds = result.value().commands;
    
    commands.reserve(cmds.size());
    for (const auto& cmd : cmds)
    {
        PluginGCommand pluginCmd;
        pluginCmd.x = cmd.x;
        pluginCmd.y = cmd.y;
        pluginCmd.z = cmd.z;
        pluginCmd.f = cmd.feed;
        
        // 转换命令类型
        switch (cmd.type)
        {
            case MoveType::Linear:
                pluginCmd.type = PluginGCommandType::LinearMove;
                break;
            case MoveType::FastLinear:
                pluginCmd.type = PluginGCommandType::Rapid;
                break;
            default:
                pluginCmd.type = PluginGCommandType::Comment;
                break;
        }
        
        commands.push_back(pluginCmd);
    }
    
    return commands;
}

// 等高刀路路径生成
std::vector<PluginGCommand> constantZToolPath(const Mesh& mesh, const ToolPathParams& params)
{
    // 创建MeshPart对象
    MeshPart meshPart(mesh);
    
    // 调用MRVoxels中的工具路径生成函数
    auto result = MR::constantZToolPath(meshPart, params);
    
    // 处理可能的错误
    if (!result.has_value())
    {
        // 出错时返回空数组
        return {};
    }
    
    // 将工具路径结果转换为PluginGCommand
    std::vector<PluginGCommand> commands;
    const auto& cmds = result.value().commands;
    
    commands.reserve(cmds.size());
    for (const auto& cmd : cmds)
    {
        PluginGCommand pluginCmd;
        pluginCmd.x = cmd.x;
        pluginCmd.y = cmd.y;
        pluginCmd.z = cmd.z;
        pluginCmd.f = cmd.feed;
        
        // 转换命令类型
        switch (cmd.type)
        {
            case MoveType::Linear:
                pluginCmd.type = PluginGCommandType::LinearMove;
                break;
            case MoveType::FastLinear:
                pluginCmd.type = PluginGCommandType::Rapid;
                break;
            default:
                pluginCmd.type = PluginGCommandType::Comment;
                break;
        }
        
        commands.push_back(pluginCmd);
    }
    
    return commands;
}

// 等余量刀路路径生成
std::vector<PluginGCommand> constantCuspToolPath(const Mesh& mesh, const ConstantCuspParams& params)
{
    // 创建MeshPart对象
    MeshPart meshPart(mesh);
    
    // 调用MRVoxels中的工具路径生成函数
    auto result = MR::constantCuspToolPath(meshPart, params);
    
    // 处理可能的错误
    if (!result.has_value())
    {
        // 出错时返回空数组
        return {};
    }
    
    // 将工具路径结果转换为PluginGCommand
    std::vector<PluginGCommand> commands;
    const auto& cmds = result.value().commands;
    
    commands.reserve(cmds.size());
    for (const auto& cmd : cmds)
    {
        PluginGCommand pluginCmd;
        pluginCmd.x = cmd.x;
        pluginCmd.y = cmd.y;
        pluginCmd.z = cmd.z;
        pluginCmd.f = cmd.feed;
        
        // 转换命令类型
        switch (cmd.type)
        {
            case MoveType::Linear:
                pluginCmd.type = PluginGCommandType::LinearMove;
                break;
            case MoveType::FastLinear:
                pluginCmd.type = PluginGCommandType::Rapid;
                break;
            default:
                pluginCmd.type = PluginGCommandType::Comment;
                break;
        }
        
        commands.push_back(pluginCmd);
    }
    
    return commands;
}

// 导入STEP模型
void ToolPathPlugin::importSTEPModel(const std::string& filename)
{
    addStatusMessage("正在导入STEP模型: " + filename, StatusMessage::Type::Info);
    
    // 使用MR::MeshLoad::fromStep加载STEP文件
    auto result = MR::MeshLoad::fromStep(filename);
    
    if (!result.has_value())
    {
        // 处理导入失败
        addStatusMessage("STEP模型导入失败: " + result.error(), StatusMessage::Type::Error);
        return;
    }
    
    // 获取导入的网格
    auto mesh = std::make_shared<Mesh>();
    *mesh = result.value();
    
    // 创建网格对象
    auto model = std::make_shared<ObjectMesh>();
    model->setName(std::filesystem::path(filename).stem().string());
    model->setMesh(mesh);
    
    // 设置默认视觉属性
    model->setFlatShading(false);
    model->setVisualizeProperty(true, MeshVisualizePropertyType::Faces, ViewportMask::all());
    model->setVisualizeProperty(true, MeshVisualizePropertyType::Edges, ViewportMask::all());
    model->setVisualizeProperty(false, MeshVisualizePropertyType::Points, ViewportMask::all());
    
    // 添加到场景并更新视图
    SceneRoot::get().addChild(model);
    Viewer::instance()->fitDataViewport();
    
    // 添加到已加载模型列表
    loadedModels_.push_back(model);
    selectedModel_ = model;
    
    // 处理导入的模型
    processImportedModel(model);
    
    addStatusMessage("STEP模型成功导入", StatusMessage::Type::Info);
}

// 创建线段可视化
void ToolPathPlugin::createLineSegment(const Vector3f& start, const Vector3f& end, const Color& /*color*/, std::shared_ptr<Polyline3>& polyline)
{
    // 根据Polyline.h中的API，我们应该使用addFromPoints方法
    // 准备两个点的数组
    std::array<Vector3f, 2> points = { start, end };
    
    // 添加一条新的线段，不闭合
    polyline->addFromPoints(points.data(), points.size(), false);
}

// 创建工具路径可视化
void ToolPathPlugin::createToolPathVisualization(const std::vector<PluginGCommand>& commands, 
                                               const Color& color,
                                               std::shared_ptr<Object>& pathObject, 
                                               std::shared_ptr<Object>& pointsObject)
{
    // 创建线路径对象
    auto lines = std::make_shared<ObjectLines>();
    pathObject = lines;
    pathObject->setName("ToolPath");
    
    // 创建多段线
    auto polyline = std::make_shared<Polyline3>();
    
    // 存储轨迹点
    std::vector<Vector3f> pathPoints;
    
    // 遍历所有命令
    for (size_t i = 0; i < commands.size(); ++i)
    {
        const auto& cmd = commands[i];
        
        // 添加点到路径中
        Vector3f point(cmd.x, cmd.y, cmd.z);
        pathPoints.push_back(point);
        
        // 如果有前一个命令，创建线段
        if (i > 0)
        {
            const auto& prevCmd = commands[i-1];
            Vector3f prevPoint(prevCmd.x, prevCmd.y, prevCmd.z);
            
            // 根据命令类型设置不同样式
            Color segmentColor = color;
            if (cmd.type == PluginGCommandType::Rapid)
            {
                segmentColor = Color(0.5f, 0.5f, 0.5f); // 快速定位移动用灰色表示
            }
            
            // 添加线段到多段线
            createLineSegment(prevPoint, point, segmentColor, polyline);
        }
    }
    
    // 设置多段线到ObjectLines对象
    lines->setPolyline(polyline);
    
    // 设置线宽
    lines->setLineWidth(2.0f);
    
    // 设置线条颜色
    lines->setFrontColor(color, false);
    
    // 创建点对象
    if (!pathPoints.empty())
    {
        // 创建点网格
        auto pointsMesh = std::make_shared<Mesh>();
        
        // 为每个点添加一个顶点
        for (const auto& point : pathPoints)
        {
            pointsMesh->addPoint(point);
        }
        
        // 创建网格对象
        auto pointsObjectMesh = std::make_shared<ObjectMesh>();
        pointsObjectMesh->setName("ToolPathPoints");
        pointsObjectMesh->setMesh(pointsMesh);
        
        // 设置点显示
        pointsObjectMesh->setPointSize(4.0f);
        pointsObjectMesh->setFrontColor(color, false);
        pointsObjectMesh->setVisualizeProperty(true, MeshVisualizePropertyType::Points, ViewportMask::all());
        pointsObjectMesh->setVisualizeProperty(false, MeshVisualizePropertyType::Faces, ViewportMask::all());
        pointsObjectMesh->setVisualizeProperty(false, MeshVisualizePropertyType::Edges, ViewportMask::all());
        
        pointsObject = pointsObjectMesh;
    }
    
    // 添加到场景
    SceneRoot::get().addChild(pathObject);
    if (pointsObject)
        SceneRoot::get().addChild(pointsObject);
}

// 简化网格
void ToolPathPlugin::simplifyMesh(std::shared_ptr<MR::Object> model, float ratio)
{
    // 确保是网格对象
    auto objMesh = std::dynamic_pointer_cast<ObjectMesh>(model);
    if (!objMesh)
        return;
    
    addStatusMessage(fmt::format("开始简化网格，比例: {:.2f}", ratio), StatusMessage::Type::Info);
    
    // 创建计时器，根据MRTimer.h的定义
    Timer timer("simplifyMesh");
    
    // 克隆原始网格以保留原始数据
    auto originalMesh = objMesh->mesh();
    auto simplifiedMesh = std::make_shared<Mesh>(*originalMesh);
    
    // 设置简化参数
    DecimateSettings settings;
    settings.maxError = originalMesh->topology.numValidFaces() * (1.0f - ratio);
    
    // 设置进度回调
    settings.progressCallback = [this](float progress) -> bool {
        // 当进度更新时，更新状态消息
        static float lastReportProgress = 0.0f;
        if (progress - lastReportProgress > 0.1f)  // 每10%更新一次
        {
            addStatusMessage(fmt::format("网格简化进度: {:.0f}%", progress * 100.0f), StatusMessage::Type::Info);
            lastReportProgress = progress;
        }
        
        // 返回true继续简化，返回false取消
        return true;
    };
    
    // 执行网格简化
    auto result = decimateMesh(*simplifiedMesh, settings);
    
    // 判断结果
    bool success = !result.cancelled;
    
    if (success)
    {
        // 更新模型网格
        objMesh->setMesh(simplifiedMesh);
        
        // 计算用时并显示结果
        double elapsedSec = timer.secondsPassed().count();
        addStatusMessage(fmt::format("网格简化完成，用时: {:.2f}秒，面数: {} -> {}", 
            elapsedSec, originalMesh->topology.numValidFaces(), simplifiedMesh->topology.numValidFaces()), 
            StatusMessage::Type::Info);
    }
    else
    {
        // 简化失败
        std::string errorMsg = "网格简化失败";
        if (result.cancelled)
            errorMsg += ": 操作已取消";
        
        addStatusMessage(errorMsg, StatusMessage::Type::Error);
    }
}

// 处理导入的模型
void ToolPathPlugin::processImportedModel(std::shared_ptr<MR::Object> model)
{
    // 确保是网格对象
    auto objMesh = std::dynamic_pointer_cast<ObjectMesh>(model);
    if (!objMesh || !objMesh->mesh())
        return;
    
    // 检查是否需要简化网格
    int numVerts = objMesh->mesh()->topology.numValidVerts();
    if (enableAutoSimplification && numVerts > maxVertexCount)
    {
        addStatusMessage("大型网格检测: " + std::to_string(numVerts) + " 顶点，正在优化以提高性能...", StatusMessage::Type::Info);
        
        // 计算简化比例，确保不会降低太多
        float simplifyRatio = std::min(0.9f, float(maxVertexCount) / float(numVerts));
        
        // 执行网格简化
        simplifyMesh(objMesh, simplifyRatio);
    }
    
    // 执行网格预处理和修复
    bool preprocessed = preprocessModel(objMesh);
    if (preprocessed)
    {
        addStatusMessage("模型预处理完成", StatusMessage::Type::Info);
    }
    
    // 分析下切区域
    visualizeUndercuts(objMesh);
}

// 预处理模型（检查并修复问题）
bool ToolPathPlugin::preprocessModel(std::shared_ptr<ObjectMesh> model)
{
    if (!model || !model->mesh())
    {
        addStatusMessage("无效的模型", StatusMessage::Type::Error);
        return false;
    }
    
    addStatusMessage("开始预处理模型...", StatusMessage::Type::Info);
    
    // 获取网格
    auto mesh = model->mesh();
    
    // 检查网格是否有效
    if (mesh->topology.numValidFaces() == 0)
    {
        addStatusMessage("模型没有有效面，无法处理", StatusMessage::Type::Error);
        return false;
    }
    
    // 标记需要修复的问题
    bool needsRepair = false;
    
    // 检查自相交
    addStatusMessage("正在检查自相交...", StatusMessage::Type::Info);
    auto selfIntersectionsResult = MR::SelfIntersections::getFaces(*mesh);
    if (!selfIntersectionsResult.has_value())
    {
        addStatusMessage(
            fmt::format("检查自相交时发生错误: {}", selfIntersectionsResult.error()),
            StatusMessage::Type::Error
        );
        return false;
    }

    if (!selfIntersectionsResult.value().empty())
    {
        addStatusMessage(
            fmt::format("检测到 {} 个自相交", selfIntersectionsResult.value().count()),
            StatusMessage::Type::Warning
        );
        needsRepair = true;
    }
    
    // 检查重复面
    addStatusMessage("正在检查重复面...", StatusMessage::Type::Info);
    auto duplicateFaces = findDuplicateFaces(*mesh);
    if (!duplicateFaces.empty())
    {
        addStatusMessage(
            fmt::format("检测到 {} 个重复面", duplicateFaces.size()),
            StatusMessage::Type::Warning
        );
        needsRepair = true;
    }
    
    // 检查非流形边
    addStatusMessage("正在检查非流形边...", StatusMessage::Type::Info);
    auto nonManifoldEdges = findNonManifoldEdges(*mesh);
    if (!nonManifoldEdges.empty())
    {
        addStatusMessage(
            fmt::format("检测到 {} 个非流形边", nonManifoldEdges.size()),
            StatusMessage::Type::Warning
        );
        needsRepair = true;
    }
    
    // 如果没有问题，直接返回
    if (!needsRepair)
    {
        addStatusMessage("模型无需修复", StatusMessage::Type::Success);
        return true;
    }
    
    // 创建网格副本进行修复
    addStatusMessage("开始修复网格问题...", StatusMessage::Type::Info);
    auto meshCopy = std::make_shared<MR::Mesh>(*mesh);
    
    // 修复自相交问题
    if (!selfIntersectionsResult.value().empty())
    {
        addStatusMessage("正在修复自相交...", StatusMessage::Type::Info);
        MR::SelfIntersections::Settings settings;
        settings.method = MR::SelfIntersections::Settings::Method::CutAndFill;
        settings.maxExpand = 3;
        settings.relaxIterations = 5;
        
        auto fixResult = MR::SelfIntersections::fix(*meshCopy, settings);
        if (!fixResult.has_value())
        {
            addStatusMessage(
                fmt::format("修复自相交失败: {}", fixResult.error()),
                StatusMessage::Type::Error
            );
        }
        else
        {
            // 检查修复结果
            auto checkIntersections = MR::SelfIntersections::getFaces(*meshCopy);
            if (checkIntersections.has_value() && checkIntersections.value().empty())
            {
                addStatusMessage("自相交已修复", StatusMessage::Type::Success);
            }
            else
            {
                addStatusMessage(
                    fmt::format("仍有 {} 个自相交未修复", 
                        checkIntersections.has_value() ? checkIntersections.value().count() : 0),
                    StatusMessage::Type::Warning
                );
            }
        }
    }
    
    // 删除重复面
    if (!duplicateFaces.empty())
    {
        addStatusMessage("正在删除重复面...", StatusMessage::Type::Info);
        for (const auto& faceId : duplicateFaces)
        {
            meshCopy->topology.deleteFace(faceId);
        }
        addStatusMessage("重复面已删除", StatusMessage::Type::Success);
    }
    
    // 处理非流形边（通过复制网格顶点来解决）
    if (!nonManifoldEdges.empty())
    {
        addStatusMessage("正在处理非流形边...", StatusMessage::Type::Info);
        // 复制有非流形边的区域
        MR::duplicateMultiHoleVertices(*meshCopy);
        addStatusMessage("非流形边已处理", StatusMessage::Type::Success);
    }
    
    // 更新模型网格
    model->setMesh(meshCopy);
    model->setDirtyFlags(DIRTY_ALL);
    
    addStatusMessage("模型预处理完成", StatusMessage::Type::Success);
    return true;
}

// 查找具有重复面的FaceId列表
std::vector<MR::FaceId> ToolPathPlugin::findDuplicateFaces(const MR::Mesh& mesh)
{
    std::vector<MR::FaceId> duplicateFaces;
    const auto& topology = mesh.topology;
    const auto& validFaces = topology.getValidFaces();
    
    // 创建一个映射来跟踪每个三角形顶点组合
    struct TriangleVerts {
        MR::VertId v[3];
        
        bool operator==(const TriangleVerts& other) const {
            // 检查两个三角形是否有相同的顶点集合（忽略顺序）
            std::array<MR::VertId, 3> a = {v[0], v[1], v[2]};
            std::array<MR::VertId, 3> b = {other.v[0], other.v[1], other.v[2]};
            std::sort(a.begin(), a.end());
            std::sort(b.begin(), b.end());
            return a == b;
        }
    };
    
    struct TriangleVertsHash {
        std::size_t operator()(const TriangleVerts& tv) const {
            // 为排序后的顶点数组创建哈希
            std::array<MR::VertId, 3> a = {tv.v[0], tv.v[1], tv.v[2]};
            std::sort(a.begin(), a.end());
            std::size_t h = 0;
            for (const auto& v : a) {
                h = h * 31 + std::hash<int>()(v);
            }
            return h;
        }
    };
    
    std::unordered_map<TriangleVerts, std::vector<MR::FaceId>, TriangleVertsHash> triangleMap;
    
    // 遍历所有有效面
    for (auto faceId : validFaces)
    {
        MR::ThreeVertIds tverts;
        topology.getTriVerts(faceId, tverts);
        
        TriangleVerts tv = {tverts[0], tverts[1], tverts[2]};
        triangleMap[tv].push_back(faceId);
    }
    
    // 查找具有相同顶点的面
    for (const auto& entry : triangleMap)
    {
        if (entry.second.size() > 1)
        {
            // 第一个面保留，其余标记为重复
            for (size_t i = 1; i < entry.second.size(); ++i)
            {
                duplicateFaces.push_back(entry.second[i]);
            }
        }
    }
    
    return duplicateFaces;
}

// 查找非流形边
std::vector<MR::UndirectedEdgeId> ToolPathPlugin::findNonManifoldEdges(const MR::Mesh& mesh)
{
    std::vector<MR::UndirectedEdgeId> nonManifoldEdges;
    const auto& topology = mesh.topology;
    
    // 遍历所有无向边
    for (MR::UndirectedEdgeId ueId{0}; ueId < topology.undirectedEdgeSize(); ++ueId)
    {
        if (topology.isLoneEdge(ueId))
            continue;
            
        MR::EdgeId eId(ueId);
        // 检查一条边是否有两个以上的相邻面
        int faceCount = 0;
        if (topology.left(eId).valid())
            faceCount++;
        if (topology.right(eId).valid())
            faceCount++;
        if (topology.left(eId.sym()).valid())
            faceCount++;
        if (topology.right(eId.sym()).valid())
            faceCount++;
            
        // 非流形边连接了超过两个面
        if (faceCount > 2)
            nonManifoldEdges.push_back(ueId);
    }
    
    return nonManifoldEdges;
}

// 可视化下切区域
void ToolPathPlugin::visualizeUndercuts(std::shared_ptr<ObjectMesh> model)
{
    if (!model || !model->mesh())
    {
        addStatusMessage("无效的模型，无法分析下切区域", StatusMessage::Type::Error);
        return;
    }
    
    addStatusMessage("正在分析下切区域...", StatusMessage::Type::Info);
    
    // 获取网格
    auto mesh = model->mesh();
    
    // 创建下切标记位图
    auto undercut = std::make_shared<FaceBitSet>(mesh->topology.faceSize());
    
    // 分析每个面的法线
    // 计算面法线
    auto normals = MR::computePerFaceNormals(*mesh);
    
    size_t undercutCount = 0;
    // 遍历所有有效面
    for (FaceId faceId : mesh->topology.getValidFaces())
    {
        // 获取面法线
        Vector3f normal = normals[faceId];
        
        // 小于或等于0的z分量表示倒角区域（假设z为上方向）
        if (normal.z <= 0)
        {
            undercut->set(faceId);
            undercutCount++;
        }
    }
    
    if (undercutCount == 0)
    {
        addStatusMessage("未检测到下切区域", StatusMessage::Type::Info);
        return;
    }
    
    addStatusMessage(fmt::format("检测到 {} 个下切面 (约占总面数的 {:.1f}%)", 
        undercutCount, 100.0f * float(undercutCount) / float(mesh->topology.numValidFaces())), 
        StatusMessage::Type::Info);
    
    // 创建下切区域的可视化
    auto undercutMesh = std::make_shared<Mesh>(*mesh);
    
    // 创建用于删除的面集合
    FaceBitSet facesToDelete = mesh->topology.getValidFaces();
    
    // 从这个集合中删除已标记的下切面
    for (FaceId faceId : mesh->topology.getValidFaces())
    {
        if (undercut->test(faceId))
            facesToDelete.reset(faceId);
    }
    
    // 删除所有非下切面
    undercutMesh->topology.deleteFaces(facesToDelete);
    
    // 创建用于显示的网格对象
    auto undercutVisualization = std::make_shared<ObjectMesh>();
    undercutVisualization->setName("下切区域");
    undercutVisualization->setMesh(undercutMesh);
    
    // 设置视觉属性
    undercutVisualization->setFrontColor(Color(1.0f, 0.0f, 0.0f, 0.7f), false);  // 红色半透明，未选中状态
    undercutVisualization->setBackColor(Color(1.0f, 0.0f, 0.0f, 0.7f));   // 红色半透明
    undercutVisualization->setFlatShading(true);
    undercutVisualization->setVisualizeProperty(true, MeshVisualizePropertyType::Faces, ViewportMask::all());
    undercutVisualization->setVisualizeProperty(true, MeshVisualizePropertyType::Edges, ViewportMask::all());
    undercutVisualization->setVisualizeProperty(false, MeshVisualizePropertyType::Points, ViewportMask::all());
    
    // 添加到场景
    SceneRoot::get().addChild(undercutVisualization);
    
    // 记住这个可视化对象，以便后续可以移除或更新
    undercutVisualization_ = undercutVisualization;
}

// 生成分层刀具路径
void ToolPathPlugin::generateLacingToolPath()
{
    if (!selectedModel_)
    {
        addStatusMessage("请先选择模型", StatusMessage::Type::Warning);
        return;
    }
    
    addStatusMessage("正在生成分层刀具路径...", StatusMessage::Type::Info);
    
    // 记录生成开始时间
    Timer timer("lacingToolPath");
    
    // 获取模型的网格
    auto mesh = selectedModel_->mesh();
    
    // 生成刀具路径
    auto commands = lacingToolPath(*mesh, cutDirection_, toolPathParams_);
    
    // 处理结果
    if (commands.empty())
    {
        addStatusMessage("生成分层刀具路径失败", StatusMessage::Type::Error);
        return;
    }
    
    // 更新路径缓存
    *lacingCommands_ = commands;
    lacingPathGenerated_ = true;
    
    // 移除旧的可视化
    if (lacingPathObject_)
    {
        SceneRoot::get().removeChild(lacingPathObject_);
        lacingPathObject_ = nullptr;
    }
    if (lacingPointsObject_)
    {
        SceneRoot::get().removeChild(lacingPointsObject_);
        lacingPointsObject_ = nullptr;
    }
    
    // 创建新的可视化
    createToolPathVisualization(commands, Color(0.0f, 0.8f, 0.0f), lacingPathObject_, lacingPointsObject_);
    
    // 输出统计信息
    double elapsedSec = timer.secondsPassed().count();
    addStatusMessage(fmt::format("分层刀具路径生成完成: {} 个点, 用时: {:.2f}秒", 
        commands.size(), elapsedSec), StatusMessage::Type::Info);
}

// 生成等高刀具路径
void ToolPathPlugin::generateConstantZToolPath()
{
    if (!selectedModel_)
    {
        addStatusMessage("请先选择模型", StatusMessage::Type::Warning);
        return;
    }
    
    addStatusMessage("正在生成等高刀具路径...", StatusMessage::Type::Info);
    
    // 记录生成开始时间
    Timer timer("constantZToolPath");
    
    // 获取模型的网格
    auto mesh = selectedModel_->mesh();
    
    // 生成刀具路径
    auto commands = constantZToolPath(*mesh, toolPathParams_);
    
    // 处理结果
    if (commands.empty())
    {
        addStatusMessage("生成等高刀具路径失败", StatusMessage::Type::Error);
        return;
    }
    
    // 更新路径缓存
    *constantZCommands_ = commands;
    constantZPathGenerated_ = true;
    
    // 移除旧的可视化
    if (constantZPathObject_)
    {
        SceneRoot::get().removeChild(constantZPathObject_);
        constantZPathObject_ = nullptr;
    }
    if (constantZPointsObject_)
    {
        SceneRoot::get().removeChild(constantZPointsObject_);
        constantZPointsObject_ = nullptr;
    }
    
    // 创建新的可视化
    createToolPathVisualization(commands, Color(0.0f, 0.0f, 0.8f), constantZPathObject_, constantZPointsObject_);
    
    // 输出统计信息
    double elapsedSec = timer.secondsPassed().count();
    addStatusMessage(fmt::format("等高刀具路径生成完成: {} 个点, 用时: {:.2f}秒", 
        commands.size(), elapsedSec), StatusMessage::Type::Info);
}

// 生成等余量刀具路径
void ToolPathPlugin::generateConstantCuspToolPath()
{
    if (!selectedModel_)
    {
        addStatusMessage("请先选择模型", StatusMessage::Type::Warning);
        return;
    }
    
    addStatusMessage("正在生成等余量刀具路径...", StatusMessage::Type::Info);
    
    // 记录生成开始时间
    Timer timer("constantCuspToolPath");
    
    // 获取模型的网格
    auto mesh = selectedModel_->mesh();
    
    // 生成刀具路径
    auto commands = constantCuspToolPath(*mesh, constantCuspParams_);
    
    // 处理结果
    if (commands.empty())
    {
        addStatusMessage("生成等余量刀具路径失败", StatusMessage::Type::Error);
        return;
    }
    
    // 更新路径缓存
    *constantCuspCommands_ = commands;
    constantCuspPathGenerated_ = true;
    
    // 移除旧的可视化
    if (constantCuspPathObject_)
    {
        SceneRoot::get().removeChild(constantCuspPathObject_);
        constantCuspPathObject_ = nullptr;
    }
    if (constantCuspPointsObject_)
    {
        SceneRoot::get().removeChild(constantCuspPointsObject_);
        constantCuspPointsObject_ = nullptr;
    }
    
    // 创建新的可视化
    createToolPathVisualization(commands, Color(0.8f, 0.0f, 0.0f), constantCuspPathObject_, constantCuspPointsObject_);
    
    // 输出统计信息
    double elapsedSec = timer.secondsPassed().count();
    addStatusMessage(fmt::format("等余量刀具路径生成完成: {} 个点, 用时: {:.2f}秒", 
        commands.size(), elapsedSec), StatusMessage::Type::Info);
}

// 创建刀具模型
std::shared_ptr<Object> ToolPathPlugin::createToolModel(ToolType type, float radius)
{
    // 创建MeshBuilder类的实例
    ::MeshBuilder builder;
    
    // 根据刀具类型创建不同形状
    switch (type)
    {
    case ToolType::BallEndMill:
        {
            // 创建球头（半球和圆柱体）
            builder.addSphere(Vector3f(0, 0, 0), radius, 16, 16, 0, M_PI_2); // 下半球
            builder.addCylinder(Vector3f(0, 0, 0), Vector3f(0, 0, radius * 3.0f), radius, 16); // 刀杆
        }
        break;
        
    case ToolType::FlatEndMill:
        {
            // 创建平底刀（圆柱体和圆盘）
            builder.addCylinder(Vector3f(0, 0, 0), Vector3f(0, 0, radius * 3.0f), radius, 16); // 刀杆
            builder.addDisk(Vector3f(0, 0, 0), Vector3f(0, 0, -1), radius, 16); // 底面
        }
        break;
        
    case ToolType::ToroidalMill:  // 将BullNoseMill修改为ToroidalMill以匹配定义
        {
            // 创建圆角刀（圆柱体和圆盘）
            float cornerRadius = radius * 0.2f; // 圆角半径
            float effectiveRadius = radius - cornerRadius;
            
            // 刀杆
            builder.addCylinder(Vector3f(0, 0, 0), Vector3f(0, 0, radius * 3.0f), radius, 16);
            
            // 圆环
            builder.addTorus(Vector3f(0, 0, 0), Vector3f(0, 0, 1), effectiveRadius, cornerRadius, 16, 16);
            
            // 底面
            builder.addDisk(Vector3f(0, 0, -cornerRadius), Vector3f(0, 0, -1), effectiveRadius, 16);
        }
        break;
    }
    
    // 创建网格
    auto toolMesh = builder.makeMesh();
    
    // 创建网格对象
    auto toolObject = std::make_shared<ObjectMesh>();
    toolObject->setName("Tool");
    toolObject->setMesh(std::make_shared<Mesh>(toolMesh));
    
    // 设置刀具可视化属性
    toolObject->setFrontColor(Color(0.6f, 0.6f, 0.7f), false); // 金属灰色
    toolObject->setFlatShading(false);
    toolObject->setVisualizeProperty(true, MeshVisualizePropertyType::Faces, ViewportMask::all());
    toolObject->setVisualizeProperty(true, MeshVisualizePropertyType::Edges, ViewportMask::all());
    toolObject->setVisualizeProperty(false, MeshVisualizePropertyType::Points, ViewportMask::all());
    
    return toolObject;
}

// 实现虚函数
bool MR::ToolPathPlugin::onEnable_()
{
    // 插件启用时的处理逻辑
    // 例如：初始化资源，设置初始状态等
    addStatusMessage("工具路径插件已启用", StatusMessage::Type::Info);
    return true;
}

bool MR::ToolPathPlugin::onDisable_()
{
    // 插件禁用时的处理逻辑
    // 例如：清理资源，恢复状态等
    addStatusMessage("工具路径插件已禁用", StatusMessage::Type::Info);
    return true;
}

// 实现其他成员函数
bool MR::ToolPathPlugin::generateCurrentToolPath()
{
    // 根据当前选择的算法生成工具路径
    switch (selectedAlgorithm_)
    {
    case Algorithm::Lacing:
        generateLacingToolPath();
        return lacingPathGenerated_;
    case Algorithm::ConstantZ:
        generateConstantZToolPath();
        return constantZPathGenerated_;
    case Algorithm::ConstantCusp:
        generateConstantCuspToolPath();
        return constantCuspPathGenerated_;
    default:
        return false;
    }
}

void MR::ToolPathPlugin::startAnimation()
{
    // 开始动画的逻辑
    if (!animating_)
    {
        animating_ = true;
        animationCommandIndex_ = 0;
        //animationLastTime_ = getTimeInSeconds();
        addStatusMessage("动画已开始", StatusMessage::Type::Info);
    }
}

void MR::ToolPathPlugin::stopAnimation()
{
    // 停止动画的逻辑
    if (animating_)
    {
        animating_ = false;
        addStatusMessage("动画已停止", StatusMessage::Type::Info);
    }
}

float MR::ToolPathPlugin::calculatePathLength(const std::vector<PluginGCommand>* commands)
{
    if (!commands || commands->empty())
        return 0.0f;
        
    float totalLength = 0.0f;
    for (size_t i = 1; i < commands->size(); ++i)
    {
        const auto& prev = (*commands)[i-1];
        const auto& curr = (*commands)[i];
        
        // 只考虑LinearMove和Rapid命令
        if (prev.type == PluginGCommandType::LinearMove || prev.type == PluginGCommandType::Rapid)
        {
            if (curr.type == PluginGCommandType::LinearMove || curr.type == PluginGCommandType::Rapid)
            {
                // 计算两点之间的欧几里得距离
                float dx = curr.x - prev.x;
                float dy = curr.y - prev.y;
                float dz = curr.z - prev.z;
                totalLength += std::sqrt(dx*dx + dy*dy + dz*dz);
            }
        }
    }
    
    return totalLength;
}

} // namespace MR 