#include "MRToolPathPlugin.h"
#include <array> // 显式包含array头文件
#include "MRViewerPluginsList.h"  // 由 CMake 在构建目录生成
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
#include "MRRibbonSchema.h"
#include "MRUIStyle.h"

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
MR_REGISTER_RIBBON_ITEM(ToolPathPlugin)

// ImGui帮助标记函数实现（在参数旁边显示问号图标，鼠标悬停时显示帮助文本）
void ToolPathPlugin::HelpMarker(const char* desc)
{
    // 使用普通字符串避免C++20 char8_t类型转换问题
    ImGui::TextDisabled("(?)");
    if (ImGui::IsItemHovered())
    {
        ImGui::BeginTooltip();
        ImGui::PushTextWrapPos(ImGui::GetFontSize() * 35.0f);
        // 使用UTF-8编码前缀确保正确显示中文
        ImGui::Text("%s", desc);
        ImGui::PopTextWrapPos();
        ImGui::EndTooltip();
    }
}

// 工具路径插件初始化
ToolPathPlugin::ToolPathPlugin() : StatePlugin("CAM Tool Path")
{
    // 设置Tab分类
    tab_ = StatePluginTabs::Mesh;
    
    // 初始化工具路径参数 - 使用标准默认值
    toolPathParams_.millRadius = 5.0f;          // 5.0mm - 标准刀具半径
    toolPathParams_.voxelSize = 0.5f;           // 0.5mm - 标准体素大小
    toolPathParams_.sectionStep = 1.0f;         // 1.0mm - 标准层高
    toolPathParams_.critTransitionLength = 10.0f; // 10.0mm - 标准过渡长度
    toolPathParams_.safeZ = 10.0f;              // 10.0mm - 标准安全高度
    toolPathParams_.baseFeed = 1000.0f;
    toolPathParams_.plungeFeed = 500.0f;
    toolPathParams_.retractFeed = 1000.0f;
    toolPathParams_.bypassDir = BypassDirection::Clockwise;

    // 初始化等余量路径参数 - 使用标准默认值
    constantCuspParams_.millRadius = 5.0f;       // 5.0mm - 与主参数保持一致
    constantCuspParams_.voxelSize = 0.5f;        // 0.5mm - 与主参数保持一致
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
    auto menuWidth = 400.0f * menuScaling;

    // 计算居中位置
    ImVec2 position{ (viewer->framebufferSize.x - menuWidth) / 2, viewer->framebufferSize.y / 6.0f };
    
    // 使用StatePlugin基类提供的ImGuiBeginWindow_函数
    if (!ImGuiBeginWindow_({ .width = menuWidth, .position = &position, .menuScaling = menuScaling }))
        return;

    // 使用ImGui TabBar绘制选项卡
    if (UI::beginTabBar("##MainTabs"))
    {
        if (UI::beginTabItem("Tool Path"))
        {
            // 调用现有的drawMainPanel函数，避免代码重复
            drawMainPanel(menuScaling);
            
            UI::endTabItem();
        }
        UI::endTabBar();
    }

    // 使用正确的结束函数
    ImGui::EndCustomStatePlugin();
    
    // 绘制状态消息（这应该是独立的窗口）
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
    // 移除之前的Begin/End调用，因为这些已经在drawDialog中处理
    ImGui::PushItemWidth(ImGui::GetContentRegionAvail().x * 0.6f);
    
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

// 绘制模型选择面板
void ToolPathPlugin::drawModelSelectionPanel(float /*menuScaling*/)
{
    // 获取场景中所有网格对象
    auto& root = SceneRoot::get();
    std::vector<std::shared_ptr<ObjectMesh>> meshObjects;

    // 遍历场景中的对象
    std::function<void(std::shared_ptr<Object>)> collectMeshObjects = [&](std::shared_ptr<Object> obj) {
        // 尝试转换为 ObjectMesh
        if (auto meshObj = std::dynamic_pointer_cast<ObjectMesh>(obj))
        {
            // 确保是有效的网格对象且不是工具路径相关的可视化对象
            if (meshObj->mesh() && meshObj->mesh()->topology.numValidFaces() > 0 &&
                meshObj->name().find("ToolPath") == std::string::npos &&
                meshObj->name().find("Tool") == std::string::npos &&
                meshObj->name() != "下切区域")
            {
                meshObjects.push_back(meshObj);
            }
        }

        // 递归搜索子对象
        for (const auto& child : obj->children())
        {
            collectMeshObjects(child);
        }
    };

    for (const auto& child : root.children())
    {
        collectMeshObjects(child);
    }

    // 显示模型列表
    if (meshObjects.empty())
    {
        ImGui::Text("无可用模型");
        ImGui::Text("请导入STEP文件或加载网格模型");
    }
    else
    {
        ImGui::Text("选择模型:");

        // 创建模型选择下拉框
        std::vector<const char*> modelNames;
        int currentSelectedIndex = -1;

        for (size_t i = 0; i < meshObjects.size(); ++i)
        {
            modelNames.push_back(meshObjects[i]->name().c_str());
            if (selectedModel_ == meshObjects[i])
            {
                currentSelectedIndex = static_cast<int>(i);
            }
        }

        if (ImGui::Combo("##ModelSelection", &currentSelectedIndex, modelNames.data(), static_cast<int>(modelNames.size())))
        {
            if (currentSelectedIndex >= 0 && currentSelectedIndex < meshObjects.size())
            {
                selectedModel_ = meshObjects[currentSelectedIndex];
                addStatusMessage("已选择模型: " + selectedModel_->name(), StatusMessage::Type::Info);

                // 聚焦到选中的模型
                Viewer::instance()->fitDataViewport();
            }
        }

        // 显示当前选中模型的信息
        if (selectedModel_)
        {
            ImGui::Separator();
            ImGui::Text("当前模型: %s", selectedModel_->name().c_str());

            auto mesh = selectedModel_->mesh();
            if (mesh)
            {
                ImGui::Text("顶点数: %d", mesh->topology.numValidVerts());
                ImGui::Text("面数: %d", mesh->topology.numValidFaces());

                // 显示包围盒信息
                auto box = mesh->computeBoundingBox();
                ImGui::Text("尺寸: %.2f x %.2f x %.2f",
                    box.size().x, box.size().y, box.size().z);

                ImGui::Text("中心: %.2f, %.2f, %.2f",
                    box.center().x, box.center().y, box.center().z);
            }
        }
    }

    ImGui::Separator();

    // 模型操作按钮
    if (ImGui::Button("导入STEP文件", ImVec2(-1, 0)))
    {
        // 使用MeshLib的文件对话框
        auto filename = MR::openFileDialog({ .filters = {{"STEP files (*.step *.stp)", "*.step;*.stp"}} });

        if (!filename.empty())
        {
            importSTEPModel(filename.string());
        }
    }

    if (selectedModel_)
    {
        if (ImGui::Button("聚焦到模型", ImVec2(-1, 0)))
        {
            Viewer::instance()->fitDataViewport();
        }

        if (ImGui::Button("预处理模型", ImVec2(-1, 0)))
        {
            processImportedModel(selectedModel_);
        }
    }
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
    
    // 刀具半径 - 恢复标准范围
    float radius = toolPathParams_.millRadius;
    if (ImGui::SliderFloat("刀具半径 (mm)", &radius, 0.5f, 20.0f))
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
    ToolPathPlugin::HelpMarker("Tool radius affects the actual size and machining precision");

    // 体素大小（用于偏移网格） - 恢复标准范围
    float voxelSize = toolPathParams_.voxelSize;
    if (ImGui::SliderFloat("体素大小 (mm)", &voxelSize, 0.1f, 2.0f))
    {
        toolPathParams_.voxelSize = voxelSize;
        constantCuspParams_.voxelSize = voxelSize;
    }
    ImGui::SameLine();
    ToolPathPlugin::HelpMarker("Voxel size affects toolpath calculation precision. Smaller values provide higher precision but require more computation time");
    
    // 安全高度
    float safeZ = toolPathParams_.safeZ;
    if (ImGui::SliderFloat("安全高度 (mm)", &safeZ, 10.0f, 100.0f))
    {
        toolPathParams_.safeZ = safeZ;
        constantCuspParams_.safeZ = safeZ;
    }
    ImGui::SameLine();
    ToolPathPlugin::HelpMarker("Safe height is the Z level where the tool can move freely without colliding with the workpiece");
    
    // 进给率
    float baseFeed = toolPathParams_.baseFeed;
    if (ImGui::SliderFloat("加工进给率 (mm/min)", &baseFeed, 100.0f, 1000.0f))
    {
        toolPathParams_.baseFeed = baseFeed;
        constantCuspParams_.baseFeed = baseFeed;
    }
    ImGui::SameLine();
    ToolPathPlugin::HelpMarker("Feed rate during machining operations");
    
    float plungeFeed = toolPathParams_.plungeFeed;
    if (ImGui::SliderFloat("下刀进给率 (mm/min)", &plungeFeed, 50.0f, 500.0f))
    {
        toolPathParams_.plungeFeed = plungeFeed;
        constantCuspParams_.plungeFeed = plungeFeed;
    }
    ImGui::SameLine();
    ToolPathPlugin::HelpMarker("Feed rate when plunging down, usually lower than normal feed rate");
    
    float retractFeed = toolPathParams_.retractFeed;
    if (ImGui::SliderFloat("抬刀进给率 (mm/min)", &retractFeed, 100.0f, 1000.0f))
    {
        toolPathParams_.retractFeed = retractFeed;
        constantCuspParams_.retractFeed = retractFeed;
    }
    ImGui::SameLine();
    ToolPathPlugin::HelpMarker("Feed rate when retracting up, usually higher than normal feed rate");
    
    // 网格简化设置
    ImGui::Separator();
    ImGui::Text("网格简化设置");
    
    bool enableSimplify = enableAutoSimplification;
    if (ImGui::Checkbox("自动简化大型网格", &enableSimplify))
    {
        enableAutoSimplification = enableSimplify;
    }
    ImGui::SameLine();
    ToolPathPlugin::HelpMarker("Automatically simplify large meshes to improve performance");
    
    int maxVerts = maxVertexCount;
    if (ImGui::SliderInt("最大顶点数", &maxVerts, 10000, 200000))
    {
        maxVertexCount = maxVerts;
    }
    ImGui::SameLine();
    ToolPathPlugin::HelpMarker("Meshes with more than this many vertices will be automatically simplified");
    
    float simplifyRatio = meshSimplificationRatio;
    if (ImGui::SliderFloat("简化比例", &simplifyRatio, 0.1f, 0.9f))
    {
        meshSimplificationRatio = simplifyRatio;
    }
    ImGui::SameLine();
    ToolPathPlugin::HelpMarker("Mesh ratio to keep after simplification. Lower values mean more simplification and faster processing");
}

// 绘制算法设置面板
void ToolPathPlugin::drawAlgorithmPanel(float /*menuScaling*/)
{
    // 算法选择
    int algorithmIndex = static_cast<int>(selectedAlgorithm_);
    const char* algorithmItems[] = { "分层刀路 (Lacing)", "等高刀路 (Constant-Z)", "等余量刀路 (Constant-Cusp)" };
    if (ImGui::Combo("算法类型", &algorithmIndex, algorithmItems, IM_ARRAYSIZE(algorithmItems)))
    {
        selectedAlgorithm_ = static_cast<Algorithm>(algorithmIndex);
        addStatusMessage("已选择算法: " + std::string(algorithmItems[algorithmIndex]), StatusMessage::Type::Info);
    }
    ImGui::SameLine();
    ToolPathPlugin::HelpMarker("Select different toolpath generation algorithms");

    ImGui::Separator();

    // 通用参数
    ImGui::Text("通用参数");

    // 层高/步长设置 - 恢复标准范围
    float sectionStep = toolPathParams_.sectionStep;
    if (ImGui::SliderFloat("层高/步长 (mm)", &sectionStep, 0.1f, 5.0f))
    {
        toolPathParams_.sectionStep = sectionStep;
        constantCuspParams_.sectionStep = sectionStep;
    }
    ImGui::SameLine();
    ToolPathPlugin::HelpMarker("Layer height affects machining precision and efficiency. Smaller values provide higher precision but require more time");

    // 关键过渡长度 - 恢复标准范围
    float critTransitionLength = toolPathParams_.critTransitionLength;
    if (ImGui::SliderFloat("关键过渡长度 (mm)", &critTransitionLength, 1.0f, 50.0f))
    {
        toolPathParams_.critTransitionLength = critTransitionLength;
    }
    ImGui::SameLine();
    ToolPathPlugin::HelpMarker("Movements longer than this distance will go through safe height");

    ImGui::Separator();

    // 根据选择的算法显示特定参数
    switch (selectedAlgorithm_)
    {
    case Algorithm::Lacing:
        drawLacingParams(0.0f);
        break;
    case Algorithm::ConstantZ:
        drawConstantZParams(0.0f);
        break;
    case Algorithm::ConstantCusp:
        drawConstantCuspParams(0.0f);
        break;
    }
}

// 绘制分层刀路特定参数
void ToolPathPlugin::drawLacingParams(float /*menuScaling*/)
{
    ImGui::Text("分层刀路参数");

    // 切割方向
    int directionIndex = static_cast<int>(cutDirection_);
    const char* directionItems[] = { "X轴方向", "Y轴方向", "Z轴方向" };
    if (ImGui::Combo("切割方向", &directionIndex, directionItems, IM_ARRAYSIZE(directionItems)))
    {
        cutDirection_ = static_cast<Axis>(directionIndex);
    }
    ImGui::SameLine();
    ToolPathPlugin::HelpMarker("Select the primary cutting direction for layered operations");

    // 绕行方向
    int bypassDir = static_cast<int>(toolPathParams_.bypassDir);
    const char* bypassItems[] = { "顺时针", "逆时针" };
    if (ImGui::Combo("绕行方向", &bypassDir, bypassItems, IM_ARRAYSIZE(bypassItems)))
    {
        toolPathParams_.bypassDir = static_cast<BypassDirection>(bypassDir);
    }
    ImGui::SameLine();
    ToolPathPlugin::HelpMarker("Direction for tool to move around the workpiece");
}

// 绘制等高刀路特定参数
void ToolPathPlugin::drawConstantZParams(float /*menuScaling*/)
{
    ImGui::Text("等高刀路参数");

    // 绕行方向
    int bypassDir = static_cast<int>(toolPathParams_.bypassDir);
    const char* bypassItems[] = { "顺时针", "逆时针" };
    if (ImGui::Combo("绕行方向", &bypassDir, bypassItems, IM_ARRAYSIZE(bypassItems)))
    {
        toolPathParams_.bypassDir = static_cast<BypassDirection>(bypassDir);
    }
    ImGui::SameLine();
    ToolPathPlugin::HelpMarker("Direction for tool to move around the workpiece");

    // 平底刀选项
    bool flatTool = toolPathParams_.flatTool;
    if (ImGui::Checkbox("平底刀模式", &flatTool))
    {
        toolPathParams_.flatTool = flatTool;
    }
    ImGui::SameLine();
    ToolPathPlugin::HelpMarker("Enable special processing mode for flat end mills");
}

// 绘制等余量刀路特定参数
void ToolPathPlugin::drawConstantCuspParams(float /*menuScaling*/)
{
    ImGui::Text("等余量刀路参数");

    // 绕行方向
    int bypassDir = static_cast<int>(constantCuspParams_.bypassDir);
    const char* bypassItems[] = { "顺时针", "逆时针" };
    if (ImGui::Combo("绕行方向", &bypassDir, bypassItems, IM_ARRAYSIZE(bypassItems)))
    {
        constantCuspParams_.bypassDir = static_cast<BypassDirection>(bypassDir);
    }
    ImGui::SameLine();
    ToolPathPlugin::HelpMarker("Direction for tool to move around the workpiece");

    // 从中心到边界
    bool fromCenterToBoundary = constantCuspParams_.fromCenterToBoundary;
    if (ImGui::Checkbox("从中心到边界", &fromCenterToBoundary))
    {
        constantCuspParams_.fromCenterToBoundary = fromCenterToBoundary;
    }
    ImGui::SameLine();
    ToolPathPlugin::HelpMarker("Path generation direction: from center outward or from outside inward");

    // 下刀长度
    float plungeLength = constantCuspParams_.plungeLength;
    if (ImGui::SliderFloat("下刀长度 (mm)", &plungeLength, 0.5f, 10.0f))
    {
        constantCuspParams_.plungeLength = plungeLength;
    }
    ImGui::SameLine();
    ToolPathPlugin::HelpMarker("Maximum length for rapid plunge movements");

    // 抬刀长度
    float retractLength = constantCuspParams_.retractLength;
    if (ImGui::SliderFloat("抬刀长度 (mm)", &retractLength, 0.5f, 10.0f))
    {
        constantCuspParams_.retractLength = retractLength;
    }
    ImGui::SameLine();
    ToolPathPlugin::HelpMarker("Maximum length for rapid retract movements");
}

// 绘制路径生成面板
void ToolPathPlugin::drawPathGenerationPanel(float /*menuScaling*/)
{
    if (!selectedModel_)
    {
        ImGui::Text("请先选择模型");
        return;
    }

    ImGui::Text("路径生成操作");

    // 生成当前算法的路径
    std::string buttonText;
    switch (selectedAlgorithm_)
    {
    case Algorithm::Lacing:
        buttonText = "生成分层刀路";
        break;
    case Algorithm::ConstantZ:
        buttonText = "生成等高刀路";
        break;
    case Algorithm::ConstantCusp:
        buttonText = "生成等余量刀路";
        break;
    }

    if (ImGui::Button(buttonText.c_str(), ImVec2(-1, 0)))
    {
        generateCurrentToolPath();
    }

    ImGui::Separator();

    // 生成所有类型的路径
    ImGui::Text("批量生成");

    if (ImGui::Button("生成所有路径类型", ImVec2(-1, 0)))
    {
        addStatusMessage("开始批量生成所有路径类型...", StatusMessage::Type::Info);

        // 生成分层刀路
        generateLacingToolPath();

        // 生成等高刀路
        generateConstantZToolPath();

        // 生成等余量刀路
        generateConstantCuspToolPath();

        addStatusMessage("所有路径类型生成完成", StatusMessage::Type::Success);
    }
    ImGui::SameLine();
    ToolPathPlugin::HelpMarker("Generate toolpaths using all three algorithms for comparison");

    ImGui::Separator();

    // 路径可见性控制
    ImGui::Text("路径显示控制");

    bool lacingVisible = lacingPathVisible_;
    if (ImGui::Checkbox("显示分层刀路", &lacingVisible))
    {
        lacingPathVisible_ = lacingVisible;
        updatePathVisibility();
    }

    bool constantZVisible = constantZPathVisible_;
    if (ImGui::Checkbox("显示等高刀路", &constantZVisible))
    {
        constantZPathVisible_ = constantZVisible;
        updatePathVisibility();
    }

    bool constantCuspVisible = constantCuspPathVisible_;
    if (ImGui::Checkbox("显示等余量刀路", &constantCuspVisible))
    {
        constantCuspPathVisible_ = constantCuspVisible;
        updatePathVisibility();
    }

    // 清除所有路径
    if (ImGui::Button("清除所有路径", ImVec2(-1, 0)))
    {
        // 移除所有路径可视化对象
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

        // 重置生成状态
        lacingPathGenerated_ = false;
        constantZPathGenerated_ = false;
        constantCuspPathGenerated_ = false;

        // 清空命令缓存
        lacingCommands_->clear();
        constantZCommands_->clear();
        constantCuspCommands_->clear();

        addStatusMessage("所有路径已清除", StatusMessage::Type::Info);
    }
}

// 更新路径可见性
void ToolPathPlugin::updatePathVisibility()
{
    // 更新分层刀路可见性
    if (lacingPathObject_)
    {
        lacingPathObject_->setVisible(lacingPathVisible_);
    }
    if (lacingPointsObject_)
    {
        lacingPointsObject_->setVisible(lacingPathVisible_);
    }

    // 更新等高刀路可见性
    if (constantZPathObject_)
    {
        constantZPathObject_->setVisible(constantZPathVisible_);
    }
    if (constantZPointsObject_)
    {
        constantZPointsObject_->setVisible(constantZPathVisible_);
    }

    // 更新等余量刀路可见性
    if (constantCuspPathObject_)
    {
        constantCuspPathObject_->setVisible(constantCuspPathVisible_);
    }
    if (constantCuspPointsObject_)
    {
        constantCuspPointsObject_->setVisible(constantCuspPathVisible_);
    }
}

// 绘制动画控制面板
void ToolPathPlugin::drawAnimationPanel(float /*menuScaling*/)
{
    // 检查是否有可用的路径数据
    bool hasAnyPath = lacingPathGenerated_ || constantZPathGenerated_ || constantCuspPathGenerated_;

    if (!hasAnyPath)
    {
        ImGui::Text("请先生成刀具路径");
        return;
    }

    ImGui::Text("动画控制");

    // 动画播放/停止按钮
    if (!animating_)
    {
        if (ImGui::Button("开始动画 (A)", ImVec2(-1, 0)))
        {
            startAnimation();
        }
    }
    else
    {
        if (ImGui::Button("停止动画 (A)", ImVec2(-1, 0)))
        {
            stopAnimation();
        }
    }

    // 动画速度控制
    float speed = animationSpeed_;
    if (ImGui::SliderFloat("动画速度", &speed, 0.1f, 5.0f))
    {
        animationSpeed_ = speed;
    }
    ImGui::SameLine();
    ToolPathPlugin::HelpMarker("Adjust animation playback speed");

    ImGui::Separator();

    // 当前路径选择（用于动画）
    ImGui::Text("动画路径选择");

    const char* pathItems[] = { "分层刀路", "等高刀路", "等余量刀路" };
    std::vector<int> availablePaths;
    std::vector<const char*> availablePathNames;

    if (lacingPathGenerated_)
    {
        availablePaths.push_back(0);
        availablePathNames.push_back(pathItems[0]);
    }
    if (constantZPathGenerated_)
    {
        availablePaths.push_back(1);
        availablePathNames.push_back(pathItems[1]);
    }
    if (constantCuspPathGenerated_)
    {
        availablePaths.push_back(2);
        availablePathNames.push_back(pathItems[2]);
    }

    if (!availablePathNames.empty())
    {
        // 找到当前选择路径的索引
        int currentSelectionIndex = 0;
        for (size_t i = 0; i < availablePaths.size(); ++i)
        {
            if (availablePaths[i] == currentPathIndex_)
            {
                currentSelectionIndex = static_cast<int>(i);
                break;
            }
        }

        if (ImGui::Combo("选择路径", &currentSelectionIndex, availablePathNames.data(), static_cast<int>(availablePathNames.size())))
        {
            currentPathIndex_ = availablePaths[currentSelectionIndex];
        }
    }

    // 动画进度控制
    ImGui::Text("动画进度");

    // 获取当前路径的命令数量
    int maxCommands = 0;
    switch (currentPathIndex_)
    {
    case 0: // 分层刀路
        maxCommands = static_cast<int>(lacingCommands_->size());
        break;
    case 1: // 等高刀路
        maxCommands = static_cast<int>(constantZCommands_->size());
        break;
    case 2: // 等余量刀路
        maxCommands = static_cast<int>(constantCuspCommands_->size());
        break;
    }

    if (maxCommands > 0)
    {
        int currentIndex = animationCommandIndex_;
        if (ImGui::SliderInt("命令索引", &currentIndex, 0, maxCommands - 1))
        {
            animationCommandIndex_ = currentIndex;
            // TODO: 更新刀具位置到对应的命令位置
        }

        ImGui::Text("进度: %d / %d (%.1f%%)",
            animationCommandIndex_, maxCommands,
            100.0f * animationCommandIndex_ / maxCommands);
    }

    ImGui::Separator();

    // 刀具可视化
    ImGui::Text("刀具显示");

    bool showTool = (toolObject_ != nullptr);
    if (ImGui::Checkbox("显示刀具模型", &showTool))
    {
        if (showTool && !toolObject_)
        {
            // 创建刀具模型
            toolObject_ = createToolModel(toolType_, toolPathParams_.millRadius);
            SceneRoot::get().addChild(toolObject_);
        }
        else if (!showTool && toolObject_)
        {
            // 移除刀具模型
            SceneRoot::get().removeChild(toolObject_);
            toolObject_ = nullptr;
        }
    }
}

// 绘制多视图设置面板
void ToolPathPlugin::drawMultiViewPanel(float /*menuScaling*/)
{
    ImGui::Text("多视图功能");
    ImGui::Text("（此功能待实现）");

    // 基础的多视图开关
    bool multiViewEnabled = multiViewEnabled_;
    if (ImGui::Checkbox("启用多视图", &multiViewEnabled))
    {
        multiViewEnabled_ = multiViewEnabled;
        // TODO: 实现多视图切换逻辑
        if (multiViewEnabled_)
        {
            addStatusMessage("多视图模式已启用", StatusMessage::Type::Info);
        }
        else
        {
            addStatusMessage("多视图模式已禁用", StatusMessage::Type::Info);
        }
    }
}

// 绘制分析面板
void ToolPathPlugin::drawAnalysisPanel(float /*menuScaling*/)
{
    ImGui::Text("路径分析");

    // 路径统计信息
    if (lacingPathGenerated_ || constantZPathGenerated_ || constantCuspPathGenerated_)
    {
        ImGui::Separator();
        ImGui::Text("路径统计对比");

        // 创建表格
        if (ImGui::BeginTable("PathComparison", 4, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg))
        {
            // 表头
            ImGui::TableSetupColumn("算法");
            ImGui::TableSetupColumn("点数");
            ImGui::TableSetupColumn("路径长度 (mm)");
            ImGui::TableSetupColumn("状态");
            ImGui::TableHeadersRow();

            // 分层刀路
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::Text("分层刀路");
            ImGui::TableNextColumn();
            if (lacingPathGenerated_)
            {
                ImGui::Text("%d", static_cast<int>(lacingCommands_->size()));
                ImGui::TableNextColumn();
                float length = calculatePathLength(lacingCommands_.get());
                ImGui::Text("%.2f", length);
                ImGui::TableNextColumn();
                ImGui::TextColored(ImVec4(0.0f, 0.8f, 0.0f, 1.0f), "已生成");
            }
            else
            {
                ImGui::Text("-");
                ImGui::TableNextColumn();
                ImGui::Text("-");
                ImGui::TableNextColumn();
                ImGui::TextColored(ImVec4(0.6f, 0.6f, 0.6f, 1.0f), "未生成");
            }

            // 等高刀路
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::Text("等高刀路");
            ImGui::TableNextColumn();
            if (constantZPathGenerated_)
            {
                ImGui::Text("%d", static_cast<int>(constantZCommands_->size()));
                ImGui::TableNextColumn();
                float length = calculatePathLength(constantZCommands_.get());
                ImGui::Text("%.2f", length);
                ImGui::TableNextColumn();
                ImGui::TextColored(ImVec4(0.0f, 0.0f, 0.8f, 1.0f), "已生成");
            }
            else
            {
                ImGui::Text("-");
                ImGui::TableNextColumn();
                ImGui::Text("-");
                ImGui::TableNextColumn();
                ImGui::TextColored(ImVec4(0.6f, 0.6f, 0.6f, 1.0f), "未生成");
            }

            // 等余量刀路
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::Text("等余量刀路");
            ImGui::TableNextColumn();
            if (constantCuspPathGenerated_)
            {
                ImGui::Text("%d", static_cast<int>(constantCuspCommands_->size()));
                ImGui::TableNextColumn();
                float length = calculatePathLength(constantCuspCommands_.get());
                ImGui::Text("%.2f", length);
                ImGui::TableNextColumn();
                ImGui::TextColored(ImVec4(0.8f, 0.0f, 0.0f, 1.0f), "已生成");
            }
            else
            {
                ImGui::Text("-");
                ImGui::TableNextColumn();
                ImGui::Text("-");
                ImGui::TableNextColumn();
                ImGui::TextColored(ImVec4(0.6f, 0.6f, 0.6f, 1.0f), "未生成");
            }

            ImGui::EndTable();
        }

        ImGui::Separator();

        // 详细分析
        ImGui::Text("详细分析");

        // 显示每个算法的详细信息
        if (lacingPathGenerated_)
        {
            if (ImGui::CollapsingHeader("分层刀路详情"))
            {
                showToolPathInfo(lacingCommands_.get(), "分层刀路");

                // 显示算法特定信息
                ImGui::Text("切割方向: %s",
                    cutDirection_ == Axis::X ? "X轴" :
                    cutDirection_ == Axis::Y ? "Y轴" : "Z轴");

                ImGui::Text("绕行方向: %s",
                    toolPathParams_.bypassDir == BypassDirection::Clockwise ? "顺时针" : "逆时针");
            }
        }

        if (constantZPathGenerated_)
        {
            if (ImGui::CollapsingHeader("等高刀路详情"))
            {
                showToolPathInfo(constantZCommands_.get(), "等高刀路");

                // 显示算法特定信息
                ImGui::Text("平底刀模式: %s", toolPathParams_.flatTool ? "是" : "否");
                ImGui::Text("绕行方向: %s",
                    toolPathParams_.bypassDir == BypassDirection::Clockwise ? "顺时针" : "逆时针");
            }
        }

        if (constantCuspPathGenerated_)
        {
            if (ImGui::CollapsingHeader("等余量刀路详情"))
            {
                showToolPathInfo(constantCuspCommands_.get(), "等余量刀路");

                // 显示算法特定信息
                ImGui::Text("从中心到边界: %s", constantCuspParams_.fromCenterToBoundary ? "是" : "否");
                ImGui::Text("绕行方向: %s",
                    constantCuspParams_.bypassDir == BypassDirection::Clockwise ? "顺时针" : "逆时针");
            }
        }

        ImGui::Separator();

        // 性能分析
        if (ImGui::CollapsingHeader("性能分析"))
        {
            ImGui::Text("加工时间估算 (基于进给率):");

            auto calculateMachineTime = [&](const std::vector<PluginGCommand>* commands, const char* name) {
                if (!commands || commands->empty()) return;

                float totalTime = 0.0f;
                float cuttingTime = 0.0f;
                float rapidTime = 0.0f;

                for (size_t i = 1; i < commands->size(); ++i)
                {
                    const auto& prev = (*commands)[i-1];
                    const auto& curr = (*commands)[i];

                    float dx = curr.x - prev.x;
                    float dy = curr.y - prev.y;
                    float dz = curr.z - prev.z;
                    float distance = std::sqrt(dx*dx + dy*dy + dz*dz);

                    if (curr.type == PluginGCommandType::LinearMove)
                    {
                        float time = distance / (toolPathParams_.baseFeed / 60.0f); // 转换为秒
                        cuttingTime += time;
                    }
                    else if (curr.type == PluginGCommandType::Rapid)
                    {
                        float time = distance / (toolPathParams_.retractFeed / 60.0f); // 转换为秒
                        rapidTime += time;
                    }
                }

                totalTime = cuttingTime + rapidTime;

                ImGui::Text("%s:", name);
                ImGui::Indent();
                ImGui::Text("  切削时间: %.1f 秒", cuttingTime);
                ImGui::Text("  快速移动时间: %.1f 秒", rapidTime);
                ImGui::Text("  总时间: %.1f 秒 (%.1f 分钟)", totalTime, totalTime / 60.0f);
                ImGui::Unindent();
            };

            if (lacingPathGenerated_)
                calculateMachineTime(lacingCommands_.get(), "分层刀路");

            if (constantZPathGenerated_)
                calculateMachineTime(constantZCommands_.get(), "等高刀路");

            if (constantCuspPathGenerated_)
                calculateMachineTime(constantCuspCommands_.get(), "等余量刀路");
        }

        // 导出功能
        ImGui::Separator();
        ImGui::Text("导出功能");

        if (ImGui::Button("导出G代码", ImVec2(-1, 0)))
        {
            // 打开文件保存对话框
            auto filename = MR::saveFileDialog({ .filters = {{"G-Code files (*.gcode *.nc)", "*.gcode;*.nc"}} });

            if (!filename.empty())
            {
                // 根据当前选择的算法导出相应的路径
                const std::vector<PluginGCommand>* commandsToExport = nullptr;

                switch (selectedAlgorithm_)
                {
                case Algorithm::Lacing:
                    if (lacingPathGenerated_) commandsToExport = lacingCommands_.get();
                    break;
                case Algorithm::ConstantZ:
                    if (constantZPathGenerated_) commandsToExport = constantZCommands_.get();
                    break;
                case Algorithm::ConstantCusp:
                    if (constantCuspPathGenerated_) commandsToExport = constantCuspCommands_.get();
                    break;
                }

                if (commandsToExport && !commandsToExport->empty())
                {
                    exportToolPath(filename.string(), ExportFormat::GCode);
                    addStatusMessage("G代码导出完成: " + filename.string(), StatusMessage::Type::Success);
                }
                else
                {
                    addStatusMessage("没有可导出的路径数据", StatusMessage::Type::Error);
                }
            }
        }
    }
    else
    {
        ImGui::Text("请先生成刀具路径以查看分析结果");
    }
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

    // 自动检测模型尺寸并调整刀路参数
    auto box = mesh->computeBoundingBox();
    auto size = box.size();
    float maxDimension = std::max({size.x, size.y, size.z});

    // 如果模型很小（小于5mm），自动调整参数
    if (maxDimension < 5.0f)
    {
        // 根据模型尺寸动态调整参数
        float scaleFactor = maxDimension / 10.0f; // 期望的基准尺寸10mm
        scaleFactor = std::max(0.1f, scaleFactor); // 确保不会太小

        toolPathParams_.millRadius = std::min(toolPathParams_.millRadius, maxDimension * 0.05f); // 刀具半径不超过模型5%
        toolPathParams_.voxelSize = std::min(toolPathParams_.voxelSize, maxDimension * 0.02f);   // 体素大小不超过模型2%
        toolPathParams_.sectionStep = std::min(toolPathParams_.sectionStep, maxDimension * 0.1f); // 层高不超过模型10%

        addStatusMessage(fmt::format("检测到小尺寸模型({}mm)，已自动调整参数: 刀具半径={:.3f}mm, 体素={:.3f}mm, 层高={:.3f}mm",
            maxDimension, toolPathParams_.millRadius, toolPathParams_.voxelSize, toolPathParams_.sectionStep),
            StatusMessage::Type::Info);
    }

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

    // 自动检测模型尺寸并调整刀路参数（与分层刀路一致）
    auto box = mesh->computeBoundingBox();
    auto size = box.size();
    float maxDimension = std::max({size.x, size.y, size.z});

    // 如果模型很小（小于5mm），自动调整参数
    if (maxDimension < 5.0f)
    {
        toolPathParams_.millRadius = std::min(toolPathParams_.millRadius, maxDimension * 0.05f);
        toolPathParams_.voxelSize = std::min(toolPathParams_.voxelSize, maxDimension * 0.02f);
        toolPathParams_.sectionStep = std::min(toolPathParams_.sectionStep, maxDimension * 0.1f);

        addStatusMessage(fmt::format("等高刀路：已自动调整参数适配小模型({}mm)", maxDimension), StatusMessage::Type::Info);
    }

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

    // 自动检测模型尺寸并调整刀路参数（与其他算法保持一致）
    auto box = mesh->computeBoundingBox();
    auto size = box.size();
    float maxDimension = std::max({size.x, size.y, size.z});

    // 如果模型很小（小于5mm），自动调整参数
    if (maxDimension < 5.0f)
    {
        constantCuspParams_.millRadius = std::min(constantCuspParams_.millRadius, maxDimension * 0.05f);
        constantCuspParams_.voxelSize = std::min(constantCuspParams_.voxelSize, maxDimension * 0.02f);
        constantCuspParams_.sectionStep = std::min(constantCuspParams_.sectionStep, maxDimension * 0.1f);

        addStatusMessage(fmt::format("等余量刀路：已自动调整参数适配小模型({}mm)", maxDimension), StatusMessage::Type::Info);
    }

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
bool ToolPathPlugin::onEnable_()
{
    // 插件启用时的处理逻辑
    // 例如：初始化资源，设置初始状态等
    addStatusMessage("Toolpath plugin is enabled", StatusMessage::Type::Info);
    return true;
}

bool ToolPathPlugin::onDisable_()
{
    // 插件禁用时的处理逻辑
    // 例如：清理资源，恢复状态等
    addStatusMessage("Toolpath plugin is disabled", StatusMessage::Type::Info);
    return true;
}

// 实现其他成员函数
bool ToolPathPlugin::generateCurrentToolPath()
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

void ToolPathPlugin::startAnimation()
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

void ToolPathPlugin::stopAnimation()
{
    // 停止动画的逻辑
    if (animating_)
    {
        animating_ = false;
        addStatusMessage("动画已停止", StatusMessage::Type::Info);
    }
}

float ToolPathPlugin::calculatePathLength(const std::vector<PluginGCommand>* commands)
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

// 导出工具路径
void ToolPathPlugin::exportToolPath(const std::string& filename, ExportFormat format)
{
    // 根据当前选择的算法获取相应的路径数据
    const std::vector<PluginGCommand>* commandsToExport = nullptr;

    switch (selectedAlgorithm_)
    {
    case Algorithm::Lacing:
        if (lacingPathGenerated_) commandsToExport = lacingCommands_.get();
        break;
    case Algorithm::ConstantZ:
        if (constantZPathGenerated_) commandsToExport = constantZCommands_.get();
        break;
    case Algorithm::ConstantCusp:
        if (constantCuspPathGenerated_) commandsToExport = constantCuspCommands_.get();
        break;
    }

    if (!commandsToExport || commandsToExport->empty())
    {
        addStatusMessage("没有可导出的路径数据", StatusMessage::Type::Error);
        return;
    }

    try
    {
        std::ofstream file(filename);
        if (!file.is_open())
        {
            addStatusMessage("无法打开文件: " + filename, StatusMessage::Type::Error);
            return;
        }

        // 根据格式导出
        switch (format)
        {
        case ExportFormat::GCode:
            exportAsGCode(file, *commandsToExport);
            break;
        case ExportFormat::APTCL:
            exportAsAPTCL(file, *commandsToExport);
            break;
        case ExportFormat::CSV:
            exportAsCSV(file, *commandsToExport);
            break;
        }

        file.close();
        addStatusMessage("文件导出成功: " + filename, StatusMessage::Type::Success);
    }
    catch (const std::exception& e)
    {
        addStatusMessage("导出失败: " + std::string(e.what()), StatusMessage::Type::Error);
    }
}

// 导出为G代码格式
void ToolPathPlugin::exportAsGCode(std::ofstream& file, const std::vector<PluginGCommand>& commands)
{
    // 写入G代码头部
    file << "; Generated by MeshLib ToolPath Plugin\n";
    file << "; Date: " << getCurrentDateTimeString() << "\n";
    file << "; Algorithm: " << getAlgorithmName(selectedAlgorithm_) << "\n";
    file << "; Tool Radius: " << toolPathParams_.millRadius << " mm\n";
    file << "; Commands Count: " << commands.size() << "\n";
    file << ";\n";

    // 初始化命令
    file << "G21 ; millimeter units\n";
    file << "G90 ; absolute positioning\n";
    file << "G17 ; XY plane selection\n";
    file << "M3 S" << spindle_ << " ; spindle on clockwise\n";
    file << "\n";

    // 导出路径命令
    for (const auto& cmd : commands)
    {
        switch (cmd.type)
        {
        case PluginGCommandType::LinearMove:
            file << fmt::format("G1 X{:.3f} Y{:.3f} Z{:.3f}", cmd.x, cmd.y, cmd.z);
            if (!std::isnan(cmd.f) && cmd.f > 0)
                file << fmt::format(" F{:.1f}", cmd.f);
            file << "\n";
            break;

        case PluginGCommandType::Rapid:
            file << fmt::format("G0 X{:.3f} Y{:.3f} Z{:.3f}\n", cmd.x, cmd.y, cmd.z);
            break;

        case PluginGCommandType::Comment:
            if (cmd.comment)
                file << "; " << *cmd.comment << "\n";
            break;
        }
    }

    // 结束命令
    file << "\n";
    file << "M5 ; spindle stop\n";
    file << "M30 ; program end\n";
}

// 导出为APTCL格式
void ToolPathPlugin::exportAsAPTCL(std::ofstream& file, const std::vector<PluginGCommand>& commands)
{
    file << "$$ Generated by MeshLib ToolPath Plugin\n";
    file << "$$ Date: " << getCurrentDateTimeString() << "\n";
    file << "$$ Algorithm: " << getAlgorithmName(selectedAlgorithm_) << "\n";
    file << "\n";

    file << "PARTNO / TOOLPATH\n";
    file << "CUTTER / " << toolPathParams_.millRadius << "\n";
    file << "SPINDL / " << spindle_ << " , CLW\n";
    file << "\n";

    for (const auto& cmd : commands)
    {
        switch (cmd.type)
        {
        case PluginGCommandType::LinearMove:
            file << fmt::format("GOTO / {:.3f}, {:.3f}, {:.3f}\n", cmd.x, cmd.y, cmd.z);
            break;

        case PluginGCommandType::Rapid:
            file << fmt::format("RAPID\nGOTO / {:.3f}, {:.3f}, {:.3f}\n", cmd.x, cmd.y, cmd.z);
            break;

        case PluginGCommandType::Comment:
            if (cmd.comment)
                file << "$$ " << *cmd.comment << "\n";
            break;
        }
    }

    file << "SPINDL / OFF\n";
    file << "END\n";
}

// 导出为CSV格式
void ToolPathPlugin::exportAsCSV(std::ofstream& file, const std::vector<PluginGCommand>& commands)
{
    // CSV 头部
    file << "Index,Type,X,Y,Z,Feed,Comment\n";

    // 导出数据
    for (size_t i = 0; i < commands.size(); ++i)
    {
        const auto& cmd = commands[i];

        std::string typeStr;
        switch (cmd.type)
        {
        case PluginGCommandType::LinearMove:
            typeStr = "Linear";
            break;
        case PluginGCommandType::Rapid:
            typeStr = "Rapid";
            break;
        case PluginGCommandType::Comment:
            typeStr = "Comment";
            break;
        }

        file << fmt::format("{},{},{:.3f},{:.3f},{:.3f},{:.1f},\"{}\"",
            i,
            typeStr,
            cmd.x, cmd.y, cmd.z,
            std::isnan(cmd.f) ? 0.0f : cmd.f,
            cmd.comment ? *cmd.comment : ""
        );
        file << "\n";
    }
}

// 获取当前日期时间字符串
std::string ToolPathPlugin::getCurrentDateTimeString()
{
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    auto tm = *std::localtime(&time_t);

    std::stringstream ss;
    ss << std::put_time(&tm, "%Y-%m-%d %H:%M:%S");
    return ss.str();
}

// 获取算法名称
std::string ToolPathPlugin::getAlgorithmName(Algorithm algorithm)
{
    switch (algorithm)
    {
    case Algorithm::Lacing:
        return "Lacing (分层刀路)";
    case Algorithm::ConstantZ:
        return "Constant-Z (等高刀路)";
    case Algorithm::ConstantCusp:
        return "Constant-Cusp (等余量刀路)";
    default:
        return "Unknown";
    }
}

} // namespace MR 
