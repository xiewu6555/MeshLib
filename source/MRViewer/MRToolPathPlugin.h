#pragma once

#include "MRViewer.h"
#include "MRMesh/MRMesh.h"
#include "MRMesh/MRObjectMesh.h"
#include "MRMesh/MRSceneRoot.h"
#include "MRMesh/MRMeshBuilder.h"
#include "MRMesh/MRMeshFwd.h"
#include "MRMesh/MRPlane3.h"
#include "MRMesh/MRVector3.h"
#include "MRMesh/MRMatrix4.h"
#include "MRMesh/MRColor.h"
#include "MRMesh/MRMeshPart.h"
#include "MRMesh/MRAxis.h"
#include "MRViewer/MRStatePlugin.h"
#include "MRViewer/MRImGuiVectorOperators.h"
#include "MRMesh/MRObjectLines.h"
#include "MRMesh/MRPolyline.h"
#include "MRIOExtras/MRStep.h"

#include "MRVoxels/MRToolPath.h"

namespace MR
{

// 工具路径命令类型
enum class PluginGCommandType
{
    LinearMove,  // 直线进给移动
    Rapid,       // 快速定位移动
    Comment      // 注释
};

// 工具路径命令
struct PluginGCommand
{
    PluginGCommandType type;       // 命令类型
    float x, y, z;                 // 坐标
    float f;                       // 进给率
    std::shared_ptr<std::string> comment; // 注释
};

// 刀具类型
enum class ToolType
{
    BallEndMill,   // 球头铣刀
    FlatEndMill,   // 平头铣刀
    ToroidalMill   // 环形铣刀
};

// 算法类型
enum class Algorithm
{
    Lacing,        // 分层水平割线路径
    ConstantZ,     // 等高加工路径
    ConstantCusp   // 等余量加工路径
};

// 导出格式
enum class ExportFormat
{
    GCode,         // G代码
    APTCL,         // APT/CL
    CSV            // CSV
};

// 多视图模式
enum class MultiViewMode
{
    Single,         // 单视图
    HorizontalSplit, // 水平分割
    VerticalSplit,   // 垂直分割
    Grid2x2          // 2x2网格
};

// 视图类型
enum class ViewType
{
    Lacing,         // 分层水平割线视图
    ConstantZ,      // 等高加工视图
    ConstantCusp,   // 等余量加工视图
    All             // 所有视图
};

// 状态消息
struct StatusMessage
{
    enum class Type
    {
        Info,      // 信息
        Warning,   // 警告
        Error,      // 错误
        Success
    };
    
    std::string message;  // 消息内容
    Type type;            // 消息类型
    std::chrono::time_point<std::chrono::steady_clock> timestamp; // 时间戳
};

// 2x2网格配置
struct Grid2x2Config
{
    ViewType topLeft = ViewType::Lacing;
    ViewType topRight = ViewType::ConstantZ;
    ViewType bottomLeft = ViewType::ConstantCusp;
    ViewType bottomRight = ViewType::All;
};

// 水平分割配置
struct HorizontalSplitConfig
{
    ViewType top = ViewType::Lacing;
    ViewType bottom = ViewType::ConstantZ;
};

// 垂直分割配置
struct VerticalSplitConfig
{
    ViewType left = ViewType::Lacing;
    ViewType right = ViewType::ConstantZ;
};

// 切片分析信息
struct SliceAnalysisInfo
{
    Vector3f position;      // 交点位置
    ViewType pathType;      // 路径类型
    PluginGCommand command; // 对应的命令
};

// 工具路径插件
class ToolPathPlugin : public StatePlugin
{
public:
    ToolPathPlugin();
    ~ToolPathPlugin();
    
    // 绘制UI
    void drawDialog(float menuScaling, ImGuiContext* ctx);
    bool onEnable_() override;
    bool onDisable_() override;
    
    // 文件拖放处理
    bool onDropFiles(int count, const char* const* paths);
    
    // 导入STEP模型
    void importSTEPModel(const std::string& filename);
    
    // 生成不同类型的工具路径
    void generateLacingToolPath();
    void generateConstantZToolPath();
    void generateConstantCuspToolPath();
    bool generateCurrentToolPath();
    
    // 处理工具路径结果
    void processToolPathResult(const std::vector<PluginGCommand>& commands, std::shared_ptr<std::vector<PluginGCommand>>& target, bool& generated);
    void updatePathVisibility();
    
    // 动画控制
    void startAnimation();
    void stopAnimation();
    void updateAnimation();
    
    // 多视图布局
    void setupMultiViewLayout(MultiViewMode mode = MultiViewMode::Single);
    
    // 导出工具路径
    void exportToolPath(const std::string& filename, ExportFormat format);
    void handleExportToolPath();
    
    // 模型处理
    bool preprocessModel(std::shared_ptr<ObjectMesh> model);
    void visualizeUndercuts(std::shared_ptr<ObjectMesh> model);
    void simplifyMesh(std::shared_ptr<Object> model, float ratio);
    void processImportedModel(std::shared_ptr<Object> model);
    void generateToolPath();
    
    // 参数预设
    void loadPresetParams(const std::string& presetName);

    // 辅助函数
    std::shared_ptr<ObjectMesh> getSelectedModel() { return selectedModel_; }
    
    // 网格检查和修复
    std::vector<MR::FaceId> findDuplicateFaces(const MR::Mesh& mesh);
    std::vector<MR::UndirectedEdgeId> findNonManifoldEdges(const MR::Mesh& mesh);
    
private:
    // 工具路径可视化
    void createToolPathVisualization(const std::vector<PluginGCommand>& commands, const Color& color, 
                               std::shared_ptr<Object>& pathObject, std::shared_ptr<Object>& pointsObject);
    
    void createLineSegment(const Vector3f& start, const Vector3f& end, const Color& color, std::shared_ptr<Polyline3>& lines);
    
    // 路径分析
    float calculatePathLength(const std::vector<PluginGCommand>* commands);
    void showToolPathInfo(const std::vector<PluginGCommand>* commands, const std::string& pathName);
    void performPathComparison();
    
    // UI面板绘制
    void drawMainPanel(float menuScaling);
    void drawModelSelectionPanel(float menuScaling);
    void drawToolPanel(float menuScaling);
    void drawAlgorithmPanel(float menuScaling);
    void drawLacingParams(float menuScaling);
    void drawConstantZParams(float menuScaling);
    void drawConstantCuspParams(float menuScaling);
    void drawPathGenerationPanel(float menuScaling);
    void drawAnimationPanel(float menuScaling);
    void drawMultiViewPanel(float menuScaling);
    void drawAnalysisPanel(float menuScaling);
    
    void drawGrid2x2Config(float menuScaling);
    void drawHorizontalSplitConfig(float menuScaling);
    void drawVerticalSplitConfig(float menuScaling);
    
    // 状态消息处理
    void addStatusMessage(const std::string& message, StatusMessage::Type type);
    void drawStatusMessages(float menuScaling);
    
    // 切片分析
    void createSlicePlane();
    void updateSliceAnalysis();
    void analyzePathSliceIntersections(const std::vector<PluginGCommand>* commands, const Plane3f& plane, ViewType pathType);
    
    bool rayPlaneIntersection(const Vector3f& rayOrigin, const Vector3f& rayDirection, const Plane3f& plane, float& t);
    
    // 导出不同格式
    void exportAsGCode(std::ofstream& file, const std::vector<PluginGCommand>& commands);
    void exportAsAPTCL(std::ofstream& file, const std::vector<PluginGCommand>& commands);
    void exportAsCSV(std::ofstream& file, const std::vector<PluginGCommand>& commands);
    std::string getCurrentDateTimeString();
    std::string getAlgorithmName(Algorithm algorithm);
    
    // 创建可视化对象
    std::shared_ptr<Object> createToolModel(ToolType type, float radius);
    std::shared_ptr<Object> createSliceIntersectionPoint(const Vector3f& position, ViewType pathType);
    
    // ImGui帮助标记
    static void HelpMarker(const char* desc);
    
    // 当前选择的模型
    std::shared_ptr<ObjectMesh> selectedModel_;
    
    // 加载的模型列表
    std::vector<std::shared_ptr<ObjectMesh>> loadedModels_;
    
    // 动画相关
    bool animating_ = false;
    float animationSpeed_ = 1.0f;
    int animationCommandIndex_ = 0;
    float animationLastTime_ = 0.0f;
    std::shared_ptr<Object> toolModel_;
    
    // 多视图相关
    bool multiViewEnabled_ = false;
    MultiViewMode multiViewMode_ = MultiViewMode::Single;
    Grid2x2Config grid2x2Config_;
    HorizontalSplitConfig horizontalSplitConfig_;
    VerticalSplitConfig verticalSplitConfig_;
    bool syncCameras_ = true;  // 多视图相机同步
    float horizontalSplitRatio_ = 0.5f;  // 水平分割比例
    float verticalSplitRatio_ = 0.5f;    // 垂直分割比例
    
    // 工具路径路径和点对象
    std::shared_ptr<Object> lacingPathObject_;
    std::shared_ptr<Object> lacingPointsObject_;
    std::shared_ptr<Object> constantZPathObject_;
    std::shared_ptr<Object> constantZPointsObject_;
    std::shared_ptr<Object> constantCuspPathObject_;
    std::shared_ptr<Object> constantCuspPointsObject_;
    
    // 工具路径命令容器
    std::shared_ptr<std::vector<PluginGCommand>> lacingCommands_;
    std::shared_ptr<std::vector<PluginGCommand>> constantZCommands_;
    std::shared_ptr<std::vector<PluginGCommand>> constantCuspCommands_;
    
    // 工具路径生成状态
    bool lacingPathGenerated_ = false;
    bool constantZPathGenerated_ = false;
    bool constantCuspPathGenerated_ = false;
    
    // 工具路径可见性状态
    bool lacingPathVisible_ = true;
    bool constantZPathVisible_ = true;
    bool constantCuspPathVisible_ = true;
    
    // 导出设置
    std::string exportFilename_;
    ExportFormat exportFormat_ = ExportFormat::GCode;
    
    // 切片平面相关
    std::shared_ptr<Object> slicePlaneObject_;
    bool slicePlaneEnabled_ = false;
    bool showSlicePlane_ = false;
    Plane3f slicePlane_ = Plane3f(Vector3f(0,0,1), 0); // 默认XY平面
    float sliceHeight_ = 0.0f;
    float slicePlaneHeight_ = 0.0f;
    float slicePlaneHeightMin_ = 0.0f;
    float slicePlaneHeightMax_ = 100.0f;
    std::vector<std::shared_ptr<Object>> slicePointObjects_; // 切片交点对象
    
    std::vector<SliceAnalysisInfo> sliceAnalysisInfo_; // 切片分析信息
    
    // UI面板显示状态
    bool showToolPanel_ = true;
    bool showAlgorithmPanel_ = true;
    bool showPathGenerationPanel_ = true;
    bool showAnimationPanel_ = false;
    bool showMultiViewPanel_ = false;
    bool showAnalysisPanel_ = false;
    
    // 倒角区域分析
    std::shared_ptr<std::vector<bool>> undercut_;  // 倒角标记
    std::shared_ptr<ObjectMesh> undercutVisualization_; // 倒角可视化
    
    // 路径长度计算
    bool pathLengthComputed_ = false;              // 路径长度是否已计算
    float lacingPathLength_ = 0.0f;                // 分层水平割线路径长度
    float constantZPathLength_ = 0.0f;             // 等高加工路径长度
    float constantCuspPathLength_ = 0.0f;          // 等余量加工路径长度
    
    // 状态消息
    std::vector<StatusMessage> statusMessages_;    // 状态消息列表
    
    // 网格简化参数
    float meshSimplificationRatio = 0.5f;  // 网格简化比例
    int maxVertexCount = 50000;            // 最大顶点数
    bool enableAutoSimplification = true;  // 是否启用自动简化
    
    // 刀具相关
    std::shared_ptr<Object> toolObject_;  // 刀具模型对象
    
    // 当前动画索引
    int currentPathIndex_ = 0;
    
    // 使用 MRToolPath.h 中定义的参数类型
    ToolPathParams toolPathParams_;                // 分层和等高路径参数
    ConstantCuspParams constantCuspParams_;        // 等余量路径参数
    ToolType toolType_ = ToolType::BallEndMill;    // 刀具类型
    Axis cutDirection_ = Axis::X;                  // 分层割线方向
    Algorithm selectedAlgorithm_ = Algorithm::Lacing; // 当前选择的算法
    int spindle_ = 6000;                           // 主轴转速
};

// 声明三个转换函数
std::vector<PluginGCommand> lacingToolPath(const Mesh& mesh, Axis direction, const ToolPathParams& params);
std::vector<PluginGCommand> constantZToolPath(const Mesh& mesh, const ToolPathParams& params);
std::vector<PluginGCommand> constantCuspToolPath(const Mesh& mesh, const ConstantCuspParams& params);

} // namespace MR 