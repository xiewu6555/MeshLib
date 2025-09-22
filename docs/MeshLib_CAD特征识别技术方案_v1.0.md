# 基于MeshLib的网格CAD模型加工特征识别技术方案

**版本：v1.0**
**日期：2025-01-21**

## 1. 技术概述与架构设计

### 1.1 技术背景

CAD模型的加工特征识别是智能制造的关键技术，通过自动识别孔、槽、凸台等特征，实现工艺规划自动化。本方案基于MeshLib框架，采用二面角特征边检测和属性邻接图(AAG)方法，实现高效的特征识别。

### 1.2 核心架构

```
输入网格模型
    ↓
特征边检测(二面角分析)
    ↓
区域分割(边界追踪)
    ↓
AAG图构建(拓扑关系)
    ↓
规则匹配(特征识别)
    ↓
特征输出
```

### 1.3 关键数据结构

```cpp
namespace MR {

// 特征边类型
enum class FeatureEdgeType {
    None = 0,
    Convex,    // 凸边 (二面角 < π)
    Concave,   // 凹边 (二面角 > π)
    Smooth     // 光滑边 (二面角 ≈ π)
};

// 特征面片
struct FeatureFace {
    FaceId id;
    Vector3f normal;
    std::vector<EdgeId> boundaries;
    FaceType type;  // Planar, Cylindrical, etc.
};

// AAG图节点
struct AAGNode {
    FeatureFace face;
    std::map<FaceId, FeatureEdgeType> adjacency;
};

}
```

## 2. 基于二面角的特征边检测与区域分割

### 2.1 二面角计算原理

二面角是两个相邻面片法向量之间的夹角，用于判断边的凹凸性：

```cpp
float computeDihedralAngle(const Mesh& mesh, EdgeId e) {
    auto [f1, f2] = mesh.topology.getFacesAroundEdge(e);
    if (!f1 || !f2) return 0.0f;

    Vector3f n1 = mesh.facesNormals()[f1];
    Vector3f n2 = mesh.facesNormals()[f2];

    // 计算边向量
    auto [v1, v2] = mesh.topology.getVertIds(e);
    Vector3f edgeVec = (mesh.points[v2] - mesh.points[v1]).normalized();

    // 计算有向二面角
    float cosAngle = dot(n1, n2);
    float sinAngle = dot(cross(n1, n2), edgeVec);

    return atan2(sinAngle, cosAngle) + PI_F;  // 范围[0, 2π]
}
```

### 2.2 特征边检测算法

```cpp
class FeatureEdgeDetector {
public:
    struct Parameters {
        float convexThreshold = 3.0f * PI_F / 4.0f;   // 135°
        float concaveThreshold = 5.0f * PI_F / 4.0f;  // 225°
        float smoothTolerance = PI_F / 36.0f;          // 5°
    };

    EdgeBitSet detectFeatureEdges(const Mesh& mesh,
                                  std::map<EdgeId, FeatureEdgeType>& edgeTypes,
                                  const Parameters& params = {}) {
        EdgeBitSet featureEdges(mesh.topology.edgeSize());

        for (EdgeId e : mesh.topology.getValidEdges()) {
            float angle = computeDihedralAngle(mesh, e);

            if (angle < params.convexThreshold) {
                edgeTypes[e] = FeatureEdgeType::Convex;
                featureEdges.set(e);
            }
            else if (angle > params.concaveThreshold) {
                edgeTypes[e] = FeatureEdgeType::Concave;
                featureEdges.set(e);
            }
            else if (std::abs(angle - PI_F) < params.smoothTolerance) {
                edgeTypes[e] = FeatureEdgeType::Smooth;
            }
        }

        return featureEdges;
    }
};
```

### 2.3 区域分割算法

基于特征边的区域生长算法，将网格分割为特征面片：

```cpp
class RegionSegmentation {
public:
    std::vector<FeatureFace> segment(const Mesh& mesh,
                                     const EdgeBitSet& featureEdges) {
        std::vector<FeatureFace> regions;
        FaceBitSet visited(mesh.topology.faceSize());

        for (FaceId seed : mesh.topology.getValidFaces()) {
            if (visited.test(seed)) continue;

            FeatureFace region = growRegion(mesh, seed, featureEdges, visited);
            if (region.boundaries.size() > 0) {
                region.type = classifyFaceType(mesh, region);
                regions.push_back(region);
            }
        }

        return regions;
    }

private:
    FeatureFace growRegion(const Mesh& mesh, FaceId seed,
                           const EdgeBitSet& featureEdges,
                           FaceBitSet& visited) {
        FeatureFace region;
        region.id = seed;

        std::queue<FaceId> queue;
        queue.push(seed);
        visited.set(seed);

        FaceBitSet regionFaces(mesh.topology.faceSize());
        regionFaces.set(seed);

        while (!queue.empty()) {
            FaceId current = queue.front();
            queue.pop();

            // 获取相邻面片
            for (EdgeId e : mesh.topology.getFaceEdges(current)) {
                if (featureEdges.test(e)) {
                    region.boundaries.push_back(e);
                    continue;  // 特征边，不跨越
                }

                auto [f1, f2] = mesh.topology.getFacesAroundEdge(e);
                FaceId neighbor = (f1 == current) ? f2 : f1;

                if (neighbor && !visited.test(neighbor)) {
                    queue.push(neighbor);
                    visited.set(neighbor);
                    regionFaces.set(neighbor);
                }
            }
        }

        // 计算区域平均法向量
        region.normal = computeAverageNormal(mesh, regionFaces);
        return region;
    }

    FaceType classifyFaceType(const Mesh& mesh, const FeatureFace& face) {
        // 基于曲率和法向量变化判断面片类型
        float maxCurvature = computeMaxCurvature(mesh, face);

        if (maxCurvature < 0.01f) return FaceType::Planar;
        if (isCylindrical(mesh, face)) return FaceType::Cylindrical;
        return FaceType::Freeform;
    }
};
```

## 3. AAG图构建与特征识别算法

### 3.1 AAG图构建

属性邻接图(AAG)编码了特征面片之间的拓扑和几何关系：

```cpp
class AAGBuilder {
public:
    using AAGraph = std::map<FaceId, AAGNode>;

    AAGraph buildAAG(const Mesh& mesh,
                     const std::vector<FeatureFace>& regions,
                     const std::map<EdgeId, FeatureEdgeType>& edgeTypes) {
        AAGraph graph;

        // 创建节点
        for (const auto& region : regions) {
            AAGNode node;
            node.face = region;
            graph[region.id] = node;
        }

        // 建立邻接关系
        for (auto& [id, node] : graph) {
            for (EdgeId e : node.face.boundaries) {
                auto [f1, f2] = mesh.topology.getFacesAroundEdge(e);
                FaceId neighbor = (f1 == id) ? f2 : f1;

                if (neighbor && graph.count(neighbor)) {
                    node.adjacency[neighbor] = edgeTypes.at(e);
                }
            }
        }

        return graph;
    }

    // 计算节点间的几何关系
    struct GeometricRelation {
        float angle;        // 法向量夹角
        float distance;     // 质心距离
        FeatureEdgeType edgeType;
    };

    GeometricRelation computeRelation(const AAGNode& n1,
                                      const AAGNode& n2,
                                      const Mesh& mesh) {
        GeometricRelation rel;
        rel.angle = acos(dot(n1.face.normal, n2.face.normal));

        Vector3f c1 = computeCentroid(mesh, n1.face);
        Vector3f c2 = computeCentroid(mesh, n2.face);
        rel.distance = (c2 - c1).length();

        if (n1.adjacency.count(n2.face.id)) {
            rel.edgeType = n1.adjacency.at(n2.face.id);
        }

        return rel;
    }
};
```

### 3.2 基于规则的特征识别

```cpp
class FeatureRecognizer {
public:
    enum class FeatureType {
        Hole,           // 孔特征
        Slot,           // 槽特征
        Pocket,         // 凹腔
        Boss,           // 凸台
        Step            // 台阶
    };

    struct RecognizedFeature {
        FeatureType type;
        std::vector<FaceId> faces;
        Box3f boundingBox;
        std::map<std::string, float> parameters;  // 特征参数
    };

    std::vector<RecognizedFeature> recognize(const AAGBuilder::AAGraph& aag,
                                            const Mesh& mesh) {
        std::vector<RecognizedFeature> features;

        // 识别孔特征
        auto holes = recognizeHoles(aag, mesh);
        features.insert(features.end(), holes.begin(), holes.end());

        // 识别槽特征
        auto slots = recognizeSlots(aag, mesh);
        features.insert(features.end(), slots.begin(), slots.end());

        // 识别其他特征...

        return features;
    }

private:
    std::vector<RecognizedFeature> recognizeHoles(const AAGBuilder::AAGraph& aag,
                                                  const Mesh& mesh) {
        std::vector<RecognizedFeature> holes;

        for (const auto& [id, node] : aag) {
            if (node.face.type != FaceType::Cylindrical) continue;

            // 检查是否为内圆柱面
            bool isInternalCylinder = true;
            for (const auto& [neighborId, edgeType] : node.adjacency) {
                if (edgeType != FeatureEdgeType::Concave) {
                    isInternalCylinder = false;
                    break;
                }
            }

            if (isInternalCylinder) {
                RecognizedFeature hole;
                hole.type = FeatureType::Hole;
                hole.faces.push_back(id);

                // 计算孔参数
                auto [center, radius, height] = fitCylinder(mesh, node.face);
                hole.parameters["diameter"] = radius * 2;
                hole.parameters["depth"] = height;

                // 查找底面
                for (const auto& [neighborId, edgeType] : node.adjacency) {
                    const auto& neighbor = aag.at(neighborId);
                    if (neighbor.face.type == FaceType::Planar &&
                        std::abs(dot(neighbor.face.normal, Vector3f(0,0,-1))) > 0.9f) {
                        hole.faces.push_back(neighborId);
                        break;
                    }
                }

                hole.boundingBox = computeBoundingBox(mesh, hole.faces);
                holes.push_back(hole);
            }
        }

        return holes;
    }

    std::vector<RecognizedFeature> recognizeSlots(const AAGBuilder::AAGraph& aag,
                                                  const Mesh& mesh) {
        std::vector<RecognizedFeature> slots;

        // 槽特征识别规则：
        // 1. 底面为平面
        // 2. 两个平行侧壁
        // 3. 两个半圆柱端面（可选）

        for (const auto& [id, node] : aag) {
            if (node.face.type != FaceType::Planar) continue;

            std::vector<FaceId> sidewalls;
            std::vector<FaceId> endwalls;

            for (const auto& [neighborId, edgeType] : node.adjacency) {
                if (edgeType != FeatureEdgeType::Concave) continue;

                const auto& neighbor = aag.at(neighborId);

                // 检查是否为侧壁
                if (neighbor.face.type == FaceType::Planar &&
                    std::abs(dot(neighbor.face.normal, node.face.normal)) < 0.1f) {
                    sidewalls.push_back(neighborId);
                }
                // 检查是否为圆柱端面
                else if (neighbor.face.type == FaceType::Cylindrical) {
                    endwalls.push_back(neighborId);
                }
            }

            if (sidewalls.size() == 2) {
                // 验证侧壁平行
                Vector3f n1 = aag.at(sidewalls[0]).face.normal;
                Vector3f n2 = aag.at(sidewalls[1]).face.normal;

                if (std::abs(dot(n1, n2) + 1.0f) < 0.1f) {  // 反向平行
                    RecognizedFeature slot;
                    slot.type = FeatureType::Slot;
                    slot.faces.push_back(id);  // 底面
                    slot.faces.insert(slot.faces.end(), sidewalls.begin(), sidewalls.end());
                    slot.faces.insert(slot.faces.end(), endwalls.begin(), endwalls.end());

                    // 计算槽参数
                    slot.parameters["width"] = computeDistance(mesh, sidewalls[0], sidewalls[1]);
                    slot.parameters["length"] = computeSlotLength(mesh, slot.faces);
                    slot.parameters["depth"] = computeSlotDepth(mesh, id, sidewalls);

                    slot.boundingBox = computeBoundingBox(mesh, slot.faces);
                    slots.push_back(slot);
                }
            }
        }

        return slots;
    }
};
```

## 4. 核心代码实现框架

### 4.1 完整的特征识别管道

```cpp
class MachiningFeatureRecognition {
public:
    struct Configuration {
        FeatureEdgeDetector::Parameters edgeParams;
        float minFeatureSize = 1.0f;
        bool mergeAdjacentFeatures = true;
    };

    struct Result {
        std::vector<FeatureRecognizer::RecognizedFeature> features;
        AAGBuilder::AAGraph aag;
        std::map<EdgeId, FeatureEdgeType> edgeTypes;
        double processingTime;
    };

    Result process(const Mesh& mesh, const Configuration& config = {}) {
        auto startTime = std::chrono::high_resolution_clock::now();
        Result result;

        // 步骤1: 特征边检测
        FeatureEdgeDetector detector;
        EdgeBitSet featureEdges = detector.detectFeatureEdges(
            mesh, result.edgeTypes, config.edgeParams
        );

        // 步骤2: 区域分割
        RegionSegmentation segmentation;
        std::vector<FeatureFace> regions = segmentation.segment(mesh, featureEdges);

        // 步骤3: 构建AAG
        AAGBuilder builder;
        result.aag = builder.buildAAG(mesh, regions, result.edgeTypes);

        // 步骤4: 特征识别
        FeatureRecognizer recognizer;
        result.features = recognizer.recognize(result.aag, mesh);

        // 步骤5: 后处理
        if (config.mergeAdjacentFeatures) {
            mergeAdjacentFeatures(result.features, mesh);
        }
        filterSmallFeatures(result.features, config.minFeatureSize);

        auto endTime = std::chrono::high_resolution_clock::now();
        result.processingTime = std::chrono::duration<double>(
            endTime - startTime
        ).count();

        return result;
    }

private:
    void mergeAdjacentFeatures(std::vector<FeatureRecognizer::RecognizedFeature>& features,
                               const Mesh& mesh) {
        // 合并相邻的同类特征
        for (size_t i = 0; i < features.size(); ++i) {
            for (size_t j = i + 1; j < features.size(); ++j) {
                if (features[i].type == features[j].type &&
                    areAdjacent(features[i], features[j], mesh)) {
                    // 合并特征
                    features[i].faces.insert(features[i].faces.end(),
                                           features[j].faces.begin(),
                                           features[j].faces.end());
                    features[i].boundingBox = features[i].boundingBox.merged(features[j].boundingBox);
                    features.erase(features.begin() + j);
                    --j;
                }
            }
        }
    }

    void filterSmallFeatures(std::vector<FeatureRecognizer::RecognizedFeature>& features,
                            float minSize) {
        features.erase(
            std::remove_if(features.begin(), features.end(),
                [minSize](const auto& f) {
                    return f.boundingBox.diagonal() < minSize;
                }),
            features.end()
        );
    }
};
```

### 4.2 与MeshLib的集成

```cpp
// 插件接口
class MachiningFeaturePlugin : public MR::ViewerPlugin {
public:
    virtual void init(MR::Viewer* viewer) override {
        viewer_ = viewer;
    }

    virtual void drawDialog(float menuScaling) override {
        if (!ImGui::Begin("Machining Features")) {
            ImGui::End();
            return;
        }

        if (ImGui::Button("Detect Features")) {
            detectFeatures();
        }

        if (!features_.empty()) {
            ImGui::Text("Found %d features", (int)features_.size());

            for (size_t i = 0; i < features_.size(); ++i) {
                if (ImGui::TreeNode(("Feature " + std::to_string(i)).c_str())) {
                    displayFeatureInfo(features_[i]);
                    ImGui::TreePop();
                }
            }
        }

        ImGui::End();
    }

private:
    void detectFeatures() {
        auto objMesh = getSelectedMesh();
        if (!objMesh) return;

        MachiningFeatureRecognition recognition;
        auto result = recognition.process(objMesh->mesh());

        features_ = result.features;

        // 可视化特征
        visualizeFeatures(objMesh->mesh());
    }

    void visualizeFeatures(const Mesh& mesh) {
        // 为不同特征分配颜色
        std::map<FeatureRecognizer::FeatureType, Color> colorMap = {
            {FeatureRecognizer::FeatureType::Hole, Color::red()},
            {FeatureRecognizer::FeatureType::Slot, Color::blue()},
            {FeatureRecognizer::FeatureType::Pocket, Color::green()},
            {FeatureRecognizer::FeatureType::Boss, Color::yellow()},
            {FeatureRecognizer::FeatureType::Step, Color::magenta()}
        };

        VertColors colors(mesh.topology.vertSize(), Color::gray());

        for (const auto& feature : features_) {
            Color featureColor = colorMap[feature.type];
            for (FaceId f : feature.faces) {
                for (VertId v : mesh.topology.getFaceVerts(f)) {
                    colors[v] = featureColor;
                }
            }
        }

        getSelectedMesh()->setVertsColorMap(std::move(colors));
    }

    MR::Viewer* viewer_ = nullptr;
    std::vector<FeatureRecognizer::RecognizedFeature> features_;
};

// 注册插件
MR_REGISTER_VIEWER_PLUGIN(MachiningFeaturePlugin)
```

## 5. 工程化应用指南

### 5.1 性能优化策略

1. **并行化处理**
```cpp
// 使用TBB并行处理区域分割
tbb::parallel_for(tbb::blocked_range<size_t>(0, seeds.size()),
    [&](const tbb::blocked_range<size_t>& range) {
        for (size_t i = range.begin(); i != range.end(); ++i) {
            localRegions[i] = growRegion(mesh, seeds[i], featureEdges);
        }
    });
```

2. **空间索引优化**
```cpp
// 使用KD-Tree加速邻域查询
AABTree tree(mesh);
tree.buildTree();
```

### 5.2 鲁棒性增强

1. **噪声处理**
```cpp
// 预处理：网格平滑
mesh = smoothMesh(mesh, SmoothingParams{.iterations = 3});
```

2. **边界条件处理**
```cpp
// 处理非流形边和边界边
if (mesh.topology.isBoundaryEdge(e)) {
    // 特殊处理边界情况
}
```

### 5.3 扩展性设计

1. **自定义特征规则**
```cpp
class CustomFeatureRule {
    virtual bool match(const AAGNode& node,
                       const AAGBuilder::AAGraph& graph) = 0;
    virtual RecognizedFeature extract(const AAGNode& node,
                                     const Mesh& mesh) = 0;
};
```

2. **机器学习集成**
```cpp
// 预留机器学习接口
class MLFeatureClassifier {
    std::vector<float> extractFeatures(const AAGNode& node);
    FeatureType predict(const std::vector<float>& features);
};
```

### 5.4 实际应用建议

1. **参数调优**
   - 凸边阈值：120°-150°（根据模型精度调整）
   - 凹边阈值：210°-240°（根据特征类型调整）
   - 最小特征尺寸：模型最小尺寸的1%

2. **质量验证**
   - 检查特征完整性
   - 验证拓扑一致性
   - 确保参数准确性

3. **与CAM系统集成**
   - 输出标准特征格式（STEP AP224）
   - 提供工艺参数建议
   - 支持刀具路径生成

## 总结

本技术方案提供了完整的基于MeshLib的加工特征识别解决方案，从理论原理到工程实现，涵盖了特征边检测、区域分割、AAG构建和规则识别的全流程。方案具有以下特点：

1. **算法完备**：基于成熟的二面角分析和AAG方法
2. **易于集成**：与MeshLib框架无缝对接
3. **性能优异**：支持并行处理和优化策略
4. **扩展性强**：预留机器学习和自定义规则接口

通过本方案，可快速构建高效、准确的CAD模型加工特征识别系统，为智能制造提供核心技术支撑。

---

**参考资料**
- Joshi, S., & Chang, T. C. (1988). Graph-based heuristics for recognition of machined features from a 3D solid model
- Babic, B., et al. (2008). A review of automated feature recognition with rule-based pattern recognition
- MeshLib Documentation: https://github.com/MeshInspector/MeshLib