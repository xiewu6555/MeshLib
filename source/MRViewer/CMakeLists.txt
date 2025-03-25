cmake_minimum_required(VERSION 3.16 FATAL_ERROR)
set(CMAKE_CXX_STANDARD ${MR_CXX_STANDARD})
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(MRViewer CXX)

option(MRVIEWER_NO_GTK "Disable GTK support (affects file dialog support on Linux and macOS)" OFF)
option(MRVIEWER_NO_VOXELS "Disable voxels support" OFF)

IF(NOT MESHLIB_BUILD_VOXELS)
  set(MRVIEWER_NO_VOXELS ON)
ENDIF()

IF(WIN32 OR APPLE OR MR_EMSCRIPTEN)
  set(MRVIEWER_NO_GTK ON)
ENDIF()

file(GLOB HEADERS "*.h")
file(GLOB SOURCES "*.cpp")
set(OBJCPP_SOURCES "")
IF(APPLE)
  file(GLOB OBJCPP_SOURCES "*.mm")
ENDIF()

IF(MR_EMSCRIPTEN)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -s USE_BOOST_HEADERS=1")
ENDIF() # MR_EMSCRIPTEN

add_library(${PROJECT_NAME} SHARED ${SOURCES} ${OBJCPP_SOURCES} ${HEADERS})

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/config_cmake.h.in ${CMAKE_CURRENT_SOURCE_DIR}/config_cmake.h)

set(MRVIEWER_OPTIONAL_DEPENDENCIES "")

IF(NOT MRVIEWER_NO_VOXELS)
  list(APPEND MRVIEWER_OPTIONAL_DEPENDENCIES MRVoxels)
  IF(MESHLIB_USE_OPENVDB)
    IF(APPLE)
      list(APPEND MRVIEWER_OPTIONAL_DEPENDENCIES OpenVDB)
    ELSEIF(WIN32 OR MESHLIB_USE_VCPKG)
      find_package(OpenVDB QUIET)
      if(OpenVDB_FOUND)
        list(APPEND MRVIEWER_OPTIONAL_DEPENDENCIES OpenVDB::openvdb)
        add_compile_definitions(MR_USE_OPENVDB)
      else()
        message(WARNING "OpenVDB not found. Some functionality in MRViewer will be disabled.")
      endif()
    ELSE()
      list(APPEND MRVIEWER_OPTIONAL_DEPENDENCIES openvdb)
    ENDIF()
  ELSE()
    message(STATUS "OpenVDB support is disabled by MESHLIB_USE_OPENVDB option in MRViewer.")
  ENDIF()
ENDIF()

IF(NOT MRVIEWER_NO_GTK)
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(gtk REQUIRED IMPORTED_TARGET gtk+-3.0)
  list(APPEND MRVIEWER_OPTIONAL_DEPENDENCIES PkgConfig::gtk)
ENDIF()

IF(MR_EMSCRIPTEN)
  target_link_libraries(${PROJECT_NAME} PRIVATE
    MRMesh
    MRIOExtras
    MRSymbolMesh
    tbb
    jsoncpp
    imgui
    glfw
    ${MRVIEWER_OPTIONAL_DEPENDENCIES}
  )
ELSEIF(APPLE)
  set_source_files_properties(${OBJCPP_SOURCES} PROPERTIES
    COMPILE_FLAGS "-x objective-c++"
    SKIP_PRECOMPILE_HEADERS ON
  )
  find_library(APPKIT_LIBRARY AppKit REQUIRED)
  target_compile_definitions(${PROJECT_NAME} PRIVATE
    _GNU_SOURCE
  )
  target_link_libraries(${PROJECT_NAME} PRIVATE
    MRMesh
    MRIOExtras
    MRSymbolMesh
    GTest::gtest
    fmt
    tbb
    spdlog
    jsoncpp
    cpr
    imgui
    glad
    glfw
    hidapi
    clip
    ${APPKIT_LIBRARY}
    ${CMAKE_DL_LIBS}
    ${MRVIEWER_OPTIONAL_DEPENDENCIES}
  )
ELSEIF(WIN32 OR MESHLIB_USE_VCPKG)
  find_package(GDCM CONFIG REQUIRED)
  find_package(GTest REQUIRED)
  find_package(cpr REQUIRED)
  find_package(glad REQUIRED)
  find_package(hidapi REQUIRED)

  target_link_libraries(${PROJECT_NAME} PRIVATE
    MRMesh
    MRIOExtras
    MRSymbolMesh
    GTest::gtest
    cpr::cpr
    imgui
    glad::glad
    glfw
    hidapi::hidapi
    JsonCpp::JsonCpp
    spdlog::spdlog
    fmt::fmt
    TBB::tbb
    ${CMAKE_DL_LIBS}
    ${MRVIEWER_OPTIONAL_DEPENDENCIES}
  )
  IF(NOT WIN32)
    find_package(clip CONFIG REQUIRED)
    target_link_libraries(${PROJECT_NAME} PRIVATE clip::clip)
  ENDIF()
ELSE() # None of: MR_EMSCRIPTEN, APPLE, WIN32, vcpkg
  find_package(GDCM CONFIG REQUIRED)

  target_link_libraries(${PROJECT_NAME} PRIVATE
    MRMesh
    MRIOExtras
    MRSymbolMesh
    gtest
    cpr
    imgui
    glad
    glfw
    hidapi-hidraw
    JsonCpp::JsonCpp
    spdlog
    fmt
    tbb
    clip
    ${CMAKE_DL_LIBS}
    ${MRVIEWER_OPTIONAL_DEPENDENCIES}
  )
ENDIF() # APPLE

file(GLOB JSONS "*.json")
file(GLOB AWESOME_FONTS "${MESHLIB_THIRDPARTY_DIR}/fontawesome-free/*.ttf")
file(GLOB IMGUI_FONTS "${MESHLIB_THIRDPARTY_DIR}/imgui/misc/fonts/*.ttf")
IF(MR_EMSCRIPTEN)
  file(GLOB MAIN_FONTS "${MESHLIB_THIRDPARTY_DIR}/Noto_Sans/*.ttf")
  file(COPY ${JSONS} DESTINATION "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/assets")
  file(COPY ${MAIN_FONTS} DESTINATION "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/assets")
  file(COPY ${AWESOME_FONTS} DESTINATION "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/assets")
  file(COPY ${IMGUI_FONTS} DESTINATION "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/assets")
  file(COPY "resource" DESTINATION "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/assets")
  file(COPY "wasm_async_func_list.txt" DESTINATION "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
ELSE() # MR_EMSCRIPTEN
  file(GLOB MAIN_FONTS "${MESHLIB_THIRDPARTY_DIR}/Noto_Sans/*.*")
  file(COPY ${JSONS} DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
  file(COPY ${MAIN_FONTS} DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
  file(COPY ${AWESOME_FONTS} DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
  file(COPY ${IMGUI_FONTS} DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
  file(COPY "resource" DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
ENDIF() # MR_EMSCRIPTEN

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION "${MR_MAIN_LIB_DIR}"
  ARCHIVE DESTINATION "${MR_MAIN_LIB_DIR}"
  RUNTIME DESTINATION "${MR_BIN_DIR}"
)

install(
  FILES ${HEADERS}
  DESTINATION "${MR_INCLUDE_DIR}/${PROJECT_NAME}"
)

install(
  FILES ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}Config.cmake
  DESTINATION ${MR_CONFIG_DIR}
)

install(FILES ${HEADERS} DESTINATION "${MR_INCLUDE_DIR}/${PROJECT_NAME}")
install(FILES ${MAIN_FONTS} DESTINATION "${MR_FONTS_DIR}")
install(FILES ${AWESOME_FONTS} DESTINATION "${MR_FONTS_DIR}")
install(FILES ${IMGUI_FONTS} DESTINATION "${MR_FONTS_DIR}")
install(FILES ${JSONS} DESTINATION "${MR_RESOURCES_DIR}")
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/resource DESTINATION "${MR_RESOURCES_DIR}")

install(
  EXPORT ${PROJECT_NAME}
  FILE ${PROJECT_NAME}Targets.cmake
  NAMESPACE MeshLib::
  DESTINATION ${MR_CONFIG_DIR}
)

IF(MR_PCH)
  TARGET_PRECOMPILE_HEADERS(${PROJECT_NAME} REUSE_FROM MRPch)
ENDIF()

# ToolPath Plugin
set(MR_VIEWER_TOOLPATH_PLUGIN_HEADERS
    MRToolPathPlugin.h
)

set(MR_VIEWER_TOOLPATH_PLUGIN_SOURCES
    MRToolPathPlugin.cpp
)

target_sources(${PROJECT_NAME} PRIVATE ${MR_VIEWER_TOOLPATH_PLUGIN_HEADERS} ${MR_VIEWER_TOOLPATH_PLUGIN_SOURCES})

# 添加依赖库
find_package(spdlog REQUIRED)
target_link_libraries(${PROJECT_NAME} PRIVATE spdlog::spdlog)

# 如果有其他依赖库如CGAL、OpenCASCADE等，也需要在这里添加

# 安装头文件
install(FILES ${MR_VIEWER_TOOLPATH_PLUGIN_HEADERS} DESTINATION "${MR_INSTALL_INCLUDE_DIR}/${PROJECT_NAME}")

# 为IDE创建分组
source_group("Plugin\\ToolPath" FILES ${MR_VIEWER_TOOLPATH_PLUGIN_HEADERS} ${MR_VIEWER_TOOLPATH_PLUGIN_SOURCES})

# 将插件添加到全局插件列表
list(APPEND MR_VIEWER_PLUGINS "ToolPathPlugin")
set(MR_VIEWER_PLUGINS ${MR_VIEWER_PLUGINS} PARENT_SCOPE)

# 注册插件信息到编译配置
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/MRViewerPluginsList.h.in" "${CMAKE_CURRENT_BINARY_DIR}/MRViewerPluginsList.h" @ONLY)
