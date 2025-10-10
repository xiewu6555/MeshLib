// Boost 异常处理兼容层
// 解决 Boost 1.86 与项目期望的 1.88 版本差异
#include "MRMeshFwd.h"
#include <exception>
#include <stdexcept>

// 为了避免包含 Boost 头文件导致的冲突，我们直接在 boost 命名空间定义所需函数
// 这些函数由 Boost::serialization 内部调用，需要导出供其他模块使用

namespace boost
{
    // 提供非模板版本的 throw_exception 实现
    // 这是 Boost.Serialization 所需要的版本
    MRMESH_API void throw_exception(std::exception const & e)
    {
        throw e;
    }

    // 某些 Boost 版本可能需要这个重载
    namespace exception_detail
    {
        MRMESH_API void throw_exception_(std::exception const & e)
        {
            throw e;
        }
    }
}