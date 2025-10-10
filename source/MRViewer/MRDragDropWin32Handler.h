#pragma once
#ifdef _WIN32
#include "MRDragDropHandler.h"
// Include Windows.h first to ensure architecture macros are properly defined
#include <Windows.h>
#include <windef.h>

namespace MR
{

class WinDropTarget;

class DragDropWin32Handler : public IDragDropHandler
{
public:
    DragDropWin32Handler( GLFWwindow* window );
    ~DragDropWin32Handler();
private:
    HWND window_{ nullptr };
    std::unique_ptr<WinDropTarget> winDropTartget_;
};

}
#endif
