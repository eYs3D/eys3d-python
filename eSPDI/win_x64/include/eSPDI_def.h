// Compatibility shim: the Linux eSPDI keeps its types, constants, and
// error codes in eSPDI_def.h; the Windows SDK defines them in
// eSPDI_Common.h and eSPDI_ErrCode.h. The engine sources include
// <eSPDI_def.h> on every platform; this header makes that spelling
// resolve to the Windows pair.
#pragma once

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>

#include "eSPDI_Common.h"
#include "eSPDI_ErrCode.h"
