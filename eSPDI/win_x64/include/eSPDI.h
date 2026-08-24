// Compatibility shim: the Linux eSPDI ships a single eSPDI.h, while the
// Windows SDK splits the API across eSPDI_Common.h and eSPDI_DM.h. The
// engine sources include <eSPDI.h> on every platform; this header makes
// that spelling resolve to the Windows pair.
#pragma once

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>

#include "eSPDI_Common.h"
#include "eSPDI_DM.h"
