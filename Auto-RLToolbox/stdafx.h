// stdafx.h : include file for standard system include files,
// or project specific include files that are used frequently, but
// are changed infrequently
//

#pragma once

#include "targetver.h"

#include <stdio.h>
#include <tchar.h>
#include <string.h>
#include <math.h>
#include <assert.h>
#include <cparameters.h>
#include <windows.h>
#include <direct.h>
#ifdef _DEBUG
	#pragma comment (lib,"../Debug/RLToolbox-v2-Debug.lib")
#else
	#pragma comment (lib,"../Release/RLToolbox-v2.lib")
#endif

// TODO: reference additional headers your program requires here
