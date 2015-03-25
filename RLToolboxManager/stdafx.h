// stdafx.h : include file for standard system include files,
// or project specific include files that are used frequently, but
// are changed infrequently
//

#pragma once

#include "targetver.h"

#include <stdio.h>
#include <tchar.h>

#include "cenvironmentmodel.h"
#include "crewardfunction.h"
#include "caction.h"
#include "cagent.h"
#include "ccontinuousactions.h"
#include "clinearfafeaturecalculator.h"
#include "ctdlearner.h"
#include "cgridworldmodel.h"
#include "cagentlogger.h"
#include "cvfunctionlearner.h"
#include "cactorcritic.h"
#include "cstate.h"
#include "cpolicies.h"
#include "ccontinuousactiongradientpolicy.h"
#include "ctransitionfunction.h"
#include "cevaluator.h"
#include "cstatecollection.h"

extern CParameters *g_pParameters;


#ifdef _DEBUG
	#pragma comment (lib,"../Debug/NewMat-Debug.lib")
	#pragma comment (lib,"../Debug/RLToolbox-v2-Debug.lib")
	#pragma comment (lib,"../Debug/Torch3.lib")
#else
	#pragma comment (lib,"../Release/NewMat.lib")
	#pragma comment (lib,"../Release/RLToolbox-v2.lib")
	#pragma comment (lib,"../Release/Torch3.lib")
#endif


// TODO: reference additional headers your program requires here
