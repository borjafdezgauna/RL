#include "stdafx.h"
#include "Actions.h"
#include "EnvironmentManager.h"


CContinuousAction *g_pCAction= 0;
CActionSet *g_pStaticCActionSet= 0;

void AddActionsToAgent(CAgent *pAgent,CContinuousTimeAndActionTransitionFunction *pModel)
{
	g_pCAction= pModel->getContinuousAction();

	if (pAgent)
		pAgent->addAction(g_pCAction);
}

CContinuousAction *GetCAction()
{
	return g_pCAction;
}

CActionSet *GetStaticCActionSet()
{
	return g_pStaticCActionSet;
}