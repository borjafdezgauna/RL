#ifndef __ACTIONS__
#define __ACTIONS__

class CAgent;
class CContinuousAction;
class CContinuousTimeAndActionTransitionFunction;
#define DISCRETE_ACTIONS 1
#define CONTINUOUS_ACTIONS_UNIFORM_DIST 2
#define CONTINUOUS_ACTIONS_QUADRATIC_DIST 3
#define CONTINUOUS_ACTIONS_CUBIC_DIST 4

void AddActionsToAgent(CAgent *pAgent,CContinuousTimeAndActionTransitionFunction *pModel);
CContinuousAction *GetCAction();
CActionSet *GetStaticCActionSet();

#endif