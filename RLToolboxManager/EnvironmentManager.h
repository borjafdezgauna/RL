#ifndef __ENVIRONMENT_MANAGER__
#define __ENVIRONMENT_MANAGER__

class CContinuousTimeAndActionTransitionFunction;

class CEnvironmentManager
{
	CContinuousTimeAndActionTransitionFunction* m_pModel;
public:
	CEnvironmentManager();
	~CEnvironmentManager();

	int getNumEnvironments();
	CContinuousTimeAndActionTransitionFunction* getModel(string model);

};

#endif