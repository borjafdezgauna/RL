#ifndef __REWARD_MANAGER__
#define __REWARD_MANAGER__

#define NUM_MAX_REWARD_COMPONENTS 10

#define VARIABLE_DIFFERENCE 0
#define DEVIATION_VARIABLE 1
#define CONSTANT_DIFFERENCE 2
#define PUNISH_IF_ABOVE 3
#define PUNISH_IF_BELOW 4

#define MAX_VAR_NAME_SIZE 256

class CErrorComponent
{
	char m_errorComponentType[MAX_VAR_NAME_SIZE];
	char m_controlledVariable[MAX_VAR_NAME_SIZE];
	char m_setpointVariable[MAX_VAR_NAME_SIZE];
	char m_controlErrorVariable[MAX_VAR_NAME_SIZE];

	double m_setpointConstant;
	double m_weight;
	int m_componentIndex;
	//double m_rewardComponentMu;
	double m_tolerance;
	double m_lastReward;
public:
	CErrorComponent();
	~CErrorComponent();

	void init(int componentIndex);
	double getRewardComponent(CState *state);
	double getLastRewardComponent(){return m_lastReward;}
};

class CEnvironmentReward: public CStateReward
{
	int m_numStateVariables;

	CErrorComponent m_errorComponents[NUM_MAX_REWARD_COMPONENTS];
	int m_numRewardComponents;

	int m_rewardSignalFunction;
	double m_lastReward;
public:
	CEnvironmentReward(CContinuousTimeAndActionTransitionFunction *model);
	virtual ~CEnvironmentReward(){};

	virtual double getStateReward(CState *state);
	virtual void getInputDerivation(CState *modelState, ColumnVector *targetState);
	double getLastReward(){return m_lastReward;}
	
	double getLastRewardComponent(int component)
	{
		if (component<m_numRewardComponents)
			return m_errorComponents[component].getLastRewardComponent();
		return 0.0;
	}
	double getNumRewardComponents(){return m_numRewardComponents;}
};

#endif