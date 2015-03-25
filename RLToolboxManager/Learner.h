#ifndef __LEARNER__
#define __LEARNER__



class CAgent;
class CAgentController;
class CTDLearner;
class CTransitionFunctionEnvironment;
class CFeatureVFunction;
class CVFunctionLearner;
class CActorFromQFunctionAndPolicy;
class CContinuousActionFeaturePolicy;
class CFeatureCalculator;
class CContinuousActionRandomPolicy;
class CCAPIController;
class CContinuousActionFeaturePolicy;
class CCritic;
class CTDCLambdaVLearner;
class CContinuousActionPolicyNoiseWrapper;

CAgentController *GetController(string controllerType);

class CLearnerManager
{
	string m_actorType;
	string m_criticType;
	string m_controllerType;
	CStateReward *m_pRewardFunction;
	CAgent *m_pAgent;

	//actor-critic
	CCritic *m_pCritic;
	CActor *m_pActor;

	CFeatureVFunction *m_rbfVFunction;


	CContinuousActionRandomPolicy *m_pRandomExploration;
	CContinuousActionFeaturePolicy *m_pInnerPolicy;
	CContinuousActionPolicyNoiseWrapper *m_pPolicy;
	CAgentController *m_pController;

	CStateProperties *m_pStateProperties;
	CTransitionFunctionEnvironment *m_pEnvironment;

	char m_expSetupDescription[1024];

public:
	CLearnerManager();
	~CLearnerManager();

	void init (CAgent *pAgent, CTransitionFunctionEnvironment *pEnvironment
				, CRewardFunction *pReward, string actor_type, string critic_type
				, string controller_type);

	void setExplorationParameter(CAgent *pAgent,double parameter);

	void ExportData(int episode);
	void LoadData();
	//void SaveControllerOutputAsPolicy(const char *pFilenamePolicy,const char* pFilenameValue);

	char* getExperimentalSetupName();



	void saveParameters(CAgent *pAgent,char* pFilename);

	//void addEvaluation(int configIndex,double avgReward);
	//void saveEvaluationResults(const char *pFilename);
};

#endif