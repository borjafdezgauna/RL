#include "stdafx.h"
#include "Learner.h"
#include "EnvironmentManager.h"
#include "Actions.h"
#include "PIController.h"
#include "ccritic.h"
#include "clstd.h"
#include "additionalVLearners.h"
#include "WindTurbine.h"
#include "StateManager.h"

CLearnerManager::CLearnerManager()
{
	m_pController= 0;
	m_pActor= 0;	
	m_pCritic= 0;

	m_pInnerPolicy= 0;



	m_rbfVFunction= 0;

	m_pStateProperties= 0;
}

CLearnerManager::~CLearnerManager()
{
	if (m_pController) {delete m_pController;m_pController= 0;}
	if (m_pActor) {delete m_pActor;m_pActor= 0;}
	if (m_pCritic) {delete m_pCritic; m_pCritic= 0;}
	
	if (m_rbfVFunction) {delete m_rbfVFunction; m_rbfVFunction= 0;}
}

char* CLearnerManager::getExperimentalSetupName()
{
	m_expSetupDescription[0]= 0;
	sprintf_s(m_expSetupDescription,1024,"\n\nActor: %s",m_actorType.c_str());


	strcat_s(m_expSetupDescription,1024,"\nCritic: ");
	strcat_s(m_expSetupDescription,1024,m_criticType.c_str());



	strcat_s(m_expSetupDescription,1024,"\nController: ");
	strcat_s(m_expSetupDescription,1024,m_controllerType.c_str());

	strcat_s(m_expSetupDescription,1024,"\n\n");

	return m_expSetupDescription;
}


void CLearnerManager::init (CAgent *pAgent
								   , CTransitionFunctionEnvironment *pEnvironment
								   , CRewardFunction *pReward
								   , string actor_type
								   , string critic_type
								   , string controller_type)
{
	m_actorType= actor_type;
	m_criticType= critic_type;
	m_controllerType= controller_type;
	m_pRewardFunction= (CStateReward*)pReward;

	m_pEnvironment = pEnvironment;
	m_pStateProperties= pEnvironment->getStateProperties();


	CStateModifier *pStateModifier;

	std::list<CFeatureCalculator *> *pModifierList;
	char paramName[512];
	char variableList[1024];
	

	//ACTOR
	if (m_actorType!=string("NONE"))
	{
		//random controller
		m_pRandomExploration = new CContinuousActionRandomPolicy(GetCAction(),
			g_pParameters->getParameter("INITIAL_RANDOM_SIGMA"),
			g_pParameters->getParameter("INITIAL_RANDOM_ALPHA"));

		pModifierList = new std::list<CFeatureCalculator *>();
		for (int i= 0; i<GetCAction()->getNumDimensions(); i++)
		{
			sprintf_s(paramName,512,"ACTOR_VARIABLES_DIM_%d",i);
			strcpy_s(variableList,1024,g_pParameters->getStringParameter(string(paramName)).c_str());
			//sprintf_s(paramName,512,"ACTOR_NUM_FEATURES_PSV_DIM_%d",i);

			pStateModifier= GetContinuousStateModifier(pEnvironment
							,g_pParameters->getParameter("STATE_MODEL")
							/*,g_pParameters->getParameter(paramName)*/
							,variableList);
			pAgent->addStateModifier(pStateModifier);
			pModifierList->push_back((CFeatureCalculator*)pStateModifier);
		}


		m_pInnerPolicy= 
			new CContinuousActionFeaturePolicy(GetCAction()
			,pEnvironment->getStateProperties(),pModifierList);
		//gradient policy RBF
		m_pPolicy= new CContinuousActionPolicyNoiseWrapper(m_pInnerPolicy,m_pInnerPolicy);
		m_pPolicy->setRandomController(m_pRandomExploration);


		if (m_actorType=="PGAC")
			m_pActor= new CActorFromContinuousActionGradientPolicy(m_pPolicy);
		else if (m_actorType=="CACLA")
			m_pActor= new C_CACLA_Actor(m_pPolicy);
			//m_pActor->setParameter("CACLA-Lambda",g_pParameters->getParameter("CACLA-Lambda"));
		
		m_pActor->setParameter("LearningRate",g_pParameters->getParameter("INITIAL_ACTOR_LEARNING_RATE"));
	}

	//CRITIC
	if(m_criticType!="NONE")
	{
		strcpy_s(variableList,1024,g_pParameters->getStringParameter(string("CRITIC_VARIABLES")).c_str());

		pStateModifier= GetContinuousStateModifier(pEnvironment
							,g_pParameters->getParameter("STATE_MODEL")
							/*,g_pParameters->getParameter("CRITIC_NUM_FEATURES_PSV")*/
							,variableList);
		pAgent->addStateModifier(pStateModifier);

		m_rbfVFunction = new CFeatureVFunction(pStateModifier);
		CContinuousActionPolicyNoiseWrapper *pTargetPolicy;
		CContinuousActionPolicyNoiseWrapper *pBehaviourPolicy;
		if (m_controllerType=="ACTOR")
		{
			pTargetPolicy= m_pPolicy;
		}
		else
			pTargetPolicy= 0;

		if (m_criticType=="TD_LAMBDA")
			m_pCritic= new CTDLambdaVLearner(pReward, m_rbfVFunction
					, new CDiscreteResidual(0.95),0,pTargetPolicy);
		else if(m_criticType=="GTD")
		{
			m_pCritic= (CCritic*) new CGTDVLearner(pReward,m_rbfVFunction
					, new CDiscreteResidual(0.95),0,pTargetPolicy);
		}
		else if(m_criticType=="GTD2")
		{
			m_pCritic= new CGTD2VLearner(pReward,m_rbfVFunction
					, new CDiscreteResidual(0.95),0,pTargetPolicy);
		}
		else if(m_criticType=="TDC")
		{
			m_pCritic= new CTDCLambdaVLearner(pReward,m_rbfVFunction
					, new CDiscreteResidual(0.95),0,pTargetPolicy);
		}
		else if(m_criticType=="BOYAN_LSTD_LAMBDA")
		{
			m_pCritic= new CBoyanVLSTDLambdaLearner(pReward,m_rbfVFunction
					, new CDiscreteResidual(0.95),0,pTargetPolicy);
			m_pCritic->setParameter("VFunction-update-step-freq",100);
		}
		else if(m_criticType=="YU_LSTD_LAMBDA")
		{
			m_pCritic= new CYuVLSTDLambdaLearner(pReward,m_rbfVFunction
					, new CDiscreteResidual(0.95),0,pTargetPolicy);
			m_pCritic->setParameter("VFunction-update-step-freq",100);
		}
		else
			printf("ERROR: invalid critic\n");
		
		if (m_actorType!="NONE")
			m_pCritic->addErrorListener(m_pActor);

		m_pCritic->setParameter("Lambda",0.4);
		m_pCritic->setParameter("ReplacingETraces",0.0);
		m_pCritic->setParameter("ETraceTreshold",0.001);
		m_pCritic->setParameter("ETraceMaxListSize",1000);
	}
	//ORDENA OSO INPORTANTEA DAAAAAA!!!!!!!!!!!!
	//HEMEN ORDUAK GALDU DITTUUUUUT!!!!
	if (m_actorType!="NONE")
		pAgent->addSemiMDPListener(m_pRandomExploration);

	if (m_criticType!="NONE")
		pAgent->addSemiMDPListener(m_pCritic);


	//CONTROLLER
	if (m_controllerType=="ACTOR")
		pAgent->setController(m_pPolicy);
	else
	{
		m_pController= GetController(m_controllerType);
		if (m_pController)
			pAgent->setController(m_pController);
	}
}

CAgentController* GetController(string controllerType)
{

	if (controllerType=="SINGLE_LOOP_PID")
		return new CSingleLoopPIDController(GetCAction(),GetStaticCActionSet(),0);
	if (controllerType=="DOUBLE_LOOP_PID")
		return new CDoubleLoopPIDController(GetCAction(),GetStaticCActionSet(),0);
	if (controllerType=="WIND_TURBINE_MV_CONTROLLER_BOUKHEZZAR")
		return new CWindTurbineControllerBoukhezzar(GetCAction(),GetStaticCActionSet(),0);
	if (controllerType=="WIND_TURBINE_MV_CONTROLLER_VIDAL")
		return new CWindTurbineControllerVidal(GetCAction(),GetStaticCActionSet(),0);
	return 0;
}


void CLearnerManager::setExplorationParameter(CAgent *pAgent,double parameter)
{
	double alpha, sigma, beta;


	if (m_actorType!="NONE")
	{
		sigma= g_pParameters->getParameter("INITIAL_RANDOM_SIGMA");//*(1.0-(1.-parameter)*(1.-parameter));//*parameter;//
		//printf("sigma= %f ", sigma);
		m_pRandomExploration->setParameter("RandomPolicySigma",sigma);
		alpha= g_pParameters->getParameter("INITIAL_RANDOM_ALPHA")*parameter;
		m_pRandomExploration->setParameter("RandomPolicySmoothFactor",alpha);
	}

}



void CLearnerManager::ExportData(int episode)
{
	char dir[512];
	char filename[512];
	char filename2[512];
	FILE *pFile;

	if (!g_pParameters->exists("MATRIX_SAVE_FREQ"))
		return;

	int matrixSaveFreq= g_pParameters->getParameter("MATRIX_SAVE_FREQ");

	if (episode % matrixSaveFreq !=0)
		return;

	sprintf_s(dir,512,"matrices/%s",g_pParameters->getStringParameter("OUTPUT_DIR").c_str());
	_mkdir(dir);

	if (g_pParameters->exists(string("CRITIC_OUTPUT_FILE")))
	{
		sprintf_s(filename,512,"%s/%s-%d.bin"
			,dir,g_pParameters->getStringParameter("CRITIC_OUTPUT_FILE").c_str(),episode);
		printf("Saving data: Critic -> %s\n",filename);
		fopen_s(&pFile,filename,"wb");
		if (pFile)
		{
			m_rbfVFunction->saveData(pFile);
			fclose(pFile);
		}
	}
	if (g_pParameters->exists(string("ACTOR_OUTPUT_FILE")))
	{
		sprintf_s(filename,512,"%s/%s-%d.bin"
			,dir,g_pParameters->getStringParameter("ACTOR_OUTPUT_FILE").c_str(),episode);
		printf("Saving data: Actor -> %s\n",filename);
		fopen_s(&pFile,filename,"wb");
		if (pFile)
		{
			m_pInnerPolicy->saveData(pFile);
			fclose(pFile);
		}
	}
}

void CLearnerManager::LoadData()
{
	char filename[256];
	FILE *pFile;



	if (g_pParameters->exists(string("CRITIC_INPUT_FILE")))
	{
		sprintf_s(filename,256,"matrices/%s/%s.bin"
			,g_pParameters->getStringParameter("INPUT_DIR").c_str()
			,g_pParameters->getStringParameter("CRITIC_INPUT_FILE").c_str());
		printf("Loading data: Critic <- %s ... ",filename);
		fopen_s(&pFile,filename,"rb");
		if (pFile)
		{
			m_rbfVFunction->loadData(pFile);
			fclose(pFile);
			printf("ok\n");
		}
		else printf("failed\n");
	}
	if (g_pParameters->exists(string("ACTOR_INPUT_FILE")))
	{
		sprintf_s(filename,256,"matrices/%s/%s.bin"
			,g_pParameters->getStringParameter("INPUT_DIR").c_str()
			,g_pParameters->getStringParameter("ACTOR_INPUT_FILE").c_str());
		printf("Loading data: Actor <- %s... ",filename);
		fopen_s(&pFile,filename,"rb");
		if (pFile)
		{
			m_pInnerPolicy->loadData(pFile);
			fclose(pFile);
			printf("ok\n");
		}
		else printf("failed\n");
	}
}

void CLearnerManager::saveParameters(CAgent *pAgent,char *pFilename)
{
	if (m_pController)
		m_pController->saveParameters(pFilename,'w');
	if (m_pActor)
		m_pActor->saveParameters(pFilename,'a');
	if (m_pCritic)
		m_pActor->saveParameters(pFilename,'a');
	g_pParameters->saveParameters(pFilename,'a');
}
