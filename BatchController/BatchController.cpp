// BatchController.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "../RLToolboxManager/EnvironmentManager.h"
#include "../RLToolboxManager/RewardManager.h"
#include "../RLToolboxManager/Actions.h"
#include "../RLToolboxManager/StateManager.h"
#include "../RLToolboxManager/Learner.h"
#include "../RLToolboxManager/FileManager.h"

CParameters *g_pParameters;
CContinuousActionController *g_pController;
CContinuousTimeAndActionTransitionFunction *g_pTransitionFunction;
CTransitionFunctionEnvironment *g_pModel;
CStateReward *g_pReward;


void BatchController(/*char *pPolicyFilename,*/char* pInputFilename,char* pOutputFilename)
{
	int numActions= GetCAction()->getNumDimensions();
	
	CStateModifier *pStateModifier;
	CStateProperties *pStateProperties= g_pModel->getStateProperties();

	std::list<CFeatureCalculator *> *pModifierList;
	char paramName[512];
	char variableList[1024];
	CStateCollectionImpl* pStateInterface= new CStateCollectionImpl(pStateProperties);


	pModifierList = new std::list<CFeatureCalculator *>();
	for (int i= 0; i<numActions; i++)
	{
		sprintf_s(paramName,512,"ACTOR_VARIABLES_DIM_%d",i);
		strcpy_s(variableList,1024,g_pParameters->getStringParameter(string(paramName)).c_str());
		/*sprintf_s(paramName,512,"ACTOR_NUM_FEATURES_PSV_DIM_%d",i);*/

		pStateModifier= GetContinuousStateModifier(g_pModel
						,g_pParameters->getParameter("STATE_MODEL")
						/*,g_pParameters->getParameter(paramName)*/
						,variableList);

		pModifierList->push_back((CFeatureCalculator*)pStateModifier);
		pStateInterface->addStateModifier((CFeatureOperatorAnd*)pStateModifier);
	}

	/*CContinuousActionFeaturePolicy *pPolicy= 
		new CContinuousActionFeaturePolicy(GetCAction(),pStateProperties,pModifierList);

	FILE *pPolicyFile;
	fopen_s(&pPolicyFile,pPolicyFilename,"rb");
	if (pPolicyFile)
	{
		pPolicy->loadData(pPolicyFile);
		fclose(pPolicyFile);
	}*/


	CState* pState= pStateInterface->getState(pStateProperties);

	CContinuousActionData* pActionData= GetCAction()->getContinuousActionData();

	int numStateVars= pStateProperties->getNumContinuousStates();
	char* token;
	FILE *pInputFile,*pOutputFile;

	fopen_s(&pOutputFile,pOutputFilename,"w");
	fopen_s(&pInputFile,pInputFilename,"r");
	if (pInputFile && pOutputFile)
	{
		char buffer [1024];
		double value, t;
		fgets(buffer,1024,pInputFile);

		while (fgets(buffer,1024,pInputFile))
		{
			g_pModel->doResetModel();
			g_pModel->getState(pState);

			token= strtok(buffer," ");
			sscanf_s(token,"%lf",&t);//get time

			token= strtok(0," "); //skip the time mark
			int i= 0;
			while (token && i<numStateVars)
			{
				sscanf_s(token,"%lf",&value);
				pState->setContinuousState(i,value);
				i++;
				token= strtok(0," ");
			}

			((CContinuousActionController*)g_pController)->getNextContinuousAction(pState, pActionData);

			fprintf(pOutputFile,"%lf ",t);
			for (int i=0; i<numActions; i++)
			{
				double maxvalue,minvalue;
				maxvalue= GetCAction()->getContinuousActionProperties()->getMaxActionValue(i);
				minvalue= GetCAction()->getContinuousActionProperties()->getMinActionValue(i);
				fprintf(pOutputFile,"%lf ", min(maxvalue,max(minvalue,pActionData->getActionValue(i))));
			}
			fprintf(pOutputFile,"\n");
		}
		fclose(pOutputFile);
		fclose(pInputFile);
	}

	//	pFeatures= pStateInterface->getState((CFeatureOperatorAnd*)m_pStateModifier);
	//	m_pStateModifier->getModifiedState(pStateInterface,pFeatures);

	//	int feature_index;
	//	double factor,value1,value2;
	//	value1= 0.0; value2= 0.0;
	//	for (int feature= 0; feature<pFeatures->getNumDiscreteStates(); feature++)
	//	{
	//		feature_index= pFeatures->getDiscreteState(feature);
	//		factor= pFeatures->getContinuousState(feature);
	//		value1+= factor* pPolicyWeights[feature_index];
	//		value2+= factor* pPolicyWeights[feature_index + numFeaturesTotal];
	//	}
	//	printf("Actor outputs: %.4f, %.4f\n", value1, value2);
	//}
}

int _tmain(int argc, _TCHAR* argv[])
{

	SetWorkPath();

	char parameterFileName[256];

	if (argc>1) sprintf_s(parameterFileName,256,"%S",argv[1]);
	else sprintf(parameterFileName,"boukhezzar-sim.txt");

	g_pParameters= new CParameters(parameterFileName);
	

	CEnvironmentManager envManager;

	g_pTransitionFunction=
		envManager.getModel(g_pParameters->getStringParameter("ENVIRONMENT"));
	g_pModel= new CTransitionFunctionEnvironment(g_pTransitionFunction);

	
	g_pReward = new CEnvironmentReward(g_pTransitionFunction);

	AddActionsToAgent(0,g_pTransitionFunction);
		//,g_pParameters->getParameter("ACTION_MODEL"));

	g_pController= 
		(CContinuousActionController*)GetController(g_pParameters->getStringParameter("CONTROLLER"));



	printf("Batch processing...->batch-output.txt\n");
	BatchController(/*"../RLToolboxManager/experiments/Yolanda-Policy.bin"*/
		/*,*/"../RLToolboxManager/experiments/boukhezzar-0-1.txt"
		,"../RLToolboxManager/experiments/batch-output.txt");

	return 0;
}

