// ControllerToActorCritic.cpp : Defines the entry point for the console application.
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

void SaveRewardAsValueFunction(const char *pFilenameValue)
{
	CStateProperties *pStateProperties= g_pModel->getStateProperties();

	char paramName[512];
	char variableList[1024];
	CStateCollectionImpl* pStateInterface= new CStateCollectionImpl(pStateProperties);


	
	strcpy_s(variableList,1024,g_pParameters->getStringParameter(string("CRITIC_VARIABLES")).c_str());
	CStateModifier* pStateModifier= GetContinuousStateModifier(g_pModel
							,g_pParameters->getParameter("STATE_MODEL")
							,variableList);

	CFeatureVFunction *pValueFunction = new CFeatureVFunction(pStateModifier);
	double *pValueWeights= pValueFunction->getWeights();

	CState* pState= pStateInterface->getState(pStateProperties);

	CState* pFeatures;
	CContinuousActionData* pActionData= GetCAction()->getContinuousActionData();

	int numFeatures=((CFeatureCalculator*)pStateModifier)->getNumFeatures();

	int index,var_index,point_index;
	
	for (int j=0; j<numFeatures; j++)
	{
		printf("Feature: %d (/%d)\r",j,numFeatures);

		g_pModel->doResetModel();
		g_pModel->getState(pState);
		index= j;


		std::list<CStateModifier *> *modifiers=
				((CFeatureOperatorAnd*)pStateModifier)->getStateModifiers();

		int i= 0;
		for (std::list<CStateModifier*>::iterator it2= modifiers->begin()
				;it2!=modifiers->end(); it2++)
		{
			//var_index= ((CSingleStateRBFFeatureCalculator*)*it2)->getDimension();
			//numFeatures= ((CFeatureCalculator*)*it2)->getNumFeatures();
			//point_index= index % numFeatures;

			//double minV= pStateProperties->getMinValue(var_index);
			//double maxV= pStateProperties->getMaxValue(var_index);
			//double state_value= minV + (((double)point_index)/(numFeatures-1))*(maxV-minV);
			//pState->setContinuousState(var_index,state_value);

			//index= index / numFeatures;
			//i++;

			var_index= ((CSingleStateRBFFeatureCalculator*)*it2)->getDimension();

			numFeatures= ((CFeatureOperatorAnd*)pStateModifier)->getModifierNumOfFeatures(i);
			point_index= index % numFeatures;

			double *pCenters= ((CSingleStateRBFFeatureCalculator*)*it2)->getCenters();
			double state_value= pCenters[point_index];
			pState->setContinuousState(var_index,state_value);

			index= index / numFeatures;
			i++;
		}


//VALUE FUNCTION
		double value= g_pReward->getStateReward(pState);
		pValueWeights[j]= value;

	}

	FILE *pValueFile;
	fopen_s(&pValueFile,pFilenameValue,"wb");
	if (pValueFile)
	{
		pValueFunction->saveData(pValueFile);
		fclose(pValueFile);
	}


	delete pValueFunction;
}

void SaveControllerAsPolicy(const char *pFilenamePolicy)
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
	/*	sprintf_s(paramName,512,"ACTOR_NUM_FEATURES_PSV_DIM_%d",i);*/

		pStateModifier= GetContinuousStateModifier(g_pModel
						,g_pParameters->getParameter("STATE_MODEL")
						/*,g_pParameters->getParameter(paramName)*/
						,variableList);

		pModifierList->push_back((CFeatureCalculator*)pStateModifier);
		pStateInterface->addStateModifier(pStateModifier);
	}

	CContinuousActionFeaturePolicy *pPolicy= 
		new CContinuousActionFeaturePolicy(GetCAction(),pStateProperties,pModifierList);
	double *pPolicyWeights= pPolicy->getWeights();

	int numFeatures;
	double minV,maxV,state_value,value;

	CState* pState= pStateInterface->getState(pStateProperties);

	CState* pFeatures;
	CContinuousActionData* pActionData= GetCAction()->getContinuousActionData();

	std::list<CFeatureCalculator *>::iterator it;
	int actionWeightOffset= 0;

	int actionDim= 0;
	g_pModel->doResetModel();
	g_pModel->getState(pState);
	for (it= pModifierList->begin(); it != pModifierList->end(); it++)
	{
		printf("Action dimension: %d\n",actionDim);

		int numActionFeatures= ((CFeatureCalculator*)*it)->getNumFeatures();

		int index,var_index,point_index;
		
		for (int j=0; j<numActionFeatures; j++)
		{
			printf("Feature: %d (/%d)\r",j,numActionFeatures);

			index= j;

			/*int numStateVariablesUsed= ((CRBFFeatureCalculator*)*it)->getNumDimensions();
			ColumnVector position(numStateVariablesUsed);

			((CRBFFeatureCalculator*)*it)->getFeaturePosition(j,&position);

			for(int i=0; i<numStateVariablesUsed; i++)
			{
				int dim= ((CRBFFeatureCalculator*)*it)->getDimension(i);
				pState->setContinuousState(dim,pStateProperties->getMinValue(dim)
					+ position(i+1) * (pStateProperties->getMaxValue(dim)-pStateProperties->getMinValue(dim)));
			}*/

			std::list<CStateModifier *> *modifiers=
				((CFeatureOperatorAnd*)*it)->getStateModifiers();

			int i= 0;
			double *pCenters;
			for (std::list<CStateModifier*>::iterator it2= modifiers->begin()
				;it2!=modifiers->end(); it2++)
			//for (int i= 0; i<numVars; i++)
			{
				var_index= ((CSingleStateRBFFeatureCalculator*)*it2)->getDimension();
				//var_index= pStateProperties->getVarIndexForLearning(i);
				numFeatures= ((CFeatureOperatorAnd*)*it)->getModifierNumOfFeatures(i);
				point_index= index % numFeatures;

				minV= pStateProperties->getMinValue(var_index);
				maxV= pStateProperties->getMaxValue(var_index);
				pCenters= ((CSingleStateRBFFeatureCalculator*)*it2)->getCenters();
				state_value= pCenters[point_index];
					//minV + (((double)point_index)/(numFeatures-1))*(maxV-minV);
				pState->setContinuousState(var_index,state_value);

				index= index / numFeatures;
				i++;
			}

	//POLICY
			g_pController->getNextContinuousAction(pState, pActionData);
			
			value = pActionData->getActionValue(actionDim);

			pPolicyWeights[j + actionWeightOffset]= value;

			//pPolicy->getNextContinuousAction(pState, pActionData);
		}

		printf("\n");
		actionDim++;
		actionWeightOffset+= numActionFeatures;
	}
	
//#ifdef __DEBUG
	printf("\n\nSANITY CHECK\n\n");
#define NUM_STATE_VALUES 14
#define NUM_TEST_VALUES 2
	double testValues[NUM_TEST_VALUES][NUM_STATE_VALUES];
	//testValues[0]= {600000.00000 599950.26176 477682.85966 20.26000 109008.74590 4.38187 
	//-0.07565 0.46509 -0.01544 136916.16422 2243.51745 -49.73824 -0.01617 
	//2.07640 -0.01544 2243.51745} //log boukhezzar-0-1.txt

	testValues[0][0]= 600000.0; testValues[0][1]= 599950.26176; testValues[0][2]= 477682.85966;
	testValues[0][3]= 20.26000; testValues[0][4]= 109008.74590; testValues[0][5]= 4.38187;
	testValues[0][6]= -0.07565; testValues[0][7]= 0.46509; testValues[0][8]= -0.01544;
	testValues[0][9]= 136916.16422; testValues[0][10]= 2243.51745; testValues[0][11]= -49.73824;
	testValues[0][12]= -0.01617; testValues[0][13]= 2.07640;
	//testValues[1]= {600000.00000 599999.75976 613983.00722 20.70000 135606.27009 4.52770
	//0.00326 0.45347 0.13851 132513.31396 -1828.37350 -0.24024 0.12946 0.09301
	//0.13851 -1828.37350} //log vidal-0-1.txt
	testValues[1][0]= 600000.0; testValues[1][1]= 599999.75976; testValues[1][2]= 613983.00722;
	testValues[1][3]= 20.70000; testValues[1][4]= 135606.27009; testValues[1][5]= 4.52770;
	testValues[1][6]= 0.00326; testValues[1][7]= 0.45347; testValues[1][8]= 0.13851;
	testValues[1][9]= 132513.31396; testValues[1][10]= -1828.37350; testValues[1][11]= -0.24024;
	testValues[1][12]= 0.12946; testValues[1][13]= 0.09301;

	actionWeightOffset= 0;



	for (int i= 0; i<NUM_TEST_VALUES; i++)
	{

		g_pModel->doResetModel();
		g_pModel->getState(pState);

		for (int j=0; j<NUM_STATE_VALUES; j++)
			pState->setContinuousState(j,testValues[i][j]);


		((CContinuousActionController*)g_pController)->getNextContinuousAction(pState, pActionData);

		printf("Controller outputs: %.4f, %.4f\n", pActionData->getActionValue(0), pActionData->getActionValue(1));

		//pPolicy->getNextContinuousAction(pState,pActionData);
		printf("Actor outputs: ");//%.4f, %.4f\n", pActionData->getActionValue(0), pActionData->getActionValue(1));
		int dim= 0;
		actionWeightOffset= 0;
		for (it= pModifierList->begin(); it != pModifierList->end(); it++)
		{	
			pFeatures= pStateInterface->getState((CFeatureCalculator*)(*it));
			(*it)->getModifiedState(pStateInterface,pFeatures);

			int feature_index;
			double factor,value;
			value= 0.0;
			printf("DIMENSION: %d\n",dim);
			printf("Features(Factor)=value:\n");
			for (int feature= 0; min(4,feature<pFeatures->getNumActiveDiscreteStates()); feature++)
			{
				feature_index= pFeatures->getDiscreteState(feature);
				factor= pFeatures->getContinuousState(feature);
				printf("%d(%.4f)=%.5f\n",feature_index,factor,pPolicyWeights[feature_index + actionWeightOffset]);
				value+= factor* pPolicyWeights[feature_index + actionWeightOffset];

			}
			printf("\nValue: %f\n", value);
			actionWeightOffset+= ((CFeatureCalculator*)*it)->getNumFeatures();
			dim++;
		}
		printf("\n");
	
	}

//#endif

	FILE *pPolicyFile,*pValueFile;
	fopen_s(&pPolicyFile,pFilenamePolicy,"wb");
	if (pPolicyFile)
	{
		pPolicy->saveData(pPolicyFile);
		fclose(pPolicyFile);
	}
	
	delete pPolicy;
}

int _tmain(int argc, _TCHAR* argv[])
{
	char parameterFileName[512];

	SetWorkPath();

	if (argc>1) sprintf_s(parameterFileName,256,"%S",argv[1]);
	else sprintf_s(parameterFileName,512,"boukhezzar-to-AC.txt");

	g_pParameters= new CParameters(parameterFileName);
	
	CEnvironmentManager envManager;

	g_pTransitionFunction=
		envManager.getModel(g_pParameters->getStringParameter("ENVIRONMENT"));
	g_pModel= new CTransitionFunctionEnvironment(g_pTransitionFunction);

	
	g_pReward = new CEnvironmentReward(g_pTransitionFunction);

	AddActionsToAgent(0,g_pTransitionFunction);

	g_pController= 
		(CContinuousActionController*)GetController(g_pParameters->getStringParameter("CONTROLLER"));


	char filename[1024];

	if (g_pParameters->exists(string("CONTROLLER_OUTPUT_FILE")))
	{
		//CONTROLLER -> ACTOR
		sprintf_s(filename,1024,"matrices/%s/%s-Policy.bin"
			,g_pParameters->getStringParameter("CONTROLLER_OUTPUT_FILE").c_str()
			,g_pParameters->getStringParameter("LOG_FILE_PREFIX").c_str());
		printf("Controller -> %s\n",filename);

		SaveControllerAsPolicy(filename);


		//REWARD -> VALUE

		sprintf_s(filename,1024,"matrices/%s/%s-Value.bin"
			,g_pParameters->getStringParameter("CONTROLLER_OUTPUT_FILE").c_str()
			,g_pParameters->getStringParameter("LOG_FILE_PREFIX").c_str());
		printf("Reward -> %s\n",filename);
		SaveRewardAsValueFunction(filename);

	


	}
	
}

