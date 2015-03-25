#include "stdafx.h"
#include "StateManager.h"
//#include "EnvironmentManager.h"


int g_StateModel= 0;

char* GetStateModelName()
{
	switch (g_StateModel)
	{
	case FEATURE_TILE_CODING: return "tiles";
	case FEATURE_RBF: return "rbf";
	case FEATURE_NN: return "nnstate";
	}
	return 0;
}

int GetVariableNumberOfFeatures(char *variableList, char* varName, bool& uniform)
{
	int numFeatures= 0;
	char * token;
	char variableListCopy[1024];
	char matchbuffer[1024];
	strcpy_s(variableListCopy,1024,variableList);
	sprintf_s(matchbuffer,1024,"%s(%%d)",varName);

	token= strtok(variableListCopy,",");
	while (token)
	{
		if (strcmp(varName,token)==0)
			return g_pParameters->getParameter("NUMBER_OF_FEATURES");
		else if (sscanf_s(token,matchbuffer,&numFeatures)==1)
			{
				uniform= strstr(token,"*")==0;
				return numFeatures;
			}
		//if (strcmp(varName,token)==0)
		//	return true;
		token= strtok(0,",");
	}
	return 0;
}

bool UseVariable(char *variableList, char* varName)
{
	char * token;
	char variableListCopy[1024];
	char matchbuffer[1024];
	int numFeatures;
	strcpy_s(variableListCopy,1024,variableList);
	sprintf_s(matchbuffer,1024,"%s(%%d)",varName);
	
	token= strtok(variableListCopy,",");
	while (token)
	{
		if (strcmp(varName,token)==0 || sscanf_s(token,matchbuffer,&numFeatures)==1)
			return true;
		token= strtok(0,",");
	}
	return false;
}
int getNumVariablesUsed(char *variableList)
{
	int numVars= 1;
	int i= 0;

	if (!variableList || variableList[0]==0) return 0;
	while(variableList[i]!=0)
	{
		if (variableList[i]==',') numVars++;
		i++;
	}
	return numVars;
}

CStateModifier* GetContinuousStateModifier(CEnvironmentModel* pEnvironment,int feature_type,char * variableList)
{
	g_StateModel= feature_type;
	CStateModifier* pStateModifier;
	unsigned int *pDimensions;
	unsigned int *pPartitions;
	double *pOffsets;
	double *pSigmas;
	double minV,maxV;
	int numVars= pEnvironment->getNumContinuousStates();
	int numUsedVars= getNumVariablesUsed(variableList);
	CStateProperties *pProperties= pEnvironment->getStateProperties();

	int relVarIndex;

	double *pCentersRBF;
	CSingleStateRBFFeatureCalculator *pRBF;
	CRBFFeatureCalculator *pRBFCalculator;
	CSingleStateLinearInterpolationFeatureCalculator* pLinearIntFeatureCalculator;


	int numFeatures;

	int numStateVarsUsedForLearning= 0;
	char parameterName[256];
	bool bUniform;


	switch (feature_type)
	{
	//case FEATURE_TILE_CODING:
	//	pDimensions= new unsigned int [numUsedVars];
	//	pPartitions= new unsigned int [numUsedVars];
	//	pOffsets= new double [numUsedVars];
	//	pStateModifier= new CFeatureOperatorAnd();
	//	CTilingFeatureCalculator *pTile;

	//	for (int i=0; i<numVars; i++)
	//	{
	//		if (UseVariable(variableList,pProperties->getVarName(i)))
	//		{
	//			pDimensions[numStateVarsUsedForLearning]= i;
	//			
	//			numFeatures= GetVariableNumberOfFeatures(variableList,pProperties->getVarName(i));

	//			pPartitions[numStateVarsUsedForLearning]= numFeatures;

	//			pOffsets[numStateVarsUsedForLearning]= 0.0;
	//			numStateVarsUsedForLearning++;
	//		}
	//	}


	//	pTile= new CTilingFeatureCalculator(numStateVarsUsedForLearning,pDimensions,pPartitions,pOffsets);

	//	((CFeatureOperatorAnd*)pStateModifier)->addStateModifier(pTile);
	//	((CFeatureOperatorAnd*)pStateModifier)->initFeatureOperator();

	//	delete [] pDimensions;
	//	delete [] pPartitions;
	//	delete [] pOffsets;

	//	break;
	//case FEATURE_RBF:
	//	pDimensions= new unsigned int [numUsedVars];
	//	pPartitions= new unsigned int [numUsedVars];
	//	pOffsets= new double [numUsedVars];
	//	pSigmas= new double[numUsedVars];
	//

	//	for (int i=0; i<numVars; i++)
	//	{
	//		if (UseVariable(variableList,pProperties->getVarName(i)))
	//		{
	//			pDimensions[numStateVarsUsedForLearning]= i;
	//			pPartitions[numStateVarsUsedForLearning]= GetVariableNumberOfFeatures(variableList,pProperties->getVarName(i));
	//			pOffsets[numStateVarsUsedForLearning]= 0.0;
	//			pSigmas[numStateVarsUsedForLearning]= 1. / (2. * pPartitions[numStateVarsUsedForLearning]);

	//			numStateVarsUsedForLearning++;
	//		}
	//	}

	//	pLinearIntFeatureCalculator= new CLinearInterpolationFeatureCalculator(numUsedVars,pDimensions,pPartitions,pOffsets);
	//	//pRBFCalculator= new CRBFFeatureCalculator(numUsedVars,pDimensions,pPartitions,pOffsets,pSigmas);
	//	pStateModifier= (CStateModifier*)pLinearIntFeatureCalculator;

	//	delete [] pDimensions;
	//	delete [] pPartitions;
	//	delete [] pOffsets;
	//	delete [] pSigmas;

	//	break;
	case FEATURE_RBF:

		pStateModifier= new CFeatureOperatorAnd();
		

		for (int i=0; i<numVars; i++)
		{
			if (UseVariable(variableList,pProperties->getVarName(i)))
			{
				/*sprintf_s(parameterName,256,"NUM_FEATURES_STATE_DIM_%d",i);
				if (g_pParameters->exists(parameterName))
					numFeatures= g_pParameters->getParameter(parameterName);
				else 
					numFeatures= numFeaturesPSV;*/
				numFeatures= GetVariableNumberOfFeatures(variableList,pProperties->getVarName(i),bUniform);

				pCentersRBF= new double[numFeatures];
				minV= pProperties->getMinValue(i);
				maxV= pProperties->getMaxValue(i);
				if (bUniform)
				{
					for (int j= 0; j<numFeatures; j++)
						pCentersRBF[j]= minV + (((double)j)/(numFeatures-1))*(maxV-minV);
				}
				else
				{
					double normalisedPos;
					double nfeatures= (double)numFeatures;
					for (int j= 0; j<numFeatures; j++)
					{
						normalisedPos=((double)j-nfeatures*.5)/(nfeatures*.5);
						/*if (normalisedPos<0) normalisedPos= -normalisedPos*normalisedPos;
						else normalisedPos*= normalisedPos;*/
						normalisedPos= pow(normalisedPos,3.0);
						pCentersRBF[j]= minV + (normalisedPos+1.0)*0.5*(maxV-minV);
					}
				}
				pRBF= new CSingleStateRBFFeatureCalculator(i,numFeatures,pCentersRBF,3);
				//pLinearIntFeatureCalculator= new CSingleStateLinearInterpolationFeatureCalculator(i,numFeatures,pCentersRBF);//,3);
				((CFeatureOperatorAnd*)pStateModifier)->addStateModifier(pRBF);//pLinearIntFeatureCalculator);
			}
		}

		((CFeatureOperatorAnd*)pStateModifier)->initFeatureOperator();


	break;

	}
	return pStateModifier;
}