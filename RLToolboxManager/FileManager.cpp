#include "stdafx.h"
#include "FileManager.h"
#include "cparameters.h"
#include "RewardManager.h"



void CLogFile::Init(char* filename)
{
	char dirpath[MAX_FILENAME_LENGTH];
	FILE* pFile;
	string outputDir= g_pParameters->getStringParameter("OUTPUT_DIR");

	sprintf_s(dirpath,MAX_FILENAME_LENGTH, "logs/%s",outputDir.c_str());
	_mkdir(dirpath);

	sprintf_s(m_fullFilename,MAX_FILENAME_LENGTH
		,"%s/%s",dirpath,filename);
	fopen_s(&pFile,m_fullFilename,"w");

	if (pFile)
	{
		//write file header
		fprintf(pFile,"Time");

		int numStateDims= ((CContinuousTimeAndActionTransitionFunction*)m_pTransitionFunction)
									->getStateProperties()->getNumContinuousStates();
		for (int j= 0; j<numStateDims; j++)
		{
			char *pVarname= ((CContinuousTimeAndActionTransitionFunction*)m_pTransitionFunction)
								->getStateProperties()->getVarName(j);
			if (pVarname)
				fprintf(pFile,"/%s",pVarname);
			else fprintf(pFile,"unnamed ");
		}
		int numActionDims= ((CContinuousTimeAndActionTransitionFunction*)m_pTransitionFunction)
								->getContinuousAction()->getNumDimensions();
		for (int j=0; j<numActionDims; j++)
		{
			string actionName= ((CContinuousTimeAndActionTransitionFunction*)m_pTransitionFunction)
								->getContinuousAction()->getContinuousActionProperties()->getActionName(j);
			fprintf(pFile,"/%s",actionName.c_str());
		}

		for (int i=0; i<((CEnvironmentReward*)m_pReward)->getNumRewardComponents();i++)
			fprintf(pFile,"/Reward #%d",i);
		fprintf(pFile,"\n");	
		fclose(pFile);
	}
	else
		printf("ERROR: Could not open %s\n",m_fullFilename);
}

void CLogFile::AddLogLine(void *pState,void* pAction,int nStep)
{
	FILE* pFile;
	fopen_s(&pFile,m_fullFilename,"a");
	fprintf(pFile,"%.5f "
		,((CContinuousTimeAndActionTransitionFunction*)m_pTransitionFunction)
						->getTimeIntervall()*nStep);

	int numStateDims= ((CContinuousTimeAndActionTransitionFunction*)m_pTransitionFunction)
								->getStateProperties()->getNumContinuousStates();
	for (int i=0; i<numStateDims; i++)
		fprintf(pFile,"%.5f ",((CState*)pState)->getContinuousState(i));

	int numActionDims= ((CContinuousTimeAndActionTransitionFunction*)m_pTransitionFunction)
							->getContinuousAction()->getNumDimensions();
	for (int i=0; i<numActionDims; i++)
		fprintf(pFile,"%.5f ",((CContinuousAction*)pAction)->getActionValue(i));

	for (int i=0; i<((CEnvironmentReward*)m_pReward)->getNumRewardComponents();i++)
		fprintf(pFile,"%.5f ",((CEnvironmentReward*)m_pReward)->getLastRewardComponent(i));

	fprintf(pFile,"\n");
	fclose(pFile);
}




void CSimpleLogFile::Init(char* filename)
{
	char dirpath[MAX_FILENAME_LENGTH];
	FILE* pFile;
	string outputDir= g_pParameters->getStringParameter("OUTPUT_DIR");

	sprintf_s(dirpath,MAX_FILENAME_LENGTH, "logs/%s",outputDir.c_str());
	_mkdir(dirpath);

	sprintf_s(m_fullFilename,MAX_FILENAME_LENGTH
		,"%s/%s",dirpath,filename);
	fopen_s(&pFile,m_fullFilename,"w");

	if (pFile) fclose(pFile);
	else printf("ERROR: Could not open %s\n",m_fullFilename);
}

void CSimpleLogFile::AddLogLine(int nStep, double value)
{
	FILE* pFile;
	fopen_s(&pFile,m_fullFilename,"a");

	fprintf(pFile,"%d %f\n",nStep,value);

	fclose(pFile);
}