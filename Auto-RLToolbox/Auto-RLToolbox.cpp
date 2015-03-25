// Auto-RLToolbox.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "../RLToolboxManager/FileManager.h"

#define MAX_NUM_CHILDREN 16

#ifdef _DEBUG
#define EXE_NAME "..\\Debug\\RLToolboxManager.exe"
#else
#define EXE_NAME "..\\Release\\RLToolboxManager.exe"
#endif

#define MAX_NUM_PARAM_PER_NODE 16
#define PARAMETER_NAME_MAX_SIZE 512
struct Node
{
	int numParameters;
	CParameter parameters[MAX_NUM_PARAM_PER_NODE];
	char parameterNames[MAX_NUM_PARAM_PER_NODE][PARAMETER_NAME_MAX_SIZE];


	int numChildren;
	Node *pChildren[MAX_NUM_CHILDREN];
	Node *pParent;
	Node()
	{
		numParameters= 0;
		numChildren= 0;
		pParent= 0;
		//name[0]= 0;
		//value= -1.0;
	}
	~Node()
	{
		for (int i= 0; i<numChildren; i++) delete pChildren[i];
	}
};

CParameters g_parameters;
int numLeafs= 0;
STARTUPINFO startupInfo;
PROCESS_INFORMATION processInfo;

void TraverseTree(Node* pNode)
{
	char dirName[512];
	char completeDirName[512];
	char fileName[512];
	char commandLine[512];
	FILE* pParameterFile;
	//set parameters, whether is leaf or not
	if (strcmp(pNode->parameterNames[0],"Root")!=0)
	{
		for (int i= 0; i<pNode->numParameters; i++)
		{
			if (strcmp(pNode->parameterNames[i],"BASE_CONFIG_FILE")==0)
				g_parameters.loadParameters((char*)pNode->parameters[i].stringValue.c_str());
			else
				g_parameters.setParameter(pNode->parameterNames[i],pNode->parameters[i]);
		}
	}

	if (pNode->numChildren==0)
	{
		numLeafs++;
		//create dir for experiment

		sprintf_s(dirName,512,"experiments/parameters/%s",g_parameters.getStringParameter("OUTPUT_DIR").c_str());
		sprintf_s(completeDirName,512,"config/experiments/parameters/%s",g_parameters.getStringParameter("OUTPUT_DIR").c_str());
		_mkdir(completeDirName);
		//create parameter file for experiment
		sprintf_s(fileName,512,"%s/parameters.txt",dirName);
		g_parameters.saveParameters(fileName);
		
		//run process

		
		sprintf_s(commandLine,512,"%s %s",EXE_NAME,fileName);
		printf("%s\n",commandLine);
		system(commandLine);

		
		printf("Process Running in dir %s\n",dirName);
		return;
	}
	for (int i= 0; i<pNode->numChildren; i++)
	{
		TraverseTree(pNode->pChildren[i]);
	}
}

int ReadParameter(char* line,char* name,CParameter& parameter)
{
	char buffer[512];
	int ret= sscanf_s(line,"%s : %lf;",name,512,&parameter.numericValue);
	if (ret==2)
	{
		parameter.numeric=true;
		return 1;
	}
	else
	{
		ret= sscanf_s(line,"%s : %[^;\n]s;",name,512,buffer,512);
		//parameter.stringValue);
		
		if (ret==2)
		{
			parameter.stringValue= string(buffer);
			parameter.numeric=false;
			return 1;
		}
		return 0;
	}
}

void ReadNodeParameters(char* pLine,Node *pNode)
{
	int i= 0;
	
	i+=	ReadParameter(pLine,pNode->parameterNames[i],pNode->parameters[i]);

	pLine= strchr(pLine,';');
	while (pLine!=0)
	{
		pLine++;
		
		i+= ReadParameter(pLine,pNode->parameterNames[i],pNode->parameters[i]);

		pLine= strchr(pLine,';');
	}
	pNode->numParameters= i;
}


int _tmain(int argc, _TCHAR* argv[])
{
	char name[64];
	double value;
	Node Root;
	strcpy_s(Root.parameterNames[0],512,"Root");
	Node *pCurrent= &Root;
	Node *pLastNode= pCurrent;
	char line[256];
	FILE *pFile;
	int numParameters= 0;
	int numLines= 0;

	SetWorkPath();

	char experimentFileName[512];
	if (argc>1) sprintf_s(experimentFileName,512,"config/experiments/%S",argv[1]);
	else
	{
		printf("ERROR: no experiment file has been given\n");
		char c;
		scanf_s("%c",&c);
		exit(0);
	}

	fopen_s(&pFile,experimentFileName,"r");
	//fopen_s(&pFile,"experiments.txt","r");
	if (pFile)
	{
		while (fgets(line, sizeof(line), pFile))
		{
			numLines++;
			if (line[0]=='{')
			{
				pCurrent= pLastNode;
			}
			else if (line[0]=='}')
			{
				pCurrent= pCurrent->pParent;
			}
			else //if (sscanf_s(line, "%s : %lf", name,&value)==2)
			{
				pLastNode= new Node;
				pCurrent->pChildren[pCurrent->numChildren]= pLastNode;

				ReadNodeParameters(line,pLastNode);

			/*	strcpy(pLastNode->name[i],name);
				pLastNode->value[i]= value;*/
				pLastNode->pParent= pCurrent;
				pCurrent->numChildren++;
				numParameters++;
			}
			//else printf("ERROR in line %d\n",numLines);
			
		}
		fclose(pFile);
		printf("%d lines and %d parameters read\n",numLines,numParameters);
	}



	/*fopen_s(&pFile,"parameters.txt","r");
	
	g_parameters.loadParameters(pFile);
	
	fclose(pFile);*/

	TraverseTree(&Root);

	return 0;
}

