#include "stdafx.h"
#include "SetPoint.h"


int countlines(char *filename)
{
  // count the number of lines in the file called filename                                    
  FILE *fp;
  
  fopen_s(&fp,filename,"r");
  int ch=0;
  int lines=1;

  while (!feof(fp))
    {
	ch= getc(fp);
      if (ch == '\n')
    lines++;
    }
  fclose(fp);
  return lines;
}

CSetPointGenerator::CSetPointGenerator(char *pFilename)
{
	char fullFilename[1024];
	m_numSteps= 0;//(int)g_pParameters->getParameter("TIME")/g_pParameters->getParameter("DELTA_T");


	FILE *pFile;
	sprintf_s(fullFilename,1024,"setpoint/%s",pFilename);

	int numLines= countlines(fullFilename);
	if (numLines==0) return;

	char buffer [1024];
	fopen_s(&pFile,fullFilename,"r");
	
	if (pFile!=0)
	{
		m_pSetPoints= new float [numLines];
		m_pTimes= new float [numLines];

		while (!feof(pFile))
		{
			fgets(buffer,1024,pFile);
			if (sscanf_s(buffer,"%f %f\n",&m_pTimes[m_numSteps],&m_pSetPoints[m_numSteps])==2)
				m_numSteps++;
		}
		fclose(pFile);
	}
	else
		printf("ERROR: could not open setpoint file %s\n",pFilename);

	m_totalTime= m_pTimes[m_numSteps-1];
	srand((int)g_pParameters->getParameter("RANDOM_SEED"));
}

CSetPointGenerator::CSetPointGenerator()
{/*

	//srand((int)g_pParameters->getParameter("RANDOM_SEED"));
	srand(3);
	//double setPointChangeProb= g_pParameters->getParameter("SET_POINT_CHANGE_PROB");
	m_numSteps= (int)g_pParameters->getParameter("TIME")/g_pParameters->getParameter("DELTA_T");
	m_pSetPoints= new float [m_numSteps];
	m_pTimes= new float [m_numSteps];

	
	m_pSetPoints[0]= g_pParameters->getParameter("MIN_GOAL")
				+ (g_pParameters->getParameter("MAX_GOAL")-g_pParameters->getParameter("MIN_GOAL"))
				*((double)(rand()%1001)/1001.0);
	m_pTimes[0]= 0.0;

	for (int i= 1; i<m_numSteps; i++)
	{
		m_pSetPoints[i]= g_pParameters->getParameter("MIN_GOAL")
				+ (g_pParameters->getParameter("MAX_GOAL")-g_pParameters->getParameter("MIN_GOAL"))
				*((double)(rand()%1001)/1001.0);
		m_pTimes[i]= m_pTimes[i-1] + 1.5 + 1.0*((double)(rand()%1001)/1001.0);
	}
	m_totalTime= m_pTimes[m_numSteps-1];
	srand((int)g_pParameters->getParameter("RANDOM_SEED"));*/
}
CSetPointGenerator::~CSetPointGenerator()
{
	delete [] m_pSetPoints;
}

float CSetPointGenerator::getPointSet(float time)
{
	int i= 0;

	if (m_totalTime==0) return 0.0;

	while (time>m_totalTime)
		time-= m_totalTime;

	while (i< m_numSteps-1 && time>m_pTimes[i+1])
		i++;

	if (i<m_numSteps-1)
	{
		double u;
		u= ((time-m_pTimes[i])/(m_pTimes[i+1]-m_pTimes[i]));
		return m_pSetPoints[i] + u* (m_pSetPoints[i+1]-m_pSetPoints[i]);
	}

	return m_pSetPoints[m_numSteps-1];//step % m_numSteps];
}