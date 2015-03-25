#ifndef __SET_POINT__
#define __SET_POINT__



class CSetPointGenerator
{
	int m_numSteps;
	float *m_pSetPoints;
	float *m_pTimes;
	float m_totalTime;
public:
	CSetPointGenerator();
	CSetPointGenerator(char* pFilename);
	~CSetPointGenerator();

	float getPointSet(float time);
};

#endif