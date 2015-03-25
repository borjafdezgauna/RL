#ifndef __FILE_MANAGER__
#define __FILE_MANAGER__

#define MAX_FILENAME_LENGTH 1024

void SetWorkPath();

class CLogFile
{
	void* m_pTransitionFunction;
	void* m_pReward;
	char m_fullFilename[MAX_FILENAME_LENGTH];
public:
	CLogFile(void* pTransitionFunction, void* pReward)
	{
		m_pTransitionFunction= pTransitionFunction;
		m_pReward= pReward;
		m_fullFilename[0]= 0;
	}
	~CLogFile(){}
	void Init(char* filename);
	void AddLogLine(void *pState, void* pAction, int nStep);
};

class CSimpleLogFile
{
	char m_fullFilename[MAX_FILENAME_LENGTH];
public:
	CSimpleLogFile()
	{
		m_fullFilename[0]= 0;
	}
	~CSimpleLogFile(){}
	void Init(char* filename);
	void AddLogLine(int nStep, double value);
};

#endif