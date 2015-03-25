#pragma once


class CContinuousAction;
class CContinuousActionController;
class CActionSet;

class CSingleLoopPIDController : public CContinuousActionController
{
	double m_accError_x,m_lastError_x;
//	double m_Kp_v,m_Ki_v,m_Kd_v;
	double m_accError_v,m_lastError_v;
	double m_lastX;

	CActionSet *m_pStaticCActionSet;
	CContinuousAction *m_contAction;
public:
	CSingleLoopPIDController(CContinuousAction *contAction, CActionSet *pStaticCActionSet,int randomControllerMode=1);
	~CSingleLoopPIDController();

	virtual void getNextContinuousAction(CStateCollection *state, CContinuousActionData *action);
};


class CDoubleLoopPIDController : public CContinuousActionController
{
	double m_accError_x,m_lastError_x;
//	double m_Kp_v,m_Ki_v,m_Kd_v;
	double m_accError_v,m_lastError_v;
	double m_lastX;

	CActionSet *m_pStaticCActionSet;
	CContinuousAction *m_contAction;
public:
	CDoubleLoopPIDController(CContinuousAction *contAction, CActionSet *pStaticCActionSet,int randomControllerMode=1);
	~CDoubleLoopPIDController();

	virtual void getNextContinuousAction(CStateCollection *state, CContinuousActionData *action);
};
