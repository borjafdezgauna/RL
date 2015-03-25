#ifndef __MAZE__
#define __MAZE__




class CMaze: public CEnvironmentModel, public CRewardFunction
{
protected:
	double m_x, m_y;
	virtual void doNextState(CPrimitiveAction *action);
public:
	CMaze();
	virtual ~CMaze();

	virtual double getReward(CStateCollection *oldState,CAction *action, CStateCollection *newState);
	virtual void getState(CState *state);
	virtual void doResetModel();

	double getSizeX();
	double getSizeY();
};

class CMazeAction:public CPrimitiveAction
{
	double m_xVec,m_yVec;
public:
	CMazeAction(double xVec,double yVec);
	~CMazeAction();

	double getXVec() {return m_xVec;}
	double getYVec() {return m_yVec;}
};

#endif