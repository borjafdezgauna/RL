#pragma once


#define DIM_X 1
#define DIM_V 0
#define DIM_X_GOAL 2


#define GOAL_SPEED 1.0

#include "SetPoint.h"



class CBallScrewDriver : public CLinearActionContinuousTimeTransitionFunction,CSetPointGenerator
{
protected:
	double m_x_dot;
	double m_x, m_x_goal;
	double m_t;
	double m_goalChangeProbability;


	virtual void doSimulationStep(CState *state, double timestep, CAction *action, CActionData *data);
public:
	double m_accRewards;

	CBallScrewDriver();
	virtual ~CBallScrewDriver();

	virtual Matrix *getB(CState *state);
	virtual ColumnVector *getA(CState *state);


	virtual bool isFailedState(CState *state);

	virtual void getResetState(CState *state);

//	double getTime(){return m_t;}
};
