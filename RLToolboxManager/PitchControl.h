#pragma once



#include "SetPoint.h"



class CPitchControl : public CContinuousTimeAndActionTransitionFunction,CSetPointGenerator
{
protected:
	double m_t;


	virtual void doSimulationStep(CState *state, double timestep, CAction *action, CActionData *data);
public:
	double m_accRewards;

	CPitchControl();
	virtual ~CPitchControl();

	void getCADerivationX(CState *oldState, CContinuousActionData *action, ColumnVector *derivationX);

	virtual bool isFailedState(CState *state);

	virtual void getResetState(CState *state);
};
