#pragma once



#include "SetPoint.h"



class CUnderwaterVehicle : public CContinuousTimeAndActionTransitionFunction,CSetPointGenerator
{
protected:
	double m_t;


	virtual void doSimulationStep(CState *state, double timestep, CAction *action, CActionData *data);
public:
	double m_accRewards;

	CUnderwaterVehicle();
	virtual ~CUnderwaterVehicle();

	void getCADerivationX(CState *oldState, CContinuousActionData *action, ColumnVector *derivationX);

	virtual bool isFailedState(CState *state);

	virtual void getResetState(CState *state);
};
