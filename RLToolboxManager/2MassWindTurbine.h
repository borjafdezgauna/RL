#ifndef __2MASS_WIND_TURBINE__
#define __2MASS_WIND_TURBINE__

class CSetPointGenerator;



class C2MassWindTurbine : public CContinuousTimeAndActionTransitionFunction
{
	double m_t;
	CSetPointGenerator *m_pWindData;
	CSetPointGenerator *m_pSetpointPower;

	virtual void doSimulationStep(CState *state, double timestep, CAction *action, CActionData *data);
public:

	C2MassWindTurbine();
	virtual ~C2MassWindTurbine();


	virtual bool isFailedState(CState *state);
	virtual void getResetState(CState *state);

	void getCADerivationX(CState *oldState, CContinuousActionData *action, ColumnVector *derivationX);

};

#endif