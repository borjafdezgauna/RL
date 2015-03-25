#ifndef __WIND_TURBINE__
#define __WIND_TURBINE__

class CSetPointGenerator;

class CWindTurbine : public CContinuousTimeAndActionTransitionFunction
{
	double m_t;
	CSetPointGenerator *m_pWindData;
	CSetPointGenerator *m_pSetpointPower;

	double initial_torque;
	double initial_blade_angle;

	virtual void doSimulationStep(CState *state, double timestep, CAction *action, CActionData *data);
public:

	CWindTurbine();
	virtual ~CWindTurbine();


	virtual bool isFailedState(CState *state);
	virtual void getResetState(CState *state);

	void getCADerivationX(CState *oldState, CContinuousActionData *action, ColumnVector *derivationX);

};

class CContinuousAction;
class CContinuousActionController;
class CActionSet;


//"Multivariable control strategy for variable speed, variable pitch wind turbines"
//B. Boukhezzar, L. Lupu, H. Siguerdidjane, M. Hand
class CWindTurbineControllerBoukhezzar : public CContinuousActionController
{

	CActionSet *m_pStaticCActionSet;
	CContinuousAction *m_contAction;
public:
	CWindTurbineControllerBoukhezzar(CContinuousAction *contAction, CActionSet *pStaticCActionSet,int randomControllerMode=1);
	~CWindTurbineControllerBoukhezzar();

	virtual void getNextContinuousAction(CStateCollection *state, CContinuousActionData *action);
};

//"Power Control Design for Variable-Speed Wind Turbines"
//Yolanda Vidal, Leonardo Acho, Ningsu Luo, Mauricio Zapateiro and Francesc Pozo
class CWindTurbineControllerVidal : public CContinuousActionController
{
	double m_integrative_omega_r_error;

	CActionSet *m_pStaticCActionSet;
	CContinuousAction *m_contAction;
public:
	CWindTurbineControllerVidal(CContinuousAction *contAction, CActionSet *pStaticCActionSet,int randomControllerMode=1);
	~CWindTurbineControllerVidal();

	virtual void getNextContinuousAction(CStateCollection *state, CContinuousActionData *action);
};

#endif