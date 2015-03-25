#include "stdafx.h"
#include "UnderwaterVehicle.h"
#include "Actions.h"


#define NUM_STATE_VARIABLES 3

#define DIM_UV_GOAL_V 0
#define DIM_UV_V 1
#define DIM_UV_V_CONTROL_DEVIATION 2


CUnderwaterVehicle::CUnderwaterVehicle() :CContinuousTimeAndActionTransitionFunction(new CStateProperties(NUM_STATE_VARIABLES, 0)
								,new CContinuousAction(new CContinuousActionProperties(1))
								, g_pParameters->getParameter("DELTA_T")),CSetPointGenerator("underwater-vehicle.txt")
{
	m_t= 0.0;

	properties->setMinValue(DIM_UV_GOAL_V,-5);
	properties->setMaxValue(DIM_UV_GOAL_V,5);

	properties->setVarName(DIM_UV_GOAL_V,"setpoint-v");
	properties->setMinValue(DIM_UV_V,-5);
	properties->setMaxValue(DIM_UV_V,5);

	properties->setVarName(DIM_UV_V,"v");
	properties->setMinValue(DIM_UV_V_CONTROL_DEVIATION,-10);
	properties->setMaxValue(DIM_UV_V_CONTROL_DEVIATION,10);

	properties->setVarName(DIM_UV_V_CONTROL_DEVIATION,"control-deviation");

	contAction->getContinuousActionProperties()->setMaxActionValue(0,30);
	contAction->getContinuousActionProperties()->setMinActionValue(0,-30);
}

CUnderwaterVehicle::~CUnderwaterVehicle()
{
	delete properties;
	delete actionProp;
	delete contAction;
}


bool CUnderwaterVehicle::isFailedState(CState *state)
{
	
	return false;
}



#define PI 3.1415926535897932384626433832795

void CUnderwaterVehicle::getResetState(CState *resetState)
{
	//CTransitionFunction::getResetState(resetState);
	
	m_t= 0.0;

	resetState->setContinuousState(DIM_UV_GOAL_V,getPointSet(m_t));
	resetState->setContinuousState(DIM_UV_V_CONTROL_DEVIATION, getPointSet(m_t)-0.0);
	resetState->setContinuousState(DIM_UV_V, 0.0);
}

void CUnderwaterVehicle::doSimulationStep(CState *state, double timestep, CAction *action, CActionData *data)
{
	double delta_t= g_pParameters->getParameter("DELTA_T");
	m_t+= delta_t;

	getDerivationX(state, action, derivation, data);

	double newSetpoint= getPointSet(m_t);
	double v= state->getContinuousState(DIM_UV_V);
	double newV= v + derivation->element(DIM_UV_V)*delta_t;

	state->setContinuousState(DIM_UV_GOAL_V,newSetpoint);
	state->setContinuousState(DIM_UV_V_CONTROL_DEVIATION,newSetpoint-newV);
	state->setContinuousState(DIM_UV_V,newV);
}

void CUnderwaterVehicle::getCADerivationX(CState *oldState, CContinuousActionData *action, ColumnVector *derivationX)
{
	assert(oldState->nrows() == derivationX->nrows());
	
	double v_dot;
	double u= action->getActionValue(0);
	double v= oldState->getContinuousState(DIM_UV_V);
	double control_dev= oldState->getContinuousState(DIM_UV_V_CONTROL_DEVIATION);

	v_dot= (u*(-0.5*tanh((fabs((1.2+0.2*sin(fabs(v)))*v*fabs(v) - u) -30.0)*0.1) + 0.5) 
		- (1.2+0.2*sin(fabs(v)))*v*fabs(v))
		/(3.0+1.5*sin(fabs(v)));
	derivationX->element(DIM_UV_GOAL_V)= (getPointSet(m_t)-getPointSet(m_t-0.001))/g_pParameters->getParameter("DELTA_T");;
	derivationX->element(DIM_UV_V)= v_dot;
	derivationX->element(DIM_UV_V_CONTROL_DEVIATION)= ((getPointSet(m_t)-v)-control_dev)/g_pParameters->getParameter("DELTA_T");
}
