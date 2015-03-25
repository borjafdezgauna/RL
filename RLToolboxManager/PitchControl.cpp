#include "stdafx.h"
#include "PitchControl.h"
#include "Actions.h"


#define NUM_STATE_VARIABLES 5

#define DIM_PC_PITCH_GOAL 0
#define DIM_PC_PITCH_ANGLE 1
#define DIM_PC_ANGLE_OF_ATTACK 2
#define DIM_PC_PITCH_RATE 3
#define DIM_PC_PITCH_DEVIATION 4


CPitchControl::CPitchControl() :CContinuousTimeAndActionTransitionFunction(new CStateProperties(NUM_STATE_VARIABLES, 0)
								,new CContinuousAction(new CContinuousActionProperties(1))
								, g_pParameters->getParameter("DELTA_T"))
								,CSetPointGenerator("pitch-control.txt")
{
	m_t= 0.0;

	properties->setMinValue(DIM_PC_PITCH_GOAL,-3.141516);
	properties->setMaxValue(DIM_PC_PITCH_GOAL,3.141516);
//	properties->setPeriodicity(DIM_PC_PITCH_GOAL,true);

	properties->setVarName(DIM_PC_PITCH_GOAL,"setpoint-pitch");
	
	properties->setMinValue(DIM_PC_PITCH_ANGLE,-3.141516);
	properties->setMaxValue(DIM_PC_PITCH_ANGLE,3.141516);
//	properties->setPeriodicity(DIM_PC_PITCH_ANGLE,true);

	properties->setVarName(DIM_PC_PITCH_ANGLE,"pitch");

	properties->setMinValue(DIM_PC_ANGLE_OF_ATTACK,-3.141516);
	properties->setMaxValue(DIM_PC_ANGLE_OF_ATTACK,3.141516);

//	properties->setPeriodicity(DIM_PC_ANGLE_OF_ATTACK,true);
	properties->setVarName(DIM_PC_PITCH_ANGLE,"attack-angle");

	properties->setMinValue(DIM_PC_PITCH_RATE,-0.1);
	properties->setMaxValue(DIM_PC_PITCH_RATE,0.1);

	properties->setVarName(DIM_PC_PITCH_RATE,"pitch-rate");
	
	properties->setMinValue(DIM_PC_PITCH_DEVIATION,-6.5);
	properties->setMaxValue(DIM_PC_PITCH_DEVIATION,6.5);

	properties->setVarName(DIM_PC_PITCH_DEVIATION,"control-deviation");

	contAction->getContinuousActionProperties()->setMaxActionValue(0,1.4);
	contAction->getContinuousActionProperties()->setMinActionValue(0,-1.4);
}

CPitchControl::~CPitchControl()
{
	delete properties;
	delete actionProp;
	delete contAction;
}


bool CPitchControl::isFailedState(CState *state)
{
	
	return false;
}



#define PI 3.1415926535897932384626433832795

void CPitchControl::getResetState(CState *resetState)
{
	//CTransitionFunction::getResetState(resetState);
	
	m_t= 0.0;

	resetState->setContinuousState(DIM_PC_PITCH_GOAL, getPointSet(m_t));
	resetState->setContinuousState(DIM_PC_ANGLE_OF_ATTACK, 0.0);
	resetState->setContinuousState(DIM_PC_PITCH_RATE, 0.0);
	resetState->setContinuousState(DIM_PC_PITCH_ANGLE, 0.0);
	resetState->setContinuousState(DIM_PC_PITCH_DEVIATION, getPointSet(m_t)-0.0);

//	#define DIM_PC_PITCH_GOAL 0
//#define DIM_PC_PITCH_ANGLE 1
//#define DIM_PC_ANGLE_OF_ATTACK 2
//#define DIM_PC_PITCH_RATE 3
//#define DIM_PC_PITCH_DEVIATION 4
}

void CPitchControl::doSimulationStep(CState *state, double timestep, CAction *action, CActionData *data)
{
	double s;
	double delta_t= g_pParameters->getParameter("DELTA_T");
	m_t+= delta_t;

	getDerivationX(state, action, derivation, data);

	state->setContinuousState(DIM_PC_ANGLE_OF_ATTACK, state->getContinuousState(DIM_PC_ANGLE_OF_ATTACK)
		+derivation->element(DIM_PC_ANGLE_OF_ATTACK)*delta_t);
	state->setContinuousState(DIM_PC_PITCH_RATE, state->getContinuousState(DIM_PC_PITCH_RATE)
		+derivation->element(DIM_PC_PITCH_RATE)*delta_t);
	state->setContinuousState(DIM_PC_PITCH_ANGLE, state->getContinuousState(DIM_PC_PITCH_ANGLE)
		+derivation->element(DIM_PC_PITCH_ANGLE)*delta_t);

	state->setContinuousState(DIM_PC_PITCH_GOAL,getPointSet(m_t));
	state->setContinuousState(DIM_PC_PITCH_DEVIATION, state->getContinuousState(DIM_PC_PITCH_GOAL)
		-state->getContinuousState(DIM_PC_PITCH_ANGLE));

}

void CPitchControl::getCADerivationX(CState *oldState, CContinuousActionData *action, ColumnVector *derivationX)
{
	assert(oldState->nrows() == derivationX->nrows());
	
	double delta_t= g_pParameters->getParameter("DELTA_T");
	double u= action->getActionValue(0);
	double angle_attack= oldState->getContinuousState(DIM_PC_ANGLE_OF_ATTACK);
	double pitch_rate= oldState->getContinuousState(DIM_PC_PITCH_RATE);
	double pitch_angle= oldState->getContinuousState(DIM_PC_PITCH_ANGLE);
	double pitch_deviation= oldState->getContinuousState(DIM_PC_PITCH_DEVIATION);
	double goal_pitch= oldState->getContinuousState(DIM_PC_PITCH_GOAL);

	double angle_attack_dot= -0.313*angle_attack + 56.7*pitch_rate + 0.232*u;
	double pitch_rate_dot= -0.0139*angle_attack -0.426*pitch_rate + 0.0203*u;
	double pitch_angle_dot= 56.7*pitch_rate;
	double pointset= oldState->getContinuousState(DIM_PC_PITCH_GOAL);

	derivationX->element(DIM_PC_PITCH_GOAL) = (goal_pitch-pointset)/delta_t;
	derivationX->element(DIM_PC_ANGLE_OF_ATTACK)= angle_attack_dot;
	derivationX->element(DIM_PC_PITCH_RATE)= pitch_rate_dot;
	derivationX->element(DIM_PC_PITCH_ANGLE) = pitch_angle_dot;
	derivationX->element(DIM_PC_PITCH_DEVIATION)= ((goal_pitch-pitch_angle+pitch_angle_dot*delta_t)
		-pitch_deviation)/delta_t;
}
//#define DIM_PC_PITCH_GOAL 0
//#define DIM_PC_ANGLE_OF_ATTACK 1
//#define DIM_PC_PITCH_RATE 2
//#define DIM_PC_PITCH_ANGLE 3