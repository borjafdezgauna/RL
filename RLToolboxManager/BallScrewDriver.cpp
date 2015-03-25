#include "stdafx.h"
#include "BallScrewDriver.h"
#include "Actions.h"


#define NUM_STATE_VARIABLES 3

double m_collisionForce= 0.0;



CBallScrewDriver::CBallScrewDriver() :CLinearActionContinuousTimeTransitionFunction(new CStateProperties(NUM_STATE_VARIABLES, 0)
								,new CContinuousAction(new CContinuousActionProperties(1))
								, g_pParameters->getParameter("DELTA_T")),CSetPointGenerator()
{
	m_x= 0.0; m_x_dot= 0.0;
	m_x_goal= 0.0;
	m_t= 0.0;
	m_accRewards= 0.0;

	m_goalChangeProbability= g_pParameters->getParameter("SET_POINT_CHANGE_PROB");

	properties->setMinValue(DIM_X,g_pParameters->getParameter("MIN_X"));
	properties->setMaxValue(DIM_X,g_pParameters->getParameter("MAX_X"));
	properties->setVarName(DIM_X,"x");
	properties->setMinValue(DIM_V,g_pParameters->getParameter("MIN_V"));
	properties->setMaxValue(DIM_V,g_pParameters->getParameter("MAX_V"));
	properties->setVarName(DIM_V,"v");
	properties->setMinValue(DIM_X_GOAL,g_pParameters->getParameter("MIN_GOAL"));
	properties->setMaxValue(DIM_X_GOAL,g_pParameters->getParameter("MAX_GOAL"));

	contAction->getContinuousActionProperties()->setMaxActionValue(0,g_pParameters->getParameter("MAX_ACTION_VALUE"));
	contAction->getContinuousActionProperties()->setMinActionValue(0,-g_pParameters->getParameter("MAX_ACTION_VALUE"));
}

CBallScrewDriver::~CBallScrewDriver()
{
	delete properties;
	delete actionProp;
	delete contAction;
}


bool CBallScrewDriver::isFailedState(CState *state)
{
	
	return false;
}



#define PI 3.1415926535897932384626433832795
#define INIT_FREQ 0.03
#define K 0.2
#define P 0.01 //0.01 m/rev
#define I 1 //factor de reducción=1
#define M 68.25
#define J_A 0.0001705 //170.5*10^-6 kg*m^2
#define J_H 0.000625 // 6.25*10^-4 kg*m^2
#define J_M 0.0085 // 85*10^-4 kg*m^2
#define J_R 0 //0 kg*m^2
#define PI2zatiP (2*PI/P)

Matrix *CBallScrewDriver::getB(CState *state)
{

	B->element(DIM_V, 0) =g_pParameters->getParameter("REAL_MAX_ACTION_VALUE") 
		/ ( (M*P/(2*PI) + (J_A+J_H+J_M)*PI2zatiP) );
		//REAL_MAX_ACTION_VALUE*(PI2zatiP)/((M+J_A+J_H)*PI2zatiP*PI2zatiP + (J_M+J_R)*PI2zatiP*I);
	B->element(DIM_X, 0) = state->getContinuousState(DIM_V);
	B->element(DIM_X_GOAL, 0) = 0.0;



	return B;
}

ColumnVector *CBallScrewDriver::getA(CState *state)
{
	A->element(DIM_X) = 0.0;
	A->element(DIM_V) = 0.0;
	A->element(DIM_X_GOAL) = 0.0;//state->getContinuousState(g_pParameters->getParameter("SETPOINT_VARIABLE"));


	return A;
}

void CBallScrewDriver::getResetState(CState *resetState)
{
	//CTransitionFunction::getResetState(resetState);
	
	m_t= 0.0;
	
	if (g_pParameters->getParameter("USE_FIXED_POINT_SET")!=0.0)
	{
		m_x= 0.5;
		m_x_goal= getPointSet(m_t);
	}
	else
	{
		m_x_goal= g_pParameters->getParameter("MIN_GOAL")
			+ (g_pParameters->getParameter("MAX_GOAL")-g_pParameters->getParameter("MIN_GOAL"))
			*((double)(rand()%1001)/1001.0);
		m_x= g_pParameters->getParameter("MIN_X")
			+ (g_pParameters->getParameter("MAX_X")-g_pParameters->getParameter("MIN_X"))
			*((double)(rand()%1001)/1001.0);
	}

	resetState->setContinuousState(DIM_X_GOAL, m_x_goal);
	resetState->setContinuousState(DIM_X, m_x);
	resetState->setContinuousState(DIM_V, 0.0);//m_x_dot);
}

void CBallScrewDriver::doSimulationStep(CState *state, double timestep, CAction *action, CActionData *data)
{
	m_t+= g_pParameters->getParameter("DELTA_T");

	getDerivationX(state, action, derivation, data);

	//double ddPhi = derivation->element(1);
	double v= state->getContinuousState(DIM_V);
	double x= state->getContinuousState(DIM_X);
	double der= derivation->element(0);

	double noiseSigma= g_pParameters->getParameter("NOISE_SIGMA");

	//new v
	double newV= m_x_dot + timestep * (derivation->element(DIM_V));
	//new x
	//noise= CDistributions::getNormalDistributionSample(0.0, noiseSigma);
	double newX= m_x + timestep * (newV);//state->getContinuousState(DIM_V);

	if (newX<g_pParameters->getParameter("MIN_X") || newX>g_pParameters->getParameter("MAX_X"))
	{
		if (newX<g_pParameters->getParameter("MIN_X"))
			m_collisionForce= g_pParameters->getParameter("MIN_X")-newX;
		else
			m_collisionForce= newX-g_pParameters->getParameter("MAX_X");
		newV= 0;
	}
	else m_collisionForce= 0.0;

	
	newV= max(g_pParameters->getParameter("MIN_V"),min(g_pParameters->getParameter("MAX_V"),newV/* + noise*/));
		double noise= CDistributions::getNormalDistributionSample(0.0, noiseSigma);
	state->setContinuousState(DIM_V, newV+noise);

	noise= 0.0;//CDistributions::getNormalDistributionSample(0.0, noiseSigma);
	newX= max(g_pParameters->getParameter("MIN_X"),min(g_pParameters->getParameter("MAX_X"),newX/* + noise*/));
	state->setContinuousState(DIM_X, newX+noise);

	m_x_dot= newV;// state->getContinuousState(DIM_V);
	m_x= newX;// state->getContinuousState(DIM_X);

	if (g_pParameters->getParameter("USE_FIXED_POINT_SET")!=0.0)
	{
		m_x_goal= getPointSet(m_t);
	}

	state->setContinuousState(DIM_X_GOAL,m_x_goal);
}

