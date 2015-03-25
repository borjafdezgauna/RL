#include "stdafx.h"
#include "PIController.h"

#define K_P_X_PARAM_NAME "KP_X"
#define K_I_X_PARAM_NAME "KI_X"
#define K_D_X_PARAM_NAME "KD_X"
#define K_P_V_PARAM_NAME "KP_V"
#define K_I_V_PARAM_NAME "KI_V"
#define K_D_V_PARAM_NAME "KD_V"
#define CONTROL_ERROR_VARIABLE_NAME "CONTROL_ERROR_VARIABLE"
#define SETPOINT_VARIABLE_NAME "SETPOINT_VARIABLE"
#define CONTROLLED_VARIABLE_NAME "CONTROLLED_VARIABLE"



CSingleLoopPIDController::CSingleLoopPIDController(CContinuousAction *contAction, CActionSet *pStaticCActionSet, int randomControllerMode):CContinuousActionController(contAction,randomControllerMode)
{
	addParameter(K_P_X_PARAM_NAME,g_pParameters->getParameter("KP_X"));
	addParameter(K_I_X_PARAM_NAME,g_pParameters->getParameter("KI_X"));
	addParameter(K_D_X_PARAM_NAME,g_pParameters->getParameter("KD_X"));
	addParameter(CONTROL_ERROR_VARIABLE_NAME,(int)g_pParameters->getParameter("CONTROL_ERROR_VARIABLE"));

	m_lastError_v= 0.0;
	m_accError_v= 0.0;
	m_lastError_x= 0.0;
	m_accError_x= 0.0;

	m_pStaticCActionSet= pStaticCActionSet;
	m_contAction= contAction;
};

CSingleLoopPIDController::~CSingleLoopPIDController()
{
};

void CSingleLoopPIDController::getNextContinuousAction(CStateCollection *stateCol, CContinuousActionData *action)
{
	CState *state = stateCol->getState();
	
	//double x= state->getContinuousState(getParameter(MEASURED_VARIABLE_NAME));
	//double goal= state->getContinuousState(getParameter(SETPOINT_VARIABLE_NAME));
	double error= state->getContinuousState(getParameter(CONTROL_ERROR_VARIABLE_NAME));

	
	double v,x_dot_goal,innerLoopError,actionValue;
	double delta_t= g_pParameters->getParameter("DELTA_T");

	//m_lastX= x;
	m_accError_x+= m_lastError_x*delta_t;
	
	actionValue= /*(goal-x)*/error*getParameter(K_P_X_PARAM_NAME) 
		+ m_accError_x*getParameter(K_I_X_PARAM_NAME) 
		+ (m_lastError_x/delta_t)*getParameter(K_D_X_PARAM_NAME);
	//actionValue= max(action->pr,min(g_pParameters->getParameter("MAX_ACTION_VALUE"),actionValue));

	m_lastError_x= error;//(goal-x);
	//m_lastX= x;

	actionValue= min(m_contAction->getContinuousActionProperties()->getMaxActionValue(0),
		max(m_contAction->getContinuousActionProperties()->getMinActionValue(0),actionValue));
	action->setActionValue(0,actionValue);//*g_pParameters->getParameter("REAL_MAX_ACTION_VALUE"));
}



//Double-loop PID controller


CDoubleLoopPIDController::CDoubleLoopPIDController(CContinuousAction *contAction, CActionSet *pStaticCActionSet, int randomControllerMode):CContinuousActionController(contAction,randomControllerMode)
{
	addParameter(K_P_X_PARAM_NAME,g_pParameters->getParameter("KP_X"));
	addParameter(K_I_X_PARAM_NAME,g_pParameters->getParameter("KI_X"));
	addParameter(K_D_X_PARAM_NAME,g_pParameters->getParameter("KD_X"));
	addParameter(K_P_V_PARAM_NAME,g_pParameters->getParameter("KP_V"));
	addParameter(K_I_V_PARAM_NAME,g_pParameters->getParameter("KI_V"));
	addParameter(K_D_V_PARAM_NAME,g_pParameters->getParameter("KD_V"));
	addParameter(CONTROLLED_VARIABLE_NAME,(int)g_pParameters->getParameter("CONTROLLED_VARIABLE"));
	addParameter(SETPOINT_VARIABLE_NAME,(int)g_pParameters->getParameter("SETPOINT_VARIABLE"));

	m_lastError_v= 0.0;
	m_accError_v= 0.0;
	m_lastError_x= 0.0;
	m_accError_x= 0.0;

	m_pStaticCActionSet= pStaticCActionSet;
	m_contAction= contAction;
};

CDoubleLoopPIDController::~CDoubleLoopPIDController()
{
};

void CDoubleLoopPIDController::getNextContinuousAction(CStateCollection *stateCol, CContinuousActionData *action)
{
	CState *state = stateCol->getState();

	double x= state->getContinuousState(getParameter(CONTROLLED_VARIABLE_NAME));
	double goal= state->getContinuousState(getParameter(SETPOINT_VARIABLE_NAME));
	double error;

	
	double v,x_dot_goal,innerLoopError,actionValue;
	double delta_t= g_pParameters->getParameter("DELTA_T");

	m_lastX= x;
	m_accError_x+= m_lastError_x*delta_t;
	
	if (state->isResetState()) v=0.0;
	else v= (x-m_lastX)/delta_t;

	x_dot_goal= (goal-x)*getParameter(K_P_X_PARAM_NAME) 
		+ m_accError_x*getParameter(K_I_X_PARAM_NAME) 
		+ (m_lastError_x/delta_t)*getParameter(K_D_X_PARAM_NAME);
	x_dot_goal= max(-2.0,min(2.0,x_dot_goal));
	
	innerLoopError= (x_dot_goal-v);
	actionValue= innerLoopError*getParameter(K_P_V_PARAM_NAME) 
		+ m_accError_v*getParameter(K_I_V_PARAM_NAME) 
		+ (m_lastError_v/delta_t)*getParameter(K_D_V_PARAM_NAME);
	m_lastError_v= innerLoopError;

	m_accError_v+= m_lastError_v*delta_t;

	m_lastError_x= (goal-x);
	m_lastX= x;

	actionValue= min(m_contAction->getContinuousActionProperties()->getMaxActionValue(0),
		max(m_contAction->getContinuousActionProperties()->getMinActionValue(0),actionValue));
	action->setActionValue(0,actionValue);//*g_pParameters->getParameter("REAL_MAX_ACTION_VALUE"));
}
