#include "stdafx.h"
#include "RewardManager.h"
#include "StateManager.h"

#define MIN_REWARD -100.0
#define MAX_REWARD 1.0

CErrorComponent::CErrorComponent()
{
	m_errorComponentType[0]= 0;
	m_controlledVariable[0]= 0;
	m_setpointVariable[0]= 0;
	m_controlErrorVariable[0]= 0;
	m_weight= 0.0;
	m_componentIndex= -1;
	m_lastReward= 0.0;
}

CErrorComponent::~CErrorComponent()
{
}

void CErrorComponent::init(int componentIndex)
{
	char parameterName[256];

	m_componentIndex= componentIndex;

	sprintf_s(parameterName,256,"REWARD_COMPONENT_TYPE_%d",componentIndex);
	sprintf_s(m_errorComponentType,MAX_VAR_NAME_SIZE,"%s",g_pParameters->getStringParameter(parameterName).c_str());
	
	sprintf_s(parameterName,256,"REWARD_COMPONENT_WEIGHT_%d",componentIndex);
	m_weight= g_pParameters->getParameter(parameterName);

	sprintf_s(parameterName,256,"REWARD_COMPONENT_TOLERANCE_%d",componentIndex);
	m_tolerance= g_pParameters->getParameter(parameterName);

	if (strcmp(m_errorComponentType,"VARIABLE_DIFFERENCE")==0)
	{
		sprintf_s(parameterName,256,"CONTROLLED_VARIABLE_%d",componentIndex);
		sprintf_s(m_controlledVariable,MAX_VAR_NAME_SIZE,"%s",g_pParameters->getStringParameter(parameterName).c_str());
		sprintf_s(parameterName,256,"SETPOINT_VARIABLE_%d",componentIndex);
		sprintf_s(m_setpointVariable,MAX_VAR_NAME_SIZE,"%s",g_pParameters->getStringParameter(parameterName).c_str());
	}
	else if (strcmp(m_errorComponentType,"DEVIATION_VARIABLE")==0)
	{
		sprintf_s(parameterName,256,"CONTROL_ERROR_VARIABLE_%d",componentIndex);
		sprintf_s(m_controlErrorVariable,MAX_VAR_NAME_SIZE,"%s", g_pParameters->getStringParameter(parameterName).c_str());
	}
	else if ( (strcmp(m_errorComponentType,"CONSTANT_DIFFERENCE")==0)
			|| (strcmp(m_errorComponentType,"PUNISH_IF_ABOVE")==0)
			|| (strcmp(m_errorComponentType,"PUNISH_IF_BELOW")==0) )
	{
		sprintf_s(parameterName,256,"CONTROLLED_VARIABLE_%d",componentIndex);
		sprintf_s(m_controlledVariable,MAX_VAR_NAME_SIZE,"%s",g_pParameters->getStringParameter(parameterName).c_str());
		sprintf_s(parameterName,256,"SETPOINT_CONSTANT_%d",componentIndex);
		m_setpointConstant= g_pParameters->getParameter(parameterName);
	}
	else assert(0);
}

double CErrorComponent::getRewardComponent(CState* state)
{
	double rew,error;
	CStateProperties *pStateProperties= state->getStateProperties();

	if (strcmp(m_errorComponentType,"VARIABLE_DIFFERENCE")==0)
	{
		error= state->getContinuousState(pStateProperties->getVarIndexFromName(m_setpointVariable))
			- state->getContinuousState(pStateProperties->getVarIndexFromName(m_controlledVariable));
	}
	else if (strcmp(m_errorComponentType,"DEVIATION_VARIABLE")==0) 
	{
		error= state->getContinuousState(pStateProperties->getVarIndexFromName(m_controlErrorVariable));
	}
	else if (strcmp(m_errorComponentType,"CONSTANT_DIFFERENCE")==0)
	{
		error= m_setpointConstant 
			- state->getContinuousState(pStateProperties->getVarIndexFromName(m_controlledVariable));
	}
	else if (strcmp(m_errorComponentType,"PUNISH_IF_ABOVE")==0)
	{
		error= max(0.0,state->getContinuousState(pStateProperties->getVarIndexFromName(m_controlledVariable))
			-m_setpointConstant);
	}
	else if (strcmp(m_errorComponentType,"PUNISH_IF_BELOW")==0)
	{
		error= max(0.0,m_setpointConstant 
			- state->getContinuousState(pStateProperties->getVarIndexFromName(m_controlledVariable)));
	}

	error= (error)/m_tolerance;

	rew= MAX_REWARD- fabs(error);
	/*rew= tanh(fabs(error)/m_rewardComponentMu);///w);
	rew*= rew;

	rew= m_weight*(1.0-rew);*/

	rew= m_weight*rew;

	rew= max(MIN_REWARD,rew);

	m_lastReward= rew;

	return rew;
}


CEnvironmentReward::CEnvironmentReward(CContinuousTimeAndActionTransitionFunction *model)
	:CStateReward(model->getStateProperties())
{
	m_numStateVariables= model->getNumContinuousStates();// + model->getNumDiscreteStates();

	m_numRewardComponents= g_pParameters->getParameter("NUM_REWARD_COMPONENTS");

	for (int i= 0; i<m_numRewardComponents; i++)
	{
		m_errorComponents[i].init(i);
	}
}



double CEnvironmentReward::getStateReward(CState *state)
{
	double rew= 0.0;
	double stateValue,minValue,maxValue,varIndex;

	for (int i= 0; i<m_numRewardComponents; i++)
	{
		rew+= m_errorComponents[i].getRewardComponent(state);
	}

	//HAU BEHAR AL DET BENETAN???
	//char *pVariableList= g_pParameters->getStringParameter("CRITIC_VARIABLES");
	//for (int i= 0; i<state->getStateProperties()->getNumVariablesUsed(pVariableList); i++)
	//{
	//	varIndex= state->getStateProperties()->getVarIndexForLearning(i);
	//	stateValue= state->getContinuousState(varIndex);
	//	minValue= state->getStateProperties()->getMinValue(varIndex);
	//	maxValue= state->getStateProperties()->getMaxValue(varIndex);
	//	if (stateValue<=minValue || stateValue>=maxValue)
	//	{
	//		m_lastReward= -1000.0;
	//		return -1000.0; //saturated state variable punishment
	//	}
	//}

	m_lastReward= rew;
	return rew;
}

void CEnvironmentReward::getInputDerivation(CState *modelState, ColumnVector *targetState)
{
	for (int i=0; i<m_numStateVariables; i++)
		targetState->element(i)= 0.0;
}
