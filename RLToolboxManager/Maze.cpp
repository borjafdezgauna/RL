#include "stdafx.h"
#include "Maze.h"

#define MAX_X 10.
#define MAX_Y 10.

CMaze::CMaze():CEnvironmentModel (2,0)
//continuous variables, discrete variables
{
	m_x= m_y= 0.5;

	properties->setMinValue(0,0);
	properties->setMaxValue(0,MAX_X);
	properties->setMinValue(1,0);
	properties->setMaxValue(1,MAX_Y);

}

CMaze::~CMaze()
{
}

double CMaze::getSizeX()
{
	return MAX_X;
}
double CMaze::getSizeY()
{
	return MAX_Y;
}

void CMaze::doResetModel()
{
	m_x= m_y= 0.5;
}

void CMaze::getState(CState *state)
{
	CEnvironmentModel::getState(state);

	state->setContinuousState(0,m_x);
	state->setContinuousState(1,m_y);
}

void CMaze::doNextState(CPrimitiveAction *act)
{
	double xVec= ((CMazeAction*)act)->getXVec();
	double yVec= ((CMazeAction*)act)->getYVec();

	double newX= max(0,min(m_x+xVec,MAX_X));
	double newY= max(0,min(m_y+yVec,MAX_Y));

	m_x= newX;
	m_y= newY;

	if (fabs(m_x-MAX_X+1)<0.6 && fabs(m_y-MAX_Y+1)<0.6)
		reset=true;

	if (reset)
	{
		printf("Goal reached");
	}
}

double CMaze::getReward(CStateCollection *oldStateCol,CAction *action, CStateCollection *newStateCol)
{
	double rew= 0.0;
	CState *newState= newStateCol->getState(getStateProperties());

//	if ( ((CMazeAction*)newState)->

	if (this->isReset())
	{
		rew= 100.0;
	}
	else rew= 0.0;
	return rew;
}

CMazeAction::CMazeAction(double xVec,double yVec):CPrimitiveAction()
{
	m_xVec= xVec;
	m_yVec= yVec;
}

CMazeAction::~CMazeAction(){};