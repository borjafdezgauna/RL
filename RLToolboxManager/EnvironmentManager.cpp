#include "stdafx.h"
#include "EnvironmentManager.h"

#include "WindTurbine.h"
#include "2MassWindTurbine.h"
#include "BallScrewDriver.h"
#include "UnderwaterVehicle.h"
#include "PitchControl.h"

class CContinuousTimeAndActionTransitionFunction;

CEnvironmentManager::CEnvironmentManager()
{
	m_pModel= 0;
}

CEnvironmentManager::~CEnvironmentManager()
{
	if (m_pModel) delete m_pModel;
}

int CEnvironmentManager::getNumEnvironments()
{
	return 2;
}

CContinuousTimeAndActionTransitionFunction* CEnvironmentManager::getModel(string model)
{
	if (!m_pModel)
	{
		if (model=="BALLSCREW_DRIVER")
			m_pModel= new CBallScrewDriver();
		else if (model=="UNDERWATER_VEHICLE")
			m_pModel= new CUnderwaterVehicle();
		else if (model=="PITCH_CONTROL")
			m_pModel= new CPitchControl();
		else if (model=="WIND_TURBINE")
			m_pModel= new CWindTurbine();
		else if (model=="TWO_MASS_WIND_TURBINE")
			m_pModel= new C2MassWindTurbine();
		else
		{
			m_pModel= 0;
			OutputDebugString(_T("non-valid model id\n"));
		}
	}
	return m_pModel;
}
