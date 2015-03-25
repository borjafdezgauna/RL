#include "stdafx.h"
#include "2MassWindTurbine.h"
#include "SetPoint.h"

//[1]
//"Torque and pitch angle control for VSWT in all operating regimes"
//Adel Merabet, Jogendra Thongam, Jason Gu

//[2]
//"Pitch and torque control strategy for VSWT"
//L. Lupu, B. Boukhezzar, H. Siguerdidjane

//[3]
//"Nonlinear Control of a Variable-Speed Wind Turbine Using a Two-Mass Model"
//Boubekeur Boukhezzar and Houria Siguerdidjane

#define DIM_P_set 0 //P_setpoint
#define DIM_P_a 1 //P_a
#define DIM_v 2 //v_w
#define DIM_T_a 3 //T_a
#define DIM_omega_t 4 //omega_t
#define DIM_d_omega_t 5 //d_omega_t
#define DIM_beta 6 //beta
#define DIM_T_g 7 //T_g
#define DIM_T_ls 8 //T_ls
#define DIM_omega_g 9//omega_g

#define NUM_STATE_VARS 10

#define MAX_BLADE_ANGLE 25 //20 degrees [4]
#define MAX_ROTOR_SPEED 9.0 //rad/s  aprox. 30 rpm [4]
#define MAX_POWER 800 //kW [4]
#define MAX_AERODYNAMIC_TORQUE 500.0

#define NOMINAL_ROTOR_SPEED 4.29350995//rad/s - 41 rpm[4]

//////////////////////////////////////////////////////////
// how the fuck do i calculate the initial values?????
#define INITIAL_ROTOR_SPEED NOMINAL_ROTOR_SPEED //rad/s nominal-speed [6]
#define INITIAL_ACTION_TORQUE -36.279653322202989 //[4]
//#define INITIAL_BLADE_PITCH 7.5

#define P_nom 650.0 //kW [4]
#define HUB_HEIGHT 36.6 //m [4]
#define N_g 43.165 //[4]
#define D 43.3 //m [4]
#define J_r 3.25e5 //kg*m^2 [3]
#define K_r 27.36 //N*m/rad/s [3]
#define rho 1.25 //kg/m^3 [3]
#define J_g 34.4 //kg*m^2 [3]
#define K_g 0.2 //N*m/rad/s [3]
#define K_ls 9500//N*m/rad/s [3]
#define B_ls 2.691e5 //N*m/rad

#define DIM_A_beta 0 //beta
#define DIM_A_T_em 1 //T_g

#define NUM_ACTION_VARS 2

#define MAX_ACTION_TORQUE 162 //kN*m [2]
#define MAX_ACTION_BLADE_RATE 10//((10.0*2*3.14159265)/360.0) //rad/s +/- 10º/s

#define MAX_BETA 24//(24*2*3.14159265/360.0)
#define MAX_WIND_SPEED 40.0
#define MAX_TSR 15.0
#define NUM_BETA_SAMPLES 30000
#define NUM_WIND_SPEED_SAMPLES 1000
#define NUM_TSR_SAMPLES 100

#define CONSTANT_WIND_SPEED 10.0


double C_p_2mass(double lambda, double beta) //[1]
{
	double lambda_i,c_p;
	//beta= beta*2*3.14159265/360.0;

	lambda_i= 1.0/((1.0/(lambda+0.08*beta)) - 0.035/(beta*beta*beta+1.0));
	if (lambda_i==0.0) return 0.0;

	c_p= 0.22*(116.0/lambda_i - 0.4*beta -5.0)*exp(-12.5/lambda_i);
	return c_p;
}
double C_q_2mass(double lambda, double beta)
{
	if (lambda==0.0) return 0.0;

	return C_p_2mass(lambda,beta)/lambda;
}
double AerodynamicTorque_2mass(double tip_speed_ratio,double beta, double wind_speed)
{
	double cq= C_q_2mass(tip_speed_ratio,beta);

	//Pa= 0.5 * rho * pi * R^2 * C_p_2mass(lambda,beta) * v^3
	double torque= 0.5*rho*3.14159265*pow(D*0.5,3.0)*cq*wind_speed*wind_speed;
	return torque;
}

double AerodynamicPower_2mass(double tip_speed_ratio,double beta, double wind_speed)
{
	double cp= C_p_2mass(tip_speed_ratio,beta);

	//Pa= 0.5 * rho * pi * R^2 * C_p_2mass(lambda,beta) * v^3
	double power= 0.5*rho*3.14159265*(D*0.5)*(D*0.5)*cp*pow(wind_speed,3.0);
	return power;
}
double AerodynamicPower_2mass(double cp, double wind_speed)
{
	//Pa= 0.5 * rho * pi * R^2 * C_p_2mass(lambda,beta) * v^3
	double power= 0.5*rho*3.14159265*(D*0.5)*(D*0.5)*cp*pow(wind_speed,3.0);
	return power;
}

double sgn_2mass(double value)
{
	if (value<0.0) return -1.0;
	else return 1.0;
}

void FindSuitableParameters_2mass(double initial_wind_speed,double initial_power,double initial_rotor_speed
							,double &initial_blade_angle)
{
	double c_p,best_c_p= -1000.0;
	double beta, tsr;
	double power, best_power= 0.0;
	double desired_c_p= (initial_power*2)/(rho*3.14159265*(D*0.5)*(D*0.5)*pow(initial_wind_speed,3.0));
	double check= AerodynamicPower_2mass(desired_c_p,initial_wind_speed);

	initial_blade_angle= 0.0;

	for (int j= 0; j<NUM_BETA_SAMPLES; j++)
	{
		beta= (double)j * (MAX_BETA/(double)NUM_BETA_SAMPLES);
		tsr= initial_rotor_speed*D*0.5/initial_wind_speed;

		c_p= C_p_2mass(tsr,beta);

		if (fabs(desired_c_p-c_p)<fabs(desired_c_p-best_c_p))
		{
			initial_blade_angle= beta;
			initial_rotor_speed= initial_wind_speed*tsr/(D*0.5);
			best_c_p= c_p;
		}
	}
}


C2MassWindTurbine::C2MassWindTurbine():CContinuousTimeAndActionTransitionFunction(new CStateProperties(NUM_STATE_VARS, 0)
								,new CContinuousAction(new CContinuousActionProperties(NUM_ACTION_VARS))
								, g_pParameters->getParameter("DELTA_T"))
{
	properties->setMinValue(DIM_P_set,0.0);
	properties->setMaxValue(DIM_P_set,MAX_POWER);

	properties->setVarName(DIM_P_set,"Power setpoint");
	properties->setMinValue(DIM_P_a,0.0);
	properties->setMaxValue(DIM_P_a,MAX_POWER);

	properties->setVarName(DIM_P_a,"Generated power");
	properties->setMinValue(DIM_v,0.0);
	properties->setMaxValue(DIM_v,60.0);

	properties->setVarName(DIM_v,"Wind speed");
	properties->setMinValue(DIM_T_a,-MAX_AERODYNAMIC_TORQUE);
	properties->setMaxValue(DIM_T_a,MAX_AERODYNAMIC_TORQUE);

	properties->setVarName(DIM_T_a,"Aerodynamic torque");
	properties->setMinValue(DIM_omega_t,0.0);
	properties->setMaxValue(DIM_omega_t,MAX_ROTOR_SPEED);

	properties->setVarName(DIM_omega_t,"Rotor speed");
	properties->setMinValue(DIM_d_omega_t,-1.0);
	properties->setMaxValue(DIM_d_omega_t,1.0);

	properties->setVarName(DIM_d_omega_t,"Rotor acceleration");
	properties->setMinValue(DIM_beta,0.0);
	properties->setMaxValue(DIM_beta,MAX_BLADE_ANGLE);

	properties->setVarName(DIM_beta,"Blade angle");
	properties->setMinValue(DIM_T_g,-MAX_ACTION_TORQUE);
	properties->setMaxValue(DIM_T_g,MAX_ACTION_TORQUE);

	properties->setVarName(DIM_T_g,"Generator torque");
	properties->setMinValue(DIM_omega_g,-MAX_ROTOR_SPEED);
	properties->setMaxValue(DIM_omega_g,MAX_ROTOR_SPEED);

	properties->setVarName(DIM_omega_g,"Generator speed");

	contAction->getContinuousActionProperties()->setMaxActionValue(DIM_A_beta,MAX_ACTION_BLADE_RATE);
	contAction->getContinuousActionProperties()->setMinActionValue(DIM_A_beta,-MAX_ACTION_BLADE_RATE);
	contAction->getContinuousActionProperties()->setMaxActionValue(DIM_A_T_em,MAX_ACTION_TORQUE);
	contAction->getContinuousActionProperties()->setMinActionValue(DIM_A_T_em,-MAX_ACTION_TORQUE);

	m_pWindData= new CSetPointGenerator("WTGenerator-wind.txt");
	m_pSetpointPower= new CSetPointGenerator("WTGenerator-power.txt");

}

C2MassWindTurbine::~C2MassWindTurbine()
{
	delete m_pWindData;
	delete m_pSetpointPower;
}

bool C2MassWindTurbine::isFailedState(CState *state)
{
	
	return false;
}

void C2MassWindTurbine::getResetState(CState *state)
{
	m_t= 0.0;
//#define DIM_P_set 0 //P_setpoint
//#define DIM_P_a 1 //P_a
//#define DIM_v 2 //v_w
//#define DIM_T_a 3 //T_a
//#define DIM_omega_t 4 //omega_t
//#define DIM_beta 5 //beta
	double initial_wind_speed= CONSTANT_WIND_SPEED;//m_pWindData->getPointSet(m_t);
	double initial_power= P_nom;//= AerodynamicPower_2mass(initial_tsr,initial_blade_angle,initial_wind_speed);
	double initial_rotor_speed= NOMINAL_ROTOR_SPEED;//= INITIAL_ROTOR_SPEED*D*0.5/initial_wind_speed;
	double initial_blade_angle;
	
	FindSuitableParameters_2mass(initial_wind_speed,initial_power,initial_rotor_speed,initial_blade_angle);
	
	
	state->setContinuousState(DIM_P_set,m_pSetpointPower->getPointSet(m_t));
	initial_power= AerodynamicPower_2mass(initial_rotor_speed*D*0.5/initial_wind_speed,initial_blade_angle,initial_wind_speed);
	state->setContinuousState(DIM_P_a,initial_power);
	state->setContinuousState(DIM_v,initial_wind_speed);
	state->setContinuousState(DIM_T_a,initial_power/initial_rotor_speed);
	state->setContinuousState(DIM_omega_t,initial_rotor_speed);
	state->setContinuousState(DIM_d_omega_t,0.0);
	state->setContinuousState(DIM_beta,initial_blade_angle);
	state->setContinuousState(DIM_T_g,INITIAL_ACTION_TORQUE);
}



// x' = B(x) * u + a(x)



void C2MassWindTurbine::doSimulationStep(CState *state, double timestep, CAction *action, CActionData *data)
{
	m_t+= timestep;

	CContinuousActionData* pActionData= (CContinuousActionData*)action->getActionData();

	//getDerivationX(state, action, derivation, data);

	state->setContinuousState(DIM_P_set,m_pSetpointPower->getPointSet(m_t));
	state->setContinuousState(DIM_v,m_pWindData->getPointSet(m_t));

	//beta= beta + d(beta)/dt
	state->setContinuousState(DIM_beta, state->getContinuousState(DIM_beta)
		+timestep*pActionData->getActionValue(DIM_A_beta));


	double tip_speed_ratio,beta;
	tip_speed_ratio= (state->getContinuousState(DIM_omega_t)*D/2.0)
		/state->getContinuousState(DIM_v);
	//C_p_2mass(tip_speed_ratio,blade_angle)
	double power_coef=C_p_2mass(tip_speed_ratio,state->getContinuousState(DIM_beta));
	//P_a= 0.5*rho*pi*R^2*C_p_2mass(lambda,beta)v^3
	double P_a= 0.5*rho*3.1415*pow(D/2,2.0)*power_coef*
		pow(state->getContinuousState(DIM_v),3.0);
	state->setContinuousState(DIM_P_a,P_a);
	//T_a= P_a/omega_t
	double T_a= P_a/state->getContinuousState(DIM_omega_t);
	state->setContinuousState(DIM_T_a,T_a);

	//d(omega_t)/dt=  -(K_r/J_r)*omega_t - T_ls/J_r + T_a/J_r
	//d(state->getContinuousState(DIM_omega_g))/dt= -(K_g/J_g)*state->getContinuousState(DIM_omega_g) - T_ls/(n_g*J_g) - T_em/J_g
	//d(T_ls)/dt= (K_ls- (B_ls*K_r/J_r))*omega_t + (1/n_g)*(B_ls*K_r/J_g - K_ls)*state->getContinuousState(DIM_omega_g) 
	//	- B_ls*((J_r+n_g*n_g*J_g)/((n_g*n_g)*J_g*J_r)*T_ls + (B_ls/J_r)*T_a + (B_ls/(n_g*J_g))*T_em
	double d_omega_t= -(K_r/J_r)*state->getContinuousState(DIM_omega_t) 
		- state->getContinuousState(DIM_T_ls)/J_r + T_a/J_r;
	double d_omega_g= -(K_g/J_g)*state->getContinuousState(DIM_omega_g) 
		+ state->getContinuousState(DIM_T_ls)/(N_g*J_g) 
		- pActionData->getActionValue(DIM_A_T_em)/J_g;
	double d_T_ls= (K_ls - (B_ls*K_r/J_r))*state->getContinuousState(DIM_omega_t)
		+ (1/N_g)*(B_ls*K_r/J_g - K_ls)*state->getContinuousState(DIM_omega_g) 
		- B_ls*((J_r+N_g*N_g*J_g)/((N_g*N_g)*J_g*J_r))*state->getContinuousState(DIM_T_ls)
		+ (B_ls/J_r)*T_a 
		+ (B_ls/(N_g*J_g))*pActionData->getActionValue(DIM_A_T_em);

	state->setContinuousState(DIM_omega_t,state->getContinuousState(DIM_omega_t)+d_omega_t*timestep);
	state->setContinuousState(DIM_omega_g,state->getContinuousState(DIM_omega_g)+d_omega_g*timestep);
	state->setContinuousState(DIM_T_ls,state->getContinuousState(DIM_T_ls)+d_T_ls*timestep);
}




void C2MassWindTurbine::getCADerivationX(CState *oldState, CContinuousActionData *action, ColumnVector *derivationX)
{

}
