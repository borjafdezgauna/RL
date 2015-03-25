#include "stdafx.h"
#include "WindTurbine.h"
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

//[4]
//"Multivariable control strategy for variable speed, variable pitch wind turbines"
//Boubekeur Boukhezzar and L. Lupu and H. Siguerdidjane, M. Hand

//[5]
//"Power control design for Variable-speed Wind Turbines"
//Yolanda Vidal, Leonardo Acho, Ningsu Luo, Mauricio Zapateiro and Francesc Pozo

//[6]
//FAST: test 13

#define DIM_P_set 0 //P_setpoint
#define DIM_P_e 1
#define DIM_P_a 2 //P_a
#define DIM_v 3 //v_w
#define DIM_T_a 4 //T_a
#define DIM_omega_r 5 //omega_r
#define DIM_d_omega_r 6 //d_omega_r
#define DIM_beta 7 //beta
#define DIM_d_beta 8 //d_beta
#define DIM_T_g 9  //T_g
#define DIM_d_T_g 10//d_T_g
#define DIM_P_error 11
#define DIM_omega_r_error 12
#define DIM_integrative_omega_r_error 13

#define NUM_STATE_VARS 14


#define MIN_BLADE_ANGLE (-5*2*3.1415/360.0) // radians //degrees [4]
#define MAX_BLADE_ANGLE (30*2*3.1415/360.0) // radians //degrees [4]  

#define MAX_AERODYNAMIC_POWER  1600000.0 //W
#define MAX_AERODYNAMIC_TORQUE 400000.0 //N/m

#define NOMINAL_ROTOR_SPEED 4.39823//rad/s - 42 rpm[thesis boukhezzar]
#define MAX_ROTOR_SPEED NOMINAL_ROTOR_SPEED+2.0
#define MIN_ROTOR_SPEED NOMINAL_ROTOR_SPEED-2.0

//////////////////////////////////////////////////////////
#define INITIAL_ROTOR_SPEED NOMINAL_ROTOR_SPEED //rad/s nominal-speed [6]
#define INITIAL_ACTION_TORQUE 142000.0//-36.279653322202989 //[4] N/m

#define MAX_GENERATOR_TORQUE_INC 100000


#define P_e_nom 600000.0 //W [4]
#define HUB_HEIGHT 36.6 //m [4]
#define N_g 43.165 //[4]
#define D 43.3 //m [4]
//#define J_r 3.25e5 //kg*m^2 [3]
//#define K_r 27.36 //N*m/rad/s [3]
#define rho 1.29 //kg/m^3 [3]
//#define J_g 34.400 //kg*m^2 [3]
//#define K_g 0.2 //N*m/rad/s [3]
#define K_t 400.0 //N*m/rad/s [thesis boukhezzar]
#define J_t 3.92e5 //kg*m^2

#define DIM_A_beta 0 //beta
#define DIM_A_torque 1 //T_g

#define NUM_ACTION_VARS 2

#define MIN_GENERATOR_TORQUE 100000 //from log
#define MAX_GENERATOR_TORQUE 162000 //N*m [2]
#define MAX_ACTION_BLADE_RATE ((10.0*2*3.14159265)/360.0)// //rad/s +/- 10º/s

#define MIN_INTEGRATIVE_ROTOR_SPEED_ERROR -100
#define MAX_INTEGRATIVE_ROTOR_SPEED_ERROR 100

#define MAX_INITIAL_BLADE_ANGLE MIN_BLADE_ANGLE//((30.0*2*3.14159265)/360.0)
#define MIN_INITIAL_BLADE_ANGLE MAX_BLADE_ANGLE//((0.0*2*3.14159265)/360.0)

#define MAX_WIND_SPEED 50.0
#define MAX_TSR 15.0
#define NUM_BETA_SAMPLES 10000
#define NUM_WIND_SPEED_SAMPLES 1000
#define NUM_TSR_SAMPLES 1000


double C_p(double lambda, double beta) //[1]
{
	double lambda_i,c_p;
	beta= beta*360.0/(2*3.14159265);

	lambda_i= 1.0/((1.0/(lambda+0.08*beta)) - 0.035/(pow(beta,3.0)+1.0));
	//lambda_i= 1.0/((1.0/(lambda+0.089)) - 0.035/(pow(beta,3.0)+1.0));
	if (lambda_i==0.0)
		return 0.00001;

	double e= exp(-16.5/lambda_i);
	c_p= 0.5*(116.0/lambda_i - 0.4*beta -5.0)*exp(-16.5/lambda_i);
	//c_p= 0.5*(116.0/lambda_i - 0.4*(beta -5.0))*exp(-16.5/lambda_i);
	return max(0.00001,c_p);
}
double C_q(double lambda, double beta)
{
	if (lambda==0.0)
		return 0.0;

	return C_p(lambda,beta)/lambda;
}
double AerodynamicTorque(double tip_speed_ratio,double beta, double wind_speed)
{
	double cq= C_q(tip_speed_ratio,beta);

	//Ta= 0.5 * rho * pi * R^3 * C_q(lambda,beta) * v^2
	double torque= 0.5*rho*3.14159265*pow(D*0.5,3.0)*cq*wind_speed*wind_speed;
	return torque;
}

double AerodynamicPower(double tip_speed_ratio,double beta, double wind_speed)
{
	double cp= C_p(tip_speed_ratio,beta);

	//Pa= 0.5 * rho * pi * R^2 * C_p(lambda,beta) * v^3
	double power= 0.5*rho*3.14159265*(D*0.5)*(D*0.5)*cp*pow(wind_speed,3.0);
	return power;
}
double AerodynamicPower(double cp, double wind_speed)
{
	//Pa= 0.5 * rho * pi * R^2 * C_p(lambda,beta) * v^3
	double power= 0.5*rho*3.14159265*(D*0.5)*(D*0.5)*cp*pow(wind_speed,3.0);
	return power;
}

double sgn(double value)
{
	if (value<0.0) return -1.0;
	else if (value>0.0) return 1.0;

	return 0.0;
}

void FindSuitableParameters(double initial_wind_speed,double initial_rotor_speed
							,double &initial_torque,double &initial_blade_angle)
{
	double c_p,best_c_p= -1000.0;
	double beta, tsr;
	double torque, best_torque= -1000000000.0;
//	double desired_c_p= (initial_power*2)/(rho*3.14159265*(D*0.5)*(D*0.5)*pow(initial_wind_speed,3.0));
//	double check= AerodynamicPower(desired_c_p,initial_wind_speed);

	initial_blade_angle= 0.01;
	tsr= initial_rotor_speed*D*0.5/initial_wind_speed;

	double minInitialBeta= MIN_INITIAL_BLADE_ANGLE;
	double initialBetaRange= MAX_INITIAL_BLADE_ANGLE-MIN_INITIAL_BLADE_ANGLE;

	for (int j= 0; j<NUM_BETA_SAMPLES; j++)
	{
		beta= minInitialBeta + (double)j * (initialBetaRange/(double)NUM_BETA_SAMPLES);
		
		torque= AerodynamicTorque(tsr,beta,initial_wind_speed);
		c_p= C_p(tsr,beta);

		if (fabs(initial_torque-torque)<fabs(initial_torque-best_torque))
		{
			initial_blade_angle= beta;
			best_c_p= c_p;
			//initial_rotor_speed= initial_wind_speed*tsr/(D*0.5);
			best_torque= torque;
		}
	}
	initial_torque= best_torque;
}







CWindTurbine::CWindTurbine():CContinuousTimeAndActionTransitionFunction(new CStateProperties(NUM_STATE_VARS, 0)
								,new CContinuousAction(new CContinuousActionProperties(NUM_ACTION_VARS))
								, g_pParameters->getParameter("DELTA_T")
								,(int)g_pParameters->getParameter("NUM_SIMULATION_STEPS"))
{
	//double omega_r,v_w,beta,tsr,t1,t2;

	//omega_r=4.4306;
	//beta= 0.5196;
	//v_w= 21.18;
	//tsr= omega_r*D*0.5/v_w;
	//t1=AerodynamicTorque(tsr,beta,v_w);

	//omega_r=44.4276;
	//beta= 0.5199;
	//v_w= 21.76;
	//tsr= omega_r*D*0.5/v_w;
	//t2=AerodynamicTorque(tsr,beta,v_w);

	properties->setMinValue(DIM_P_set,P_e_nom-100000);
	properties->setMaxValue(DIM_P_set,P_e_nom+100000);
	properties->setVarName(DIM_P_set,"Power setpoint");

	properties->setMinValue(DIM_P_e,P_e_nom-100000);
	properties->setMaxValue(DIM_P_e,P_e_nom+100000);
	properties->setVarName(DIM_P_e,"Electrical power");

	properties->setMinValue(DIM_P_error,-100000);
	properties->setMaxValue(DIM_P_error,100000);
	properties->setVarName(DIM_P_error,"Power error");

	properties->setMinValue(DIM_P_a,0.0);
	properties->setMaxValue(DIM_P_a,MAX_AERODYNAMIC_POWER);
	properties->setVarName(DIM_P_a,"Mechanical power");

	properties->setMinValue(DIM_v,1.0);
	properties->setMaxValue(DIM_v,50.0);
	properties->setVarName(DIM_v,"Wind speed");

	properties->setMinValue(DIM_T_a,0);
	properties->setMaxValue(DIM_T_a,MAX_AERODYNAMIC_TORQUE);
	properties->setVarName(DIM_T_a,"Aerodynamic torque");

	properties->setMinValue(DIM_omega_r,MIN_ROTOR_SPEED);
	properties->setMaxValue(DIM_omega_r,MAX_ROTOR_SPEED);
	properties->setVarName(DIM_omega_r,"Rotor speed");

	properties->setMinValue(DIM_omega_r_error,(MIN_ROTOR_SPEED)-(MAX_ROTOR_SPEED));
	properties->setMaxValue(DIM_omega_r_error,(MAX_ROTOR_SPEED)-(MIN_ROTOR_SPEED));
	properties->setVarName(DIM_omega_r_error,"Rotor speed error");

	properties->setMinValue(DIM_d_omega_r,-2.0);
	properties->setMaxValue(DIM_d_omega_r,2.0);
	properties->setVarName(DIM_d_omega_r,"Rotor acceleration");

	properties->setMinValue(DIM_beta,MIN_BLADE_ANGLE);
	properties->setMaxValue(DIM_beta,MAX_BLADE_ANGLE);
	properties->setVarName(DIM_beta,"Blade angle");
	
	properties->setMinValue(DIM_d_beta,-MAX_ACTION_BLADE_RATE);
	properties->setMaxValue(DIM_d_beta,MAX_ACTION_BLADE_RATE);
	properties->setVarName(DIM_d_beta,"d_beta");

	properties->setMinValue(DIM_T_g,MIN_GENERATOR_TORQUE);
	properties->setMaxValue(DIM_T_g,MAX_GENERATOR_TORQUE);
	properties->setVarName(DIM_T_g,"Generator torque");

	properties->setMinValue(DIM_d_T_g,-MAX_GENERATOR_TORQUE_INC);
	properties->setMaxValue(DIM_d_T_g,MAX_GENERATOR_TORQUE_INC);
	properties->setVarName(DIM_d_T_g,"d_T_g");
	
	properties->setMinValue(DIM_integrative_omega_r_error,MIN_INTEGRATIVE_ROTOR_SPEED_ERROR);
	properties->setMaxValue(DIM_integrative_omega_r_error,MAX_INTEGRATIVE_ROTOR_SPEED_ERROR);
	properties->setVarName(DIM_integrative_omega_r_error,"Integrative rotor speed error");

	contAction->getContinuousActionProperties()->setActionName(DIM_A_beta,string("d(Beta)"));
	contAction->getContinuousActionProperties()->setMaxActionValue(DIM_A_beta,MAX_ACTION_BLADE_RATE);
	contAction->getContinuousActionProperties()->setMinActionValue(DIM_A_beta,-MAX_ACTION_BLADE_RATE);
	contAction->getContinuousActionProperties()->setActionName(DIM_A_torque,string("d(T_g)"));
	contAction->getContinuousActionProperties()->setMaxActionValue(DIM_A_torque,MAX_GENERATOR_TORQUE_INC);
	contAction->getContinuousActionProperties()->setMinActionValue(DIM_A_torque,-MAX_GENERATOR_TORQUE_INC);

	m_pWindData= new CSetPointGenerator("WTGenerator-wind.txt");
	m_pSetpointPower= new CSetPointGenerator("WTGenerator-power.txt");

	double initial_T_g= P_e_nom/NOMINAL_ROTOR_SPEED;
	initial_torque= initial_T_g + K_t*NOMINAL_ROTOR_SPEED;
	initial_blade_angle= 0.0;
}

CWindTurbine::~CWindTurbine()
{
	delete m_pWindData;
	delete m_pSetpointPower;
}

bool CWindTurbine::isFailedState(CState *state)
{
	
	return false;
}



void CWindTurbine::getResetState(CState *state)
{
	m_t= 0.0;

	double initial_wind_speed= m_pWindData->getPointSet(m_t);//CONSTANT_WIND_SPEED;;
	double initial_rotor_speed= NOMINAL_ROTOR_SPEED;//= INITIAL_ROTOR_SPEED*D*0.5/initial_wind_speed;

	double tsr= initial_rotor_speed*D*0.5/initial_wind_speed;


	if (initial_blade_angle==0.0)
	{
		printf("Calculating initial torque and blade angle parameters...\n");
		FindSuitableParameters(initial_wind_speed,initial_rotor_speed,initial_torque,initial_blade_angle);
		printf("T_g= %f     //    Beta= %f\n",initial_torque,initial_blade_angle);
	}

	state->setContinuousState(DIM_T_a,AerodynamicTorque(tsr,initial_blade_angle,initial_wind_speed));
	state->setContinuousState(DIM_P_a, state->getContinuousState(DIM_T_a)*initial_rotor_speed);
	state->setContinuousState(DIM_P_set,m_pSetpointPower->getPointSet(m_t));

	state->setContinuousState(DIM_P_e, P_e_nom);
	state->setContinuousState(DIM_P_error,state->getContinuousState(DIM_P_e)
		-state->getContinuousState(DIM_P_set));
	state->setContinuousState(DIM_v,initial_wind_speed);

	state->setContinuousState(DIM_omega_r,initial_rotor_speed);
	state->setContinuousState(DIM_omega_r_error,initial_rotor_speed-NOMINAL_ROTOR_SPEED);
	state->setContinuousState(DIM_d_omega_r,0.0);
	state->setContinuousState(DIM_beta,initial_blade_angle);
	state->setContinuousState(DIM_d_beta,0.0);
	state->setContinuousState(DIM_T_g,P_e_nom/initial_rotor_speed);
	state->setContinuousState(DIM_d_T_g,0.0);
	state->setContinuousState(DIM_integrative_omega_r_error, 0.0);
}


void CWindTurbine::doSimulationStep(CState *state, double timestep, CAction *action, CActionData *data)
{
	m_t+= timestep;

	CContinuousActionData* pActionData= (CContinuousActionData*)action->getActionData();

	//getDerivationX(state, action, derivation, data);

	state->setContinuousState(DIM_P_set,m_pSetpointPower->getPointSet(m_t));
	state->setContinuousState(DIM_v,m_pWindData->getPointSet(m_t));//CONSTANT_WIND_SPEED);//-0.05+0.1*((double)(rand()%1000))/1000.0);//

	//beta= beta + d(beta)/dt
	double beta;
	beta= state->getContinuousState(DIM_beta);
	//if (USE_ABS_ACTIONS)
	//	beta= pActionData->getActionValue(DIM_A_beta);
	//else
	//	beta= state->getContinuousState(DIM_beta) + pActionData->getActionValue(DIM_A_beta)*timestep;
	
	//P_e= T_g*omega_r
	double T_g, omega_r;
	omega_r= state->getContinuousState(DIM_omega_r);

	T_g= state->getContinuousState(DIM_T_g);
	//if (USE_ABS_ACTIONS)
	//	T_g= pActionData->getActionValue(DIM_A_torque);
	//else
	//	T_g= state->getContinuousState(DIM_T_g) + pActionData->getActionValue(DIM_A_torque)*timestep;

	state->setContinuousState(DIM_P_e,T_g*omega_r);
	state->setContinuousState(DIM_P_error,state->getContinuousState(DIM_P_e)
		-state->getContinuousState(DIM_P_set));

	double tip_speed_ratio;
	double d_omega_r;

	tip_speed_ratio= (state->getContinuousState(DIM_omega_r)*D*0.5)
		/state->getContinuousState(DIM_v);
	
	//C_p(tip_speed_ratio,blade_angle)
	//double power_coef=C_p(tip_speed_ratio,beta);
	//P_a= 0.5*rho*pi*R^2*C_p(lambda,beta)v^3
	double P_a= AerodynamicPower(tip_speed_ratio,beta,state->getContinuousState(DIM_v));
	state->setContinuousState(DIM_P_a,P_a);
	//T_a= P_a/omega_r
	double T_a= P_a/omega_r;//AerodynamicTorque(tip_speed_ratio,beta,state->getContinuousState(DIM_v));
	state->setContinuousState(DIM_T_a,T_a);
	state->setContinuousState(DIM_omega_r_error,state->getContinuousState(DIM_omega_r)-NOMINAL_ROTOR_SPEED);
	state->setContinuousState(DIM_integrative_omega_r_error,
								state->getContinuousState(DIM_integrative_omega_r_error) 
								+ state->getContinuousState(DIM_omega_r_error)*timestep);

	//d(omega_r)= (T_a - K_t*omega_r - T_g) / J_t
	d_omega_r= (T_a - K_t*omega_r - T_g) / J_t;

	state->setContinuousState(DIM_d_omega_r,d_omega_r);

	state->setContinuousState(DIM_omega_r,omega_r + d_omega_r*timestep);

	
	state->setContinuousState(DIM_d_T_g, pActionData->getActionValue(DIM_A_torque));
	T_g= state->getContinuousState(DIM_T_g) + pActionData->getActionValue(DIM_A_torque)*timestep;
	state->setContinuousState(DIM_d_beta, pActionData->getActionValue(DIM_A_beta));
	beta= state->getContinuousState(DIM_beta) + pActionData->getActionValue(DIM_A_beta)*timestep;

	//state->setContinuousState(DIM_T_g_inc,T_g-state->getContinuousState(DIM_T_g));
	state->setContinuousState(DIM_T_g,T_g);
	state->setContinuousState(DIM_beta, beta);
}


void CWindTurbine::getCADerivationX(CState *oldState, CContinuousActionData *action, ColumnVector *derivationX)
{

}


//CWindTurbineMVController BOUKHEZZAR
CWindTurbineControllerBoukhezzar::CWindTurbineControllerBoukhezzar(CContinuousAction *contAction, CActionSet *pStaticCActionSet, int randomControllerMode):CContinuousActionController(contAction,randomControllerMode)
{
	addParameter("KP",g_pParameters->getParameter("KP"));
	addParameter("C0",g_pParameters->getParameter("C0"));



	m_pStaticCActionSet= pStaticCActionSet;
	m_contAction= contAction;
};

CWindTurbineControllerBoukhezzar::~CWindTurbineControllerBoukhezzar()
{
};

void CWindTurbineControllerBoukhezzar::getNextContinuousAction(CStateCollection *stateCol, CContinuousActionData *action)
{
	CState *state = stateCol->getState();

	//d(Tg)/dt= (1/omega_r)*(C_0*error_P - (1/J_t)*(T_a*T_g - K_t*omega_r*T_g - T_g*T_g))
	//d(beta)/dt= K_p*(omega_ref - omega_r)

	double omega_r= state->getContinuousState(DIM_omega_r);
	double C_0= getParameter("C0");
	double error_P= -state->getContinuousState(DIM_P_error);
	double T_a= state->getContinuousState(DIM_T_a);
	double K_p= getParameter("KP");

	double omega_ref= NOMINAL_ROTOR_SPEED;//GetOmegaRef(state->getContinuousState(DIM_v));
	double T_g= state->getContinuousState(DIM_T_g);//action->getActionValue(DIM_A_torque);
	double beta= state->getContinuousState(DIM_beta);
	
	double d_T_g= (1.0/omega_r)*(C_0*error_P - (1.0/J_t)
		*(T_a*T_g - K_t*omega_r*T_g - T_g*T_g));

	double d_beta= K_p*state->getContinuousState(DIM_omega_r_error);

	action->setActionValue(DIM_A_beta,d_beta);
	action->setActionValue(DIM_A_torque,d_T_g);
}


//CWindTurbineMVController VIDAL
CWindTurbineControllerVidal::CWindTurbineControllerVidal(CContinuousAction *contAction, CActionSet *pStaticCActionSet, int randomControllerMode):CContinuousActionController(contAction,randomControllerMode)
{
	addParameter("A",g_pParameters->getParameter("A"));
	addParameter("K_alpha",g_pParameters->getParameter("K_alpha"));
	addParameter("KP",g_pParameters->getParameter("KP"));
	addParameter("KI",g_pParameters->getParameter("KI"));

	m_pStaticCActionSet= pStaticCActionSet;
	m_contAction= contAction;
	m_integrative_omega_r_error= 0.0;
};

CWindTurbineControllerVidal::~CWindTurbineControllerVidal()
{
};

void CWindTurbineControllerVidal::getNextContinuousAction(CStateCollection *stateCol, CContinuousActionData *action)
{
	CState *state = stateCol->getState();

	if (state->isResetState())
		m_integrative_omega_r_error=0.0;

	//d(Tg)/dt= (-1/omega_r)*(T_g*(a*omega_r-d_omega_r)-a*P_setpoint + K_alpha*sgn(P_a-P_setpoint))
	//d(beta)/dt= K_p*(omega_ref - omega_r) + K_i*(error_integral)

	double A= getParameter("A");
	double K_alpha= getParameter("K_alpha");
	double K_p= getParameter("KP");
	double K_i= getParameter("KI");

	double T_a= state->getContinuousState(DIM_T_a);
	double omega_r= state->getContinuousState(DIM_omega_r);
	double d_omega_r= state->getContinuousState(DIM_d_omega_r);
	double P_set= state->getContinuousState(DIM_P_set);
	double P_e= state->getContinuousState(DIM_P_e);
	double error_P= state->getContinuousState(DIM_P_error);
	double omega_ref= NOMINAL_ROTOR_SPEED;
	double T_g= state->getContinuousState(DIM_T_g);
	double beta= state->getContinuousState(DIM_beta);
	
	double d_T_g;
	
	if (omega_r!=0.0) d_T_g= (-1/omega_r)*(T_g*(A*omega_r+d_omega_r) - A*P_set + K_alpha*sgn(error_P));
	else d_T_g= 0.0;

	double error_omega= state->getContinuousState(DIM_omega_r_error);
	
	double d_beta=K_p*(error_omega) + K_i*state->getContinuousState(DIM_integrative_omega_r_error);
				 /*0.5*K_p*error_omega*(1.0+sgn(error_omega))
				+ K_i*state->getContinuousState(DIM_integrative_omega_r_error);*/

	action->setActionValue(DIM_A_beta,d_beta);
	action->setActionValue(DIM_A_torque,d_T_g);
}
