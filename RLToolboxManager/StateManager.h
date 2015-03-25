#ifndef __STATE_REPRESENTATION__
#define __STATE_REPRESENTATION__

#define FEATURE_TILE_CODING 1
#define FEATURE_RBF 2
#define FEATURE_RBF_AND 3
#define FEATURE_NN 10

class CStateModifier;
class CEnvironmentModel;

CStateModifier* GetContinuousStateModifier(CEnvironmentModel* pEnvironment,int feature_type= FEATURE_TILE_CODING
										   , char* variableList= 0);
char* GetStateModelName();
bool UseVariable(char *variableList, char* varName);
int getNumVariablesUsed(char *variableList);
#endif