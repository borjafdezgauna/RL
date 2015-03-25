#include "stdafx.h"
#include "FileManager.h"


void SetWorkPath()
{
	char buffer[1024];
	int ret= _chdir("../data");
	if (ret!=0)
		printf("ERROR: can't set the working directory to ../data\n");
	else printf("Working directory: %s\n",_getcwd(buffer,1024));
}
