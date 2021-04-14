



#include "Sanran.hpp"


#include <stdio.h>
#include "main.h"


Sanran::Sanran()
{

	printf("oppai...\n");

	count = 0;

}


void Sanran::UpdateAsync()
{

	HAL_Delay(1000);

	printf("count = %d\n", count++);

}


void Sanran::UpdateSyncHS()
{

}


void Sanran::UpdateSyncLS()
{

}





