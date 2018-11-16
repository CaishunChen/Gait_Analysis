#include <stdio.h>

#include "config.h"
#include "Dataprocess.h"
#include "Implement.h"

int main(int argv, char *arg[])
{
	int c,count;
	count = 0;
	INT16S bf[6*100];


	ArithmeticSetWeight(55);


	FILE *fp = NULL; //需要注意
	fp = fopen(FILE_PATH, "r");
	if (NULL == fp)
		return -1; //要返回错误代码

	while (fscanf(fp, "%d", &c) != EOF)
	{
		bf[count++] = c;
		if (count == 6 * 100)
		{
			count = 0;
			ArithProcessNewAccData(bf, 100);
			GaitanalyseGetStancePercent();
			GaitanalyseGetswingSpeed();
			GaitanalyseGetSpeed();
			GaitanalyseGetStrideLength();
			GaitanalyseGetStepFrequence();
			ArithmeticGetStepNum();
			ArithmeticGetTotaldistance();
			EnergyArithmatic();
			// GaitanalyseGetVerticalLength();
		}
	}
	fclose(fp);
	fp = NULL; //需要指向空，否则会指向原打开文件地址
	
	return 0;
}
