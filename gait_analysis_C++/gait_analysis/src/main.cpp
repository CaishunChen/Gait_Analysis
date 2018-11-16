#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include "config.h"
#include "Dataprocess.h"

using namespace std;

int main(int argv, char *arg[])
{
	fstream f(FILE_PATH); 		//创建一个fstream文件流对象
	vector<string> data;		 //创建一个vector<string>对象
	string line;				 //保存读入的每一行
	while (getline(f, line))	 //会自动把\n换行符去掉
	{
		data.push_back(line);
	}
	RegisterInfo(HUMAN_WEIGHT,HUMAN_HEIGHT);
	ProcessNewData(data);

	cout << "步数为 " << getStepNum() << endl;
	cout << "步频为 " << GaitanalyseGetStepFrequence() <<"步/min"<< endl;
    cout << "支撑相占比为 " << GaitanalyseGetStancePercent() << " %" << endl;
    cout << "沿x轴脚落地角度(内外翻) " << getStrickRollMean() << "°" << endl;
    cout << "步距为 " << GaitanalyseGetStrideLength() << " cm" << endl;
    cout << "平均步速为 " << getSpeedMean() << " m/s" << endl;
    cout << "平均最大摆速为 " << GaitanalyseSwingSpeedMean() << " m/s" << endl;
	cout << "经过身高体重公式法计算单步距离，再根据步数计算的行走距离为 " << getTotalDistance_Formula() << " m" << endl;
	cout << "只对沿X轴加速度二重积分，所得到的行走距离为 " << getTotalDistance_Integral() << " m" << endl;
	cout << "经过单步积分再以单步步长估计总体的行走距离为 " << getTotalDistance_Mix() << " m" << endl;
	return 0;
}
