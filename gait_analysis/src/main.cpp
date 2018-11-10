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
	DataInput(data);
	getResult();
	return 0;
}
