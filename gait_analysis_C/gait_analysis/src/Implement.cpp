#include "Implement.h"
#include <iostream>
#include<config.h>
#include<stdio.h>
#include "Dataprocess.h"
#include <fstream>
static float ax_tmp[MAX_ARRAY_SIZE];
static float ay_tmp[MAX_ARRAY_SIZE];
static float az_tmp[MAX_ARRAY_SIZE];

static float gx_tmp[MAX_ARRAY_SIZE];
static float gy_tmp[MAX_ARRAY_SIZE];
static float gz_tmp[MAX_ARRAY_SIZE];

static int full = 0;
void ArithProcessNewAccData(INT16S * piNewAcc,const INT8U uNum)
{
	INT16U   i;
	if (uNum == 0)
		return;

    full += uNum;
    for(i = 0;i<MAX_ARRAY_SIZE-uNum;i++)
    {
        ax_tmp[i] = ax_tmp[i+uNum]; 
        ay_tmp[i] = ay_tmp[i+uNum]; 
        az_tmp[i] = az_tmp[i+uNum]; 
        gx_tmp[i] = gx_tmp[i+uNum]; 
        gy_tmp[i] = gy_tmp[i+uNum]; 
        gz_tmp[i] = gz_tmp[i+uNum]; 
    }
    S16 count = -1;
    for(i = MAX_ARRAY_SIZE-uNum;i<MAX_ARRAY_SIZE;i++){
        ay_tmp[i] = 1.0*(piNewAcc[++count]); 
        ax_tmp[i] = 1.0*(piNewAcc[++count]); 
        az_tmp[i] = 1.0*(piNewAcc[++count]); 
        gy_tmp[i] = 1.0*(piNewAcc[++count]); 
        gx_tmp[i] = 1.0*(piNewAcc[++count]); 
        gz_tmp[i] = 1.0*(piNewAcc[++count]); 
        
    }
    // cout<<ax_tmp[0]<<endl;
    if(full >=MAX_ARRAY_SIZE )
        ProcessNewData(ax_tmp,ay_tmp,az_tmp,gy_tmp,gx_tmp,gz_tmp);
// {ofstream fout;
// fout.open("test.txt",ios::app);
// for(i=0;i<MAX_ARRAY_SIZE;i++)
// fout<<ax_tmp[i]<<" "<<ay_tmp[i]<<" "<<az_tmp[i]<<" "<<gx_tmp[i]<<" "<<gy_tmp[i]<<" "<<gz_tmp[i]<<" \n";
// fout.close();}
}
