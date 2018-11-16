#include "filter.h"

#include <stdlib.h>
#include <config.h>

using namespace std;

void Raw2Real_A(F32 *input,U16 input_length, F32 rate)
{
    for(U16 i = 0; i < input_length; i++)
        input[i] = input[i]/rate;
}

void Mean_Filter_A(F32 *input,U16 input_length, U16 windowsize)
{
    F32 *output = (F32*)malloc(input_length * sizeof(F32));
    F32 sum = 0;
    for (U16 i = 0; i < windowsize-1; i++)
    {
        sum += input[i];
        output[i]= (sum /windowsize);
    }
    for (U16 i = windowsize-1; i < input_length; i++)
    {
        sum = 0;
        for (U16 j = i - windowsize + 1; j < i + 1; j++)
            sum += input[j];
        sum /= windowsize;
        output[i] = sum;
    }
    
    for (U16 i =0; i < input_length; i++)
        input[i] = output[i];

}

void Kalman_Filter_A(F32 *input,U16 len){
    F32 *K = (F32*)malloc(len * sizeof(F32));
    F32 *P = (F32*)malloc(len * sizeof(F32));
    F32 *output = (F32*)malloc(len * sizeof(F32));

    output[0] = Kalman_X0;
    P[0] = Kalman_P0;

    for(U16 i = 1;i<len;i++){
        K[i] = P[i-1] / (P[i-1] + Kalman_R);
        output[i] = output[i-1] + K[i] * (input[i] - output[i-1]);
        P[i] = P[i-1] - K[i] * P[i-1] + Kalman_Q;
    }
    for (U16 i =0; i < len; i++)
        input[i] = output[i];
}
