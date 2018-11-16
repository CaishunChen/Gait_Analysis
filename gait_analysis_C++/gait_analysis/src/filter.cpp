#include "filter.h"

#include <vector>

#include <config.h>

using namespace std;

vector<F32> Raw2Real(vector<F32> input, F32 rate)
{
    vector<F32> output;
    for (U16 i = 0; i < input.size(); i++)
        output.push_back(input[i] / rate);
    return output;
}

vector<F32> Mean_Filter(vector<F32> input, U16 windowsize)
{
    vector<F32> output;
    F32 sum = 0;
    for (U16 i = 0; i < windowsize-1; i++)
    {
        sum += input[i];
        output.push_back(sum /windowsize);
    }
    for (U16 i = windowsize-1; i < input.size(); i++)
    {
        sum = 0;
        for (U16 j = i - windowsize + 1; j < i + 1; j++)
            sum += input[j];
        sum /= windowsize;
        output.push_back(sum);
    }
    // for (U16 i = input.size() - windowsize; i < input.size(); i++)
    //     output.push_back(input[i]);
    return output;
}

vector<F32> Kalman_Filter(vector<F32> input){
    U16 len = input.size();
    vector<F32> K(len);
    vector<F32> output(len);
    vector<F32> P(len);

    output[0] = Kalman_X0;
    P[0] = Kalman_P0;

    for(U16 i = 1;i<len;i++){
        K[i] = P[i-1] / (P[i-1] + Kalman_R);
        output[i] = output[i-1] + K[i] * (input[i] - output[i-1]);
        P[i] = P[i-1] - K[i] * P[i-1] + Kalman_Q;
    }
    return output;
}
