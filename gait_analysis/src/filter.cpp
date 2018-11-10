#include <vector>
#include <iostream>

#include <config.h>
#include "filter.h"

using namespace std;

vector<double> Raw2Real(vector<int> input, double rate)
{
    vector<double> output;
    for (int i = 0; i < input.size(); i++)
        output.push_back(input[i] / rate);
    return output;
}

vector<double> Mean_Filter(vector<double> input, int windowsize)
{
    vector<double> output;
    float sum = 0;
    for (int i = 0; i < windowsize-1; i++)
    {
        sum += input[i];
        output.push_back(sum /windowsize);
    }
    for (int i = windowsize-1; i < input.size(); i++)
    {
        sum = 0;
        for (int j = i - windowsize + 1; j < i + 1; j++)
            sum += input[j];
        sum /= windowsize;
        output.push_back(sum);
    }
    // for (int i = input.size() - windowsize; i < input.size(); i++)
    //     output.push_back(input[i]);
    return output;
}

vector<double> Kalman_Filter(vector<double> input){
    int len = input.size();
    vector<double> K(len);
    vector<double> output(len);
    vector<double> P(len);

    output[0] = Kalman_X0;
    P[0] = Kalman_P0;

    for(int i = 1;i<len;i++){
        K[i] = P[i-1] / (P[i-1] + Kalman_R);
        output[i] = output[i-1] + K[i] * (input[i] - output[i-1]);
        P[i] = P[i-1] - K[i] * P[i-1] + Kalman_Q;
    }
    return output;
}
