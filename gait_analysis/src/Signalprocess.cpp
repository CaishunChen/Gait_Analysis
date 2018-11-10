#include "Signalprocess.h"

#include <iostream>
#include <vector>
/*
* 函数:  findPeaks&findVallies
* 参数:  *src        源数据数组
*          src_lenth   源数据数组长度
*          distance    峰与峰,谷与谷的搜索间距
*          *indMax     找到的峰的index数组
*          *indMax_len 数组长度
*          *indMin     找到的谷的index数组
*          *indMin_len 数组长度
*/
vector<int> findPeaks(vector<double> src, double distance, double lowbound, double upbound)
{
    int src_lenth = src.size();
    vector<int> sign(src_lenth);

    int indMax[9999] = {0};
    int max_index = 0, min_index = 0;
    int indMax_len = 0;

    for (int i = 1; i < src_lenth; i++)
    {
        double diff = src[i] - src[i - 1];
        if (diff > 0)
            sign[i - 1] = 1;
        else if (diff < 0)
            sign[i - 1] = -1;
        else
            sign[i - 1] = 0;
    }
    for (int j = 1; j < src_lenth - 1; j++)
    {
        double diff = sign[j] - sign[j - 1];
        if (diff < 0)
            indMax[max_index++] = j;
    }
    int *flag_max_index = (int *)malloc(sizeof(int) * (max_index > min_index ? max_index : min_index));
    int *idelete = (int *)malloc(sizeof(int) * (max_index > min_index ? max_index : min_index));
    int *temp_max_index = (int *)malloc(sizeof(int) * (max_index > min_index ? max_index : min_index));
    int bigger = 0;
    double tempvalue = 0;
    int i, j, k;
    //波峰
    for (int i = 0; i < max_index; i++)
    {
        flag_max_index[i] = 0;
        idelete[i] = 0;
    }
    for (i = 0; i < max_index; i++)
    {
        tempvalue = -1;
        for (j = 0; j < max_index; j++)
            if (!flag_max_index[j])
                if (src[indMax[j]] > tempvalue)
                {
                    bigger = j;
                    tempvalue = src[indMax[j]];
                }
        flag_max_index[bigger] = 1;
        if (!idelete[bigger])
        {
            for (k = 0; k < max_index; k++)
            {
                idelete[k] |= (indMax[k] - distance <= indMax[bigger] & indMax[bigger] <= indMax[k] + distance);
            }
            idelete[bigger] = 0;
        }
    }
    for (i = 0, j = 0; i < max_index; i++)
    {
        if (!idelete[i])
            temp_max_index[j++] = indMax[i];
    }
    for (i = 0; i < max_index; i++)
    {
        if (i < j)
            indMax[i] = temp_max_index[i];
        else
            indMax[i] = 0;
    }
    max_index = j;
    indMax_len = max_index;

    vector<int> peaks_index;
    for (i = 0; i < indMax_len; i++)
        if (src[indMax[i]] >= lowbound && src[indMax[i]] <= upbound)
            peaks_index.push_back(indMax[i]);

    free(flag_max_index);
    free(temp_max_index);
    free(idelete);

    return peaks_index;
}

vector<int> findVallies(vector<double> src, double distance, double lowbound, double upbound)
{
    int src_lenth = src.size();
    vector<int> sign(src_lenth);
    int max_index = 0,
        min_index = 0;
    int indMin[9999] = {0};
    int indMin_len = 0;

    for (int i = 1; i < src_lenth; i++)
    {
        double diff = src[i] - src[i - 1];
        if (diff > 0)
            sign[i - 1] = 1;
        else if (diff < 0)
            sign[i - 1] = -1;
        else
            sign[i - 1] = 0;
    }
    for (int j = 1; j < src_lenth - 1; j++)
    {
        double diff = sign[j] - sign[j - 1];
        if (diff > 0)
            indMin[min_index++] = j;
    }

    int *flag_max_index = (int *)malloc(sizeof(int) * (max_index > min_index ? max_index : min_index));
    int *idelete = (int *)malloc(sizeof(int) * (max_index > min_index ? max_index : min_index));
    int *temp_max_index = (int *)malloc(sizeof(int) * (max_index > min_index ? max_index : min_index));
    int bigger = 0;
    double tempvalue = 0;
    int i, j, k;

    //波谷
    for (int i = 0; i < min_index; i++)
    {
        flag_max_index[i] = 0;
        idelete[i] = 0;
    }
    for (i = 0; i < min_index; i++)
    {
        tempvalue = 1;
        for (j = 0; j < min_index; j++)
            if (!flag_max_index[j])
                if (src[indMin[j]] < tempvalue)
                {
                    bigger = j;
                    tempvalue = src[indMin[j]];
                }

        flag_max_index[bigger] = 1;
        if (!idelete[bigger])
        {
            for (k = 0; k < min_index; k++)
                idelete[k] |= (indMin[k] - distance <= indMin[bigger] & indMin[bigger] <= indMin[k] + distance);
            idelete[bigger] = 0;
        }
    }
    for (i = 0, j = 0; i < min_index; i++)
        if (!idelete[i])
            temp_max_index[j++] = indMin[i];

    for (i = 0; i < min_index; i++)
        if (i < j)
            indMin[i] = temp_max_index[i];
        else
            indMin[i] = 0;
    min_index = j;
    indMin_len = min_index;

    vector<int> vallies_index;
    for (i = 0; i < indMin_len; i++)
        if (src[indMin[i]] >= lowbound && src[indMin[i]] <= upbound)
            vallies_index.push_back(indMin[i]);

    free(flag_max_index);
    free(temp_max_index);
    free(idelete);

    return vallies_index;
}

vector<vector<int>> findNearzeros(vector<double> src, int minnum, double lowbound, double upbound)
{
    vector<int> buffer;
    vector<int> buffer2;
    vector<vector<int>> index;
    bool count = false;
    for (int i = 0; i < src.size(); i++)
        if (src[i] <= upbound && src[i] >= lowbound)
            buffer.push_back(i);
    for (int i = 1; i < buffer.size(); i++)
    {
        if (buffer2.size() == minnum)
            count = true;
        if (buffer[i] - buffer[i - 1] != 1)
        {
            if (count)
            {
                index.push_back(buffer2);
                count = false;
            }
            buffer2.clear();
        }
        buffer2.push_back(buffer[i]);
    }
    return index;
}