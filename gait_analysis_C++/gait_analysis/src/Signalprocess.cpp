#include "Signalprocess.h"

#include "config.h"
#include <vector>
#include <stdlib.h>
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
vector<U16> findPeaks(vector<F32> src, F32 distance, F32 lowbound, F32 upbound)
{
    S16 src_lenth = src.size();
    vector<S16> sign(src_lenth);
    S16 indMax[9999] = {0};
    S16 max_index = 0, min_index = 0;
    S16 indMax_len = 0;
    S16 i, j, k;
    for (i = 1; i < src_lenth; i++)
    {
        F32 diff = src[i] - src[i - 1];
        if (diff > 0)
            sign[i - 1] = 1;
        else if (diff < 0)
            sign[i - 1] = -1;
        else
            sign[i - 1] = 0;
    }
    for (j = 1; j < src_lenth - 1; j++)
    {
        F32 diff = sign[j] - sign[j - 1];
        if (diff < 0)
            indMax[max_index++] = j;
    }
    S16 *flag_max_index = (S16 *)malloc(sizeof(S16) * (max_index > min_index ? max_index : min_index));
    S16 *idelete = (S16 *)malloc(sizeof(S16) * (max_index > min_index ? max_index : min_index));
    S16 *temp_max_index = (S16 *)malloc(sizeof(S16) * (max_index > min_index ? max_index : min_index));
    S16 bigger = 0;
    F32 tempvalue = 0;

    //波峰
    for (i = 0; i < max_index; i++)
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
                idelete[k] |= (indMax[k] - distance <= indMax[bigger] & indMax[bigger] <= indMax[k] + distance);
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
    indMax_len = j;
    // max_index = j;
    // indMax_len = max_index;

    vector<U16> peaks_index;
    for (i = 0; i < indMax_len; i++)
        if (src[indMax[i]] >= lowbound && src[indMax[i]] <= upbound)
            peaks_index.push_back(indMax[i]);

    free(flag_max_index);
    free(temp_max_index);
    free(idelete);

    return peaks_index;
}

vector<U16> findVallies(vector<F32> src, F32 distance, F32 lowbound, F32 upbound)
{
    S16 src_lenth = src.size();
    vector<S16> sign(src_lenth);
    vector<U16> vallies_index;
    S16 max_index = 0,
        min_index = 0;
    S16 indMin[9999] = {0};
    S16 indMin_len = 0;
    U16 i, j, k;
    F32 diff;

    for (i = 1; i < src_lenth; i++)
    {
        diff = src[i] - src[i - 1];
        if (diff > 0)
            sign[i - 1] = 1;
        else if (diff < 0)
            sign[i - 1] = -1;
        else
            sign[i - 1] = 0;
    }
    for (j = 1; j < src_lenth - 1; j++)
    {
        diff = sign[j] - sign[j - 1];
        if (diff > 0)
            indMin[min_index++] = j;
    }

    S16 *flag_max_index = (S16 *)malloc(sizeof(S16) * (max_index > min_index ? max_index : min_index));
    S16 *idelete = (S16 *)malloc(sizeof(S16) * (max_index > min_index ? max_index : min_index));
    S16 *temp_max_index = (S16 *)malloc(sizeof(S16) * (max_index > min_index ? max_index : min_index));
    S16 bigger = 0;
    F32 tempvalue = 0;

    //波谷
    for (i = 0; i < min_index; i++)
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
    indMin_len = j;
    // min_index = j;
    // indMin_len = min_index;

    for (i = 0; i < indMin_len; i++)
        if (src[indMin[i]] >= lowbound && src[indMin[i]] <= upbound)
            vallies_index.push_back(indMin[i]);

    free(flag_max_index);
    free(temp_max_index);
    free(idelete);

    return vallies_index;
}

vector<vector<U16> > findNearzeros(vector<F32> src, U16 minnum, F32 lowbound, F32 upbound)
{
    vector<U16> buffer;
    vector<U16> buffer2;
    vector<vector<U16> > index;
    bool count = false;
    S16 i;

    for (i = 0; i < src.size(); i++)
        if (src[i] <= upbound && src[i] >= lowbound)
            buffer.push_back(i);
    for (i = 1; i < buffer.size(); i++)
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