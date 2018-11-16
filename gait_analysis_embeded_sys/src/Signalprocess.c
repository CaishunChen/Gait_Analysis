#include "Signalprocess.h"
#include "config.h"
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
void findPeaks(F32 *src, int src_lenth, int *indMax, int *indMax_len, U16 distance, F32 lowbound, F32 upbound)
{
    int *sign = (int *)malloc(src_lenth * sizeof(int));

    S16 max_index = 0, min_index = 0;
    S16 i, j, k;
    *indMax_len = 0;
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
    // indMax_len = j;
    max_index = j;

    int *peaks_index = (int *)malloc(max_index * sizeof(int));
    int peaks_count = 0;
    for (i = 0; i < max_index; i++)
        if (src[indMax[i]] >= lowbound && src[indMax[i]] <= upbound)
            peaks_index[peaks_count++] = indMax[i];
    for (i = 0; i < peaks_count; i++)
        indMax[i] = peaks_index[i];
    *indMax_len = peaks_count;

    free(peaks_index);
    free(sign);
    free(flag_max_index);
    free(temp_max_index);
    free(idelete);

    // return peaks_index;
}

void findVallies(F32 *src, int src_lenth, int *indMin, int *indMin_len, U16 distance, F32 lowbound, F32 upbound)
{
    int *sign = (int *)malloc(src_lenth * sizeof(int));
    S16 max_index = 0,
        min_index = 0;
    U16 i, j, k;
    F32 diff;
    *indMin_len = 0;
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
    // indMin_len = j;
    min_index = j;

    int *vallies_index = (int *)malloc(min_index * sizeof(int));
    int vallies_count = 0;
    for (i = 0; i < min_index; i++)
        if (src[indMin[i]] >= lowbound && src[indMin[i]] <= upbound)
            vallies_index[vallies_count++] = indMin[i];
    for (i = 0; i < vallies_count; i++)
        indMin[i] = vallies_index[i];
    *indMin_len = vallies_count;

    free(vallies_index);

    free(sign);
    free(flag_max_index);
    free(temp_max_index);
    free(idelete);
}

void findNearzeros(F32 *src, int len, int *nz, int *resize,
                   U16 minnum, F32 lowbound, F32 upbound)
{
    U16 i;
    U16 nz_index = 0, count = 0;

    for (i = 0; i < len; i++)
    {
        if (src[i] <= upbound && src[i] >= lowbound)
        {
            nz[nz_index] = i;
            count++;
            nz_index++;
        }
        else
        {
            if (count < minnum)
                nz_index = nz_index - count;
            count = 0;
        }
    }
    *resize = nz_index;
}