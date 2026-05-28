#include "filter.h"

#include <stdlib.h>
#include <stdint.h>
#include <string.h>

/**
 * @brief 对数据进行限幅操作，将数据限制在指定的最小值和最大值之间。
 *
 * @param data 待处理的数组指针。
 * @param size 数组的大小。
 * @param min_val 最小允许值。
 * @param max_val 最大允许值。
 */
void limit_value(uint32_t *data, int size, uint32_t min_val, uint32_t max_val) 
{
    for (int i = 0; i < size; i++) 
    {
        if (data[i] < min_val) {
            data[i] = min_val;
        } else if (data[i] > max_val) 
        {
            data[i] = max_val;
        }
    }
}

/**
 * @brief 用于qsort函数的比较函数，比较两个uint32_t类型的值。
 *
 * @param a 第一个比较的元素。
 * @param b 第二个比较的元素。
 * @return int 返回值小于0表示a小于b，返回值大于0表示a大于b，返回值等于0表示a等于b。
 */
int compare(const void *a, const void *b) 
{
    return (*(uint32_t *)a - *(uint32_t *)b);
}

/**
 * @brief 计算数组中数据的中位数。
 *
 * @param data 待处理的数组指针。
 * @param size 数组的大小。
 * @return uint32_t 返回计算出的中位数。
 */
uint32_t mid_value(uint32_t *data, int size) 
{
    uint32_t *copy = (uint32_t *)malloc(size * sizeof(uint32_t));
    memcpy(copy, data, size * sizeof(uint32_t));
    qsort(copy, size, sizeof(uint32_t), compare);
    uint32_t median = (size % 2 == 0) ? (copy[size / 2 - 1] + copy[size / 2]) / 2 : copy[size / 2];
    free(copy);
    return median;
}

/**
 * @brief 计算数组中数据的算术平均值。
 *
 * @param data 待处理的数组指针。
 * @param size 数组的大小。
 * @return uint32_t 返回计算出的算术平均值。
 */
uint32_t avg_value(uint32_t *data, int size) 
{
    uint32_t sum = 0;
    for (int i = 0; i < size; i++) {
        sum += data[i];
    }
    return sum / size;
}

/**
 * @brief 一阶滞后滤波器，用于平滑输入数据。
 *
 * @param CurrValue 当前输入值。
 * @return uint32_t 返回滤波后的值。
 */
uint32_t adc_filter(uint32_t CurrValue) 
{
    static uint32_t LastValue = 0;  // 滤波器上一次的输出值，初始为0
    uint32_t tmp = (32 * CurrValue + 96 * LastValue) >> 7;
    LastValue = tmp;
    return LastValue;
}
