/**
 * @file param.h
 * @brief 角度の制限範囲を指定
 * @author 38ta2
 * @date 14Des2022
 * @details 角度を指定している
 */
#pragma once
typedef struct
{
    //! minimum movable range
    double min;
    //! maximum movable range
    double max;
} JOINT_RANGE;

#ifndef JOINT_NUM
#define JOINT_NUM (3)
#endif

static const JOINT_RANGE joint_range[JOINT_NUM] = {
    /*  min [rad], max[rad]  */
    {-2.740, 2.740}, // joint 1
    {0.000, 3.141},  // joint 2
    {-2.740, 2.740}, // joint 3
};
void getJointRange(JOINT_RANGE *joint_parameter);