/**
 * @file param.h
 * @brief 角度の制限範囲
 * @author 38ta2
 * @date 14Des2022
 * @details 角度を指定している変数をアクセス可能変数に変える
 */
#include <param.hpp>

/**
 * @fn void getJointRange
 * @brief 定数保護
 * @param [in] JOINT_RANGE 制限角度定数
 * @return joint_parameter 計算に用いる制限角度
 * @details 制限角度を入力してアクセス用の制限角度を出力する
 */
void getJointRange(JOINT_RANGE *joint_parameter)
{
    int i;
    for (i = 0; i < JOINT_NUM; i++)
    {
        joint_parameter[i] = joint_range[i];
    }
}
