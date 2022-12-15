/**
 * @file caluclate.cpp
 * @brief 逆運動学演算ライブラリ、リンク長さ固定
 * @author 38ta2
 * @date 14Des2022
 * @details 指定された位置ベクトルで逆運動学をといてtheta1,3を出力
 */

#include <stdio.h>
#include <math.h>
#include "calculate.hpp"
#include "param.hpp"
/**
 * @fn int forwardKinematics2Dof
 * @brief 順運動学ライブラリ
 * @param [in] theta 入力角度行列
 * @return VECTOR_3D 入力した位置ベクトルの値
 * @details 現在の角度を入力して現在地ベクトルを出力する
 */
int forwardKinematics2Dof(VECTOR_3D *p, double *theta)
{
    //! 制作したリンクの長さ
    double L1 = 0.080;
    double L2 = 0.055;

    //! 三角関数の定義
    double C1 = cos(theta[1]);
    double C12 = cos(theta[1] + theta[3]);
    double S1 = sin(theta[1]);
    double S12 = sin(theta[1] + theta[3]);

    //! 座標の計算法
    p->x = L1 * C1 + L2 * C12;
    p->y = L1 * S1 + L2 * S12;

    return 0;
}

/**
 * @fn int inverseKinematics2Dof
 * @brief 逆運動学
 * @param [in] VECTOR_3D 目標位置をベクトル指定
 * @return theta 目標角度
 * @details lib 逆運動学をといて目標姿勢となる角度を割り出す
 */
int inverseKinematics2Dof(VECTOR_3D p, double *theta)
{
    double L1 = 0.080;
    double L2 = 0.055;

    double S2, C2;

    // ！ 手先位置の値が可動範囲の外であればエラー値を返す
    if (((pow(L1 - L2, 2)) > (p.x * p.x + p.y * p.y)) || ((p.x * p.x + p.y * p.y) > (pow(L1 + L2, 2))))
    {
        printf("Target position is out of range.\n");
        return 1;
    }

    //! 2軸目theta[1]と4軸目theta[3]以外は0 radで固定
    for (int i = 0; i < 3; i++)
    {
        theta[i] = 0;
    }

    //! θ2の角度
    C2 = (p.x * p.x + p.y * p.y - L1 * L1 - L2 * L2) / (2 * L1 * L2);
    // !acosの戻り値[0:π]、CRANE-X7においてθ2の可動はマイナス方向のみ
    theta[3] = -acos(C2);
    // ！ θ1の角度
    S2 = sin(theta[3]);
    theta[1] = atan2((-L2 * S2 * p.x + (L1 + L2 * C2) * p.y), ((L1 + L2 * C2) * p.x + L2 * S2 * p.y));

    // ！ 得られた関節角度が可動範囲外であればエラーを返す
    for (int i = 0; i < 3; i++)
    {
        int r = i - 1;
        // printf("%d is i_limit", i);
        if (((joint_range[i].min) > (theta[r])) || ((theta[r]) > (joint_range[i].max)))
        {
            printf("Theta %d is out of range. \n", i);
            return 1;
        }
    }
    return 0;
}
