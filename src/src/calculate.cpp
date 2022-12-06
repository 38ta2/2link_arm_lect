#include <stdio.h>
#include <math.h>
#include "calculate.hpp"
#include "param.hpp"

int forwardKinematics2Dof(VECTOR_3D *p, double *theta)
{
    double L1 = 0.080;
    double L2 = 0.055;

    double C1 = cos(theta[1]);
    double C12 = cos(theta[1] + theta[3]);
    double S1 = sin(theta[1]);
    double S12 = sin(theta[1] + theta[3]);

    p->x = L1 * C1 + L2 * C12;
    p->y = L1 * S1 + L2 * S12;

    return 0;
}

int inverseKinematics2Dof(VECTOR_3D p, double *theta)
{
    double L1 = 80;
    double L2 = 55;

    double S2, C2;

    //手先位置の値が可動範囲の外であればエラー値を返す
    if (((pow(L1 - L2, 2)) > (p.x * p.x + p.y * p.y)) || ((p.x * p.x + p.y * p.y) > (pow(L1 + L2, 2))))
    {
        printf("Target position is out of range.\n");
        return 1;
    }

    // 2軸目theta[1]と4軸目theta[3]以外は0 radで固定
    for (int i = 0; i < 3; i++)
    {
        theta[i] = 0;
    }

    //θ2の角度
    C2 = (p.x * p.x + p.y * p.y - L1 * L1 - L2 * L2) / (2 * L1 * L2);
    theta[3] = -acos(C2); // acosの戻り値[0:π]、CRANE-X7においてθ2の可動はマイナス方向のみ
    //θ1の角度
    S2 = sin(theta[3]);
    theta[1] = atan2((-L2 * S2 * p.x + (L1 + L2 * C2) * p.y), ((L1 + L2 * C2) * p.x + L2 * S2 * p.y));

    //得られた関節角度が可動範囲外であればエラーを返す
    for (int i = 0; i < 3; i++)
    {
        if (((joint_range[i].min) > (theta[i])) || ((theta[i]) > (joint_range[i].max)))
        {
            printf("Theta[ %d]　is out of range.\n", i);
            return 1;
        }
    }
    return 0;
}
