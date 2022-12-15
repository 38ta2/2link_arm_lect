/**
 * @file caluculate.h
 * @brief 角度の制限範囲
 * @author 38ta2
 * @date 14Des2022
 * @details 角度を指定している変数をアクセス可能変数に変える
 */
#pragma once
#include <matrix.h>
int forwardKinematics2Dof(VECTOR_3D *p, double *theta);
int inverseKinematics2Dof(VECTOR_3D p, double *theta);