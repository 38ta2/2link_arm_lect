#pragma once
#include <matrix.h>
int forwardKinematics2Dof(VECTOR_3D *p, double *theta);
int inverseKinematics2Dof(VECTOR_3D p, double *theta);