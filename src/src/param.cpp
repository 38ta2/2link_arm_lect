#include <param.hpp>
void getJointRange(JOINT_RANGE *joint_parameter)
{
    int i;
    for (i = 0; i < JOINT_NUM; i++)
    {
        joint_parameter[i] = joint_range[i];
    }
}
