typedef struct
{
    double min; // minimum movable range
    double max; // maximum movable range
} JOINT_RANGE;

#ifndef JOINT_NUM
#define JOINT_NUM (3)
#endif

static const JOINT_RANGE joint_range[JOINT_NUM] = {
    /*  min [rad], max[rad]  */
    {-1.570, 1.570}, // joint 1
    {-1.570, 1.570}, // joint 2
    {-1.570, 1.570}, // joint 3
};
void getJointRange(JOINT_RANGE *joint_parameter)
{
    int i;
    for (i = 0; i < JOINT_NUM; i++)
    {
        joint_parameter[i] = joint_range[i];
    }
}
