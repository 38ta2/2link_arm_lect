#include <Arduino.h>
#include <stdio.h>
#include <unistd.h>
#include <Servo.h>
#include "matrix.h"
#include "calculate.hpp"
#include "param.hpp"

Servo myservo1, myservo2, myservo3; // Servoオブジェクトの宣言
void setup()
{
    myservo1.attach(6, 500, 2400); // サーボの割当(パルス幅500~2400msに指定)
    myservo2.attach(5, 500, 2400); // サーボの割当(パルス幅500~2400msに指定)
    myservo3.attach(9, 500, 2400); // サーボの割当(パルス幅500~2400msに指定)
    // printf("2");
    myservo1.write(90);
    myservo2.write(90);
    myservo3.write(90);
    delay(1000);
}
void loop()
{

    // uint8_t operating_mode[JOINT_NUM] = {POSITION_CONTROL_MODE, POSITION_CONTROL_MODE, POSITION_CONTROL_MODE, POSITION_CONTROL_MODE, POSITION_CONTROL_MODE, POSITION_CONTROL_MODE, POSITION_CONTROL_MODE, POSITION_CONTROL_MODE};
    VECTOR_3D target_pos = {0};                // 目標位置格納用の変数
    VECTOR_3D target_pos1 = {0.090, 0.050, 0}; // 目標位置1
    VECTOR_3D target_pos2 = {0.090, 0.090, 0}; // 目標位置2
    VECTOR_3D target_pos3 = {0.120, 0.090, 0}; // 目標位置3
    VECTOR_3D target_pos4 = {0.120, 0.050, 0}; // 目標位置4
    VECTOR_3D present_pos = {0};               // 現在位置格納用の変数
    VECTOR_3D current_pos = {0};

    double target_theta[JOINT_NUM] = {0};  // 目標角度格納用の変数
    double present_theta[JOINT_NUM] = {0}; // 現在角度を格納する変数
    double rad[JOINT_NUM] = {0};

    // double present_angvel[JOINT_NUM] = {0};  //現在速度を格納する変数
    // double present_current[JOINT_NUM] = {0}; //現在トルクを格納する変数

    int state = 0; // 目標位置を切り替えるための変数
    int cnt = 0;   // ループのカウント

    // printf("Press any key to start (or press q to quit)\n");
    // if (getchar() == ('q'))
    // return 0;

    // 運動学ライブラリの初期化
    // initParam();

    // サーボ関連の設定の初期化
    /// if (initilizeCranex7(operating_mode))
    /// {
    ///    return 1;
    ///}
    // CRANE-X7のトルクON
    // setCranex7TorqueEnable(TORQUE_ENABLE);

    // target_pos = target_pos1;

    target_pos = target_pos4;
    // main関数のループ
    for (cnt = 1; cnt <= 10; cnt++)
    { // 10回繰り返したらプログラムを終了する
        cnt += 1;

        if (inverseKinematics2Dof(target_pos, target_theta))
        {
            break;
        }
        // setCranex7Angle(target_theta);
        for (int i = 1; i < 4; i++)
        {
            rad[i] = target_theta[i] * (180 / M_PI);
            rad[i] += 90;
            rad[i] = abs(rad[i]);
        }
        myservo1.write(rad[1]);
        myservo3.write(rad[3]);

        delay(300);

        if (state == 0)
        {
            state = 1;
            target_pos = target_pos2;
        }
        else if (state == 1)
        {
            state = 2;
            target_pos = target_pos3;
        }
        else if (state == 2)
        {
            state = 3;
            target_pos = target_pos4;
        }
        else
        {
            state = 0;
            target_pos = target_pos1;
        }
    }
}