/**
 * @file main.cpp
 * @brief メインプログラムarduinoUNO
 * @author 38ta2
 * @date 14Des2022
 * @details loop内に記述target_posの値で動く
 */
#include <Arduino.h>
#include <stdio.h>
#include <unistd.h>
#include <Servo.h>
#include "matrix.h"
#include "calculate.hpp"
#include "param.hpp"

// ！ Servoオブジェクトの宣言
Servo myservo1, myservo2, myservo3;

/**
 * @fn void setup
 * @brief サーボ初期設定
 * @details サーボのピン割とパルス、初期値設定
 */
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
/**
 * @fn void loop
 * @brief メインloop
 * @details Arduinoのメイン関数
 */
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

    int state = 0; // 目標位置を切り替えるための変数
    int cnt = 0;   // ループのカウント

    target_pos = target_pos4;
    // main関数のループ
    for (cnt = 1; cnt <= 10; cnt++)
    { // 10回繰り返したらプログラムを終了する
        cnt += 1;
        double t_int;
        double t_now;
        double t;
        double s = 0;

        t_int = millis();
        t_now = millis();

        while (s < 1)
        {
            t_now = millis();
            t = t_now - t_int;
            s = t / 3000;
            current_pos.x = target_pos.x * s + present_pos.x * (1 - s);
            current_pos.y = target_pos.y * s + present_pos.y * (1 - s);
            current_pos.z = target_pos.z * s + present_pos.z * (1 - s);
            if (inverseKinematics2Dof(current_pos, target_theta))
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
        }

        present_theta[JOINT_NUM] = target_theta[JOINT_NUM]; // 現在角度を格納する変数
        //   double present_current[JOINT_NUM] = {1.7, 1.7, 1.7};       //現在トルクを格納する変数
        //   getCranex7JointState(present_theta, present_angvel, present_current);
        forwardKinematics2Dof(&present_pos, present_theta);
        //   printf("Target position [x y]:[%lf %lf]\n", target_pos.x, target_pos.y);
        //   printf("Present position [x y]:[%lf %lf]\n", present_pos.x, present_pos.y);

        // target_angleを変更
        if (state == 0)
        {
            state = 1;
            target_pos = target_pos2;
        }

        else
        {
            state = 0;
            target_pos = target_pos1;
        }
    } // end main while

    // brakeCranex7Joint(); // CRANE X7をブレーキにして終了
    //  closeCranex7Port();  //シリアルポートを閉じる
    // return 0;
}
