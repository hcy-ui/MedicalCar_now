#ifndef __TIM3_PID_H
#define __TIM3_PID_H

typedef struct 
{
    float Target;
    float Actual;
    float Out;

    float Kp;
    float Ki;
    float Kd;

    float Error0;
    float Error1;
    float ErrorInt;

    float OutMax;
    float OutMin;

    float DifOut;//滤波变量


    int32_t ErrorIntMax;//最大积分（参考20000）
    int32_t ErrorIntMin;//反向最大积分

    int16_t Speed;
    
} PID_t;

void PID_Update(PID_t *p);
void PID_Sim_Update(PID_t *p);
void TIM3_PID_Init(void);
float TIM3_PID_Limit(float x, float min, float max);
void Update_Speed_By_Position(float outer_out, float error_pos, uint8_t wheel_id);
// float TIM3_PID_Locate(float Actual_Location);
// float TIM3_Calculate_Pulse(float Pulse);
float get_distance_per_pulse_cm();
float calculate_distance_cm(int pulse_count);


#endif


// #include "stm32f10x.h" // Device header
// #include "LED.h"
// #include "Buzzer.h"
// #include "OLED.h"
// #include "TIM1_Motor.h"
// #include "TIM2_4_Encoder.h"
// #include "TIM3_pid.h"
// #include "Track.h"
// #include "USART3_WabCam.h"
// #include "math.h"
// #include "RP.h"
// #include "usart2.h"
// #include "jy61p.h"
// #include "DDelay.h"

// // #include "Delay.h"
// // #include "DDelay.h"
// // #include "RP.h"
// // #include "Timer3_Blooteeth.h"
// // #include "Key.h"
// // #include "menu.h"
// // #include "MYRTC.h"
// // #include "MPU6050.h"
// // #include "MPU_EXTI.h"
// // #include "inv_mpu.h"
// // #include "Infrared.h"

// int32_t Speed_Left, Speed_Right, Location_Left, Location_Right;
// uint8_t OLED_Update_Flag = 0;
// float out_left, out_right;
// float error_pos;
// float error, track_out;
// float distance_left,distance_right;
// // int8_t menu2;

// // #define Base_Speed -30
// #define MAX_ANGLE_COMP 13
// int16_t Base_Speed;

// // 左右内环速度环
// PID_t Inner_Left = {
// 	.Kp = 2.01,
// 	.Ki = 0.10,
// 	.Kd = 0,

// 	.Target = 0,
// 	.Actual = 0,
// 	.Out = 0,

// 	.Speed = 0,

// 	.OutMax = 100,
// 	.OutMin = -100,

// 	.Error0 = 0,
// 	.Error1 = 0,

// 	.ErrorInt = 0,
// 	.ErrorIntMax = 10000,
// 	.ErrorIntMin = -10000,
// };

// PID_t Inner_Right = {
// 	.Kp = 2.01,
// 	.Ki = 0.10,
// 	.Kd = 0,

// 	.Target = 0,
// 	.Actual = 0,
// 	.Out = 0,

// 	.Speed = 0,

// 	.OutMax = 100,
// 	.OutMin = -100,

// 	.Error0 = 0,
// 	.Error1 = 0,
// 	.ErrorInt = 0,
// 	.ErrorIntMax = 10000,
// 	.ErrorIntMin = -10000,
// };



// // 外环角度环
// PID_t Angle_PID = {
// 	.Kp = 0.61,
// 	.Ki = 0,
// 	.Kd = 0.91,

// 	.Target = 0,
// 	.Actual = 0,
// 	.Out = 0,

// 	.OutMax = 100,
// 	.OutMin = -100,

// 	.Error0 = 0,
// 	.Error1 = 0,
// 	.ErrorInt = 0,
// 	.ErrorIntMax = 20000,
// 	.ErrorIntMin = -20000,
// };

// // 最外环位置环
// // 左右环位置环（外）

// PID_t Position_Control_Left = {
// 	.Kp = 0.17,
// 	.Ki = 0.09,
// 	.Kd = 0,

// 	.Target = 0,
// 	.Actual = 0,
// 	.Out = 0,

// 	.OutMax = 50,
// 	.OutMin = -50,

// 	.Error0 = 0,
// 	.Error1 = 0,
// 	.ErrorInt = 0,
// 	.ErrorIntMax = 1000,
// 	.ErrorIntMin = -1000,
// };

// PID_t Position_Control_Right = {
// 	.Kp = 0.17,
// 	.Ki = 0.09,
// 	.Kd = 0,

// 	.Target = 0,
// 	.Actual = 0,
// 	.Out = 0,

// 	.OutMax = 50,
// 	.OutMin = -50,

// 	.Error0 = 0,
// 	.Error1 = 0,
// 	.ErrorInt = 0,
// 	.ErrorIntMax = 1000,
// 	.ErrorIntMin = -1000,
// };


// int main(void)
// {
// 	usart2_Init(9600);
// 	OLED_Init();
// 	TIM1_Motor_Init();

// 	TIM2_Encoder_Init();
// 	TIM4_Encoder_Init();
// 	DDelay_s(1);
// 	TIM3_PID_Init();

// 	RP_Init();
// //	TIM1_Motor_SetSpeed(20,20);


// 	while (1)
// 	{
// 		Trck_Read();




// 		if (OLED_Update_Flag)
// 		{ /*********内环速度环******************/
// 			// OLED_Printf(0, 48, OLED_8X16, "yaw:%4.2f", Yaw);
// 			// OLED_Printf(0, 0, OLED_8X16, "Kp:%4.2f", Inner_Left.Kp);
// //			OLED_Printf(0, 16, OLED_8X16, "Ki:%4.2f", Inner_Left.Ki);
// //			OLED_Printf(0, 32, OLED_8X16, "Kd:%4.2f", Inner_Left.Kd);
// //			OLED_Printf(0, 48, OLED_8X16, "Ta:%+04.0f", Inner_Left.Target);

// 			// OLED_Printf(64, 0, OLED_8X16, "SL:%04d", Speed_Left);
// 			// OLED_Printf(64, 16, OLED_8X16, "SR:%04d", Speed_Right);
// 			// OLED_Printf(64, 32, OLED_8X16, "Out:%04.0f", (Inner_Left.Out + Inner_Right.Out) / 2);
// 			// OLED_Printf(64, 48, OLED_8X16, "Act:%04.0f", (Inner_Left.Actual + Inner_Right.Actual) / 2);
// 			/*************内环速度环**************/

// 			/*********内环角度环******************/
// 			// OLED_Printf(0, 0, OLED_8X16, "Kp:%4.2f", Angle_PID.Kp);
// 			// OLED_Printf(0, 16, OLED_8X16, "Ki:%4.2f", Angle_PID.Ki);
// 			// OLED_Printf(0, 32, OLED_8X16, "Kd:%4.2f", Angle_PID.Kd);
// 			// OLED_Printf(0, 48, OLED_8X16, "Ta:%+04.0f", Angle_PID.Target);

// 			// OLED_Printf(64, 0, OLED_8X16, "yaw:%4.2f", Yaw);

// 			// OLED_Printf(64, 0, OLED_8X16, "SL:%04d", -Speed_Left);
// 			// OLED_Printf(64, 16, OLED_8X16, "SLT:%04.0f", Inner_Left.Target);
// 			// OLED_Printf(64, 32, OLED_8X16, "SR:%04d", -Speed_Right);
// 			// OLED_Printf(64, 48, OLED_8X16, "SRT:%04.0f", Inner_Right.Target);
// 			/*************内环灰度环**************/

// 			// OLED_Printf(0, 0, OLED_8X16, "Kp:%d", Gray1);
// 			// OLED_Printf(0, 16, OLED_8X16, "Ki:%d", Gray2);
// 			// OLED_Printf(0, 32, OLED_8X16, "Kd:%d", Gray3);
// 			// OLED_Printf(0, 48, OLED_8X16, "Ta:%d", Gray4);

// 			// OLED_Printf(64, 0, OLED_8X16, "SL:%d", Gray5);
// 			// OLED_Printf(64, 16, OLED_8X16, "SLT:%d", Gray6);
// 			// OLED_Printf(64, 32, OLED_8X16, "SR:%d", Gray7);
// 			// OLED_Printf(64, 48, OLED_8X16, "SRT:%d", Gray8);

// 			/*********外环位置环******************/
// 			OLED_Printf(0, 0, OLED_8X16, "kp:%4.2f", Position_Control_Left.Kp);
// 			OLED_Printf(0, 16, OLED_8X16, "ki:%4.2f", Position_Control_Left.Ki);
// 			OLED_Printf(0, 32, OLED_8X16, "kd:%4.2f", Position_Control_Left.Kd);
// 			OLED_Printf(0, 48, OLED_8X16, "Ta:%+04.0f", Position_Control_Left.Target);

// 			OLED_Printf(64, 0, OLED_8X16, "LT:%4.2f", Position_Control_Left.Target);
// 			OLED_Printf(64, 16, OLED_8X16, "LA:%4.2f", Position_Control_Left.Actual);
// 			OLED_Printf(64, 32, OLED_8X16, "RT:%04.2f", Position_Control_Right.Target);
// 			OLED_Printf(64, 48, OLED_8X16, "RA:%04.2f", Position_Control_Right.Actual);
// 			/*********外环位置环******************/

// 			OLED_Update();

// 			OLED_Update_Flag = 0; // 清除标志位
// 		}

// //		Angle_PID.Kp = RP_GetValue(1) / 4095.0 * 5; // 内环角度控制
// //		Angle_PID.Ki = RP_GetValue(3) / 4095.0 * 2;
// //		Angle_PID.Kd = RP_GetValue(4) / 4095.0 * 2;
// 		// Angle_PID.Target = RP_GetValue(2) / 4095.0 * 180 - 90;

// //		Inner_Left.Kp = RP_GetValue(1) / 4095.0 * 5;//内环速度控制
// //		Inner_Left.Ki = RP_GetValue(3) / 4095.0 * 2;
// //		Inner_Left.Kd = RP_GetValue(4) / 4095.0 * 2;
// //		Inner_Left.Target = RP_GetValue(2) / 4095.0 * 310 - 155;

// //		Inner_Right.Kp = Inner_Left.Kp;
// //		Inner_Right.Ki = Inner_Left.Ki;
// //		Inner_Right.Kd = Inner_Left.Kd;
// //		Inner_Right.Target = Inner_Left.Target;

// 		Position_Control_Left.Kp = RP_GetValue(1) / 4095.0 * 1;//外环位置控制
// 		Position_Control_Left.Ki = RP_GetValue(3) / 4095.0 * 1;
// 		Position_Control_Left.Kd = RP_GetValue(4) / 4095.0 * 1;
// 		Position_Control_Left.Target = RP_GetValue(2) / 4095.0 * 600 - 300;

// 		Position_Control_Right.Kp = Position_Control_Left.Kp;
// 		Position_Control_Right.Ki = Position_Control_Left.Ki;
// 		Position_Control_Right.Kd = Position_Control_Left.Kd;
// 		Position_Control_Right.Target = Position_Control_Left.Target;


// 	}
// }

// void TIM3_IRQHandler(void)
// {
// 	static uint16_t Count1, Count2, Count3;
// 	static int16_t Angle_Speed = 0;
// 	if (TIM_GetITStatus(TIM3, TIM_IT_Update) == SET)
// 	{

// 		Count1++;
// 		if (Count1 >= 10) // 内环
// 		{
// 			Count1 = 0;
// 			OLED_Update_Flag = 1; // OLED刷新标志位（每10ms刷新一下OLED显示）

// 			Speed_Left = TIM4_Encoder_Get();
// 			Speed_Right = -TIM2_Encoder_Get();
// 			Location_Left -= Speed_Left;
// 			Location_Right -= Speed_Right; // 获取速度and位置

// 			/*********速度PID******************/
// 			Inner_Left.Actual = -Speed_Left;   // 速度环（左）
// 			Inner_Right.Actual = -Speed_Right; // 速度环（右）

// 			Inner_Left.Target = Base_Speed - Angle_Speed; // 左
// 			PID_Sim_Update(&Inner_Left);
// 			out_left = Inner_Left.Out;

// 			Inner_Right.Target = Base_Speed + Angle_Speed; // 右
// 			PID_Sim_Update(&Inner_Right);
// 			out_right = Inner_Right.Out;

// 			TIM1_Motor_SetSpeed(-out_left, -out_right);
			
// //			if((fabs(Position_Control_Left.Actual) >= 300 ) && ( fabs(Position_Control_Right.Actual) >= 300 ))
// //			{
// //				TIM1_Motor_SetSpeed(0, 0);
// //			}
// //			
	
// 			/*********速度PID******************/
// 		}
		
// 		Count2 ++;
// 		if (Count2 >= 20)   //外环（角度）
// 		{
// 			Count2 = 0;
// 			Angle_PID.Actual = Yaw;
// 			Angle_PID.Target = 0;
// 			PID_Sim_Update(&Angle_PID);

// 			Angle_Speed = Angle_PID.Out;
// 			if(Angle_Speed >= MAX_ANGLE_COMP)
// 			{
// 				Angle_Speed = MAX_ANGLE_COMP;
// 			}
// 			else if(Angle_Speed <= -MAX_ANGLE_COMP)
// 			{
// 				Angle_Speed = -MAX_ANGLE_COMP;
// 			}
// 		}

		
// 		Count3++;
// 		if (Count3 >= 40) // 外环（位置）
// 		{
// 			Count3 = 0;
// 			//赋值左右距离
// 			distance_left = calculate_distance_cm(Location_Left);
// 			distance_right = calculate_distance_cm(Location_Right);

// 			Position_Control_Left.Actual = distance_left;
// 			Position_Control_Right.Actual = distance_right;

// 			PID_Sim_Update(&Position_Control_Left);
// 			PID_Sim_Update(&Position_Control_Right);

// 			Base_Speed = Position_Control_Left.Out;
// 			Base_Speed = Position_Control_Right.Out;


// 		}
// 		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
	
// 	}
// }