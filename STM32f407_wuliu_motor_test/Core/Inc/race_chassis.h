#ifndef RACE_CHASSIS_H
#define RACE_CHASSIS_H

#include <stdint.h>

typedef struct
{
  float kp;
  float ki;
  float kd;
  float error;
  float lastError;
  float integral;
  float maxIntegral;
  float output;
  float maxOutput;
} PID;

extern PID mypid;
extern int flag;
extern uint8_t dianji_move_flag;
extern uint8_t PID_control_flag;
extern float fAcc[3], fGyro[3], fAngle[3];

void race_chassis_init(void);
void race_heat_set(uint16_t compare);

void change_A(int a_in);
void change_SNA1(int a_in, int speed);
void car_move(int v_x, int v_y, int w);
void car_move_distance_x(float x);
void car_move_distance_y(float y);
void car_move1(float x, float y, int a_in, int car_speed);
void car_move2(float x, float y, int a_in, int car_speed);
void car_move3(float x, float y, int a_in, int car_speed);
void car_move4(float x, float y, int a_in, int car_speed);
void car_move_distance(float x, float y, int a_in, int speed_in);
double car_move_delay(float distance, uint16_t a_of_car, uint16_t speed_of_car, uint8_t direct);

void PIDInit(void);
void PID_Calc(PID *pid, float reference, float feedback);
void PID_DIL(int v_x, int v_y, int w, uint32_t time, float target);
uint8_t PID_move(int v_x, int v_y, int w, uint32_t time, float target, int pid_choose);
void race_chassis_tim4_callback(void);

double shengjiang_control(int target_pos, uint16_t speed, uint8_t accel);
float shengjiang_current_pos(void);
void shengjiang_set_current_pos(float pos);
double pingtui_control(float target_pos, uint16_t speed, uint8_t accel);
float pingtui_current_pos(void);
void pingtui_set_current_pos(float pos);
void pingtui_reset(void);

void set_yuantai_Angle(float target, float time);
void set_zhuashou_Angle(float target_abs, float speed);
void set_wukuaipingtai_Angle(float target_abs, float speed);
void set_zhuashou_he(void);
void set_zhuashou_kai(void);
void set_wukuaipingtai_weizhi(int pos);
int move_all(float target_pos, uint16_t speed, uint8_t accel, float target_pos1, uint16_t speed1, uint8_t accel1, float target, float speed_of_yuntai);
int move_all1(float target_pos, uint16_t speed, uint8_t accel, float target_pos1, uint16_t speed1, uint8_t accel1, float target, float speed_of_yuntai);

#endif
