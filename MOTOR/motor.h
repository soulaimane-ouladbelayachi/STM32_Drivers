/*
 * motor.h
 *
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_


typedef enum MotorNum
{
    MOTOR_FRONT_LEFT = TIM_CHANNEL_3,  // PWM4 -- PA8
    MOTOR_FRONT_RIGHT = TIM_CHANNEL_4, // PWM3 -- PA9
    MOTOR_BACK_LEFT = TIM_CHANNEL_2,   // PWM2 -- PA10
    MOTOR_BACK_RIGHT = TIM_CHANNEL_1,  // PWM1 -- PA11
} MotorNum;

void motorsStart();
void motorsStop();
void motorsDeinit();
void setMotor(MotorNum motor, uint32_t val);
void motorsInit(void);


#endif /* INC_MOTOR_H_ */
