#ifndef SERVO_H
#define SERVO_H

#include <stdint.h>

#define SERVO_LEFT   0U
#define SERVO_RIGHT  1U

/* Direction as seen from the robot: +1 forward, -1 reverse, 0 stop. */
void     Servo_Init(void);
void     Servo_SetDirection(uint8_t servo, int8_t dir);
void     Servo_Update(uint16_t target_rpm);   /* call every 100 ms */
uint16_t Servo_GetPulseUs(uint8_t servo);     /* pulse being sent now */
uint16_t Servo_GetRPM(uint8_t servo);         /* last measured RPM */

/* Called from the TIM1 capture ISR in main.c (one call per falling edge).
   rise / prev_rise = last two rising-edge timestamps, fall = falling edge. */
void     Servo_ProcessCapture(uint8_t servo, uint16_t rise,
                              uint16_t fall, uint16_t prev_rise);

#endif