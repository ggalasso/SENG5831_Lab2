//
//  main.h
//  lab2_seng
//
//  Created by Giovanni Galasso on 4/4/15.
//  Copyright (c) 2015 Giovanni Galasso. All rights reserved.
//

#ifndef lab2_seng_main_h
#define lab2_seng_main_h

void enable_pwm_m1(void);
void set_direction(int);
void set_motor_speed(int);
void init_encoders(void);
void enable_compa_isr(void);
int degrees_to_revs(int);
int get_P(void);
float get_D(void);
float get_I(void);
void set_P(int desired, int current);
void set_D(int);
void set_I(int);
void set_Kp(float);
void set_Kd(float);
void set_Ki(float);
int get_motor_speed(void);
float get_Kp(void);
float get_Ki(void);
float get_Kd(void);
int get_V(void);
int get_Torq(void);

#endif
