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
void init_encoders(void);
void enable_compa_isr(void);

void set_direction(int);
void set_motor_speed(int);

void resetPID(void);
void pid_speed(void);
void pid_position(void);

void interpolator(void);
void check_settled(void);
#endif
