#include <pololu/orangutan.h>
#include "menu.h"
#include "main.h"


/*
 * Giovanni Galasso
 * SENG5831 Spring 2015
 * Lab2
 *
 * All write up info can be found in the README_Lab2 file.
 *
 *
 *
 * http://www.pololu.com/docs/0J20
 * http://www.pololu.com
 * http://forum.pololu.com
 */

//Not necessary but rotation count is 2249 encoder counts. If we were converting
//to degrees we would have used this.
//int ROTATION_COUNT = 2249;

//Last encoder value
volatile long global_last_enc_value = 0;
//Current velocity
volatile long global_cur_velo;
//Comp A ISR counter
volatile int global_compa_counter = 0;
volatile int global_m1a;
volatile int global_m1b;
//Global encoder counts for motor
volatile long global_counts_m1;
volatile int global_error_m1;
volatile int global_last_m1a_val;
volatile int global_last_m1b_val;


#define BIT_M1_ENCA (1 << 2)
#define BIT_M1_ENCB (1 << 3)

#define GET_M1_ENCA ((PIND & BIT_M1_ENCA) > 0)?1:0
#define GET_M1_ENCB ((PIND & BIT_M1_ENCB) > 0)?1:0


//TABLE 14.3.4

//PWM1 PD7 PCINT31 OC2A
//DIR1 PC7 PCINT23 TOSC2  -- c is output
//M1ENCA PD2 PCINT26
//M1ENCB PD3 PCINT27
//PWM2 PD6 PCINT30 OC2B
//DIR2 PC6 PCINT22 TOSC1 -- c is output
//M2ENCA PD0 PCINT24
//M2ENCB PD1 PCINT25
//PCICR: 0x08 -> PCIE3
//PCIFR -> flag register


//For positional Kp = .25, Ki = .0077, Kd = 3.1 and time period = 1
//For speed use: Kp = .4, Ki = .05, Kd = 2 and time_period = 1

volatile float Kp = .25; //Kp = Proportional gain
volatile float Ki = .0077; //Ki = Integral gain
volatile float Kd = 3.1; //Derivative gain
volatile float D; //Current position - last position
volatile float I; //Sum of errors
volatile long P; //Reference position - measured position
volatile long last_P = 0; //Last value of P
volatile int desiredV = 0; //Desired velocity
volatile float time_period = 1; //Time period or dt value
volatile int Torq; //Torq to give the motor
volatile int myMotorSpeed = 0; //Motor speed variable for easy access
volatile long target_position = 0; //Target position

//Determines if after a logging command has been given to log for speed/position
volatile int spe_cm = 0;
volatile int pos_cm = 0;
volatile int log_cm = 0;

//Arrays for logging
volatile long log_pr[500];
volatile long log_pm[500];
volatile long log_tj[500];
volatile long log_p[500];
volatile float log_d[500];
volatile float log_i[500];
volatile long log_t[500];
volatile int logging_index = 0;

//Variables for type of request
volatile int speedRequest = 0;
volatile int positionRequest = 0;
volatile int interpolatorRequest = 0;
//Settle counts checks if the target is near the current position and increments if so
volatile int settle_counts = 0;
//Determines when to set the next target position for the interpolator
static int interpTarget = 0;


int main(void)
{
    init_menu();	// this is initialization of serial comm through USB
    clear();	// clear the LCD
    
    lcd_init_printf();
    //Intialize PWM for motors
    enable_pwm_m1();
    //Intialize Compa ISR for 10 ms polling
    enable_compa_isr();
    //Initalize the encoder counting
    init_encoders();

    while(1) {
        
        
        if(positionRequest == 1 || interpolatorRequest == 1) {
            lcd_goto_xy(0,0);
            printf("T:%02ld", target_position);
            lcd_goto_xy(0,1);
            printf("C:%02ld", global_counts_m1);
        } else if(speedRequest == 1) {
            lcd_goto_xy(0,0);
            printf("S:%03ld", global_cur_velo);
            lcd_goto_xy(0,1);
            printf("DS:%03d E:%3ld", desiredV, desiredV - global_cur_velo);
        }
        //Check for serial input
        serial_check();
        check_for_new_bytes_received();

        //If the logging index is full then print to screen and reset logging commands
        if(logging_index >= 500) {
            char tempBuffer[60];
            int length;
            length = sprintf( tempBuffer, "Logging Printing\r\n");
            print_usb( tempBuffer, length );
            length = sprintf( tempBuffer, "Kp:%f,Ki:%f,Kd:%f\r\n", Kp, Ki, Kd);
            print_usb( tempBuffer, length );
            for (int in = 0; in < 500; in++) {
                length = sprintf( tempBuffer, "%6ld,%6ld,% 3ld,%6ld,%.3f,%.3f,%3ld\r\n", log_pr[in], log_pm[in], log_tj[in],log_p[in], log_i[in], log_d[in], log_t[in]);
                print_usb( tempBuffer, length );
            }
            logging_index = 0;
            spe_cm = 0;
            pos_cm = 0;
            log_cm = 0;
        }
    }
}

//Check if the motor has settled on a position
void check_settled(void) {
    long topRange = target_position + 6;
    long botRange = target_position - 6;
    if(global_counts_m1 <= topRange && global_counts_m1 >= botRange) {
        settle_counts++;
    }
}

void interpolator(void) {
    //Use if you want interpolator defaults.
    //Kp = .25; Ki = .0077; Kd = 3.1;

    if(interpolatorRequest == 1) {
        if(interpTarget == 0) {
            interpTarget = 1;
            target_position = global_counts_m1 + 562;
        }
        if(settle_counts >= 50) {
            interpolatorRequest = 2;
            interpTarget = 0;
            settle_counts = 0;
           
        }
    }
    if(interpolatorRequest == 2) {
        if(interpTarget == 0) {
            interpTarget = 1;
            target_position = global_counts_m1 - 2249;
        }
        if(settle_counts >= 50) {
            interpolatorRequest = 3;
            interpTarget = 0;
            settle_counts = 0;
           
        }
    }
    if(interpolatorRequest == 3) {
        if(interpTarget == 0) {
            interpTarget = 1;
            target_position = global_counts_m1 + 31;
        }
        if(settle_counts >=50) {
            interpolatorRequest = 0;
            interpTarget = 0;
            settle_counts = 0;
        }
    }
    pid_position();

}


void set_motor_speed(int my_speed) {
    if(my_speed < 0) {
        //set_direction(0);
        PORTC |= (1 << PORTC7);
    } else {
        //set_direction(1);
        PORTC &= ~(1 << PORTC7);
    }
    if (my_speed < -255) {
        my_speed = -255;
    } else if(my_speed > 255) {
        my_speed = 255;
    }
    OCR2A = abs(my_speed);
    myMotorSpeed = my_speed;
}


ISR(TIMER1_COMPA_vect) {
    //Increase the global number of TIMER1_COMPA counts
    global_compa_counter++;
    //Calculate velocity by determining the number of encoder counts since the last poll
    global_cur_velo = global_counts_m1 - global_last_enc_value;

    //Comment out below to adjust controller frequency updates
    //if (global_compa_counter % 20 == 0) {
        if(positionRequest == 1) {
            pid_position();
        } else if (speedRequest == 1) {
            pid_speed();
        } else if (interpolatorRequest > 0) {
            interpolator();
            //pid_position();
        }
        //Save current error as the last error
        last_P = P;
        set_motor_speed(myMotorSpeed);
        //Set this to lower the rate of logging
        //if(global_compa_counter % 33 == 0) {
            if (log_cm == 1 && (spe_cm == 1 || pos_cm == 1) && logging_index < 500) {
                if(spe_cm == 1) {
                    log_pr[logging_index] = desiredV;
                    log_pm[logging_index] = global_cur_velo;
                } else {
                    log_pr[logging_index] = target_position;
                    log_pm[logging_index] = global_counts_m1;
                }
                log_tj[logging_index] = myMotorSpeed;
                log_p[logging_index] = P;
                log_i[logging_index] = Ki * I;
                log_d[logging_index] = Kd * D;
                log_t[logging_index] = Torq;
                logging_index++;
            }
        //}

    //Comment out below to adjust controller frequency updates
    //}
    
    //Save the current encoders counts as the last encoder count value for the next poll
    global_last_enc_value = global_counts_m1;
    
}


void pid_speed(void) {
    //Speed Error == desired encoder counts per 10ms and cur encoder counts per 10ms
    P = desiredV - global_cur_velo;
    //Integral
    I += (P * time_period);
    if (I > 1000) {I = 1000;} else if (I < -1000) {I = -1000;}
    //Derivative
    D = (P - last_P) / time_period;
    //Torq
    Torq = (int)((Kp * P) + (Ki * I) + (Kd * D));
    //Set motor speed to it's last value + torque
    myMotorSpeed += Torq;
}

void pid_position(void) {
    //Position Error -- target position minus the current position
    P = target_position - global_counts_m1;
    //Integral
    I += (P * time_period);
    if (I > 1000) {I = 1000;} else if (I < -1000) {I = -1000;}
    //Derivative
    D = (P - last_P) / time_period;
    //Torq
    Torq = (int)((Kp * P) + (Ki * I) + (Kd * D));
    //Set motor speed to it's last value + torque
    myMotorSpeed = Torq;
    check_settled();
}





void enable_pwm_m1(void) {
    //Data direction
    DDRD |= (1 << PORTD7);
    //Motor direction
    DDRC |= (1 << PORTC7);
    //DDD7 set(one);
    
    //PORTC7 = 1;
    
    //Fast PWM - TOP =OCRA, Update OCRx at BOTTOM, TOV FLag Set on TOP
    //Mode 7, WGM2 = 1, WGM1 = 1, WGM0 = 1
    //CLEAR OC21 on Compare Match, set OC2A at BOTTOM (non-inverting mode). Table 17-3 Page 151
    //Page 151
    TCCR2A = (1 << COM2A1) | (0 << COM2A0) | (0 << COM2B1) | (0 << COM2B0) | (0 << 3) | (0 << 2) | (1 << WGM21) | (1 << WGM20); //0x83;
    
    // use the system clock/8 (=2.5 MHz) as the timer clock,
    // which will produce a PWM frequency of 10 kHz
    //Page 153
    TCCR2B = (0 << FOC2A) | (0 << FOC2B) | (0x0 << 5) | (0x0 << 4) | (0 << WGM22) |  (0 << CS22) | (1 << CS21) | (0 << CS20); //0x02;
    //
    //TIMSK2 = (0x0 << 7) | (0x0 << 6) | (0x0 << 5) | (0x0 << 4) | (0x0 << 3) | (0 << OCIE2B) | (1 << OCIE2A) | (0 << TOIE2); //0x02;
    
    //Determines the motor speed
    OCR2A = 0;
}


//Wrote my own function to determine encoder counts.
ISR(PCINT3_vect)
{
    unsigned int m1a_val = GET_M1_ENCA;
    unsigned int m1b_val = GET_M1_ENCB;
    
    if (global_last_m1b_val == m1a_val) {
        global_counts_m1++;
    } else if (global_last_m1a_val == m1b_val){
        global_counts_m1--;
    }
    global_last_m1a_val = m1a_val;
    global_last_m1b_val = m1b_val;
}

//Intialize encoder interrupts.
void init_encoders(void) {
    PCMSK3 = (0 << PCINT31) | (0 << PCINT30) | (0 << PCINT29) | (0 << PCINT28) | (1 << PCINT27) | (1 << PCINT26) | (1 << PCINT25) | (1 << PCINT24);
    PCICR = 0xFF;
    PCIFR = 0xFF;
    global_last_m1a_val = GET_M1_ENCA;
    global_last_m1b_val = GET_M1_ENCB;
    sei();
}

//Enable comp A ISR
void enable_compa_isr(void) {
    //
    TCCR1A = (1 << COM1A1) | (0 << COM1A0) | (0 << COM1B1) | (0 << COM1B0) | (0 << 3) | (0 << 2) | (0 << WGM11) | (0 << WGM10);
    //
    TCCR1B = (0 << ICNC1) | (0 << ICES1) | (0x0 << 5) | (0 << WGM13) | (1 << WGM12) |  (0 << CS12) | (1 << CS11) | (0 << CS10);
    //Polls every 10ms
    OCR1A = 25000;
    //
    TIMSK1 = (0x0 << 7) | (0x0 << 6) | (0 << ICIE1) | (0x0 << 4) | (0x0 << 3) | (0 << OCIE1B) | (1 << OCIE1A) | (0 << TOIE1);
}

//Reset PID values
void resetPID(void) {
    I = 0;
    P = 0;
    D = 0;
    last_P = 0;
}

void setPositionDefaults(void) {
    //For positional Kp = .25, Ki = .01, Kd = 4 and time period = 1
    Kp = .25;
    Ki = .0077;
    Kd = 3.1;
}

void setSpeedDefaults(void) {
    //For speed use: Kp = .4, Ki = .05, Kd = 2 and time_period = 1}
    Kp = .4;
    Ki = .05;
    Kd = 2;
}


