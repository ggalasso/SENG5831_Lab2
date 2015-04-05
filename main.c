#include <pololu/orangutan.h>
#include "menu.h"
#include "main.h"
//#include "../OrangutanDigital/OrangutanDigital.h"       // digital I/O routines
//#include "../OrangutanResources/include/OrangutanModel.h"

/*
 * Giovanni Galasso
 * SENG5831 Spring 2015
 * HW3
 *
 * I had some issues getting the encoder counts to print correctly. It works fine
 * using the function below, up until the int's top number. So I tried using a bigger
 * variable and resetting the counts every time we checked for them, but going in reverse
 * came up with odd results. 
 * http://www.pololu.com/docs/0J20
 * http://www.pololu.com
 * http://forum.pololu.com
 */

int ROTATION_COUNT = 2249;

volatile long global_last_enc_value = 0;
volatile long global_cur_velo;

volatile int global_compa_counter = 0;

volatile int global_m1a;
//static char global_m2a;
volatile int global_m1b;
//static char global_m2b;

volatile long global_counts_m1;
//static int global_counts_m2;

volatile int global_error_m1;
//static char global_error_m2;

volatile int global_last_m1a_val;
//static char global_last_m2a_val;
volatile int global_last_m1b_val;
//static char global_last_m2b_val;


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

/*
DDRC = 0xB0;
DDRD &= ~(BIT_M1A | BIT_M1B | BIT_M2A | BIT_M2B);
PCMSK3 = (0 << PCINT31) | (0 << PCINT30) | (0 << PCINT29) | (0 << PCINT28) | (1 << PCINT27) | (1 << PCINT26) | (1 << PCINT25) | (1 << PCINT24);
 */
volatile int Pm; //Pm = measured value
volatile int Pr; //Desired value (motor or speed for us)
volatile float Kp = .525; //Kp = Proportional gain
volatile float Ki = 0; //Ki = Integral gain
volatile float Kd = 0; //Derivative gain
volatile float D; //Current position - last position
volatile float I; //Sum of errors
volatile int P; //Reference position - measured position
volatile int last_P = 0;
volatile int V;
volatile int desiredV = 0;
volatile int posErr; //cur position - desired position
volatile float time_period = .01;
volatile int Torq;
volatile int myMotorSpeed = 0;

int main(void)
{
    init_menu();	// this is initialization of serial comm through USB
    clear();	// clear the LCD
    
    lcd_init_printf();
    enable_pwm_m1();
    enable_compa_isr();
    
    //set_motor_speed(25);

    init_encoders();

    
    //T = Kp(Pr - Pm) + Ki(SUM over time (Pr-Pm)*dt) - Kd( delta(Pr-Pm) / dt )
    //T = Output motor signal (torque)
    //Pr = Desired value (motor or speed for us)
    //Pm = Measured value
    //Kp = Proportional gain
    //Ki = Integral gain
    //Kd = Derivative gain
    
    //Note that when controlling position, the derivative term at time i becomes:
    //delta( Pr - Pm ) / dt  = ( Pr - Pm(i) ) - ( Pr - Pm(i-1) ) / dt = ( Pr - Pr - Pm(i) + Pm(i-1) ) / dt = -Velocity
    //int Pr = 15;
    //int Pm;

    while(1) {
        lcd_goto_xy(0,0);
        //printf("C:%ld", global_counts_m1);
        //printf("I: %d", get_I());
        
        printf("I:% .2fD:% 04f", get_I(), D);
        //printf("D:%d E:%d", desired, get_P());
        //printf("D=% 06.1f P=%03d", D, P);
        
        
        //printf("P: %d", get_P());
        lcd_goto_xy(0, 1);
        printf("Velo:%ld P:% 03d", global_cur_velo, P);
        //printf("lP=%03d", last_P);
        
        
        
        
        serial_check();
        check_for_new_bytes_received();

    }
}


void set_direction(int direction) {
    if(direction ==  1) {
        PORTC &= ~(0 << PORTC7);
        //lcd_goto_xy(0,0);
        //print("Clockwise");
    } else if (direction == 0) {
        //lcd_goto_xy(0,0);
        //print("Counter Clockwise");
        PORTC |= (1 << PORTC7);
    }
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

    
    
    
    /*if(abs(my_speed) <= 255) {
        OCR2A = abs(my_speed);
        //lcd_goto_xy(0,1);
        //printf("Speed:%d", my_speed);
    } else {
        if(my_speed < 0) {my_speed = -255;} else {my_speed = 255;}
        OCR2A = 255;
        //lcd_goto_xy(0,1);
        //printf("Speed:%d", my_speed);
    }*/
    
    
    
}

int get_motor_speed() {
    return OCR2A;
}


ISR(TIMER1_COMPA_vect) {
    //Increase the global number of TIMER1_COMPA counts
    global_compa_counter++;
    //Calculate velocity by determining the number of encoder counts since the last poll
    global_cur_velo = global_counts_m1 - global_last_enc_value;
    V = global_counts_m1 - global_last_enc_value;
    
    //PID STUFF
    //Set D value: cur_position - measured_position
    //set_D(global_counts_m1);
    //Set P value: desired positon - measured_position
    //set_P(desired, global_cur_velo);
    P = desiredV - global_cur_velo;
    set_I(P);
    
    D = (P - last_P) / time_period;
    //D = (last_P - P);
    //- (int)(Kd * D)
    Torq = (int)((Kp * P) + (Ki * get_I()) - (Kd * D));
    //int my_speed = OCR2A;
    myMotorSpeed += Torq;
    
    set_motor_speed(myMotorSpeed);
    
    //Save the current encoders counts as the last encoder count value for the next poll
    global_last_enc_value = global_counts_m1;
    //Save current error as the last error
    last_P = P;
    
}


int get_Torq(void) {
    return Torq;
}

int get_desiredV(void) {
    return desiredV;
}

int get_V(void) {
    return V;
}

int get_P(void) {
    return P;
}

void set_P(int desired, int current) {
    P = desired - current;
}


float get_I(void) {
    return I;
}

void set_I(int error) {
    int max = 1000;
    int min = -1000;
    //I += get_I() + get_P();
    if(I < max && I > min) {
        I +=  error * time_period;
    } else if(I > max) {
        I = max;
    } else if (I < min) {
        I = min;
    }
}

float get_D(void) {
    return D;
}

void set_D(int cur_position) {
}

int get_Pm(void) {
    return Pm;
}

int get_Pr(void) {
    return Pr;
}


float get_Kp(void) {
    return Kp;
}

float get_Kd(void) {
    return Kd;
}

float get_Ki(void) {
    return Ki;
}


void set_Pm(int Pm_val) {
    Pm = Pm_val;
}

void set_Pr(int Pr_val) {
    Pr = Pr_val;
}

void set_Kp(float Kp_val) {
    Kp = Kp_val;
}

void set_Kd(float Kd_val) {
    Kd = Kd_val;
}

void set_Ki(float Ki_val) {
    Ki = Ki_val;
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


void init_encoders(void) {
    //DDRD &= ~(PIND3 | PIND2);
    //DDRD = (1 << DDD7) | (0 << DDD6) | (0 << DDD5) | (0 << DDD4) | (0 << DDD3) | (0 << DDD2) | (0 << DDD1) | (0 << DDD0);
    //DDRD &= ~(1 << PORTD3) | (1 << PORTD2);
    //See 13.2.6 for PCINT31 - PCINT24
    PCMSK3 = (0 << PCINT31) | (0 << PCINT30) | (0 << PCINT29) | (0 << PCINT28) | (1 << PCINT27) | (1 << PCINT26) | (1 << PCINT25) | (1 << PCINT24);
    //Table 13.2.4 page 69
    //PCICR = (0x0 << 7) | (0x0 << 6) | (0x0 << 5) | (0x0 << 4) | (1 << PCIE3) | (0x0 << PCIE2) | (0x0 << PCIE1) | (0x0 << PCIE0);
    PCICR = 0xFF;
    PCIFR = 0xFF;
    global_last_m1a_val = GET_M1_ENCA;
    global_last_m1b_val = GET_M1_ENCB;
    sei();
}

int degrees_to_revs(int degrees) {
    return degrees * 6.247222222;
}

int move_degrees(int degrees, int direction) {
    long cur_position = global_counts_m1;
    if (direction == 1) {
        set_direction(1);
    } else {
        set_direction(0);
    }
}



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

