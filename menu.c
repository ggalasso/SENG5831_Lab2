#include "menu.h"
#include <stdio.h>
#include <inttypes.h>
#include "main.h"

// GLOBALS
extern long global_last_enc_value;
extern long global_cur_velo;
extern long global_counts_m1;
extern int global_compa_counter;
extern int global_m1a;
extern int global_m1b;
extern int global_error_m1;
extern int global_last_m1a_val;
extern int global_last_m1b_val;
extern int myMotorSpeed;
extern int desiredV;
//extern long target;
extern long target_position;
extern long P;
extern float Kp;
extern float I;
extern float Ki;
extern float D;
extern float Kd;
extern int Torq;
extern float time_period;
extern int spe_cm;
extern int pos_cm;
extern int log_cm;
extern int positionRequest;
extern int speedRequest;
extern int interpolatorRequest;


// local "global" data structures
char receive_buffer[32];
unsigned char receive_buffer_position;
char send_buffer[32];


// A generic function for whenever you want to print to your serial comm window.
// Provide a string and the length of that string. My serial comm likes "\r\n" at 
// the end of each string (be sure to include in length) for proper linefeed.
void print_usb( char *buffer, int n ) {
	serial_send( USB_COMM, buffer, n );
	wait_for_sending_to_finish();
}	
		
//------------------------------------------------------------------------------------------
// Initialize serial communication through USB and print menu options
// This immediately readies the board for serial comm
void init_menu() {
	
	char printBuffer[32];
	
	// Set the baud rate to 9600 bits per second.  Each byte takes ten bit
	// times, so you can get at most 960 bytes per second at this speed.
	serial_set_baud_rate(USB_COMM, 9600);

	// Start receiving bytes in the ring buffer.
	serial_receive_ring(USB_COMM, receive_buffer, sizeof(receive_buffer));

	//memcpy_P( send_buffer, PSTR("USB Serial Initialized\r\n"), 24 );
	//snprintf( printBuffer, 24, "USB Serial Initialized\r\n");
	//print_usb( printBuffer, 24 );
	//print_usb( "USB Serial Initialized\r\n", 28);
    int length = sprintf( printBuffer, "USB Serial Initialized\r\n");
    print_usb( printBuffer, length );
	//memcpy_P( send_buffer, MENU, MENU_LENGTH );
	print_usb( MENU, MENU_LENGTH );
}

//------------------------------------------------------------------------------------------
// process_received_byte: Parses a menu command (series of keystrokes) that 
// has been received on USB_COMM and processes it accordingly.
// The menu command is buffered in check_for_new_bytes_received (which calls this function).
void process_received_string(const char* buffer)
{
	// Used to pass to USB_COMM for serial communication

	char tempBuffer[48];
    int length;
	
	// parse and echo back to serial comm window (and optionally the LCD)
	char op_char;
	float value;
	int parsed;
	parsed = sscanf(buffer, "%c %f", &op_char, &value);
#ifdef ECHO2LCD
	lcd_goto_xy(0,0);
	printf("Got %c %d\n", op_char, value);
#endif
	length = sprintf( tempBuffer, "\r\n0p:%c V:%d\r\n", op_char, value );
	print_usb( tempBuffer, length );

	// Check valid command and implement
	switch (op_char) {
        //L/l: Start/Stop Logging (print) the values of Pr, Pm, and T.
        case 'l':
        case 'L':
            length = sprintf( tempBuffer, "Log enabled\r\n");
            print_usb( tempBuffer, length );
            length = sprintf( tempBuffer, "Provide position(txxxx) or speed(sxx) command\r\n");
            print_usb( tempBuffer, length );
            spe_cm = 0;
            pos_cm = 0;
            log_cm = 1;
            break;
        //V/v: View the current values Kd, Kp, Vm, Pr, Pm, and T
        case 'v':
        case 'V':
            length = sprintf( tempBuffer, "Encoder Counts: %ld\r\n", global_counts_m1);
            print_usb( tempBuffer, length );
            length = sprintf( tempBuffer, "Current motor speed: %d\r\n", myMotorSpeed);
            print_usb( tempBuffer, length );
            length = sprintf( tempBuffer, "Error: %ld\r\n", P);
            print_usb( tempBuffer, length );
            length = sprintf( tempBuffer, "Torq: %d\r\n", Torq);
            print_usb( tempBuffer, length );
            length = sprintf( tempBuffer, "Kp:% .5f, Ki:% .5f, Kd:% .5f\r\n", Kp, Ki, Kd);
            print_usb(tempBuffer, length );
            length = sprintf( tempBuffer, "P:% 2ld I:% 4.2f D% 4.2f\r\n", P, I, D);
            print_usb( tempBuffer, length );
            length = sprintf( tempBuffer, "Desired V:%d Current:V%d\r\n",desiredV, global_cur_velo);
            print_usb( tempBuffer, length );
            float total = (Kp * P) + (Ki * I) + (Kd * D);
            length = sprintf( tempBuffer, "Formula P:% .3f I:% .4f D:% .2f T:% .1f\r\n",Kp * P, Ki * I, Kd * D, total);
            print_usb( tempBuffer, length );
            length = sprintf( tempBuffer, "Period: %f\r\n", time_period);
            print_usb( tempBuffer, length );
            break;
        //R/r : Set the reference position (use unit "counts")
        case 'r':
        case 'R':
            break;
        //S/s : Set the reference speed (use unit "counts"/sec)
        case 's':
        case 'S':
            //Reset PID values
            resetPID();
            //Set speed command to true
            spe_cm = 1;
            //Set speed reques to true
            speedRequest = 1;
            //Set position reques to false
            positionRequest = 0;
            //Set interpolator request to false
            interpolatorRequest = 0;
            //Set the desired velocity to the passed value
            desiredV = value;
            break;
        //P: Set Kp value by passing a value
        case 'P':
        case 'p':
            Kp = value;
            break;
        //D: Set Kd value by passing a value
        case 'D':
        case 'd':
            Kd = value;
            break;
        //I: Set Ki value by passing a value
        case 'I':
        case 'i':
            Ki = value;
            break;
        //T: Set the time period, defaults to 1.
        case 'T':
            time_period = value;
            break;
        case 't':
            //target = value;
            target_position = global_counts_m1 + value;
            length = sprintf( tempBuffer, "Target Loc:% 05ld Current Loc:% 05ld\r\n", target_position, global_counts_m1);
            print_usb( tempBuffer, length );
            resetPID();
            positionRequest = 1;
            speedRequest = 0;
            interpolatorRequest = 0;
            pos_cm = 1;
            break;
        case 'U':
            resetPID();
            positionRequest = 0;
            speedRequest = 0;
            pos_cm = 1;
            spe_cm = 0;
            interpolatorRequest = 1;
            break;
        default:
            print_usb( "Command does not compute.\r\n", 27 );
    }
		
	print_usb( MENU, MENU_LENGTH);

} //end menu()

//---------------------------------------------------------------------------------------
// If there are received bytes to process, this function loops through the receive_buffer
// accumulating new bytes (keystrokes) in another buffer for processing.
void check_for_new_bytes_received()
{
	/* 
	The receive_buffer is a ring buffer. The call to serial_check() (you should call prior to this function) fills the buffer.
	serial_get_received_bytes is an array index that marks where in the buffer the most current received character resides. 
	receive_buffer_position is an array index that marks where in the buffer the most current PROCESSED character resides. 
	Both of these are incremented % (size-of-buffer) to move through the buffer, and once the end is reached, to start back at the beginning.
	This process and data structures are from the Pololu library. See examples/serial2/test.c and src/OrangutanSerial/*.*
	
	A carriage return from your comm window initiates the transfer of your keystrokes.
	All key strokes prior to the carriage return will be processed with a single call to this function (with multiple passes through this loop).
	On the next function call, the carriage return is processes with a single pass through the loop.
	The menuBuffer is used to hold all keystrokes prior to the carriage return. The "received" variable, which indexes menuBuffer, is reset to 0
	after each carriage return.
	*/ 
	char menuBuffer[32];
	static int received = 0;
	
	// while there are unprocessed keystrokes in the receive_buffer, grab them and buffer
	// them into the menuBuffer
	while(serial_get_received_bytes(USB_COMM) != receive_buffer_position)
	{
		// place in a buffer for processing
		menuBuffer[received] = receive_buffer[receive_buffer_position];
		++received;
		
		// Increment receive_buffer_position, but wrap around when it gets to
		// the end of the buffer. 
		if ( receive_buffer_position == sizeof(receive_buffer) - 1 )
		{
			receive_buffer_position = 0;
		}			
		else
		{
			receive_buffer_position++;
		}
	}
	// If there were keystrokes processed, check if a menue command
	if (received) {
		// if only 1 received, it was MOST LIKELY a carriage return. 
		// Even if it was a single keystroke, it is not a menu command, so ignore it.
        if ( 1 == received && menuBuffer[0] == '\n') {
            received = 0;
            return;
        }
		// Process buffer: terminate string, process, reset index to beginning of array to receive another command
		menuBuffer[received] = '\0';
#ifdef ECHO2LCD
		lcd_goto_xy(0,1);			
		print("RX: (");
		print_long(received);
		print_character(')');
		for (int i=0; i<received; i++)
		{
			print_character(menuBuffer[i]);
		}
#endif
		process_received_string(menuBuffer);
		received = 0;
	}
}
	
//-------------------------------------------------------------------------------------------
// wait_for_sending_to_finish:  Waits for the bytes in the send buffer to
// finish transmitting on USB_COMM.  We must call this before modifying
// send_buffer or trying to send more bytes, because otherwise we could
// corrupt an existing transmission.
void wait_for_sending_to_finish()
{
	while(!serial_send_buffer_empty(USB_COMM))
		serial_check();		// USB_COMM port is always in SERIAL_CHECK mode
}


