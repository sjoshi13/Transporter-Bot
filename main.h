/*
* Team Id: e-YRC#783
* Author List: Shruti Joshi, Shivangi Mishra,Jyotsna Sharma,Ram Mahesh
 
* Filename: main.h
* Theme: Transporter Bot
* Functions: buzzer_on(),buzzer_off(),adjust_left(),adjust_left_s(),adjust_right(),adjust_right_s(),forward(),back(),
             left(),right(),soft_left(),soft_right(),soft_left_2(),soft_right_2(),stop(),angle_rotate(),linear_distance_mm(),
			 left_degrees(),right_degrees(),soft_left_degrees(),soft_right_degrees(),soft_left_2_degrees(),soft_right_2_degrees(),
			 print_sensor(),uart_transmit(),sensor(),pick(),def_servo(),drop(),linefollower(),path(),forward_mm()
			 path1(),path2(),path3(),path4(),path5(),path6(),path7(),path8(),path9(),path10(),path11(),path12();
* Global Variables: char ps[], unsigned char ADC_Value, unsigned char Left_white_line = 0, unsigned char Center_white_line = 0,
*					unsigned char Right_white_line = 0, volatile unsigned long int ShaftCountLeft = 0,volatile unsigned long int ShaftCountRight = 0,
*					volatile unsigned int Degrees,int t=0,n=0;
*/

void port_init();
#define BAUD 9600                                  // define baud
#define BAUDRATE ((F_CPU)/(BAUD*16UL)-1)
void timer5_init();
void velocity(unsigned char, unsigned char);
void motors_delay();
void linear_distance_mm1(unsigned int DistanceInMM);
unsigned char ADC_Conversion(unsigned char);   
unsigned char ADC_Value;
char ps[12]={'b',' ','y',' ','b',' ','r',' ','r',' ',' ','g'};
unsigned char Left_white_line = 0;
unsigned char Center_white_line = 0;
unsigned char Right_white_line = 0;
volatile unsigned long int ShaftCountLeft = 0; //to keep track of left position encoder
volatile unsigned long int ShaftCountRight = 0; //to keep track of right position encoder
volatile unsigned int Degrees; //to accept angle in degrees for turning
int t=0,n=0,i4,flag;

ISR(TIMER4_OVF_vect)
{
	//TIMER4 has overflowed
	TCNT4H = 0xF9; //reload counter high value
	TCNT4L = 0xB3; //reload counter low value
}	
void adc_init()
{
	ADCSRA = 0x00;
	ADCSRB = 0x00;		//MUX5 = 0
	ADMUX = 0x20;		//Vref=5V external --- ADLAR=1 --- MUX4:0 = 0000
	ACSR = 0x80;
	ADCSRA = 0x86;		//ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0
}
unsigned char ADC_Conversion(unsigned char Ch){
	unsigned char a;
	if(Ch>7)
     	{
	    	ADCSRB = 0x08;
	    }
	Ch = Ch & 0x07;
	ADMUX= 0x20| Ch;
	ADCSRA = ADCSRA | 0x40;		//Set start conversion bit
	while((ADCSRA&0x10)==0);	//Wait for ADC conversion to complete
	a=ADCH;
	ADCSRA = ADCSRA|0x10; //clear ADIF (ADC Interrupt Flag) by writing 1 to it
	ADCSRB = 0x00;
	return a;
}
void buzzer_pin_config (void){
	DDRC = DDRC | 0x08;		//Setting PORTC 3 as output
	PORTC = PORTC & 0xF7;		//Setting PORTC 3 logic low to turnoff buzzer
}
/*   Function Name: buzzer_on
*    Input:     None 
*    Output:    None 
*    Logic:     This function will turn the buzzer on  
*    Example Call: buzzer_on();  
*/
void buzzer_on (void)
{
	unsigned char port_restore = 0;
	port_restore = PINC;
	port_restore = port_restore | 0x08;
	PORTC = port_restore;
}
/*   Function Name: buzzer_off
*    Input:     None
*    Output:    None
*    Logic:     This function will turn the buzzer off
*    Example Call: buzzer_off();
*/

void buzzer_off (void)
{
	unsigned char port_restore = 0;
	port_restore = PINC;
	port_restore = port_restore & 0xF7;
	PORTC = port_restore;
}
/*   Function Name: adjust_left
*    Input:     None
*    Output:    None
*    Logic:     it will rotate left until robot comes on the black line.Bith the wheels rotate.
*    Example Call: buzzer_on();
*/

void adjust_left()
{
	while(1)
	{
		
		Left_white_line = ADC_Conversion(3);	//Getting data of Left WL Sensor
		Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
		Right_white_line = ADC_Conversion(1);	//Getting data of Right WL Sensor

		if(Left_white_line<25 && Center_white_line>25 && Right_white_line<25) //this condition will check weather robot is on black line or not and if it is on black line than it will break the loop
		{
			break;
		}
		velocity(200,200);
		left();//this will take left turn until it find black line


	}
}
/*   Function Name: adjust_left_s()
*    Input:     None
*    Output:    None
*    Logic:     it will rotate left until robot comes on the black line, only the right wheel rotates while the left wheel remains static
*    Example Call: adjust_left_s()
*/
void adjust_left_s()
{
	while(1)
	{
		
		Left_white_line = ADC_Conversion(3);	//Getting data of Left WL Sensor
		Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
		Right_white_line = ADC_Conversion(1);	//Getting data of Right WL Sensor

		if(Left_white_line<25 && Center_white_line>25 && Right_white_line<25) //this condition will check weather robot is on black line or not and if it is on black line than it will break the loop
		{
			break;
		}
		velocity(250,220);//i changed this
		soft_left();//this will take left turn until it find black line


	}
}
/*   Function Name: adjust_right()
*    Input:     None
*    Output:    None
*    Logic:     it will rotate right until robot comes on the black line. Both th ewheels are rotating.
*    Example Call: adjust_right()
*/
void adjust_right()
{
	while(1)
	{
		

		Left_white_line = ADC_Conversion(3);	//Getting data of Left WL Sensor
		Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
		Right_white_line = ADC_Conversion(1);	//Getting data of Right WL Sensor

		if(Left_white_line<25 && Center_white_line>25 && Right_white_line<25)//this condition will check weather robot is on black line or not and if it is on black line than it will break the loop
		{
			break;
		}
		velocity(220,220);
		right(); //it will take right turn until it find black line

	}
}
/*   Function Name: adjust_right_s()
*    Input:     None
*    Output:    None
*    Logic:     it will rotate right until robot comes on the black line, only the left wheel rotates while the right wheel remains static
*    Example Call: adjust_right_s()
*/
void adjust_right_s()
{
	while(1)
	{
		

		Left_white_line = ADC_Conversion(3);	//Getting data of Left WL Sensor
		Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
		Right_white_line = ADC_Conversion(1);	//Getting data of Right WL Sensor

		if(Left_white_line<15 && Center_white_line>25 && Right_white_line<15)//this condition will check weather robot is on black line or not and if it is on black line than it will break the loop
		{
			break;
		}
		velocity(200,200);//i cahnged this
		soft_right(); //it will take right turn until it find black line

	}
}
//Function to configure INT5 (PORTE 5) pin as input for the left position encoder
void left_encoder_pin_config (void)
{
	DDRE  = DDRE & 0xEF;  //Set the direction of the PORTE 4 pin as input
	PORTE = PORTE | 0x10; //Enable internal pull-up for PORTE 4 pin
}

//Function to configure INT5 (PORTE 5) pin as input for the right position encoder
void right_encoder_pin_config (void)
{
	DDRE  = DDRE & 0xDF;  //Set the direction of the PORTE 4 pin as input
	PORTE = PORTE | 0x20; //Enable internal pull-up for PORTE 4 pin
}

//Function to initialize ports
void port_init()
{
    servo1_pin_config(); //Configure PORTB 5 pin for servo motor 1 operation
    servo2_pin_config(); //Configure PORTB 6 pin for servo motor 2 operation

	motion_pin_config(); //robot motion pins config
	left_encoder_pin_config(); //left encoder pin config
	right_encoder_pin_config(); //right encoder pin config
	buzzer_pin_config();
	adc_pin_config();
	lcd_port_config();
}


void left_position_encoder_interrupt_init (void) 
{
	cli(); //Clears the global interrupt
	EICRB = EICRB | 0x02; // INT4 is set to trigger with falling edge
	EIMSK = EIMSK | 0x10; // Enable Interrupt INT4 for left position encoder
	sei();   // Enables the global interrupt
}

void right_position_encoder_interrupt_init (void) 
{
	cli(); //Clears the global interrupt
	EICRB = EICRB | 0x08; // INT5 is set to trigger with falling edge
	EIMSK = EIMSK | 0x20; // Enable Interrupt INT5 for right position encoder
	sei();   // Enables the global interrupt
}
//ISR for right position encoder
ISR(INT5_vect)
{
	ShaftCountRight++;  //increment right shaft position count
}


//ISR for left position encoder
ISR(INT4_vect)
{
	ShaftCountLeft++;  //increment left shaft position count
}
void motion_pin_config (void)
{
	DDRA = DDRA | 0x0F;
	PORTA = PORTA & 0xF0;
	DDRL = DDRL | 0x18;   //Setting PL3 and PL4 pins as output for PWM generation
	PORTL = PORTL | 0x18; //PL3 and PL4 pins are for velocity control using PWM.
}

void motion_set (unsigned char Direction)
{
	unsigned char PortARestore = 0;

	Direction &= 0x0F; 		// removing upper nibbel for the protection
	PortARestore = PORTA; 		// reading the PORTA original status
	PortARestore &= 0xF0; 		// making lower direction nibbel to 0
	PortARestore |= Direction; // adding lower nibbel for forward command and restoring the PORTA status
	PORTA = PortARestore; 		// executing the command
}

void forward (void) //both wheels forward
{
	motion_set(0x06);
}

void back (void) //both wheels backward
{
	motion_set(0x09);
}

void left (void) //Left wheel backward, Right wheel forward
{
	velocity(200,200);
	motion_set(0x05);
}

void right (void) //Left wheel forward, Right wheel backward
{
	velocity(200,200);
	motion_set(0x0A);
}

void soft_left (void) //Left wheel stationary, Right wheel forward
{
	motion_set(0x04);
}

void soft_right (void) //Left wheel forward, Right wheel is stationary
{
	motion_set(0x02);
}

void soft_left_2 (void) //Left wheel backward, right wheel stationary
{
	motion_set(0x01);
}

void soft_right_2 (void) //Left wheel stationary, Right wheel backward
{
	motion_set(0x08);
}

void stop (void)
{
	motion_set(0x00);
}
/*
*Name of function: angle_rotate(unsigned int Degrees)
*Input: unsigned int Degrees
*Outut: The robot will turn by specified degrees 
*Logic: We calculated the no. of shafts required to turn by specified degrees, until that is matched, the bot rotates ultimately rotating by the required 
*		angle 
*Example Call: angle_rotate(90)
*/
void angle_rotate(unsigned int Degrees)
{
	float ReqdShaftCount = 0;
	unsigned long int ReqdShaftCountInt = 0;

	ReqdShaftCount = (float) Degrees/ 4.090; // division by resolution to get shaft count
	ReqdShaftCountInt = (unsigned int) ReqdShaftCount;
	ShaftCountRight = 0;
	ShaftCountLeft = 0;

	while (1)
	{


		if((ShaftCountRight >= ReqdShaftCountInt) | (ShaftCountLeft >= ReqdShaftCountInt))
		{

			break;
		}
	}
	stop(); //Stop robot
}
/*
*Name of function: linear_distance_mm(unsigned int DistanceInMM)
*Input: unsigned int DistanceInMM, the distance in mm that is to be moved 
*Outut:	the bot will move by specifed distance
*Logic: we calculated the no. of shafts required to move forward by specified distane, until that is matched, the bot moves forward 
*		 ultimately moving the bot by the required distance 
*Example Call: linear_distance_mm(850)
*/
void linear_distance_mm(unsigned int DistanceInMM)
{
	float ReqdShaftCount = 0;
	unsigned long int ReqdShaftCountInt = 0;

	ReqdShaftCount = DistanceInMM / 5.338; // division by resolution to get shaft count
	ReqdShaftCountInt = (unsigned long int) ReqdShaftCount;

	ShaftCountRight = 0;
	while(1)
	{
		if(ShaftCountRight > ReqdShaftCountInt)
		{
			break;
		}
	}
	stop(); //Stop robot
}
/*
*Name of function: forward_mm(unsigned int DistanceInMM)
*Input: unsigned int DistanceInMM, the distance in mm that is to be moved 
*Outut:	the bot will move forward by specifed distance
*Logic:  linear_distance_mm(unsigned int DistanceInMM) is called and forward() function is called to move the bot forward by specified distance
*Example Call: forward_mm(800)
*/
void forward_mm(unsigned int DistanceInMM)
{
	forward();
	linear_distance_mm(DistanceInMM);
}
/*
*Name of function: left_degrees(unsigned int Degrees)
*Input: degrees to move left.
*Outut: None
*Logic: The bot moves to left and angle_rotate() function is called to move by specified degrees 
*Example Call: left_degrees(75)
*/
void left_degrees(unsigned int Degrees)
{
	// 88 pulses for 360 degrees rotation 4.090 degrees per count
	left(); //Turn left
	angle_rotate(Degrees);
}
/*
*Name of function: right_degrees(unsigned int Degrees)
*Input: degrees to move right.
*Outut: None
*Logic: The bot moves to right and angle_rotate() function is called to move by specified degrees 
*Example Call: right_degrees(75)
*/
void right_degrees(unsigned int Degrees)
{
	// 88 pulses for 360 degrees rotation 4.090 degrees per count
	right(); //Turn right
	angle_rotate(Degrees);
}
/*
*Name of function: soft_left_degrees(unsigned int Degrees)
*Input: degrees to move left.
*Outut: None
*Logic: The bot moves to left by moving right wheel and other is kept static and angle_rotate() function is called to move by specified degrees 
*Example Call: right_degrees(75)
*/
void soft_left_degrees(unsigned int Degrees)
{
	// 176 pulses for 360 degrees rotation 2.045 degrees per count
	soft_left(); //Turn soft left
	Degrees=Degrees*2;
	angle_rotate(Degrees);
}
/*
*Name of function: soft_right_degrees(unsigned int Degrees)
*Input: degrees to move right.
*Outut: None
*Logic: The bot moves to right by moving left wheel and other is kept static and angle_rotate() function is called to move by specified degrees 
*Example Call: right_degrees(75)
*/
void soft_right_degrees(unsigned int Degrees)
{
	// 176 pulses for 360 degrees rotation 2.045 degrees per count
	soft_right();  //Turn soft right
	Degrees=Degrees*2;
	angle_rotate(Degrees);
}
//ADC pin configuration
void adc_pin_config (void)
{
	DDRF = 0x00;
	PORTF = 0x00;
	DDRK = 0x00;
	PORTK = 0x00;
}

void timer5_init()
{
	TCCR5B = 0x00;	//Stop
	TCNT5H = 0xFF;	//Counter higher 8-bit value to which OCR5xH value is compared with
	TCNT5L = 0x01;	//Counter lower 8-bit value to which OCR5xH value is compared with
	OCR5AH = 0x00;	//Output compare register high value for Left Motor
	OCR5AL = 0xFF;	//Output compare register low value for Left Motor
	OCR5BH = 0x00;	//Output compare register high value for Right Motor
	OCR5BL = 0xFF;	//Output compare register low value for Right Motor
	OCR5CH = 0x00;	//Output compare register high value for Motor C1
	OCR5CL = 0xFF;	//Output compare register low value for Motor C1
	TCCR5A = 0xA9;	/*{COM5A1=1, COM5A0=0; COM5B1=1, COM5B0=0; COM5C1=1 COM5C0=0}
					For Overriding normal port functionality to OCRnA outputs.
						{WGM51=0, WGM50=1} Along With WGM52 in TCCR5B for Selecting FAST PWM 8-bit Mode*/

	TCCR5B = 0x0B;	//WGM12=1; CS12=0, CS11=1, CS10=1 (Prescaler=64)
}
//Function for velocity control
void velocity (unsigned char left_motor, unsigned char right_motor)
{
	OCR5AL = (unsigned char)left_motor;
	OCR5BL = (unsigned char)right_motor;
}
/*
*Name of function: soft_left_2_degrees(unsigned int Degrees)
*Input:unsigned int Degrees, degrees to move left.
*Outut: the bot moves to the left by taking a reverse soft left
*Logic: The bot moves to left by moving backward and takes a left turn, angle_rotate() function is called to move by specified degrees 
*Example Call: soft_left_2_degrees(76);
*/

void soft_left_2_degrees(unsigned int Degrees)
{
	// 176 pulses for 360 degrees rotation 2.045 degrees per count
	soft_left_2(); //Turn reverse soft left
	Degrees=Degrees*2;
	angle_rotate(Degrees);
}
/*
*Name of function: soft_right_2_degrees(unsigned int Degrees)
*Input:unsigned int Degrees, degrees to move right.
*Outut: the bot moves to the right by taking a reverse soft right
*Logic: The bot moves to right by moving backward and takes a right turn, angle_rotate() function is called to move by specified degrees. 
*Example Call: soft_left_2_degrees(76);
*/
void soft_right_2_degrees(unsigned int Degrees)
{
	// 176 pulses for 360 degrees rotation 2.045 degrees per count
	soft_right_2();  //Turn reverse soft right
	Degrees=Degrees*2;
	angle_rotate(Degrees);
}

//Function to initialize all the devices
void init_devices()
{
	cli(); //Clears the global interrupt
	port_init();  //Initializes all the ports
	left_position_encoder_interrupt_init();
	right_position_encoder_interrupt_init();
	adc_init();
	timer5_init();
	timer4_init();
	timer1_init();
	TIMSK4 = 0x01; //timer4 overflow interrupt enable
	sei();   // Enables the global interrupt
}
//TIMER4 initialize - prescale:64
// WGM: 0) Normal, TOP=0xFFFF
// desired value: 1Hz
// actual value:  1.000Hz (0.0%)
void timer4_init(void)
{
	TCCR4B = 0x00; //stop
	TCNT4H = 0xF9; //Counter higher 8 bit value
	TCNT4L = 0xB3; //Counter lower 8 bit value
	OCR4AH = 0x00; //Output Compare Register (OCR)- Not used
	OCR4AL = 0x00; //Output Compare Register (OCR)- Not used
	OCR4BH = 0x00; //Output Compare Register (OCR)- Not used
	OCR4BL = 0x00; //Output Compare Register (OCR)- Not used
	OCR4CH = 0x00; //Output Compare Register (OCR)- Not used
	OCR4CL = 0x00; //Output Compare Register (OCR)- Not used
	ICR4H  = 0x00; //Input Capture Register (ICR)- Not used
	ICR4L  = 0x00; //Input Capture Register (ICR)- Not used
	TCCR4A = 0x00;
	TCCR4C = 0x00;
	TCCR4B = 0x03; //start Timer
}
char data;     // variable that stores the character to be sent to blender and rotating disc via ZigBee
//function to initialise the uart transmission
void uart_init (void)
{
	UBRR0H = (BAUDRATE>>8);                      // shift the register right by 8 bits
	UBRR0L = BAUDRATE;                           // set baud rate
	UCSR0B|= (1<<TXEN0)|(1<<RXEN0);                // enable receiver and transmitter
	UCSR0C|= /*(1<<UMSEL)|*/(1<<UCSZ00)|(1<<UCSZ01);   // 8bit data format
}
//Function to configure LCD port
void lcd_port_config (void)
{
	DDRC = DDRC | 0xF7; //all the LCD pin's direction set as output
	PORTC = PORTC & 0x80; // all the LCD pins are set to logic 0 except PORTC 7
}
//Function To Print Sesor Values At Desired Row And Coloumn Location on LCD
void print_sensor(char row, char coloumn,unsigned char channel)
{
	
	ADC_Value = ADC_Conversion(channel);
	lcd_print(row, coloumn, ADC_Value, 3);
}
//Function to initialize the Sharp Sensor
unsigned int Sharp_GP2D12_estimation(unsigned char adc_reading)
{
	float distance;
	unsigned int distanceInt;
	distance = (int)(10.00*(2799.6*(1.00/(pow(adc_reading,1.1546)))));
	distanceInt = (int)distance;
	if(distanceInt>800)
	{
		distanceInt=800;
	}
	return distanceInt;
}
/*
*Name of function: uart_transmit (unsigned char data)
*Input: unsigned char data,character to be transmitted
*Outut:	none
*Logic: stores the character in UDR0 register and transmits the ASCII value of the character to zigbee
*Example Call: uart_transmit (114)
*/

void uart_transmit (unsigned char data)
{
	while (!( UCSR0A & (1<<UDRE0)));                // wait while register is free
	UDR0 = data;                                   // load data in the register
}
/*
*Name of function: sensor()
*Input: None
*Outut:	None
*Logic: the analog values from sensor are converted to digital and are displayed on the lcd panel 
*Example Call: sensor
*/
void sensor(){
while(1){
Left_white_line = ADC_Conversion(3);	//Getting data of Left WL Sensor
Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
Right_white_line = ADC_Conversion(1);	//Getting data of Right WL Sensor

flag=0;

print_sensor(1,1,3);	//Prints value of White Line Sensor1
print_sensor(1,5,2);	//Prints Value of White Line Sensor2
print_sensor(1,9,1);	//Prints Value of White Line Sensor3
}
}
//Configure PORTA 6 pin for servo motor 1 operation
void servo1_pin_config (void)
{
	DDRB  = DDRB | 0x20;  //making PORTB 5 pin output
	 PORTB = PORTB | 0x20; //setting PORTB 5 pin to logic 1
}

//Configure PORTB 6 pin for servo motor 2 operation
void servo2_pin_config (void)
{
	DDRB  = DDRB | 0x40;  //making PORTB 6 pin output
	PORTB = PORTB | 0x40; //setting PORTB 6 pin to logic 1
}


//Function to rotate Servo 1 by a specified angle in the multiples of 1.86 degrees
void servo_1(unsigned char degrees)
{
	float PositionPanServo = 0;
	PositionPanServo = ((float)degrees / 1.86) + 35.0;
	OCR1AH = 0x00;
	OCR1AL = (unsigned char) PositionPanServo;
}


//Function to rotate Servo 2 by a specified angle in the multiples of 1.86 degrees
void servo_2(unsigned char degrees)
{
	float PositionTiltServo = 0;
	PositionTiltServo = ((float)degrees / 1.86) + 35.0;
	OCR1BH = 0x00;
	OCR1BL = (unsigned char) PositionTiltServo;
}
void servo_1_free (void) //makes servo 1 free rotating
{
	OCR1AH = 0x03;
	OCR1AL = 0xFF; //Servo 1 off
}

void servo_2_free (void) //makes servo 2 free rotating
{
	OCR1BH = 0x03;
	OCR1BL = 0xFF; //Servo 2 off
}
/*
*Name of function: pick()
*Input: none
*Outut:	none 
*Logic:	moves the servo attached to the bot to zero degrees to come close to the block to be picked up. 
*		Then the servo attached at the end of the arm is closed (i.e., becomes zero degree) to grip the block 
*		after the block is gripped the servo attached to the bot is rotated to 60 degree so as to lift the block
*		and the block is picked up
*Example Call: pick()
*/
void pick(){
	servo_1(0);
	_delay_ms(2000);
	servo_2(0);
	_delay_ms(500);
	servo_1(60);
	
}	
/*
*Name of function: def_servo()
*Input: none
*Outut:	none 
*Logic:	defines the default position of the arm while the bot traverses the arena
*		moves the servo attached to the bot to 120 degrees and servo attached to the arm end to 90 degree
*Example Call: def_servo()
*/
void def_servo()
{
	servo_2(90);
	//_delay_ms(300);
	servo_1(120);
}
/*
*Name of function: drop()
*Input: none
*Outut:	none 
*Logic:	drops the block on the rotating disc by ungripping the block. Ungripping is done by changing the angles of the servos 
*Example Call: drop()
*/
void drop(){
	_delay_ms(1000);
	servo_2(30);
	servo_1(60);
	_delay_ms(500);
	servo_1(120);
	uart_transmit(100);///d
}
void timer1_init(void)
{
 TCCR1B = 0x00; //stop
 TCNT1H = 0xFC; //Counter high value to which OCR1xH value is to be compared with
 TCNT1L = 0x01;	//Counter low value to which OCR1xH value is to be compared with
 OCR1AH = 0x03;	//Output compare Register high value for servo 1
 OCR1AL = 0xFF;	//Output Compare Register low Value For servo 1
 OCR1BH = 0x03;	//Output compare Register high value for servo 2
 OCR1BL = 0xFF;	//Output Compare Register low Value For servo 2
 OCR1CH = 0x03;	//Output compare Register high value for servo 3
 OCR1CL = 0xFF;	//Output Compare Register low Value For servo 3
 ICR1H  = 0x03;	
 ICR1L  = 0xFF;
 TCCR1A = 0xAB; /*{COM1A1=1, COM1A0=0; COM1B1=1, COM1B0=0; COM1C1=1 COM1C0=0}
 					For Overriding normal port functionality to OCRnA outputs.
				  {WGM11=1, WGM10=1} Along With WGM12 in TCCR1B for Selecting FAST PWM Mode*/
 TCCR1C = 0x00;
 TCCR1B = 0x0C; //WGM12=1; CS12=1, CS11=0, CS10=0 (Prescaler=256)
}
/*
*Name of function: linefollower();
*Input: none	
*Outut: none
*Logic: Follows the line by taking the appropriate decision at condiiton of the sensors and the bot detects the nodes
 when left and centre or right and centre sensors are on the black line
*
*Example Call: linefollower();
*/
void linefollower()
{
	while(1)
	{
		Left_white_line = ADC_Conversion(3);	//Getting data of Left WL Sensor
		Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
		Right_white_line = ADC_Conversion(1);	//Getting data of Right WL Sensor
		forward();
		if(Left_white_line > cut_off &&  Center_white_line <cut_off && Right_white_line <cut_off)
		{
			stop();
			velocity(220,250);
			soft_left();	//soft left
		}
		if(Left_white_line<cut_off && Center_white_line > cut_off && Right_white_line <cut_off)
		{
			
			forward();
		}
		
		if(Left_white_line< cut_off && Center_white_line <cut_off && Right_white_line >cut_off)
		{
			stop();
			velocity(250,220);
			soft_right();	//soft right
		}
          if(((Right_white_line>cut_off && Center_white_line>cut_off) || (Left_white_line>cut_off && Center_white_line>cut_off)) && (ShaftCountLeft>30 || ShaftCountRight>30))
        {   
			stop();
			buzzer_on();
			_delay_ms(5);
			buzzer_off();
	        _delay_ms(5);
			n=n+1;
	        lcd_print(2,13,n,2);
			ShaftCountLeft=0,ShaftCountRight=0;
	        break;
		}
	}		
}
/*
*Name of function: path(int position) 
*Input: int position
*Outut: returns the path of the specified position
*Logic: it basically takes the position on which the block is kept and gives the path associated with that position.
*	comparing the value of the nodes counted is compared with the cases the bot moves as per the instructions given
*Example Call: path(4)
*/
void path(int position)
{
	switch (position)
	{
		case 1: path1();
		break;
		case 2: path2();
		break;
		case 3: path3();
		break;
		case 4: path4();
		break;
		case 5: path5();
		break;
		case 6: path6();
		break;
		case 7: path7();
		break;
		case 8: path8();
		break;
		case 9: path9();
		break;
		case 10: path10();
		break;
		case 11: path11();
		break;
		case 12: path12();
		break;
	}
}
/*
*Name of function: path2()
*Input: none
*Outut: none
*Logic: The bot moves to the pickup point 2 by detecting the nodes and accordingly turning right or left or going straight.
*	comparing the value of the nodes counted is compared with the cases the bot moves as per the instructions given
*Example Call: path2()
*/	
void path2()
{ 
	char c1=ps[1];
	int c=c1;
	right_degrees(75);
	linefollower();
	if(n==1)
	{
		forward();
		_delay_ms(100);
		stop();
		_delay_ms(1);
		linefollower();
	}
	if(n==2)
	{
		forward();
		_delay_ms(400);
		stop();
		left_degrees(85);
		adjust_left_s();
		linefollower();
	}
	
	if(n==3)
	{
		def_servo();
		forward();
		_delay_ms(500);
		stop();
		_delay_ms(1000);
		right_degrees(80);
		_delay_ms(1000);
		uart_transmit(c);
		pick();
		buzzer_on();
		_delay_ms(1000);
		buzzer_off();
		right_degrees(100);
		linefollower();
	}
	
	if(n==4)
	{
		forward();
		_delay_ms(600);
		stop();
		_delay_ms(1);
		right_degrees(90);
		adjust_right_s();
		linefollower();
	}
	
	if(n==5)
	{
		forward();
		_delay_ms(200);
		linefollower();
		stop();
	}
	if(n==6)
	{
		forward();
		_delay_ms(600);
		stop();
		_delay_ms(1);
		right_degrees(90);
		adjust_right_s();
		linefollower();
	}

	if(n==7)
	{
		forward();
		_delay_ms(200);
		linefollower();
		stop();
		
	}
	if(n==8)
	{  _delay_ms(500);
		drop();
		buzzer_on();
		_delay_ms(2000);
		buzzer_off();
		_delay_ms(500);
		_delay_ms(500);
		right_degrees(180);
		linefollower();
		
	}
	if(n==9)
	{
		forward();
		_delay_ms(200);
		linefollower();
		stop();
	}
	if(n==10)
	{
		stop();
		forward();
		_delay_ms(800);
		stop();
		_delay_ms(300);
		right_degrees(170);
		_delay_ms(1);
		adjust_right_s();
		n=0;
	}
}
/*
*Name of function: path1()
*Input: none
*Outut: none
*Logic: The bot moves to the pickup point 1 by detecting the nodes and accordingly turning right or left or going straight.
*	comparing the value of the nodes counted is compared with the cases the bot moves as per the instructions given
*Example Call: path1()
*/	
void path1()
{
	
	char c1=ps[0];
	int c=c1;
	right_degrees(75);
	linefollower();
	if(n==1)
	{
		forward();
		_delay_ms(100);
		stop();
		_delay_ms(1);
		linefollower();
	}
	if(n==2)
	{
		forward();
		_delay_ms(400);
		stop();
		left_degrees(80);
		adjust_left_s();
		linefollower();
	}
	if(n==3)
	{
		forward();
		_delay_ms(300);
		stop();
		_delay_ms(1);
		linefollower();
	}
	if(n==4)
	{
		def_servo();
		forward();
		_delay_ms(500);
		stop();
		_delay_ms(500);
		right_degrees(70);
		_delay_ms(500);
		uart_transmit(c);
		pick();
		buzzer_on();
		_delay_ms(500);
		buzzer_off();
		right_degrees(90);
		adjust_right_s();
		linefollower();
	}
	if(n==5)
	{
		forward();
		_delay_ms(300);
		stop();
		_delay_ms(1);
		linefollower();
	}
	if(n==6)
	{
		forward();
		_delay_ms(600);
		stop();
		_delay_ms(1);
		right_degrees(90);
		adjust_right_s();
		linefollower();
	}
	
    if(n==7)
	{
		forward();
		_delay_ms(200);
		linefollower();
		stop();
	}
if(n==8)
{
	forward();
	_delay_ms(600);
	stop();
	_delay_ms(1);
	right_degrees(90);
	adjust_right_s();
	linefollower();
}

if(n==9)
{
	forward();
	_delay_ms(200);
	linefollower();
	stop();
	
}
if(n==10)
{  _delay_ms(500);
	drop();
	buzzer_on();
	_delay_ms(500);
	buzzer_off();
	_delay_ms(500);
	_delay_ms(500);
	right_degrees(180);
	linefollower();
	
}
if(n==11)
{
	forward();
	_delay_ms(200);
	linefollower();
	stop();
}
if(n==12)
{
	stop();
	forward();
	_delay_ms(800);
	stop();
	_delay_ms(300);
	right_degrees(170);
	_delay_ms(1);
	adjust_right_s();
	stop();
	_delay_ms(100);
	n=0;
}
}
/*
*Name of function: path3()
*Input: none
*Outut: none
*Logic: The bot moves to the pickup point 3 by detecting the nodes and accordingly turning right or left or going straight.
*	comparing the value of the nodes counted is compared with the cases the bot moves as per the instructions given
*Example Call: path3()
*/	
void path3()
{
	
	char c1=ps[2];
	int c=c1;
	right_degrees(75);
	_delay_ms(2);
	adjust_right_s();
	linefollower();
	if(n==1)
	{
		forward();
		_delay_ms(300);
		stop();
		_delay_ms(1) ;
		linefollower();
	}
	if(n==2)
	{
		forward();
		_delay_ms(500);
		stop();
		right_degrees(80);
		adjust_right_s();
		linefollower();
	}
	if(n==3)
	{
		def_servo();
		forward_mm(50);
		_delay_ms(50);
		stop();
		_delay_ms(500);
		left_degrees(90);
		_delay_ms(500);
		uart_transmit(c);
		pick();
		buzzer_on();
		_delay_ms(500);
		buzzer_off();
		left_degrees(100);
		linefollower();
	}
	if(n==4)
	{
		forward();
		_delay_ms(400);
		stop();
		_delay_ms(1);
		left_degrees(90);
		//adjust_left_s();
		linefollower();
	}
	if(n==5)
	{   forward();
		_delay_ms(300);
		stop();
		_delay_ms(100);
		linefollower();
	}
	if(n==6)
	{
		forward();
		_delay_ms(600);
		stop();
		_delay_ms(1);
		right_degrees(90);
		adjust_right_s();
		linefollower();
	}

	if(n==7)
	{
		forward();
		_delay_ms(200);
		linefollower();
		stop();
		
	}
	if(n==8)
	{  _delay_ms(500);
		drop();
		buzzer_on();
		_delay_ms(2000);
		buzzer_off();
		_delay_ms(500);
		_delay_ms(500);
		right_degrees(180);
		adjust_right_s();
		linefollower();
		
	}
	if(n==9)
	{
		forward();
		_delay_ms(200);
		linefollower();
		stop();
	}
	if(n==10)
	{
		stop();
		forward();
		_delay_ms(800);
		stop();
		_delay_ms(300);
		right_degrees(180);
		n=0;
	}
}
/*
*Name of function: path4()
*Input: none
*Outut: none
*Logic: The bot moves to the pickup point 4 by detecting the nodes and accordingly turning right or left or going straight.
*	comparing the value of the nodes counted is compared with the cases the bot moves as per the instructions given
*Example Call: path4()
*/	
void path4()
{
	
	char c1=ps[3];
	int c=c1;
	right_degrees(90);
	linefollower();
	if(n==1)
	{
		forward();
		_delay_ms(300);
		stop();
		_delay_ms(1);
		linefollower();
	}
	if(n==2)
	{
		forward();
		_delay_ms(500);
		stop();
		right_degrees(90);
		adjust_right_s();
		linefollower();
	}
	if(n==3)
	{
		forward();
		_delay_ms(300);
		stop();
		_delay_ms(1);
		linefollower();
	}
	if(n==4)
	{
		def_servo();
		forward_mm(40);
		_delay_ms(50);
		stop();
		_delay_ms(500);
		left_degrees(90);
		_delay_ms(1000);
		uart_transmit(c);
		pick();
		buzzer_on();
		_delay_ms(500);
		buzzer_off();
		left_degrees(75);
		_delay_ms(20);
		adjust_left_s();
		linefollower();
	}
	if(n==5)
	{
		forward();
		_delay_ms(300);
		stop();
		_delay_ms(1);
		linefollower();
	}
	if(n==6)
	{
		forward();
		_delay_ms(600);
		stop();
		_delay_ms(1);
		left_degrees(90);
		adjust_left_s();
		linefollower();
	}
	if(n==7)
	{   forward();
		_delay_ms(300);
		stop();
		_delay_ms(100);
		linefollower();
	}
	
	if(n==8)
	{
		forward();
		_delay_ms(600);
		stop();
		_delay_ms(1);
		right_degrees(90);
		adjust_right_s();
		linefollower();
	}

	if(n==9)
	{
		forward();
		_delay_ms(200);
		linefollower();
		stop();
		
	}
	if(n==10)
	{  _delay_ms(500);
		drop();
		buzzer_on();
		_delay_ms(2000);
		buzzer_off();
		_delay_ms(500);
		_delay_ms(500);
		right_degrees(180);
		adjust_right_s();
		linefollower();
		
	}
	if(n==11)
	{
		forward();
		_delay_ms(200);
		linefollower();
		stop();
	}
	if(n==12)
	{
		stop();
		forward();
		_delay_ms(800);
		stop();
		_delay_ms(300);
		right_degrees(180);
		n=0;
	}
}
/*
*Name of function: path5()
*Input: none
*Outut: none
*Logic: The bot moves to the pickup point 5 by detecting the nodes and accordingly turning right or left or going straight.
*	comparing the value of the nodes counted is compared with the cases the bot moves as per the instructions given
*Example Call: path5()
*/	
void path5()
{
	
	char c1=ps[4];
	int c=c1;
	right_degrees(135);
	_delay_ms(20);
	soft_right_2();
	linefollower();
    if(n==1)
	{
		forward();
		_delay_ms(300);
		stop();
		_delay_ms(1);
		linefollower();
	}
	if(n==2)
	{
     forward();
     _delay_ms(300);
     stop();
     _delay_ms(1);
     right_degrees(135);
	 linefollower();
	}    
	if(n==3)
	{
		def_servo();
		forward_mm(70);
		_delay_ms(50);
		stop();
		_delay_ms(1000);
		left_degrees(70);
		adjust_left_s();
		stop();
		_delay_ms(1000);
		uart_transmit(c);
		pick();
		buzzer_on();
		_delay_ms(1000);
		buzzer_off();
		left_degrees(70);
		adjust_left_s();
		stop();
		linefollower();
	}
	if(n==4)
	{
		forward();
		_delay_ms(600);
		stop();
		_delay_ms(1);
		left_degrees(70);
		adjust_left_s();
		linefollower();
	}
	if(n==5)
	{   forward();
		_delay_ms(300);
		stop();
		_delay_ms(100);
		linefollower();
	}
	if(n==6)
	{   forward();
		_delay_ms(300);
		stop();
		_delay_ms(100);
		linefollower();
	}
	
		
	if(n==7)
	{
		forward();
		_delay_ms(400);
		stop();
		_delay_ms(1);
		left_degrees(90);
		//adjust_left_s();
		linefollower();
	}
	if(n==8)
	{   forward();
		_delay_ms(300);
		stop();
		_delay_ms(100);
		linefollower();
	}
	if(n==9)
	{
		forward();
		_delay_ms(600);
		stop();
		_delay_ms(1);
		right_degrees(90);
		adjust_right_s();
		linefollower();
	}

	if(n==10)
	{
		forward();
		_delay_ms(200);
		linefollower();
		stop();
		
	}
	if(n==11)
	{  _delay_ms(500);
		drop();
		buzzer_on();
		_delay_ms(2000);
		buzzer_off();
		_delay_ms(500);
		_delay_ms(500);
		right_degrees(180);
		adjust_right_s();
		linefollower();
		
	}
	if(n==12)
	{
		forward();
		_delay_ms(200);
		linefollower();
		stop();
	}
	if(n==13)
	{
		stop();
		forward();
		_delay_ms(800);
		stop();
		_delay_ms(300);
		right_degrees(180);
		n=0;
	}
}
/*
*Name of function: path8()
*Input: none
*Outut: none
*Logic: The bot moves to the pickup point 8 by detecting the nodes and accordingly turning right or left or going straight.
*	comparing the value of the nodes counted is compared with the cases the bot moves as per the instructions given
*Example Call: path8()
*/	
void path8()
{
	char c1=ps[7];
	int c=c1;
	left_degrees(120);
	_delay_ms(20);
	soft_left_2();
	linefollower();
	if(n==1)
	{
		forward();
		_delay_ms(300);
		stop();
		_delay_ms(1);
		linefollower();
	}
	if(n==2)
	{
		forward();
		_delay_ms(300);
		stop();
		_delay_ms(1);
		left_degrees(135);
		linefollower();
	}
	if(n==3)
	{
		def_servo();
		forward_mm(70);
		_delay_ms(50);
		stop();
		_delay_ms(1000);
		right_degrees(70);
		adjust_right_s();
		stop();
		_delay_ms(1000);
		uart_transmit(c);
		pick();
		buzzer_on();
		_delay_ms(1000);
		buzzer_off();
		right_degrees(70);
		adjust_right_s();
		stop();
		linefollower();
	}
	if(n==4)
	{
		forward();
		_delay_ms(600);
		stop();
		_delay_ms(1);
		right_degrees(70);
		adjust_right_s();
		linefollower();
	}
	if(n==5)
	{   forward();
		_delay_ms(300);
		stop();
		_delay_ms(100);
		linefollower();
	}
	if(n==6)
	{   forward();
		_delay_ms(300);
		stop();
		_delay_ms(100);
		linefollower();
	}
	
	
	if(n==7)
	{
		forward();
		_delay_ms(400);
		stop();
		_delay_ms(1);
		right_degrees(70);
		adjust_right_s();
		stop();
		_delay_ms(1);
		linefollower();
	}
	if(n==8)
	{   forward();
		_delay_ms(300);
		stop();
		_delay_ms(100);
		linefollower();
	}
	if(n==9)
	{
		forward();
		_delay_ms(600);
		stop();
		_delay_ms(1);
		left_degrees(80);
		adjust_left_s();
		linefollower();
	}

	if(n==10)
	{
		forward();
		_delay_ms(200);
		linefollower();
		stop();
	}
	if(n==11)
	{  _delay_ms(500);
		drop();
		buzzer_on();
		_delay_ms(2000);
		buzzer_off();
		_delay_ms(500);
		_delay_ms(500);
		right_degrees(180);
		adjust_right_s();
		linefollower();
		
	}
	if(n==12)
	{
		forward();
		_delay_ms(200);
		linefollower();
		stop();
	}
	if(n==13)
	{
		stop();
		forward();
		_delay_ms(800);
		stop();
		_delay_ms(300);
		right_degrees(180);
		n=0;
	}
}
/*
*Name of function: path7()
*Input: none
*Outut: none
*Logic: The bot moves to the pickup point 7 by detecting the nodes and accordingly turning right or left or going straight.
*	comparing the value of the nodes counted is compared with the cases the bot moves as per the instructions given
*Example Call: path7()
*/	
void path7()
{
	
char c1=ps[6];
int c=c1;
left_degrees(120);
_delay_ms(20);
soft_left_2();
linefollower();
if(n==1)
{
	forward();
	_delay_ms(300);
	stop();
	_delay_ms(1);
	linefollower();
}
if(n==2)
{
	forward();
	_delay_ms(300);
	stop();
	_delay_ms(1);
	left_degrees(135);
	linefollower();
}
if(n==3)
{
	forward();
	_delay_ms(300);
	stop();
	_delay_ms(1);
	linefollower();
}
if(n==4)
{
	def_servo();
	forward_mm(90);
	_delay_ms(50);
	stop();
	_delay_ms(1000);
	right_degrees(75);
	//adjust_right_s();
	stop();
	_delay_ms(20);
	
	uart_transmit(c);
	pick();
	buzzer_on();
	_delay_ms(1000);
	buzzer_off();
	right_degrees(70);
	adjust_right_s();
	stop();
	linefollower();
}
if(n==5)
{
	forward();
	_delay_ms(300);
	stop();
	_delay_ms(100);
	linefollower();
}
if(n==6)
{
	forward();
	_delay_ms(600);
	stop();
	_delay_ms(1);
	right_degrees(70);
	adjust_right_s();
	linefollower();
}
if(n==7)
{   forward();
	_delay_ms(300);
	stop();
	_delay_ms(100);
	linefollower();
}
if(n==8)
{   forward();
	_delay_ms(300);
	stop();
	_delay_ms(100);
	linefollower();
}


if(n==9)
{
	forward();
	_delay_ms(400);
	stop();
	_delay_ms(1);
	right_degrees(70);
	adjust_right_s();
	linefollower();
}
if(n==10)
{   forward();
	_delay_ms(300);
	stop();
	_delay_ms(100);
	linefollower();
}
if(n==11)
{
	forward();
	_delay_ms(600);
	stop();
	_delay_ms(1);
	left_degrees(90);
	adjust_left_s();
	stop();
	linefollower();
}

if(n==12)
{
	forward();
	_delay_ms(200);
	linefollower();
	stop();
	
}
if(n==13)
{  _delay_ms(500);
	drop();
	buzzer_on();
	_delay_ms(2000);
	buzzer_off();
	_delay_ms(500);
	_delay_ms(500);
	right_degrees(180);
	adjust_right_s();
	linefollower();
	
}
if(n==14)
{
	forward();
	_delay_ms(200);
	linefollower();
	stop();
}
if(n==15)
{
	stop();
	forward();
	_delay_ms(800);
	stop();
	_delay_ms(300);
	right_degrees(180);
	n=0;
}
}
/*
*Name of function: path6()
*Input: none
*Outut: none
*Logic: The bot moves to the pickup point 6 by detecting the nodes and accordingly turning right or left or going straight.
*	comparing the value of the nodes counted is compared with the cases the bot moves as per the instructions given
*Example Call: path6()
*/	
void path6()
{
		
		char c1=ps[5];
		int c=c1;
		right_degrees(135);
		_delay_ms(20);
		soft_right_2();
		linefollower();
		if(n==1)
		{
			forward();
			_delay_ms(300);
			stop();
			_delay_ms(1);
			linefollower();
		}
		if(n==2)
		{
			forward();
			_delay_ms(300);
			stop();
			_delay_ms(1);
			right_degrees(135);
			linefollower();
		}
		if(n==3)
		{
			forward();
			_delay_ms(300);
			stop();
			_delay_ms(1);
			linefollower();
		}
		if(n==4)
		{
			def_servo();
			forward_mm(70);
			_delay_ms(50);
			stop();
			_delay_ms(1000);
			left_degrees(70);
			adjust_left_s();
			stop();
			_delay_ms(20);
			
			uart_transmit(c);
			pick();
			buzzer_on();
			_delay_ms(1000);
			buzzer_off();
			left_degrees(70);
			adjust_left_s();
			stop();
			linefollower();
		}
		if(n==5)
		{
			forward();
			_delay_ms(300);
			stop();
			_delay_ms(100);
			linefollower();
		}
		if(n==6)
		{
			forward();
			_delay_ms(600);
			stop();
			_delay_ms(1);
			left_degrees(70);
			adjust_left_s();
			linefollower();
		}
		if(n==7)
		{   forward();
			_delay_ms(300);
			stop();
			_delay_ms(100);
			linefollower();
		}
		if(n==8)
		{   forward();
			_delay_ms(300);
			stop();
			_delay_ms(100);
			linefollower();
		}
		
		
		if(n==9)
		{
			forward();
			_delay_ms(400);
			stop();
			_delay_ms(1);
			left_degrees(90);
			//adjust_left_s();
			linefollower();
		}
		if(n==10)
		{   forward();
			_delay_ms(300);
			stop();
			_delay_ms(100);
			linefollower();
		}
		if(n==11)
		{
			forward();
			_delay_ms(600);
			stop();
			_delay_ms(1);
			right_degrees(90);
			adjust_right_s();
			linefollower();
		}

		if(n==12)
		{
			forward();
			_delay_ms(200);
			linefollower();
			stop();
			
		}
		if(n==13)
		{  _delay_ms(500);
			drop();
			buzzer_on();
			_delay_ms(2000);
			buzzer_off();
			_delay_ms(500);
			_delay_ms(500);
			right_degrees(180);
			adjust_right_s();
			linefollower();
			
		}
		if(n==14)
		{
			forward();
			_delay_ms(200);
			linefollower();
			stop();
		}
		if(n==15)
		{
			stop();
			forward();
			_delay_ms(800);
			stop();
			_delay_ms(300);
			right_degrees(180);
			n=0;
		}
}
/*
*Name of function: path10()
*Input: none
*Outut: none
*Logic: The bot moves to the pickup point 10 by detecting the nodes and accordingly turning right or left or going straight.
*	comparing the value of the nodes counted is compared with the cases the bot moves as per the instructions given
*Example Call: path10()
*/	
void path10()
{
	
	char c1=ps[9];
	int c=c1;
	left_degrees(70);
	_delay_ms(20);
	adjust_left_s();
	linefollower();
	if(n==1)
	{
		forward();
		_delay_ms(300);
		stop();
		_delay_ms(1);
		linefollower();
	}
	if(n==2)
	{
		forward();
		_delay_ms(500);
		stop();
		left_degrees(70);
		adjust_left_s();
		linefollower();
	}
	if(n==3)
	{
		def_servo();
		forward_mm(50);
		_delay_ms(50);
		stop();
		_delay_ms(1000);
		right_degrees(80);
		_delay_ms(1000);
		uart_transmit(c);
		pick();
		buzzer_on();
		_delay_ms(1000);
		buzzer_off();
		right_degrees(80);
		adjust_right_s();
		linefollower();
	}
	if(n==4)
	{
		forward();
		_delay_ms(400);
		stop();
		_delay_ms(1);
		right_degrees(70);
		adjust_right_s();
		linefollower();
	}
	if(n==5)
	{   forward();
		_delay_ms(300);
		stop();
		_delay_ms(100);
		linefollower();
	}
	
	if(n==6)
	{
		forward();
			_delay_ms(450);
			stop();
			_delay_ms(1);
			left_degrees(70);
			adjust_left_s();
			linefollower();
	}

	if(n==7)
	{
		forward();
		_delay_ms(200);
		linefollower();
		stop();
		
	}
	if(n==8)
	{  _delay_ms(500);
		drop();
		buzzer_on();
		_delay_ms(2000);
		buzzer_off();
		_delay_ms(500);
		_delay_ms(500);
		right_degrees(180);
		adjust_right_s();
		linefollower();
		
	}
	if(n==9)
	{
		forward();
		_delay_ms(200);
		linefollower();
		stop();
	}
	if(n==10)
	{
		stop();
		forward();
		_delay_ms(800);
		stop();
		_delay_ms(300);
		right_degrees(180);
		n=0;
	}
}
/*
*Name of function: path9()
*Input: none
*Outut: none
*Logic: The bot moves to the pickup point 9 by detecting the nodes and accordingly turning right or left or going straight.
*	comparing the value of the nodes counted is compared with the cases the bot moves as per the instructions given
*Example Call: path9()
*/	
void path9()
{
	
	char c1=ps[8];
	int c=c1;
	left_degrees(60);
	_delay_ms(20);
	adjust_left_s();
	linefollower();
	if(n==1)
	{
		forward();
		_delay_ms(300);
		stop();
		_delay_ms(1);
		linefollower();
	}
	if(n==2)
	{
		forward();
		_delay_ms(500);
		stop();
		left_degrees(90);
		adjust_left_s();
		linefollower();
	}
	if(n==3)
	{
		forward();
		_delay_ms(300);
		stop();
		_delay_ms(1);
		linefollower();
	}
	if(n==4)
	{
		def_servo();
		forward_mm(60);
		_delay_ms(50);
		stop();
		_delay_ms(1000);
		right_degrees(70);
		_delay_ms(1000);
		uart_transmit(c);
		pick();
		buzzer_on();
		_delay_ms(1000);
		buzzer_off();
		right_degrees(75);
		_delay_ms(20);
		adjust_right_s();
		linefollower();
	}
	if(n==5)
	{
		forward();
		_delay_ms(300);
		stop();
		_delay_ms(1);
		linefollower();
	}
	if(n==6)
	{
		forward();
		_delay_ms(600);
		stop();
		_delay_ms(1);
		right_degrees(90);
		adjust_right_s();
		linefollower();
	}
	if(n==7)
	{   forward();
		_delay_ms(300);
		stop();
		_delay_ms(100);
		linefollower();
	}
	
	if(n==8)
	{
		forward();
		_delay_ms(400);
		stop();
		_delay_ms(1);
		left_degrees(60);
		adjust_left_s();
		linefollower();
	}

	if(n==9)
	{
		forward();
		_delay_ms(200);
		linefollower();
		stop();
		
	}
	if(n==10)
	{  _delay_ms(500);
		drop();
		buzzer_on();
		_delay_ms(2000);
		buzzer_off();
		_delay_ms(500);
		_delay_ms(500);
		right_degrees(180);
		adjust_right_s();
		linefollower();
		
	}
	if(n==11)
	{
		forward();
		_delay_ms(200);
		linefollower();
		stop();
	}
	if(n==12)
	{
		stop();
		forward();
		_delay_ms(800);
		stop();
		_delay_ms(300);
		right_degrees(180);
		n=0;
	}
}
/*
*Name of function: path11()
*Input: none
*Outut: none
*Logic: The bot moves to the pickup point 11 by detecting the nodes and accordingly turning right or left or going straight.
*	comparing the value of the nodes counted is compared with the cases the bot moves as per the instructions given
*Example Call: path11()
*/	
void path11()
{
	
	char c1=ps[10];
	int c=c1;
	left_degrees(60);
	_delay_ms(20);
	adjust_left_s();
	linefollower();
	if(n==1)
	{
		forward();
		_delay_ms(100);
		stop();
		_delay_ms(1);
		linefollower();
	}
	if(n==2)
	{
		forward();
		_delay_ms(400);
		stop();
		right_degrees(75);
		adjust_right_s();
		linefollower();
	}
	
	if(n==3)
	{
		def_servo();
		forward();
		_delay_ms(500);
		stop();
		_delay_ms(1000);
		left_degrees(80);
		_delay_ms(1000);
		uart_transmit(c);
		pick();
		buzzer_on();
		_delay_ms(1000);
		buzzer_off();
		left_degrees(100);
		linefollower();
}
	
	if(n==4)
	{
		forward();
		_delay_ms(600);
		stop();
		_delay_ms(1);
		left_degrees(75);
		adjust_left_s();
		linefollower();
	}
	
	if(n==5)
	{
		forward();
		_delay_ms(200);
		linefollower();
		stop();
	}
	if(n==6)
	{
		forward();
		_delay_ms(400);
		stop();
		_delay_ms(1);
		left_degrees(70);
		adjust_left_s();
		linefollower();
	}
	if(n==7)
	{
		forward();
		_delay_ms(200);
		linefollower();
		stop();
		
	}
	if(n==8)
	{  _delay_ms(500);
		drop();
		buzzer_on();
		_delay_ms(2000);
		buzzer_off();
		_delay_ms(500);
		_delay_ms(500);
		right_degrees(180);
		adjust_right_s();
		linefollower();
		
	}
	if(n==9)
	{
		forward();
		_delay_ms(200);
		linefollower();
		stop();
	}
	if(n==10)
	{
		stop();
		forward();
		_delay_ms(800);
		stop();
		_delay_ms(300);
		right_degrees(180);
		n=0;
	}
}

/*
*Name of function: path12()
*Input: none
*Outut: none
*Logic: The bot moves to the pickup point 12 by detecting the nodes and accordingly turning right or left or going straight.
*	comparing the value of the nodes counted is compared with the cases the bot moves as per the instructions given
*Example Call: path12()
*/	


void path12()
{
	char c1=ps[11];
	int c=c1;
	printf("%d",c);
	left_degrees(60);
	_delay_ms(20);
	adjust_left_s();
	linefollower();
	if(n==1)
	{
		forward();
		_delay_ms(100);
		stop();
		_delay_ms(1);
		linefollower();
	}
	if(n==2)
	{
		forward();
		_delay_ms(400);
		stop();
		right_degrees(70);
		adjust_right_s();
		linefollower();
	}
	if(n==3)
	{
		forward();
		_delay_ms(300);
		stop();
		_delay_ms(1);
		linefollower();
	}
	if(n==4)
	{
		def_servo();
		forward();
		_delay_ms(500);
		stop();
		_delay_ms(1000);
		left_degrees(80);
		_delay_ms(1000);
		uart_transmit(c);
		pick();
		buzzer_on();
		_delay_ms(1000);
		buzzer_off();
		left_degrees(100);
		linefollower();
	}
	if(n==5)
	{
		forward();
		_delay_ms(300);
		stop();
		_delay_ms(1);
		linefollower();
	}
	if(n==6)
	{
		forward();
		_delay_ms(600);
		stop();
		_delay_ms(1);
		left_degrees(70);
		adjust_left_s();
		linefollower();
	}
	
	if(n==7)
	{
		forward();
		_delay_ms(200);
		linefollower();
		stop();
	}
	if(n==8)
	{
		forward();
		_delay_ms(400);
		stop();
		_delay_ms(1);
		left_degrees(75);
		adjust_left_s();
		linefollower();
	}
	if(n==9)
	{
		forward();
		_delay_ms(200);
		linefollower();
		stop();
		
	}
	if(n==10)
	{  _delay_ms(500);
		drop();
		buzzer_on();
		_delay_ms(2000);
		buzzer_off();
		_delay_ms(500);
		_delay_ms(500);
		right_degrees(180);
		adjust_right_s();
		linefollower();
		
	}
	if(n==11)
	{
		forward();
		_delay_ms(200);
		linefollower();
		stop();
	}
	if(n==12)
	{
		stop();
		forward();
		_delay_ms(800);
		stop();
		_delay_ms(300);
		right_degrees(180);
		n=0;
	}
}

/////////////////end of code//////////////////