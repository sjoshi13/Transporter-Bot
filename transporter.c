
/*
* Team Id: e-YRC#783
* Author List: Shruti Joshi, Shivangi Mishra,Jyotsna Sharma,Ram Mahesh
 
* Filename: transporterbot.c
* Theme: Transporter Bot
* Functions: main();
* Global Variables: unsigned int value,right_speed,left_speed; char ps1[12]; int position,flag,total_blocks,total_blocks1,i4;
*/
#define cut_off 28
#define F_CPU 14745600
#include<avr/io.h>
#include<avr/interrupt.h>
#include<util/delay.h>
#include "lcd.h"
#include "main.h"
unsigned int value,right_speed,left_speed;
char ps1[12]={'b',' ','y',' ','b',' ','r',' ','r',' ',' ','g'};//placement sequence

int position,flag,total_blocks,total_blocks1,i4;
// 8bit data format
/*   Function Name: main()
*    Input:     None 
*    Output:    Results in the motion of bot across the arena, collect blocks and drop them on the deposite area  
*    Logic:     firstly we count what all values are not null and then transveres the paths using the functions we defined.
*				when finally all the blocks are collected, a buzzer rings to indicate the end of the task.  
*    Example Call: buzzer_on();  
*/
void main()
{
	for(i4=0;i4<12;i4++)
	{
		if(ps1[i4]!=' ')
		total_blocks=total_blocks+1;//count increased by one if a block is at specified position in the sequence
	}
	init_devices();
	lcd_set_4bit();
	lcd_init();
	uart_init();
	
    _delay_ms(2000);
	for (int i=0;i<12;i++)
	{
		if(ps1[i]==' ')
		continue;
		else
		{
			position=i+1;
			path(position);
			_delay_ms(2000);
			total_blocks1=total_blocks1+1;
		}
	}
	if(total_blocks==total_blocks1)//if number of blocks counted till now is equal to the total blocks in placement sequence
	buzzer_on();
        _delay_ms(5000);
        buzzer_off();
}
/////////////////////////end of code////////////////////////////