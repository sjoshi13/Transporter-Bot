/*Team Id:eYRC#783
* Author List: Shruti Joshi
* Filename:Stepper_rotating_structure
* Theme:Transporter Bot
* Functions:void setup(),void loop().323
* Global Variables:char ss[],char color,int index,
*/
#include <Stepper.h> 
char ss[4]={'y','g','r','b'};//ss is an array the stores the structure sequence.Position 0 is north,1 west,2 south,3east.'r' is red 'y' yellow 'b' blue 'g'green
char color;//variable to store character recieved from xbee
int index=0;//this variable stores the index of color in ss[]
Stepper myStepper(200,7,8,9,10);// initialize the stepper library on pins 4 through 7;stepper has 200 steps per revolution
/*Function Name:setup
Input:None
Output:None
Logic:This function initializes stepper speed and sets baud rate for serial monitor.It runs only once.
Example Call:setup();
*/ 
void setup() 
{
myStepper.setSpeed(11);
Serial.begin(9600);
}
/*Function Name:loop
Input:None
Output:None
Logic:This function runs repeatedly and so the commands given in this functions get executed periodically.
The function matches the value of color with every character in ss[].The index of matching character is stored in 
index variable.The stepper is then made to rotate as many steps as will bring the color corresponding c to position 0(north).
Example Call:loop();
*/ 
void loop()
{

char ss_copy[4];//stores a copy of structure sequence of previous loop
for(int i=0;i<4;i++)
{
 ss_copy[i]=ss[i];//copying the values
  }
if(Serial.available()>0)//checking if xbee has recieved any data 
{

  digitalWrite(7,HIGH);
  digitalWrite(8,HIGH);
  digitalWrite(9,HIGH);
  digitalWrite(10,HIGH);
  color=Serial.read();
  Serial.println();//the character recieved is stored in c
}
for(int i=0;i<4;i++)
{
 if(ss[i]==color)//matching the value of c with eaach element in array ss[]
 {
 //getting the index of variable received
 index=i;
 break;//break if match found 
 }
}
if(index==0){
myStepper.step(0);
   digitalWrite(7,LOW);
  digitalWrite(8,LOW);
  digitalWrite(9,LOW);
  digitalWrite(10,LOW);}
if(index==1){
myStepper.step(50);
   digitalWrite(7,LOW);
  digitalWrite(8,LOW);
  digitalWrite(9,LOW);
  digitalWrite(10,LOW);}
if(index==2){
myStepper.step(100);
digitalWrite(7,LOW);
  digitalWrite(8,LOW);
  digitalWrite(9,LOW);
  digitalWrite(10,LOW);}
if(index==3){
 myStepper.step(-50);
    digitalWrite(7,LOW);
  digitalWrite(8,LOW);
  digitalWrite(9,LOW);
  digitalWrite(10,LOW);}
ss[0]=ss_copy[index];//the structure sequence is now updated,0 position has color at index m
if((index+1)<4)
ss[1]=ss_copy[index+1];
else
ss[1]=ss_copy[(index+1)-4];//since maximum index is 3
if((index+2)<4)
ss[2]=ss_copy[index+2];//updating color at 2
else
ss[2]=ss_copy[(index+2)-4];//since maximum index is 3
if((index+3)<4)
ss[3]=ss_copy[index+3];//updating color at west 
else
ss[3]=ss_copy[(index+3)-4];//since maximum index is 3

}
