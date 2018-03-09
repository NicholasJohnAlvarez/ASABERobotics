/* Ping Sensor & ActivityBot program to demonstrate a simple proportional feedback control system to
 * allow the ActivityBot to navigate by using echo location to sense the distance to an adjacent wall
 * and to maintain a constant distance between the Ping sensor and the wall while traveling.
 * 
 * The program measures the forward distance to the next obstacle, assuming it is the wall and tracks
 * forward travel by using wheel encoder ticks.  The final stopping position is controlled by using
 * the Ping sensor.
 * 
 * This version of the program is incomplete, and does not fully implement: 
 * Ping reading outlier control
 * Missing wall section detection or mitigation
 * detection of obstacles other than walls
 * use of a full PID control algorithm.
 * 
 * Test the PING))) sensor before using it to navigate with the ActivityBot.
 * 9 waypoints with 256 inches (6.5 meters) of travel at 75 ticks/s (244mm/s) takes ~45 seconds
 *
 *  http://learn.parallax.com/activitybot/build-and-test-ping-sensor-circuit
 *
 * IBM’s C library reference: https://www.ibm.com/support/knowledgecenter/ssw_ibm_i_71/rtref/sc41560702.htm  
 * Example program demonstrating the use of the Parallax Laser Range Finder
 * in a closed-loop feedback control system using proportional control to keep the 
 * Activity Bot at a fixed distance, FORWARD_SETPOINT_MM, away from an object directly
 * in front of the Bot.

  Author: D.C. Slaughter, UC Davis. January 6, 2018
  
  Code written in the C language for the Parallax Activity Board WX

  Setup the Laser Range Finder (LRF) Module. The response rate is about 4 Hz (supposed to be 5 Hz).
  
  When the LRF powers on, it launches an auto-baud routine to determine the host's baud rate. 
  It will stay in this state until a "U" ($55) character is sent by the host. 
  On a cold start, the program must transmit a 'U' character to let the LRF identify the baud rate.
  
  Output:
  If the baud rate of the LRF has not been set (on cold start), then the response to the 
  'U' command will be 2 characters ^M:
  if the baud rate has been set (warm program reload), then the response to the 'U'
  command will be 3 characers ?^M:
  
  The 'B' command causes the LRF to take a single measurement and reply with a 2-byte integer
  value in mm units followed by 2 characters (the last on is a :).
 */

#include "simpletools.h"                      // Include simpletools header
#include "abdrive360.h"
#include "ping.h"                             // Include ping header
#include "fdserial.h"
#include "servo.h"

// Constant values used in the system control
#define TRAVELSPEED 50 
#define SIDE_SETPOINT_MM 226   //  was 100 
#define KP_FORWARD 1
#define KP_SiDEWAYS 1
#define PING_FRONT_PIN 16
#define PING_BACK_PIN 17
#define MM_PER_TICK 3.25

// Define some constants used in this program
//#define MM_PER_TICK 3.25
#define FORWARD_SETPOINT_MM 736    // 26 cm from wall 285
#define rxPin    02  // Serial input (connects to the LRF's SOUT pin)
#define txPin    03  // Serial output (connects to the LRF's SIN pin)
#define BAUD  38400  // Baud rate for Laser Range Finder

// Global variables for cogs
static volatile int distance_mm, Front_mm, Back_mm, heading_mm, state = -1;        
static unsigned int Dstack[40 + 40]; // stacks for cogs, add more memory if you add variables or code.

void getPing() { // separate cog function for continuous Ping distance measurements
  int echoTimeFront, echoTimeBack;  
  
  while(state > -2) {
    pause(5);
    echoTimeFront = ping(PING_FRONT_PIN);
    pause(5);
    echoTimeBack  = ping(PING_BACK_PIN);
    Front_mm = (echoTimeFront*1713 - 38395 + 5000)/10000; // Regression equation for echo time to mm
    Back_mm  = (echoTimeBack*1713 - 38395 + 5000)/10000; // Regression equation for echo time to mm
    heading_mm = Front_mm - Back_mm;
    distance_mm = (Front_mm + Back_mm +1) / 2;
      
   }  
}  



//Global variables for cog 0
int Right_mm, Left_mm;
float oldHeading = 0.0, relativeHeading = 0.0;

void rightTurn(){
  
   //pause(1000);
   
   drive_speed(40, 40); // send the control signal to the wheel servos 
   pause(4000);
   
   drive_speed(30, 0); // send the control signal to the wheel servos 
   pause(1650);        //

   
   drive_speed(40, 40); // send the control signal to the wheel servos 
   pause(3700);   
   
   drive_speed(30, 0); // send the control signal to the wheel servos 
   pause(1650);    
   
   drive_speed(30, 30); // send the control signal to the wheel servos 
   pause(3000);
   //Forward_Error = 100; 
         
}


void leftTurn(){
  
   //pause(1000);
   
   drive_speed(40, 40); // send the control signal to the wheel servos 
   pause(4000);
   
   drive_speed(0, 30); // send the control signal to the wheel servos 
   pause(1650);        //

   
   drive_speed(40, 40); // send the control signal to the wheel servos 
   pause(3700);   
   
   drive_speed(0, 30); // send the control signal to the wheel servos 
   pause(1650);    
   
   drive_speed(30, 30); // send the control signal to the wheel servos 
   pause(3000);
   //Forward_Error = 100; 
         
}
    

int main()                                    // main function
{
  int sideError[2], sideErrorSum, sideErrorChange, controlSignal, checkForward =0;
  int i, t, state = -1;
  float Kp = 0.4;
  
  int turnState = 0;
  
  drive_speed(0, 0);
  state = -1;
  i = cogstart(getPing, NULL, Dstack, sizeof(Dstack)); // start getPing process in new cog
    
  sideErrorSum = 0;
  t = CNT;
  
  // laser range finder variables
  int i2, j, distance_mm2;
  char LRF_in1[1] ;
  float Kp2 = 1.0;
  int Forward_Error=100, Control_Signal;
 

  
  
    // Setup serial communication with the LRF
  fdserial *lrf = fdserial_open(rxPin, txPin, FDSERIAL_MODE_NONE, BAUD);  //Open a full duplex serial connection. 
  
  fdserial_txChar(lrf, 'U');
// When the LRF has initialized and is ready, it will send a ':' character, 

  while((i2 = fdserial_rxCount(lrf)) < 2);
  pause(4); // delay in case of a warm start, 3 character, response.
  i2 = fdserial_rxCount(lrf);
//  print("count1 = %d \n", i);
  while(i2 > 0) {
      LRF_in1[0] = fdserial_rxChar(lrf);
//      print("%c", LRF_in1[0]);          // Display result
      i2--;
    }   
    
    
    
  
   while(1) {        // Travelling straight, tracking wall 
   
//   print("distance_mm=%d, heading_mm=%d\n", distance_mm, heading_mm);
//   pause(50);
    
    // The heading_mm is the difference in mm between the Front_mm and Back_mm echo locations.
    // As an angle, theta_radians = arctan(heading_mm / 177 mm).  Since heading_mm and theta
    // are correlated, the arctan function is not used.
    // distance_mm is the average distance from the robot to the wall.
    // Using the (SIDE_SETPOINT_MM - distance_mm) as the heading setpoint seems to give reasonable performance
    // It avoids the problem of oversteering, lack of perpendicularity of ping echo, and crashing into the wall.
    
              sideError[0] = (SIDE_SETPOINT_MM - distance_mm) - heading_mm; // 
              if(Front_mm < 30) controlSignal = 30;  // Risk of crashing when Front_mm < 30 mm.
              else { if(abs(heading_mm) > 20) controlSignal = (int) -(Kp * (float) heading_mm +0.5);
                     else  controlSignal = (int) (Kp * (float) sideError[0] +0.5); 
                   }
              if(controlSignal > 6) controlSignal = 6;
              if(controlSignal < -6) controlSignal = -6;
              
              if(abs(controlSignal) > 1)
                drive_speed(TRAVELSPEED +controlSignal, TRAVELSPEED -controlSignal );
              else drive_speed(TRAVELSPEED, TRAVELSPEED ); 
                    
        fdserial_txChar(lrf, 'B'); // gets a two byte measurment in millimeters 
                  
                  
   while((i2 = fdserial_rxCount(lrf)) < 4);  // 1024mm > is good?
      LRF_in1[0] = fdserial_rxChar(lrf);    // gets first byte
      distance_mm2 = LRF_in1[0] << 8;       // shifts first byte over by a byte
      i2--;                                 
      LRF_in1[0] = fdserial_rxChar(lrf);     // gets second byte
      distance_mm2 = distance_mm2 | LRF_in1[0];     // combines the two bytes
//      print("%d %d\n", distance_mm, i);          // Display result
      i2--;
  while(i2 > 0) {   // making sure reading is not negative
      LRF_in1[0] = fdserial_rxChar(lrf);
      i2--;
    }      
    
    Forward_Error = FORWARD_SETPOINT_MM - distance_mm2; // calculate the distance error
    Control_Signal = (int) (Kp2 * (float) Forward_Error +0.5); // calculate the feedback control signal
    if(Control_Signal > 125) Control_Signal = 125;  // bound the control signal to be within the servo limits
    if(Control_Signal < -125) Control_Signal = -125;
    //drive_speed(-Control_Signal, -Control_Signal); // send the control signal to the wheel servos
    
    if(!(Forward_Error > 25 || Forward_Error < -25 )){ //originally was 2 
    //we can modify the code above to account for turning a distance further away from the wall (distance from side wall to end of board)
    //  
      
        if(turnState == 3){
          drive_speed(80, 80); // send the control signal to the wheel servos 
          pause(2000);
   
          drive_speed(0,0);
          return 1;
        }          
        else if(turnState % 2 == 0){
          rightTurn();
        }          
        else{
          leftTurn();
        }   
        turnState++;     
         Forward_Error = 100; 
         
         // we need another if statement to account for the large turn 
         
         // if ( Forward_Error
    }      
            }                
}
