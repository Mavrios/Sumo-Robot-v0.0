#include <NewPing.h>

//UltraSonic
#define MAX_DISTANCE 200
NewPing sonar[3] = {            // Sensor object array.
  // Each sensor's trigger pin, echo pin, and max distance to ping. 
  NewPing(A0, A1, MAX_DISTANCE),  // sonar[0] ---> Front Left
  NewPing(A3, A2, MAX_DISTANCE),  // sonar[1] ---> Front Center
  NewPing(A5, A4, MAX_DISTANCE),// sonar[2] ---> Front Right
};
int Distance[3];


const int MaximumRange = 40; // Ultrasonic sensor maximum range needed, in cm
const int AverageRange = 35; // Ultrasonic sensor average range needed, in cm
const int MaxSearchRange = 100;

#define R1_pin   5          // Forward right motor pin
#define R2_pin   6          // Backward right motor pin
//#define pwm_R    6         // Right motor pwm
#define L1_pin   9         // Forward left motor pin
#define L2_pin   10         // Backward left motor pin
//#define pwm_L    13         // Left motor pwm


//#define EdgeBackPin    4  //Edge Sensor Rear Center
#define EdgeRightPin   2  //Edge Sensor Front Left
//#define EdgeLeftPin    3  //Edge Sensor Front Right

//#define BuzzerPin 100 //Buzzer sound output pin


bool BackEdgeState = 0;    //Logic variable for recording if this edge was triggered or not
bool RightEdgeState = 0;   //Logic variable for recording if this edge was triggered or not
bool LeftEdgeState = 0;    //Logic variable for recording if this edge was triggered or not
bool EdgeSensed = 0;
bool UltraSensed = 0;
bool EnemySensed = 0;

int StuckCounter = 0;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//____SETUP SECTION_________________________________________________________________________________________________
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup(){
  pinMode(L1_pin, OUTPUT);
  pinMode(L2_pin, OUTPUT);
  pinMode(R1_pin, OUTPUT);
  pinMode(R2_pin, OUTPUT);
  //pinMode(pwm_L, OUTPUT);
//  pinMode(pwm_R, OUTPUT);

//  pinMode(BuzzerPin, OUTPUT);
  SitStill();  //This is a function written for troubleshooting purposes
  /*do
  {
  CheckSensors();
  } while(EnemySensed == false);*/
  Countdown();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//____ LOOP SECTION_________________________________________________________________________________________________________
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void loop(){
   
  CheckSensors(); //Review all input sensors for fresh data
  if (EdgeSensed == true){ //If Edge is Detected
    //Buzz();
    StayInRing(); // Move & reorient the robot away from ring edge
  }else if(UltraSensed == true){  //Enemy Detected via Ultrasonic sensor
    // Buzz();
    Persue(); // Reorient the robot with intent to attack
  }
  else if(EnemySensed == true){  //Enemy Detected via Button sensor
     //Buzz();
     Attack(); // Brief charge forward
  
   
     /*StuckCounter = StuckCounter + 1; //Stuck counter is to prevent stalemate when 2 robots find each other.
     if(StuckCounter >random(20,40)){
       if(random(1,2)<2){
         MoveBackwardRight();
       }else{
         MoveBackwardLeft();
       }
       delay(random(50,500));
       //MoveRandom();
       delay(random(1,100));
       StuckCounter = 0;
       }*/
  }else{
    Search();  // Move in roving pattern seeking enemy
  }


}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//____ FUNCTIONS SECTION___________________________________________________________________________________________________
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void UltraSense(){ 
  for (uint8_t i = 0; i < 3; i++) { // Loop through each sensor and display results.
    delay(30); // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
    Distance[i] = sonar[i].ping_cm();
  }
} 
 
void CheckSensors(){ //Review all input sensors for fresh data and set the robot state

//  BackEdgeState = digitalRead(EdgeBackPin);
  RightEdgeState = digitalRead(EdgeRightPin);
//  LeftEdgeState = digitalRead(EdgeLeftPin);
  UltraSense();
 
  if(RightEdgeState == LOW){
    
    EdgeSensed = true; //Edge positively sensed
  }else {
    EdgeSensed = false;
  }

  if(Distance[1] < MaximumRange){
    UltraSensed = true;  //Enemy Detected via Ultrasonic sensor
  }else{
    UltraSensed = false;
  }
  
  if(Distance[0] < AverageRange || Distance[2] < AverageRange ){
    EnemySensed = true;  //Enemy Detected via Button sensor
  }
  else{
    EnemySensed = false;
  }
}
/*
void Buzz() { //Increasing the duration of the buzz changes the volume drastically. Set delay to 80+ms for a loud beep!
//   digitalWrite(BuzzerPin, HIGH);
   delay(5);
//   digitalWrite(BuzzerPin, LOW);
}

void BuzzLong() {  //Increasing the duration of the buzz changes the volume drastically. Set delay to 80+ms for a loud beep!
//   digitalWrite(BuzzerPin, HIGH);
   delay(50);
//   digitalWrite(BuzzerPin, LOW);
}

*/
void StayInRing(){   // Move & reorient the robot away from ring edge
  if(RightEdgeState == LOW ){
    MoveBackward();
    delay (300); //Duration of this motion in milliseconds
    RotateLeft();
    delay (200); //Duration of this motion in milliseconds
  }/*else if(RightEdgeState == HIGH){
    MoveBackwardRight();
    delay (random(300,600)); //Duration of this motion in milliseconds
    RotateLeft(); 
    delay (random(10,550)); //Duration of this motion in milliseconds
  }else if(LeftEdgeState == HIGH){
    MoveBackwardLeft();
    delay (random(300,600)); //Duration of this motion in milliseconds
    RotateRight(); 
     delay (random(10,450)); //Duration of this motion in milliseconds
  }*/
  /*if(BackEdgeState == HIGH) {
    MoveForwardFast();
    delay (random(500,1500)); //Duration of this motion in milliseconds
  }*/
}

void Persue(){ // Reorient the robot with intent to attack
  MoveForwardFast();
  }

void Attack(){// Brief charge forward
  if ( Distance[0] < AverageRange) {
    RotateLeft();
  }
  else if ( Distance[2] < AverageRange) {
    RotateRight();
  }
}

void Search(){ // Move in roving pattern seeking enemy
  int min = 0;
  for(uint8_t i = 0; i < 3; i++){
    if(Distance[0] > MaxSearchRange && Distance[1] > MaxSearchRange && Distance[2] > MaxSearchRange){
      min = -1;
      break;
    }
    if(Distance[min] >= Distance[i]){
      min = i;
    }
  }
  switch(min){
    case 0:
      PivotLeft();
      //delay(50);
    //  break;
      
    case 1:
      MoveForwardSlow();
      //delay(50);
      //break;
      
    case 2:
      PivotRight();
      //delay(50);
     // break;
      

 //   default:
   //   MoveRandom();
     // delay(random(20,50));
    
  }
  
}  

void Countdown(){ //This pauses the robot for 5 seconds (5000 milliseconds) after it is turned on, per competition requirements. Then it beeps the 5 sec countdown. 
//  Buzz();
  delay (995); 
//  Buzz();
  delay (995); 
//  Buzz();
  delay (995); 
//  Buzz();
//  BuzzLong(); 
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//____ MOTION COMMANDS (Just a sublevel of the functions)__________________________________________________________________
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void PivotLeft(){
  // Set Left Motors forward
  digitalWrite(L2_pin, LOW);
  digitalWrite(L1_pin, HIGH);
  // Set Right Motors forward
  digitalWrite(R2_pin, LOW);
  digitalWrite(R1_pin, HIGH);
  

}

void PivotRight(){
  // Set Left Motors forward
  digitalWrite(L2_pin, LOW);
  digitalWrite(L1_pin, HIGH);
  // Set Right Motors forward
  digitalWrite(R2_pin, LOW);
  digitalWrite(R1_pin, HIGH);

}  

void RotateRight(){
  // Set Left Motors  forward
  digitalWrite(L1_pin, HIGH);
  digitalWrite(L2_pin, LOW);
  // Set Right Motors backward
  digitalWrite(R1_pin, LOW);
  digitalWrite(R2_pin, HIGH);


} 

void RotateLeft(){
  // Set Left Motors backward
  digitalWrite(L1_pin, LOW);
  digitalWrite(L2_pin, HIGH);
  // Set Right Motors  forward
  digitalWrite(R1_pin, HIGH);
  digitalWrite(R2_pin, LOW);
 
} 


void SitStill(){
  // Break
  digitalWrite(L1_pin, LOW);
  digitalWrite(L2_pin, LOW);
   
  digitalWrite(R1_pin, LOW);
  digitalWrite(R2_pin, LOW);
  
}

void MoveBackward(){
  // Set Left Motors backward
  digitalWrite(L1_pin, LOW);
  digitalWrite(L2_pin, HIGH);
  // Set Right Motors backward
  digitalWrite(R1_pin, LOW);
  digitalWrite(R2_pin, HIGH);


  
}

void MoveBackwardLeft(){
  // Set Left Motors backward
  digitalWrite(L1_pin, LOW);
  digitalWrite(L2_pin, HIGH);
  // Set Right Motors backward
  digitalWrite(R1_pin, LOW);
  digitalWrite(R2_pin, HIGH);

}

void MoveBackwardRight(){
  // Set Left Motors backward
  digitalWrite(L1_pin, LOW);
  digitalWrite(L2_pin, HIGH);
  // Set Right Motors backward
  digitalWrite(R1_pin, LOW);
  digitalWrite(R2_pin, HIGH);


}

void MoveForwardSlow(){
  // Set Left Motors forward
  digitalWrite(L2_pin, LOW);
  digitalWrite(L1_pin, HIGH);
  // Set Right Motors forward
  digitalWrite(R2_pin, LOW);
  digitalWrite(R1_pin, HIGH);
  

}

void MoveForwardFast(){
  // Set Left Motors forward
  digitalWrite(L2_pin, LOW);
  digitalWrite(L1_pin, HIGH);
  // Set Right Motors forward
  digitalWrite(R2_pin, LOW);
  digitalWrite(R1_pin, HIGH);
  

}

void MoveRandom(){
  bool direction = random(0,1);

  digitalWrite(L2_pin, direction);
  digitalWrite(L1_pin, !direction);

  digitalWrite(R2_pin, direction);
  digitalWrite(R1_pin, !direction);
 
}

void RotateRandom(){
  bool Rotation = random(0,1);

  digitalWrite(L1_pin, Rotation);
  digitalWrite(L2_pin, !Rotation);

  digitalWrite(R1_pin, !Rotation);
  digitalWrite(R2_pin, Rotation);}
