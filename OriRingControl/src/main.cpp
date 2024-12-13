/**********************************************************************************************
 * Haptic OriRing
 * by Mustafa Mete <mustafa.mete@epfl.ch>
 **********************************************************************************************/
#include "main.h"

String inputString = "";         // a String to hold incoming data
bool stringComplete = false;  // whether the string is complete


// Timing control
Timer Timer_01, Timer_02;

// pres_sens_pin, valve1_pin, valve2_pin, id
Pouch pouch1(A0, 3, 2, 1);
int pins[4] = {A4,A5,A6,A7};
PFSR pfsr1(pins);

// stiffness control
double fsrForceZ = 0; // [N] read from the sensor
double fsrForceZSetpoint = 20 ; // [N] target force value
double forceControlOutput = 0;

double stiffnessSetpoint = 0.2; // [N/mm] // changes between [0.072-0.275] for [10kPa-100kPA]
double targetRingHeight = 8; // [mm]

// Force PID variables and objectsx
double kP_FC=0, kI_FC=0, kD_FC=0;

PID forceControlPID(&fsrForceZ, &forceControlOutput, &fsrForceZSetpoint,
kP_FC, kI_FC, kD_FC, FORCE_CONTROL_PERIOD, DIRECT);

// https://stackoverflow.com/questions/9072320/split-string-into-string-array
String getValue(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length()-1;

  for(int i=0; i<=maxIndex && found<=index; i++){
    if(data.charAt(i)==separator || i==maxIndex){
        found++;
        strIndex[0] = strIndex[1]+1;
        strIndex[1] = (i == maxIndex) ? i+1 : i;
    }
  }
  return found>index ? data.substring(strIndex[0], strIndex[1]) : "";
}


float lowpassFilter(double previousFilteredValue, double input, double  beta){
  return beta * previousFilteredValue + (1 - beta) * input;
}

// Need calibration time to time!!
float calculateRingLegAngle(int sensorDiffRaw){ 
  float x = (float)sensorDiffRaw;
  float x_1 = (float)(bendingAngle1_0 - bendingAngle2_0);
  float x_2 = (float)(bendingAngle1_90 - bendingAngle2_90);
  float y_1 = 0.1, y_2 = PI/2;
  bendingAngleSlope = (y_2 - y_1)/(x_2-x_1);
  float y = (x - x_1)*(bendingAngleSlope) + y_1;
  return y;
}


// based on the current PFSR wiring and directions
int calculateSwipeDirection_PFSR(int changeArray[4]){
  // 0 no swipe, 1 +x direction, 2 -x direction, 3 +y direction, 4 -y direction
  int direction = 0;

  /* more strict swiping condition    

  if(changeArray[0]>0 && changeArray[1]>0 && changeArray[2]<0 && changeArray[3]<0)
    direction = 1;
  else if(changeArray[0]<0 && changeArray[1]<0 && changeArray[2]>0 && changeArray[3]>0)
    direction = 2;
  else if(changeArray[0]>0 && changeArray[1]<0 && changeArray[2]<0 && changeArray[3]>0)
    direction = 3;
  else if(changeArray[0]<0 && changeArray[1]>0 && changeArray[2]>0 && changeArray[3]<0)
    direction = 4;  
  else
    direction = 0;
 */

  /* least strict swiping condition  */
  if( !(changeArray[0]<=0 || changeArray[1]<=0) && !(changeArray[2]>=0 || changeArray[3]>=0))
    direction = 1;
  else if( !(changeArray[0]>=0 || changeArray[1]>=0) && !(changeArray[2]<=0 || changeArray[3]<=0))
    direction = 2;
  else if( !(changeArray[0]<=0 || changeArray[3]<=0) && !(changeArray[1]>=0 || changeArray[2]>=0))
    direction = 3;
  else if( !(changeArray[0]>=0 || changeArray[3]>=0) && !(changeArray[1]<=0 || changeArray[2]<=0))
  direction = 4;  
  else
    direction = 0;

  return direction;
}

float modelControlStiffnessAndHeight(float desiredHeightRaw, float desiredStiffness, float currentHeightRaw){
  float desiredHeight = desiredHeightRaw - 12.5;
  float currentHeight = currentHeightRaw - 12.5;
  float maxHeight = 13.5, minHeight = 0; // mm //minHeight = 7.5; // mm
  float maxPressure = 100, minPressure = 0; //minPressure = 20; // kPa
  
  if (desiredHeight>maxHeight)
    desiredHeight = maxHeight;
  else if(desiredHeight<minHeight)
    desiredHeight = minHeight;

  float desiredInitalPressure = 1.724*exp(0.2983*desiredHeight);
 
  if (desiredInitalPressure>maxPressure)
    desiredInitalPressure = maxPressure;
  else if(desiredInitalPressure<minPressure)
    desiredInitalPressure = minPressure;

  float desiredForceAtZeroExtension = desiredHeight * desiredStiffness;

  float desiredFinalPressure = 23.424 * desiredForceAtZeroExtension - 2.5797;

  if (desiredFinalPressure>maxPressure)
    desiredFinalPressure = maxPressure;
  else if(desiredFinalPressure<minPressure)
    desiredFinalPressure = minPressure;
  float a = (desiredFinalPressure - desiredInitalPressure)/desiredHeight;

  float pressureSetpoint =  a * (desiredHeight-currentHeight) + desiredInitalPressure;
  if (pressureSetpoint>maxPressure)
    pressureSetpoint = maxPressure;
  else if(pressureSetpoint<minPressure)
    pressureSetpoint = minPressure;
  return pressureSetpoint;
}

void setup() {
  inputString.reserve(200);
  //SET analog write resolution to 12 bits [0-4095]
  analogWriteResolution(12);

  // sets the input,output, and frequencies of the pins
  pouch1.setupPINs();

  // set the bs pins
  pinMode(bendingSensorPin1,INPUT);
  pinMode(bendingSensorPin2,INPUT);

  // section values array for moving average
  for (int i = 0; i < windowSize; i++) {
    for (int j = 0; j < 4; j++) {
      sectionValues[j][i] = 0.0;
    }
  }

  // POUCH PIDS
  // initialize the setpoints
  pouch1.setpointPouch1Pressure = 0; // [kPa]
  pouch1.setupPIDs();

  // setup pouch pressure pids
  double kP= 40, kI = 300, kD = 0.1; // 1;
  pouch1.pouch1PID->SetTunings(kP, kI, kD); // p,i,d

  // FORCE PIDs
  double max_pressure = MAX_PRESSURE, min_pressure = MIN_PRESSURE; // [kPa]
  forceControlPID.SetOutputLimits(min_pressure,max_pressure);
  forceControlPID.IntegratorAntiWindUpLimits(-max_pressure, max_pressure); // tune this

  kP_FC= 3, kI_FC=10, kD_FC=0;
  forceControlPID.SetTunings(kP_FC, kI_FC, kD_FC);

  // Pyramid FSR pin setup
  pfsr1.setupPINs();

  startTime = millis();
  initialTime = startTime;
  
  // initialize the times
  Timer_01.initialize(PRESSURE_CONTROL_PERIOD);
  Timer_02.initialize(SERIAL_PRINT_PERIOD);

	Serial.begin(BAUD_RATE); // update in .ini file as well
  Keyboard.begin(); // initialize the keyboard
}

void loop() {
  // ---- Pressure Control --- @pressureControlLoop[1ms]
  if(Timer_01.Tick()){
    while (Serial.available()>0) {
        char inChar = (char)Serial.read();
        // add it to the inputString:
        inputString += inChar;
        // if the incoming character is a newline, set a flag so the main loop can
        // do something about it:
        if (inChar == '\n') {
          stringComplete = true;
        }
    }
    if(stringComplete){ 
      stiffnessSetpoint = getValue(inputString, ',', 0).toFloat(); // stiffness
      targetRingHeight = getValue(inputString, ',', 1).toFloat() - 20.5; // cube size - VR sends [40-46]
      //Reset string
      inputString = "";
      stringComplete = false;
      Serial.flush();
    }  

    currentTime = (millis() - startTime)/1000.0; //[s]
    // --- Bending sensor ----  
    ringLegAngleBS = calculateRingLegAngle(analogRead(bendingSensorPin1)-analogRead(bendingSensorPin2)); // update this with diff!
    // can be filtered in the future
    ringHeightBS = 30*sin(ringLegAngleBS);

    // --- PFSR ----
    pfsr1.readNormalForce();

    // Update the circular buffer
    for(int i=0; i<4; i++){
      sectionValues[i][valueIndex] = (float)pfsr1.sectionValues[i];
    }
    valueIndex = (valueIndex + 1) % windowSize;

    // Calculate the average of the values in the window
    for(int i=0; i<4; i++){ // init
      sectionSum[i]= 0.0;
      sectionAverage[i] = 0.0;
    }

    for (int i = 0; i < windowSize; i++) { // sum
      for(int j=0; j<4; j++){
        sectionSum[j] += sectionValues[j][i];
      }
    }
    
    for(int i=0; i<4; i++){ // avg
      sectionAverage[i] = sectionSum[i] / windowSize;
      // sectionThreshold[i] = sectionAverage[i]*0.1; // threshold as 10% of the average
      sectionThreshold[i] = changeThreshold; // constant threshold
    }

    for(int i=0; i<4; i++){
      // Check for rising or falling conditions
      if (pfsr1.sectionValues[i] > sectionAverage[i] + sectionThreshold[i] ) {
        sectionChange[i] = 1; // Rising
      } else if (pfsr1.sectionValues[i] < sectionAverage[i] - sectionThreshold[i] ) {
        sectionChange[i] = -1; // Falling
      } else {
        sectionChange[i] = 0; // Staying still
      }
    }
    
    // without edge detection
    // swipeDirection = calculateSwipeDirection_PFSR(sectionChange); 
    
    /* edge detection*/
    if(swipeWasHigh == 0){
      swipeDirection = calculateSwipeDirection_PFSR(sectionChange);
      
      // this controls keyboard inputs
      if(swipeDirection == 1){
        Keyboard.press(KEY_UP_ARROW);
      }
      else if(swipeDirection == 2){
        Keyboard.press(KEY_DOWN_ARROW);
      }
      else if(swipeDirection == 3){
        Keyboard.press(KEY_LEFT_ARROW);
        if(stiffnessSetpoint < 1)
         stiffnessSetpoint = stiffnessSetpoint - 0.2;
      }
      else if(swipeDirection == 4){
        Keyboard.press(KEY_RIGHT_ARROW);
        if(stiffnessSetpoint > 0)
          stiffnessSetpoint = stiffnessSetpoint + 0.2;
      }
    }
    else // swipeWasHigh == 1
    {
      swipeDirection = 0;
      swipeRisingEdgeCounter ++;
    }

    if(swipeRisingEdgeCounter > swipeRisingEdgeMaxLimit){
      swipeRisingEdgeCounter = 0;
      swipeWasHigh = 0;
      }

    if ( (swipeDirection != 0) && (swipeRisingEdgeCounter == 0) ) {
      swipeWasHigh = 1;
      Serial.print(ringHeightBS,2);Serial.print(","); 
      Serial.print(swipeDirection); Serial.println();
      swipeDirection = 0;
      Keyboard.releaseAll();

      // Reset the sensor buffer. othwerwise, it remembers previous high values
      for(int i=0; i<4; i++){
        for(int j=0; j<windowSize; j++){
          sectionValues[i][j] = 135.0;
        }
      }

    }
    
    // ---- Stiffness Control! -------- //
    // stiffnessSetpoint = 1; // [N/mm], targetRingHeight = 20; // [mm], fsrForceZSetpoint = 10;
    fsrForceZSetpoint = (targetRingHeight - ringHeightBS) * stiffnessSetpoint;

    // RUN FORCE CONTROLLER
    forceControlPID.Compute();

    // ---- Pressure Control! -------- //

    // Bandwidth test
    /*
    if(frequency < maxFrequency){
      // Update the cycle count
      if (t >= cyclesToDouble * 1.0 / frequency) {
        frequency *= 2.0;
        t = 0;
        startTime = millis();
      }

      // Calculate the current time in seconds
      t = 0.001 * (millis() - startTime);
      t_all = 0.001 * (millis() - initialTime);

      // Calculate the sinusoidal signal value
      sinPressureTrajectory = amplitude * (sin(2 * PI * frequency * t) + 1 ) + offsetPressure;
    }
    else{
      sinPressureTrajectory = 0;
    }
    pouch1.setpointPouch1Pressure = sinPressureTrajectory;
    */

    // for VR
    //pouch1.setpointPouch1Pressure = modelControlStiffnessAndHeight(targetRingHeight, stiffnessSetpoint, ringHeightBS); // [kPa]
    
    // open-loop stiffness control
    // pouch1.setpointPouch1Pressure = stiffnessSetpoint * 150; 
    
    // closed-loop stiffness control
    modelControlStiffnessAndHeight(targetRingHeight,stiffnessSetpoint, ringHeightBS); // [kPa]
    pouch1.setpointPouch1Pressure = forceControlOutput; // assign the pid output of force control

    // test
    //pouch1.setpointPouch1Pressure = 100; // // [kPa]

    pouch1.readPressure();
    // RUN THE PRESSURE CONTROLLER (PID)
    pouch1.computePressureController();  // updates the desired PWM outputs
    // GET THE PID OUTPUT - PWM OUTPUTS
    pouch1.computePWMSetpoints(); // calculates the desired PWM setpoints
    
    // SET PWM VALUES
    pouch1.setPWMs();

    if(Timer_02.Tick()){ // Print to serial

      /* // For bending's sensor calibration       
      int a1 = analogRead(bendingSensorPin1);
      int a2 = analogRead(bendingSensorPin2);
      Serial.print(a1);Serial.print("\t");
      Serial.print(a2);Serial.print("\t");
      */ 
      
      // VR communication     
      // Serial.print(ringHeightBS,2);Serial.print(","); 
      // Serial.print(swipeDirection); // Serial.print(","); 

      // PC control
      Serial.print(swipeDirection); Serial.print(", ");
      Serial.print(pouch1.setpointPouch1Pressure);
      Serial.println();
    }
  }
}

