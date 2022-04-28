/*

  MinIMU-9-Arduino-AHRS
  Pololu MinIMU-9 + Arduino AHRS (Attitude and Heading Reference System)

  Copyright (c) 2011-2016 Pololu Corporation.
  http://www.pololu.com/

  MinIMU-9-Arduino-AHRS is based on sf9domahrs by Doug Weibel and Jose Julio:
  http://code.google.com/p/sf9domahrs/

  sf9domahrs is based on ArduIMU v1.5 by Jordi Munoz and William Premerlani, Jose
  Julio and Doug Weibel:
  http://code.google.com/p/ardu-imu/

  MinIMU-9-Arduino-AHRS is free software: you can redistribute it and/or modify it
  under the terms of the GNU Lesser General Public License as published by the
  Free Software Foundation, either version 3 of the License, or (at your option)
  any later version.

  MinIMU-9-Arduino-AHRS is distributed in the hope that it will be useful, but
  WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
  FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for
  more details.

  You should have received a copy of the GNU Lesser General Public License along
  with MinIMU-9-Arduino-AHRS. If not, see <http://www.gnu.org/licenses/>.

*/


//-----------------------------INCLUDES--------------------------------------
#include <Wire.h>
#include <Arduino.h>
#include <NMEA2000_CAN.h>  // This will automatically choose right CAN library and create suitable NMEA2000 object
#include <N2kMessages.h>
#include <N2kMsg.h>
#include <NMEA2000.h>

int ProgramState;

//------------------N2K DEFINITIONS AND DECLARATIONS------------------------
#define WindUpdatePeriod 1000

typedef struct {
  unsigned long PGN;
  void (*Handler)(const tN2kMsg &N2kMsg);
} tNMEA2000Handler;

unsigned char SID;
double WindAngle;
double WindSpeed;
double CorrectedWindAngle;

tN2kWindReference windRef;
tN2kMsg N2kMsg(0);

void ApparentWind(const tN2kMsg &N2kMsg);

tNMEA2000Handler NMEA2000Handlers[] = {
  {130306L, &ApparentWind},
  {0, 0}
};

// Forward declarations for led blinking
void LedOn(unsigned long OnTime);
void UpdateLedState();
void HandleNMEA2000Msg(const tN2kMsg &N2kMsg);

// List here messages your device will transmit.
const unsigned long TransmitMessages[] PROGMEM={130306L,0};



//---------------I2C AHRS DEFINITIONS AND DECLARATIONS-----------------------
// Uncomment the following line to use a MinIMU-9 v5 or AltIMU-10 v5. Leave commented for older IMUs (up through v4).
#define IMU_V5

// Uncomment the below line to use this axis definition:
// X axis pointing forward
// Y axis pointing to the right
// and Z axis pointing down.
// Positive pitch : nose up
// Positive roll : right wing down
// Positive yaw : clockwise
// int SENSOR_SIGN[9] = {1, 1, 1, -1, -1, -1, 1, 1, 1}; //Correct directions x,y,z - gyro, accelerometer, magnetometer
// Uncomment the below line to use this axis definition:
// X axis pointing forward
// Y axis pointing to the left
// and Z axis pointing up.
// Positive pitch : nose down
// Positive roll : right wing down
// Positive yaw : counterclockwise
// int SENSOR_SIGN[9] = {-1,-1,1,1,1,-1,-1,-1,1};

int SENSOR1_SIGN[9] = {1,-1,-1,-1,1,1,1,-1,-1};
int SENSOR2_SIGN[9] = {-1,1,-1,1,-1,1,-1,1,-1};

float mag1Cal[3][3] = {{0.972,0.002,0.006},{0.002,1.003,-0.004},{0.006,-0.004,1.026}};
float mag1Offset[3] = {97.8,507.1,-499.2}; //10x Value from SensorCal

//float mag2Cal[3][3] = {{0.975,-0.023,0.013},{-0.023,1.010,0.009},{0.013,0.009,1.017}};
//float mag2Offset[3] = {-143.1,-347.4,-488.9}; //10x Value from SensorCal

float mag2Cal[3][3] = {{1.009,-0.023,0.015},{-0.023,1.024,-0.019},{0.015,-0.019,0.969}};
float mag2Offset[3] = {-95.3,-354.4,-485.6}; //10x Value from SensorCal

#define IMU1_PIN 22
#define IMU2_PIN 30

// accelerometer: 8 g sensitivity
// 3.9 mg/digit; 1 g = 256
#define GRAVITY 256  //this equivalent to 1G in the raw data coming from the accelerometer

#define ToRad(x) ((x)*0.01745329252)  // *pi/180
#define ToDeg(x) ((x)*57.2957795131)  // *180/pi

// gyro: 2000 dps full scale
// 70 mdps/digit; 1 dps = 0.07
#define Gyro_Gain_X 0.07 //X axis Gyro gain
#define Gyro_Gain_Y 0.07 //Y axis Gyro gain
#define Gyro_Gain_Z 0.07 //Z axis Gyro gain
#define Gyro_Scaled_X(x) ((x)*ToRad(Gyro_Gain_X)) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Y(x) ((x)*ToRad(Gyro_Gain_Y)) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Z(x) ((x)*ToRad(Gyro_Gain_Z)) //Return the scaled ADC raw data of the gyro in radians for second

// LSM303/LIS3MDL magnetometer calibration constants; use the Calibrate example from
// the Pololu LSM303 or LIS3MDL library to find the right values for your board


//#define M1_X_MIN -2622
//#define M1_Y_MIN -8224
//#define M1_Z_MIN +1537
//#define M1_X_MAX +4600
//#define M1_Y_MAX -1744
//#define M1_Z_MAX +7803

#define M1_X_MIN -414
#define M1_Y_MIN -6026
#define M1_Z_MIN +1591
#define M1_X_MAX +2298
#define M1_Y_MAX -3406
#define M1_Z_MAX +1865

#define M2_X_MIN -2200
#define M2_Y_MIN -7110
#define M2_Z_MIN +1222
#define M2_X_MAX +5100
#define M2_Y_MAX -90
#define M2_Z_MAX +8509

#define Kp_ROLLPITCH 0.02
#define Ki_ROLLPITCH 0.00002
#define Kp_YAW 1.2
#define Ki_YAW 0.00002

/*For debugging purposes*/
//OUTPUTMODE=1 will print the corrected data,
//OUTPUTMODE=0 will print uncorrected data of the gyros (with drift)
#define OUTPUTMODE 1

#define PRINT_DCM 0     //Will print the whole direction cosine matrix
#define PRINT_ANALOGS 0 //Will print the analog raw data
#define PRINT_EULER 0   //Will print the Euler angles Roll, Pitch and Yaw

#define STATUS_LED 13

float G_Dt = 0.02;  // Integration time (DCM algorithm)  We will run the integration loop at 50Hz if possible

long timer = 0; //general purpuse timer
long timer_old;
long timer24 = 0; //Second timer used to print values
int AN1[6]; //array that stores the gyro and accelerometer data
int AN2[6]; 
int AN1_OFFSET[6] = {0, 0, 0, 0, 0, 0}; //Array that stores the Offset of the sensors
int AN2_OFFSET[6] = {0, 0, 0, 0, 0, 0};

int gyro1_x;
int gyro1_y;
int gyro1_z;
int accel1_x;
int accel1_y;
int accel1_z;
int magnetom1_x;
int magnetom1_y;
int magnetom1_z;
float c_magnetom1_x;
float c_magnetom1_y;
float c_magnetom1_z;
float MAG1_Heading;
int gyro2_x;
int gyro2_y;
int gyro2_z;
int accel2_x;
int accel2_y;
int accel2_z;
int magnetom2_x;
int magnetom2_y;
int magnetom2_z;
float c_magnetom2_x;
float c_magnetom2_y;
float c_magnetom2_z;
float MAG2_Heading;

float magConversion = 0.1;

float Accel1_Vector[3] = {0, 0, 0}; //Store the acceleration in a vector
float Gyro1_Vector[3] = {0, 0, 0}; //Store the gyros turn rate in a vector
float Omega1_Vector[3] = {0, 0, 0}; //Corrected Gyro_Vector data
float Omega1_P[3] = {0, 0, 0}; //Omega Proportional correction
float Omega1_I[3] = {0, 0, 0}; //Omega Integrator
float Omega1[3] = {0, 0, 0};
float Accel2_Vector[3] = {0, 0, 0}; //Store the acceleration in a vector
float Gyro2_Vector[3] = {0, 0, 0}; //Store the gyros turn rate in a vector
float Omega2_Vector[3] = {0, 0, 0}; //Corrected Gyro_Vector data
float Omega2_P[3] = {0, 0, 0}; //Omega Proportional correction
float Omega2_I[3] = {0, 0, 0}; //Omega Integrator
float Omega2[3] = {0, 0, 0};

// Euler angles
float roll1;
float pitch1;
float yaw1;
float roll2;
float pitch2;
float yaw2;

float errorRollPitch1[3] = {0, 0, 0};
float errorYaw1[3] = {0, 0, 0};
float errorRollPitch2[3] = {0, 0, 0};
float errorYaw2[3] = {0, 0, 0};

float diffMag;
float diffYaw;

unsigned int counter = 0;
byte gyro_sat = 0;

float DCM1_Matrix[3][3] = {
  {
    1, 0, 0
  }
  , {
    0, 1, 0
  }
  , {
    0, 0, 1
  }
};
float DCM2_Matrix[3][3] = {
  {
    1, 0, 0
  }
  , {
    0, 1, 0
  }
  , {
    0, 0, 1
  }
};

float Update1_Matrix[3][3] = {{0, 1, 2}, {3, 4, 5}, {6, 7, 8}}; //Gyros here
float Update2_Matrix[3][3] = {{0, 1, 2}, {3, 4, 5}, {6, 7, 8}}; //Gyros here

float Temporary1_Matrix[3][3] = {
  {
    0, 0, 0
  }
  , {
    0, 0, 0
  }
  , {
    0, 0, 0
  }
};
float Temporary2_Matrix[3][3] = {
  {
    0, 0, 0
  }
  , {
    0, 0, 0
  }
  , {
    0, 0, 0
  }
};


void setup()
{
  Serial.begin(115200);
  pinMode (STATUS_LED, OUTPUT); // Status LED
  
  //-----------------------N2K Device Setup-----------------------
  
  // Set Product information
  NMEA2000.SetProductInformation("00000002", // Manufacturer's Model serial code
                                 100, // Manufacturer's product code
                                 "Satterwhite Rotation Correction",  // Manufacturer's Model ID
                                 "1.0.0 (2019-07-06)",  // Manufacturer's Software version code
                                 "1.0.0 (2019-07-06)" // Manufacturer's Model version
                                 );
  // Set device information
  NMEA2000.SetDeviceInformation(1, // Unique number. Use e.g. Serial number.
                                130, // Device function=Atmospheric. See codes on http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                85, // Device class=External Environment. See codes on  http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                2046 // Just choosen free from code list on http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf                               
                               );

  NMEA2000.SetN2kCANMsgBufSize(8);
  NMEA2000.SetN2kCANReceiveFrameBufSize(100);
  NMEA2000.SetForwardStream(&Serial);  // PC output on due native port
  NMEA2000.SetForwardType(tNMEA2000::fwdt_Text); // Show in clear text
  NMEA2000.SetMsgHandler(HandleNMEA2000Msg);

  NMEA2000.SetMode(tNMEA2000::N2km_ListenAndNode,23);
  NMEA2000.ExtendTransmitMessages(TransmitMessages); 

  NMEA2000.EnableForward(false);
  
  NMEA2000.Open();


  //----------------------I2C AHRS UNITS SETUP-----------------------
  pinMode(IMU1_PIN, OUTPUT);
  pinMode(IMU2_PIN, OUTPUT);
  
  I2C_Init();

  digitalWrite(STATUS_LED, LOW);
  delay(1500);

  Accel_Init();
  Compass_Init();
  Gyro_Init();

  delay(20);

  for (int i = 0; i < 32; i++) // We take some readings...
  {
    Read_Gyro();
    Read_Accel();
    for (int y = 0; y < 6; y++) // Cumulate values
    {
      AN1_OFFSET[y] += AN1[y];
      AN2_OFFSET[y] += AN2[y];
    }
    delay(20);
  }

  for (int y = 0; y < 6; y++)
  {
    AN1_OFFSET[y] = AN1_OFFSET[y] / 32;
    AN2_OFFSET[y] = AN2_OFFSET[y] / 32;
  }

  AN1_OFFSET[5] -= GRAVITY * SENSOR1_SIGN[5];
  AN2_OFFSET[5] -= GRAVITY * SENSOR2_SIGN[5];

  //  Serial.println("Offset:");
  //  for (int y = 0; y < 6; y++)
  //    Serial.println(AN_OFFSET[y]);

  delay(2000);
  digitalWrite(STATUS_LED, HIGH);

  timer = millis();
  delay(20);
  counter = 0;

}

void loop() //Main Loop
{

  if ((millis() - timer) >= 20) // Main loop runs at 50Hz
  {
    ProgramState = 1;
    Serial.print(ProgramState);
    counter++;
    timer_old = timer;
    timer = millis();
    if (timer > timer_old)
    {
      G_Dt = (timer - timer_old) / 1000.0; // Real time of loop run. We use this on the DCM algorithm (gyro integration time)
      if (G_Dt > 0.2)
        G_Dt = 0; // ignore integration times over 200 ms
    }
    else
      G_Dt = 0;

    ProgramState = 2;
    Serial.print(ProgramState);

    // *** DCM algorithm
    // Data adquisition
    Read_Gyro();   // This read gyro data
    ProgramState = 3;
    Serial.print(ProgramState);
    
    Read_Accel();     // Read I2C accelerometer
    

//    if (counter > 5)  // Read compass data at 10Hz... (5 loop runs)
    if (counter > 2)  // Read compass data at 25Hz... (2 loop runs)
    {
      ProgramState = 4;
      Serial.print(ProgramState);
      counter = 0;
      Read_Compass();    // Read I2C magnetometer
      ProgramState = 5;
      Serial.print(ProgramState);
      Compass_Heading(); // Calculate magnetic heading
    }

    // Calculations...
    ProgramState = 6;
    Serial.print(ProgramState);
    Matrix_update();
    Normalize();
    Drift_correction();
    Euler_angles();
    // ***

    if (MAG1_Heading < 0) 
      {
        MAG1_Heading = 2*PI+MAG1_Heading; //Convert to positive if between -180 and 0
      }
    if (MAG2_Heading < 0)
      {
        MAG2_Heading = 2*PI+MAG2_Heading;
      }

//    printdata();
    diffMag = MAG2_Heading-MAG1_Heading;
    diffYaw = yaw2-yaw1;
    
//    Serial.print(",   Yaw Diff = ");
//    Serial.println(ToDeg(diffYaw)); 

    // Correct Wind Speed and Send to Display
    ProgramState = 7;
    Serial.print(ProgramState);
    NMEA2000.ParseMessages();
    ProgramState = 8;
    Serial.print(ProgramState);
    if ( Serial.available() ) { Serial.read(); } 
//    UpdateLedState();
  
    CorrectedWindAngle = WindAngle+diffMag;

    while (CorrectedWindAngle < 0)
    {
      CorrectedWindAngle = CorrectedWindAngle + 2*PI;
    }
    while (CorrectedWindAngle > 2*PI)
    {
      CorrectedWindAngle = CorrectedWindAngle - 2*PI;
    }
  
    SendN2kWind(WindSpeed,CorrectedWindAngle);

    Serial.print("  Mag 1 = ");
    Serial.print(ToDeg(MAG1_Heading));
    Serial.print(", Mag 2 = ");
    Serial.print(ToDeg(MAG2_Heading));
    Serial.print(", Diff = ");
    Serial.print(ToDeg(diffMag));
    Serial.print(", Raw Wing Angle = ");
    Serial.print(ToDeg(WindAngle));
    Serial.print(", Corrected Wind Angle = ");
    Serial.println(ToDeg(CorrectedWindAngle));
  }

}


void selectAHRS(int AHRS)
{
  bool wait = false;
  if (AHRS == 1)
    {
      wait = digitalRead(IMU1_PIN);
      
      digitalWrite(IMU1_PIN, LOW);
      digitalWrite(IMU2_PIN, HIGH);
    }
   else if (AHRS == 2)
   {
      wait = digitalRead(IMU2_PIN);
      digitalWrite(IMU1_PIN, HIGH);
      digitalWrite(IMU2_PIN, LOW);
   }
   if (wait)
   {
     delay(20);
   }
}
