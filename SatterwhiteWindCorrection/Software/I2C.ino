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

//#ifdef IMU_V5

#include <LSM6.h>
#include <LIS3MDL.h>

LSM6 gyro_acc1;
LIS3MDL mag1;
LSM6 gyro_acc2;
LIS3MDL mag2;

//#else // older IMUs through v4
//
//#include <L3G.h>
//#include <LSM303.h>
//
//L3G gyro;
//LSM303 compass;
//
//#endif


void I2C_Init()
{
  Wire.begin();
//  Wire.setClock(50000);
}

void Gyro_Init()
{
  // Accel_Init() should have already called gyro_acc.init() and enableDefault()
  selectAHRS(1);
  gyro_acc1.writeReg(LSM6::CTRL2_G, 0x4C); // 104 Hz, 2000 dps full scale

  Serial.println("Gyro1 Initialized");

  selectAHRS(2);
  gyro_acc2.writeReg(LSM6::CTRL2_G, 0x4C); // 104 Hz, 2000 dps full scale

  Serial.println("Gyro2 Initialized");

}

void Read_Gyro()
{
  selectAHRS(1);
  Serial.print("A");
  while (!gyro_acc1.readGyro()) 
  {
    Accel_Init(1);
    Serial.println("Accelerometer 1 reinitialized");
  }
  
  AN1[0] = gyro_acc1.g.x;
  AN1[1] = gyro_acc1.g.y;
  AN1[2] = gyro_acc1.g.z;

  gyro1_x = SENSOR1_SIGN[0] * (AN1[0] - AN1_OFFSET[0]);
  gyro1_y = SENSOR1_SIGN[1] * (AN1[1] - AN1_OFFSET[1]);
  gyro1_z = SENSOR1_SIGN[2] * (AN1[2] - AN1_OFFSET[2]);

  selectAHRS(2);
  Serial.print("B");
  while (!gyro_acc2.readGyro()) 
  {
    Accel_Init(2);
    Serial.println("Accelerometer 2 reinitialized");
  }
  AN2[0] = gyro_acc2.g.x;
  AN2[1] = gyro_acc2.g.y;
  AN2[2] = gyro_acc2.g.z;

  gyro2_x = SENSOR2_SIGN[0] * (AN2[0] - AN2_OFFSET[0]);
  gyro2_y = SENSOR2_SIGN[1] * (AN2[1] - AN2_OFFSET[1]);
  gyro2_z = SENSOR2_SIGN[2] * (AN2[2] - AN2_OFFSET[2]);

}

void Accel_Init()
{
  selectAHRS(1);
  delay(10);
  gyro_acc1.init();
  gyro_acc1.enableDefault();
  gyro_acc1.setTimeout(500);
  gyro_acc1.writeReg(LSM6::CTRL1_XL, 0x3C); // 52 Hz, 8 g full scale

  Serial.println("Accel1 Initialized");

  selectAHRS(2);
  delay(10);
  gyro_acc2.init();
  gyro_acc2.enableDefault();
  gyro_acc2.setTimeout(500);
  gyro_acc2.writeReg(LSM6::CTRL1_XL, 0x3C); // 52 Hz, 8 g full scale

  Serial.println("Accel2 Initialized");
}

void Accel_Init(int AHRS)
{
  delay(10);
  if (AHRS == 1)
  {
    selectAHRS(1);
    gyro_acc1.init();
    gyro_acc1.enableDefault();
    gyro_acc1.setTimeout(500);
    gyro_acc1.writeReg(LSM6::CTRL1_XL, 0x3C); // 52 Hz, 8 g full scale
  }
  else if (AHRS == 2)
  {
    selectAHRS(2);
    gyro_acc2.init();
    gyro_acc2.enableDefault();
    gyro_acc2.setTimeout(500);
    gyro_acc2.writeReg(LSM6::CTRL1_XL, 0x3C); // 52 Hz, 8 g full scale
  }
  else
  {
    Serial.println("Invalid AHRS Initialization Requested");
  }
}

// Reads x,y and z accelerometer registers
void Read_Accel()
{
  selectAHRS(1);
  Serial.print("A");
  while (!gyro_acc1.readAcc()) 
  {
    Accel_Init(1);
    Serial.println("Accelerometer 1 reinitialized");
  }

  AN1[3] = gyro_acc1.a.x >> 4; // shift right 4 bits to use 12-bit representation (1 g = 256)
  AN1[4] = gyro_acc1.a.y >> 4;
  AN1[5] = gyro_acc1.a.z >> 4;

  accel1_x = SENSOR1_SIGN[3] * (AN1[3] - AN1_OFFSET[3]);
  accel1_y = SENSOR1_SIGN[4] * (AN1[4] - AN1_OFFSET[4]);
  accel1_z = SENSOR1_SIGN[5] * (AN1[5] - AN1_OFFSET[5]);

  selectAHRS(2);
  Serial.print("B");
  while (!gyro_acc2.readAcc()) 
  {
    Accel_Init(2);
    Serial.println("Accelerometer 2 reinitialized");
  }

  AN2[3] = gyro_acc2.a.x >> 4; // shift right 4 bits to use 12-bit representation (1 g = 256)
  AN2[4] = gyro_acc2.a.y >> 4;
  AN2[5] = gyro_acc2.a.z >> 4;

  accel2_x = SENSOR2_SIGN[3] * (AN2[3] - AN2_OFFSET[3]);
  accel2_y = SENSOR2_SIGN[4] * (AN2[4] - AN2_OFFSET[4]);
  accel2_z = SENSOR2_SIGN[5] * (AN2[5] - AN2_OFFSET[5]);
}

void Compass_Init()
{
  selectAHRS(1);
  mag1.init();
  mag1.enableDefault();
  mag1.setTimeout(500);
  mag1.writeReg(mag1.CTRL_REG1, 0x70);
  Serial.println("Magnetometer 1 Default Overridden. Set to Ultra High Performance Mode for X and Y");
  Serial.println("Magnetometer 1 Initialized");

  selectAHRS(2);
  mag2.init();
  mag2.enableDefault();
  mag2.setTimeout(500);
  Serial.println("Compass2 Initialized");


}

void Compass_Init(int AHRS)
{
  if (AHRS == 1)
  {
    selectAHRS(1);
    mag1.init();
    mag1.enableDefault();
    mag1.setTimeout(500);
  }
  else if (AHRS == 2)
  {
    selectAHRS(2);
    mag2.init();
    mag2.enableDefault();
    mag2.setTimeout(500);
  }
  else
  {
    Serial.println("Invalid AHRS Initialization Request");
  }

}

void Read_Compass()
{
  selectAHRS(1);
  Serial.print("A");
  while (!mag1.read())
  {
    Compass_Init(1);
    Serial.println("Magnetometer 1 reinitialized");
  }

  magnetom1_x = SENSOR1_SIGN[6] * mag1.m.x;
  magnetom1_y = SENSOR1_SIGN[7] * mag1.m.y;
  magnetom1_z = SENSOR1_SIGN[8] * mag1.m.z;

  selectAHRS(2);
  Serial.print("B");
  while (!mag2.read())
  {
    Compass_Init(2);
    Serial.println("Magnetometer 2 reinitialized");
  }

  magnetom2_x = SENSOR2_SIGN[6] * mag2.m.x;
  magnetom2_y = SENSOR2_SIGN[7] * mag2.m.y;
  magnetom2_z = SENSOR2_SIGN[8] * mag2.m.z;

}
