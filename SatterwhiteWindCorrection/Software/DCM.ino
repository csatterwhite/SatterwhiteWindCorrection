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

/**************************************************/
void Normalize(void)
{
  float error1=0;
  float temporary1[3][3];
  float renorm1=0;

  float error2=0;
  float temporary2[3][3];
  float renorm2=0;
  
  error1= -Vector_Dot_Product(&DCM1_Matrix[0][0],&DCM1_Matrix[1][0])*.5; //eq.19
  error2= -Vector_Dot_Product(&DCM2_Matrix[0][0],&DCM2_Matrix[1][0])*.5; //eq.19

  Vector_Scale(&temporary1[0][0], &DCM1_Matrix[1][0], error1); //eq.19
  Vector_Scale(&temporary1[1][0], &DCM1_Matrix[0][0], error1); //eq.19
  
  Vector_Scale(&temporary2[0][0], &DCM2_Matrix[1][0], error2); //eq.19
  Vector_Scale(&temporary2[1][0], &DCM2_Matrix[0][0], error2); //eq.19
  
  Vector_Add(&temporary1[0][0], &temporary1[0][0], &DCM1_Matrix[0][0]);//eq.19
  Vector_Add(&temporary1[1][0], &temporary1[1][0], &DCM1_Matrix[1][0]);//eq.19

  Vector_Add(&temporary2[0][0], &temporary2[0][0], &DCM2_Matrix[0][0]);//eq.19
  Vector_Add(&temporary2[1][0], &temporary2[1][0], &DCM2_Matrix[1][0]);//eq.19
  
  Vector_Cross_Product(&temporary1[2][0],&temporary1[0][0],&temporary1[1][0]); // c= a x b //eq.20
  
  Vector_Cross_Product(&temporary2[2][0],&temporary2[0][0],&temporary2[1][0]); // c= a x b //eq.20
  
  renorm1= .5 *(3 - Vector_Dot_Product(&temporary1[0][0],&temporary1[0][0])); //eq.21
  Vector_Scale(&DCM1_Matrix[0][0], &temporary1[0][0], renorm1);

  renorm2= .5 *(3 - Vector_Dot_Product(&temporary2[0][0],&temporary2[0][0])); //eq.21
  Vector_Scale(&DCM2_Matrix[0][0], &temporary2[0][0], renorm2);
  
  renorm1= .5 *(3 - Vector_Dot_Product(&temporary1[1][0],&temporary1[1][0])); //eq.21
  Vector_Scale(&DCM1_Matrix[1][0], &temporary1[1][0], renorm1);

  renorm2= .5 *(3 - Vector_Dot_Product(&temporary2[1][0],&temporary2[1][0])); //eq.21
  Vector_Scale(&DCM2_Matrix[1][0], &temporary2[1][0], renorm2);
  
  renorm1= .5 *(3 - Vector_Dot_Product(&temporary1[2][0],&temporary1[2][0])); //eq.21
  Vector_Scale(&DCM1_Matrix[2][0], &temporary1[2][0], renorm1);

  renorm2= .5 *(3 - Vector_Dot_Product(&temporary2[2][0],&temporary2[2][0])); //eq.21
  Vector_Scale(&DCM2_Matrix[2][0], &temporary2[2][0], renorm2);
}

/**************************************************/
void Drift_correction(void)
{
  float mag_heading1_x;
  float mag_heading1_y;
  float errorCourse1;
  //Compensation the Roll, Pitch and Yaw drift. 
  static float Scaled_Omega1_P[3];
  static float Scaled_Omega1_I[3];
  float Accel1_magnitude;
  float Accel1_weight;

  float mag_heading2_x;
  float mag_heading2_y;
  float errorCourse2;
  //Compensation the Roll, Pitch and Yaw drift. 
  static float Scaled_Omega2_P[3];
  static float Scaled_Omega2_I[3];
  float Accel2_magnitude;
  float Accel2_weight;
  
  
  //*****Roll and Pitch***************

  // Calculate the magnitude of the accelerometer vector
  Accel1_magnitude = sqrt(Accel1_Vector[0]*Accel1_Vector[0] + Accel1_Vector[1]*Accel1_Vector[1] + Accel1_Vector[2]*Accel1_Vector[2]);
  Accel1_magnitude = Accel1_magnitude / GRAVITY; // Scale to gravity.

  Accel2_magnitude = sqrt(Accel2_Vector[0]*Accel2_Vector[0] + Accel2_Vector[1]*Accel2_Vector[1] + Accel2_Vector[2]*Accel2_Vector[2]);
  Accel2_magnitude = Accel2_magnitude / GRAVITY; // Scale to gravity.
  
  // Dynamic weighting of accelerometer info (reliability filter)
  // Weight for accelerometer info (<0.5G = 0.0, 1G = 1.0 , >1.5G = 0.0)
  Accel1_weight = constrain(1 - 2*abs(1 - Accel1_magnitude),0,1);  // 
  Accel2_weight = constrain(1 - 2*abs(1 - Accel2_magnitude),0,1); 

  Vector_Cross_Product(&errorRollPitch1[0],&Accel1_Vector[0],&DCM1_Matrix[2][0]); //adjust the ground of reference
  Vector_Scale(&Omega1_P[0],&errorRollPitch1[0],Kp_ROLLPITCH*Accel1_weight);
  
  Vector_Cross_Product(&errorRollPitch2[0],&Accel2_Vector[0],&DCM2_Matrix[2][0]); //adjust the ground of reference
  Vector_Scale(&Omega2_P[0],&errorRollPitch2[0],Kp_ROLLPITCH*Accel2_weight);
  
  Vector_Scale(&Scaled_Omega1_I[0],&errorRollPitch1[0],Ki_ROLLPITCH*Accel1_weight);
  Vector_Add(Omega1_I,Omega1_I,Scaled_Omega1_I); 

  Vector_Scale(&Scaled_Omega2_I[0],&errorRollPitch2[0],Ki_ROLLPITCH*Accel2_weight);
  Vector_Add(Omega2_I,Omega2_I,Scaled_Omega2_I);  
  
  //*****YAW***************
  // We make the gyro YAW drift correction based on compass magnetic heading
 
  mag_heading1_x = cos(MAG1_Heading);
  mag_heading1_y = sin(MAG1_Heading);

  mag_heading2_x = cos(MAG2_Heading);
  mag_heading2_y = sin(MAG2_Heading);
  
  errorCourse1=(DCM1_Matrix[0][0]*mag_heading1_y) - (DCM1_Matrix[1][0]*mag_heading1_x);  //Calculating YAW error
  Vector_Scale(errorYaw1,&DCM1_Matrix[2][0],errorCourse1); //Applys the yaw correction to the XYZ rotation of the aircraft, depeding the position.

  errorCourse2=(DCM2_Matrix[0][0]*mag_heading2_y) - (DCM2_Matrix[1][0]*mag_heading2_x);  //Calculating YAW error
  Vector_Scale(errorYaw2,&DCM2_Matrix[2][0],errorCourse2); //Applys the yaw correction to the XYZ rotation of the aircraft, depeding the position.
  
  
  Vector_Scale(&Scaled_Omega1_P[0],&errorYaw1[0],Kp_YAW);//.01proportional of YAW.
  Vector_Add(Omega1_P,Omega1_P,Scaled_Omega1_P);//Adding  Proportional.

  Vector_Scale(&Scaled_Omega2_P[0],&errorYaw2[0],Kp_YAW);//.01proportional of YAW.
  Vector_Add(Omega2_P,Omega2_P,Scaled_Omega2_P);//Adding  Proportional.
  
  Vector_Scale(&Scaled_Omega1_I[0],&errorYaw1[0],Ki_YAW);//.00001Integrator
  Vector_Add(Omega1_I,Omega1_I,Scaled_Omega1_I);//adding integrator to the Omega_I

  Vector_Scale(&Scaled_Omega2_I[0],&errorYaw2[0],Ki_YAW);//.00001Integrator
  Vector_Add(Omega2_I,Omega2_I,Scaled_Omega2_I);//adding integrator to the Omega_I
}
/**************************************************/
/*
void Accel_adjust(void)
{
 Accel_Vector[1] += Accel_Scale(speed_3d*Omega[2]);  // Centrifugal force on Acc_y = GPS_speed*GyroZ
 Accel_Vector[2] -= Accel_Scale(speed_3d*Omega[1]);  // Centrifugal force on Acc_z = GPS_speed*GyroY 
}
*/
/**************************************************/

void Matrix_update(void)
{
  Gyro1_Vector[0]=Gyro_Scaled_X(gyro1_x); //gyro x roll
  Gyro1_Vector[1]=Gyro_Scaled_Y(gyro1_y); //gyro y pitch
  Gyro1_Vector[2]=Gyro_Scaled_Z(gyro1_z); //gyro Z yaw

  Gyro2_Vector[0]=Gyro_Scaled_X(gyro2_x); //gyro x roll
  Gyro2_Vector[1]=Gyro_Scaled_Y(gyro2_y); //gyro y pitch
  Gyro2_Vector[2]=Gyro_Scaled_Z(gyro2_z); //gyro Z yaw
  
  Accel1_Vector[0]=accel1_x;
  Accel1_Vector[1]=accel1_y;
  Accel1_Vector[2]=accel1_z;

  Accel2_Vector[0]=accel2_x;
  Accel2_Vector[1]=accel2_y;
  Accel2_Vector[2]=accel2_z;
    
  Vector_Add(&Omega1[0], &Gyro1_Vector[0], &Omega1_I[0]);  //adding proportional term
  Vector_Add(&Omega1_Vector[0], &Omega1[0], &Omega1_P[0]); //adding Integrator term

  Vector_Add(&Omega2[0], &Gyro2_Vector[0], &Omega2_I[0]);  //adding proportional term
  Vector_Add(&Omega2_Vector[0], &Omega2[0], &Omega2_P[0]); //adding Integrator term

  //Accel_adjust();    //Remove centrifugal acceleration.   We are not using this function in this version - we have no speed measurement
  
 #if OUTPUTMODE==1         
  Update1_Matrix[0][0]=0;
  Update1_Matrix[0][1]=-G_Dt*Omega1_Vector[2];//-z
  Update1_Matrix[0][2]=G_Dt*Omega1_Vector[1];//y
  Update1_Matrix[1][0]=G_Dt*Omega1_Vector[2];//z
  Update1_Matrix[1][1]=0;
  Update1_Matrix[1][2]=-G_Dt*Omega1_Vector[0];//-x
  Update1_Matrix[2][0]=-G_Dt*Omega1_Vector[1];//-y
  Update1_Matrix[2][1]=G_Dt*Omega1_Vector[0];//x
  Update1_Matrix[2][2]=0;

  Update2_Matrix[0][0]=0;
  Update2_Matrix[0][1]=-G_Dt*Omega2_Vector[2];//-z
  Update2_Matrix[0][2]=G_Dt*Omega2_Vector[1];//y
  Update2_Matrix[1][0]=G_Dt*Omega2_Vector[2];//z
  Update2_Matrix[1][1]=0;
  Update2_Matrix[1][2]=-G_Dt*Omega2_Vector[0];//-x
  Update2_Matrix[2][0]=-G_Dt*Omega2_Vector[1];//-y
  Update2_Matrix[2][1]=G_Dt*Omega2_Vector[0];//x
  Update2_Matrix[2][2]=0;
 #else                    // Uncorrected data (no drift correction)
  Update1_Matrix[0][0]=0;
  Update1_Matrix[0][1]=-G_Dt*Gyro1_Vector[2];//-z
  Update1_Matrix[0][2]=G_Dt*Gyro1_Vector[1];//y
  Update1_Matrix[1][0]=G_Dt*Gyro1_Vector[2];//z
  Update1_Matrix[1][1]=0;
  Update1_Matrix[1][2]=-G_Dt*Gyro1_Vector[0];
  Update1_Matrix[2][0]=-G_Dt*Gyro1_Vector[1];
  Update1_Matrix[2][1]=G_Dt*Gyro1_Vector[0];
  Update1_Matrix[2][2]=0;

  Update2_Matrix[0][0]=0;
  Update2_Matrix[0][1]=-G_Dt*Gyro2_Vector[2];//-z
  Update2_Matrix[0][2]=G_Dt*Gyro2_Vector[1];//y
  Update2_Matrix[1][0]=G_Dt*Gyro2_Vector[2];//z
  Update2_Matrix[1][1]=0;
  Update2_Matrix[1][2]=-G_Dt*Gyro2_Vector[0];
  Update2_Matrix[2][0]=-G_Dt*Gyro2_Vector[1];
  Update2_Matrix[2][1]=G_Dt*Gyro2_Vector[0];
  Update2_Matrix[2][2]=0;
 #endif

  Matrix_Multiply(DCM1_Matrix,Update1_Matrix,Temporary1_Matrix); //a*b=c
  Matrix_Multiply(DCM2_Matrix,Update2_Matrix,Temporary2_Matrix); //a*b=c

  for(int x=0; x<3; x++) //Matrix Addition (update)
  {
    for(int y=0; y<3; y++)
    {
      DCM1_Matrix[x][y]+=Temporary1_Matrix[x][y];
      DCM2_Matrix[x][y]+=Temporary2_Matrix[x][y];
    } 
  }
}

void Euler_angles(void)
{
  pitch1 = -asin(DCM1_Matrix[2][0]);
  roll1 = atan2(DCM1_Matrix[2][1],DCM1_Matrix[2][2]);
  yaw1 = atan2(DCM1_Matrix[1][0],DCM1_Matrix[0][0]);

  pitch2 = -asin(DCM2_Matrix[2][0]);
  roll2 = atan2(DCM2_Matrix[2][1],DCM2_Matrix[2][2]);
  yaw2 = atan2(DCM2_Matrix[1][0],DCM2_Matrix[0][0]);
}
