/*

  MinIMU-9-Arduino-AHRS
  Pololu MinIMU-9 + Arduino AHRS (Attitude and Heading Reference System)

  Copyright (c) 2011 Pololu Corporation.
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

void Compass_Heading()
{
  float MAG1_X;
  float MAG1_Y;
  float cos_roll1;
  float sin_roll1;
  float cos_pitch1;
  float sin_pitch1;

  float MAG2_X;
  float MAG2_Y;
  float cos_roll2;
  float sin_roll2;
  float cos_pitch2;
  float sin_pitch2;

  float c_hardIron1_x;
  float c_hardIron1_y;
  float c_hardIron1_z;
  float c_hardIron2_x;
  float c_hardIron2_y;
  float c_hardIron2_z;

  cos_roll1 = cos(roll1);
  sin_roll1 = sin(roll1);
  cos_pitch1 = cos(pitch1);
  sin_pitch1 = sin(pitch1);

  cos_roll2 = cos(roll2);
  sin_roll2 = sin(roll2);
  cos_pitch2 = cos(pitch2);
  sin_pitch2 = sin(pitch2);



  //  Adjust for Hard Iron Offsets first then Soft Iron
  c_hardIron1_x = magnetom1_x * magConversion - mag1Offset[0];
  c_hardIron1_y = magnetom1_y * magConversion - mag1Offset[1];
  c_hardIron1_z = magnetom1_z * magConversion - mag1Offset[2];
  c_magnetom1_x = (c_hardIron1_x * mag1Cal[0][0] + c_hardIron1_y * mag1Cal[0][1] + c_hardIron1_z * mag1Cal[0][2]) / sqrt(sq(c_hardIron1_x) + sq(c_hardIron1_y) + sq(c_hardIron1_z));
  c_magnetom1_y = (c_hardIron1_x * mag1Cal[1][0] + c_hardIron1_y * mag1Cal[1][1] + c_hardIron1_z * mag1Cal[1][2]) / sqrt(sq(c_hardIron1_x) + sq(c_hardIron1_y) + sq(c_hardIron1_z));
  c_magnetom1_z = (c_hardIron1_x * mag1Cal[2][0] + c_hardIron1_y * mag1Cal[2][1] + c_hardIron1_z * mag1Cal[2][2]) / sqrt(sq(c_hardIron1_x) + sq(c_hardIron1_y) + sq(c_hardIron1_z));

  c_hardIron2_x = magnetom2_x * magConversion - mag2Offset[0];
  c_hardIron2_y = magnetom2_y * magConversion - mag2Offset[1];
  c_hardIron2_z = magnetom2_z * magConversion - mag2Offset[2];
  c_magnetom2_x = (c_hardIron2_x * mag2Cal[0][0] + c_hardIron2_y * mag2Cal[0][1] + c_hardIron2_z * mag2Cal[0][2]) / sqrt(sq(c_hardIron2_x) + sq(c_hardIron2_y) + sq(c_hardIron2_z));
  c_magnetom2_y = (c_hardIron2_x * mag2Cal[1][0] + c_hardIron2_y * mag2Cal[1][1] + c_hardIron2_z * mag2Cal[1][2]) / sqrt(sq(c_hardIron2_x) + sq(c_hardIron2_y) + sq(c_hardIron2_z));
  c_magnetom2_z = (c_hardIron2_x * mag2Cal[2][0] + c_hardIron2_y * mag2Cal[2][1] + c_hardIron2_z * mag2Cal[2][2]) / sqrt(sq(c_hardIron2_x) + sq(c_hardIron2_y) + sq(c_hardIron2_z));

  // Tilt compensated Magnetic filed X:
  MAG1_X = c_magnetom1_x * cos_pitch1 + c_magnetom1_y * sin_roll1 * sin_pitch1 + c_magnetom1_z * cos_roll1 * sin_pitch1;
  MAG2_X = c_magnetom2_x * cos_pitch2 + c_magnetom2_y * sin_roll2 * sin_pitch2 + c_magnetom2_z * cos_roll2 * sin_pitch2;

  // Tilt compensated Magnetic filed Y:
  MAG1_Y = c_magnetom1_y * cos_roll1 - c_magnetom1_z * sin_roll1;
  MAG2_Y = c_magnetom2_y * cos_roll2 - c_magnetom2_z * sin_roll2;
  // Magnetic Heading
  MAG1_Heading = atan2(-MAG1_Y, MAG1_X);
  MAG2_Heading = atan2(-MAG2_Y, MAG2_X);

//  MAG1_Heading = atan2(-c_magnetom1_y, c_magnetom1_x);
//  MAG2_Heading = atan2(-c_magnetom2_y, c_magnetom2_x);
}
