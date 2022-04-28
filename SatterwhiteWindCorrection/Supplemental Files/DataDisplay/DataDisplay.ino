// Demo: NMEA2000 library. Read some data and their show values.
// See also DataDisplay2.ino, which handles more data.

#define USE_DUE_CAN 1
//#define N2K_SOURCE 15

#include <Arduino.h>
#include <NMEA2000_CAN.h>  // This will automatically choose right CAN library and create suitable NMEA2000 object
#include <N2kMsg.h>
#include <NMEA2000.h>
#include <N2kMessages.h>


typedef struct {
  unsigned long PGN;
  void (*Handler)(const tN2kMsg &N2kMsg); 
} tNMEA2000Handler;

void WindData(const tN2kMsg &N2kMsg);
void WaterDepth(const tN2kMsg &N2kMsg);

int AnemometerSource = 0;
unsigned char SID;
double WindSpeed;
double WindDirection;

tNMEA2000Handler NMEA2000Handlers[]={
  {130306L,&WindData},
  {0,0}
};

void setup() {
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
  Serial.begin(115200);
  NMEA2000.SetN2kCANMsgBufSize(8);
  NMEA2000.SetN2kCANReceiveFrameBufSize(100);
  NMEA2000.SetMode(tNMEA2000::N2km_ListenAndNode,23);
  NMEA2000.SetForwardStream(&Serial);  // PC output on due native port
  NMEA2000.SetForwardType(tNMEA2000::fwdt_Text); // Show in clear text
  // Do not forward bus messages at all
//  NMEA2000.EnableForward(false);
  NMEA2000.SetMsgHandler(HandleNMEA2000Msg);
  NMEA2000.Open();
}


void WindData(const tN2kMsg &N2kMsg) {
    
    tN2kWindReference WindReference;
    double spd;
    double dir;
    ParseN2kWindSpeed(N2kMsg,SID,spd,dir,WindReference);
    // Only overwrite speed and direction if message is not coming from Arduino.
    if (SID != 23){
      WindSpeed = spd;
      WindDirection = dir;
    }
}

//NMEA 2000 message handler
void HandleNMEA2000Msg(const tN2kMsg &N2kMsg) {
  int iHandler;
  // Find handler
  for (iHandler=0; NMEA2000Handlers[iHandler].PGN!=0 && !(N2kMsg.PGN==NMEA2000Handlers[iHandler].PGN); iHandler++);
  if (NMEA2000Handlers[iHandler].PGN!=0) {
    NMEA2000Handlers[iHandler].Handler(N2kMsg); 
  }
}

void loop() {
  NMEA2000.ParseMessages();
  SendN2kWind(WindSpeed+5.15,WindDirection+.785);
}

#define WindUpdatePeriod 1000

void SendN2kWind(double windSpeed, double windDir) {
  static unsigned long WindUpdated=millis();
  tN2kMsg N2kMsg;

  if ( WindUpdated+WindUpdatePeriod<millis() ) {
    SetN2kWindSpeed(N2kMsg, 1, windSpeed, windDir,N2kWind_Apprent);
    WindUpdated=millis();
    NMEA2000.SendMsg(N2kMsg);
  }
}
