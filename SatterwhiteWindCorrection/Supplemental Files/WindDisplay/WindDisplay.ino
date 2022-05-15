// Demo: NMEA2000 library. Read some data and their show values.
// See also DataDisplay2.ino, which handles more data.

// #define USE_DUE_CAN 1
#define N2K_SOURCE 15

#include <Arduino.h>
#include <NMEA2000_CAN.h>  // This will automatically choose right CAN library and create suitable NMEA2000 object
#include <N2kMsg.h>
#include <NMEA2000.h>
#include <N2kMessages.h>


typedef struct {
  unsigned long PGN;
  void (*Handler)(const tN2kMsg &N2kMsg); 
} tNMEA2000Handler;

void ApparentWind(const tN2kMsg &N2kMsg);
void WaterDepth(const tN2kMsg &N2kMsg);

tNMEA2000Handler NMEA2000Handlers[]={
  {130306L,&ApparentWind},
  {128267L,&WaterDepth},
  {0,0}
};

void setup() {
  Serial.begin(115200);
  // Do not forward bus messages at all
  NMEA2000.EnableForward(false);
  NMEA2000.SetMsgHandler(HandleNMEA2000Msg);
  NMEA2000.Open();
}

void WaterDepth(const tN2kMsg &N2kMsg) {
    unsigned char SID;
    double DepthBelowTransducer;
    double Offset;

    if (ParseN2kWaterDepth(N2kMsg,SID,DepthBelowTransducer,Offset) ) {
      if (Offset>0) {
        Serial.print("Water depth:");
      } else {
        Serial.print("Depth below keel:");
      }
      Serial.println(DepthBelowTransducer+Offset);
    }
}

void ApparentWind(const tN2kMsg &N2kMsg) {
  unsigned char SID;
  double windSpeed;
  double windDir;
  tN2kWindReference windRef;
  if (ParseN2kWindSpeed(N2kMsg,SID,windSpeed,windDir,windRef)) {
    Serial.print("Wind Speed:");
    Serial.println(windSpeed);
    Serial.print("Wind Direction");
    Serial.println(windDir);
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
}
