#include <Arduino.h>
#include <NMEA2000_CAN.h>  // This will automatically choose right CAN library and create suitable NMEA2000 object
#include <N2kMessages.h>
#include <N2kMsg.h>
#include <NMEA2000.h>

unsigned char SID;
double windSpeed;
double windDir;

bool ParseSuccessful;

tN2kWindReference windRef;
tN2kMsg N2kMsg(0);

typedef struct {
  unsigned long PGN;
  void (*Handler)(const tN2kMsg &N2kMsg);
} tNMEA2000Handler;


void ApparentWind(const tN2kMsg &N2kMsg);

tNMEA2000Handler NMEA2000Handlers[] = {
  {130306L, &ApparentWind},
  {0, 0}
};

void ApparentWind(const tN2kMsg &N2kMsg);

// Forward declarations for led blinking
void LedOn(unsigned long OnTime);
void UpdateLedState();
void HandleNMEA2000Msg(const tN2kMsg &N2kMsg);

// List here messages your device will transmit.
const unsigned long TransmitMessages[] PROGMEM = {130306L, 0};


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


  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  NMEA2000.SetN2kCANMsgBufSize(8);
  NMEA2000.SetN2kCANReceiveFrameBufSize(100);
  NMEA2000.SetForwardStream(&Serial);  // PC output on due native port
  NMEA2000.SetForwardType(tNMEA2000::fwdt_Text); // Show in clear text
  NMEA2000.SetMsgHandler(HandleNMEA2000Msg);

  NMEA2000.SetMode(tNMEA2000::N2km_ListenAndNode, 23);
  NMEA2000.ExtendTransmitMessages(TransmitMessages);

  NMEA2000.EnableForward(false);

  NMEA2000.Open();
}

void loop() {

  NMEA2000.ParseMessages();
  if ( Serial.available() ) {
    Serial.read();
  }
  UpdateLedState();

  Serial.print(ParseSuccessful);
  Serial.print("    ");
  Serial.print(millis());
  Serial.print(' ');
  Serial.print(windSpeed);
  Serial.print(' ');
  Serial.print(windDir);
  Serial.println();

    SendN2kWind(windSpeed+5.15,windDir+.785);
//  SendN2kWind(5.15, .785);

}



// Code below is just for handling led blinking.

#define LedOnTime 2
#define LedBlinkTime 1000
unsigned long TurnLedOffTime = 0;
unsigned long TurnLedOnTime = millis() + LedBlinkTime;

//*****************************************************************************
void LedOn(unsigned long OnTime) {
  digitalWrite(LED_BUILTIN, HIGH);
  TurnLedOffTime = millis() + OnTime;
  TurnLedOnTime = 0;
}

//*****************************************************************************
void UpdateLedState() {
  if ( TurnLedOffTime > 0 && TurnLedOffTime < millis() ) {
    digitalWrite(LED_BUILTIN, LOW);
    TurnLedOffTime = 0;
    TurnLedOnTime = millis() + LedBlinkTime;
  }

  if ( TurnLedOnTime > 0 && TurnLedOnTime < millis() ) LedOn(LedBlinkTime);
}

//*****************************************************************************
//NMEA 2000 message handler
void HandleNMEA2000Msg(const tN2kMsg &N2kMsg) {
  int iHandler;
  // Find handler
  for (iHandler = 0; NMEA2000Handlers[iHandler].PGN != 0 && !(N2kMsg.PGN == NMEA2000Handlers[iHandler].PGN); iHandler++);
  if (NMEA2000Handlers[iHandler].PGN != 0) {
    NMEA2000Handlers[iHandler].Handler(N2kMsg);
  }
}

#define WindUpdatePeriod 1000

void SendN2kWind(double windSpeed, double windDir) {
  static unsigned long WindUpdated = millis();
  tN2kMsg N2kMsg;

  if ( WindUpdated + WindUpdatePeriod < millis() ) {
    SetN2kWindSpeed(N2kMsg, 1, windSpeed, windDir, N2kWind_Apprent);
    WindUpdated = millis();
    NMEA2000.SendMsg(N2kMsg);
  }
}


void ApparentWind(const tN2kMsg &N2kMsg) {
  ParseSuccessful = ParseN2kWindSpeed(N2kMsg, SID, windSpeed, windDir, windRef);
  if (ParseSuccessful) {
    Serial.print("Wind Speed:");
    Serial.println(windSpeed);
    Serial.print("Wind Direction");
    Serial.println(windDir);
  }
}
