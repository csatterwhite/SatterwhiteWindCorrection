


//-----------------------------INCLUDES--------------------------------------
#include <Arduino.h>
#include <NMEA2000_CAN.h>  // This will automatically choose right CAN library and create suitable NMEA2000 object
#include <N2kMessages.h>
#include <N2kMsg.h>
#include <NMEA2000.h>

int ProgramState;
long timer=0;

//------------------N2K DEFINITIONS AND DECLARATIONS------------------------
#define WindUpdatePeriod 1000

typedef struct {
  unsigned long PGN;
  void (*Handler)(const tN2kMsg &N2kMsg);
} tNMEA2000Handler;

tN2kWindReference windRef;
tN2kMsg N2kMsg(0);

void Rudder(const tN2kMsg &N2kMsg);
void ApparentWind(const tN2kMsg &N2kMsg);

unsigned char SID;
double WindAngle;
double WindSpeed;
double CorrectedWindAngle;


tNMEA2000Handler NMEA2000Handlers[] = {
  {130306L, &ApparentWind},
  {127245L, &Rudder}
};

// Forward declarations for led blinking
void LedOn(unsigned long OnTime);
void UpdateLedState();
void HandleNMEA2000Msg(const tN2kMsg &N2kMsg);

// List here messages your device will transmit.
const unsigned long TransmitMessages[] PROGMEM={130306L,0};

// Define resistor values used in voltage divider circuit Vread = Vsense*R2/(R1+R2)
int r1
int r2

double mastRot
double rot1
double rot2
int pin1
int pin2

double voltScale
double voltRead
double voltReadInv
double voltSense
double voltSenseInv

// SPS-A100D-HAWS Calibration Data
double voltA = 0.45;
double voltAinv = 4.55;
double angleA = -50;
double voltB = 4.55;
double voltBinv = 0.45;
double angleB = 50;

#define ToRad(x) ((x)*PI/180)  // *pi/180
#define ToDeg(x) ((x)*180/PI)  // *180/pi

void setup()
{
  Serial.begin(115200);
  pinMode (STATUS_LED, OUTPUT); // Status LED
  
  //-----------------------N2K Device Setup-----------------------
  
  // Set Product information
  NMEA2000.SetProductInformation("00000002", // Manufacturer's Model serial code
                                 100, // Manufacturer's product code
                                 "Satterwhite Rotation Correction",  // Manufacturer's Model ID
                                 "2.0.0 (2022-04-28)",  // Manufacturer's Software version code
                                 "2.0.0 (2022-04-28)" // Manufacturer's Model version
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
  
// Calculate voltage divider scaling
  voltScale = R2/(R1+R2);
}

void loop() //Main Loop
{

  if ((millis() - timer) >= 100) // Main loop runs at 10Hz
  {
    ProgramState = 1;
    Serial.print(ProgramState);
    
    voltRead = analogRead(pin1);
    voltReadInv = analogRead(pin2);
    voltSense = voltRead/voltScale;
    voltSenseInv = voltReadInv/voltScale;

    rot1 = (AngleB-AngleA)/(voltB-VoltA)*voltSense+AngleA;
    rot2 = (AngleB-AngleA)/(voltBinv-voltAinv)*voltSenseInv+AngleA;
    mastRot = (rot1+rot2)/2;

    // Correct Wind Speed and Send to Display
    ProgramState = 2;
    Serial.print(ProgramState);
    NMEA2000.ParseMessages();
    ProgramState = 8;
    Serial.print(ProgramState);
    if ( Serial.available() ) { Serial.read(); } 
//    UpdateLedState();
  
    CorrectedWindAngle = WindAngle+toRad(mastRot);

    while (CorrectedWindAngle < 0)
    {
      CorrectedWindAngle = CorrectedWindAngle + 2*PI;
    }
    while (CorrectedWindAngle > 2*PI)
    {
      CorrectedWindAngle = CorrectedWindAngle - 2*PI;
    }
  
    SendN2kWind(WindSpeed,CorrectedWindAngle);

    Serial.print("  Output Rotation = ");
    Serial.print(rot1);
    Serial.print(", Inverted Output Rotation = ");
    Serial.print(rot2);
    Serial.print(", Mast Rotation = ");
    Serial.print(mastRot);
    Serial.print(", Raw Wing Angle = ");
    Serial.print(ToDeg(WindAngle));
    Serial.print(", Corrected Wind Angle = ");
    Serial.println(ToDeg(CorrectedWindAngle));
  }

}
