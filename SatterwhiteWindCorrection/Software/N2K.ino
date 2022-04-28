//NMEA 2000 message handler
void HandleNMEA2000Msg(const tN2kMsg &N2kMsg) {
  int iHandler;
  // Find handler
  for (iHandler = 0; NMEA2000Handlers[iHandler].PGN != 0 && !(N2kMsg.PGN == NMEA2000Handlers[iHandler].PGN); iHandler++);
  if (NMEA2000Handlers[iHandler].PGN != 0) {
    NMEA2000Handlers[iHandler].Handler(N2kMsg);
  }
}

void ApparentWind(const tN2kMsg &N2kMsg) {
  double windSpeed;
  double windDir;
  bool ParseSuccessful;
  
  ParseSuccessful = ParseN2kWindSpeed(N2kMsg, SID, windSpeed, windDir, windRef);
  if (!ParseSuccessful)
  {
    Serial.println("Failure parsing N2K Wind Message");
  }
  if (ParseSuccessful && SID !=23) {
    WindSpeed = windSpeed;
    WindAngle = windDir;
  }
}

void SendN2kWind(double windSpeed, double windDir) {
  static unsigned long WindUpdated=millis();
  tN2kMsg N2kMsg;

  if ( WindUpdated+WindUpdatePeriod<millis() ) {
    SetN2kWindSpeed(N2kMsg, 1, windSpeed, windDir,N2kWind_Apprent);
    WindUpdated=millis();
    NMEA2000.SendMsg(N2kMsg);
  }
}
