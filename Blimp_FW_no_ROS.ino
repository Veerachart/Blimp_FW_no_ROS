#include <CurieBLE.h>

#define LEDBLINK_PIN  13
int LEDBLINK_MS = 1000;

BLEPeripheral blePeripheral;

BLEService batteryService("180F"); // BLE Battery Service
BLEUnsignedCharCharacteristic batteryLevelChar("2A19",  // standard 16-bit characteristic UUID
    BLERead | BLENotify);     // remote clients will be able to

BLEService motorsService("301c9b00-a61b-408a-a8bf-5efcd95a3486");
BLECharacteristic motorsChar("301c9b01-a61b-408a-a8bf-5efcd95a3486", BLERead | BLEWriteWithoutResponse, 8);
//BLEShortCharacteristic m2Char("301c9b02-a61b-408a-a8bf-5efcd95a3486", BLERead | BLEWrite);
//BLEShortCharacteristic m3Char("301c9b03-a61b-408a-a8bf-5efcd95a3486", BLERead | BLEWrite);
//BLEShortCharacteristic m4Char("301c9b04-a61b-408a-a8bf-5efcd95a3486", BLERead | BLEWrite);
const unsigned char *motors;

BLEService commandService("301c9b20-a61b-408a-a8bf-5efcd95a3486");
BLECharacteristic commandChar(   "301c9b21-a61b-408a-a8bf-5efcd95a3486", BLEWriteWithoutResponse, 8);
//BLEShortCharacteristic frontCommandChar(   "301c9b21-a61b-408a-a8bf-5efcd95a3486", BLEWrite);
//BLEShortCharacteristic turnCommandChar(    "301c9b22-a61b-408a-a8bf-5efcd95a3486", BLEWrite);
//BLEShortCharacteristic verticalCommandChar("301c9b23-a61b-408a-a8bf-5efcd95a3486", BLEWrite);
//BLEShortCharacteristic sideCommandChar(    "301c9b24-a61b-408a-a8bf-5efcd95a3486", BLEWrite);
const unsigned char *cmd;

int powerPins[] = {9,3,5,6};
//int powerPins[] = {3,2,6,9};
//int powerPin1 = 3;
//int powerPin2 = 5;
//int powerPin3 = 6;
//int powerPin4 = 9;

int phasePins[] = {8,2,4,7};
//int phasePins[] = {2,4,7,8};
//int phasePin1 = 2;
//int phasePin2 = 4;
//int phasePin3 = 7;
//int phasePin4 = 8;

short int powers[] = {0,0,0,0};

short int oldBatteryLevel = 0;  // last battery level reading from analog input
long previousMillis = 0;  // last time the battery level was checked, in ms


void setup() {
  // put your setup code here, to run once:
  for (int i=0; i<4; i++) {
    pinMode(phasePins[i], OUTPUT);
    pinMode(powerPins[i], OUTPUT);
    analogWriteFrequency(powerPins[i],25000);
  }
  /*pinMode(phasePin1, OUTPUT);
  pinMode(powerPin1, OUTPUT);
  pinMode(phasePin2, OUTPUT);
  pinMode(powerPin2, OUTPUT);
  pinMode(phasePin3, OUTPUT);
  pinMode(powerPin3, OUTPUT);
  pinMode(phasePin4, OUTPUT);
  pinMode(powerPin4, OUTPUT);*/
  pinMode(LEDBLINK_PIN, OUTPUT);

  //Serial.begin(9600);
  
  blePeripheral.setLocalName("BLIMP");
  blePeripheral.setAdvertisedServiceUuid(batteryService.uuid());  // add the service UUID
  blePeripheral.addAttribute(batteryService);   // Add the BLE Battery service
  blePeripheral.addAttribute(batteryLevelChar); // add the battery level characteristic
  batteryLevelChar.setValue(oldBatteryLevel);   // initial value for this characteristic

  blePeripheral.setAdvertisedServiceUuid(motorsService.uuid());  // add the service UUID
  blePeripheral.addAttribute(motorsService);   // Add the BLE Battery service
  blePeripheral.addAttribute(motorsChar); // add M1 characteristic
  //motorsService.addCharacteristic(m2Char); // add M2 characteristic
  //motorsService.addCharacteristic(m3Char); // add M3 characteristic
  //motorsService.addCharacteristic(m4Char); // add M4 characteristic
  
  blePeripheral.setAdvertisedServiceUuid(commandService.uuid());
  blePeripheral.addAttribute(commandService);
  blePeripheral.addAttribute(commandChar);
  //commandService.addCharacteristic(turnCommandChar);
  //commandService.addCharacteristic(verticalCommandChar);
  //commandService.addCharacteristic(sideCommandChar);
  
  // assign event handlers for connected, disconnected to peripheral
  blePeripheral.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  blePeripheral.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);

  motorsChar.setEventHandler(BLEWritten, motorsCharacteristicWritten);
  //motorsChar.setValue(0x00000000);
  //m2Char.setEventHandler(BLEWritten, m2CharacteristicWritten);
  //m2Char.setValue(0);
  //m3Char.setEventHandler(BLEWritten, m3CharacteristicWritten);
  //m3Char.setValue(0);
  //m4Char.setEventHandler(BLEWritten, m4CharacteristicWritten);
  //m4Char.setValue(0);

  commandChar.setEventHandler(BLEWritten, commandCharacteristicWritten);
  //commandChar.setValue(0);
  //frontCommandChar.setEventHandler(BLEWritten, frontCommandCharacteristicWritten);
  //frontCommandChar.setValue(0);
  //turnCommandChar.setEventHandler(BLEWritten, turnCommandCharacteristicWritten);
  //turnCommandChar.setValue(0);
  //verticalCommandChar.setEventHandler(BLEWritten, verticalCommandCharacteristicWritten);
  //verticalCommandChar.setValue(0);
  //sideCommandChar.setEventHandler(BLEWritten, sideCommandCharacteristicWritten);
  //sideCommandChar.setValue(0);

  // start advertising
  blePeripheral.begin();

  //Serial.println("Bluetooth device active, waiting for connections...");
}

void blePeripheralConnectHandler(BLECentral& central) {
  // central connected event handler 
  // Blink faster when connected
  LEDBLINK_MS = 250;
  //Serial.print("Connected event, central: ");
  //Serial.println(central.address());
}

void blePeripheralDisconnectHandler(BLECentral& central) {
  // central disconnected event handler
  // Blink every second, wait for connection
  LEDBLINK_MS = 1000;
  //Serial.print("Disconnected event, central: ");
  //Serial.println(central.address());
}

void driveMotors() {
  for (int i=0; i<4; i++) {
    if (powers[i] >= 0) {
      digitalWrite(phasePins[i], HIGH);
      analogWrite(powerPins[i], powers[i]);
    }
    else {
      digitalWrite(phasePins[i], LOW);
      analogWrite(powerPins[i], -powers[i]);
    }
  }
  /*if (power1 >= 0) {
    digitalWrite(phasePin1, HIGH);
    analogWrite(powerPin1, power1);
  }
  else {
    digitalWrite(phasePin1, LOW);
    analogWrite(powerPin1, -power1);
  }
  if (power2 >= 0) {
    digitalWrite(phasePin2, HIGH);
    analogWrite(powerPin2, power2);
  }
  else {
    digitalWrite(phasePin2, LOW);
    analogWrite(powerPin2, -power2);
  }
  if (power3 >= 0) {
    digitalWrite(phasePin3, HIGH);
    analogWrite(powerPin3, power3);
  }
  else {
    digitalWrite(phasePin3, LOW);
    analogWrite(powerPin3, -power3);
  }
  if (power4 >= 0) {
    digitalWrite(phasePin4, HIGH);
    analogWrite(powerPin4, power4);
  }
  else {
    digitalWrite(phasePin4, LOW);
    analogWrite(powerPin4, -power4);
  }*/
}

void motorsCharacteristicWritten(BLECentral& central, BLECharacteristic& characteristic) {
  //Serial.println("Got motor values");
  //power1 = m1Char.value();
  motors = characteristic.value();
  for (int i=0; i<4; i++) {
    powers[i] = motors[2*i] << 8 | motors[2*i+1];
  }
  /*char buffer[4];
  for (int i=0; i<4; i++) {
    buffer[2] = motors[2*i];
    buffer[3] = motors[2*i+1];
    if (motors[2*i] == 255) {
      buffer[0] = 0xFF;
      buffer[1] = 0xFF;
    }
    else {
      buffer[0] = 0x00;
      buffer[1] = 0x00;
    }
    powers[i] = atoi(buffer);
    Serial.print(powers[i]);
  }
  */
  //Serial.println();
  driveMotors();
}

/*
void m1CharacteristicWritten(BLEDevice central, BLECharacteristic characteristic) {
  power1 = m1Char.value();
  driveMotors();
}

void m2CharacteristicWritten(BLEDevice central, BLECharacteristic characteristic) {
  power2 = m1Char.value();
  driveMotors();
}

void m3CharacteristicWritten(BLEDevice central, BLECharacteristic characteristic) {
  power3 = m1Char.value();
  driveMotors();
}

void m4CharacteristicWritten(BLEDevice central, BLECharacteristic characteristic) {
  power4 = m1Char.value();
  driveMotors();
}
*/

void commandCharacteristicWritten(BLECentral& central, BLECharacteristic& characteristic) {
  cmd = characteristic.value();
  short int commands[4];
  for (int i=0; i<4; i++) {
    commands[i] = cmd[2*i] << 8 | cmd[2*i+1];
    //Serial.print(commands[i]);
    //Serial.print(" ");
  }
  //Serial.println();
  
  powers[0] = max(min(-commands[0] + commands[1], 255),-255);
  powers[1] = max(min(commands[0] + commands[1], 255),-255);
  powers[2] = commands[2];
  powers[3] = commands[3];
  //power1 = frontCommandChar.value();
  //power2 = frontCommandChar.value();
  driveMotors();
}

/*
void frontCommandCharacteristicWritten(BLEDevice central, BLECharacteristic characteristic) {
  power1 = frontCommandChar.value();
  power2 = frontCommandChar.value();
  driveMotors();
}

void turnCommandCharacteristicWritten(BLEDevice central, BLECharacteristic characteristic) {
  power1 += turnCommandChar.value();
  power2 -= turnCommandChar.value();
  driveMotors();
}

void verticalCommandCharacteristicWritten(BLEDevice central, BLECharacteristic characteristic) {
  power3 = verticalCommandChar.value();
  driveMotors();
}

void sideCommandCharacteristicWritten(BLEDevice central, BLECharacteristic characteristic) {
  power4 += sideCommandChar.value();
  driveMotors();
}
*/

void updateBatteryLevel() {
  /* Read the current voltage level on the A0 analog input pin.
     This is used here to simulate the charge level of a battery.
  */
  int battery = analogRead(A0);
  short int batteryLevel = map(battery, 800, 1023, 0, 100);

  if (batteryLevel != oldBatteryLevel) {      // if the battery level has changed
    //Serial.print("Battery Level % is now: "); // print it
    //Serial.println(batteryLevel);
    batteryLevelChar.setValue(batteryLevel);  // and update the battery level characteristic
    oldBatteryLevel = batteryLevel;           // save the level for next comparison
    if (batteryLevel < 15) {
      // Low battery, below 25%
      LEDBLINK_MS = 100;
    }
  }
}

void loop() {
  long currentMillis = millis();
  // if 200ms have passed, check the battery level:
  if (currentMillis - previousMillis >= 200) {
    previousMillis = currentMillis;
    updateBatteryLevel();
  }
  blePeripheral.poll();
  ledBlink();
}

//
// LED Heartbeat routine by Allen C. Huffman (www.appleause.com)
//
void ledBlink()
{
  static unsigned int  ledStatus = LOW;  // Last set LED mode.
  static unsigned long ledBlinkTime = 0; // LED blink time.

  // LED blinking heartbeat. Yes, we are alive.
  // For explanation, see:
  // http://playground.arduino.cc/Code/TimingRollover
  if ( (long)(millis()-ledBlinkTime) >= 0 )
  {
    // Toggle LED.
    ledStatus = (ledStatus==HIGH ? LOW : HIGH);

    // Set LED pin status.
    digitalWrite(LEDBLINK_PIN, ledStatus);

    // Reset "next time to toggle" time.
    ledBlinkTime = millis()+LEDBLINK_MS;
  }
}
