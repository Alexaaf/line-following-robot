#include <string.h>
#include <Arduino.h>
#include <SPI.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"

#if SOFTWARE_SERIAL_AVAILABLE
  #include <SoftwareSerial.h>
#endif

/*
    APPLICATION SETTINGS

    FACTORYRESET_ENABLE       Perform a factory reset when running this sketch
   
                              Enabling this will put your Bluefruit LE module
                              in a 'known good' state and clear any config
                              data set in previous sketches or projects, so
                              running this at least once is a good idea.
   
                              When deploying your project, however, you will
                              want to disable factory reset by setting this
                              value to 0.  If you are making changes to your
                              Bluefruit LE device via AT commands, and those
                              changes aren't persisting across resets, this
                              is the reason why.  Factory reset will erase
                              the non-volatile memory where config data is
                              stored, setting it back to factory default
                              values.
       
                              Some sketches that require you to bond to a
                              central device (HID mouse, keyboard, etc.)
                              won't work at all with this feature enabled
                              since the factory reset will clear all of the
                              bonding data stored on the chip, meaning the
                              central device won't be able to reconnect.
    MINIMUM_FIRMWARE_VERSION  Minimum firmware version to have some new features
    MODE_LED_BEHAVIOUR        LED activity, valid options are
                              "DISABLE" or "MODE" or "BLEUART" or
                              "HWUART"  or "SPI"  or "MANUAL"
    -----------------------------------------------------------------------*/
    #define FACTORYRESET_ENABLE         1
    #define MINIMUM_FIRMWARE_VERSION    "0.6.6"
    #define MODE_LED_BEHAVIOUR          "MODE"

Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

// function prototypes over in packetparser.cpp
uint8_t readPacket(Adafruit_BLE *ble, uint16_t timeout);
float parsefloat(uint8_t *buffer);
void printHex(const uint8_t * data, const uint32_t numBytes);

// the packet buffer
extern uint8_t packetbuffer[];


/**************************/
/*!
    @brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)
*/
/**************************/

//Right motor
//int enableRightMotor=6;
int rightMotorPin1=5;
int rightMotorPin2=11;

//Left motor
//int enableLeftMotor=5;
int leftMotorPin1=9;
int leftMotorPin2=10;

//IR sensors
int leftSensor = 12;
int rightSensor = 13;

bool flag = false;

void setup(void)
{
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);
  
  //pinMode(enableLeftMotor, OUTPUT);
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);

  pinMode(leftSensor, INPUT);
  pinMode(rightSensor, INPUT);
  
  while (!Serial);
  delay(500);
  
  Serial.begin(115200);
  Serial.println(F("Adafruit Bluefruit App Controller Example"));
  Serial.println(F("-----------------------------------------"));

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ){
      error(F("Couldn't factory reset"));
    }
  }


  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  Serial.println(F("Please use Adafruit Bluefruit LE app to connect in Controller mode"));
  Serial.println(F("Then activate/use the sensors, color picker, game controller, etc!"));
  Serial.println();

  ble.verbose(false);  // debug info is a little annoying after this point!

  /* Wait for connection */
  while (! ble.isConnected()) {
      delay(500);
  }

  Serial.println(F("**********"));

  // LED Activity command is only supported from 0.6.6
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    // Change Mode LED Activity
    Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
  }

  // Set Bluefruit to DATA mode
  Serial.println( F("Switching to DATA mode!") );
  ble.setMode(BLUEFRUIT_MODE_DATA);

  Serial.println(F("**********"));

}

/**************************/
/*!
    @brief  Constantly poll for new command or response data
*/
/**************************/
void loop(void)
{
  /* Wait for new data to arrive */
  //testFunction();
  uint8_t len = readPacket(&ble, BLE_READPACKET_TIMEOUT);
  if (len == 0) return;

  // Buttons
  if (packetbuffer[1] == 'B') {
    uint8_t buttnum = packetbuffer[2] - '0';
    boolean pressed = packetbuffer[3] - '0';
    Serial.print ("Button "); Serial.print(buttnum);
    if (pressed) {
      Serial.println(" pressed");
      if(buttnum == 5)
      {
          fataController();
      }
      //If right sensor detects black line, then turn right
      if(buttnum == 8)
      {
          dreaptaController();
      }
      //If left sensor detects black line, then turn left  
      if(buttnum == 7)
      {
         stangaController();
      } 

      if(buttnum == 1)
      {
        flag = true;
        lineFollowingProtocol();
              
      }
      if(buttnum == 2)
      {
        flag = false;
      }
      
    } else {

      stopAll();
      
    }
  }
  delay(100);

}

void testFunction()
{
  delay(10000);
  Serial.println("Secventa test 1");//dr spate
  digitalWrite(rightMotorPin1,HIGH);
  digitalWrite(rightMotorPin2,LOW);
  digitalWrite(leftMotorPin1,LOW);
  digitalWrite(leftMotorPin2,LOW); 
  delay(5000);
   Serial.println("Secventa test 2");//dr fata
  digitalWrite(rightMotorPin1,LOW);
  digitalWrite(rightMotorPin2,HIGH);
  digitalWrite(leftMotorPin1,LOW);
  digitalWrite(leftMotorPin2,LOW);
  delay(5000);
   Serial.println("Secventa test 3");//st spate
  digitalWrite(rightMotorPin1,LOW);
  digitalWrite(rightMotorPin2,LOW);
  digitalWrite(leftMotorPin1,HIGH);
  digitalWrite(leftMotorPin2,LOW);
  delay(5000);
   Serial.println("Secventa test 4");//st fata
  digitalWrite(rightMotorPin1,LOW);
  digitalWrite(rightMotorPin2,LOW);
  digitalWrite(leftMotorPin1,LOW);
  digitalWrite(leftMotorPin2,HIGH);
  delay(5000);
}

void fata()
{
  digitalWrite(rightMotorPin2,LOW); 
  digitalWrite(rightMotorPin1,HIGH); 
  digitalWrite(leftMotorPin1,HIGH);
  digitalWrite(leftMotorPin2,LOW);
  Serial.println("Fata");

  delay(10);
  stopAll();
  lineFollowingProtocol();
  

}

void dreapta()
{
  digitalWrite(rightMotorPin1,5);
  Serial.println("Dreapta");
  delay(10);
  stopAll();
  lineFollowingProtocol();
}

void stanga()
{
  digitalWrite(leftMotorPin1,5);          
  Serial.println("Stanga");
  delay(10);
  stopAll();
  lineFollowingProtocol();

}

void fataController()
{
  digitalWrite(rightMotorPin2,LOW); 
  digitalWrite(rightMotorPin1,HIGH); 
  digitalWrite(leftMotorPin1,HIGH);
  digitalWrite(leftMotorPin2,LOW);
  Serial.println("Fata");
}

void dreaptaController()
{
  digitalWrite(rightMotorPin1,5);
  Serial.println("Dreapta");
}

void stangaController()
{
  digitalWrite(leftMotorPin1,5);          
  Serial.println("Stanga");
}
void stopAll()
{
  Serial.println(" released");
      digitalWrite(rightMotorPin1,LOW);
      digitalWrite(rightMotorPin2,LOW); 
      digitalWrite(leftMotorPin1,LOW);
      digitalWrite(leftMotorPin2,LOW);
        delay(10);

}

void lineFollowingProtocol()
{
 if(flag)
 {
  int rightIRSensorValue = digitalRead(rightSensor);
  int leftIRSensorValue = digitalRead(leftSensor);

  //If none of the sensors detects black line, then go straight
  if (rightIRSensorValue == LOW && leftIRSensorValue == LOW)
  {
    fata();
  }
  //If right sensor detects black line, then turn right
  else if (rightIRSensorValue == HIGH && leftIRSensorValue == LOW )
  {
      dreapta();
  }
  //If left sensor detects black line, then turn left  
  else if (rightIRSensorValue == LOW && leftIRSensorValue == HIGH )
  {
      stanga();
  } 
  //If both the sensors detect black line, then stop 
  else 
  {
   stopAll();
  }
 }
}
