//CANSAT 2020 - Equipo Josellite - IES Ramon y Cajal, Albacete (Spain)
//Programa para adquisicion de datos de BMP280, MPU9250, UV y GPS con LORA-ESP32O-LED

//Board: TTGO LoRa32 -OLEDv1


//IMPORTANTE: para la placa LORA-ESP32 con OLED incorporado,
//de los dos puertos de comunicación serie hardware disponibles del ESP32,
//para el 2 tenemos el pin RX2 (GPIO16) ocupado por la pantalla OLED
//y para el 1 (RX1) no tenemos disponible en la placa el GPIO09.
//Por lo tanto es necesario reconfigurar, para el 1, el GPIO09 
//por el GPIO13. Asi, editaremos el archivo HardwareSerial.cpp
//de libreria (ruta para MACOS, puede que sea distinta en Windows):
// /Users/username/Library/Arduino15/packages/
//esp32/hardware/esp32/1.0.4/cores/esp32/HardwareSerial.cpp
//y cambiaremos la linea #define RX1 9 por #define RX1 13
//Ahora podremos conectar el cable del TX del GPS al pin 13
//(GPIO13) de la placa.


//BMP280+MPU y UV por I2C a los pines de LORA-ESP:
//SCL -> GPIO15
//SDA -> GPIO04
//GPS al pin GPIO13

#include <Arduino.h>
#include <Wire.h>
#include <LoRa.h>
#include <TinyGPS++.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "Seeed_BMP280.h"
#include "MPU9250.h"
#include <SparkFun_VEML6075_Arduino_Library.h>


//define the pins used by the LoRa transceiver module
#define SCK 5
#define MISO 19
#define MOSI 27
#define SS 18
#define RST 14
#define DIO0 26

//433E6 for Asia
//866E6 for Europe
//915E6 for North America
#define BAND 866E6

//OLED pins
//pines I2C  para TODOS los sensores (BMP280-MPU, UV).
#define OLED_SDA 4  
#define OLED_SCL 15 

#define OLED_RST 16
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
//#define I2C_ADDRESS 0x76
#define RXD2 16
#define TXD2 17


Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RST);
TinyGPSPlus gps;
BMP280 bmp280; //0x76
MPU9250 IMU(Wire,0x68);
int status;
// UV 0x10
VEML6075 uv;


void setup() {
  // put your setup code here, to run once:
	Serial.begin(9600);
  Serial1.begin(9600);

  //reset OLED display via software
  pinMode(OLED_RST, OUTPUT);
  digitalWrite(OLED_RST, LOW);
  delay(20);
  digitalWrite(OLED_RST, HIGH);

  //initialize OLED
  Wire.begin(OLED_SDA, OLED_SCL);
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3c, false, false)) { // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
  }
  
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0,0);
  display.print("LORA SENDER ");
  display.display();
  
  Serial.println("LoRa Sender Data");

  //SPI LoRa pins
  SPI.begin(SCK, MISO, MOSI, SS);
  //setup LoRa transceiver module
  LoRa.setPins(SS, RST, DIO0);
  
  if (!LoRa.begin(BAND)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  Serial.println("LoRa Initializing OK!");
  display.setCursor(0,10);
  display.print("LoRa Initializing OK!");
  display.display();
  
  if (!uv.begin())
  {
    Serial.println("Unable to communicate with VEML6075.");
   
  }
  
  // start communication with IMU 
  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    //while(1) {}
  }

  
	//begin() checks the Interface, reads the sensor ID (to differentiate between BMP280 and BME280)
	//and reads compensation parameters.
	if(!bmp280.init())
    Serial.println("BMP280 error!");
	else Serial.println("BMP280 ok!");
	
}

void loop() {
  // put your main code here, to run repeatedly:
  float pressure;
  //delay(3000);

	//start a measurement
   //get and print temperatures
  Serial.println("****** BMP280 *******");
  Serial.print("Temp: ");
  Serial.print(bmp280.getTemperature());
  Serial.println("C"); // The unit for  Celsius because original arduino don't support speical symbols
  
  //get and print atmospheric pressure data
  Serial.print("Pressure: ");
  Serial.print(pressure = bmp280.getPressure());
  Serial.println("Pa");
  
  //get and print altitude data
  Serial.print("Altitude: ");
  Serial.print(bmp280.calcAltitude(pressure));
  Serial.println("m");
  
  IMU.readSensor();
  Serial.println("****** IMU *******");
  Serial.println("AccelXYZ");
  Serial.print(IMU.getAccelX_mss(),6);
  Serial.print("\t");
  Serial.print(IMU.getAccelY_mss(),6);
  Serial.print("\t");
  Serial.print(IMU.getAccelZ_mss(),6);
  Serial.println("\t");
  Serial.println("GyroXYZ");
  Serial.print(IMU.getGyroX_rads(),6);
  Serial.print("\t");
  Serial.print(IMU.getGyroY_rads(),6);
  Serial.print("\t");
  Serial.print(IMU.getGyroZ_rads(),6);
  Serial.println("\t");
  Serial.println("MagXYZ");
  Serial.print(IMU.getMagX_uT(),6);
  Serial.print("\t");
  Serial.print(IMU.getMagY_uT(),6);
  Serial.print("\t");
  Serial.print(IMU.getMagZ_uT(),6);
  Serial.println("\t");
  Serial.println("****** GPS *******");

  
  Serial.print(gps.location.lat(),6);      // Latitud GPS
  Serial.print(",\t");
  Serial.print(gps.location.lng(),6);      // Longitud GPS
  Serial.print(",\t");
  Serial.println(gps.altitude.meters());   // Altitud GPS

  Serial.println("****** UV *******");
  Serial.println(String(uv.a()) + ", " + String(uv.b()) + ", " +
                String(uv.uvComp1()) + ", " + String(uv.uvComp2()) + ", " + 
                String(uv.index()));
 
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0,0);
  display.print(IMU.getMagX_uT());
  display.setCursor(50,0);
  display.print(IMU.getMagY_uT());
  display.setCursor(90,0);
  display.print(IMU.getMagZ_uT());
  display.setCursor(0,10);
  display.print("Temperatura:");
  display.setCursor(75,10);
  display.print(bmp280.getTemperature());        
  display.setCursor(0,20);  
  display.print("Altitud:");
  display.setCursor(50,20);
  display.print(bmp280.calcAltitude(pressure));      
  display.setCursor(0,30);  
  display.print("Indice UV:");
  display.setCursor(70,30);
  display.print(uv.index());
  display.setCursor(0,40);  
  display.print("GPS latitud:");
  display.setCursor(80,40);
  display.print(gps.location.lat());
  display.setCursor(0,50);  
  display.print("GPS longitud:");
  display.setCursor(80,50);
  display.print(gps.location.lng());
  
  display.display();            
  smartDelay(500);
}

static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (Serial1.available())
      gps.encode(Serial1.read());
  } while (millis() - start < ms);
}
