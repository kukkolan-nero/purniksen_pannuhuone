#include <Arduino.h>
#include <OneWire.h>
#include <LiquidCrystal_I2C.h>
#include <DallasTemperature.h>

#define LCD_SDA 18 // LCD-näytöt I2C-väylään
#define LCD_SCL 19
#define LAMPOANTURIT 10 // Dallas DS18xx(x)
#define DEBUG 14 // DEBUG-painikkeen pinni
#define ALARAJA 21
#define YLARAJA 20

boolean debugMode = false;

LiquidCrystal_I2C lcd(0x27, 20, 4);

OneWire anturivayla(LAMPOANTURIT);
DallasTemperature anturit(&anturivayla);
DeviceAddress pannuhuone, alaputki;


uint8_t aste[8] = {0b00111, 0b00101, 0b00111, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000};

// function to print a device address
// example directly from DallasTemperature by Miles Burton
void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}

void setup() {
  byte lcdRivi = 0;

  Serial.begin(9600);

  pinMode(ALARAJA, INPUT);
  pinMode(YLARAJA, INPUT);
  pinMode(DEBUG, INPUT);

  if (digitalRead(DEBUG))
    debugMode = true;
  
  lcd.init();
  lcd.backlight();
  lcd.createChar(0, aste);
  lcd.clear();
  lcd.print("PURNIKSEN PANNUHUONE");
  lcd.setCursor(0, ++lcdRivi);
  if (debugMode) {
    lcd.print("Debug-tila paalla");
    lcd.setCursor(0, ++lcdRivi);
  }
  
  anturit.begin();

  if (!anturit.getAddress(pannuhuone, 0)) {
    if (debugMode) Serial.println("Anturi 0: Ei osoitetta");
    lcd.print("Anturi 0: No addr");
    lcd.setCursor(0, ++lcdRivi);
  } else {
    lcd.print("Anturi 0: ok");
    lcd.setCursor(0, ++lcdRivi);
    if (debugMode) {
      Serial.print("Anturi 0: osoite: ");
      printAddress(pannuhuone);
      Serial.println();
    }
  }

  if (!anturit.getAddress(alaputki, 1)) {
    if (debugMode) Serial.println("Anturi 1: Ei osoitetta");
    lcd.print("Anturi 1: No addr");
    lcd.setCursor(0, ++lcdRivi);
  } else {
    lcd.print("Anturi 1: ok");
    lcd.setCursor(0, ++lcdRivi);
    if (debugMode) {
      Serial.print("Anturi 1: osoite: ");
      printAddress(alaputki);
      Serial.println();
    }
  }
  delay(2000);
}

void lcdAste() {
  lcd.write(aste);
  lcd.print("C");
}

void loop() {
  byte lcdRivi = 0;

  anturit.requestTemperatures();

  float lampoPannuhuone = anturit.getTempC(pannuhuone);
  float lampoAlaputki = anturit.getTempC(alaputki);
  
  int alaraja = analogRead(ALARAJA);
  int ylaraja = analogRead(YLARAJA);

  if (ylaraja <= alaraja)
    ylaraja = alaraja + 2;

  lcd.clear();
  lcd.print("Pannuhuone: ");
  lcd.print(lampoPannuhuone);
  lcdAste(); 
  lcd.setCursor(0, ++lcdRivi);
  lcd.print("Alaputki: ");
  lcd.print(lampoAlaputki);
  lcdAste();
  lcd.setCursor(0, ++lcdRivi);
  lcd.print("Rajat: ");
  lcd.print(alar);
  lcdAste();
  lcd.print(" - ");
  lcd.print(ylar);
  lcdAste();
}