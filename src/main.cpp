#include <Arduino.h>
#include <OneWire.h>
#include <LiquidCrystal_I2C.h>
#include <DallasTemperature.h>
#include <SoftwareSerial.h>

#define LCD_SDA_PIN 18 // LCD-näytöt I2C-väylään
#define LCD_SCL_PIN 19
#define LCD_ROWS 4 // LCD-näytössä rivien määrä ...
#define LCD_COLS 20 // ... ja riville mahtuvien merkkien määrä

#define LAMPOANTURIT_PIN 10 // Dallas DS18xx(x)
#define DEBUG_PIN 14 // DEBUG_PIN-painikkeen pinni
#define ALARAJA_PIN 21
#define YLARAJA_PIN 20
#define WIFI_RX_PIN 5
#define WIFI_TX_PIN 6

// Hystereesin rajat lämpövastuksen käytölle, ADC_BITIT = Arduinon muunnostarkkuus
#define LAMPOTILAN_ALARAJA 2
#define LAMPOTILAN_YLARAJA 60
#define ADC_BITIT 10

/* Analogitulo muunnetaan lämpötilaksi kaavalla y=kx+b,
   missä y on lämpötila, x analogitulon arvo, k kulmakerroin ja b vakiotermi.
   Kulmakerroin ja vakiotermi saadaan ratkaistua kaavan y1-y0 = k(x1-x0) avulla.
   Huom. Koska adc-muunnoksen pienin arvo on 0, on aina x0 = 0. */
#define KULMAKERROIN (float)(LAMPOTILAN_YLARAJA-LAMPOTILAN_ALARAJA)/(1<<(ADC_BITIT))
#define VAKIOTERMI LAMPOTILAN_ALARAJA


/* Mittaustarkkuus voi olla
  bittiä    astetta C     muunnosaika (MS)
  9         0,5           93,75
  10        0,25          187,5
  11        0,125         375
  12        0,0625        750
  https://www.makerguides.com/ds18b20-arduino-tutorial

  Kaikille antureille asetetaan sama tarkkuus.
*/
#define DALLAS_RESOLUTION 9
#define DALLAS_MAX_RESOLUTION 12

// Kuinka kauan lämpötila-antureille annetaan millisekunneissa aikaa valmistella tulosta?
#define DALLAS_DELAY 750

// Montako millisekuntia sitten lämpötilaa pyydettiin viimeksi?
unsigned long lastTempRequest = 0;

boolean debugMode = false;
boolean vastusPaalla = false; // Lämpövastuksen tila
float lampoPannuhuone = -99.9; // Pannuhuoneen ilman lämpötila
float lampoAlaputki = -99.9; // Varaajan alaputken lämpötila
int ylaraja = 99999; // Alaputken lämpötilan yläraja, jolloin vastus sammutetaan
int alaraja = 9999; // Alaputken lämpötilan alaraja, jolloin vastus laitetaan päälle
char lcdStr[LCD_COLS]; // Yksi LCD-näytölle tulostettava rivi
char lampoStr[7];

LiquidCrystal_I2C lcd(0x27, LCD_COLS, LCD_ROWS);

OneWire anturivayla(LAMPOANTURIT_PIN);
DallasTemperature anturit(&anturivayla);
DeviceAddress pannuhuone, alaputki;

SoftwareSerial wifi(WIFI_RX_PIN, WIFI_TX_PIN);


// Asteen symboli LCD-näytöllä
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

  pinMode(ALARAJA_PIN, INPUT);
  pinMode(YLARAJA_PIN, INPUT);
  pinMode(DEBUG_PIN, INPUT);
  pinMode(LAMPOANTURIT_PIN, INPUT);

  if (digitalRead(DEBUG_PIN))
    debugMode = true;
  
  lcd.init();
  lcd.backlight();
  lcd.createChar(0, aste);
  lcd.clear();
  lcd.print("PURNIKSEN PANNUHUONE");
  lcd.setCursor(0, ++lcdRivi);
  if (debugMode) {
    lcd.print("Debug_PIN-tila paalla");
    lcd.setCursor(0, ++lcdRivi);
  }
  
  anturit.begin();

  wifi.begin(9600);
  if (debugMode) Serial.println("WIFI: Odotetaan sarjaporttia.");
  while (!wifi) {}
  if (debugMode) Serial.println("WIFI: Sarjaporttia ok.");


  if (!anturit.getAddress(pannuhuone, 0)) {
    if (debugMode) Serial.println("Anturi 0: Pannuhuone: Ei osoitetta");
    lcd.print("Anturi 0: No addr");
    lcd.setCursor(0, ++lcdRivi);
  } else {
    lcd.print("Anturi 0: ok");
    lcd.setCursor(0, ++lcdRivi);
    if (debugMode) {
      Serial.print("Anturi 0: Pannuhuone: osoite: ");
      printAddress(pannuhuone);
      Serial.println();
    }
  }

  if (!anturit.getAddress(alaputki, 1)) {
    if (debugMode) Serial.println("Anturi 1: Alaputki: Ei osoitetta");
    lcd.print("Anturi 1: No addr");
    lcd.setCursor(0, ++lcdRivi);
  } else {
    lcd.print("Anturi 1: ok");
    lcd.setCursor(0, ++lcdRivi);
    if (debugMode) {
      Serial.print("Anturi 1: Alaputki: osoite: ");
      printAddress(alaputki);
      Serial.println();
    }
  }

  anturit.setResolution(alaputki, DALLAS_RESOLUTION);
  anturit.setResolution(pannuhuone, DALLAS_RESOLUTION);
  anturit.setWaitForConversion(false);
  anturit.requestTemperatures();
  lastTempRequest = millis();
  if (debugMode) {
    Serial.print("lastTempRequest: ");
    Serial.println(lastTempRequest);
  }

  delay(2000);
  lcd.clear();
}

void lcdAste() {
  lcd.write(aste);
  lcd.print("C");
}

void lcdPaivita() {
  lcd.setCursor(0, 0);
  dtostrf(lampoPannuhuone, 6, 1, lampoStr);
  strcpy(lcdStr, "Pannuhuone: ");
  strcat(lcdStr, lampoStr);
  lcd.print(lcdStr);
  lcdAste();

  lcd.setCursor(0, 1);
  dtostrf(lampoAlaputki, 6, 1, lampoStr);
  strcpy(lcdStr, "Alaputki:   ");
  strcat(lcdStr, lampoStr);
  lcd.print(lcdStr);
  lcdAste();

  lcd.setCursor(0,2);
  strcpy(lcdStr, "Rajat: ");
  snprintf(lampoStr, 7, "%d", alaraja);
  strcat(lcdStr, lampoStr);
  strcat(lcdStr, " - ");
  snprintf(lampoStr, 7, "%d", ylaraja);
  strcat(lcdStr, lampoStr);
  lcd.print(lcdStr);
  lcdAste();
}

/*
  Muutetaan analogitulon arvo lämpötilaksi. Analogitulon arvoa
  säädetään potikalla. Kaavahan on y = kx + b, mitä tässä ratkotaan.
*/
float muunnaLampotilaksi(int tulonArvo)
{
  return KULMAKERROIN * tulonArvo + VAKIOTERMI;
}

void pollaaWifi() {
  while (wifi.available()) {
    char a = wifi.read();
    Serial.print(a);
  }
  while (Serial.available()) {
    char a = Serial.read();
    Serial.write(a);
    wifi.write(a);
  }

}

void loop() {
  /*
    millis() on unsigned long -tyyppiä ja pyörähtää ympäri n. 50 vrk:n kuluttua.
    Alla oleva testi on immuuni tilanteelle, ks.
    https://arduino.stackexchange.com/questions/12587/how-can-i-handle-the-millis-rollover/12588#12588
  */
  if (millis() - lastTempRequest > DALLAS_DELAY) {
    lampoPannuhuone = anturit.getTempC(pannuhuone);
    lampoAlaputki = anturit.getTempC(alaputki);

    anturit.requestTemperatures();
    lastTempRequest = millis();

    if (debugMode) {
      Serial.print("lampoPannuhuone: ");
      Serial.println(lampoPannuhuone);
      Serial.print("lampoAlaputki: ");
      Serial.println(lampoAlaputki);
      Serial.print("lastTempRequest: ");
      Serial.println(lastTempRequest);
    }
  }
  
  alaraja = muunnaLampotilaksi(analogRead(ALARAJA_PIN));
  ylaraja = muunnaLampotilaksi(analogRead(YLARAJA_PIN));

  if (debugMode)  {
    Serial.print("Alaraja: ");
    Serial.println(alaraja);
    Serial.print("Ylaraja: ");
    Serial.println(ylaraja);
  }

  if (ylaraja <= alaraja)
    ylaraja = alaraja + 2;

  lcdPaivita();

  pollaaWifi();
}