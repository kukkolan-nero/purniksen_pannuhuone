#include <Arduino.h>
#include <OneWire.h>
#include <LiquidCrystal_I2C.h>
#include <DallasTemperature.h>
#include <string.h>
//#include <SoftwareSerial.h>

#define LCD_SDA_PIN 20 // LCD-näytöt I2C-väylään
#define LCD_SCL_PIN 21
#define LCD_ROWS 4 // LCD-näytössä rivien määrä ...
#define LCD_COLS 20 // ... ja riville mahtuvien merkkien määrä

#define LAMPOANTURIT_PIN 49 // Dallas DS18xx(x)
#define DEBUG_BTN_PIN 47 // DEBUG-painikkeen pinni
#define DEBUG_LED_PIN 46 // DEBUG-ledin pinni
#define ALARAJA_PIN A14
#define YLARAJA_PIN A15
//#define WIFI_RX_PIN 7
//#define WIFI_TX_PIN 6
#define WIFI_DH_CP 51
#define WIFI_OK_LED_PIN 48 // Wifi-yhteys kunnossa LED
#define SARJAPUSKURIN_KOKO 64

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

boolean debugTila = false;
boolean vastusPaalla = false; // Lämpövastuksen tila
float lampoPannuhuone = -99.9; // Pannuhuoneen ilman lämpötila
float lampoAlaputki = -99.9; // Varaajan alaputken lämpötila
int ylaraja = 999; // Alaputken lämpötilan yläraja, jolloin vastus sammutetaan
int alaraja = 999; // Alaputken lämpötilan alaraja, jolloin vastus laitetaan päälle
char lcdStr[LCD_COLS]; // Yksi LCD-näytölle tulostettava rivi
char lampoStr[7];
char* merkkijono; // Sarjaporteista saatavat vastaukset / data luetaan tähän merkkijonoon.
boolean wifiOK = false; // Vastaako wifi-moduuli AT-komentoon?

LiquidCrystal_I2C lcd(0x27, LCD_COLS, LCD_ROWS);

OneWire anturivayla(LAMPOANTURIT_PIN);
DallasTemperature anturit(&anturivayla);
DeviceAddress pannuhuone, alaputki;

//oftwareSerial wifi(WIFI_RX_PIN, WIFI_TX_PIN);

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

void lueSarjaportti(Stream &portti) {
  if (debugTila)
    Serial.println(">>> lueSarjaportti");

  // Oletusarvoisesti kaikkiin portteihin liikennöidään AT-komennoin, ja vastaus loppuu merkkiin '\r'.

  int pos = 0;
  char merkki;
  char *pMerkkijono = merkkijono;

  while ( (pMerkkijono - merkkijono) >= (SARJAPUSKURIN_KOKO-1) ) {
    if (portti.available()) {
      merkki = portti.read();
  
      if (debugTila)
        Serial.print(merkki);

      *pMerkkijono++ = merkki;

      if (merkki == '\r')
        break;
    } // portti.available()
  } // while
  *pMerkkijono--;
  *pMerkkijono = '\0';
}

void kirjoitaSarjaporttiin(Stream &portti, char *tmpStr) {
  if (debugTila)
    Serial.println(">>> kirjoitaSarjaporttiin");

  int pituus = strlen(tmpStr);
  int pos = 0;

  while ( (pos < pituus) && (pos < SARJAPUSKURIN_KOKO - 1) ) {
    portti.write(tmpStr[pos]);
    if (debugTila)
      Serial.write(tmpStr[pos]);
    ++pos;
  }
  portti.print("\r\n");
  if (debugTila)
    Serial.println(" ");

  delay(500);
}

boolean onksWifii(Stream portti) {
  if (debugTila)
    Serial.println(">>> onksWifii");

  kirjoitaSarjaporttiin(portti, "ATE0");
  delay(1000);
  lueSarjaportti(portti);

  if (strstr(merkkijono, "OK") != NULL) {
    digitalWrite(WIFI_OK_LED_PIN, HIGH);
    return true;
  }
  else {
    digitalWrite(WIFI_OK_LED_PIN, LOW);
    return false;
  }
}


void setup() {
  byte lcdRivi = 0;

  Serial.begin(9600);

  merkkijono = (char*)calloc(SARJAPUSKURIN_KOKO, sizeof(char));
  if (debugTila)
    Serial.println("Varataan muistia: merkkijono");

  if (merkkijono == NULL) {
    lcd.print("!CALLOC: merkkijono");
    lcd.setCursor(0, lcdRivi++);
    if (debugTila)
      Serial.println("Muistin varaaminen ei onnistunut: merkkijono");
  }
  
  pinMode(DEBUG_LED_PIN, OUTPUT);
  digitalWrite(DEBUG_LED_PIN, LOW);

  if (digitalRead(DEBUG_BTN_PIN)) {
    debugTila = true;
    digitalWrite(DEBUG_LED_PIN, HIGH);
  }

  if (debugTila)
    Serial.println("pinMode:t");
  pinMode(ALARAJA_PIN, INPUT);
  pinMode(YLARAJA_PIN, INPUT);
  pinMode(DEBUG_BTN_PIN, INPUT);
  pinMode(LAMPOANTURIT_PIN, INPUT);
  pinMode(WIFI_DH_CP, OUTPUT);
  pinMode(WIFI_OK_LED_PIN, OUTPUT);
  digitalWrite(WIFI_DH_CP, LOW);
  
  lcd.init();
  lcd.backlight();
  lcd.createChar(0, aste);
  lcd.clear();
  if (debugTila) {
    lcd.print("Debug-tila paalla");
    lcd.setCursor(0, ++lcdRivi);
    Serial.println("PURNIKSEN PANNUHUONE");
    Serial.println("Debug-tila paalla");
  }
  lcd.print("PURNIKSEN PANNUHUONE");
  lcd.setCursor(0, ++lcdRivi);
  anturit.begin();

  if (!anturit.getAddress(pannuhuone, 0)) {
    if (debugTila) Serial.println("Anturi 0: Pannuhuone: Ei osoitetta");
    lcd.print("Anturi 0: No addr");
    lcd.setCursor(0, ++lcdRivi);
  } else {
    lcd.print("Anturi 0: ok");
    lcd.setCursor(0, ++lcdRivi);
    if (debugTila) {
      Serial.print("Anturi 0: Pannuhuone: osoite: ");
      printAddress(pannuhuone);
      Serial.println();
    }
  }

  if (!anturit.getAddress(alaputki, 1)) {
    if (debugTila) Serial.println("Anturi 1: Alaputki: Ei osoitetta");
    lcd.print("Anturi 1: No addr");
    lcd.setCursor(0, ++lcdRivi);
  } else {
    lcd.print("Anturi 1: ok");
    lcd.setCursor(0, ++lcdRivi);
    if (debugTila) {
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
  if (debugTila) {
    Serial.print("lastTempRequest: ");
    Serial.println(lastTempRequest);
  }
  
  if (debugTila) Serial.println("WIFI: CH_PD ylös");
  digitalWrite(WIFI_DH_CP, HIGH);

  // Wifi-moduulin (ESP8266) alustus
  Serial1.begin(115200);
  if (debugTila) Serial.println("WIFI: Odotetaan sarjaporttia.");
  while (!Serial1) {}
  if (debugTila) Serial.println("WIFI: Sarjaportti ok.");

  delay(1000);
  wifiOK = onksWifii(Serial1);
  if (debugTila) {
    Serial.print("wifiOK: ");
    Serial.println(wifiOK);
  }

  delay(2000);
  lcd.clear();
}

void lcdAste() {
  lcd.write(aste);
  lcd.print("C");
}

void lcdPaivita() {
  if (debugTila)
    Serial.println(">>> lcdPaivita");

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

void pollaaWifi(Stream &portti) {
  if (debugTila)
    Serial.println(">>> pollaaWIFI");

  while (portti.available()) {
    char a = portti.read();
    Serial.print(a);
  }
  while (Serial.available()) {
    char a = Serial.read();
    Serial.print(a);
    portti.print(a);
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

    if (debugTila) {
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

  if (debugTila)  {
    Serial.print("Alaraja: ");
    Serial.println(alaraja);
    Serial.print("Ylaraja: ");
    Serial.println(ylaraja);
  }

  if (ylaraja <= alaraja)
    ylaraja = alaraja + 2;

  lcdPaivita();

  pollaaWifi(Serial1);
}