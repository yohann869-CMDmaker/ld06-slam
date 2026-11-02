// ============================================================================
//  ESP32 LD06 Capture : acquisition de scans à la demande par le câble USB
//
//  Basé sur ESP32_LD06_Test.ino (même câblage, même librairie LD06).
//  Le lidar est lu en continu. Quand le PC envoie le caractère 's', l'ESP32
//  renvoie le dernier tour complet au format CSV, lu directement par
//  l'application ld06-slam (bouton « Capturer un scan ») :
//
//    # LD06 scan n=452 speed=3602deg/s crc_errors=0
//    angle_deg,distance_m,intensity
//    359.62,1.234,212
//    ...
//    # end
//
//  Les angles sont ceux de la librairie : déjà convertis en sens antihoraire.
//  Rien n'est envoyé en continu : contrairement au mode Teleplot du test,
//  la liaison USB (115200 bauds, plus lente que le lidar) ne sature pas.
//
//  Commandes (depuis l'application ou le moniteur série à 115200 bauds) :
//    s : envoyer un scan
//    i : informations (vitesse de rotation, points par tour, erreurs CRC)
//    r : diagnostic, octets bruts reçus du lidar en 300 ms
// ============================================================================

#include "ld06.h"

#define RX_LIDAR 33       // TX du lidar sur GPIO33
#define TX_LIDAR -1       // l'ESP32 n'envoie rien au lidar
#define LD06_PWM_PIN 32   // entrée PWM du lidar sur GPIO32
#define LIDAR_BAUD 230400
#define USB_BAUD 115200

LD06 ld06(Serial2, LD06_PWM_PIN);
uint32_t scanCount = 0;  // tours complets reçus depuis le démarrage

void setup() {
  Serial.begin(USB_BAUD);
  Serial2.setRxBufferSize(4096);
  Serial2.begin(LIDAR_BAUD, SERIAL_8N1, RX_LIDAR, TX_LIDAR);
  ld06.init();
  Serial.println(F("# LD06 ready"));
}

// Attend deux tours complets lus sans interruption. Pendant l'envoi d'un scan
// (~0,8 s), des octets du lidar sont perdus ; le tour suivant peut donc avoir
// des trous. Le deuxième tour lu ici est complet.
bool waitFreshScan(uint32_t timeoutMs) {
  uint8_t fresh = 0;
  const uint32_t t0 = millis();
  while (fresh < 2 && millis() - t0 < timeoutMs) {
    if (ld06.readScan()) {
      fresh++;
      scanCount++;
    }
  }
  return fresh >= 2;
}

void sendScan() {
  if (!waitFreshScan(2000)) {
    Serial.println(F("# error aucun scan complet recu du lidar (cablage, alimentation ?)"));
    return;
  }
  const uint16_t n = ld06.getNbPointsInScan();
  Serial.print(F("# LD06 scan n="));
  Serial.print(n);
  Serial.print(F(" speed="));
  Serial.print(ld06.getSpeed());
  Serial.print(F("deg/s crc_errors="));
  Serial.println(ld06.getChecksumFailCount());
  Serial.println(F("angle_deg,distance_m,intensity"));
  for (uint16_t i = 0; i < n; i++) {
    const DataPoint *p = ld06.getPoints(i);
    Serial.print(p->angle, 2);
    Serial.print(',');
    Serial.print(p->distance / 1000.0f, 3);
    Serial.print(',');
    Serial.println(p->intensity);
  }
  Serial.println(F("# end"));
}

void sendInfo() {
  Serial.print(F("# info speed="));
  Serial.print(ld06.getSpeed());
  Serial.print(F("deg/s points="));
  // avant le premier tour complet, le buffer de la librairie n'est pas encore initialisé
  Serial.print(scanCount ? ld06.getNbPointsInScan() : 0);
  Serial.print(F(" crc_errors="));
  Serial.print(ld06.getChecksumFailCount());
  Serial.print(F(" scans="));
  Serial.println(scanCount);
}

// Diagnostic : lit directement les octets reçus du lidar pendant 300 ms, sans la librairie.
// Attendu : ~5300 octets et ~110 débuts de trame "54 2C". 0 octet : le lidar n'envoie rien
// (alimentation, connecteur, fil TX -> GPIO33).
void sendRaw() {
  uint32_t bytes = 0;
  uint32_t headers = 0;
  uint8_t first[16];
  uint8_t nFirst = 0;
  uint8_t prev = 0;
  const uint32_t t0 = millis();
  while (millis() - t0 < 300) {
    while (Serial2.available()) {
      const uint8_t b = Serial2.read();
      if (nFirst < sizeof(first)) first[nFirst++] = b;
      if (prev == 0x54 && b == 0x2C) headers++;
      prev = b;
      bytes++;
    }
  }
  Serial.print(F("# raw octets="));
  Serial.print(bytes);
  Serial.print(F(" en_tetes="));
  Serial.print(headers);
  Serial.print(F(" debut="));
  for (uint8_t i = 0; i < nFirst; i++) {
    if (first[i] < 0x10) Serial.print('0');
    Serial.print(first[i], HEX);
    Serial.print(' ');
  }
  Serial.println();
}

void loop() {
  if (ld06.readScan()) scanCount++;
  while (Serial.available()) {
    const char c = Serial.read();
    if (c == 's') sendScan();
    else if (c == 'i') sendInfo();
    else if (c == 'r') sendRaw();
  }
}
