// ============================================================================
//  ESP32 Robot USB : lidar LD06 + moteurs du robot, par le câble USB
//
//  Même protocole de scan que ESP32_LD06_Capture (lu par scripts/live.py et
//  scripts/capture.py), plus la commande des moteurs. Liaison à 921600 bauds :
//  un scan (~9 ko de texte) passe en 0,1 s au lieu de 0,8 s à 115200 bauds,
//  assez vite pour suivre le robot qui roule (4 à 5 scans par seconde).
//
//    # LD06 scan n=452 speed=3602deg/s crc_errors=0
//    angle_deg,distance_m,intensity
//    359.62,1.234,212
//    ...
//    # end
//
//  Commandes (application via live.py, ou moniteur série à 921600 bauds) :
//    s : envoyer un scan        i : informations      r : octets bruts du lidar
//    F : avancer   B : reculer   G : glisser à gauche   D : glisser à droite
//    Q : tourner à gauche        E : tourner à droite   X : arrêt
//    V120 + Entrée : vitesse des moteurs (0-255)
//  Un ordre de mouvement dure 400 ms : pour rouler en continu, le renvoyer
//  (l'application le fait toutes les 150 ms tant que la touche est maintenue).
//  Sans nouvel ordre, le robot s'arrête seul (câble arraché, appli fermée).
//
//  Moteurs commandés directement par l'ESP32 (pas de carte I2C), deux broches
//  par roue, une par sens, en PWM : câblage dans WHEELS ci-dessous.
// ============================================================================

#include "ld06.h"

#define RX_LIDAR 33       // TX du lidar sur GPIO33
#define TX_LIDAR -1       // l'ESP32 n'envoie rien au lidar
#define LD06_PWM_PIN 32   // entrée PWM du lidar sur GPIO32
#define LIDAR_BAUD 230400
#define USB_BAUD 921600

// Roues : broche qui fait AVANCER la roue, broche qui la fait RECULER, inversion (true si la roue
// tourne à l'envers), nom. Câblage relevé sur le robot.
struct Wheel {
  uint8_t avance;
  uint8_t recule;
  bool inverse;
  const char* nom;
};
Wheel WHEELS[4] = {
  {18, 19, false, "avant gauche"},    // M2 : m2a (D18) avance, m2b (D19) recule
  { 5, 17, false, "avant droite"},    // M1 : m1b (D5)  avance, m1a (D17) recule
  {15,  2, false, "arriere gauche"},  // M3 : m3a (D15) avance, m3b (D2)  recule
  {16,  4, false, "arriere droite"},  // M4 : m4b (D16) avance, m4a (D4)  recule
};

uint8_t vitesse = 140;            // rapport cyclique PWM 0-255 (commande V)
const uint32_t PWM_FREQ = 1000;   // Hz
const uint32_t HOLD_MS = 400;     // durée d'un ordre de mouvement

LD06 ld06(Serial2, LD06_PWM_PIN);
uint32_t scanCount = 0;  // tours complets reçus depuis le démarrage
uint32_t moveUntil = 0;  // fin du mouvement en cours (millis), 0 = arrêté
String speedCmd;         // chiffres de la commande V en cours de saisie
bool readingSpeed = false;

// ------------------------------------------------------------------ moteurs

// sens : +1 avance, -1 recule, 0 arrêt ; une seule des deux broches en PWM, l'autre à 0
void wheel(const Wheel& w, int sens) {
  if (w.inverse) sens = -sens;
  analogWrite(w.avance, sens > 0 ? vitesse : 0);
  analogWrite(w.recule, sens < 0 ? vitesse : 0);
}

// (avant gauche, avant droite, arrière gauche, arrière droite)
void wheels(int avg, int avd, int arg, int ard) {
  wheel(WHEELS[0], avg);
  wheel(WHEELS[1], avd);
  wheel(WHEELS[2], arg);
  wheel(WHEELS[3], ard);
}

void stopMotors() {
  wheels(0, 0, 0, 0);
  moveUntil = 0;
}

// roues Mecanum : translation latérale = roues en diagonale en sens opposés
void drive(char c) {
  switch (c) {
    case 'F': wheels(1, 1, 1, 1); break;
    case 'B': wheels(-1, -1, -1, -1); break;
    case 'G': wheels(-1, 1, 1, -1); break;
    case 'D': wheels(1, -1, -1, 1); break;
    case 'Q': wheels(-1, 1, -1, 1); break;
    case 'E': wheels(1, -1, 1, -1); break;
    default: return;
  }
  moveUntil = millis() + HOLD_MS;
}

// arrêt automatique quand l'ordre n'a pas été renouvelé
void motorTick() {
  if (moveUntil && (int32_t)(millis() - moveUntil) >= 0) stopMotors();
}

// ------------------------------------------------------------------ lidar

// Attend un tour complet lu sans interruption (à 921600 bauds, l'envoi d'un scan est assez court
// pour ne pas faire perdre d'octets au lidar : un seul tour suffit).
bool waitFreshScan(uint32_t timeoutMs) {
  const uint32_t t0 = millis();
  while (millis() - t0 < timeoutMs) {
    motorTick();
    handleCommands(false);
    if (ld06.readScan()) {
      scanCount++;
      return true;
    }
  }
  return false;
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
    const DataPoint* p = ld06.getPoints(i);
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
  Serial.print(scanCount ? ld06.getNbPointsInScan() : 0);
  Serial.print(F(" crc_errors="));
  Serial.print(ld06.getChecksumFailCount());
  Serial.print(F(" scans="));
  Serial.print(scanCount);
  Serial.print(F(" vitesse_moteurs="));
  Serial.println(vitesse);
}

// Diagnostic : octets reçus du lidar en 300 ms, sans la librairie (attendu ~5300 octets).
void sendRaw() {
  uint32_t bytes = 0;
  uint32_t headers = 0;
  uint8_t prev = 0;
  const uint32_t t0 = millis();
  while (millis() - t0 < 300) {
    while (Serial2.available()) {
      const uint8_t b = Serial2.read();
      if (prev == 0x54 && b == 0x2C) headers++;
      prev = b;
      bytes++;
    }
  }
  Serial.print(F("# raw octets="));
  Serial.print(bytes);
  Serial.print(F(" en_tetes="));
  Serial.println(headers);
}

// ------------------------------------------------------------------ commandes

// Lit les caractères reçus. allowScan = false pendant l'attente d'un scan (pas de scan imbriqué) :
// seuls les ordres moteurs sont traités, les autres sont gardés pour la boucle principale.
void handleCommands(bool allowScan) {
  while (Serial.available()) {
    const char c = Serial.peek();
    if (!allowScan && (c == 's' || c == 'i' || c == 'r')) return;
    Serial.read();
    if (readingSpeed) {
      if (c >= '0' && c <= '9') {
        speedCmd += c;
        continue;
      }
      vitesse = (uint8_t)constrain(speedCmd.toInt(), 0, 255);
      readingSpeed = false;
      speedCmd = "";
      if (c == '\n' || c == '\r') continue;
    }
    if (c == 's') sendScan();
    else if (c == 'i') sendInfo();
    else if (c == 'r') sendRaw();
    else if (c == 'X' || c == 'x') stopMotors();
    else if (c == 'V' || c == 'v') readingSpeed = true;
    else drive(c);
  }
}

// ------------------------------------------------------------------ démarrage

void setup() {
  // roues arrêtées avant tout le reste : les 8 broches moteur en sortie, PWM à 0
  for (auto& w : WHEELS) {
    analogWriteFrequency(w.avance, PWM_FREQ);
    analogWriteFrequency(w.recule, PWM_FREQ);
    analogWrite(w.avance, 0);
    analogWrite(w.recule, 0);
  }
  Serial.setRxBufferSize(1024);
  Serial.begin(USB_BAUD);
  // Serial2 avec sa broche RX d'abord, puis ld06.init() (qui garde ces broches ; seul, il prendrait
  // RX = GPIO4, une broche moteur sur ce robot)
  Serial2.setRxBufferSize(4096);
  Serial2.begin(LIDAR_BAUD, SERIAL_8N1, RX_LIDAR, TX_LIDAR);
  ld06.init();
  Serial.println(F("# LD06 ready"));
}

void loop() {
  if (ld06.readScan()) scanCount++;
  motorTick();
  handleCommands(true);
}
