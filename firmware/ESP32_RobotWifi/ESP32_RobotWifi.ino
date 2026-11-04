// Robot Mecanum + LD06 en Wi-Fi, pour l'application ld06-slam (pilotage manuel, SLAM, exploration).
// Adapté du programme ESP32RobotControl du module : même point d'accès, mêmes routes HTTP, même
// format de scan binaire (l'application et RobotControl marchent sans changement).
//
// Différence avec le robot du module : ici les moteurs ne passent pas par la carte I2C Keyestudio,
// ils sont commandés DIRECTEMENT par l'ESP32, deux broches par roue (une par sens, en PWM). Le
// programme du module utilisait GPIO19/21 pour l'I2C : sur ce robot, ce sont des broches moteur,
// d'où les roues qui tournaient toutes seules.
//
// Routes : /forward /backward /left /right /rotateleft /rotateright ?time=ms (rotations : durée
// divisée par 2, comme le programme du module), /stop, /scan (binaire), /pose?x=&y=&z=, /ping,
// et en plus /speed?v=0..255 (vitesse des moteurs). Pilotage continu : l'application renvoie la
// commande toutes les 150 ms tant que la touche est maintenue ; chaque commande repousse l'arrêt
// automatique, qui coupe les moteurs si plus rien n'arrive (Wi-Fi perdu, page fermée).

#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "ld06.h"

// ============================================================
// CONFIGURATION DU ROBOT
// ============================================================

// Roues : broche qui fait AVANCER la roue, broche qui la fait RECULER, inversion du sens (true si
// la roue tourne à l'envers), nom. Câblage relevé sur le robot.
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
enum { AVG = 0, AVD = 1, ARG = 2, ARD = 3 };  // indices dans WHEELS

// vitesse des moteurs au démarrage : rapport cyclique PWM de 0 à 255 (réglable ensuite par /speed)
uint8_t vitesse = 180;
const uint32_t PWM_FREQ = 1000;  // Hz

// lidar LD06 : TX du lidar sur LIDAR_RX, entrée PWM du lidar sur LIDAR_PWM
const int LIDAR_RX = 33;
const int LIDAR_PWM = 32;

// point d'accès Wi-Fi (le PC s'y connecte ; adresse du robot : 192.168.4.1)
const char* ssid = "DON MIGUEL3";
const char* password = "12345678";

// ============================================================

AsyncWebServer server(80);
LD06 lidar(Serial2, LIDAR_PWM);

// pose transmise par /pose (renvoyée dans l'en-tête des scans, comme le programme du module)
int16_t posex = 0;
int16_t posey = 0;
uint16_t posez = 0;

int getArg(AsyncWebServerRequest* req, const char* name, int def) {
  return req->hasParam(name) ? req->getParam(name)->value().toInt() : def;
}

void addCORS(AsyncWebServerResponse* resp) {
  resp->addHeader("Access-Control-Allow-Origin", "*");
  resp->addHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
  resp->addHeader("Access-Control-Allow-Headers", "*");
}

// ============================================================
// MOTEURS
// ============================================================

bool moving = false;
uint32_t moveStarted = 0;
uint32_t moveDuration = 0;
SemaphoreHandle_t motorMutex;
const int MAX_MOVE_MS = 2000;

// sens : +1 avance, -1 recule, 0 arrêt ; une seule des deux broches en PWM, l'autre à 0
void wheel(const Wheel& w, int sens) {
  if (w.inverse) sens = -sens;
  analogWrite(w.avance, sens > 0 ? vitesse : 0);
  analogWrite(w.recule, sens < 0 ? vitesse : 0);
}

void wheels(int avg, int avd, int arg, int ard) {
  wheel(WHEELS[AVG], avg);
  wheel(WHEELS[AVD], avd);
  wheel(WHEELS[ARG], arg);
  wheel(WHEELS[ARD], ard);
}

void stopRobot() {
  xSemaphoreTake(motorMutex, portMAX_DELAY);
  wheels(0, 0, 0, 0);
  moving = false;
  xSemaphoreGive(motorMutex);
}

void doMove(int avg, int avd, int arg, int ard, int t) {
  xSemaphoreTake(motorMutex, portMAX_DELAY);
  if (t <= 0) {
    wheels(0, 0, 0, 0);
    moving = false;
  } else {
    moveDuration = (uint32_t)constrain(t, 1, MAX_MOVE_MS);
    moveStarted = millis();
    wheels(avg, avd, arg, ard);
    moving = true;
  }
  xSemaphoreGive(motorMutex);
}

// roues Mecanum (avant gauche, avant droite, arrière gauche, arrière droite) : translation
// latérale = roues en diagonale en sens opposés
void forward(int t) { doMove(1, 1, 1, 1, t); }
void backward(int t) { doMove(-1, -1, -1, -1, t); }
void left(int t) { doMove(-1, 1, 1, -1, t); }
void right(int t) { doMove(1, -1, -1, 1, t); }
void rotateleft(int t) { doMove(-1, 1, -1, 1, t / 2); }
void rotateright(int t) { doMove(1, -1, 1, -1, t / 2); }

// ============================================================
// SCAN (format binaire du module : pose x, y en mm, θ en 0,01°, n, puis n × (angle, distance, intensité))
// ============================================================

const uint16_t MAX_SCAN_POINTS = 460;
const size_t MAX_SCAN_BYTES = 8 + 5 * MAX_SCAN_POINTS;
uint8_t publishedScan[MAX_SCAN_BYTES];
size_t publishedSize = 0;
portMUX_TYPE scanMux = portMUX_INITIALIZER_UNLOCKED;

void publishScan() {
  const uint16_t n = lidar.getNbPointsInScan();
  if (n == 0 || n > MAX_SCAN_POINTS) return;
  uint8_t snapshot[MAX_SCAN_BYTES];
  snapshot[6] = n & 0xff;
  snapshot[7] = n >> 8;
  size_t off = 8;
  for (uint16_t i = 0; i < n; ++i) {
    const auto* pt = lidar.getPoints(i);
    const uint16_t angle = (uint16_t)(pt->angle * 100);
    snapshot[off++] = angle & 0xff;
    snapshot[off++] = angle >> 8;
    snapshot[off++] = pt->distance & 0xff;
    snapshot[off++] = pt->distance >> 8;
    snapshot[off++] = pt->intensity;
  }
  portENTER_CRITICAL(&scanMux);
  memcpy(publishedScan + 6, snapshot + 6, off - 6);
  publishedSize = off;
  portEXIT_CRITICAL(&scanMux);
}

void sendScanBinary(AsyncWebServerRequest* req) {
  uint8_t snapshot[MAX_SCAN_BYTES];
  size_t size;
  portENTER_CRITICAL(&scanMux);
  size = publishedSize;
  memcpy(snapshot, publishedScan, size);
  snapshot[0] = posex & 0xff;
  snapshot[1] = (uint16_t)posex >> 8;
  snapshot[2] = posey & 0xff;
  snapshot[3] = (uint16_t)posey >> 8;
  snapshot[4] = posez & 0xff;
  snapshot[5] = posez >> 8;
  portEXIT_CRITICAL(&scanMux);
  if (!size) {
    auto response = req->beginResponse(503, "text/plain", "NO_VALID_SCAN");
    addCORS(response);
    req->send(response);
    return;
  }
  auto response = req->beginResponseStream("application/octet-stream");
  response->write(snapshot, size);
  addCORS(response);
  req->send(response);
}

void ok(AsyncWebServerRequest* r) {
  auto x = r->beginResponse(200, "text/plain", "OK");
  addCORS(x);
  r->send(x);
}

// ============================================================
// SETUP / LOOP
// ============================================================

void setup() {
  // roues arrêtées avant tout le reste : les 8 broches moteur en sortie, PWM à 0
  for (auto& w : WHEELS) {
    analogWriteFrequency(w.avance, PWM_FREQ);
    analogWriteFrequency(w.recule, PWM_FREQ);
    analogWrite(w.avance, 0);
    analogWrite(w.recule, 0);
  }
  Serial.begin(115200);
  motorMutex = xSemaphoreCreateMutex();
  if (!motorMutex) {
    while (true) delay(1000);
  }
  stopRobot();

  // Ordre important : Serial2 d'abord, avec sa broche RX, puis lidar.init(). lidar.init() rappelle
  // Serial2.begin() sans broches, qui garde alors RX = LIDAR_RX ; appelé seul, il prendrait les
  // broches par défaut de l'UART2 (RX = GPIO4 : une broche moteur sur ce robot). La broche TX de
  // l'UART, inutilisée, part sur GPIO25 (libre ici).
  Serial2.setRxBufferSize(1024);
  Serial2.begin(230400, SERIAL_8N1, LIDAR_RX, -1);
  Serial2.setTimeout(2);
  lidar.init();  // met LIDAR_PWM à l'état haut

  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);
  delay(200);
  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());
  for (auto& w : WHEELS) Serial.printf("Roue %s : avance D%d, recule D%d%s\n", w.nom, w.avance, w.recule, w.inverse ? " (inversée)" : "");

  server.on("/stop", HTTP_GET, [](AsyncWebServerRequest* r) {
    stopRobot();
    auto response = r->beginResponse(200, "text/plain", "STOPPED");
    addCORS(response);
    r->send(response);
  });
  server.on("/forward", HTTP_GET, [](AsyncWebServerRequest* r) { forward(getArg(r, "time", 300)); ok(r); });
  server.on("/backward", HTTP_GET, [](AsyncWebServerRequest* r) { backward(getArg(r, "time", 300)); ok(r); });
  server.on("/left", HTTP_GET, [](AsyncWebServerRequest* r) { left(getArg(r, "time", 300)); ok(r); });
  server.on("/right", HTTP_GET, [](AsyncWebServerRequest* r) { right(getArg(r, "time", 300)); ok(r); });
  server.on("/rotateleft", HTTP_GET, [](AsyncWebServerRequest* r) { rotateleft(getArg(r, "time", 300)); ok(r); });
  server.on("/rotateright", HTTP_GET, [](AsyncWebServerRequest* r) { rotateright(getArg(r, "time", 300)); ok(r); });
  server.on("/scan", HTTP_GET, [](AsyncWebServerRequest* r) { sendScanBinary(r); });
  server.on("/pose", HTTP_GET, [](AsyncWebServerRequest* r) {
    const int xPose = constrain(getArg(r, "x", 0), -32768, 32767);
    const int yPose = constrain(getArg(r, "y", 0), -32768, 32767);
    const int angle = getArg(r, "z", 0);
    portENTER_CRITICAL(&scanMux);
    posex = xPose;
    posey = yPose;
    posez = ((angle % 36000) + 36000) % 36000;
    portEXIT_CRITICAL(&scanMux);
    ok(r);
  });
  server.on("/ping", HTTP_GET, [](AsyncWebServerRequest* r) { ok(r); });
  server.on("/speed", HTTP_GET, [](AsyncWebServerRequest* r) {
    vitesse = (uint8_t)constrain(getArg(r, "v", vitesse), 0, 255);
    ok(r);
  });
  server.begin();
}

void loop() {
  if (lidar.readScan()) publishScan();

  static uint32_t lastYield = 0;
  if (millis() - lastYield > 20) {
    lastYield = millis();
    yield();
  }

  // arrêt automatique à la fin de la durée demandée
  xSemaphoreTake(motorMutex, portMAX_DELAY);
  if (moving && (uint32_t)(millis() - moveStarted) >= moveDuration) {
    wheels(0, 0, 0, 0);
    moving = false;
  }
  xSemaphoreGive(motorMutex);
}
