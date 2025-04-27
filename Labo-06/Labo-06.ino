#include <LCD_I2C.h>
#include <AccelStepper.h>
#include <HCSR04.h>
#include <U8g2lib.h>

#pragma region Configuration

LCD_I2C lcd(0x27, 16, 2);
HCSR04 hc(6, 7);
AccelStepper stepper(AccelStepper::FULL4WIRE, 8, 10, 9, 11);

// MAX7219 Pins
#define CLK_PIN 30
#define DIN_PIN 34
#define CS_PIN 32

U8G2_MAX7219_8X8_F_4W_SW_SPI u8g2(
  U8G2_R0,
  CLK_PIN,
  DIN_PIN,
  CS_PIN,
  U8X8_PIN_NONE,
  U8X8_PIN_NONE
);

const float STEPS_PER_DEG = 2048.0 * (64.0 / 360.0);
const int MIN_DEG = 10;
const int MAX_DEG = 170;
const unsigned long MOVE_DURATION = 2000;

// Pin definitions
const int redPin = 2;
const int bluePin = 3;
const int buzzerPin = 4;

// Configuration variables
float alarmThreshold = 15.0;
float lim_inf = 30.0;
float lim_sup = 60.0;

// Display feedback
enum DisplayStatus { NORMAL, CHECKMARK, CROSS, ERROR_CIRCLE };
DisplayStatus displayStatus = NORMAL;
unsigned long displayStatusEndTime = 0;

bool isValidNumber(String str); // Prototype à ajouter

enum AppState { INIT, FERME, OUVERT, OUVERTURE, FERMETURE };
AppState appState = INIT;
unsigned long currentTime = 0;

String serialBuffer = "";

#pragma endregion

#pragma region Tâches

float mesureDistanceTask(unsigned long ct) {
  static unsigned long lastTime = 0;
  const unsigned long rate = 50;
  static float lastDistance = 0.0;

  if (ct - lastTime < rate) return lastDistance;

  lastTime = ct;
  lastDistance = hc.dist();
  return lastDistance;
}

void lcdTask(unsigned long ct, float distance, int degres) {
  static unsigned long lastTime = 0;
  const unsigned long rate = 100;

  if (ct - lastTime < rate) return;

  lastTime = ct;
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Dist : ");
  lcd.print(distance);
  lcd.print(" cm");

  lcd.setCursor(0, 1);
  if (appState == FERME) lcd.print("Port: Fermer");
  else if (appState == OUVERT) lcd.print("Port: Ouverte");
  else {
    lcd.print("Porte: ");
    lcd.print(degres);
    lcd.print(" deg");
  }
}

void alarmTask(unsigned long ct, float distance) {
  static bool alarmActive = false;
  static unsigned long lastDetectionTime = 0;
  static unsigned long lastGyroChange = 0;
  static bool gyroRed = false;

  if (distance <= alarmThreshold) {
    lastDetectionTime = ct;
  }

  bool shouldAlarmBeActive = (ct - lastDetectionTime) < 3000;

  if (shouldAlarmBeActive) {
    if (!alarmActive) {
      alarmActive = true;
      tone(buzzerPin, 1000);
      lastGyroChange = ct;
      gyroRed = false;
      digitalWrite(bluePin, HIGH);
      digitalWrite(redPin, LOW);
    }

    if (ct - lastGyroChange >= 500) {
      lastGyroChange = ct;
      gyroRed = !gyroRed;
      digitalWrite(redPin, gyroRed ? HIGH : LOW);
      digitalWrite(bluePin, !gyroRed ? HIGH : LOW);
    }
  } else {
    if (alarmActive) {
      alarmActive = false;
      noTone(buzzerPin);
      digitalWrite(redPin, LOW);
      digitalWrite(bluePin, LOW);
    }
  }
}

void max7219Task(unsigned long currentTime) {
  static unsigned long lastUpdate = 0;
  const unsigned long rate = 100;

  if (currentTime - lastUpdate < rate) return;
  lastUpdate = currentTime;

  u8g2.clearBuffer();

  if (displayStatus != NORMAL && currentTime < displayStatusEndTime) {
    switch (displayStatus) {
      case CHECKMARK: // ✔️
        u8g2.drawLine(1, 5, 3, 7);
        u8g2.drawLine(3, 7, 7, 1);
        u8g2.drawLine(7, 1, 6, 0); // Légère extension pour meilleure visibilité
        break;
      
      case CROSS: // ❌
        u8g2.drawLine(0, 0, 7, 7);
        u8g2.drawLine(0, 7, 7, 0);
        break;
      
      case ERROR_CIRCLE: // 🚫
        u8g2.drawCircle(3, 3, 3);
        u8g2.drawLine(1, 1, 5, 5);
        u8g2.drawLine(1, 5, 5, 1);
        break;
      
      default: break;
    }
  }

  u8g2.sendBuffer();
}

bool isValidNumber(String str) {
  if(str.length() == 0) return false;
  bool hasDecimal = false;
  
  for(unsigned int i = 0; i < str.length(); i++) {
    if(isdigit(str[i])) continue;
    if(str[i] == '.' && !hasDecimal) {
      hasDecimal = true;
      continue;
    }
    if(i == 0 && (str[i] == '+' || str[i] == '-')) continue;
    return false;
  }
  return true;
}

void handleCommand(String command, float distance) {
  command.trim();
  Serial.print("[DEBUG] Commande reçue : ");
  Serial.println(command);

  if (command == "g_dist") {
    Serial.println(distance);
    displayStatus = CHECKMARK;
    displayStatusEndTime = millis() + 3000;
  }
  else if (command.startsWith("cfg")) {
    // Nouvelle méthode de découpage
    int firstSemi = command.indexOf(';');
    int secondSemi = command.indexOf(';', firstSemi + 1);
    
    if (firstSemi == -1 || secondSemi == -1) {
      displayStatus = CROSS;
      displayStatusEndTime = millis() + 3000;
      return;
    }

    String cfgType = command.substring(firstSemi + 1, secondSemi);
    cfgType.trim(); // Nettoyage supplémentaire
    String valueStr = command.substring(secondSemi + 1);
    valueStr.trim();

    // Debug intermédiaire
    Serial.print("[DEBUG] Type cfg: ");
    Serial.println(cfgType);
    Serial.print("[DEBUG] Valeur: ");
    Serial.println(valueStr);

    // Validation numérique renforcée
    if (!isValidNumber(valueStr)) {
      displayStatus = CROSS;
      displayStatusEndTime = millis() + 3000;
      return;
    }

    float value = valueStr.toFloat();

    if (cfgType == "alm") {
      alarmThreshold = value;
      Serial.print("[SUCCÈS] Alarme configurée à ");
      Serial.print(value);
      Serial.println(" cm");
      displayStatus = CHECKMARK;
      displayStatusEndTime = millis() + 3000;
    }
    else if (cfgType == "lim_inf" || cfgType == "lim_sup") {
      float oldInf = lim_inf;
      float oldSup = lim_sup;

      // Mise à jour conditionnelle
      if (cfgType == "lim_inf") lim_inf = value;
      else lim_sup = value;

      // Validation des limites
      if (lim_inf >= lim_sup) {
        lim_inf = oldInf;
        lim_sup = oldSup;
        Serial.println("[ERREUR] lim_inf >= lim_sup");
        displayStatus = ERROR_CIRCLE;
        displayStatusEndTime = millis() + 3000;
      } else {
        Serial.print("[SUCCÈS] ");
        Serial.print(cfgType);
        Serial.print(" configuré à ");
        Serial.println(value);
        displayStatus = CHECKMARK;
        displayStatusEndTime = millis() + 3000;
      }
    } else {
      Serial.println("[ERREUR] Type cfg inconnu");
      displayStatus = CROSS;
      displayStatusEndTime = millis() + 3000;
    }
  }
  else {
    Serial.println("[ERREUR] Commande inconnue");
    displayStatus = CROSS;
    displayStatusEndTime = millis() + 3000;
  }
}

void processSerialCommands(float distance) {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n') {
      handleCommand(serialBuffer, distance);
      serialBuffer = "";
    } else {
      if (serialBuffer.length() < 64) {
        serialBuffer += c;
      }
    }
  }
}

#pragma endregion

#pragma region États

long degToSteps(int deg) {
  return deg * STEPS_PER_DEG;
}

void initState(unsigned long ct) {
  static bool firstTime = true;
  static unsigned long startTime = 0;

  if (firstTime) {
    lcd.begin();
    lcd.backlight();
    lcd.print("ETD:2305204");
    lcd.setCursor(0, 1);
    lcd.print("Labo 4A");

    stepper.setMaxSpeed((MAX_DEG - MIN_DEG) * STEPS_PER_DEG / (MOVE_DURATION / 1000.0));
    stepper.setAcceleration(1000);
    startTime = ct;
    firstTime = false;
    return;
  }

  if (ct - startTime >= 2000) {
    lcd.clear();
    stepper.moveTo(degToSteps(MIN_DEG));
    stepper.enableOutputs();
    while (stepper.run());
    appState = FERME;
    firstTime = true;
  }
}

void fermeState(unsigned long ct) {
  static bool firstTime = true;

  if (firstTime) {
    stepper.disableOutputs();
    firstTime = false;
    return;
  }

  float distance = mesureDistanceTask(ct);

  if (distance < lim_inf) {
    appState = OUVERTURE;
    firstTime = true;
  }
}

void ouvertState(unsigned long ct) {
  static bool firstTime = true;

  if (firstTime) {
    stepper.disableOutputs();
    firstTime = false;
    return;
  }

  float distance = mesureDistanceTask(ct);

  if (distance > lim_sup) {
    appState = FERMETURE;
    firstTime = true;
  }
}

void ouvertureState(unsigned long ct) {
  static bool firstTime = true;

  if (firstTime) {
    stepper.enableOutputs();
    stepper.moveTo(degToSteps(MAX_DEG));
    firstTime = false;
    return;
  }

  stepper.run();

  if (stepper.distanceToGo() == 0) {
    appState = OUVERT;
    firstTime = true;
  }
}

void fermetureState(unsigned long ct) {
  static bool firstTime = true;

  if (firstTime) {
    stepper.enableOutputs();
    stepper.moveTo(degToSteps(MIN_DEG));
    firstTime = false;
    return;
  }

  stepper.run();

  if (stepper.distanceToGo() == 0) {
    appState = FERME;
    firstTime = true;
  }
}

void stateManager(unsigned long ct) {
  switch (appState) {
    case INIT: initState(ct); break;
    case FERME: fermeState(ct); break;
    case OUVERT: ouvertState(ct); break;
    case OUVERTURE: ouvertureState(ct); break;
    case FERMETURE: fermetureState(ct); break;
  }
}

#pragma endregion

void setup() {
  Serial.begin(9600);

  pinMode(redPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);
  digitalWrite(redPin, LOW);
  digitalWrite(bluePin, LOW);
  digitalWrite(buzzerPin, LOW);

  u8g2.begin();
  u8g2.setContrast(100); // Augmenter la luminosité (0-255)
  u8g2.clearBuffer();
  u8g2.sendBuffer();
}

void loop() {
  currentTime = millis();

  stateManager(currentTime);

  float distance = mesureDistanceTask(currentTime);
  int degres = stepper.currentPosition() / STEPS_PER_DEG;
  lcdTask(currentTime, distance, degres);
  alarmTask(currentTime, distance);

  processSerialCommands(distance);
  max7219Task(currentTime);
}