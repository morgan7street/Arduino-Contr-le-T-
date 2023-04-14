#include <PID_v1.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_MAX31855.h>


float mesureTemp();
bool powerdown(bool posact);
void setPower(double output);
float targetTemp();
float calculPerte(float temperature);

 
#define TFT_CS 10
#define TFT_RST 9 // Ou mettez à -1 et connectez à Arduino RESET pin
#define TFT_DC 8
#define TFT_MOSI 11
#define TFT_SCLK 13

#define K4 17 // relais K4 polarisation
#define K3 18 // relais K3 polarisation
#define K2 19 // relais K2 ON/OFF

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);

// initialisation du thermocouple
int thermoDO = 2;
int thermoCS = 3;
int thermoCLK = 4;


Adafruit_MAX31855 thermocouple(thermoCLK, thermoCS, thermoDO);
double Setpoint, Input, Output;

// initialisation PID
double Kp = 2, Ki = 5, Kd = 1; // coefficients du régulateur PID
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// initialisation des relais
bool posact = true;
bool relayState = false;

float tref[7] = {40, 60, 80, 100, 120, 140, 160};


void setup() {
Serial.begin(9600);

// initialisation de l'affichage
tft.initR(INITR_BLACKTAB);
tft.setRotation(-45);
Serial.println(F("Hello! ST77xx TFT Test"));
Serial.println(F("Initialized"));
delay(1000);

// initialisation des relais
pinMode(K2, OUTPUT);
pinMode(K3, OUTPUT);
pinMode(K4, OUTPUT);
digitalWrite(K2, HIGH);
digitalWrite(K3, HIGH);
digitalWrite(K4, HIGH);

// initialisation du PID
Setpoint = 90;
Input = mesureTemp();
myPID.SetMode(AUTOMATIC);
myPID.SetOutputLimits(0, 255);

// initialisation de l'actionneur
posact = powerdown(posact);
}

void loop() {
Setpoint = getSetpoint();
Input = mesureTemp();

myPID.Compute();

if (Input > Setpoint) {
if (relayState) {
powerdown(posact);
relayState = false;
}
} else {
if (!relayState) {
powerdown(posact);
relayState = true;
}
setPower(Output);
}

Serial.print("Temp mesurée: ");
Serial.print(Input);
Serial.print(", Setpoint: ");
Serial.println(Setpoint);

delay(1000);
}
int zone = 0;

float getSetpoint() {
  float setpoint = targetTemp();
  float temperature = Input;
  int zone = getZone(temperature);

  if (zone >= 0 && zone <= 6) {
    setpoint = tref[zone] + calculPerte(temperature) + 0.1;
  }

  // Gestion du régulateur de ralenti
  if (temperature <= 65) {
    setpoint = 65;
  } else if (temperature <= 68) {
    setpoint = 68;
  } else if (temperature >= 90) {
    setpoint = 90;
  }

  return setpoint;
}

int getZone(float temp) {
  for (int i = 0; i < 7; i++) {
    if (temp <= tref[i]) {
      return i;
    }
  }
  return 6; // Si la température est supérieure à toutes les valeurs de référence, retournez la dernière zone
}
float mesureTemp() {
  double temperature = thermocouple.readCelsius();
  if (isnan(temperature)) {
    Serial.println("Failed to read temperature from thermocouple!");
    return 0;
  }
  return temperature;
}

bool powerdown(bool posact) {
  if (posact) {
    digitalWrite(K3, LOW);
    digitalWrite(K4, HIGH);
  } else {
    digitalWrite(K3, HIGH);
    digitalWrite(K4, LOW);
  }
  return !posact;
}

void setPower(double output) {
  const int RELAY_PIN = 12; // Changer pour le numéro de broche approprié pour votre relais
  pinMode(RELAY_PIN, OUTPUT);

  if (output <= 105) {
    digitalWrite(RELAY_PIN, LOW); // Éteindre le poêle
  } else {
    float tempDiff = output - Input;

    if (tempDiff >= 4) {
      digitalWrite(RELAY_PIN, HIGH); // Allumer le poêle
    } else if (tempDiff >= 3) {
      digitalWrite(RELAY_PIN, HIGH); // Allumer le poêle
    } else if (tempDiff >= 2) {
      digitalWrite(RELAY_PIN, HIGH); // Allumer le poêle
    } else if (tempDiff >= 1) {
      digitalWrite(RELAY_PIN, HIGH); // Allumer le poêle
    } else if (Input >= 65 && Input <= 68) {
      digitalWrite(RELAY_PIN, HIGH); // Régulateur de ralenti, allumer le poêle
    } else {
      digitalWrite(RELAY_PIN, LOW); // Éteindre le poêle
    }
  }
}


float calculPerte(float temperature) {
  float perte = 0;

  if (temperature > 200 && temperature <= 105) {
    perte = 0; // Température déjà élevée, pas de perte
  } else if (temperature > 100 && temperature <= 105) {
    perte = -4.0;
  } else if (temperature > 90 && temperature <= 100) {
    perte = -3.0;
  } else if (temperature > 80 && temperature <= 90) {
    perte = -2.0;
  } else if (temperature > 70 && temperature <= 80) {
    perte = -1.0;
  } else if (temperature > 60 && temperature <= 70) {
    perte = -0.5;
  } else {
    perte = 0; // Pour les températures en dehors des plages spécifiées, considérez qu'il n'y a pas de perte
  }

  return perte;
}



float targetTemp() {
  float temperature = Input;
  if (temperature > 150) {
    return 150;
  } else if (temperature <= 105) {
    return 105;
  } else if (temperature > 105 && temperature < 200) {
    return temperature + 0.1;
  } else {
    return 200;
  }
}
