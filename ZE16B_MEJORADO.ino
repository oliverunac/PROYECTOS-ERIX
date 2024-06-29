#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>

SoftwareSerial mySerial(10, 11); // RX, TX
static const int REQUEST_CNT = 9;
static const int RESPONSE_CNT = 9;
uint8_t getppm[REQUEST_CNT] = {0xff, 0x04, 0x03, 0x00, 0x00, 0x00, 0x01, 0xF4, 0x00};
#define WAIT_READ_TIMES 100
#define WAIT_READ_DELAY 10

LiquidCrystal_I2C lcd(0x27, 20, 4);
int measurement;
int measurement1;

unsigned long previousSensorMillis = 0;
const long sensorInterval = 2000; // intervalo para leer el sensor y actualizar la pantalla

unsigned long previousLedMillis = 0;
const long ledInterval = 500; // intervalo para actualizar los LEDs y el buzzer

unsigned long previousDelayMillis = 0; // Control del retraso no bloqueante

const int buzzerPin = 2;
const int greenLedPin = 3;
const int yellowLedPin = 4;
const int redLedPin = 5;

const int buzzerThreshold = 50; // Establece tu umbral deseado aquí

void setup() {
  lcd.init(); 
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("                ");
  lcd.setCursor(0, 1);
  lcd.print("   ZE16B-CO     ");

  Serial.begin(9600); // MOSTRAR POR CONSOLA
  mySerial.begin(9600); // CONECTAR NUESTRO SENSOR

  pinMode(buzzerPin, OUTPUT);
  pinMode(greenLedPin, OUTPUT);
  pinMode(yellowLedPin, OUTPUT);
  pinMode(redLedPin, OUTPUT);

  preheatSensor();
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousSensorMillis >= sensorInterval) {
    previousSensorMillis = currentMillis;
    readSensor();
  }

  if (currentMillis - previousLedMillis >= ledInterval) {
    previousLedMillis = currentMillis;
    updateLedsAndBuzzer();
  }
}

void preheatSensor() {
  mySerial.write(getppm, REQUEST_CNT); // ENVIA POR SERIAL COMANDO
  mySerial.flush(); // LIMPIA EL BUFFER SERIAL
  Serial.println("PRECALENTAMIENTO....");
  for (int i = 0; i < 30; i++) {
    Serial.println(i);
    delay(1000); // Precalentamiento durante 30 segundos
  }
  Serial.println("SENSOR READY");
}

void readSensor() {
  unsigned long currentMillis = millis();
  static byte buf[RESPONSE_CNT - 1]; // BUF LLENA CON LOS DATOS DEL SENSOR
  byte cheksum;

  // Llenar de ceros el buffer
  for (int i = 0; i < RESPONSE_CNT; i++) {
    buf[i] = 0x0;
  }

  writeCommand(getppm, buf); // Enviamos el comando y el buffer se llena de nuevo

  // Esperar 500 ms sin bloquear
  if (currentMillis - previousDelayMillis >= 500) {
    previousDelayMillis = currentMillis;
    writeCommand(getppm, buf); // Enviamos el comando y el buffer se llena de nuevo

    // Parsear los datos recibidos
    cheksum = (buf[1] + buf[2] + buf[3] + buf[4] + buf[5] + buf[6] + buf[7]);
    cheksum = (~cheksum) + 1;

    if (buf[0] == 0xff && buf[1] == 0x04 && buf[8] == cheksum) {
      measurement = (buf[4] * 256 + buf[5]) * 1; // 0 500
      measurement1 = (0.6873 * measurement) + 21.137;
    } else {
      measurement1 = -1;
    }

    Serial.println(measurement1);
    lcd.setCursor(0, 0);
    lcd.print("PPM: ");
    lcd.print(measurement1);
  }
}

void updateLedsAndBuzzer() {
  if (measurement1 > buzzerThreshold) {
    digitalWrite(buzzerPin, HIGH);
  } else {
    digitalWrite(buzzerPin, LOW);
  }

  if (measurement1 >= 0 && measurement1 < 35) {
    digitalWrite(greenLedPin, HIGH);
    digitalWrite(yellowLedPin, LOW);
    digitalWrite(redLedPin, LOW);
  } else if (measurement1 >= 35 && measurement1 < 100) {
    digitalWrite(greenLedPin, LOW);
    digitalWrite(yellowLedPin, HIGH);
    digitalWrite(redLedPin, LOW);
  } else if (measurement1 >= 100) {
    digitalWrite(greenLedPin, LOW);
    digitalWrite(yellowLedPin, LOW);
    digitalWrite(redLedPin, HIGH);
  } else {
    digitalWrite(greenLedPin, LOW);
    digitalWrite(yellowLedPin, LOW);
    digitalWrite(redLedPin, LOW);
  }
}

void writeCommand(uint8_t cmd[], uint8_t *response) {
  mySerial.write(cmd, REQUEST_CNT); // ENVIA POR SERIAL
  mySerial.flush(); // LIMPIA EL BUFFER SERIAL

  if (response != NULL) {
    int i = 0;
    while (mySerial.available() <= 0) {
      if (++i > WAIT_READ_TIMES) {
        Serial.println("can't get ZE16B-CO response.");
        return;
      }
      delay(WAIT_READ_DELAY);
    }
    mySerial.readBytes(response, RESPONSE_CNT);
  }
}
