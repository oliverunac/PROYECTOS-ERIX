//*by yaploxoigolyb
//this file is based on MH-Z19 sensor library
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>
SoftwareSerial mySerial(10, 11); // RX, TX
static const int REQUEST_CNT = 9;
static const int RESPONSE_CNT = 9;
uint8_t getppm[REQUEST_CNT] = {0xff, 0x04, 0x03, 0x00, 0x00, 0x00, 0x01, 0xF4,0x00};
#define WAIT_READ_TIMES 100
#define WAIT_READ_DELAY 10

LiquidCrystal_I2C lcd(0x27, 20, 4);
int  measurement;

void setup() {
  lcd.init(); 
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("  ERIX ES UNA   ");
  lcd.setCursor(0, 1);
  lcd.print("    PERRITA     ");

  Serial.begin(9600);//MOSTRAR POR CONSOLA
  mySerial.begin(9600);//CONECTAR NUESTRO SENSOR
  //precalentamientp
  mySerial.write(getppm, REQUEST_CNT);//ENVIA POR SERIAL COMANDO -.....
  mySerial.flush();//LIMPIA EL BUFFER SERIAL
  //delay(1200000);//si es por primera vez
  Serial.println("PRECALENTAMIENTO....");
  for(int i=0;i<30;i++){
    Serial.println(i);
    delay(1000);//
  }
  Serial.println("SENSOR READY");
}
void loop() {
byte buf[RESPONSE_CNT-1];//BUF LLENA CON LOS DATOS DEL SENSOR
byte cheksum;
writeCommand(getppm, buf);//ENVIAMOS COMANDO read

  for (int i = 0; i < RESPONSE_CNT; i++) //LIMPIA BUF PARA RECEPECIONAR
  {
    buf[i] = 0x0;
  }//LLENAR DE CEROS EL BUF
  delay(500);
  writeCommand(getppm, buf);//ENVIAMOS COMNDOY BUF SE LLENA DE NUEVO
  // parse
  cheksum = (buf[1]+buf[2]+buf[3]+buf[4]+buf[5]+buf[6]+buf[7]);
  cheksum = (~cheksum)+1;
  
  if (buf[0] == 0xff && buf[1] == 0x04 && buf[8] == cheksum)
  {
    measurement = (buf[4] * 256 + buf[5])*1; // 0 500

  }
  else
  {
    measurement = -1;
  }

    Serial.println(measurement);

    delay(2000);

}

void writeCommand(uint8_t cmd[], uint8_t *response)
{
 mySerial.write(cmd, REQUEST_CNT);//ENVIA POR SERIAL
 mySerial.flush();//LIMPIA EL BUFFER SERIAL

  if (response != NULL)
  {
    int i = 0;
    while (mySerial.available() <= 0)
    {
      if (++i > WAIT_READ_TIMES)
      {
        Serial.println("can't get ZE16b-CO response.");
        return;
      }
      delay(WAIT_READ_DELAY);
    }
    mySerial.readBytes(response, RESPONSE_CNT);
  }
}
