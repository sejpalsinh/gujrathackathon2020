#include <EEPROM.h>
#include "Adafruit_FONA.h"
#define SIM800L_RX 27
#define SIM800L_TX 26
#define SIM800L_PWRKEY 4
#define SIM800L_RST 5
#define SIM800L_POWER 23
char replybuffer[255];
HardwareSerial *sim800lSerial = &Serial1;
Adafruit_FONA sim800l = Adafruit_FONA(SIM800L_PWRKEY);
uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout = 0);
#define LED_BLUE 13
#define RELAY 14
String smsString = "";
void setup()
{
    setupForSim();
}

long prevMillis = 0;
int interval = 1000;
char sim800lNotificationBuffer[64];  //for notifications from the FONA
char smsBuffer[250];
int call_counter = 0;
int call_counter1 = 0;
boolean ledState = false;
boolean on_flag = false;

void loop()
{
    if (millis() - prevMillis > interval)
    {
        ledState = !ledState;
        prevMillis = millis();
    }
    if(on_flag)
    {
        Serial.println("power is on");
    }
    else
    {
        Serial.println("power is off");
    }
    char *bufPtr = sim800lNotificationBuffer; 
    if (sim800l.available())
    {
        int slot = 0;
        int charCount = 0;
        do {
            *bufPtr = sim800l.read();
            delay(1);
        } while ((*bufPtr++ != '\n') && (sim800l.available()) && (++charCount < (sizeof(sim800lNotificationBuffer) - 1)));
        *bufPtr = 0;

        if(1 == sscanf(sim800lNotificationBuffer, "+CMTI: \"SM\",%d", &slot))
        {
            Serial.println("ON SMS IF CONDITION");
            char callerIDbuffer[32];
            if (!sim800l.getSMSSender(slot, callerIDbuffer, 31))
            {
                Serial.println("Didn't find SMS message in slot!");
            }
            uint16_t smslen;
            if (sim800l.readSMS(slot, smsBuffer, 250, &smslen))
            {
                smsString = String(smsBuffer);
                Serial.println("this is sms");
                Serial.println(smsString);
            }

            if (smsString == "OFF")
            {
                Serial.println("in off sms");
                TurnOff();
                Serial.print(smsString);
                Serial.println(" : OFF");
                smsString = "done";
            }
            else if (smsString == "ON")
            {
                Serial.println("in on sms");
                TurnOn();
                Serial.print(smsString);
                Serial.println(" : ON");
                smsString = "done";
            }

            while (1)
            {
                if (sim800l.deleteSMS(slot))
                {
                    Serial.println(F("OK!"));
                    break;
                }
                else
                {
                    Serial.print(F("Couldn't delete SMS in slot "));
                    Serial.println(slot);
                    sim800l.print(F("AT+CMGD=?\r\n"));
                }
            }
        }
    }
    delay(1000);
}

void setupForSim()
{
    pinMode(LED_BLUE, OUTPUT);
    pinMode(RELAY, OUTPUT);
    pinMode(SIM800L_POWER, OUTPUT);
    digitalWrite(SIM800L_POWER, HIGH);
    Serial.begin(115200);
    Serial.println(F("ESP32 with GSM SIM800L"));
    Serial.println(F("Initializing....(May take more than 10 seconds)"));
    delay(10000);
    sim800lSerial->begin(4800, SERIAL_8N1, SIM800L_TX, SIM800L_RX);
    if (!sim800l.begin(*sim800lSerial))
    {
        Serial.println(F("Couldn't find GSM SIM800L"));
        while (1);
    }

    Serial.println(F("GSM SIM800L is OK"));
    char imei[16] = { 0 };  // MUST use a 16 character buffer for IMEI!
    uint8_t imeiLen = sim800l.getIMEI(imei);
    if (imeiLen > 0)
    {
        Serial.print("SIM card IMEI: ");
        Serial.println(imei);
    }

    sim800lSerial->print("AT+CNMI=2,1\r\n");
    Serial.println("GSM SIM800L Ready");
}

void TurnOn()
{
    makeonoff(18);
    on_flag = true;
    delay(2000);
}

void TurnOff()
{
    makeonoff(19);
    on_flag = false;
    delay(2000);
}
void makeonoff(int n)
{
  pinMode(n, OUTPUT);
  Serial.println(n);
  digitalWrite(n, HIGH);
  Serial.print(n);
  Serial.print("high");
  delay(40);
  Serial.print(n);
  Serial.print("low");
  pinMode(n, INPUT);
  digitalWrite(n, LOW);
}
bool isCalling()
{
  if(sim800l.getCallStatus() == 3){
    return true;
  }
  else{
    return false;
  }
}
