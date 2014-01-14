#include <Enrf24.h>
#include <nRF24L01.h>
#include <string.h>
#include <SPI.h>

Enrf24 radio(P2_0, P2_1, P2_2);
const uint8_t rxaddr[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0x01 };

const char *str_on = "ON";
const char *str_off = "OFF";

void dump_radio_status_to_serialport(uint8_t);

void setup() {
  Serial.begin(9600);

  SPI.begin();
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
  
  radio.begin();  // Defaults 1Mbps, channel 0, max TX power
  dump_radio_status_to_serialport(radio.radioState());

  radio.setRXaddress((void*)rxaddr);
  
  pinMode(P1_0, OUTPUT);
  digitalWrite(P1_0, LOW);
  
  radio.enableRX();  // Start listening
}

void loop() {
  char inbuf[33];
  
  dump_radio_status_to_serialport(radio.radioState());  // Should show Receive Mode

  while (!radio.available(true))
    ;
  if (radio.read(inbuf)) {
    Serial.print("Received packet: ");
    Serial.println(inbuf);

    if (!strcmp(inbuf, str_on))
      digitalWrite(P1_0, HIGH);
    if (!strcmp(inbuf, str_off))
      digitalWrite(P1_0, LOW);
  }
}

void dump_radio_status_to_serialport(uint8_t status)
{
  Serial.print("Enrf24 radio transceiver status: ");
  switch (status) {
    case ENRF24_STATE_NOTPRESENT:
      Serial.println("NO TRANSCEIVER PRESENT");
      break;

    case ENRF24_STATE_DEEPSLEEP:
      Serial.println("DEEP SLEEP <1uA power consumption");
      break;

    case ENRF24_STATE_IDLE:
      Serial.println("IDLE module powered up w/ oscillators running");
      break;

    case ENRF24_STATE_PTX:
      Serial.println("Actively Transmitting");
      break;

    case ENRF24_STATE_PRX:
      Serial.println("Receive Mode");
      break;

    default:
      Serial.println("UNKNOWN STATUS CODE");
  }
}
