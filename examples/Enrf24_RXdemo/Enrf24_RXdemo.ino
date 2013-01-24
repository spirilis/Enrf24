#include <Enrf24.h>
#include <nRF24L01.h>
#include <string.h>
#include <SPI.h>

Enrf24 radio(P2_0, P2_1, P2_2);
const uint8_t rxaddr[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0x01 };

const char *str_on = "ON";
const char *str_off = "OFF";

void setup() {
  SPI.begin();
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(1); // MSB-first
  
  radio.begin();  // Defaults 1Mbps, channel 0, max TX power
  radio.setRXaddress((void*)rxaddr);
  
  pinMode(P1_0, OUTPUT);
  digitalWrite(P1_0, LOW);
  
  radio.enableRX();  // Start listening
}

void loop() {
  char inbuf[33];
  
  while (!radio.available(true))
    ;
  if (radio.read(inbuf)) {
    if (!strcmp(inbuf, str_on))
      digitalWrite(P1_0, HIGH);
    if (!strcmp(inbuf, str_off))
      digitalWrite(P1_0, LOW);
  }
}
