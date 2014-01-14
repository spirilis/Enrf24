#include <Enrf24.h>
#include <nRF24L01.h>
#include <SPI.h>

Enrf24 radio(P2_0, P2_1, P2_2);  // CE, CSN, IRQ pins

void setup()
{
  Serial.begin(9600);  // Serial port used to display scan results
  SPI.begin();
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
  
  radio.begin();  // Default 1Mbps, put 250000 or 2000000 to test those bitrates
  radio.enableRX();  // Wake up the transceiver
}

void loop()
{
  int16_t i, j;
  uint8_t strength, chan=0, dumpbuf[33];
  
  Serial.println("nRF24L01+ Channel Activity Scan");
  Serial.println("-------------------------------");

  // Sample channels 0 through 125.
  for (chan=0; chan < 126; chan++) {
    // Force RX off, change channel & force back on.
    radio.disableRX();
    radio.setChannel(chan);
    radio.enableRX();
    delay(1);  // Strawman delay to make sure the RF engine is fully spun-up on the new channel
    
    /* Sample the RPD register (Receive Power Detect, >= -64dBm signal detected)
     * 256 times, once every 4 milliseconds, incrementing the 'strength' variable each
     * time it's detected.  This will be used to produce a relative estimate of how
     * strong the ambient noise/crosstalk is on this channel.
     */
    strength = 0;
    for (i=0; i<256; i++) {
      if (radio.rfSignalDetected())
        strength++;
      delay(4);
    }
    
    /* Discover the highest bit set in our 'strength' variable; that will
     * be the size of the bargraph we'll show for this channel.
     */
    j = 0;
    for (i=7; i >= 0; i--) {
      if (strength & (1 << i)) {
        j = i+1;
        break;
      }
    }

    /* Each channel represents a 1MHz bandwidth starting at 2400MHz, e.g.
     * channel 125 is 2525MHz, channel 64 is 2464MHz.
     */
    Serial.print("Channel "); Serial.print(chan);
    Serial.print(" ("); Serial.print(2400+chan); Serial.print("MHz): ");
    for (i=0; i < j; i++) {
      Serial.print("*");
    }
    Serial.println(" ");
    
    if (radio.available()) {  /* Just in case some data got inadvertently received while
                               * we were doing our scan, clear it out.
                               */
      radio.read(dumpbuf);
    }
  }  // Loop to the next channel...
  
  Serial.println("Channel scan done; halting CPU.  Hit RESET to do another scan.");
  Serial.flush();
  radio.deepsleep();
  while(1) ;  // Permanently halt the CPU.  Using while(1) is more portable e.g. to Stellaris LP.
}

