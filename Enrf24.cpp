/* nRF24L01+ I/O for Energia
 *
 * Copyright (c) 2013 Eric Brundick <spirilis [at] linux dot com>
 *  Permission is hereby granted, free of charge, to any person 
 *  obtaining a copy of this software and associated documentation 
 *  files (the "Software"), to deal in the Software without 
 *  restriction, including without limitation the rights to use, copy, 
 *  modify, merge, publish, distribute, sublicense, and/or sell copies 
 *  of the Software, and to permit persons to whom the Software is 
 *  furnished to do so, subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be 
 *  included in all copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, 
 *  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF 
 *  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
 *  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT 
 *  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, 
 *  WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER 
 *  DEALINGS IN THE SOFTWARE.
 */

#include <Arduino.h>
#include <stdint.h>
#include "Enrf24.h"

/* Constructor */
Enrf24::Enrf24(uint8_t cePin, uint8_t csnPin, uint8_t irqPin)
{
  _cePin = cePin;
  _csnPin = csnPin;
  _irqPin = irqPin;
  spibus = &SPI;

  rf_status = 0;
  rf_addr_width = 5;
  txbuf_len = 0;
  readpending = 0;
}

/* Initialization */
void Enrf24::begin(uint32_t datarate, uint8_t channel)
{
  pinMode(_cePin, OUTPUT);
  digitalWrite(_cePin, LOW);
  pinMode(_csnPin, OUTPUT);
  digitalWrite(_csnPin, HIGH);
  pinMode(_irqPin, INPUT);
  //digitalWrite(_irqPin, LOW);  // No pullups; the transceiver provides this!

  spibus->transfer(0);  // Strawman transfer, fixes USCI issue on G2553

  // Is the transceiver present/alive?
  if (!_isAlive())
    return;  // Nothing more to do here...

  // Wait 100ms for module to initialize
  while ( millis() < 100 )
    ;

  //Write to FEATURE Register and see if changes stick 
  _writeReg(RF24_FEATURE, RF24_EN_DPL);
  
  if (_readReg(RF24_FEATURE) == 0x00) {
    //If changes do not stick, issue an activate command
    uint8_t _activate_data = 0x73;
    _issueCmdPayload(RF24_ACTIVATE, &_activate_data, 1);
  }

  // Init certain registers
  _writeReg(RF24_CONFIG, 0x00);  // Deep power-down, everything disabled
  _writeReg(RF24_EN_AA, 0x03);
  _writeReg(RF24_EN_RXADDR, 0x03);
  _writeReg(RF24_RF_SETUP, 0x00);
  _writeReg(RF24_STATUS, ENRF24_IRQ_MASK);  // Clear all IRQs
  _writeReg(RF24_DYNPD, 0x03);
  _writeReg(RF24_FEATURE, RF24_EN_DPL);  // Dynamic payloads enabled by default

  // Set all parameters
  if (channel > 125)
    channel = 125;
  deepsleep();
  _issueCmd(RF24_FLUSH_TX);
  _issueCmd(RF24_FLUSH_RX);
  readpending = 0;
  _irq_clear(ENRF24_IRQ_MASK);
  setChannel(channel);
  setSpeed(datarate);
  setTXpower();
  setAutoAckParams();
  setAddressLength(rf_addr_width);
  setCRC(true);  // Default = CRC on, 8-bit
}

/* Formal shut-down/clearing of library state */
void Enrf24::end()
{
  txbuf_len = 0;
  rf_status = 0;
  rf_addr_width = 5;

  if (!_isAlive())
    return;
  deepsleep();
  _issueCmd(RF24_FLUSH_TX);
  _issueCmd(RF24_FLUSH_RX);
  readpending = 0;
  _irq_clear(ENRF24_IRQ_MASK);
  digitalWrite(_cePin, LOW);
  digitalWrite(_csnPin, HIGH);
}

/* Basic SPI I/O */
uint8_t Enrf24::_readReg(uint8_t addr)
{
  uint8_t result;

  digitalWrite(_csnPin, LOW);
  rf_status = spibus->transfer(RF24_R_REGISTER | addr);
  result = spibus->transfer(RF24_NOP);
  digitalWrite(_csnPin, HIGH);
  return result;
}

void Enrf24::_readRegMultiLSB(uint8_t addr, uint8_t *buf, size_t len)
{
  uint8_t i;
  digitalWrite(_csnPin, LOW);
  rf_status = spibus->transfer(RF24_R_REGISTER | addr);
  for (i=0; i<len; i++) {
    buf[len-i-1] = spibus->transfer(RF24_NOP);
  }
  digitalWrite(_csnPin, HIGH);
}

void Enrf24::_writeReg(uint8_t addr, uint8_t val)
{
  digitalWrite(_csnPin, LOW);
  rf_status = spibus->transfer(RF24_W_REGISTER | addr);
  spibus->transfer(val);
  digitalWrite(_csnPin, HIGH);
}

void Enrf24::_writeRegMultiLSB(uint8_t addr, uint8_t *buf, size_t len)
{
  size_t i;

  digitalWrite(_csnPin, LOW);
  rf_status = spibus->transfer(RF24_W_REGISTER | addr);
  for (i=0; i<len; i++) {
    spibus->transfer(buf[len-i-1]);
  }
  digitalWrite(_csnPin, HIGH);
}

void Enrf24::_issueCmd(uint8_t cmd)
{
  digitalWrite(_csnPin, LOW);
  rf_status = spibus->transfer(cmd);
  digitalWrite(_csnPin, HIGH);
}

void Enrf24::_issueCmdPayload(uint8_t cmd, uint8_t *buf, size_t len)
{
  size_t i;

  digitalWrite(_csnPin, LOW);
  rf_status = spibus->transfer(cmd);
  for (i=0; i<len; i++) {
    spibus->transfer(buf[i]);
  }
  digitalWrite(_csnPin, HIGH);
}

void Enrf24::_readCmdPayload(uint8_t cmd, uint8_t *buf, size_t len, size_t maxlen)
{
  size_t i;

  digitalWrite(_csnPin, LOW);
  rf_status = spibus->transfer(cmd);
  for (i=0; i<len; i++) {
    if (i < maxlen) {
      buf[i] = spibus->transfer(RF24_NOP);
    } else {
      spibus->transfer(RF24_NOP);  // Beyond maxlen bytes, just discard the remaining data.
    }
  }
  digitalWrite(_csnPin, HIGH);
}

boolean Enrf24::_isAlive()
{
  uint8_t aw;

  aw = _readReg(RF24_SETUP_AW);
  return ((aw & 0xFC) == 0x00 && (aw & 0x03) != 0x00);
}

uint8_t Enrf24::_irq_getreason()
{
  lastirq = _readReg(RF24_STATUS) & ENRF24_IRQ_MASK;
  return lastirq;
}

// Get IRQ from last known rf_status update without querying module over SPI.
uint8_t Enrf24::_irq_derivereason()
{
  lastirq = rf_status & ENRF24_IRQ_MASK;
  return lastirq;
}

void Enrf24::_irq_clear(uint8_t irq)
{
  _writeReg(RF24_STATUS, irq & ENRF24_IRQ_MASK);
}

#define ENRF24_CFGMASK_CRC(a) (a & (RF24_EN_CRC | RF24_CRCO))

void Enrf24::_readTXaddr(uint8_t *buf)
{
  _readRegMultiLSB(RF24_TX_ADDR, buf, rf_addr_width);
}

void Enrf24::_writeRXaddrP0(uint8_t *buf)
{
  _writeRegMultiLSB(RF24_RX_ADDR_P0, buf, rf_addr_width);
}


/* nRF24 I/O maintenance--called as a "hook" inside other I/O functions to give
 * the library a chance to take care of its buffers et al
 */
void Enrf24::_maintenanceHook()
{
  uint8_t i;

  _irq_getreason();

  if (lastirq & ENRF24_IRQ_TXFAILED) {
    lastTXfailed = true;
    _issueCmd(RF24_FLUSH_TX);
    _irq_clear(ENRF24_IRQ_TXFAILED);
  }

  if (lastirq & ENRF24_IRQ_TX) {
    lastTXfailed = false;
    _irq_clear(ENRF24_IRQ_TX);
  }

  if (lastirq & ENRF24_IRQ_RX) {
    if ( !(_readReg(RF24_FIFO_STATUS) & RF24_RX_FULL) ) {  /* Don't feel it's necessary
                                                            * to be notified of new
                                                            * incoming packets if the RX
                                                            * queue is full.
                                                            */
      _irq_clear(ENRF24_IRQ_RX);
    }

    /* Check if RX payload is 0-byte or >32byte (erroneous conditions)
     * Also check if data was received on pipe#0, which we are ignoring.
     * The reason for this is pipe#0 is needed for receiving AutoACK acknowledgements,
     * its address gets reset to the module's default and we do not care about data
     * coming in to that address...
     */
    if ( (rf_status & 0x0E) != 0x0E ) {  // Only check if there is an actual packet
      _readCmdPayload(RF24_R_RX_PL_WID, &i, 1, 1);
      if (i == 0 || i > 32 || ((rf_status & 0x0E) >> 1) == 0) {
                               /* Zero-width RX payload is an error that happens a lot
                                * with non-AutoAck, and must be cleared with FLUSH_RX.
                                * Erroneous >32byte packets are a similar phenomenon.
                                */
        _issueCmd(RF24_FLUSH_RX);
        _irq_clear(ENRF24_IRQ_RX);
        readpending = 0;
      } else {
        readpending = 1;
      }
    }
    // Actual scavenging of RX queues is performed by user-directed use of read().
  }
}



/* Public functions */
boolean Enrf24::available(boolean checkIrq)
{
  if (checkIrq && digitalRead(_irqPin) == HIGH && readpending == 0)
    return false;
  _maintenanceHook();
  if ( !(_readReg(RF24_FIFO_STATUS) & RF24_RX_EMPTY) ) {
    return true;
  }
  if (readpending) {
    return true;
  }
  return false;
}

size_t Enrf24::read(void *inbuf, uint8_t maxlen)
{
  uint8_t *buf = (uint8_t *)inbuf;
  uint8_t plwidth;

  _maintenanceHook();
  readpending = 0;
  if ((_readReg(RF24_FIFO_STATUS) & RF24_RX_EMPTY) || maxlen < 1) {
    return 0;
  }
  _readCmdPayload(RF24_R_RX_PL_WID, &plwidth, 1, 1);
  _readCmdPayload(RF24_R_RX_PAYLOAD, buf, plwidth, maxlen);
  buf[plwidth] = '\0';  // Zero-terminate in case this is a string.
  if (_irq_derivereason() & ENRF24_IRQ_RX) {
    _irq_clear(ENRF24_IRQ_RX);
  }

  return (size_t) plwidth;
}

// Perform TX of current ring-buffer contents
void Enrf24::flush()
{
  uint8_t reg, addrbuf[5];
  boolean enaa=false, origrx=false;

  if (!txbuf_len)
    return;  // Zero-length buffer?  Nothing to send!

  reg = _readReg(RF24_FIFO_STATUS);
  if (reg & BIT5) {  // RF24_TX_FULL #define is BIT0, which is not the correct bit for FIFO_STATUS.
    // Seen this before with a user whose CE pin was messed up.
    _issueCmd(RF24_FLUSH_TX);
    txbuf_len = 0;
    return;  // Should never happen, but nonetheless a precaution to take.
  }

  _maintenanceHook();

  if (reg & RF24_TX_REUSE) {
    // If somehow TX_REUSE is enabled, we need to flush the TX queue before loading our new payload.
    _issueCmd(RF24_FLUSH_TX);
  }

  if (_readReg(RF24_EN_AA) & 0x01 && (_readReg(RF24_RF_SETUP) & 0x28) != 0x20) {
    /* AutoACK enabled, must write TX addr to RX pipe#0
     * Note that 250Kbps doesn't support auto-ack, so we check RF24_RF_SETUP to verify that.
     */
    enaa = true;
    _readTXaddr(addrbuf);
    _writeRXaddrP0(addrbuf);
  }

  reg = _readReg(RF24_CONFIG);
  if ( !(reg & RF24_PWR_UP) ) {
    //digitalWrite(_cePin, HIGH);  // Workaround for SI24R1 knockoff chips
    _writeReg(RF24_CONFIG, ENRF24_CFGMASK_IRQ | ENRF24_CFGMASK_CRC(reg) | RF24_PWR_UP);
    delay(5);  // 5ms delay required for nRF24 oscillator start-up
    //digitalWrite(_cePin, LOW);
  }
  if (reg & RF24_PRIM_RX) {
    origrx=true;
    digitalWrite(_cePin, LOW);
    _writeReg(RF24_CONFIG, ENRF24_CFGMASK_IRQ | ENRF24_CFGMASK_CRC(reg) | RF24_PWR_UP);
  }

  _issueCmdPayload(RF24_W_TX_PAYLOAD, txbuf, txbuf_len);
  digitalWrite(_cePin, HIGH);
  delayMicroseconds(100);
  digitalWrite(_cePin, LOW);

  txbuf_len = 0;  // Reset TX ring buffer

  while (digitalRead(_irqPin) == HIGH)  // Wait until IRQ fires
    ;
  // IRQ fired
  _maintenanceHook();  // Handle/clear IRQ

  // Purge Pipe#0 address (set to module's power-up default)
  if (enaa) {
    addrbuf[0] = 0xE7; addrbuf[1] = 0xE7; addrbuf[2] = 0xE7; addrbuf[3] = 0xE7; addrbuf[4] = 0xE7;
    _writeRXaddrP0(addrbuf);
  }

  // If we were in RX mode before writing, return back to RX mode.
  if (origrx) {
    enableRX();
  }
}

void Enrf24::purge()
{
  txbuf_len = 0;
}

size_t Enrf24::write(uint8_t c)
{
  if (txbuf_len == 32) {  // If we're trying to stuff an already-full buffer...
    flush();  // Blocking OTA TX
  }

  txbuf[txbuf_len] = c;
  txbuf_len++;

  return 1;
}

uint8_t Enrf24::radioState()
{
  uint8_t reg;

  if (!_isAlive())
    return ENRF24_STATE_NOTPRESENT;
  
  reg = _readReg(RF24_CONFIG);
  if ( !(reg & RF24_PWR_UP) )
    return ENRF24_STATE_DEEPSLEEP;

  // At this point it's either Standby-I, II or PRX.
  if (reg & RF24_PRIM_RX) {
    if (digitalRead(_cePin))
      return ENRF24_STATE_PRX;
    // PRIM_RX=1 but CE=0 is a form of idle state.
    return ENRF24_STATE_IDLE;
  }
  // Check if TX queue is empty, if so it's idle, if not it's PTX.
  if (_readReg(RF24_FIFO_STATUS) & RF24_TX_EMPTY)
    return ENRF24_STATE_IDLE;
  return ENRF24_STATE_PTX;
}

void Enrf24::deepsleep()
{
  uint8_t reg;

  reg = _readReg(RF24_CONFIG);
  if (reg & (RF24_PWR_UP | RF24_PRIM_RX)) {
    _writeReg(RF24_CONFIG, ENRF24_CFGMASK_IRQ | ENRF24_CFGMASK_CRC(reg));
  }
  digitalWrite(_cePin, LOW);
}

void Enrf24::enableRX()
{
  uint8_t reg;

  reg = _readReg(RF24_CONFIG);
  _writeReg(RF24_CONFIG, ENRF24_CFGMASK_IRQ | ENRF24_CFGMASK_CRC(reg) | RF24_PWR_UP | RF24_PRIM_RX);
  digitalWrite(_cePin, HIGH);

  if ( !(reg & RF24_PWR_UP) ) {  // Powering up from deep-sleep requires 5ms oscillator start delay
    delay(5);
  }
}

void Enrf24::disableRX()
{
  uint8_t reg;

  digitalWrite(_cePin, LOW);

  reg = _readReg(RF24_CONFIG);
  if (reg & RF24_PWR_UP) {  /* Keep us in standby-I if we're coming from RX mode, otherwise stay
                             * in deep-sleep if we call this while already in PWR_UP=0 mode.
                             */
    _writeReg(RF24_CONFIG, ENRF24_CFGMASK_IRQ | ENRF24_CFGMASK_CRC(reg) | RF24_PWR_UP);
  } else {
    _writeReg(RF24_CONFIG, ENRF24_CFGMASK_IRQ | ENRF24_CFGMASK_CRC(reg));
  }
}

void Enrf24::autoAck(boolean onoff)
{
  uint8_t reg;

  reg = _readReg(RF24_EN_AA);
  if (onoff) {
    if ( !(reg & 0x01) || !(reg & 0x02) ) {
      _writeReg(RF24_EN_AA, 0x03);
    }
  } else {
    if (reg & 0x03) {
      _writeReg(RF24_EN_AA, 0x00);
    }
  }
}

void Enrf24::setChannel(uint8_t channel)
{
  if (channel > 125)
    channel = 125;
  _writeReg(RF24_RF_CH, channel);
}

void Enrf24::setTXpower(int8_t dBm)
{
  uint8_t reg, pwr;

  reg = _readReg(RF24_RF_SETUP) & 0xF8;  // preserve RF speed settings
  pwr = 0x06;
  if (dBm >= 7)
    pwr = 0x07;
  if (dBm < 0)
    pwr = 0x04;
  if (dBm < -6)
    pwr = 0x02;
  if (dBm < -12)
    pwr = 0x00;
  _writeReg(RF24_RF_SETUP, reg | pwr);
}

void Enrf24::setSpeed(uint32_t rfspeed)
{
  uint8_t reg, spd;

  reg = _readReg(RF24_RF_SETUP) & 0xD7;  // preserve RF power settings
  spd = 0x01;
  if (rfspeed < 2000000)
    spd = 0x00;
  if (rfspeed < 1000000)
    spd = 0x04;
  _writeReg(RF24_RF_SETUP, reg | (spd << 3));
}

void Enrf24::setCRC(boolean onoff, boolean crc16bit)
{
  uint8_t reg, crcbits=0;

  reg = _readReg(RF24_CONFIG) & 0xF3;  // preserve IRQ mask, PWR_UP/PRIM_RX settings
  if (onoff)
    crcbits |= RF24_EN_CRC;
  if (crc16bit)
    crcbits |= RF24_CRCO;
  _writeReg(RF24_CONFIG, reg | crcbits);
}

void Enrf24::setAutoAckParams(uint8_t autoretry_count, uint16_t autoretry_timeout)
{
  uint8_t setup_retr=0;

  setup_retr = autoretry_count & 0x0F;
  autoretry_timeout -= 250;
  setup_retr |= ((autoretry_timeout / 250) & 0x0F) << 4;
  _writeReg(RF24_SETUP_RETR, setup_retr);
}

void Enrf24::setAddressLength(size_t len)
{
  if (len < 3)
    len = 3;
  if (len > 5)
    len = 5;

  _writeReg(RF24_SETUP_AW, len-2);
  rf_addr_width = len;
}

void Enrf24::setRXaddress(const void *rxaddr)
{
  _writeRegMultiLSB(RF24_RX_ADDR_P1, (uint8_t*)rxaddr, rf_addr_width);
}

void Enrf24::setTXaddress(const void *rxaddr)
{
  _writeRegMultiLSB(RF24_TX_ADDR, (uint8_t*)rxaddr, rf_addr_width);
}

boolean Enrf24::rfSignalDetected()
{
  uint8_t rpd;

  rpd = _readReg(RF24_RPD);
  return (boolean)rpd;
}

uint32_t Enrf24::getSpeed()
{
  uint8_t reg = _readReg(RF24_RF_SETUP) & 0x28;

  switch (reg) {
    case 0x00:
      return 1000000UL;
    case 0x08:
      return 2000000UL;
    case 0x20:
      return 250000UL;
  }
  return 0UL;
}

int8_t Enrf24::getTXpower()
{
  uint8_t reg = _readReg(RF24_RF_SETUP) & 0x07;

  if (reg & 0x01)
    return 7;  // SI24R1-only +7dBm mode
  switch (reg) {
    case 0x02:
      return -12;
    case 0x04:
      return -6;
    case 0x06:
      return 0;
  }
  return -18;
}

boolean Enrf24::getAutoAck()
{
  uint8_t reg = _readReg(RF24_EN_AA);

  if (reg)
    return true;
  return false;
}

void Enrf24::getRXaddress(void *buf)
{
  _readRegMultiLSB(RF24_RX_ADDR_P1, (uint8_t*)buf, rf_addr_width);
}

void Enrf24::getTXaddress(void *buf)
{
  _readRegMultiLSB(RF24_TX_ADDR, (uint8_t*)buf, rf_addr_width);
}

unsigned int Enrf24::getCRC()
{
  uint8_t reg = _readReg(RF24_CONFIG) & 0x0C;

  switch (reg) {
    case 0x08:
      return 8;
    case 0x0C:
      return 16;
  }

  return 0;
}
