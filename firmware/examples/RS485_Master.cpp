/**
 *  Modbus master example:
 *  The purpose of this example is to query several sets of data
 *  from an external Modbus slave device.
 *  The link media can be USB or RS232.
 *
 *  Recommended Modbus slave:
 *  diagslave http://www.modbusdriver.com/diagslave.html
 *
 *  In a Linux box, run
 *  "./diagslave /dev/ttyUSB0 -b 9600 -d 8 -s 1 -p none -m rtu -a 1"
 * 	This is:
 * 		serial port /dev/ttyUSB0 at 9600 baud 8N1
 *		RTU mode and address @1
 */

#include "application.h"

SYSTEM_MODE(MANUAL); // no need for cell connection in this fw

#include "ModbusRtu.h"

uint16_t au16data[16]; //!< data array for modbus network sharing
uint8_t u8state; //!< machine state
uint8_t u8query; //!< pointer to message query

/**
 *  Modbus object declaration
 *  u8id : node id = 0 for master, = 1..247 for slave
 *  u8serno : serial port (use 0 for Serial)
 *  u8txenpin : 0 for RS-232 and USB-FTDI
 *               or any pin number > 1 for RS-485
 *  u8rxenpin : 0 for RS-232 and USB-FTDI
 *               or any pin number > 1 for RS-485
 *  can be declared with just TXEN_PIN via:
 *        Modbus master(0, 1, TXEN_PIN);
 */
#define TXEN_PIN A3
#define RXEN_PIN A2
// Modbus master();                            // initiaization of master on serial 0
// Modbus master(0, 1);                        // initiaization using no TXEN or RXEN control
// Modbus master(0, 1, TXEN_PIN);              // initiaization using only TXEN control
Modbus master(0, 1, TXEN_PIN, RXEN_PIN);    // initiaization using independent RXEN and TXEN control

/**
 * This is a struct which contains a query to a slave device
 */
#define NUMBER_OF_QUERIES 2
modbus_t telegram[NUMBER_OF_QUERIES];

unsigned long u32wait;

void setup() {
  // telegram 0: read registers
  telegram[0].u8id = 1; // slave address
  telegram[0].u8fct = 3; // function code (this one is registers read)
  telegram[0].u16RegAdd = 0; // start address in slave
  telegram[0].u16CoilsNo = 4; // number of elements (coils or registers) to read
  telegram[0].au16reg = au16data; // pointer to a memory array in the Arduino

  // telegram 1: write a single register
  telegram[1].u8id = 1; // slave address
  telegram[1].u8fct = 6; // function code (this one is write a single register)
  telegram[1].u16RegAdd = 6; // start address in slave
  telegram[1].u16CoilsNo = 1; // number of elements (coils or registers) to read
  telegram[1].au16reg = au16data+4; // pointer to a memory array in the Arduino

  master.begin( 9600 ); // baud-rate at 9600
  master.setTimeOut( 5000 ); // if there is no answer in 5000 ms, roll over
  u32wait = millis() + 1000;
  u8state = u8query = 0;

  Serial.begin(9600);
  while(!Serial.available());
  Serial.println("Starting up!");
}

void loop() {
  switch( u8state ) {
  case 0:
    if (millis() > u32wait) u8state++; // wait state
    break;
  case 1:
    master.query( telegram[u8query] ); // send query (only once)
    u8state++;
	u8query++;
	if (u8query > NUMBER_OF_QUERIES - 1) u8query = 0;
    break;
  case 2:
    master.poll(); // check incoming messages
    if (master.getState() == COM_IDLE) {
      u8state = 0;
      u32wait = millis() + 1000;

      Serial.println();
      Serial.println("Data:");
      for (uint8_t i = 0; i < 16; i++) {
        Serial.print(" ");
        Serial.print(au16data[i]);
      }
      Serial.println();

    }
    break;
  }

  au16data[4] = 5000;

}
