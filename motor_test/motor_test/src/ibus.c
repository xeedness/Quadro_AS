/*
 * ibus.c
 *
 * Created: 6/1/2021 9:42:38 PM
 *  Author: xeedn
 */ 

#include "ibus.h"
#include <asf.h>


// if you have an opentx transciever you can add additional sensor types here.
// see https://github.com/cleanflight/cleanflight/blob/7cd417959b3cb605aa574fc8c0f16759943527ef/src/main/telemetry/ibus_shared.h
// below the values supported by the Turnigy FS-MT6 transceiver
#define IBUSS_INTV 0x00 // Internal voltage (in 0.01)
#define IBUSS_TEMP 0x01 // Temperature (in 0.1 degrees, where 0=-40'C)
#define IBUSS_RPM  0x02 // RPM
#define IBUSS_EXTV 0x03 // External voltage (in 0.01)
#define IBUS_PRESS 0x41 // Pressure (in Pa)
#define IBUS_SERVO 0xfd // Servo value


uint16_t readChannel(uint8_t channelNr); // read servo channel 0..9
	
volatile uint8_t cnt_poll; // count received number of sensor poll messages
volatile uint8_t cnt_sensor; // count times a sensor value has been sent back
volatile uint8_t cnt_rec; // count received number of servo messages
	
enum State {GET_LENGTH, GET_DATA, GET_CHKSUML, GET_CHKSUMH, DISCARD};

#define PROTOCOL_LENGTH 0x20
const uint8_t PROTOCOL_OVERHEAD = 3; // packet is <len><cmd><data....><chkl><chkh>, overhead=cmd+chk bytes
const uint8_t PROTOCOL_TIMEGAP = 3; // Packets are received very ~7ms so use ~half that for the gap
#define PROTOCOL_CHANNELS 14
const uint8_t PROTOCOL_COMMAND40 = 0x40;        // Command to set servo or motor speed is always 0x40
const uint8_t PROTOCOL_COMMAND_DISCOVER = 0x80; // Command discover sensor (lowest 4 bits are sensor)
const uint8_t PROTOCOL_COMMAND_TYPE = 0x90;     // Command discover sensor (lowest 4 bits are sensor)
const uint8_t PROTOCOL_COMMAND_VALUE = 0xA0;    // Command send sensor data (lowest 4 bits are sensor)
const uint8_t SENSORMAX = 10; // Max number of sensors

#include "uart_bridge.h"
#include "timer.h"

uint8_t state = 0;                    // state machine state for iBUS protocol
uint32_t last;                    // milis() of prior message
uint8_t buffer[PROTOCOL_LENGTH];  // message buffer
uint8_t ptr;                      // pointer in buffer
uint8_t len;                      // message length
uint16_t channel[PROTOCOL_CHANNELS]; // servo data received
uint16_t chksum;                  // checksum calculation
uint8_t lchksum;                  // checksum lower byte received
/*typedef struct {
	uint8_t sensorType;             // sensor type (0,1,2,3, etc)
	uint8_t sensorLength;           // data length for defined sensor (can be 2 or 4)
	int32_t sensorValue;            // sensor data for defined sensors (16 or 32 bits)
} sensorinfo;
sensorinfo sensors[SENSORMAX];*/
uint8_t NumberSensors = 0;        // number of sensors
	
uint32_t startTicks;

void ibus_setup(void) {
	uart_bridge_init();
	startTicks = current_ticks();
}

void ibus_loop(void) {
	
	// only process data already in our UART receive buffer
	while (1) {
		// only consider a new data package if we have not heard anything for >3ms
		uint32_t now = elapsed_time_ms(startTicks);
		if (now - last >= PROTOCOL_TIMEGAP){
			state = GET_LENGTH;
		}
		last = now;
		uint8_t v;
		status_code_t status = usart_serial_read_packet(USART0, &v, 1);
		printf("Status %d: %d\n", status, v);
		if(status != STATUS_OK) {
			continue;
		}
		delay_us(100);
		continue;
		switch (state) {
		case GET_LENGTH:
			if (v <= PROTOCOL_LENGTH && v > PROTOCOL_OVERHEAD) {
				ptr = 0;
				len = v - PROTOCOL_OVERHEAD;
				chksum = 0xFFFF - v;
				state = GET_DATA;
			} else {
				state = DISCARD;
			}
			break;

		case GET_DATA:
			buffer[ptr++] = v;
			chksum -= v;
			if (ptr == len) {
				state = GET_CHKSUML;
			}
			break;
			
		case GET_CHKSUML:
			lchksum = v;
			state = GET_CHKSUMH;
			break;

		case GET_CHKSUMH:
			// Validate checksum
			if (chksum == (v << 8) + lchksum) {
				// Checksum is all fine Execute command -
				uint8_t adr = buffer[0] & 0x0f;
				if (buffer[0]==PROTOCOL_COMMAND40) {
					// Valid servo command received - extract channel data
					for (uint8_t i = 1; i < PROTOCOL_CHANNELS * 2 + 1; i += 2) {
						channel[i / 2] = buffer[i] | (buffer[i + 1] << 8);
					}
					cnt_rec++;
				} else if (adr<=NumberSensors && adr>0 && len==1) {

					// all sensor data commands go here
					// we only process the len==1 commands (=message length is 4 bytes incl overhead) to prevent the case the
					// return messages from the UART TX port loop back to the RX port and are processed again. This is extra
					// precaution as it will also be prevented by the PROTOCOL_TIMEGAP required
					/*sensorinfo *s = &sensors[adr-1];
					delayMicroseconds(100);
					switch (buffer[0] & 0x0f0) {
						case PROTOCOL_COMMAND_DISCOVER: // 0x80, discover sensor
						cnt_poll++;
						// echo discover command: 0x04, 0x81, 0x7A, 0xFF
						stream->write(0x04);
						stream->write(PROTOCOL_COMMAND_DISCOVER + adr);
						chksum = 0xFFFF - (0x04 + PROTOCOL_COMMAND_DISCOVER + adr);
						break;
						case PROTOCOL_COMMAND_TYPE: // 0x90, send sensor type
						// echo sensortype command: 0x06 0x91 0x00 0x02 0x66 0xFF
						stream->write(0x06);
						stream->write(PROTOCOL_COMMAND_TYPE + adr);
						stream->write(s->sensorType);
						stream->write(s->sensorLength);
						chksum = 0xFFFF - (0x06 + PROTOCOL_COMMAND_TYPE + adr + s->sensorType + s->sensorLength);
						break;
						case PROTOCOL_COMMAND_VALUE: // 0xA0, send sensor data
						cnt_sensor++;
						uint8_t t;
						// echo sensor value command: 0x06 0x91 0x00 0x02 0x66 0xFF
						stream->write(t = 0x04 + s->sensorLength);
						chksum = 0xFFFF - t;
						stream->write(t = PROTOCOL_COMMAND_VALUE + adr);
						chksum -= t;
						stream->write(t = s->sensorValue & 0x0ff);
						chksum -= t;
						stream->write(t = (s->sensorValue >> 8) & 0x0ff);
						chksum -= t;
						if (s->sensorLength==4) {
							stream->write(t = (s->sensorValue >> 16) & 0x0ff);
							chksum -= t;
							stream->write(t = (s->sensorValue >> 24) & 0x0ff);
							chksum -= t;
						}
						break;
						default:
						adr=0; // unknown command, prevent sending chksum
						break;
					}
					if (adr>0) {
						stream->write(chksum & 0x0ff);
						stream->write(chksum >> 8);
					}*/
				}
			}
			state = DISCARD;
			break;

		case DISCARD:
			default:
			break;
		}
		
		for(uint8_t channelNr=0; channelNr < PROTOCOL_CHANNELS; channelNr++) {
			uint16_t channelValue = readChannel(channelNr);
			if(channelValue != 0) {
				printf("Channel %d: %d\n", channelNr, channelValue);
			}
		}
	}
}

uint16_t readChannel(uint8_t channelNr) {
	if (channelNr < PROTOCOL_CHANNELS) {
		return channel[channelNr];
		} else {
		return 0;
	}
}