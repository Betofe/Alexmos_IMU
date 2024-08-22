/******************************************************************************
 This is example sketch for Arduino.
 Shows how to control SimpleBGC-driven gimbal via Serial API.
 API specs are available at http://www.simplebgc.com/eng/downloads/

 Connection:
 Arduino GND -> SimpleBGC GND
 Arduino TX -> SimpleBGC RX

 Power Arduino separatly or via +5V from onboard FTDI connector
 (C) Aleksey Moskalenko
*******************************************************************************/
#include <inttypes.h>
#include <HardwareSerial.h>

HardwareSerial Alexmos_Serial(1);
// default is 115200 but may be changed in the Advanced tab of the SimpleBGC GUI
#define SERIAL_SPEED 115200


#define CMD_BOARD_INFO 86


// delay between commands, ms
#define SBGC_CMD_DELAY 20
// Some definitions required to send commands
#define SBGC_CMD_CONTROL 'C'
#define SBGC_CMD_REALTIME 'D'
#define SBGC_CMD_READ 'R'
#define SBGC_CMD_TRIGGER 'T'
#define SBGC_CMD_VERSION 'V'
#define SBGC_CONTROL_MODE_SPEED 1
#define SBGC_CONTROL_MODE_ANGLE 2
#define SBGC_CONTROL_MODE_SPEED_ANGLE 3
// Pins that may be triggered
#define SBGC_RC_INPUT_ROLL 1
#define SBGC_RC_INPUT_PITCH 2
#define SBGC_RC_INPUT_EXT_ROLL 3
#define SBGC_RC_INPUT_EXT_PITCH 4
#define SBGC_RC_INPUT_YAW 5 // not connected in 1.0 board
#define SBGC_PIN_AUX1 16
#define SBGC_PIN_AUX2 17
#define SBGC_PIN_AUX3 18
#define SBGC_PIN_BUZZER 32
// Conversion from degree/sec to units that command understand
#define SBGC_SPEED_SCALE (1.0f/0.1220740379f)
// Holder for command parameters

typedef struct {
	int16_t cfg;
} SBGC_cmd_version_data;


typedef struct {
	uint8_t mode;
	int16_t speedROLL;
	int16_t angleROLL;
	int16_t speedPITCH;
	int16_t anglePITCH;
	int16_t speedYAW;
	int16_t angleYAW;
} SBGC_cmd_control_data;
typedef struct {
	uint8_t pin;
	int8_t state;
} SBGC_cmd_trigger_data;

typedef struct {
	int16_t acc;
	int16_t gyro;
	int16_t reserved_sensor;
	int16_t debug;
	int16_t rc_roll;
	int16_t rc_pitch;
	int16_t rc_yaw;
	int16_t rc_cmd;
	int16_t ext_fc_roll;
	int16_t ext_fc_pitch;
	int16_t angle_roll;
	int16_t angle_pitch;
	int16_t angle_yaw;
	int16_t cycle_time;
	int16_t i2c_error_count;
	int8_t error_code;
	int16_t bat_level;
	int8_t other_flags;
	int8_t cur_profile;
} SBGC_cmd_realtime_data;
typedef struct {
	int8_t profile_id;
} SBGC_cmd_read_data;
// This helper function formats and sends a command to SimpleBGC Serial API
void sendCommand(uint8_t cmd, void* data, uint8_t size) {
	uint8_t i, checksum = 0;
	// Header
	/*Alexmos_Serial.write(0x24);
	Alexmos_Serial.write(0x56);
	Alexmos_Serial.write(0x02);
	Alexmos_Serial.write(0x58);
	Alexmos_Serial.write(0x00);
	Alexmos_Serial.write(0x00);
	Alexmos_Serial.write(0xE6);
	Alexmos_Serial.write(0x13);*/

	 
	 
	Alexmos_Serial.write('$');
	Alexmos_Serial.write(cmd);
	Alexmos_Serial.write(size);
	Alexmos_Serial.write(cmd + size);
	// Body
	for (i = 0; i < size; i++) {
		checksum += ((uint8_t*)data)[i];
		Alexmos_Serial.write(((uint8_t*)data)[i]);
	}
	Alexmos_Serial.write(checksum);
}

void setup() {
	Serial.begin(SERIAL_SPEED);
	Alexmos_Serial.begin(SERIAL_SPEED, SERIAL_8N1, 18, 19);
}
void loop() {
	//SBGC_cmd_trigger_data t = { 0, 0 };
	//t.pin = SBGC_PIN_BUZZER;
	//t.state = 1;
	//SBGC_sendCommand(SBGC_CMD_TRIGGER, &t, sizeof(t));
	//delay(500);
	//t.pin = SBGC_PIN_BUZZER;
	//t.state = 0;
	//SBGC_sendCommand(SBGC_CMD_TRIGGER, &t, sizeof(t));
	//delay(2000);
	/*SBGC_cmd_realtime_data d = { 0, 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 ,0 };
	d.cur_profile = 2;*/
	SBGC_cmd_version_data v = { 0 };
	sendCommand(CMD_BOARD_INFO, &v, sizeof(v));

	Serial.println(Alexmos_Serial.read());
	delay(500);
}
