#include "EngineSignalSimulator.h"

/*
Info from service manual:

 Crankshaft Sensor:
 The engine speed (RPM) sensor provides the Engine Control Module (ECM) with information about the speed and position of the crankshaft. The engine control module (ECM) is able to use the signal from the engine speed (RPM) sensor to determine when the piston in cylinder 1 is approaching top dead center (TDC). However it is unable to use the signal from the engine speed (RPM) sensor to determine whether the piston is in the combustion stroke or whether the exhaust valve is open (exhaust stroke). The signal from the camshaft position (CMP) sensor is also required to determine the operating cycle of the engine.
 The signal from the engine speed (RPM) sensor is also used to check the engine for misfires. For more information, see: Misfire diagnostic

 There is a steel ring with stamped holes welded to the rim of the primary section (the section fixed to the crankshaft) of the flywheel.
 The holes are positioned with a gap of 6째 between each hole. This arrangement creates a hole for each tooth. There are 360째 in one revolution. 6째 between each hole means that there are 60 holes. However two holes are not stamped, to create a reference position (long gap - missing tooth) for the crankshaft. The first tooth after the reference position is located 84째 before TDC on cylinder 1. See: Function

 The engine speed (RPM) sensor is at the rear of the engine above the flywheel.
 The sensor is inductive with a permanent magnet. An alternating current is induced in the sensor when the flywheel/carrier plate passes the engine speed (RPM) sensor. The generated voltage and frequency increases with the engine speed (rpm).
 The signal varies between 0.1-100 V depending on the engine speed (RPM).

 The Engine Control Module (ECM) is able to determine the engine speed (RPM) by counting the number of holes per time unit. When the reference position passes the engine speed (RPM) sensor, the voltage and frequency drop momentarily to zero, even though the engine is still running. This allows the engine control module (ECM) to determine the position of the crankshaft.

 If the signal from the engine speed (RPM) sensor is incorrect or missing, the control module will use the signals from the camshaft position (CMP) sensor, on the condition that the position of the camshaft has been adapted. This means that the car can be driven if the signal is missing.

 The engine control module (ECM) can diagnose the engine speed (RPM) sensor. The sensor value (engine speed (rpm)) can be read off using the diagnostic tool.

 Camshaft Sensor:
 The function of the camshaft position (CMP) sensor is to detect the flanks of the camshaft rotor. The signal from the sensor is used by the engine control module (ECM) to determine the angle of the camshaft.
 Each camshaft has four segments per camshaft revolution. A pulse wheel on the camshaft consisting of four teeth (the teeth are positioned by each flank) is used by the camshaft position sensor (CMP) to detect the segments.

 The teeth on the camshaft gear wheel are not equally wide. This allows the control module to determine which flank is detected and therefore which operating cycle the camshaft is in.
 When the operating cycle of the camshaft is established, the control module is able to determine which cylinder should be ignited. In the event of misfire or engine knock, the control module is also able to determine which cylinder is misfiring or knocking. Also see Design:Knock sensor (KS) and Design:Engine speed (RPM) sensor .

 Data about the position of the camshaft is used during camshaft control (CVVT). See also: Function

 The sensor, which is a magnetic resistor with a permanent magnet, is grounded in the control module and supplied with 5 V from the control module. When one of the teeth on the camshaft pulse wheel passes the camshaft position (CMP) sensor, a signal is transmitted to the control module from the camshaft position (CMP) sensor. The signal varies between 0-5 V and is high when a tooth is in contact with the camshaft position (CMP) sensor and low when the tooth leaves the camshaft position (CMP) sensor.

 The camshaft position (CMP) sensor is positioned at the rear of the engine by the controllable camshaft (CVVT).

 The engine control module (ECM) can diagnose the camshaft position (CMP) sensor.

 Detecting the position of the camshaft in relation to the position of the crankshaft
Each camshaft flank aligns with pre-defined positions on the crankshaft when the camshaft is in its 0 position. These positions on the crankshaft are called reference positions for the flanks.
The illustration shows how the signals relate to each other when the camshaft is in its 0 position (the camshaft is not deployed).
A: Engine speed (RPM) sensor signal.
B: Camshaft position (CMP) sensor signal. From high to low signal when the teeth on the camshaft pulley leave the camshaft position (CMP) sensor.
C: Low engine speed (RPM) sensor signal because of the holes in the flywheel/carrier plate.
D: Top dead center (TDC) cylinder 1, 0캜A (84캜A after hole "C" in the flywheel/carrier plate).
1: Detection of flank 1, reference position 47캜A "D1".
2: Detection of flank 2, reference position 227캜A "D2".
3: Detection of flank 3, reference position 407캜A "D3".
4: Detection of flank 4, reference position 587캜A "D4".
If the flanks do not correspond to the reference positions on the crankshaft when the camshaft is in the 0 position (not deployed), the engine control module (ECM) will store the difference. There may be a difference from the camshaft 0 position if the timing belt is incorrectly seated or the camshaft are not correctly set for example. A mechanically damaged camshaft reset valve may prevent the camshaft moving to the 0 position when the engine control module (ECM) stores the adaptation value for the deviation of the camshaft. This may result in high deviation and a diagnostic trouble code (DTC) being stored.
Deviations can be read out in the diagnostic tool.
 */

#define CFG_SERIAL_SPEED 115200

const int mafPin = 48; // the pin where the mass airflow sensor is assigned (PWM) - D2 on GEVCU2
const int mapPin = 32; // the pin where the manifold air pressure sensor is assigned (PWM) - D3 on GEVCU2
const int crankShaftPin = 52; // the pin where the crank shaft position signal will be sent - D0 on GEVCU2
const int camShaftPin = 22; // the pin where the cam shaft position (CMP) signal will be sent - D1 on GEVCU2

const boolean invertSignal = true; // are the signals being inverted (e.g. by a mosfet)
boolean crankShaftState = false; //state of the crankshaft pin (high/low)

const int flywheelTeeth = 58; // number of actual teeth on the flywheel (not counting the missing teeth)
const int flywheelMissingTeeth = 2; // number of missing teeth on the flywheel
const int idleRpm = 850; // the rpm at engine idle
const int maxRpm = 4000;

int motorRpm; // the current motor rpm (later queried from motor controller)
int crankShaftCounter = 0; // counts the transitions between teeth and holes (2 per tooth)
int crankShaftCounterMax = 0; // calculated max value of the crankShaftCounter (so it doesn't have to be re-calculated at every interrupt)
bool camShaftSecondHalf = false; // as the camshaft turns at half speed, this indicator shows which half of the camshaft is active
bool throttleRampUp = true;
uint16_t throttle = 0;

void crankShaftHandler() {

	// calculate and modify crank shaft signal
	if (crankShaftCounter < flywheelMissingTeeth * 2)
		crankShaftState = (invertSignal ? true : false); // simulate missing teeth
	else
		crankShaftState = !crankShaftState; // normal crank shaft signal

	digitalWrite(crankShaftPin, crankShaftState);


	// calculate and modify cam shaft signal
	if (crankShaftCounter == 43 || crankShaftCounter == 103) {
		digitalWrite(camShaftPin, (invertSignal ? true : false));// switch off cam shaft after 22 or 102 crank shaft pulses (131deg CA, 311deg CA)
	}

	if (camShaftSecondHalf) {
		if (crankShaftCounter == 95)
			digitalWrite(camShaftPin, (invertSignal ? false : true));// switch on cam shaft after 48 pulses (407deg CA)
	} else {
		if (crankShaftCounter == 35 || crankShaftCounter == 51 || crankShaftCounter == 111)
			digitalWrite(camShaftPin, (invertSignal ? false : true));// switch on cam shaft after 18, 26 or 56 pulses (23.5deg CA, 50.5deg CA, 250.5deg CA)
	}

	// check if crank shaft made one complete revolution
	if (++crankShaftCounter > crankShaftCounterMax) {
		crankShaftCounter = 0;
		camShaftSecondHalf = !camShaftSecondHalf;
	}
}

void mafMapHandler() {
	analogWrite(mafPin, throttle * motorRpm / maxRpm); // only a very rough approximation, requires more analysis for good model
	analogWrite(mapPin, throttle * 0.8); // only a very rough approximation, requires more analysis for good model
}

void setup() {
	Serial.begin(CFG_SERIAL_SPEED);
	Serial.println("starting rpm simulation");

	motorRpm = idleRpm;
	crankShaftCounterMax = (flywheelTeeth + flywheelMissingTeeth) * 2 - 1; // pre-calculate number of ticks for one revolution of crank shaft

	pinMode(crankShaftPin, OUTPUT);
	pinMode(camShaftPin, OUTPUT);

	Timer8.attachInterrupt(crankShaftHandler).setFrequency(motorRpm * 2).start(); // start timer for crank shaft handler
  	Timer1.attachInterrupt(mafMapHandler).setPeriod(100000).start(); // start timer for maf and map handler
}

void loop() {
	delay(100);

	Serial.print(crankShaftCounter);
	Serial.print(":");
	Serial.print(crankShaftState);
	Serial.print(" ");
	Serial.println(digitalRead(camShaftPin));

	// simulate a throttle change
	if (throttleRampUp) {
		if (++throttle > 255)
			throttleRampUp = !throttleRampUp;
	} else {
		if (--throttle < 1) {
			throttleRampUp = !throttleRampUp;
		}
	}

	motorRpm = throttle * maxRpm / 255; // simulate motor rpm according to throttle level (later to be taken read from motor controller
	Timer8.setFrequency(motorRpm * 2); // adjust tick frequency for crank shaft handler according to rpm

//	Serial.print("throttle: ");
//	Serial.print(throttle);
//	Serial.print(" rpm: ");
//	Serial.println(motorRpm);
}
