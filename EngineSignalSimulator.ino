#include <DueTimer.h>

/*
Info from service manual:

 Crankshaft Sensor:
 The engine speed (RPM) sensor provides the Engine Control Module (ECM) with information about the speed and position of the crankshaft. The engine control module (ECM) is able to use the signal from the engine speed (RPM) sensor to determine when the piston in cylinder 1 is approaching top dead center (TDC). However it is unable to use the signal from the engine speed (RPM) sensor to determine whether the piston is in the combustion stroke or whether the exhaust valve is open (exhaust stroke). The signal from the camshaft position (CMP) sensor is also required to determine the operating cycle of the engine.
 The signal from the engine speed (RPM) sensor is also used to check the engine for misfires. For more information, see: Misfire diagnostic

 There is a steel ring with stamped holes welded to the rim of the primary section (the section fixed to the crankshaft) of the flywheel.
 The holes are positioned with a gap of 6° between each hole. This arrangement creates a hole for each tooth. There are 360° in one revolution. 6° between each hole means that there are 60 holes. However two holes are not stamped, to create a reference position (long gap - missing tooth) for the crankshaft. The first tooth after the reference position is located 84° before TDC on cylinder 1. See: Function

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
 */

const int mafPin = 8; // the pin where the mass airflow sensor is assigned (PWM)
const int mapPin = 9; // the pin where the manifold air pressure sensor is assigned (PWM)
const int crankShaftPin = 10; // the pin where the crank shaft position signal will be sent
const int camShaftPin = 11; // the pin where the cam shaft position (CMP) signal will be sent

boolean crankShaftState = false; //state of the crankshaft pin (high/low)

const int flywheelTeeth = 58; // number of actual teeth on the flywheel (not counting the missing teeth)
const int flywheelMissingTeeth = 2; // number of missing teeth on the flywheel
const int idleRpm = 700; // the rpm at engine idle
const int maxRpm = 9000;

int motorRpm; // the current motor rpm (later queried from motor controller)
int crankShaftCounter = 0; // counts the transitions between teeth and holes (2 per tooth)
int crankShaftCounterMax = 0; // calculated max value of the crankShaftCounter (so it doesn't have to be re-calculated at every interrupt)
bool camShaftSecondHalf = false; // as the camshaft turns at half speed, this indicator shows which half of the camshaft is active
bool throttleRampUp = true;
uint16_t motorControllerThrottle = 0;

void crankShaftHandler() {
	if (crankShaftCounter < flywheelMissingTeeth * 2)
		crankShaftState = false;
	else
		crankShaftState = !crankShaftState;

	digitalWrite(crankShaftPin, crankShaftState);

	if (crankShaftCounter == 43 || crankShaftCounter == 103) {
		digitalWrite(camShaftPin, false);// switch off after 22 or 102 pulses (131� CA, 311� CA)
	}

	if (camShaftSecondHalf) {
		if (crankShaftCounter == 95)
			digitalWrite(camShaftPin, true);// switch on after 48 pulses (407� CA)
	} else {
		if (crankShaftCounter == 35 || crankShaftCounter == 51 || crankShaftCounter == 111)
			digitalWrite(camShaftPin, true);// switch on after 18, 26 or 56 pulses (23.5� CA, 50.5�CA, 250.5�CA)
	}

	if (++crankShaftCounter > crankShaftCounterMax) {
		crankShaftCounter = 0;
		camShaftSecondHalf = !camShaftSecondHalf;
	}
}

void handleTick() {
	uint16_t throttle = motorControllerThrottle;
	uint16_t rpm = motorRpm;

	uint16_t mapLevel = throttle * 0.8; // only a very rough approximation, requires more analysis for good model
	uint16_t mafLevel = throttle * rpm / maxRpm; // only a very rough approximation, requires more analysis for good model

	analogWrite(mapPin, mapLevel);
	analogWrite(mafPin, mafLevel);
}

void setup() {
	motorRpm = idleRpm;
	crankShaftCounterMax = (flywheelTeeth + flywheelMissingTeeth) * 2 - 1;

	pinMode(crankShaftPin, OUTPUT);
	pinMode(camShaftPin, OUTPUT);

	Timer8.attachInterrupt(crankShaftHandler).setFrequency(motorRpm * 2).start();
  	Timer1.attachInterrupt(handleTick).setPeriod(100000).start();
}

void loop() {
	delay(10);

	// simulate changes in the rpm and adjust cam/crankshaft timer
	motorRpm += 10;
	if (motorRpm > maxRpm)
		motorRpm = idleRpm;
	Timer8.setFrequency(motorRpm * 2);

	// simulate a throttle change
	if (throttleRampUp) {
		if (++motorControllerThrottle > 255)
			throttleRampUp = !throttleRampUp;
	} else {
		if (--motorControllerThrottle < 1) {
			throttleRampUp = !throttleRampUp;
		}
	}

}
