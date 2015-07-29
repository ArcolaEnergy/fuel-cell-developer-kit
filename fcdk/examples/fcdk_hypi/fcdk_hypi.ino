/*
  Example program for running a fuel cell stack and powering a RaspberryPi.
  For more info see the website:
  http://arcolaenergy.com/collections/open-source-systems
  
  Copyright (C) 2014,2015 Arcola Energy, 
  Derrived from works (C) Andy Stanford Clark, IBM used under license
  
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
  
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
  
  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "fcdk.h"
//Important: Select the correct stack type here.
typedef fcdk::Controller<fcdk::H12Stack> FCDKController;
//typedef fcdk::Controller<fcdk::H30Stack> FCDKController;
//typedef fcdk::Controller<fcdk::MiniPakStack> FCDKController;
//typedef fcdk::Controller<fcdk::MiniPakStackX2> FCDKController;
FCDKController stackController;

//Pins the pressure sensor, shorting button and, relays are connected to
const int pressureSwitchPin = 12;
const int shortingButtonPin = 13;
const int isolateRelayPin = 11;
const int usbPowerRelayPin = 10;

//Counters
unsigned int lowVoltageCounter = 0;
unsigned int lowPressureCounter = 0;
unsigned int highCurrentCounter = 0;

bool shortingEnabled = false;
bool usbOn = false;

void setup()
{
	Serial.begin(9600);
	
	//As soon as we are powered up, close the isolation relay to keep power on when the switch is released.
	pinMode(isolateRelayPin, OUTPUT);
	digitalWrite(isolateRelayPin, HIGH);
	
	//Make sure the USB power relay is open until we are ready to provide stable power.
	pinMode(usbPowerRelayPin, OUTPUT);
	digitalWrite(usbPowerRelayPin, LOW);
	
	//Set pressure sense as input with PULLUP resistor active.
	pinMode(pressureSwitchPin, INPUT);
	digitalWrite(pressureSwitchPin, HIGH);
	
	//Set shorting pushbutton as input with PULLUP resistor active.
	pinMode(shortingButtonPin, INPUT);
	digitalWrite(shortingButtonPin, HIGH);
	
	//If the shorting button is low, the button is pressed, so turn on shorting.
	if (digitalRead(shortingButtonPin) == LOW) {
		Serial.println("<<<<<< shorting button not pressed >>>>>>");
		shortingEnabled = true;
	} else {
		Serial.println("<<<<<< shorting disabled >>>>>>");
		shortingEnabled = false;
	}
	
	lowVoltageCounter = 0;
	lowPressureCounter = 0;
	highCurrentCounter = 0;
	usbOn = false;
  
	stackController.start(); //This blocks until the capacitor is charged.
	                         //This only happens if the stack is connected.
	                         //It does not charge when the Arduino is on USB power.
}

void loop()
{
	//this takes care of short circuit, purging and updating electrical values
	stackController.update();
	
	if (stackController.getCurrent() > FCDKController::stack_type::maxCurrent()) {
		++highCurrentCounter;
		Serial.print("High current ");
		Serial.println(highCurrentCounter);
		//If the current is too high for 100 or more consecutive counts, disconnect the stack to protect it from damage and hang the program.
		if (highCurrentCounter >= 100) {
			Serial.println("Current too high for too long! - shutting down.");
			delay(1000); //allow serial message to reach the RaspberryPi
			digitalWrite(isolateRelayPin, LOW);
			while(true) {}
		}
	} else {
		//reset the counter
		highCurrentCounter = 0;
	}
	if (stackController.getVoltage() < FCDKController::stack_type::minVoltage()) {
		if (usbOn) {
			++lowVoltageCounter;
			Serial.print("Low voltage ");
			Serial.println(lowVoltageCounter);
			//If the voltage is too low for 100 or more consecutive counts, disconnect the stack to protect it from damage and hang the program.
			if (lowVoltageCounter >= 100) {
				Serial.println("Voltage too low for too long! - shutting down.");
				delay(1000); //allow serial message to reach the RaspberryPi
				digitalWrite(isolateRelayPin, LOW);
				while(true) {}
			}
		}
	} else {
		//reset the counter
		lowVoltageCounter = 0;
	}
	
	//Pressure switch goes high when the pressure is too low
	if (digitalRead(pressureSwitchPin) == HIGH) {
		++lowPressureCounter;
		Serial.print("Low pressure ");
		Serial.println(lowPressureCounter);
		//If the low pressure count reaches 100, assume the Hydrogen is running out and disconnect the stack.
		if (lowPressureCounter >= 100) {
			Serial.println("Pressure too low for too long! - shutting down.");
			delay(1000); //allow serial message to reach the RaspberryPi
			digitalWrite(isolateRelayPin, LOW);
			while(true) {}
		} else {
			Serial.print("Low pressure. Counter: ");
			Serial.println(lowPressureCounter);
		}
	} else {
		lowPressureCounter = 0;
	}
	
	//After at least one purge and at least four shorts, enable the USB power
	if ( (!usbOn) && (stackController.getNumPurges() >= 1) && (stackController.getNumShortCircuits() >= 4) ) {
		//Disable shorting so that power is stable, unless the shorting button was pressed.
		if (!shortingEnabled) {
			stackController.disableShortCircuit();
		}
		//Wait to make sure that the power has returned to normal after the last short.
		delay(1000);
		//switch on the USB power
		digitalWrite(usbPowerRelayPin, HIGH);
		usbOn = true;
		Serial.println("------- USB power turned on ---------");
	}
}
