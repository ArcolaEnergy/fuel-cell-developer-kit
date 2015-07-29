/*
  Example program for running a fuel cell stack.
  
  Copyright (C) 2014,2015 Arcola Energy
  
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

void setup()
{
	Serial.begin(9600);
	stackController.start(); //This blocks until the capacitor is charged.
	                         //This only happens if the stack is connected.
	                         //It does not charge when the Arduino is on USB power.
}

void loop()
{
	//This takes care of short circuit, purging and updating electrical values.
	stackController.update();
	
	//You can get current and voltage from these functions:
	if ( stackController.getCurrent() > 4 )
		Serial.println( "Current too high!" );
	if ( stackController.getVoltage() < FCDKController::stack_type::minVoltage() )
		Serial.println( "Voltage too low!" );
}
