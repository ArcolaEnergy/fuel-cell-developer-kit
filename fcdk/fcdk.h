/*
  Arcola Energy FCDK fuelcell controller for 3, 12 and 30W stacks.
  http://arcolaenergy.com/collections/open-source-systems
  
  Copyright (C) 2014 Arcola Energy
  
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
#ifndef fcdk_h
#define fcdk_h

#if ARDUINO >= 100
	#include <Arduino.h>
#else
	#include <WProgram.h>
#endif

namespace fcdk
{
	
	//Timings for short-circuiting and purging of the stack
	//All times are in milliseconds
	class StackTimings
	{
		public:
			static const unsigned int defaultShortCircuitOffset = 5000;
			
			StackTimings(const unsigned int shortCircuitPeriod,
			             const unsigned int shortCircuitLength,
			             const unsigned long purgePeriod,
			             const unsigned int purgeLength,
			             const unsigned int shortCircuitOffset = defaultShortCircuitOffset)
				:	_shortCircuitPeriod(shortCircuitPeriod),
					_shortCircuitLength(shortCircuitLength),
					_purgePeriod(purgePeriod),
					_purgeLength(purgeLength),
					_shortCircuitOffset(shortCircuitOffset)
				{ }
			
			unsigned int getShortCircuitPeriod() const { return _shortCircuitPeriod; }
			StackTimings& setShortCircuitPeriod(const unsigned int shortCircuitPeriod)
				{ _shortCircuitPeriod = shortCircuitPeriod; return *this; }
			unsigned int getShortCircuitLength() const { return _shortCircuitLength; }
			StackTimings& setShortCircuitLength(const unsigned int shortCircuitLength)
				{ _shortCircuitLength = shortCircuitLength; return *this; }
			unsigned long getPurgePeriod() const { return _purgePeriod; }
			StackTimings& setPurgePeriod(const unsigned long purgePeriod)
				{ _purgePeriod = purgePeriod; return *this; }
			unsigned int getPurgeLength() const { return _purgeLength; }
			StackTimings& setPurgeLength(const unsigned int purgeLength)
				{ _purgeLength = purgeLength; return *this; }
			//offset time to ensure that purge and short don't happen together
			unsigned int getShortCircuitOffset() const { return _shortCircuitOffset; }
			StackTimings& setShortCircuitOffset(const unsigned int shortCircuitOffset)
				{ _shortCircuitOffset = shortCircuitOffset; return *this; }
			
			StackTimings(const StackTimings& other)
		  	:	_shortCircuitPeriod(other._shortCircuitPeriod),
    			_shortCircuitLength(other._shortCircuitLength),
    			_purgePeriod(other._purgePeriod),
    			_purgeLength(other._purgeLength),
    			_shortCircuitOffset(other._shortCircuitOffset)
  			{ }
  		
		  StackTimings& operator = (const StackTimings& other)
  		{
    		_shortCircuitPeriod = other._shortCircuitPeriod;
    		_shortCircuitLength = other._shortCircuitLength;
    		_purgePeriod = other._purgePeriod;
    		_purgeLength = other._purgeLength;
    		_shortCircuitOffset = other._shortCircuitOffset;
    		return *this;
  		}
			
		private:
			unsigned int _shortCircuitPeriod;
			unsigned int _shortCircuitLength;
			unsigned long _purgePeriod;
			unsigned int _purgeLength;
			unsigned int _shortCircuitOffset;
	};
	
	//Timings for operation of board
	//All times are in milliseconds
	class BoardTimings
	{
		public:
			static const unsigned int defaultBlinkPeriod = 500;
			static const unsigned int defaultElectricUpdatePeriod = 100;
			static const unsigned long defaultCurrentLPFPeriod = 500;
			
			BoardTimings(const unsigned int blinkPeriod = defaultBlinkPeriod,
			             const unsigned int electricUpdatePeriod = defaultElectricUpdatePeriod,
			             const unsigned long currentLPFPeriod = defaultCurrentLPFPeriod)
				: _blinkPeriod(blinkPeriod),
					_electricUpdatePeriod(electricUpdatePeriod),
					_currentLPFPeriod(currentLPFPeriod),
					_currentLPFCoeff(_calculateCurrentLPFCoeff())
				{ }
			
			//period of LED blinking
			unsigned int getBlinkPeriod() const { return _blinkPeriod; }
			BoardTimings& setBlinkPeriod(const unsigned int blinkPeriod)
				{ _blinkPeriod = blinkPeriod; return *this; }
			//period of updates to electrical status values
			unsigned int getElectricUpdatePeriod() const { return _electricUpdatePeriod; }
			BoardTimings& setElectricUpdatePeriod(const unsigned int electricUpdatePeriod)
			{
				_electricUpdatePeriod = electricUpdatePeriod;
				_currentLPFCoeff = _calculateCurrentLPFCoeff();
				return *this;
			}
			//period over which to smooth the current measurements (must be significantly longer than electricUpdatePeriod)
			unsigned long getCurrentLPFPeriod() const { return _currentLPFPeriod; }
			BoardTimings& setCurrentLPFPeriod(const unsigned long currentLPFPeriod)
			{
				_currentLPFPeriod = currentLPFPeriod;
				_currentLPFCoeff = _calculateCurrentLPFCoeff();
				return *this;
			}
			float getCurrentLPFCoeff() const { return _currentLPFCoeff; }
			
			BoardTimings(const BoardTimings& other)
				:	_blinkPeriod(other._blinkPeriod),
					_electricUpdatePeriod(other._electricUpdatePeriod),
					_currentLPFPeriod(other._currentLPFPeriod),
					_currentLPFCoeff(_calculateCurrentLPFCoeff())
				{ }
			
			BoardTimings& operator = (const BoardTimings& other)
			{
				_blinkPeriod = other._blinkPeriod;
				_electricUpdatePeriod = other._electricUpdatePeriod;
				_currentLPFPeriod = other._currentLPFPeriod;
				_currentLPFCoeff = _calculateCurrentLPFCoeff();
				return *this;
			}
			
		private:
			//calculate coefficient for LPF filter, calculated from period and electric update period
			float _calculateCurrentLPFCoeff() const { return float(_electricUpdatePeriod)/float(_currentLPFPeriod); }
			
			unsigned int _blinkPeriod;
			unsigned int _electricUpdatePeriod;
			unsigned long _currentLPFPeriod;
			float _currentLPFCoeff;
	};
	
	//Implements the details of operations needed by controller using constants defined by BoardVersion.
	//Unless otherwise stated, voltages are in V.
	template <typename BoardVersion>
	struct FCDKBoard
	{
		static float currentFromRawValue(const float rawCurrent);
		static float rawValueForZeroCurrent();
		static void initPins();
		static const char* versionString() { return BoardVersion::versionString(); }
		static void setLEDState(const bool state);
		static float capVoltageThresholdMilliVolts();
		static float readCapVoltageMilliVolts();
		static float readStackVoltage();
		static float readRawCurrent();
		static void disconnectLoadIfPossible();
		static void reconnectLoadIfPossible();
		static void setPurgeState(const bool state);
		static void setShortCircuitState(const bool state);
	};
	
	template <typename BoardVersion>
	float FCDKBoard<BoardVersion>::currentFromRawValue(const float rawCurrent)
	{
		const float currentSensorVoltage = float(BoardVersion::AREF) / 1024.0f * rawCurrent;
		return (currentSensorVoltage - float(BoardVersion::currentSensorPotential) / 2.0f ) / 185.0f; //185 mV per A
	}
	
	template <typename BoardVersion>
	float FCDKBoard<BoardVersion>::rawValueForZeroCurrent()
	{
		return 1024.0f * 2500.0f / float(BoardVersion::AREF); 
	}
	
	template <typename BoardVersion>
	void FCDKBoard<BoardVersion>::initPins()
	{
		if (BoardVersion::usesExternalReferenceVoltage) {
			analogReference(EXTERNAL);
		}
		pinMode(BoardVersion::statusLEDPin, OUTPUT);
		if (BoardVersion::canDisconnectLoad) {
			pinMode(BoardVersion::loadPin, OUTPUT);
			//connect the output so the caps can charge
			digitalWrite(BoardVersion::loadPin, BoardVersion::invertIfNeeded(HIGH));
		}
		if (BoardVersion::needsMOSFETChargePump) {
			analogWrite(BoardVersion::oscillatorPin, 128);
		}
		pinMode(BoardVersion::shortCircuitPin, OUTPUT);
		digitalWrite(BoardVersion::shortCircuitPin, BoardVersion::invertIfNeeded(LOW));
		pinMode(BoardVersion::purgePin, OUTPUT);
		digitalWrite(BoardVersion::purgePin, BoardVersion::invertIfNeeded(LOW));
	}
	
	template <typename BoardVersion>
	void FCDKBoard<BoardVersion>::setLEDState(const bool state)
	{
		digitalWrite(BoardVersion::statusLEDPin, state);
	}
	
	template <typename BoardVersion>
	float FCDKBoard<BoardVersion>::capVoltageThresholdMilliVolts()
	{
		return BoardVersion::capVoltageThreshold;
	}
	
	template <typename BoardVersion>
	float FCDKBoard<BoardVersion>::readCapVoltageMilliVolts()
	{
		return BoardVersion::capDivider() * float(BoardVersion::AREF) * analogRead(BoardVersion::capSensePin) / 1024.0f;
	}
	
	template <typename BoardVersion>
	float FCDKBoard<BoardVersion>::readStackVoltage()
	{
		return BoardVersion::stackDivider() * float(BoardVersion::AREF) * analogRead(BoardVersion::stackSensePin) / 1024.0f / 1000.0f; 
	}
	
	template <typename BoardVersion>
	float FCDKBoard<BoardVersion>::readRawCurrent()
	{
		return analogRead(BoardVersion::currentSensePin);
	}
	
	template <typename BoardVersion>
	void FCDKBoard<BoardVersion>::disconnectLoadIfPossible()
	{
		if (BoardVersion::canDisconnectLoad) {
			digitalWrite(BoardVersion::loadPin, BoardVersion::invertIfNeeded(LOW));
		}
	}
	
	template <typename BoardVersion>
	void FCDKBoard<BoardVersion>::reconnectLoadIfPossible()
	{
		if (BoardVersion::canDisconnectLoad) {
			digitalWrite(BoardVersion::loadPin, BoardVersion::invertIfNeeded(HIGH));
		}
	}
	
	template <typename BoardVersion>
	void FCDKBoard<BoardVersion>::setPurgeState(const bool state)
	{
		digitalWrite(BoardVersion::purgePin, BoardVersion::invertIfNeeded(state));
	}
	
	template <typename BoardVersion>
	void FCDKBoard<BoardVersion>::setShortCircuitState(const bool state)
	{
		digitalWrite(BoardVersion::shortCircuitPin, BoardVersion::invertIfNeeded(state));
	}
	
	//Constants for board versions
	//These do not inherit from a common base class because they are intended to be used for template meta-programming only.
	//Voltages are in mV.
	struct V1_3
	{
		static const int statusLEDPin = 6;
		static const int shortCircuitPin = 5;
		static const int purgePin = 3;
		static const int loadPin = 4;
		static const int oscillatorPin = 3;
		static const int capSensePin = A3;
		static const int stackSensePin = A1;
		static const int currentSensePin = A2;
		static const int AREF = 3300;
		static const bool usesExternalReferenceVoltage = true;
		static const bool canDisconnectLoad = false;
		static const bool needsMOSFETChargePump = false;
		static const int currentSensorPotential = 5000;
		static const int capVoltageThreshold = 3500;
		static float capDivider() { return 2.0f; }
		static float stackDivider() { return 97.00f/22.00f; } // R1=75k R2=22
		static const char* versionString() { return "1.3"; }
		static bool invertIfNeeded(const bool state) { return state; }
	};
	struct V1_2
	{
		static const int statusLEDPin = 6;
		static const int shortCircuitPin = 5;
		static const int purgePin = 3;
		static const int loadPin = 4;
		static const int oscillatorPin = 3;
		static const int capSensePin = A3;
		static const int stackSensePin = A1;
		static const int currentSensePin = A2;
		static const int AREF = 3300;
		static const bool usesExternalReferenceVoltage = true;
		static const bool canDisconnectLoad = true;
		static const bool needsMOSFETChargePump = false;
		static const int currentSensorPotential = 5020;
		static const int capVoltageThreshold = 3500;
		static float capDivider() { return 2.0f; }
		static float stackDivider() { return 2.0f; }
		static const char* versionString() { return "1.2"; }
		static bool invertIfNeeded(const bool state) { return state; }
	};
	struct V1_0for1_5to3W
	{
		static const int statusLEDPin = 5;
		static const int shortCircuitPin = 4;
		static const int purgePin = 2;
		static const int loadPin = 3;
		static const int oscillatorPin = 3;
		static const int capSensePin = A3;
		static const int stackSensePin = A1;
		static const int currentSensePin = A2;
		static const int AREF = 5000;
		static const bool usesExternalReferenceVoltage = false;
		static const bool canDisconnectLoad = true;
		static const bool needsMOSFETChargePump = false;
		static const int currentSensorPotential = 5020;
		static const int capVoltageThreshold = 3500;
		static float capDivider() { return 1.0f; }
		static float stackDivider() { return 1.0f; }
		static const char* versionString() { return "1.0 for 1.5-3W Stack"; }
		static bool invertIfNeeded(const bool state) { return state; }
	};
	struct V1_0for10to30W
	{
		static const int statusLEDPin = 5;
		static const int shortCircuitPin = 4;
		static const int purgePin = 2;
		static const int loadPin = 3;
		static const int oscillatorPin = 3;
		static const int capSensePin = A3;
		static const int stackSensePin = A1;
		static const int currentSensePin = A2;
		static const int AREF = 5000;
		static const bool usesExternalReferenceVoltage = false;
		static const bool canDisconnectLoad = false;
		static const bool needsMOSFETChargePump = true;
		static const int currentSensorPotential = 5000;
		static const int capVoltageThreshold = 3500;
		static float capDivider() { return 1.0f; }
		static float stackDivider() { return 326.00f/100.00f; }
		static const char* versionString() { return "1.0 for 10-30W Stack"; }
		static bool invertIfNeeded(const bool state) { return !state; }
	};
	
	/*Timing infor from Horizon:
	1.5W stack (info from Horizon)
	Purge: 100ms every 4 mins
	Short circuit: 100ms every 10s
	
	H-12 (no info from Horizon as they don't have H-12 controller)
	Purge: Shall we assume 50ms every 25s (because H2 consumption is less)
	Short circuit: 100ms every 10s (assuming same as other stacks)
	
	H-30 (info from Horizon)
	Purge: 50ms every 10s
	Short circuit: 100ms every 10s
	*/
	//Stack classes define default timings for a stack, minimum voltage and, are used for selection of default board.
	//These do not inherit from a common base class because they are intended to be used for template meta-programming only.
	//Voltages are in V.
	struct H30Stack
	{
		static const char* nameString() { return "H30 Stack"; }
		static StackTimings defaultTimings() { return StackTimings(10000,100,10000,100); }	
		static float minVoltage() { return 8.0f; }
		static float maxCurrent() { return 30.0f/minVoltage(); }
	};
	struct H12Stack
	{
		static const char* nameString() { return "H12 Stack"; }
		static StackTimings defaultTimings() { return StackTimings(10000,100,25000,100); }
		static float minVoltage() { return 7.5f; }
		static float maxCurrent() { return 12.0f/minVoltage(); }
	};
	struct MiniPakStack
	{
		static const char* nameString() { return "MiniPak Stack"; }
		static StackTimings defaultTimings() { return StackTimings(10000,100,60000,100); }
		static float minVoltage() { return 1.0f; }
		static float maxCurrent() { return 2.0f/minVoltage(); }
	};
	struct MiniPakStackX2
	{
		static const char* nameString() { return "Two MiniPak Stacks"; }
		static StackTimings defaultTimings() { return StackTimings(10000,100,60000,100); }
		static float minVoltage() { return 1.0f; }
		static float maxCurrent() { return 4.0f/minVoltage(); }
	};
	
	//meta-functions to define default board types for each stack type
	template <typename Stack> struct releaseBoard { };
	template <> struct releaseBoard<H30Stack> { typedef FCDKBoard<V1_3> type; };
	template <> struct releaseBoard<H12Stack> { typedef FCDKBoard<V1_3> type; };
	template <> struct releaseBoard<MiniPakStack> { typedef FCDKBoard<V1_2> type; };
	template <> struct releaseBoard<MiniPakStackX2> { typedef FCDKBoard<V1_2> type; };
	
	//Controller for a particular stack and board.
	//Board defaults to the distributed version of the board if omitted.
	//Call start() in Arduino start() and then call update() in Arduino loop().
	//Voltage is in V, current is in A.
	//This class implements the strategy pattern.
	//The control strategy is implemented by this class but, details of operations are delegated to the Board template argument.
	template <typename Stack, typename Board = typename releaseBoard<Stack>::type>
	class Controller
	{
		public:
			typedef Stack stack_type;
			typedef Board board_type;
			
			Controller(const StackTimings& stackTimings = Stack::defaultTimings(),
			           const BoardTimings& boardTimings = BoardTimings())
				:	_stackTimings(stackTimings),
					_boardTimings(boardTimings),
					_doPurge(true),
					_doShort(true),
					_smoothCurrent(true),
					_voltage(0.0f),
					_filteredRawCurrent(0.0f),
					_capVoltage(0.0f),
					_numPurges(0),
					_numShorts(0),
					_lastUpdate(0),
					_ledState(false),
					_blinkTimer(0),
					_statusTimer(0),
					_shortCircuitTimer(0),
					_purgeTimer(0)
				{ }
			
			//wait for capacitor to charge and start timers
			void start();
			//update timers and status, do purge and short if appropriate, print status to serial when appropriate.
			void update();
			
			float getVoltage() const { return _voltage; }
			float getCurrent() const { return Board::currentFromRawValue(_filteredRawCurrent); }
			unsigned int getNumPurges() const { return _numPurges; }
			unsigned int getNumShortCircuits() const { return _numShorts; }
			
			Controller& enablePurge();
			Controller& disablePurge();
			Controller& enableShortCircuit();
			Controller& disableShortCircuit();
			Controller& enableCurrentSmoothing();
			Controller& disableCurrentSmoothing();
			
			const StackTimings& getStackTimings() const { return _stackTimings; }
			Controller& setStackTimings(const StackTimings& newTimings);
			const BoardTimings& getBoardTimings() const { return _boardTimings; }
			Controller& setBoardTimings(const BoardTimings& newTimings) { _boardTimings = newTimings; return *this; }
		private:
			void _printStackTimings();
			void _waitCapCharge();
			void _updateElec();
			void _updateTimers();
			void _statusBySerial();
			void _purge();
			void _shortCircuit();
			
			StackTimings _stackTimings;
			BoardTimings _boardTimings;
			bool _doPurge;
			bool _doShort;
			bool _smoothCurrent;
			float _voltage;
			float _filteredRawCurrent;
			float _capVoltage; //millivolts
			unsigned int _numPurges;
			unsigned int _numShorts;
			unsigned long _lastUpdate;
			bool _ledState;
			unsigned long _blinkTimer;
			unsigned long _statusTimer;
			unsigned long _shortCircuitTimer;
			unsigned long _purgeTimer;
	};
	
	template <typename Stack, typename Board>
	void Controller<Stack,Board>::start()
	{
		_lastUpdate = millis();
		_blinkTimer = _statusTimer = _shortCircuitTimer = _purgeTimer = 0;
		_shortCircuitTimer += _stackTimings.getShortCircuitOffset();
		_numPurges = _numShorts = 0;
		_ledState = false;
		_filteredRawCurrent = Board::rawValueForZeroCurrent();
		Board::initPins();
		Serial.print("Arcola Energy fuel cell controller for ");
		Serial.print(Stack::nameString());
		Serial.print(" stack and FCDK board version ");
		Serial.print(Board::versionString());
		Serial.println(".");
		Serial.println("Stack timings:");
		_printStackTimings();
		Serial.println("Waiting for capacitor to charge... Ensure fuel cell stack is operational.");
		_waitCapCharge();
		Serial.println("Starting.");
		//double blink
		Board::setLEDState(true);
		delay(2*_boardTimings.getBlinkPeriod());
		Board::setLEDState(false);
		delay(_boardTimings.getBlinkPeriod());
		Board::setLEDState(true);
		delay(2*_boardTimings.getBlinkPeriod());
		Board::setLEDState(false);
	}
	
	template <typename Stack, typename Board>
	void Controller<Stack,Board>::update()
	{
		_updateTimers();
		if (_blinkTimer > _boardTimings.getBlinkPeriod()) {
			_blinkTimer = 0;
			Board::setLEDState(_ledState = !_ledState);
		}
		if (_statusTimer > _boardTimings.getElectricUpdatePeriod()) {
			_statusTimer = 0;
			_updateElec();
			_statusBySerial();
		}
		if (_shortCircuitTimer > _stackTimings.getShortCircuitPeriod()) {
			_shortCircuitTimer = 0;
			_shortCircuit();
		}
		if (_purgeTimer > _stackTimings.getPurgePeriod()) {
			_purgeTimer = 0;
			_purge();
		}
	}
	
	template <typename Stack, typename Board>
	Controller<Stack,Board>& Controller<Stack,Board>::enablePurge()
	{
		if (_doPurge) {
			return *this;
		} else {
			Serial.println("Enabling purging.");
			_doPurge = true;
			return *this;
		}
	}
	
	template <typename Stack, typename Board>
	Controller<Stack,Board>& Controller<Stack,Board>::disablePurge()
	{
		if (!_doPurge) {
			return *this;
		} else {
			Serial.println("Disabling purging.");
			_doPurge = false;
			return *this;
		}
	}
	
	template <typename Stack, typename Board>
	Controller<Stack,Board>& Controller<Stack,Board>::enableShortCircuit()
	{
		if (_doShort) {
			return *this;
		} else {
			Serial.println("Enabling short-circuiting.");
			_doShort = true;
			return *this;
		}
	}
	
	template <typename Stack, typename Board>
	Controller<Stack,Board>& Controller<Stack,Board>::disableShortCircuit()
	{
		if (!_doShort) {
			return *this;
		} else {
			Serial.println("Disabling short-circuiting.");
			_doShort = false;
			return *this;
		}
	}
	
	template <typename Stack, typename Board>
	Controller<Stack,Board>& Controller<Stack,Board>::enableCurrentSmoothing()
	{
		if (_smoothCurrent) {
			return *this;
		} else {
			Serial.println("Enabling current measurement smoothing.");
			_smoothCurrent = true;
			return *this;
		}
	}
	
	template <typename Stack, typename Board>
	Controller<Stack,Board>& Controller<Stack,Board>::disableCurrentSmoothing()
	{
		if (!_smoothCurrent) {
			return *this;
		} else {
			Serial.println("Disabling current measurement smoothing.");
			_smoothCurrent = false;
			return *this;
		}
	}
	
	template <typename Stack, typename Board>
	Controller<Stack,Board>& Controller<Stack,Board>::setStackTimings(const StackTimings& newTimings)
	{
		_stackTimings = newTimings;
		Serial.println("Updated stack timings. New timings:");
		_printStackTimings();
		return *this;
	}
	
	template <typename Stack, typename Board>
	void Controller<Stack,Board>::_printStackTimings()
	{
		Serial.print("Short-circuit for ");
		Serial.print(_stackTimings.getShortCircuitLength());
		Serial.print("ms every ");
		Serial.print(_stackTimings.getShortCircuitPeriod()/1000);
		Serial.println(" seconds.");
		Serial.print("Purge for ");
		Serial.print(_stackTimings.getPurgeLength());
		Serial.print("ms every ");
		Serial.print(_stackTimings.getPurgePeriod()/1000);
		Serial.println(" seconds.");
	}
	
	template <typename Stack, typename Board>
	void Controller<Stack,Board>::_waitCapCharge()
	{
		while(_capVoltage < Board::capVoltageThresholdMilliVolts()) {
			_updateElec();
			Serial.print("Capacitor voltage: ");
			Serial.print(_capVoltage);
			Serial.println("mV");
			Board::setLEDState(true);
			delay(_boardTimings.getBlinkPeriod());
			Board::setLEDState(false);
			delay(_boardTimings.getBlinkPeriod());
		}
		Serial.println("Capacitor charged.");
	}
	
	template <typename Stack, typename Board>
	void Controller<Stack,Board>::_updateElec()
	{
		_capVoltage = Board::readCapVoltageMilliVolts();
		_voltage = Board::readStackVoltage();
		if (_smoothCurrent) {
			_filteredRawCurrent = _filteredRawCurrent*(1.0f-_boardTimings.getCurrentLPFCoeff())
			                     +_boardTimings.getCurrentLPFCoeff()*float(Board::readRawCurrent());
		} else {
			_filteredRawCurrent = Board::readRawCurrent();
		}
		if (_voltage < Stack::minVoltage()) {
			Serial.println("WARNING! Stack voltage is too low!");
		}
	}
	
	template <typename Stack, typename Board>
	void Controller<Stack,Board>::_updateTimers()
	{
		const unsigned long now = millis();
		const unsigned long interval = now - _lastUpdate;
		_lastUpdate = now;
		_blinkTimer += interval;
		_statusTimer += interval;
		_shortCircuitTimer += interval;
		_purgeTimer += interval;
	}
	
	template <typename Stack, typename Board>
	void Controller<Stack,Board>::_statusBySerial()
	{
		Serial.print(getVoltage());
		Serial.print("V, ");
		Serial.print(getCurrent());
		Serial.println("A");
	}
	
	template <typename Stack, typename Board>
	void Controller<Stack,Board>::_purge()
	{
		if (!_doPurge) {
			Serial.println("Skipping purge as purging is disabled.");
			return;
		}
		Serial.println("PURGE");
		Board::disconnectLoadIfPossible();
		Board::setPurgeState(true);
		delay(_stackTimings.getPurgeLength());
		Board::setPurgeState(false);
		Board::reconnectLoadIfPossible();
		++_numPurges;
	}
	
	template <typename Stack, typename Board>
	void Controller<Stack,Board>::_shortCircuit()
	{
		if (!_doShort) {
			Serial.println("Skipping short circuit as short-circuiting is disabled.");
			return;
		}
		if (_capVoltage < Board::capVoltageThresholdMilliVolts()) {
			Serial.println("Skipping short circuit as the capacitor voltage is too low.");
			return;
		}
		Serial.println("SHORT-CIRCUIT");
		Board::disconnectLoadIfPossible();
		Board::setShortCircuitState(true);
		delay(_stackTimings.getShortCircuitLength());
		Board::setShortCircuitState(false);
		Board::reconnectLoadIfPossible();
		++_numShorts;
	}
	
}
#endif //ndef fcdk_h