#include "Arduino.h"
#include "SerialControl.h"

SerialControl::SerialControl(String in){
	_maxIndex = in.length() - 1;
	_input = in;
}

String SerialControl::getValue(char separator, int stIndex){
	for (int i = stIndex; i <= _maxIndex; i++) {
		if (_input.charAt(i) == separator || i == _maxIndex) {
			if(i == _maxIndex){
				i++;
			}
			return _input.substring(stIndex, i);
		}
	}
}

int SerialControl::getSpeed(){
	if(_input.charAt(0) != 't'){
		return -1;
	}
	else{
		return _input.substring(1, _input.indexOf('s')).toInt();
	}
}

int SerialControl::getAngle(){
	for (int i = 0; i <= _maxIndex; i++){
		if(_input.charAt(i) == 't'){
			if(_input.indexOf('t', i) < _maxIndex){
				return _input.substring(i, _input.indexOf('t')).toInt();
			}
			else{
				return _input.substring(i, _maxIndex).toInt();
			}
		}
	}
}