#include "ModbusMasterUtils.h"
#include <Arduino.h>

void ModbusMasterUtils::begin(uint8_t slaveId, HardwareSerial &serial, uint32_t baud) {
    _serial = &serial;
    _serial->begin(baud);
    _modbus.begin(slaveId, serial);
    _errorCount = 0;
    _lastCommTime = 0;
    
    // Allow hardware to settle
    delay(100);
}

void ModbusMasterUtils::ensureInterFrameDelay() {
    uint32_t elapsed = millis() - _lastCommTime;
    if (elapsed < INTER_FRAME_DELAY) {
        delay(INTER_FRAME_DELAY - elapsed);
    }
}

void ModbusMasterUtils::clearSerialBuffer() {
    if (_serial) {
        while (_serial->available()) {
            _serial->read();
        }
    }
}

void ModbusMasterUtils::recordSuccess() {
    _errorCount = 0;
    _lastCommTime = millis();
}

void ModbusMasterUtils::recordError() {
    if (_errorCount < 255) {
        _errorCount++;
    }
}

void ModbusMasterUtils::resetErrors() {
    _errorCount = 0;
    clearSerialBuffer();
}

bool ModbusMasterUtils::writeFloat(uint16_t startReg, float value) {
    union { float f; uint16_t u16[2]; } data;
    data.f = value;
    
    for (uint8_t retry = 0; retry < _maxRetries; retry++) {
        ensureInterFrameDelay();
        
        _modbus.setTransmitBuffer(0, data.u16[0]); // Low word
        _modbus.setTransmitBuffer(1, data.u16[1]); // High word
        
        uint8_t result = _modbus.writeMultipleRegisters(startReg, 2);
        
        if (result == _modbus.ku8MBSuccess) {
            recordSuccess();
            return true;
        }
        
        delay(_retryDelay);
        clearSerialBuffer();
    }
    
    recordError();
    return false;
}

float ModbusMasterUtils::readFloat(uint16_t startReg) {
    for (uint8_t retry = 0; retry < _maxRetries; retry++) {
        ensureInterFrameDelay();
        
        uint8_t result = _modbus.readHoldingRegisters(startReg, 2);
        
        if (result == _modbus.ku8MBSuccess) {
            union { float f; uint16_t u16[2]; } data;
            data.u16[0] = _modbus.getResponseBuffer(0);
            data.u16[1] = _modbus.getResponseBuffer(1);
            recordSuccess();
            return data.f;
        }
        
        delay(_retryDelay);
        clearSerialBuffer();
    }
    
    recordError();
    return NAN;
}

bool ModbusMasterUtils::writeRegister(uint16_t reg, uint16_t value) {
    for (uint8_t retry = 0; retry < _maxRetries; retry++) {
        ensureInterFrameDelay();
        
        uint8_t result = _modbus.writeSingleRegister(reg, value);
        
        if (result == _modbus.ku8MBSuccess) {
            recordSuccess();
            return true;
        }
        
        delay(_retryDelay);
        clearSerialBuffer();
    }
    
    recordError();
    return false;
}

bool ModbusMasterUtils::readRegister(uint16_t reg, uint16_t *value) {
    for (uint8_t retry = 0; retry < _maxRetries; retry++) {
        ensureInterFrameDelay();
        
        uint8_t result = _modbus.readHoldingRegisters(reg, 1);
        
        if (result == _modbus.ku8MBSuccess) {
            *value = _modbus.getResponseBuffer(0);
            recordSuccess();
            return true;
        }
        
        delay(_retryDelay);
        clearSerialBuffer();
    }
    
    recordError();
    return false;
}
