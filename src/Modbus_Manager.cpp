#include "Modbus_Manager.h"

Modbus_Manager::Modbus_Manager() {}

void Modbus_Manager::_manage_pins(int8_t de_pin, int8_t re_pin, Modbus_Pin_State state)
{
  static int8_t _de_pin = -1;
  static int8_t _re_pin = -1;

  if (state == INIT)
  {
    _de_pin = de_pin;
    _re_pin = re_pin;
    return;
  }

  if (state == TX_MODE)
  {
    if (_re_pin >= 0) digitalWrite(_re_pin, HIGH);
    if (_de_pin >= 0) digitalWrite(_de_pin, HIGH);
  }
  else if (state == RX_MODE)
  {
    if (_re_pin >= 0) digitalWrite(_re_pin, LOW);
    if (_de_pin >= 0) digitalWrite(_de_pin, LOW);
  }
}

void Modbus_Manager::_pre_tx()  {_manage_pins(-1, -1, TX_MODE);}
void Modbus_Manager::_post_tx() {_manage_pins(-1, -1, RX_MODE);}

void Modbus_Manager::begin(uint8_t slave_id, Stream& serial_port, int8_t de_pin, int8_t re_pin)
{
  _node.begin(slave_id, serial_port);

  if (de_pin >= 0 || re_pin >= 0)
  {
    _manage_pins(de_pin, re_pin, INIT);

    if (de_pin >= 0)
    {
      pinMode(de_pin, OUTPUT);
      digitalWrite(de_pin, LOW);
    }
    if (re_pin >= 0)
    {
      pinMode(re_pin, OUTPUT);
      digitalWrite(re_pin, LOW);
    }

    _node.preTransmission(_pre_tx);
    _node.postTransmission(_post_tx);
  }
}

uint8_t Modbus_Manager::_core_read(uint16_t address, uint8_t num_words, Modbus_Register_Type register_type)
{
  _code = (register_type == HOLDING_REGISTER) ? _node.readHoldingRegisters(address, num_words) : _node.readInputRegisters(address, num_words);
  return _code;
}

bool Modbus_Manager::read(uint16_t address, uint16_t &result, Modbus_Register_Type register_type)
{
  if (_core_read(address, 1, register_type) == _node.ku8MBSuccess) 
  {
    result = _node.getResponseBuffer(0);
  }
  return (_code == _node.ku8MBSuccess);
}

bool Modbus_Manager::read(uint16_t address, int16_t &result, Modbus_Register_Type register_type)
{
  if (_core_read(address, 1, register_type) == _node.ku8MBSuccess) 
  {
    result = (int16_t)_node.getResponseBuffer(0);
  }
  return (_code == _node.ku8MBSuccess);
}

bool Modbus_Manager::read(uint16_t address, uint32_t &result, Modbus_Register_Type register_type, bool swap_words)
{
  if (_core_read(address, 2, register_type) == _node.ku8MBSuccess) 
  {
    uint16_t high_word = swap_words ? _node.getResponseBuffer(1) : _node.getResponseBuffer(0);
    uint16_t low_word  = swap_words ? _node.getResponseBuffer(0) : _node.getResponseBuffer(1);
    
    result = ((uint32_t)high_word << 16) | low_word;
  }
  return (_code == _node.ku8MBSuccess);
}

bool Modbus_Manager::read(uint16_t address, int32_t &result, Modbus_Register_Type register_type, bool swap_words)
{
  if (_core_read(address, 2, register_type) == _node.ku8MBSuccess) 
  {
    uint16_t high_word = swap_words ? _node.getResponseBuffer(1) : _node.getResponseBuffer(0);
    uint16_t low_word  = swap_words ? _node.getResponseBuffer(0) : _node.getResponseBuffer(1);
    
    uint32_t val = ((uint32_t)high_word << 16) | low_word;
    memcpy(&result, &val, sizeof(result));
  }
  return (_code == _node.ku8MBSuccess);
}

bool Modbus_Manager::read(uint16_t address, float &result, Modbus_Register_Type register_type, bool swap_words)
{
  if (_core_read(address, 2, register_type) == _node.ku8MBSuccess) 
  {
    uint16_t high_word = swap_words ? _node.getResponseBuffer(1) : _node.getResponseBuffer(0);
    uint16_t low_word  = swap_words ? _node.getResponseBuffer(0) : _node.getResponseBuffer(1);
    
    uint32_t val = ((uint32_t)high_word << 16) | low_word;
    memcpy(&result, &val, sizeof(result));
  }
  return (_code == _node.ku8MBSuccess);
}

bool Modbus_Manager::write(uint16_t address, uint16_t value)
{
  _code = _node.writeSingleRegister(address, value);
  return (_code == _node.ku8MBSuccess);
}

bool Modbus_Manager::write(uint16_t address, int16_t value)
{
  _code = _node.writeSingleRegister(address, (uint16_t)value);
  return (_code == _node.ku8MBSuccess);
}

bool Modbus_Manager::write(uint16_t address, uint32_t value, bool swap_words)
{
  uint16_t high_word = (value >> 16) & 0xFFFF;
  uint16_t low_word  = value & 0xFFFF;
  
  _node.setTransmitBuffer(swap_words ? 1 : 0, high_word);
  _node.setTransmitBuffer(swap_words ? 0 : 1, low_word);
  
  _code = _node.writeMultipleRegisters(address, 2);
  return (_code == _node.ku8MBSuccess);
}

bool Modbus_Manager::write(uint16_t address, int32_t value, bool swap_words)
{
  uint32_t val;
  memcpy(&val, &value, sizeof(val));
  return write(address, val, swap_words); 
}

bool Modbus_Manager::write(uint16_t address, float value, bool swap_words)
{
  uint32_t val;
  memcpy(&val, &value, sizeof(val));
  return write(address, val, swap_words); 
}

uint8_t Modbus_Manager::get_code() {return _code;}

ModbusMaster &Modbus_Manager::get_master() {return _node;}