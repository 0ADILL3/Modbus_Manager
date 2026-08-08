#include "Modbus_Manager.h"

Modbus_Manager::Modbus_Manager() {}

Modbus_Manager* Modbus_Manager::active_instance_ = nullptr;

void Modbus_Manager::pre_tx_()
{
  if (active_instance_)
  {
    if (active_instance_->re_pin_ >= 0) {digitalWrite(active_instance_->re_pin_, HIGH);}
    if (active_instance_->de_pin_ >= 0) {digitalWrite(active_instance_->de_pin_, HIGH);}
  }
}
void Modbus_Manager::post_tx_()
{
  if (active_instance_)
  {
    if (active_instance_->re_pin_ >= 0) {digitalWrite(active_instance_->re_pin_, LOW);}
    if (active_instance_->de_pin_ >= 0) {digitalWrite(active_instance_->de_pin_, LOW);}
  }
}

void Modbus_Manager::begin(uint8_t slave_id, Stream& serial_port, int8_t de_pin, int8_t re_pin)
{
  node_.begin(slave_id, serial_port);
  
  de_pin_ = de_pin;
  re_pin_ = re_pin;

  if (de_pin_ >= 0)
  {
    pinMode(de_pin_, OUTPUT);
    digitalWrite(de_pin_, LOW);
  }
  if (re_pin_ >= 0 && re_pin_ != de_pin_)
  {
    pinMode(re_pin_, OUTPUT);
    digitalWrite(re_pin_, LOW);
  }

  if (de_pin_ >= 0 || re_pin_ >= 0)
  {
    node_.preTransmission(pre_tx_);
    node_.postTransmission(post_tx_);
  }
}

uint8_t Modbus_Manager::core_read_(uint16_t address, uint8_t num_words, Modbus_Register_Type register_type)
{
  active_instance_ = this;

  code_ = (register_type == HOLDING_REGISTER) ? node_.readHoldingRegisters(address, num_words) : node_.readInputRegisters(address, num_words);
  return code_;
}

bool Modbus_Manager::read(uint16_t address, uint16_t &result, Modbus_Register_Type register_type)
{
  if (core_read_(address, 1, register_type) == node_.ku8MBSuccess) 
  {
    result = node_.getResponseBuffer(0);
  }
  return (code_ == node_.ku8MBSuccess);
}

bool Modbus_Manager::read(uint16_t address, int16_t &result, Modbus_Register_Type register_type)
{
  if (core_read_(address, 1, register_type) == node_.ku8MBSuccess) 
  {
    result = (int16_t)node_.getResponseBuffer(0);
  }
  return (code_ == node_.ku8MBSuccess);
}

bool Modbus_Manager::read(uint16_t address, uint32_t &result, Modbus_Register_Type register_type, bool swap_words)
{
  if (core_read_(address, 2, register_type) == node_.ku8MBSuccess) 
  {
    uint16_t high_word = swap_words ? node_.getResponseBuffer(1) : node_.getResponseBuffer(0);
    uint16_t low_word  = swap_words ? node_.getResponseBuffer(0) : node_.getResponseBuffer(1);
    
    result = ((uint32_t)high_word << 16) | low_word;
  }
  return (code_ == node_.ku8MBSuccess);
}

bool Modbus_Manager::read(uint16_t address, int32_t &result, Modbus_Register_Type register_type, bool swap_words)
{
  if (core_read_(address, 2, register_type) == node_.ku8MBSuccess) 
  {
    uint16_t high_word = swap_words ? node_.getResponseBuffer(1) : node_.getResponseBuffer(0);
    uint16_t low_word  = swap_words ? node_.getResponseBuffer(0) : node_.getResponseBuffer(1);
    
    uint32_t val = ((uint32_t)high_word << 16) | low_word;
    memcpy(&result, &val, sizeof(result));
  }
  return (code_ == node_.ku8MBSuccess);
}

bool Modbus_Manager::read(uint16_t address, float &result, Modbus_Register_Type register_type, bool swap_words)
{
  if (core_read_(address, 2, register_type) == node_.ku8MBSuccess) 
  {
    uint16_t high_word = swap_words ? node_.getResponseBuffer(1) : node_.getResponseBuffer(0);
    uint16_t low_word  = swap_words ? node_.getResponseBuffer(0) : node_.getResponseBuffer(1);
    
    uint32_t val = ((uint32_t)high_word << 16) | low_word;
    memcpy(&result, &val, sizeof(result));
  }
  return (code_ == node_.ku8MBSuccess);
}

bool Modbus_Manager::write(uint16_t address, uint16_t value)
{
  active_instance_ = this;

  code_ = node_.writeSingleRegister(address, value);
  return (code_ == node_.ku8MBSuccess);
}

bool Modbus_Manager::write(uint16_t address, int16_t value)
{
  return write(address, (uint16_t)value);
}

bool Modbus_Manager::write(uint16_t address, uint32_t value, bool swap_words)
{
  active_instance_ = this;

  uint16_t high_word = (value >> 16) & 0xFFFF;
  uint16_t low_word  = value & 0xFFFF;
  
  node_.setTransmitBuffer(swap_words ? 1 : 0, high_word);
  node_.setTransmitBuffer(swap_words ? 0 : 1, low_word);
  
  code_ = node_.writeMultipleRegisters(address, 2);
  return (code_ == node_.ku8MBSuccess);
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

uint8_t Modbus_Manager::get_code() const {return code_;}

ModbusMaster &Modbus_Manager::get_master() {return node_;}