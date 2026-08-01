#pragma once

#include <Arduino.h>
#include <ModbusMaster.h>

enum Modbus_Register_Type {HOLDING_REGISTER, INPUT_REG};
enum Modbus_Pin_State {RX_MODE, TX_MODE, INIT};

class Modbus_Manager
{
  private:
    ModbusMaster _node;
    uint8_t _code = 0;

    static void _manage_pins(int8_t de_pin, int8_t re_pin, Modbus_Pin_State state);
    static void _pre_tx();
    static void _post_tx();

    uint8_t _core_read(uint16_t address, uint8_t num_words, Modbus_Register_Type register_type);

  public:
    Modbus_Manager();

    // Initialize the Modbus Manager
    void begin(uint8_t slave_id, Stream& serial_port, int8_t de_pin = -1, int8_t re_pin = -1);
    
    // Read 16-bit unsigned integer
    bool read(uint16_t address, uint16_t &result, Modbus_Register_Type register_type = HOLDING_REGISTER);
    // Read 16-bit signed integer
    bool read(uint16_t address, int16_t &result, Modbus_Register_Type register_type = HOLDING_REGISTER);
    // Read 32-bit unsigned integer
    bool read(uint16_t address, uint32_t &result, Modbus_Register_Type register_type = HOLDING_REGISTER, bool swap_words = false);
    // Read 32-bit signed integer
    bool read(uint16_t address, int32_t &result, Modbus_Register_Type register_type = HOLDING_REGISTER, bool swap_words = false);
    // Read 32-bit float
    bool read(uint16_t address, float &result, Modbus_Register_Type register_type = HOLDING_REGISTER, bool swap_words = false);

    // Write 16-bit unsigned integer
    bool write(uint16_t address, uint16_t value);
    // Write 16-bit signed integer
    bool write(uint16_t address, int16_t value);
    // Write 32-bit unsigned integer
    bool write(uint16_t address, uint32_t value, bool swap_words = false);
    // Write 32-bit signed integer
    bool write(uint16_t address, int32_t value, bool swap_words = false);
    // Write 32-bit float
    bool write(uint16_t address, float value, bool swap_words = false);

    // Get the code from the Modbus transaction
    uint8_t get_code();

    // Get Modbus Master
    ModbusMaster &get_master();
};