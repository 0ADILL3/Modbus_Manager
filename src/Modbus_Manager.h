#pragma once

#include <Arduino.h>
#include <ModbusMaster.h>

/**
 * @brief Jenis register Modbus yang akan diakses untuk operasi pembacaan {HOLDING_REGISTER, INPUT_REGISTER}.
 */
enum Modbus_Register_Type {HOLDING_REGISTER, INPUT_REGISTER};

class Modbus_Manager
{
  private:
    ModbusMaster node_;
    uint8_t code_ = 0;

    int8_t de_pin_ = -1;
    int8_t re_pin_ = -1;

    static Modbus_Manager* active_instance_;

    static void pre_tx_();
    static void post_tx_();

    uint8_t core_read_(uint16_t address, uint8_t num_words, Modbus_Register_Type register_type);

public:
    /**
     * @brief Konstruktor default untuk Modbus_Manager.
     */
    Modbus_Manager();

    /**
     * @brief Menginisialisasi komunikasi Modbus, port serial, dan pin kontrol RS-485.
     * 
     * @param slave_id ID Modbus dari perangkat slave tujuan (1-247).
     * @param serial_port Referensi ke objek Stream serial (misal: Serial, Serial1, SoftwareSerial).
     * @param de_pin Nomor pin untuk Driver Enable (DE). Biarkan -1 jika tidak digunakan.
     * @param re_pin Nomor pin untuk Receiver Enable (RE). Biarkan -1 jika tidak digunakan.
     */
    void begin(uint8_t slave_id, Stream& serial_port, int8_t de_pin = -1, int8_t re_pin = -1);
    
    /**
     * @brief Membaca data 16-bit unsigned integer (uint16_t) dari sebuah register.
     * 
     * @param address Alamat register awal.
     * @param result Variabel referensi untuk menyimpan hasil pembacaan.
     * @param register_type Jenis register yang dibaca (HOLDING_REGISTER atau INPUT_REGISTER). Default: HOLDING_REGISTER.
     * @return true Jika pembacaan berhasil.
     * @return false Jika pembacaan gagal. Gunakan get_code() untuk melihat detail error.
     */
    bool read(uint16_t address, uint16_t &result, Modbus_Register_Type register_type = HOLDING_REGISTER);

    /**
     * @brief Membaca data 16-bit signed integer (int16_t) dari sebuah register.
     * 
     * @param address Alamat register awal.
     * @param result Variabel referensi untuk menyimpan hasil pembacaan.
     * @param register_type Jenis register yang dibaca (HOLDING_REGISTER atau INPUT_REGISTER). Default: HOLDING_REGISTER.
     * @return true Jika pembacaan berhasil.
     * @return false Jika pembacaan gagal.
     */
    bool read(uint16_t address, int16_t &result, Modbus_Register_Type register_type = HOLDING_REGISTER);

    /**
     * @brief Membaca data 32-bit unsigned integer (uint32_t) dari dua register berurutan.
     * 
     * @param address Alamat register awal.
     * @param result Variabel referensi untuk menyimpan hasil pembacaan.
     * @param register_type Jenis register yang dibaca. Default: HOLDING_REGISTER.
     * @param swap_words Jika true, posisi high-word dan low-word akan ditukar (endianness). Default: false.
     * @return true Jika pembacaan berhasil.
     * @return false Jika pembacaan gagal.
     */
    bool read(uint16_t address, uint32_t &result, Modbus_Register_Type register_type = HOLDING_REGISTER, bool swap_words = false);

    /**
     * @brief Membaca data 32-bit signed integer (int32_t) dari dua register berurutan.
     * 
     * @param address Alamat register awal.
     * @param result Variabel referensi untuk menyimpan hasil pembacaan.
     * @param register_type Jenis register yang dibaca. Default: HOLDING_REGISTER.
     * @param swap_words Jika true, posisi high-word dan low-word akan ditukar. Default: false.
     * @return true Jika pembacaan berhasil.
     * @return false Jika pembacaan gagal.
     */
    bool read(uint16_t address, int32_t &result, Modbus_Register_Type register_type = HOLDING_REGISTER, bool swap_words = false);

    /**
     * @brief Membaca data 32-bit floating point (float) dari dua register berurutan.
     * 
     * @param address Alamat register awal.
     * @param result Variabel referensi untuk menyimpan hasil pembacaan.
     * @param register_type Jenis register yang dibaca. Default: HOLDING_REGISTER.
     * @param swap_words Jika true, posisi high-word dan low-word akan ditukar. Default: false.
     * @return true Jika pembacaan berhasil.
     * @return false Jika pembacaan gagal.
     */
    bool read(uint16_t address, float &result, Modbus_Register_Type register_type = HOLDING_REGISTER, bool swap_words = false);

    /**
     * @brief Menulis nilai 16-bit unsigned integer ke dalam sebuah Holding Register.
     * 
     * @param address Alamat register tujuan.
     * @param value Nilai yang akan ditulis.
     * @return true Jika penulisan berhasil.
     * @return false Jika penulisan gagal.
     */
    bool write(uint16_t address, uint16_t value);

    /**
     * @brief Menulis nilai 16-bit signed integer ke dalam sebuah Holding Register.
     * 
     * @param address Alamat register tujuan.
     * @param value Nilai yang akan ditulis.
     * @return true Jika penulisan berhasil.
     * @return false Jika penulisan gagal.
     */
    bool write(uint16_t address, int16_t value);

    /**
     * @brief Menulis nilai 32-bit unsigned integer ke dalam dua Holding Register berurutan.
     * 
     * @param address Alamat register awal.
     * @param value Nilai yang akan ditulis.
     * @param swap_words Jika true, urutan pengiriman high-word dan low-word akan ditukar. Default: false.
     * @return true Jika penulisan berhasil.
     * @return false Jika penulisan gagal.
     */
    bool write(uint16_t address, uint32_t value, bool swap_words = false);

    /**
     * @brief Menulis nilai 32-bit signed integer ke dalam dua Holding Register berurutan.
     * 
     * @param address Alamat register awal.
     * @param value Nilai yang akan ditulis.
     * @param swap_words Jika true, urutan pengiriman high-word dan low-word akan ditukar. Default: false.
     * @return true Jika penulisan berhasil.
     * @return false Jika penulisan gagal.
     */
    bool write(uint16_t address, int32_t value, bool swap_words = false);

    /**
     * @brief Menulis nilai 32-bit floating point ke dalam dua Holding Register berurutan.
     * 
     * @param address Alamat register awal.
     * @param value Nilai yang akan ditulis.
     * @param swap_words Jika true, urutan pengiriman high-word dan low-word akan ditukar. Default: false.
     * @return true Jika penulisan berhasil.
     * @return false Jika penulisan gagal.
     */
    bool write(uint16_t address, float value, bool swap_words = false);

    /**
     * @brief Mengambil kode status hasil dari transaksi baca/tulis Modbus terakhir.
     * 
     * @return uint8_t Kode dari library ModbusMaster (contoh: 0x00 untuk ku8MBSuccess).
     */
    uint8_t get_code() const;

    /**
     * @brief Mengambil referensi dari objek ModbusMaster dasar jika interaksi langsung diperlukan.
     * 
     * @return ModbusMaster& Referensi ke instansi objek ModbusMaster internal.
     */
    ModbusMaster &get_master();
};