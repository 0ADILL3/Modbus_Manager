/*
 * CONTOH PENGGUNAAN MODBUS MANAGER UNTUK PZEM-017
 * 
 * Sketsa ini mendemonstrasikan cara membaca register 16-bit dan 32-bit 
 * dari modul PZEM-017 (DC Energy Meter). Sketsa ini juga dilengkapi 
 * dengan logika fail-safe (Error Counter) jika kabel RS485 terputus.
 */

#include <Modbus_Manager.h>

// Buat instance Modbus_Manager
Modbus_Manager pzem;

// Variabel data (Raw Data dari alat)
uint16_t raw_voltage;
uint16_t raw_current;
uint32_t raw_power;

// Variabel untuk melacak status koneksi
uint8_t error_counter = 0;
const uint8_t MAX_FAILURES = 3; 
bool is_connected = false;

void setup() {
  Serial.begin(115200);
  
  // Inisialisasi Serial Modbus (Contoh menggunakan ESP32 Pin 16 & 17)
  Serial1.begin(9600, SERIAL_8N1, 16, 17);
  
  // Mulai Modbus Manager (Slave ID 1, Serial1, Pin DE/RE di Pin 4)
  pzem.begin(1, Serial1, 4);

  Serial.println("Memulai Pembacaan PZEM-017...");
}

void loop() {
  // PZEM-017 menggunakan INPUT REGISTER (Bukan Holding Register)
  // Alamat 0x0000 = Voltage (1 word)
  // Alamat 0x0001 = Current (1 word)
  // Alamat 0x0002 = Power (2 words - 32 bit!)

  // 1. Eksekusi Pembacaan menggunakan fungsi read() yang mengembalikan tipe boolean
  bool success_v = pzem.read(0x0000, raw_voltage, INPUT_REG);
  bool success_i = pzem.read(0x0001, raw_current, INPUT_REG);
  bool success_p = pzem.read(0x0002, raw_power, INPUT_REG); // Otomatis overload ke uint32_t

  // 2. Evaluasi Hasil Pembacaan
  if (success_v && success_i && success_p) {
    // --- JIKA SUKSES ---
    error_counter = 0; 
    is_connected = true;

    // Terapkan Skala (PZEM-017 memiliki aturan skala pembagian pada manualnya)
    float volt_aktual = raw_voltage * 0.01; // Skala 0.01 V
    float amp_aktual = raw_current * 0.01;  // Skala 0.01 A
    float watt_aktual = raw_power * 0.1;    // Skala 0.1 W

    Serial.println("--- DATA PZEM-017 ---");
    Serial.print("Tegangan : "); Serial.print(volt_aktual); Serial.println(" V");
    Serial.print("Arus     : "); Serial.print(amp_aktual); Serial.println(" A");
    Serial.print("Daya     : "); Serial.print(watt_aktual); Serial.println(" W");
    Serial.println("---------------------");
    
  } else {
    // --- JIKA GAGAL ---
    error_counter++; 
    
    // Cek error spesifik dari Library ModbusMaster
    if (pzem.get_code() == pzem.get_master().ku8MBResponseTimedOut) {
      Serial.println("Warning: Timeout! PZEM-017 tidak merespons.");
    } else {
      Serial.print("Warning: Data korup (Noise). Kode Error: 0x");
      Serial.println(pzem.get_code(), HEX);
    }

    // Eksekusi Logika Fail-Safe jika putus koneksi total
    if (error_counter >= MAX_FAILURES) {
      is_connected = false;
      error_counter = MAX_FAILURES; 
      
      // Reset data agar sistem automasi tidak membaca angka terakhir saat alat mati
      raw_voltage = 0;
      raw_current = 0;
      raw_power = 0;
      
      Serial.println("ALARM: KONEKSI PZEM TERPUTUS!");
    }
  }

  // Polling setiap 1 detik
  delay(1000); 
}