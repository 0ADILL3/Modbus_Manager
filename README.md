# Modbus_Manager Library

Modbus Manager adalah *wrapper library* untuk Arduino yang dibangun di atas [ModbusMaster](https://github.com/4-20ma/ModbusMaster). Library ini dirancang untuk menyelesaikan masalah umum pada Modbus RTU, seperti manipulasi memori (*bit-shifting*) pada data 32-bit (Float/Integer) dan manajemen pin Transceiver RS485 (DE/RE).

## ✨ Fitur Utama

* **Type-Safe API via Overloading:** Tidak perlu lagi memanggil nama fungsi yang rumit. Cukup gunakan `read()` atau `write()`, dan kompiler akan otomatis menyesuaikan *buffer* untuk tipe data `uint16_t`, `int16_t`, `uint32_t`, `int32_t`, maupun `float`.
* **Memori Aman:** Menggunakan `memcpy` untuk operasi data 32-bit untuk mencegah kerentanan *Strict Aliasing* di C++.
* **Auto RS485 Pin Management:** Mendukung konfigurasi modul RS485 dengan DE/RE terpisah, DE/RE digabung, maupun *Auto-Direction* (tanpa pin kontrol).
* **Clean Error Handling:** Fungsi mengembalikan `bool` (True jika sukses, False jika gagal). Kode error spesifik tetap dapat diakses via `get_code()`.
* **Multi Stream Support:** Mendukung beberapa Serial Port dengan metode active instance yang otomatis mengatur proses TX dan RX.

## 📦 Instalasi

1. Unduh repositori ini sebagai file `.zip`.
2. Buka Arduino IDE.
3. Masuk ke menu **Sketch** -> **Include Library** -> **Add .ZIP Library...**
4. Pilih file `.zip` yang baru saja diunduh.
5. Arduino IDE akan secara otomatis meminta Anda untuk menginstal dependensi `ModbusMaster` jika belum ada.

## 🚀 Dokumentasi API

### Inisialisasi
```cpp
// Format: begin(slave_id, serial_port, de_pin, re_pin)
modbus.begin(1, Serial1, 4, 5); // Mode DE dan RE Terpisah
modbus.begin(1, Serial1, 4);    // Mode DE dan RE Digabung (1 Pin)
modbus.begin(1, Serial1);       // Mode Modul Auto-Direction