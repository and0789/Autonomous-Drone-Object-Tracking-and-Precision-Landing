Autonomous Drone Object Tracking and Precision Landing

Deskripsi Proyek

Proyek ini merupakan sistem autonomous drone tracking yang memungkinkan drone untuk mendeteksi, mengikuti, dan mendarat secara presisi menggunakan kamera dan algoritma object tracking berbasis OpenCV dan PID Controller.

Proyek ini sangat berguna untuk berbagai aplikasi seperti:

Pengiriman paket dengan drone 🚁📦

Inspeksi otomatis (misalnya pada infrastruktur atau pertanian) 🏗️🌿

Pencarian dan penyelamatan (SAR - Search and Rescue) ⛑️🚨

Fitur Utama

✅ Tracking Objek Secara Otomatis menggunakan OpenCV (CSRT Tracker)✅ Koneksi ke Drone via MAVLink untuk kontrol navigasi otomatis✅ Precision Landing berbasis tracking objek dan PID Controller✅ ROI Selector untuk memilih target objek menggunakan mouse✅ Fail-Safe Mechanism untuk pendaratan darurat jika objek hilang

Hardware yang Digunakan

Untuk menjalankan proyek ini, berikut adalah perangkat keras yang diperlukan:

Raspberry Pi 5 (sebagai pemroses utama)

Alfa Network AWUS036ACM 802.11ac AC1200 (WiFi Adapter untuk komunikasi)

WiFi 6 Outdoor Long Range Access Point (sebagai sumber internet dan komunikasi ke drone)

Raspberry Pi AI Camera (untuk deteksi objek dan tracking)

Pixhawk 6C (autopilot drone)

Holybro Telemetry Radio (untuk komunikasi telemetri dengan Ground Station)

Seperangkat Drone Lengkap (frame, ESC, motor, propeller, baterai, dll.)

Struktur Proyek

.
├── camera_config.py       # Konfigurasi kamera
├── cleanup.py             # Modul untuk membersihkan sistem saat keluar
├── config.py              # File konfigurasi (parameter PID, kamera, dan drone)
├── drone_controller.py    # Kelas untuk mengendalikan drone (takeoff, landing, dll)
├── init_system.py         # Modul inisialisasi sistem (drone, kamera, tracker)
├── main.py                # File utama untuk menjalankan sistem
├── object_tracker.py      # Kelas untuk tracking objek dengan OpenCV
├── pid_controller.py      # Implementasi kontrol PID
├── roi_selector.py        # Modul pemilihan ROI menggunakan mouse
├── tracking.py            # Modul utama untuk pelacakan dan precision landing

Instalasi & Dependensi

1. Clone Repository

git clone https://github.com/username/autonomous-drone-tracking.git
cd autonomous-drone-tracking

2. Install Dependensi

pip install -r requirements.txt

3. Jalankan Program

python main.py

Cara Kerja

Jalankan main.py untuk menginisialisasi drone dan kamera.

Gunakan mouse untuk memilih objek yang akan di-tracking.

Sistem akan secara otomatis mengendalikan drone untuk mengikuti objek.

Jika objek berada dalam posisi optimal, drone akan otomatis mendarat dengan presisi.

Saat program dihentikan, sistem akan otomatis membersihkan koneksi dan melakukan pendaratan darurat.

Kontribusi

Kami menerima kontribusi dalam bentuk perbaikan bug, optimasi algoritma tracking, serta peningkatan stabilitas sistem. Jika ingin berkontribusi, silakan buat pull request atau laporkan issue.

Untuk pertanyaan lebih lanjut, silakan hubungi: 📧 andreseptian07@icloud.com

🚀 Selamat Mencoba dan Happy Coding! 🚀

