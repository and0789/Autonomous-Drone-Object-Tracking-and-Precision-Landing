Autonomous Drone Object Tracking and Precision Landing

Deskripsi Proyek

Proyek ini merupakan sistem autonomous drone tracking yang memungkinkan drone untuk mendeteksi, mengikuti, dan mendarat secara presisi menggunakan kamera dan algoritma object tracking berbasis OpenCV dan PID Controller.

Proyek ini sangat berguna untuk berbagai aplikasi seperti:

- Pengiriman paket dengan drone 🚁📦
- Inspeksi otomatis (misalnya pada infrastruktur atau pertanian) 🏗️🌿
- Pencarian dan penyelamatan (SAR - Search and Rescue) ⛑️🚨

Fitur Utama
✅ Tracking Objek Secara Otomatis menggunakan OpenCV (CSRT Tracker)
✅ Koneksi ke Drone via MAVLink untuk kontrol navigasi otomatis
✅ Precision Landing berbasis tracking objek dan PID Controller
✅ ROI Selector untuk memilih target objek menggunakan mouse
✅ Fail-Safe Mechanism untuk pendaratan darurat jika objek hilang

Hardware yang Digunakan
Untuk menjalankan proyek ini, berikut adalah perangkat keras yang diperlukan:
1. Raspberry Pi 5 (sebagai pemroses utama)
2. Alfa Network AWUS036ACM 802.11ac AC1200 (WiFi Adapter untuk komunikasi)
3. WiFi 6 Outdoor Long Range Access Point (sebagai sumber internet dan komunikasi ke drone)
5. Raspberry Pi AI Camera (untuk deteksi objek dan tracking)
6. Pixhawk 6C (autopilot drone)
7. Holybro Telemetry Radio (untuk komunikasi telemetri dengan Ground Station)
8. Seperangkat Drone Lengkap (frame, ESC, motor, propeller, baterai, RC, dll.)


Instalasi & Dependensi

1. Clone Repository
git clone https://github.com/username/autonomous-drone-tracking.git
cd autonomous-drone-tracking

2. Install Dependensi
pip install -r requirements.txt

3. Jalankan Program
python main.py


Cara Kerja
1. Jalankan main.py untuk menginisialisasi drone dan kamera.
2. Gunakan mouse untuk memilih objek yang akan di-tracking.
3. Sistem akan secara otomatis mengendalikan drone untuk mengikuti objek.
4. Jika objek berada dalam posisi optimal, drone akan otomatis mendarat dengan presisi.
5. Saat program dihentikan, sistem akan otomatis membersihkan koneksi dan melakukan pendaratan darurat.

Kontribusi

Kami menerima kontribusi dalam bentuk perbaikan bug, optimasi algoritma tracking, serta peningkatan stabilitas sistem. Jika ingin berkontribusi, silakan buat pull request atau laporkan issue.

Untuk pertanyaan lebih lanjut, silakan hubungi: 📧 andreseptian07@icloud.com

🚀 Selamat Mencoba dan Happy Coding! 🚀

