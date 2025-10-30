# Paket Simulasi Aruku Mujoco

## Gambaran Umum

Paket ROS 2 (`aruku_mujoco`) ini adalah simulasi untuk algoritma berjalan `aruku` yang menggunakan MuJoCo.

Paket ini menggabungkan versi sederhana dari package `aruku` dan antarmuka kontrol `tachimawari` yang sudah disesuaikan khusus untuk MuJoCo. Tujuannya untuk dapat melakukan tes dan visualisasi pola berjalan tanpa robot fisik.

Komponen intinya adalah:
* **`ArukuMujocoNode`**: Node ROS 2 utama yang mengatur simulasi dan menjembatani ROS 2 dengan MuJoCo.
* **`Simulator`**: Kelas C++ yang menjadi "dasboard" untuk MuJoCo (mengatur *load*, *step*, *render*, sensor, dan aktuator).
* **`aruku::mini`**: Versi sederhana dari logika inti `aruku` (CPG dan Inverse Kinematics).
* **`tachimawari::control::MujocoControlManager`**: "Sistem saraf" virtual yang mengirim perintah sendi ke MuJoCo.

## Yang Dibutuhkan

### Perangkat Lunak Sistem
* Ubuntu 24.04
* ROS 2 Jazzy
* MuJoCo Simulator
* GLFW
* CMake
* Compiler C++17

### Dependensi ROS 2
Pastikan paket-paket ROS 2 berikut ada di folder `src` dan sudah berhasil di-build:
* `aruku_interfaces`
* `tachimawari_interfaces`
* `keisan` (Library matematika)
* `jitsuyo` (Library utilitas)
* *(Dan dependensi lain yang mungkin dibutuhkan)*

## Cara Build

1.  Pastikan semua dependensi (termasuk semua paket ROS 2 di atas) sudah di-*clone* ke folder `src`.
2.  Buka terminal di *root* workspace Anda (misal, `~/ros2_ws`):
    ```bash
    cd ~/ros2_ws
    ```
3.  Gunakan `rosdep` untuk menginstal dependensi sistem yang mungkin kurang:
    ```bash
    rosdep install --from-paths src --ignore-src -r -y
    ```
4.  Build workspace-nya memakai `colcon`:
    ```bash
    colcon build --symlink-install
    ```

## Cara Menjalankan

1.  Buka terminal baru dan *source* workspace:
    ```bash
    source ~/ros2_ws/install/setup.bash
    ```
2.  Jalankan node utamanya:
    ```bash
    ros2 run aruku_mujoco aruku_mujoco_node
    ```
3.  Sebuah jendela MuJoCo akan muncul.
    * **Kontrol Gerak:** Gunakan **tombol panah** atau **W, A, S, D** untuk berjalan.
    * **Reset:** Tekan **R** untuk mereset simulasi.
    * **Keluar:** Tutup jendela simulasi atau tekan `Ctrl+C` di terminal.

## Pengaturan (Tuning)

Gaya berjalan robot dapat diatur lewat file `config/kinematic.json`

Isinya adalah pengaturan untuk:
* **`balance`**: Pengaturan *gain* untuk keseimbangan aktif menggunakan data IMU (giroskop) virtual.
* **`ratio`**: Ritme berjalan (periode langkah, rasio DSP, tinggi angkat kaki, ayunan tubuh).
* **`length`**: Ukuran fisik robot (panjang paha, betis) untuk perhitungan Inverse Kinematics.
* **`offset`**: Postur berdiri awal robot.

Nilai di file ini dapat diubah untuk menyempurnakan gaya berjalan robot.

## Struktur Folder

* **`src/main.cpp`**: Titik masuk (`main`) aplikasi. Menyalakan ROS 2, membuat node, dan mengatur *thread* untuk *render* dan *spin* ROS.
* **`src/aruku_mini/`**: Berisi logika inti berjalan (CPG, IK, balancing) yang sudah disederhanakan.
* **`src/simulator.hpp/.cpp`**: "Dasbor" & "Setir" MuJoCo. Mengatur `step` (fisika), `render` (visual), `set_control` (perintah motor), dan `get_gyro` (baca sensor).
* **`src/tachimawari_mini/`**: "Sistem Saraf" virtual yang berfungsi untuk menerjemahkan perintah ID sendi dari `aruku` menjadi perintah nama aktuator untuk `simulator`.
* **`src/jitsuyo/` & `src/keisan/`**: Utilitas untuk membaca JSON dan perhitungan matematika (Angle, Point3, Euler, Matrix).
* **`config/`**: Tempat menyimpan file (`kinematic.json`).
* **`model/`**: Semua aset 3D dan file XML (`scene.xml`, `robot.xml`) untuk MuJoCo.