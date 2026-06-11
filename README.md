# ros2_orb_slam3

Dieses ROS 2-Package integriert **ORB-SLAM3** (Monokular / Monokular-Inertial) in eine modulare Pipeline zur Bildvorverarbeitung und Echtzeit-Lokalisierung. Es ist speziell für den Einsatz in **Straßenszenarien** auf fahrzeugmontierten Plattformen optimiert, verfügt über ein dynamisches Region-of-Interest (ROI) Masking zur Ausblendung von Störobjekten (z. B. der eigenen Motorhaube) und bietet umfassende Analyse- und Debug-Werkzeuge.

---

## Systemarchitektur & Datenfluss

Die Pipeline ist modular aufgebaut, um maximale Flexibilität zwischen Live-Betrieb, Datensatz-Auswertung (EuRoC) und Rosbag-Wiedergabe zu gewährleisten.

* **Datenquellen (Driver / Playback):** `rosbag_driver_node.py` liest Bild- und IMU-Daten aus MCAP/db3-Bags, während `mono_py_driver` EuRoC-konforme Datensätze einliest. Beide synchronisieren sich via Handshake mit dem C++-Kern.
* **Preprocessing:** `camera_preprocessing_node` übernimmt die Rektifizierung (Undistortion) der rohen Kamerabilder.
* **ROI-Filterung:** `roi_mask_node.py` maskiert unerwünschte Bildbereiche (Karosserieteile) und entfernt Frame-Duplikate basierend auf Timestamps oder CRC32-Prüfsummen.
* **SLAM-Kern:** `mono_node_cpp` verarbeitet die bereinigten Daten asynchron in einem dedizierten Worker-Thread, um Frame-Drops zu minimieren.

---

## Key Features

* **Asynchrones SLAM-Tracking:** Entkopplung von ROS 2-Callback-Threads und dem ORB-SLAM3-Tracking mittels internem Thread-Sicheren Frame-Queue-Buffer.
* **Automotive-Optimierung:** Dedizierte Konfigurationsprofile (`smartrollerz.yaml`) mit abgesenkten FAST-Schwellenwerten für kontrastarme Asphalt- und Betonoberflächen.
* **Geometrisches ROI-Masking:** Dynamisch konfigurierbare Ausblendung des unteren Bildbereichs (0.0 - 1.0) zur Eliminierung von Eigenbewegungs-Artefakten des Trägerfahrzeugs.
* **Deduplizierungs-Filter:** Schutz vor CPU-Overhead durch automatische Erkennung und Eliminierung doppelt gesendeter Frames.
* **Echtzeit-Performance-Metriken:** Kontinuierliche Überwachung von Tracking-FPS, Queue-Tiefe, Frame-Drops und Latenzen im C++-Kern.

---


## Schritt 1: Workspace vorbereiten

Voraussetzung ist ein ROS2-Jazzy-Workspace auf Ubuntu 24.04 oder einer kompatiblen Umgebung.

```bash
source /opt/ros/jazzy/setup.bash
```

## Schritt 2: Abhängige Pakete abrufen

Dieses Projekt benötigt zusätzliche Pakete, die du separat clonen musst.

### Smartrollerz Utilities (camera_preprocessing, smarty_utils)


Diese beinhalten `camera_preprocessing` und `smarty_utils` (setup_utils, timing, state_msgs, lane_msgs, uart).

## Schritt 3: Dieses Repository clonen

```bash
git clone https://github.com/DHBW-Smart-Rollerz/slam.git
```

## Schritt 4: Dependencies installieren

```bash
sudo apt update
sudo apt install -y \
  libpangolin-dev \
  libeigen3-dev \
  libopencv-dev \
  python3-pip
```

Für Python-Kompatibilität mit `cv_bridge`:

```bash
pip install "numpy<2" "opencv-python<4.10"
```

## Schritt 5: Bauen


```bash
colcon build --symlink-install
source install/setup.bash
```

## Projektstruktur

Nach dem Clone dieses Repositories befindest du dich im Ordner `slam-orbSlam3/`, der das Paket `ros2_orb_slam3` enthält.

| Bereich | Inhalt |
|--------|--------|
| `orb_slam3/` | ORB-SLAM3 Quellcode, Konfigurationen und Vocabulary |
| `scripts/` | `mono_driver_node.py`, `rosbag_driver_node.py`, `roi_mask_node.py` |
| `src/` | `mono_example.cpp` und `common.cpp` |
| `launch/` | Kern-Pipelines |


## Launch-Dateien


| Typ | Datei | Zweck |
|-----|-------|-------|
| Pipeline | `launch/live_with_preprocessing.launch.py` | Live-Kamera, Vorverarbeitung und SLAM |
| Pipeline | `launch/rosbag_with_preprocessing.launch.py` | Rosbag, Vorverarbeitung und SLAM |

### Live-Pipeline

```bash
source install/setup.bash
ros2 launch ros2_orb_slam3 live_with_preprocessing.launch.py \
  settings_name:=smartrollerz \
  input_topic:=/camera/image/raw \
  preprocessed_topic:=/camera/image/undistorted \
  roi_output_topic:=/camera/image/roi_masked \
  use_roi_mask:=true \
  roi_mask_top_percent:=0.45 \
  num_skip_frames:=0 \
  debug:=false
```

Dieser Ablauf ist für Live-Kameradaten gedacht. Das C++-Node `mono_node_cpp` nutzt dabei standardmäßig den Zeitstempel aus `Image.header.stamp`.

### Rosbag-Pipeline

```bash
source install/setup.bash
ros2 launch ros2_orb_slam3 rosbag_with_preprocessing.launch.py \
  settings_name:=smartrollerz \
  bag_path:=TEST_ROSBAGS/rosbag2_2026_01_24-13_56_00 \
  input_topic:=/camera/image/raw \
  playback_rate:=1.0 \
  use_roi_mask:=true \
  roi_mask_top_percent:=0.45 \
  enable_imu:=true \
  auto_detect_imu_topic:=true \
  debug:=false
```

Diese Pipeline spielt einen Rosbag ab.

### Wiedergabe eines EuRoC-Datensatzes

1. Terminal

```bash
source ./install/setup.bash
ros2 run ros2_orb_slam3 mono_py_driver.py --ros-args \
    -p settings_name:="smartrollerz" \
    -p image_seq:="MH01" \
    -p dataset_parent_path:="/pfad/zu/den/datasets"
```
2. Terminal
```bash
source ./install/setup.bash
ros2 run ros2_orb_slam3 mono_node_cpp --ros-args -p node_name_arg:=mono_slam_cpp
```

## Testdatensätze und Rosbags

Zum Testen des Systems werden Rosbags benötigt. Diese sind nicht im Repository enthalten, können aber vom Smartrollerz NAS bezogen werden.

```bash
# NAS-Zugriff (Smartrollerz-intern)

# Alternativ: MH5 von EuRoC herunterladen (öffentlich verfügbar)
# https://projects.asl.ethz.ch/datasets/EuRoC/
```

Nach dem Download oder NAS-Mount sollte die Struktur so aussehen:

```bash
slam/
├── TEST_ROSBAGS/
│   ├── rosbag2_2026_01_24-13_56_00/
│   └── ...
├── TEST_DATASET/
│   ├── MH_05_difficult/
│   └── ...
```

## Wichtige Parameter

| Parameter | Bedeutung | Typischer Wert |
|-----------|-----------|----------------|
| `settings_name` | ORB-SLAM3-Konfiguration aus `orb_slam3/config` | `smartrollerz`, `Monocular` |
| `bag_path` | Pfad zum Rosbag-Ordner | `TEST_ROSBAGS/...` oder absolut |
| `input_topic` | Bild-Topic im Bag oder Live-System | `/camera/image/raw`, `/camera/cam0/image_raw` |
| `preprocessed_topic` | Topic nach `camera_preprocessing` | `/camera/image/undistorted` |
| `roi_output_topic` | Topic nach ROI-Maske | `/camera/image/roi_masked` |
| `use_roi_mask` | ROI-Maske aktivieren | `true` oder `false` |
| `roi_mask_top_percent` | Anteil des oberen Bildbereichs, der erhalten bleibt | `0.45` |
| `playback_rate` | Wiedergabegeschwindigkeit des Bags | `1.0` |
| `enable_imu` | IMU mit abspielen | `true` oder `false` |
| `auto_detect_imu_topic` | IMU-Topic automatisch suchen | `true` oder `false` |
| `num_skip_frames` | Anzahl übersprungener Frames im Preprocessing | `0` |
| `debug` | Debug-Ausgabe im Preprocessing | `true` oder `false` |

## Topic-Fluss

Für die Live-Pipeline ist der normale Datenfluss so aufgebaut.

```text
/camera/image/raw -> camera_preprocessing -> /camera/image/undistorted -> roi_mask_node -> /camera/image/roi_masked -> mono_node_cpp
```

Wenn ROI-Masking deaktiviert ist, wird das vorverarbeitete Bild direkt an `mono_node_cpp` weitergegeben.




## Konfiguration

Die Parameter für Fahrzeugszenarien werden über die Datei `smartrollerz.yaml` gesteuert. Wichtige Anpassungen für Texturen auf Straßenoberflächen:

```yaml
# Erhöhte Sensitivität für feine Grauwertunterschiede auf Asphalt/Beton
ORBextractor.iniThFAST: 8
ORBextractor.minThFAST: 3

```

---


## Diagnose & Pipeline-Überwachung

Zur Verifikation der Datenströme und zur Fehlersuche ohne schweren GUI-Overhead enthält das Package zwei Hilfs-Nodes.

### Grafischer Kombi-Viewer

Zeigt das rohe Eingangsbild, das rektifizierte Bild und die Bird's-Eye-View (falls aktiv) nebeneinander in einer OpenCV-Collage inklusive Frame-Zähler an:

```bash
ros2 run ros2_orb_slam3 image_debug_viewer.py

```

### CLI-Verbindungsmonitor

Analysiert rein textbasiert die Frequenz (FPS) und den Status aller relevanten Topics, um Remapping-Fehler oder blockierte Pipelines sofort zu identifizieren:

```bash
ros2 run ros2_orb_slam3 simple_image_monitor.py --ros-args \
    -p orb_input_topic:="/camera/image/roi_masked"

```
