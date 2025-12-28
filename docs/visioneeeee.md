# ArUco Marker Detection Dokumentation

## Übersicht

Die `AMD` (ArUco Marker Detection) Klasse kapselt die OpenCV ArUco-Erkennungslogik zur Detektion von ArUco-Markern und Berechnung ihrer räumlichen Position relativ zur Roboter-Kamera. Die Klasse wird extern aufgerufen, um pro Detektionsaufruf folgende Informationen zu liefern:

- **Marker-ID**: Identifikation des erkannten Markers
- **Distanz**: Entfernung vom Roboter zum Marker (Z-Koordinate)
- **Winkel**: Horizontaler Winkelversatz vom Roboter zum Marker (Y-Rotation)

Die Klasse unterstützt mehrere Marker mit unterschiedlichen physischen Größen und verwendet vorkalibrierte Kameraparameter für präzise 3D-Positionsschätzung.

## Systemintegration

Die AMD-Klasse wird in einem dedizierten ROS2-Node aufgerufen, der die detektierten Aruco-Daten (Marker-ID, Distanz, Winkel) über Topics publisht. Diese publizierten Daten werden dann von anderen Ros2 teilen verwendet.

Die Klasse selbst ist unabhängig von ROS2 und dient ausschließlich der Bildverarbeitung und Positionsberechnung.

## Klasseninitialisierung

### Konstruktor `__init__()`

**Zweck**: Initialisiert den ArUco-Detektor mit vorkonfigurierten Kameraparametern und Marker-Größen.

```python
def init(self):
    # Marker-Größen aus der Konfiguration laden
    self.marker_sizes = config.MARKER_SIZES
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_1000)
    aruco_params = aruco.DetectorParameters()
    self.detector = aruco.ArucoDetector(aruco_dict, aruco_params)
    self.aruco_dict = aruco_dict
    self.camera_matrix = np.array(config.CAMERA_MATRIX_RAW, dtype=np.float32)
    self.dist_coeffs = np.array(config.DISTANCE_COEFFICIENT, dtype=np.float32)
```

### Initialisierte Komponenten

**ArUco-Dictionary (`aruco.DICT_5X5_1000`)**:
- Verwendet 5×5-Marker für optimalen Kompromiss zwischen Erkennungsrobustheit und ID-Vielfalt
- Bessere Erkennbarkeit aus größerer Entfernung und bei niedrigerer Auflösung als 6×6-Marker

**Marker-Größen (`self.marker_sizes`)**:
- Dictionary mit Marker-ID als Schlüssel und physischer Größe als Wert
- Unterstützt mehrere Marker mit unterschiedlichen Dimensionen

- Beispiel: `{0: 50.0, 69: 30.0}` (id: size in mm)

**Kamera-Kalibrierungsparameter**:

`self.camera_matrix`:
- 3×3-Matrix mit intrinsischen Kameraparametern (Brennweite, optischer Mittelpunkt)
- Wurde durch separate Kamera-Kalibrierung für die verwendete Kamera ermittelt
- Format: `[[fx, 0, cx], [0, fy, cy], [0, 0, 1]]`
- Typ: `numpy.float32`

`self.dist_coeffs`:
- Verzerrungskoeffizienten der Kamera (radiale und tangentiale Verzerrung)
- Kompensiert Linsenverzerrungen für präzise 3D-Positionsberechnung
- Wurde durch separate Kamera-Kalibrierung ermittelt
- Typ: `numpy.float32`

**Detector-Objekt (`self.detector`)**:
- OpenCV `ArucoDetector`-Instanz mit Standard-Detektionsparametern
- Verwendet für die eigentliche Marker-Erkennung im Bild

**ArUco-Dictionary-Referenz (`self.aruco_dict`)**:
- Gespeicherte Referenz für einfacheres Handling in anderen Methoden
- Ermöglicht Zugriff auf Dictionary-Informationen ohne Neuinitialisierung

### Konfigurationsabhängigkeiten

Die Klasse benötigt folgende Parameter aus dem `config`-Modul:
- `MARKER_SIZES`: Dictionary mit Marker-IDs und ihren physischen Größen [mm]
- `CAMERA_MATRIX_RAW`: 3×3-Kamera-Intrinsik-Matrix
- `DISTANCE_COEFFICIENT`: Linsenverzerrungskoeffizienten

---

## Methode `aruco_detection()`

**Zweck**: Detektiert ArUco-Marker im Bild und gibt Distanz, Winkel und Marker-ID zurück.

### Methodensignatur

```python
def aruco_detection(self, img):


```

### Eingabeparameter

**`img` (numpy.ndarray)**:
- Erwartet ein **Graustufenbild** (single-channel)
- Grayscale-Format wird aus Übertragungsoptimierungsgründen verwendet (reduzierte Bandbreite, höhere Geschwindigkeit, mehr Bilder)
- Kein RGB-zu-Grayscale-Konvertierung innerhalb der Methode notwendig
- Die Konvertierung erfolgt extern vor dem Methodenaufruf

### Marker-Prioritätslogik

**Unterstützte Marker-IDs**:
- **ID 69**: Roboter-Marker (höchste Priorität)
- **ID 0**: Standard-Zielmarker (niedrigere Priorität)

**Priorisierungsregel**:
```python
index_69 = None
index_0 = None

for i in range(len(ids)):
    if ids[i] == 69:
    index_69 = i
    elif ids[i] == 0:
    index_0 = i

index_to_use = index_69 if index_69 is not None else index_0

```

**Entscheidungslogik**:
- Wenn **nur Marker 69** erkannt wird → Folgemanöver wird ausgelöst
- Wenn **nur Marker 0** erkannt wird → Roboter fährt einfach zum Ziel (durchs Rohr bis zum anderen Ende)
- Wenn **beide Marker gleichzeitig** erkannt werden → Marker 69 wird priorisiert, Marker 0 wird ignoriert
- Wenn **kein relevanter Marker** erkannt wird → Fehlerwert wird zurückgegeben

**Systemintegration**:
- Marker 69 triggert im MainController den Übergang zum FOLLOW-Zustand
- Marker 0 wird für Standard-Navigationsziele verwendet
- Die Priorisierung stellt sicher, dass Follow-Modus immer Vorrang hat

### Fehlerbehandlung

**Fehlerwerte bei keiner Detektion**: `(-1000.0, math.pi, 9999)`

Diese Werte signalisieren verschiedenen Logik-Modulen einen Detektionsfehler:
- **z_pos = -1000.0**: Ungültige Distanz (offensichtlich außerhalb des physikalischen Bereichs)
- **y_rot = math.pi**: Ungültiger Winkel (π rad = 180°),da er bei 180° physikalisch keinen Aruco Marker erkennen kann.
- **marker_id = 9999**: Fehler-ID (in Steuerungslogik als Abbruch-Signal verwendet), Diese ID ist keine existierende im verwendeten DIctionary

**Fehlerszenarien**:
1. Kein Marker im Bild erkannt (`ids is None`)
2. Marker erkannt, aber weder ID 69 noch ID 0 (`index_to_use is None`)
3. Marker-ID nicht in `config.MARKER_SIZES` konfiguriert
4. `cv.solvePnP()` schlägt fehl (`success == False`)

### Marker-Größen-Handling

**Dynamische Marker-Größen**:
```python
marker_id = ids[index_to_use]
marker_size = self.marker_sizes.get(marker_id)
```

Wenn die Marker-ID nicht konfiguriert ist, Fehler zurückgeben

```python
if marker_size is None:
return -1000.0, math.pi, 9999
```

**Zweck**:
- Unterstützt Marker mit unterschiedlichen physischen Abmessungen
- Ermöglicht präzise Distanzberechnung durch Verwendung der tatsächlichen Marker-Größe
- Flexibilität bei der Marker-Platzierung auf verschiedenen Robotern (Platzprobleme)

**3D-Objektpunkte**:

```python
obj_points = np.array([[-marker_size / 2, marker_size / 2, 0], # Obere linke Ecke
                        [ marker_size / 2, marker_size / 2, 0], # Obere rechte Ecke
                        [ marker_size / 2, -marker_size / 2, 0], # Untere rechte Ecke
                        [-marker_size / 2, -marker_size / 2, 0]], # Untere linke Ecke
dtype=np.float32)
```

- Definiert die 3D-Koordinaten der vier Marker-Ecken im Marker-Koordinatensystem
- Marker liegt in der XY-Ebene (Z=0)
- Ursprung im Marker-Zentrum

### Pose-Estimation mit solvePnP

**Methode**: `cv.solvePnP()` mit `SOLVEPNP_IPPE_SQUARE`-Flag

```python
success, _, self.tvec = cv.solvePnP(obj_points,
                                    corners[index_to_use],
                                    self.camera_matrix,
                                    self.dist_coeffs,
                                    flags=cv.SOLVEPNP_IPPE_SQUARE)

```

**Eingaben**:
- `obj_points`: 3D-Koordinaten der Marker-Ecken in Millimetern
- `corners[index_to_use][0]`: Detektierte 2D-Bildkoordinaten der Marker-Ecken
- `self.camera_matrix`: Intrinsische Kameraparameter
- `self.dist_coeffs`: Linsenverzerrungskoeffizienten

**Ausgabe**:
- `self.tvec`: Translationsvektor [x, y, z] vom Kamera-Koordinatensystem zum Marker
  - `tvec[0][0]`: X-Position (horizontal, positiv = rechts)
  - `tvec[1][0]`: Y-Position (vertikal, positiv = oben)
  - `tvec[2][0]`: Z-Position (Tiefe/Distanz, positiv = vom Kamera weg)

**SOLVEPNP_IPPE_SQUARE**:
- Optimiert für quadratische planare Objekte (ArUco-Marker)
- Infinitesimal Plane-based Pose Estimation (IPPE)
- Höhere Genauigkeit und Robustheit als Standard-Algorithmen

### Rückgabewerte

**Bei erfolgreicher Detektion**:

```python
if success:
    angle_rad = self.calculate_angle_to_marker()
    marker_id = ids[index_to_use]
    return self.tvec, angle_rad, marker_id
```

- **z_pos** (`self.tvec[2][0]`): Distanz zum Marker in Millimetern
- **y_rot** (`angle_rad`): Winkel zum Marker in Radiant (berechnet mit `calculate_angle_to_marker()`)
- **marker_id**: Erkannte Marker-ID (69 oder 0)

**Bei Fehler**:

```python
return -1000.0, math.pi, 9999
```

### Ablaufdiagramm

---

Eingabe: Graustufenbild
↓
detectMarkers() → corners, ids
↓
ids vorhanden?
├─ Nein → Rückgabe: (-1000.0, π, 9999)
└─ Ja → Suche nach ID 69 und ID 0
↓
Priorisiere ID 69, sonst ID 0
↓
Marker-ID gefunden?
├─ Nein → Rückgabe: (-1000.0, π, 9999)
└─ Ja → Lade marker_size aus Config
↓
marker_size konfiguriert?
├─ Nein → Rückgabe: (-1000.0, π, 9999)
└─ Ja → Erstelle obj_points
↓
solvePnP() → success, tvec
↓
success?
├─ Nein → Rückgabe: (-1000.0, π, 9999)
└─ Ja → Berechne angle_rad
↓
Rückgabe: (tvec, angle_rad, marker_id)

---

## Methode `calculate_angle_to_marker()`

**Zweck**: Berechnet den horizontalen Yaw-Winkel vom Roboter zum erkannten Marker mithilfe der 3D-Positionsdaten.

### Methodensignatur

```python
def calculate_angle_to_marker(self):
```

### Geometrische Grundlage

**Rechtwinkliges Dreieck**:

Die Methode konstruiert ein rechtwinkliges Dreieck im Kamera-Koordinatensystem:

                Kamera
                  *
                 /|
                /α|
  x_offset     /  | distance (Z-Achse)
(Gegenkathete)/   | (Ankathete)
             /    |
            /     |
Marker     --------


**Koordinatensystem**:
- **Z-Achse** (distance): Geradeaus von der Kamera, optische Achse (Bildmitte)
- **X-Achse** (x_offset): Horizontal, positiv = rechts von der Kamera
- **Y-Achse**: Vertikal (nicht verwendet für Yaw-Berechnung)

### Berechnungslogik

```python
distance = self.tvec # Z-Koordinate: Tiefe/Distanz zum Marker​
x_offset = self.tvec # X-Koordinate: Horizontaler Versatz

angle_rad = math.atan2(x_offset, distance)
```

**Trigonometrische Beziehung**:
\[
\alpha = \arctan\left(\frac{\text{x\_offset}}{\text{distance}}\right) = \text{atan2}(\text{x\_offset}, \text{distance})
\]

- **Gegenkathete**: `x_offset` (horizontaler Abstand vom Bildmittelpunkt)
- **Ankathete**: `distance` (Entfernung entlang der optischen Achse)
- **Winkel α**: Yaw-Winkel vom Roboter zum Marker

### Verwendung von `atan2` statt `atan`

**Vorteile von `math.atan2(y, x)`**:

1. **Vollständiger Wertebereich**: Gibt Winkel im Bereich \([-\pi, \pi]\) zurück (alle vier Quadranten)
2. **Division-by-Zero-Sicherheit**: Kein Problem wenn `distance = 0`
3. **Vorzeichenerhaltung**: Berücksichtigt automatisch das Vorzeichen von `x_offset` und `distance`
4. **Genauigkeit**: Präzisere Berechnung durch Nutzung beider Komponenten

**Vergleich**:
- `atan(x_offset / distance)`: Nur Bereich \((- \pi/2, \pi/2)\), Division-by-Zero-Fehler möglich
- `atan2(x_offset, distance)`: Bereich \([-\pi, \pi]\), robust gegen alle Eingabewerte

### Vorzeichenkonvention

**Positiver Winkel** (`angle_rad > 0`):
- Marker befindet sich **rechts** von der Bildmitte
- Roboter ist **links versetzt** vom Marker
- Roboter muss **nach rechts drehen**, um auf den Marker auszurichten
- Steuerbefehl: Positive Winkelgeschwindigkeit (`angular_velocity_z > 0`)

**Negativer Winkel** (`angle_rad < 0`):
- Marker befindet sich **links** von der Bildmitte
- Roboter ist **rechts versetzt** vom Marker
- Roboter muss **nach links drehen**, um auf den Marker auszurichten
- Steuerbefehl: Negative Winkelgeschwindigkeit (`angular_velocity_z < 0`)

**Null-Winkel** (`angle_rad ≈ 0`):
- Marker liegt in der Bildmitte
- Roboter ist bereits korrekt ausgerichtet
- Keine Drehung erforderlich

### Integration mit Steuerlogik

Der von `calculate_angle_to_marker()` zurückgegebene Winkel wird direkt als Winkelfehler in den P-Reglern der Logik-Module verwendet:

- Positiver Winkelfehler → Positive Winkelgeschwindigkeit → Rechtsdrehung
- Negativer Winkelfehler → Negative Winkelgeschwindigkeit → Linksdrehung
- Die P-Regler-Negation erfolgt in `p_regulator()`, nicht in der AMD-Klasse

Siehe `logic_2.md` für Details zur Verwendung in InitLogic, DriveLogic, TurnLogic und FollowLogic.

### Beispielrechnung

**Szenario 1: Marker rechts versetzt**
