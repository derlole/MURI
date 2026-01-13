 <!-- Written, maintained and owned by Louis Moser, Linus Braun, Benjamin Keppler (MURI DEVELOPMENT TEAM) -->

# MURI Projekt - Dokumentations-Grundgerüst

**Projektversion**: 2.0.0  
**Datum**: Dezember 2025  
**Status**: In Entwicklung

---

## Inhaltsverzeichnis

1. [Projektplan](#projektplan)
2. [Software-Architektur](#software-architektur)
3. [Designentscheidungen](#designentscheidungen)
4. [Technische Herleitungen](#technische-herleitungen)
5. [Lessons Learned](#Lessons-Learned)
5. [Documente und Referenzen](#Dokumente-und-Referenzen)

---

## Zugehörige Projektdokumentationen
[logic_2.md](logic_2)  
[ros.md](ros.md)  
[config.py](../muri_logics/config.py)  


---

# Projektplan

## 1. Projektübersicht

### 1.1 Ziele
- Entwicklung eines autonomen mobilen Roboters MURI (Mechanische Untergrund Ratte für Inspektionen) zur Navigation und Objektverfolgung
- Implementierung einer hierarchischen State-Machine-basierte Steuerungslogik
- Integration mit ROS2 für Echtzeitfähigkeit und Modularität
- Unterstützung für Kamera-basierte Zielerfassung und Aruco-Marker-Verfolgung

### 1.2 Projektumfang
- **Kernkomponenten**: 5 Logik-Module (Init, Drive, Turn, Follow, MainController)
- **Vision-Systeme**: Bild-Verarbeitung, Aruco-Marker-Erkennung
- **ROS2-Integration**: Action Server für alle Module
- **Dokumentation**: Design-Spezifikationen, Testpläne, Deployment-Guides

### 1.3 Stakeholder
- Entwicklungsteam 
    - Braun Linus
    - Keppler Benjamin
    - Moser Louis

- Auftraggeber
    - Prof. Dr. Mathias Lorenzen

---

## 2. Meilensteine und Zeitplan

### Phase 1: Projektinitialisierung & Planung (KW43-KW44/2025)
| Meilenstein | Termin | Status | Beschreibung |
|------------|--------|--------|------------|
| M1.1 | KW43 (23.10.25) | ✅ | Einarbeitung ROS2 abgeschlossen, Roboter über ROS ansprechbar |
| M1.2 | KW44 (30.10.25) | ✅ | Projektplan/Projektarchitektur erstellt und abgestimmt |

### Phase 2: Repository-Setup & Architektur (KW45/2025)
| Meilenstein | Termin | Status | Beschreibung |
|------------|--------|--------|------------|
| M2.1 | KW45 (06.11.25) | ✅ | Repository initialisiert, Ordner- und Dateistruktur entsprechend Softwarearchitektur aufgesetzt |

### Phase 3: Basis-Fahrfunktionalität (KW46-KW47/2025)
| Meilenstein | Termin | Status | Beschreibung |
|------------|--------|--------|------------|
| M3.1 | KW47 (20.11.25) | ✅ | Allgemeine Fahrfunktionalität implementiert (Geschwindigkeitssteuerung, Lageregelung) |
| M3.2 | KW48 (27.11.25) | ✅ | Unit-Tests und Debugging für Basis-Fahrfunktionalität abgeschlossen |

### Phase 4: ArUco-Erkennung & Folgemanöver (KW49-KW51/2025)
| Meilenstein | Termin | Status | Beschreibung |
|------------|--------|--------|------------|
| M4.1 | KW49 (04.12.25) | ✅ | ArUco-ID-Unterscheidung und Robotererkennung implementiert |
| M4.2 | KW51 (18.12.25) | ✅ | Folgemanöver-Code vollständig implementiert und integriert |
| M4.3 | KW52 (23.12.25) | ✅ | Testing und Debugging für Folgemanöver |

### Phase 5: Finalisierung & Dokumentation (KW1-KW3/2026)
| Meilenstein | Termin | Status | Beschreibung |
|------------|--------|--------|------------|
| M5.1 | KW2 (02.01.26) | ✅ | Restliche Tests abgeschlossen, alle funktionalen Anforderungen validiert |
| M5.2 | KW3 (21.01.26) | ⏳ | Gesamtdokumentation finalisiert (Projektplan, Architektur, technische Herleitungen, Systemauswertung) |
| M5.3 | KW3 (21.01.26) | ✅ | Build-/Install-Anleitung erstellt |
| M5.4 | KW3 (21.01.26) | 🎯 | **Abschlusspräsentation und finale Abgabe** |

---

**Legende:**
- ✅ Abgeschlossen (100%)
- ⚠️ In Arbeit mit Verzögerung
- ⏳ Geplant/In Bearbeitung
- 🎯 Kritischer Meilenstein (Deadline)

---

## 3. Geplante Aktualisierungen & Fehlerbehandlung

### 3.1 Fehler (Priorität: P1)
| Fehler | Modul | Auswirkung | Status | Zielversion |
|--------|-------|-----------|--------|------------|

### 3.2 Feature-Enhancements (Priorität: P2)
| Feature | Beschreibung | Abhängigkeiten | Status | Zielversion |
|---------|-------------|-----------------|--------|------------|
| Pause-Mechanismus | Vollständige Implementierung von PAUSE-State | M3.1 | ⏳ Geplant | v2.1.0 |
| Error-Recovery | Automatische Fehlerbehandlung & Retry-Logik | M4.1 | ⏳ Geplant | v2.1.0 |
| Dynamic-Tuning | Runtime-Anpassung von Regelparametern | M4.2 | ⏳ Geplant | v2.2.0 |
| Logging-System | Strukturiertes Logging statt print() | M5.1 | ⏳ Geplant | v2.1.0 |

### 3.3 Bekannte Limitierungen
- **FollowLogic**: Nur ein Marker gleichzeitig verfolgbar
- **DriveLogic**: Keine Kollisionserkennung (ralativ)
- **MainController**: PAUSE-Zustand nicht persistent
- **Allgemein**: Keine Multi-Robot-Koordination

### 3.4 Abhängigkeiten & Versioning

```
MURI v2.0.0 (Current)
├── ROS2 Humble (min: Iron)
├── Python 3.9+ (min: 3.8)
├── OpenCV 4.5+ (für Vision)
├── numpy 1.20+
└── pytest (für Unit-Tests)

MURI v2.0.1 (Hotfix - geplant)
├── TurnLogic Key-Fix
├── Method-Name Konsistenz
└── State-Name Harmonisierung

MURI v2.1.0 (Feature-Release - geplant)
├── Pause-Mechanism
├── Logging-System
├ ── Error-Recovery
└── Documentation Updates
```

---

# Software-Architektur

## 1. Architektur-Übersicht

![Softweare Architektur Übersicht](<softweare_architektur.png>)

### 1.1 Schichtmodell

```
┌─────────────────────────────────────────────────────────┐
│                   ROS2 Interface Layer                  │
│    (Action Servers, Topics, Services)                   │
├─────────────────────────────────────────────────────────┤
│            High-Level Control Layer                     │
│         (MainController State Machine)                  │
├─────────────────────────────────────────────────────────┤
│          Behavior Logic Layer                           │
│  (InitLogic, DriveLogic, TurnLogic, FollowLogic)        │
├─────────────────────────────────────────────────────────┤
│            Foundation Layer                             │
│   (Interfaces, Common Functions, Configuration)         │
├─────────────────────────────────────────────────────────┤
│            Hardware Abstraction Layer                   │
│    (Sensors, Odometry, Motors, Camera)                  │
└─────────────────────────────────────────────────────────┘
```

### 1.2 Modularer Aufbau

```
muri_logics/
├── logic_interface.py          # Basis-Interfaces
├── general_funcs.py            # Gemeinsame Funktionen
├── config.py                   # Konfigurationsparameter
├── init_logic.py               # InitLogic-Modul
├── drive_logic.py              # DriveLogic-Modul
├── turn_logic.py               # TurnLogic-Modul
├── follow_logic.py             # FollowLogic-Modul
└── main_controller.py          # MainController
```

---

## 2. Komponentenbeschreibung

### 2.1 Logic Interface Layer

**Datei**: `logic_interface.py`

```python
# Base Classes
class Out(ABC):
    """Abstrakte Output-Klasse"""
    - values: Dict[str, float]
    - isValid: bool
    - resetOut()
    - getError()
    
class LogicInterface(ABC):
    """Standard Interface für Logic-Module"""
    - getOut() → Out
    - setActive() → bool
    - state_machine()
    - getActiveState() → Enum
    - reset()
    - setOdomData(x, y, t)
    - setCameraData(angle, distance)
    
class ExtendedLogicInterface(LogicInterface):
    """Erweiterte Interface mit Aruco-Support"""
    - setArucoData(id)
```

**Designentscheidung**: 
- Abstract Base Classes für Konsistenz
- Klare Trennung von Input/Output
- Erweiterbar ohne Bruch der API

### 2.2 Behavior Logic Modules

#### InitLogic
- **Verantwortung**: Initiale Ausrichtung zum Ziel
- **State-Count**: 6 States
- **Ausgaben**: angular_velocity_z, turned_angle
- **Abhängigkeiten**: p_regulator, quaternion_to_yaw

#### DriveLogic
- **Verantwortung**: Vorwärtsbewegung mit Ausrichtungskontrolle
- **State-Count**: 6 States
- **Ausgaben**: linear_velocity_x, angular_velocity_z, distance_remaining
- **Abhängigkeiten**: p_regulator, quaternion_to_yaw, Schpieth-Parameter

#### TurnLogic
- **Verantwortung**: Drehmanöver zur Neuausrichtung
- **State-Count**: 6 States
- **Ausgaben**: angular_velocity_z, turned_angle
- **Abhängigkeiten**: p_regulator, quaternion_to_yaw

#### FollowLogic
- **Verantwortung**: Verfolgung eines Markers
- **State-Count**: 7 States (mit ABORT)
- **Ausgaben**: linear_velocity_x, angular_velocity_z, distance_remaining
- **Abhängigkeiten**: p_regulator, quaternion_to_yaw, Aruco-Daten

### 2.3 MainController

**Verantwortung**: 
- High-Level Orchestration
- State-Transitions zwischen Modulen
- Zielmanagement
- Aruco-Trigger-Logik

**State-Graph**:
```
INIT → IDLE → INIT_ROBOT → DRIVE ↔ TURN
                              ↓
                           FOLLOW 
                              └──→ (zurück zu DRIVE)
```

---

## 3. Schnittstellen-Spezifikation

### 3.1 Input-Schnittstellen

| Schnittstelle | Typ | Einheit | Quelle | Update-Rate |
|---------------|-----|---------|--------|------------|
| `/odom` | pose (x, y, θ) | m, rad | Odometrie | 100 Hz |
| `/camera_data` | (angle, distance) | rad, m | Vision | 30 Hz |
| `/aruco_markers` | marker_id | int | Vision | 30 Hz |
| `setOdomData()` | (x, y, t) | m, rad | ROS-Topic | - |
| `setCameraData()` | (angle, distance) | rad, m | ROS-Topic | - |
| `setArucoData()` | id | int | ROS-Topic | - |

### 3.2 Output-Schnittstellen

| Schnittstelle | Typ | Einheit | Ziel | Frequenz |
|---------------|-----|---------|------|----------|
| `/cmd_vel` | (linear_x, angular_z) | m/s, rad/s | Motor-Controller | 100 Hz |
| `getOut()` | Out-Objekt | - | Action-Server | On-Demand |
| `values['linear_velocity_x']` | float | m/s | Cmd-Vel | - |
| `values['angular_velocity_z']` | float | rad/s | Cmd-Vel | - |

---

## 4. Datenfluss und Timing

### 4.1 Kontrollschleife

```
┌─────────────────────────────────────────────────┐
│ ROS2 Timer (100 Hz)                             │
├─────────────────────────────────────────────────┤
│ 1. Sensor-Daten einlesen                        │
│    └─ /odom → setOdomData()                     │
│    └─ /camera → setCameraData()                 │
│    └─ /aruco → setArucoData()                   │
│                                                  │
│ 2. State-Machine ausführen                      │
│    └─ MainController.state_machine()            │
│    └─ Aktives Modul.state_machine()             │
│                                                  │
│ 3. Berechnung durchführen                       │
│    └─ Aktives Modul.calculate()                 │
│                                                  │
│ 4. Output publishen                             │
│    └─ /cmd_vel ← linear_velocity_x, angular_z  │
│                                                  │
│ 5. Feedback senden                              │
│    └─ Action-Feedback aktualisieren             │
└─────────────────────────────────────────────────┘
```

---

# Designentscheidungen

## 1. State Machine Pattern

### 1.1 Warum State Machines?

**Entscheidung**: Verwendung des State Machine Patterns für alle Logik-Module

**Begründung**:
1. **Determinismus**: Vorhersagbares Verhalten, klare Zustandsübergänge
2. **Debugging**: Einfaches Tracking des aktuellen Zustands
3. **Testing**: Isolierbare States, leicht zu testen
4. **Wartbarkeit**: Logische Struktur, einfach erweiterbar
5. **Fehlerbehandlung**: SUCCESS/FAILED-States für klare Ausnahmefälle

**Alternativen betrachtet**:
- Behavior Trees: ❌ Zu komplex für diesen Anwendungsfall
- FSM mit Events: ❌ ROS2 hat keine Event-Systeme
- Hierarchische FSM: ✅ Ähnlich gewählt (MainController + Sub-Module)

### 1.2 State-Struktur

Alle Module verwenden 6 Standard-States:
```
INIT → IDLE → READY → *MOVE → SUCCESS/FAILED
```

**Warum diese Struktur?**
- `INIT`: Ressourcen-Initialisierung
- `IDLE`: Bereitschaftszustand, spart Rechenzeit
- `READY`: Konfiguration vor Aktivität
- `*MOVE`: Aktive Kontrolle
- `SUCCESS/FAILED`: Abschlusszustände mit klarer Aussagekraft

---

## 2. Proportional-Regler (P-Regler)

### 2.1 Warum P-Regler und nicht PID?

**Entscheidung**: Verwendung eines einfachen P-Reglers statt PID

**Begründung**:
1. **Einfachheit**: Weniger Parameter zu tunen
2. **Robustheit**: Weniger Überschwinger bei schnellen Änderungen
3. **Reaktionsgeschwindigkeit**: Für diese Anwendung ausreichend
4. **Recheneffizienz**: Keine Integration/Differentiation nötig
5. **Stabilität**: In der Praxis stabil ohne I/D-Anteile

**Herleitung**:
```
Error = Sollwert - Ist-Wert
Output = -Kp * Error

Beispiel (Winkelfehler):
Error = angle_to_target
Kp = 1.0
Output = -1.0 * Error

Wenn Error = 0.5 rad (28°):
Output = -0.5 rad/s (dreht gegen Error)
```

**Sättigung**:
```python
output = max(-max_vel, min(output, max_vel))
# Verhindert Übersteuerung
```

---

## 3. Hierarchische Steuerungsarchitektur

### 3.1 Warum Hierarchie?

**Entscheidung**: MainController orchestriert Sub-Module statt flache Struktur

**Begründung**:
1. **Separation of Concerns**: Jedes Modul hat klare Verantwortung
2. **Wiederverwendbarkeit**: Modules können einzeln getestet werden
3. **Skalierbarkeit**: Neue Verhalten ohne Änderung bestehender Module
4. **Fehlertoleranz**: Isolation von Fehlern auf Modul-Ebene

**Struktur**:
```
MainController (Orchestrator)
├─ InitLogic (Initialisierung)
├─ DriveLogic (Navigation)
├─ TurnLogic (Rotation)
└─ FollowLogic (Verfolgung)
```

### 3.2 Alternativarchitekturen betrachtet

| Ansatz | Vor | Nachteil | Entscheidung |
|--------|-----|---------|------------|
| **Monolith** | Einfach | Schwer zu debuggen | ❌ |
| **Flache Module** | Modular | Schwer zu koordinieren | ❌ |
| **Hierarchisch** | Klar strukturiert | Etwas Overhead | ✅ |
| **Actor Model** | Robust | ROS2 Hat keine Actor-Libs | ❌ |

---

## 4. Aruco-Marker Integration

### 4.1 Warum Aruco-Marker?

**Entscheidung**: Verwendung von Aruco-Markern für Objektverfolgung

**Begründung**:
1. **Robustheit**: Funktioniert unter verschiedenen Lichtsituationen
2. **Eindeutigkeit**: Jeder Marker hat eindeutige ID
3. **Performance**: Schnell zu erkennen (OpenCV optimiert)
4. **Standardisiert**: IEEE-Standard, weit verbreitet
5. **Kosteneffizient**: Einfach druckbar
---

## 5. DriveLogic Schpieth-Parameter

### 5.1 Warum separater Geschwindigkeit-Parameter?

**Entscheidung**: `__schpieth` statt fest kodierte Geschwindigkeit

**Begründung**:
1. **Flexibilität**: Fahrtgeschwindigkeit zur Laufzeit anpassen
2. **Sicherheit**: Langsamer fahren in kritischen Situationen
3. **Optimierung**: Schneller fahren auf freien Flächen
4. **Testbarkeit**: Verschiedene Geschwindigkeiten einfach testen
---

## 6. FollowLogic Abstands-P-Regler

### 6.1 Warum Negation im Fehler?

**Code**:
```python
linear_velocity = p_regulator(
    -(distance - follow_distance),  # Negation!
    KP_FOLLOW_LINEAR,
    MAX_VELOCITY
)
```

**Begründung**:
```
Ohne Negation:
- Soll: 0.2 m (follow_distance)
- Ist: 0.5 m (distance)
- Error = 0.5 - 0.2 = +0.3
- Output = -Kp * 0.3 = negativ → fährt rückwärts ❌

Mit Negation:
- Error = -(0.5 - 0.2) = -0.3
- Output = -Kp * (-0.3) = positiv → fährt vorwärts ✅
```

**Intuitive Regel**: Distanz > Sollwert → vorwärts fahren

---

# Vision-System: Design-Entscheidungen

---

## Architektur-Entscheidungen (Vision)

### 1. Zwei separate ROS2-Nodes + ausgelagerte OpenCV-Logik

**Entscheidung**: CameraReadOut (Bilderfassung) und ImageProcessing (Verarbeitung) als getrennte ROS2-Nodes, mit OpenCV-Logik in separate AMD-Klasse ausgelagert

**Begründung**:
- **Skalierbarkeit**: Verarbeitungsteil kann auf anderem Rechner ausgeführt werden
- **Parallelisierung**: Bilderfassung und Verarbeitung laufen asynchron
- **Code-Trennung**: OpenCV-Logik (AMD) ist unabhängig von ROS2, wiederverwendbar in anderen Kontexten
- **Wartbarkeit**: Getrennte Zuständigkeiten (Single Responsibility Principle)
- **Fehlertoleranz**: Ausfall eines Nodes beeinflusst den anderen nicht direkt


---

### 2. Grayscale-Konvertierung in CameraReadOut

**Entscheidung**: RGB → Grayscale im CameraReadOut-Node, nicht im ImageProcessing

**Begründung**:

| Aspekt | Benefit |
|---|---|
| **Bandbreite** | 66% weniger Daten bei Übertragung |
| **Bildrate** | Höherer Durchsatz von Bildern möglich (30 Hz vs. mögliche Drosselung bei RGB) |
| **Speicher** | Weniger RAM-Nutzung |
| **ArUco-Detektion** | Funktioniert gleichwertig mit Grayscale (Kontraste reichen aus) |

---

### 3. Timer-Frequenz 30 Hz in CameraReadOut

**Entscheidung**: Bilderfassung mit maximaler Kamera-Frequenz (30 Hz)

**Begründung**:
- **Maximale Aktualität**: Neuste verfügbare Bilder für Detektion
- **Kamera-Spezifikation**: Roboter-Kamera liefert max. 30 fps
- **Echtzeit-Anforderung**: Roboter-Steuerung benötigt niedrige Latenz
- **Pufferoptimierung**: Buffer-Size=1 verhindert veraltete Frames

---

### 3.1 CAP_PROP_BUFFERSIZE = 1 in CameraReadOut

**Entscheidung**: OpenCV-Kamera-Buffer auf Größe 1 setzen

**Begründung**:
- **Minimale Latenz**: Nur aktuellster Frame wird gepuffert
- **Echtzeitnavigation**: Roboter reagiert auf aktuelle Sensorposition
- **Stabilität**: Verhindert veraltete Frames in P-Regler

**Code**: `cap.set(cv.CAP_PROP_BUFFERSIZE, 1)`

---

## Marker-Erkennungs-Entscheidungen

### 4. ArUco-Dictionary DICT_5X5_1000

**Entscheidung**: 5×5-Marker mit bis zu 1000 verschiedenen IDs

**Vergleich mit Alternativen**:

| Dictionary | Marker-Größe | ID-Bereich | Erkennungs-Qualität |
|---|---|---|---|
| **4X4_50** | 4×4 Bits | 0-49 | Weniger robust bei Distanz/Bewegungsunschärfe |
| **5X5_1000** | 5×5 Bits | 0-999 | **Optimal** ✓ |
| **6X6_250** | 6×6 Bits | 0-249 | Bessere Reichweite, aber weniger IDs verfügbar |

**Begründung für 5X5_1000**:
- ✓ Bessere Erkennbarkeit aus Distanz/niedriger Auflösung als 4X4
- ✓ Ausreichend viele IDs für Mehrroboter-Szenarien (nicht begrenzt auf 2-3 Roboter)
- ✓ Schnellere Verarbeitung als 6X6
- ✓ Balance zwischen Robustheit und Vielfalt

---

### 5. Marker-Prioritäts-Logik (ID 69 > ID 0)

**Entscheidung**: Marker 69 vor Marker 0 bevorzugen (bei Mehrfach-Erkennung)

**Begründung**:
- **Performance**: Priorität vor solvePnP entscheiden (spart Berechnungen)
- **Szenario**: Fokus auf anderen Roboter (ID 69) statt Objekt (ID 0)
- **Flexibilität**: Leicht konfigurierbar
---


---

## Distanz- und Winkelberechnungen

### 6. solvePnP mit SOLVEPNP_IPPE_SQUARE

**Entscheidung**: Pose-Estimation via `cv.solvePnP()` mit IPPE_SQUARE-Flag

**Begründung**:
- Nutzt vorkalibrierte Kameraparameter für präzise Pose
- IPPE_SQUARE spezialisiert auf quadratische, planare Marker (ArUco)
- OpenCV-Standard, etabliert und optimiert

---

### 7. Manuelle Winkelberechnung statt rvec

**Entscheidung**: Winkel via `atan2(tvec[0], tvec[2])` statt aus `rvec`

**Begründung**:
- `rvec` liefert sprunghafte Werte bei Grenzfällen (zu instabil für P-Regler)
- Manuelle Berechnung aus `tvec` ist mathematisch stabil und vorhersagbar
- Direkt für Regelung nutzbar (Wertebereich [-π, π])

**Visualisierung** (Draufsicht von oben):
```
         Kamera
          *
         /|
        / | z_distance (tvec[2])
  x_off/  |
      /   |
     -----+
    Marker
    
tan(angle) = x_offset / z_distance
angle = atan2(x_offset, z_distance)
```

**Bereich**: [-π, π] (alle 4 Quadranten)

---

### 8. Last-Valid-Value-Filter (3-Wert-Buffer)

**Entscheidung**: Robustheits-Filter mit 3-Wert-Schiebe-Buffer

**Begründung**: Filtert temporäre Fehler (Lichtwechsel, Verdeckung) aus; gibt neuesten gültigen Wert zurück


---

## Fehlerbehandlung-Entscheidungen

### 9. Fehlercodes (-1000.0, π, 9999)

**Entscheidung**: Sentinel-Werte statt Exceptions für robuste Node-Ausführung

**Fehlerwerte**: Distanz=-1000.0 (unmöglich), Winkel=π (unerreichbar), ID=9999 (außerhalb DICT)

**Begründung**: ROS2-Nodes bleiben stabil; Logik-Module können Fehler erkennen und reagieren

---

### 10. Error-Counter & Schwellwert (> 10)

**Entscheidung**: Error-Flag setzen nach 10 aufeinanderfolgenden Fehlern

**Begründung**: Unterscheidet Kurzzeitfehler (Lichtwechsel) von permanenten Ausfällen;
---

## Kalibrierungs-Entscheidungen

### 11. CharUco statt Chess-Board für Kalibrierung

**Entscheidung**: CharUco-Kalibrierung für Kamera-Matrix

**Begründung**:
- Funktioniert mit weniger Bildern als Chess-Board
- Robuster gegen Verzerrungen und Beleuchtung
- Bessere Ecken-Erkennung → präziserer optischer Mittelpunkt


---

# Technische Herleitungen

## 1. Quaternion zu Yaw-Konversion

### 1.1 Mathematischer Hintergrund

**Gegeben**: Quaternion q = (x, y, z, w)

**Gesucht**: Yaw-Winkel (Rotation um Z-Achse)

### 1.2 Herleitung

```
Rotation Matrix aus Quaternion:
        ⎡ 1-2(y²+z²)   2(xy-wz)     2(xz+wy)   ⎤
    R = ⎢ 2(xy+wz)    1-2(x²+z²)   2(yz-wx)   ⎥
        ⎣ 2(xz-wy)    2(yz+wx)    1-2(x²+y²) ⎦

Yaw aus Rotation Matrix:
    yaw = atan2(R[1,0], R[0,0])
        = atan2(2(xy+wz), 1-2(y²+z²))

Vereinfacht (häufig benutzte Formel):
    yaw = atan2(2*w*z + 2*x*y, 1 - 2*y² - 2*z²)
```

---

## 2. Winkel-Wrap-Around Behandlung

### 2.1 Problem

```
Wenn Roboter von -2.9 rad zu +2.9 rad dreht:
Naiver Error: 2.9 - (-2.9) = 5.8 rad (>> π)
Ist aber: nur 0.4 rad Drehung
```

### 2.2 Herleitung der Lösung

```
Wenn |angle_error| > π:
    Es ist kürzer, in die andere Richtung zu gehen
    
Beispiel:
    angle_error = 3.5 rad (200°)
    Statt 200° zu drehen → 160° in andere Richtung
    
    Mathematik:
    if angle_error > π:
        angle_error -= 2π  (normalisiere)
    elif angle_error < -π:
        angle_error += 2π  (normalisiere)
```

---

## 3. Regler-Auslegung für Multi-Modul-System

### 3.1 P-Rgler

**Struktur**:
```
Soll-Winkel → P-Regler → angular_velocity
    Error = Soll - Ist
    Output = -Kp * Error

Soll-Distanz → P-Regler → linear_velocity
    Error = Soll - Ist
    Output = -Kp * Error
```

### 3.2 Kp-Wert Auslegung

**Methode**: Empirisches Tuning (Ziegler-Nichols Light)

**Prozess**:
1. Kp = 0.5 starten (konservativ)
2. Inkrementelle Erhöhung (+0.1) bis Oszillation auftritt
3. Kp auf 50% der Oszillations-Kp setzen
4. Mit echtem Robot validieren

---

## 4. Odometrie-Integration

### 4.1 Datenfluss von /odom

```
ROS2 /odom Topic
├─ message.pose.pose.position.x [m]
├─ message.pose.pose.position.y [m]
└─ message.pose.pose.orientation (Quaternion)
        └─ x, y, z, w
        
Konvertierung:
quaternion_to_yaw(x, y, z, w) → θ [rad]

Nutzung:
- InitLogic: Erste θ speichern, dann Differenz tracken
- DriveLogic: Für relative Ausrichtung nutzen
- TurnLogic: Gedrehten Winkel berechnen
```

### 4.2 Position-Tracking

**Nicht direkt genutzt** für:
- FollowLogic und DriveLogic verwenden relative Zieldistanz (von Camera)
- MainController berechnet estimated_goal_pose (6m voraus)

**Grund**: 
- Zu sensorabhängig (Odometrie drift)
- Vision ist augenblicklich verfügbar
- Relative Koordinaten sind robuster

---

## 5. Toleranzbereiche und Kalibrierungsergebnisse

### 5.1 ANGLE_TOLERANCE Herleitungen

**Definition**: Winkelfehler, unterhalb dem Regler abschaltet

**Formel für Minimaltoleranzen**:
```
ANGLE_TOL_MIN = (Sensor_Rauschen + Quantisierung) * 2

Beispiele:
- Camera Auflösung: ±0.05 rad (Vision-Rauschen)
- Quaternion Genauigkeit: ±0.01 rad
- Sicherheitsmargin: 2×
├─ InitLogic: 0.1 rad (minimal)
├─ DriveLogic: 0.15 rad (größer wegen Fahrt)
├─ TurnLogic: 0.1 rad (präzise Rotation)
└─ FollowLogic: 0.2 rad (toleranter)
```

### 5.2 GOAL_DISTANCE Herleitungen

**Definition**: Entfernung, die als "Ziel erreicht" zählt
                    |
                    └─> vom Benutzer frei wählbar

**Faktoren**:
- Kollisionsfreiraum: Mindestens 10cm Abstand
- Sensor-Genauigkeit: ±5cm

**Typische Werte**:
```
GOAL_DISTANCE = 0.3m (30cm)
├─ Roboter-Länge: 0.2m
├─ Sensor-Fehler: ±0.05m
└─ Safety-Margin: 0.05m
```

---

## 6. Follow-Modus Trigger-Logik

### 6.1 Aruco-ID 69 Spezifikation

**Warum ID 69?**
- Eindeutige ID (nicht 0-3 für andere Zwecke)
- Leicht zu merken (Standard-Wert)
- Dezimal: 69, Hex: 0x45

**Trigger-Logik im MainController**:
```python
if _dominant_aruco_id == 69:
    _goToFollow = True
    # Wird beim nächsten DRIVE→FOLLOW Übergang genutzt
```

**Flow**:
```
DRIVE State
  └─ Camera erkennt Marker mit ID=69
     └─ setArucoData(69) aufgerufen
        └─ MainController setzt _goToFollow=True
           └─ Beim nächsten Erfolg: DRIVE→FOLLOW
```

---

## 7. Validierungs- und Teststrategien

### 7.1 Integrations-Tests

```
Test: Init → Drive → Turn → Success
1. MainController initialisieren
2. Goal setzen
3. Sensordaten simulieren
4. State-Übergänge überprüfen
5. Final-Output validieren
```

### 7.2 Field-Tests

- Verschiedene Lichtverhältnisse
- Verschiedene Boden-Beschaffenheit
- Verschiedene Zielentfernungen

---

# Lessons Learned
## 1. Python Logic (Init, Drive, Turn, Follow)


### 1.1 Interface-Design 
Vorteile der einheitlichen Schnittstelle:

- Alle Module implementieren identische Methoden (getOut(), setActive(), state_machine())
- Konsistente API erleichtert Verständnis und Wartung
- MainController kann Module austauschen ohne Code-Änderungen
- Neue Module in 2-4 Stunden implementierbar durch vorgegebene Struktur
- Klare Trennung von Input/Output über standardisierte Datenklassen

#### Lesson Learned:  
Investition in durchdachte Interfaces zu Projektbeginn ermöglicht schnelle Erweiterungen während der gesamten Entwicklung

### 1.2 Unit-Test
Schnelle Fehlererkennung durch automatisierte Tests  
Vorteile:

- Sofortige Rückmeldung bei Funktionsänderungen
- Fehler werden vor Integration in Gesamtsystem erkannt
- Refactoring ohne Angst vor unerkannten Nebenwirkungen
- Module können isoliert ohne ROS2-Umgebung getestet werden
- Jeder State-Übergang systematisch validierbar
- Dokumentation des erwarteten Verhaltens durch Tests

#### Lesson Learned:  
Unit-Tests als Sicherheitsnetz ermöglichen schnelle Iteration und verhindern zeitraubende Debugging-Sessions auf dem Roboter

### 1.3 Modulare Aufgabentrennung
Separate Module für Init, Drive, Turn, Follow  
Vorteile der Trennung:

- Jedes Modul hat klar definierte Verantwortung
- Änderungen in einem Modul beeinflussen andere nicht
- Bugs sind leicht zu lokalisieren
- Neue Verhaltensweisen einzelner module einfach implementierbar
- Neue Module leicht hinzufügbar
- Wiederverwendbarkeit einzelner Module in anderen Projekten

#### Lesson Learned:  
Klare Aufgabentrennung beschleunigt Entwicklung, Debugging und Erweiterung erheblich

### 1.4 Weitere-Erkentnisse
#### Config-File (config.py):  
- Zentrale Parameter-Verwaltung ermöglicht schnelles Tuning ohne Code-Änderung
- Experimente in "Minuten" statt "Stunden"
#### Gemeinsame Funktionen (general_funcs.py):  
- Vermeidung von Code-Duplikation
- Konsistentes Verhalten über alle Module
#### State Machine Pattern:  
- Vorhersagbares und nachvollziehbares Verhalten
- Einfaches Debugging durch klare Zustandsverfolgung
- Systematische Testabdeckung aller Übergänge


## 2. Camera & ArUco-Tracking

### 2.1 Custom Marker sind zu aufwendig - Standard ArUco ist deutlich besser
Anfängliche Idee: Eigene Marker zur Position- und Distanz-Verfolgung entwickeln

**Problem**:
- Insgesamt sehr zeitaufwendig und fehleranfällig

**Lösung**: OpenCV DICT_5X5_1000 ArUco-Marker verwenden
- Sofort einsatzbereit, dokumentiert und getestet
- Robuste Erkennung, einfache Implementierung

#### Lesson Learned:  
**Standard-Lösungen bevorzugen**: Bewährte Marker sparen Wochen an Entwicklungszeit gegenüber Custom-Implementierungen

---

### 2.2 Kreis-Erkennung funktioniert nicht im Rohr
Versuch: HoughCircles für Rohr-Tracking

**Problem**:
- Rohr ist mit einem Brett ausgelegt → Halbkreis statt Kreis
- Keine zuverlässige Verfolgung möglich

#### Lesson Learned:  
**Feature-Geometrie muss zur Umgebung passen**: Generische Formen (Kreise) funktionieren nicht in nicht eindeutigen Szenarien — eindeutig markierte Positionen (ArUco) sind robust

---

### 2.3 solvePnP rvec-Werte sind zu instabil für die Regelung
Anfängliche Idee: Rotationsvektor direkt für P-Regler nutzen

**Beobachtung**: rvec springt zwischen Lösungen → keine stabilen Steuerwerte

**Lösung**: Manuelle Berechnung aus tvec: `angle = atan2(tvec[0], tvec[2])`
- Mathematisch stabil und monoton
- Direkt verwendbar für P-Regler

#### Lesson Learned:  
**Physikalische Interpretation vor mathematischer**: Die X-Z-Position des Markers ist robuster als die Rotationsdarstellung — Pose aus Translationsvektor ist praktikabler für Regelung

---

### 2.4 Kalibrierungs-Qualität prägt Pose-Berechnung entscheidend
Anfängliches Problem: Chess-Board-Kalbrierung lieferte falschen optischen Mittelpunkt (cx, cy)

**Symptom**: Konstante Positions-Fehler ohne erkennbare Ursache

**Lösung**: CharUco-Kalibrierung mit 10–20 hochqualitativen Bildern
- Bessere Ecken-Detektion → präziser optischer Mittelpunkt
- Direkter Einfluss auf solvePnP

#### Lesson Learned:  
**Kalibrierungs-Qualität ist essentiel**: Falscher optischer Mittelpunkt führt zu hartnäckigen systematischen Fehlern — Zeit in saubere Kalibrierung investieren spart Tage beim Debugging

---

### 2.5 OpenCV Camera-Buffer muss auf Größe 1 gesetzt werden
Anfängliches Problem: Standardmäßig große Buffer → veraltete Frames in Echtzeit-Loop

**Symptom**: Roboter lenkt zu weit aus → P-Regler bekommt verzögerte Sensordaten

**Lösung**: `cap.set(cv.CAP_PROP_BUFFERSIZE, 1)`
- Nur aktuellster Frame wird gepuffert
- Minimale Latenz für Regelung

#### Lesson Learned:  
**Hardware-Puffering ist eine versteckte Latenzquelle**: Standard-OpenCV-Buffer verursachen Navigation-Instabilität — explizit auf Größe 1 setzen ist essentiell für Echtzeit-Systeme

---

### 2.6 Raspberry Pi Performance & Auflösungs-Optimierung

**Problem**: Raspberry Pi zu schwach für Full-HD-Bilder (1920×1080)
- Kamera liefert theoretisch HD-Qualität
- Verarbeitung wird CPU-limitiert
- Bildrate kann nicht gehalten werden

**Experiment**: Verschiedene Auflösungen getestet


**Kritische Erkenntnisse**:
1. **Auflösungs-Reduktion**: 640×480 ist Standard-OpenCV-Größe
2. **RGB→Grayscale**: Skaliert Performanz erheblich (3 Kanäle → 1 Kanal)
3. **Timing der Konvertierung**: Je früher die Konvertierung erfolgt, desto besser
   - Konvertierung im CameraReadOut spart Bandbreite
   - ImageProcessing kann dann kleinere Frames zu verarbeiten

#### Lesson Learned:
Bildauswahl muss spezifisch zu dem verwendeten System angepasst werden
- Niedrige Auflösung wählen (640×480 statt HD)
- Grayscale früh im Pipeline konvertieren

---

## 3. ROS
### 3.1 Softwarearchitektur vor Implementierung
Bedeutung einer frühzeitig geplanten ROS-Architektur:

- Klare Aufteilung in Nodes, Topics, Services und Actions
- Gemeinsames Verständnis im Projektteam über Datenflüsse
- Vermeidung späterer, aufwendiger Architekturänderungen
- Verantwortlichkeiten einzelner Komponenten klar definiert
- Grundlage für skalierbare und wartbare Systeme

#### Lesson Learned:  
Bei Frameworks ist es besonders wichtig, **vor der Implementierung eine durchdachte Softwarearchitektur zu entwerfen**, um dem Projektteam eine gemeinsame Orientierung zu geben und spätere strukturelle Probleme zu vermeiden

---

### 3.2 Frühzeitige Interface-Erstellung
Vorteile einer frühzeitig definierten Schnittstellenstruktur:

- Einheitliche Message-Typen und klare Kommunikationsverträge
- Austauschbarkeit einzelner Nodes ohne Anpassung anderer Komponenten
- Reduzierte Kopplung zwischen Modulen
- Vereinfachte Teamarbeit durch klare Erwartungen an Ein- und Ausgaben
- Langfristige Stabilität des Gesamtsystems

#### Lesson Learned:  
Eine **frühzeitig durchdachte Interface-Erstellung** hilft, langfristig Problemen vorzubeugen und reduziert den Wartungsaufwand erheblich

---

### 3.3 Integrationsstrategie (Big-Bang vs. Inkrementell)
Erfahrungen aus der Systemintegration:

- Big-Bang-Integrationen sind schwer planbar und fehleranfällig
- Fehlerursachen sind bei später Gesamtintegration schwer zu lokalisieren
- Hoher Koordinationsaufwand im Team notwendig
- Funktioniert nur, wenn sich das gesamte Team der Risiken bewusst ist
- Kontinuierliche Gegenmaßnahmen und Tests erforderlich

#### Lesson Learned:  
**Big-Bang-Integrationen sind sehr aufwendig** und funktionieren nur dann, wenn sich das gesamte Team der Nachteile bewusst ist und aktiv gegen entstehende Probleme anarbeitet

---

### 3.4 Dokumentation & Framework-Verständnis
Bedeutung guter Dokumentation bei der Arbeit mit ROS:

- Fehlende oder unklare Dokumentation verlangsamt die Entwicklung stark
- Innovative Nutzung von Frameworks erfordert tiefes Systemverständnis
- Implizite Annahmen führen schnell zu Fehlkonfigurationen
- Eigene Dokumentation wird essenziell bei unklaren Framework-Details
- Wissenstransfer im Team ohne Dokumentation kaum möglich

#### Lesson Learned:  
**Schlecht dokumentierte Frameworks führen zu großen Problemen**, insbesondere dann, wenn sie innovativ oder außerhalb typischer Anwendungsfälle genutzt werden



---

## Dokumente und Referenzen

**Siehe auch**:  

Documentation:  
[logic_2.md](logic_2)  
[ros.md](ros.md)  
[config.py](../muri_logics/config.py)  

Unit-Tests:  
[test_init_logic.py](../muri_logics/muri_logics/test/test_init_logic.py)  
[test_drive_logic.py](../muri_logics/muri_logics/test/test_drive_logic.py)  
[test_turn_logic.py](../muri_logics/muri_logics/test/test_turn_logic.py)  
[test_follow_logic.py](../muri_logics/muri_logics/test/test_follow_logic.py)  

---

**Dokumentversion**: 2.0.0  
**Status**: In Review  
**Nächste Aktualisierung**: Nach v2.0.1 Release