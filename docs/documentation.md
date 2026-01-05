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
| M4.1 | KW49 (04.12.25) | ⚠️ | ArUco-ID-Unterscheidung und Robotererkennung implementiert (60% abgeschlossen) |
| M4.2 | KW51 (18.12.25) | ✅ | Folgemanöver-Code vollständig implementiert und integriert |
| M4.3 | KW52 (23.12.25) | ⏳ | Testing und Debugging für Folgemanöver (75% abgeschlossen, 1 offenes Issue) |

### Phase 5: Finalisierung & Dokumentation (KW1-KW3/2026)
| Meilenstein | Termin | Status | Beschreibung |
|------------|--------|--------|------------|
| M5.1 | KW2 (02.01.26) | ⏳ | Restliche Tests abgeschlossen, alle funktionalen Anforderungen validiert |
| M5.2 | KW3 (21.01.26) | ⏳ | Gesamtdokumentation finalisiert (Projektplan, Architektur, technische Herleitungen, Systemauswertung) |
| M5.3 | KW3 (21.01.26) | ⏳ | Build-/Install-Anleitung und Test-Dokumentation erstellt |
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
| Method-Name Tippfehler (`setGoalStautusFinished`) | MainController | API-Inkonsistenz | 🟡 Known | v2.0.1 |

### 3.2 Feature-Enhancements (Priorität: P2)
| Feature | Beschreibung | Abhängigkeiten | Status | Zielversion |
|---------|-------------|-----------------|--------|------------|
| Pause-Mechanismus | Vollständige Implementierung von PAUSE-State | M3.1 | ⏳ Geplant | v2.1.0 |
| Error-Recovery | Automatische Fehlerbehandlung & Retry-Logik | M4.1 | ⏳ Geplant | v2.1.0 |
| Dynamic-Tuning | Runtime-Anpassung von Regelparametern | M4.2 | ⏳ Geplant | v2.2.0 |
| Logging-System | Strukturiertes Logging statt print() | M5.1 | ⏳ Geplant | v2.1.0 |

### 3.3 Bekannte Limitierungen
- **FollowLogic**: Nur ein Marker gleichzeitig verfolgbar
- **DriveLogic**: Keine Kollisionserkennung
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

## 4. Ressourcenplanung

### 4.1 Team
- **Robotik-Engineer**: ───┐
- **Software-Engineer**: ──┤
- **Test-Engineer**: ─────┤─── 3 FTE (Entwiklungsteam)
- **Technischer Writer**: ──┤
- **Hardware-Engineer**: ──┘ 

### 4.2 Infrastruktur
- Entwicklung: Linux-Workstations mit ROS2
- Testing: 2x Roboter-Prototypen
- CI/CD: GitHub Actions
- Dokumentation: GitBook / ReadTheDocs

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

### 4.1 Kontrollschleife (Control Loop)

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

**Timing-Anforderungen**:
- Control-Loop: 100 Hz
- Vision-Input: 30 Hz
- Odometrie-Input: 100 Hz
- Output-Publikation: 100 Hz

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

### 2.2 Kp-Wert Auslegung

**Methode**: Empirisches Tuning (Ziegler-Nichols Light)

**Prozess**:
1. Kp = 0.5 starten (konservativ)
2. Inkrementelle Erhöhung (+0.1) bis Oszillation auftritt
3. Kp auf 50% der Oszillations-Kp setzen
4. Mit echtem Robot validieren

**Typische Werte**:
- `KP_INIT`: 1.5 (schnelle Ausrichtung)
- `KP_DRIVE`: 0.8 (sanfte Winkelkorrektur während Fahrt)
- `KP_TURN`: 1.2 (genaue Rotation)
- `KP_FOLLOW_ANGULAR`: 1.0 (stabile Verfolgung)
- `KP_FOLLOW_LINEAR`: 0.5 (sanfte Abstandsanpassung)

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

**Marker-IDs und Bedeutungen**:
- `ID = -1`: Kein Marker erkannt
- `ID = 69`: Trigger für Follow-Modus (spezial)
- `ID = 0`: Success-Code in FollowLogic
- `ID = 9999`: Fehler/Abbruch-Code

---

## 5. DriveLogic Schpieth-Parameter

### 5.1 Warum separater Geschwindigkeit-Parameter?

**Entscheidung**: `__schpieth` statt fest kodierte Geschwindigkeit

**Begründung**:
1. **Flexibilität**: Fahrtgeschwindigkeit zur Laufzeit anpassen
2. **Sicherheit**: Langsamer fahren in kritischen Situationen
3. **Optimierung**: Schneller fahren auf freien Flächen
4. **Testbarkeit**: Verschiedene Geschwindigkeiten einfach testen

**Implementierung**:
```python
def setSchpieth(self, s):
    """Nur außerhalb DRIVEMOVE änderbar"""
    if self.__state != DriveStates.DRIVEMOVE:
        self.__schpieth = s
```

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

# Technische Herleitungen #TODO kommlett überarbeiten

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

### 1.3 Implementierung

```python
import math

def quaternion_to_yaw(quaternion):
    """Konvertiert Quaternion (x,y,z,w) zu Yaw [rad]"""
    q = quaternion
    
    # Formel
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)

    yaw = math.atan2(siny_cosp, cosy_cosp)
    
    return yaw
```

**Validierung**:
- Yaw = 0: Roboter zeigt nach vorne
- Yaw = π/2: Roboter zeigt nach links
- Yaw = -π/2: Roboter zeigt nach rechts

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

### 2.3 Implementierung in InitLogic

```python
def calculate(self):
    turned_angle = self.__positionTheta - self.__firstTheta
    
    # Wrap-Around Handling
    if abs(turned_angle) > math.pi:
        turned_angle = turned_angle + 2 * math.pi
    
    # Jetzt ist turned_angle im Bereich [-π, π]
    return angular_velocity, turned_angle
```

---

## 3. Regler-Auslegung für Multi-Modul-System

### 3.1 Kaskadenregelung (Cascade Control)

**Struktur**:
```
Soll-Winkel → P-Regler → angular_velocity
    Error = Soll - Ist
    Output = -Kp * Error
    
    Kp groß (z.B. 1.5):
    ├─ Schnelle Reaktion
    ├─ Aber: Überschwinger & Oszillationen
    └─ Nicht für kontinuierliche Bewegung ideal

Soll-Distanz → P-Regler → linear_velocity
    Error = Soll - Ist
    Output = -Kp * Error
```

### 3.2 Wahl der Reglerparameter

**Für InitLogic (reine Rotation)**:
- Kp = 1.5 (aggresiv)
- Grund: Schnell ausgerichtet, minimale Wartezeit
- Test: Mit 0.5 rad Fehler → 0.75 rad/s Output ✓

**Für DriveLogic (Fahrt + Winkelkorrektur)**:
- Kp_Angular = 0.8 (sanfter)
- Grund: Sollte nicht zu aggressiv korrigieren während Fahrt
- Test: Mit 0.5 rad Fehler → 0.4 rad/s Output ✓

**Für FollowLogic (stabile Verfolgung)**:
- Kp_Angular = 1.0, Kp_Linear = 0.5
- Grund: Asymmetrisch (Rotation wichtiger als Distanz)
- Test: Stabile Verfolgung ohne Oszillationen ✓

### 3.3 Reglervalidierung

```
Schrittantwort-Test:
1. Fehler = 1.0 rad setzen
2. Output messen
3. Überprüfen:
   - Keine Oszillation
   - Schnelle Konvergenz (< 2 Sek)
   - Steady-State Error ≈ 0

Erwartete Ergebnisse (Kp=1.0):
- t=0ms: Error=1.0, Output=-1.0
- t=100ms: Error≈0.7, Output≈-0.7
- t=500ms: Error≈0.1, Output≈-0.1
- t=1000ms: Error≈0.0, Output≈0.0
```

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

**Faktoren**:
- Roboter-Größe: Länge/Breite
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

## 7. Fehlercodes und Fehlerbehandlung

### 7.1 Spezielle Werte

| Wert | Kontext | Bedeutung | Behandlung |
|------|---------|-----------|-----------|
| distance = -1.0 | Camera | Fehler/kein Ziel | Stop-Befehl |
| aruco_id = -1 | Camera | Kein Marker | Ignorieren |
| aruco_id = 0 | FollowLogic | Success-Code | → SUCCESS State |
| aruco_id = 69 | DRIVE | Follow-Trigger | Setze _goToFollow |
| aruco_id = 9999 | FollowLogic | Fehler/Abbruch | → FAILED State |

### 7.2 Error-Recovery-Strategie

```
FAILED State erreicht
├─ Fehler loggen
├─ Module zurücksetzen
├─ Zu IDLE zurückspringen
└─ Externe Recovery initiieren
    └─ Neuversuch oder Abbruch
```

---

## 8. Performance-Charakteristiken

### 8.1 Rechenlast pro Modul

| Modul | calculate() | state_machine() | Gesamt |
|-------|------------|-----------------|--------|
| InitLogic | ~0.1ms | ~0.05ms | ~0.15ms |
| DriveLogic | ~0.1ms | ~0.05ms | ~0.15ms |
| TurnLogic | ~0.1ms | ~0.05ms | ~0.15ms |
| FollowLogic | ~0.15ms | ~0.05ms | ~0.2ms |
| MainController | ~0.05ms | ~0.1ms | ~0.15ms |
| **Gesamt** | | | **~0.8ms** |

**Zielbudget**: 10ms (100 Hz Control-Loop)
**Verfügbar**: 10ms - 0.8ms = 9.2ms (92% Reserve) ✓

### 8.2 Speicher-Footprint

```
Pro Modul:
- State-Machine: ~100 bytes
- Output-Object: ~200 bytes
- Variables: ~300 bytes
├─ Gesamt/Modul: ~600 bytes
└─ 5 Module: ~3 KB

Overhead (Config, Interfaces): ~2 KB
Gesamt: ~5 KB (sehr sparsam)
```

---

## 9. Skalierungsüberlegungen

### 9.1 Hinzufügen neuer Verhaltensmodi

**Neue Logic hinzufügen**:
1. Neue Klasse: `NewLogic(LogicInterface)`
2. 6 States implementieren
3. `calculate()` und `state_machine()` füllen
4. Im MainController: Neuer State + Übergang
5. Neuer Action Server in ROS2

**Aufwand**: ~2-4 Stunden pro Modul

### 9.2 Multi-Roboter-Koordination

**Aktuell**: Nicht unterstützt

**Zukünftige Lösung**:
- Leader-Follower Architektur
- Shared MainController für Koordination
- Broadcast von Goal-Status zwischen Robots

---

## 10. Validierungs- und Teststrategien

### 10.1 Unit-Tests pro Modul

```python
# Beispiel: InitLogic Test
def test_init_logic_alignment():
    logic = InitLogic()
    logic.setActive()
    
    # Setze Fehler
    logic.setCameraData(angle=0.5, distance=2.0)
    logic.setOdomData(x=0, y=0, t=(0,0,0,1))
    
    # Berechne Output
    logic.calculate()
    logic.state_machine()
    
    # Validiere
    output = logic.getOut()
    assert output.isValid == True
    assert output.values['angular_velocity_z'] != 0
    assert output.values['turned_angle'] == 0
```

### 10.2 Integrations-Tests

```
Test: Init → Drive → Turn → Success
1. MainController initialisieren
2. Goal setzen
3. Sensordaten simulieren
4. State-Übergänge überprüfen
5. Final-Output validieren
```

### 10.3 Field-Tests

- Verschiedene Lichtverhältnisse
- Verschiedene Boden-Beschaffenheit
- Verschiedene Zielentfernungen
- Multi-Modul-Übergang testen

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


## 2. Camera

## 3. ROS 


---

## Zusammenfassung der Design-Philosophie

```
1. CLARITÄT
   ├─ State Machines für determinism
   ├─ Klare Module/Verantwortungen
   └─ Explizite Input/Output-Schnittstellen

2. EINFACHHEIT
   ├─ P-Regler statt PID
   ├─ Keine komplexen Algorithmen
   └─ Wartbarkeit über Optimierung

3. ROBUSTHEIT
   ├─ Wrap-Around Handling
   ├─ Fehler-Codes (distance=-1.0, id=9999)
   └─ SUCCESS/FAILED-States

4. SKALIERBARKEIT
   ├─ Modularer Aufbau
   ├─ Erweiterbare Interfaces
   └─ Hierarchische Steuerung

5. TESTBARKEIT
   ├─ Isolierbare Module
   ├─ Deterministische Ausgabe
   └─ Simulierbare Eingaben
```

---

## Dokumente und Referenzen

**Siehe auch**:
- `logic_2.md` - Vollständige Logik-Dokumentation
- `ros.md` - ROS2 Integration
- `config.py` - Alle Tuning-Parameter
- `test/` - Unit & Integration Tests

---

**Dokumentversion**: 2.0.0  
**Status**: In Review  
**Nächste Aktualisierung**: Nach v2.0.1 Release