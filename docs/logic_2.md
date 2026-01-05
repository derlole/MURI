# Roboter-Steuerungslogik Dokumentation

## Übersicht

Diese Dokumentation beschreibt die Roboter-Steuerungslogik, die aus fünf State-Machine-Modulen besteht, die zusammenarbeiten, um das Navigationsverhalten eines mobilen Roboters zu steuern. Das System implementiert eine hierarchische Steuerungsarchitektur mit einem Haupt-Controller, der vier spezialisierte Verhaltensmodule koordiniert: Initialisierung, Fahren, Drehen und Folgen.

## Systemarchitektur

Das Steuerungssystem folgt einer modularen State-Machine-Architektur:

- **MainController**: Übergeordneter Orchestrator, der Übergänge zwischen verschiedenen Verhaltenszuständen verwaltet
- **InitLogic**: Behandelt die initiale Roboter-Ausrichtung zu einem Ziel
- **DriveLogic**: Steuert die Vorwärtsbewegung zum Ziel unter Beibehaltung der Ausrichtung
- **TurnLogic**: Verwaltet Drehmanöver zur Neuausrichtung des Roboters
- **FollowLogic**: Verfolgt ein Ziel (dem anderen Roboter) mittels Aruco-Marker

Alle Module implementieren die `LogicInterface` oder `ExtendedLogicInterface`.

## Schnittstellen (Interfaces)

### LogicInterface
Basis-Interface, das von InitLogic, DriveLogic und TurnLogic implementiert wird:

```python
class LogicInterface(ABC):
    def getOut() -> Out
    def setActive() -> bool
    def state_machine()
    def getActiveState() -> Enum
    def reset()
    def setOdomData(x: float, y: float, t: float)
    def setCameraData(angle_in_rad: float, distance_in_meters: float)
```

### ExtendedLogicInterface
Erweiterte Interface für FollowLogic und MainController mit zusätzlicher Aruco-Unterstützung:

```python
class ExtendedLogicInterface(LogicInterface):
    def setArucoData(dominant_aruco_id: int)
```

## Gemeinsame Komponenten

### State Machine Pattern

Jedes Logik-Modul implementiert ein konsistentes State-Machine-Muster mit folgenden Zuständen:

- `INIT`: Initialer Zustand für Variableninitialisierung
- `IDLE`: Wartezustand, bereit zur Aktivierung
- `READY`: Übergangszustand zur Vorbereitung des aktiven Betriebs
- `*MOVE`: Aktiver Ausführungszustand (INITMOVE, DRIVEMOVE, TURNMOVE, FOLLOWMOVE)
- `SUCCESS`: Ziel erfolgreich erreicht
- `FAILED`: Fehler- oder Fehlschlagbedingung
- `ABORT`: Abbruchzustand (nur FollowLogic)

### Output Interface

Alle Output-Klassen (`InitOut`, `DriveOut`, `TurnOut`, `FollowOut`, `MainOut`) implementieren:

- `values`: Dictionary-Property mit Ausgabewerten
- `isValid`: Boolean-Flag, das anzeigt, ob die Ausgabe gültig ist
- `resetOut()`: Setzt die Ausgabe auf den Anfangszustand zurück
- `outValid()`: Prüft die Gültigkeit der Ausgabe
- `getError()`: Gibt den Fehlerzustand zurück
- `setError(er)`: Setzt den Fehlerzustand

### Gemeinsame Methoden

Jedes Logik-Modul implementiert:

- `getOut()`: Gibt das Output-Objekt zurück
- `setActive()`: Aktiviert die Logik aus dem IDLE-Zustand
- `getActiveState()`: Gibt den aktuellen Zustand zurück
- `reset()`: Setzt das Modul auf den Anfangszustand zurück
- `setOdomData(x, y, t)`: Setzt Odometriedaten (Position und Orientierung)
- `setCameraData(angle, distance)`: Setzt Kameradaten
- `calculate()`: Berechnet Steuerbefehle
- `state_machine()`: Führt die State-Machine-Logik aus

---

## Modul-Details

### 1. InitLogic

**Zweck**: Dreht den Roboter, um initial auf ein erkanntes Ziel auszurichten.

**Zustände**:
- `INIT` → `IDLE` → `READY` → `INITMOVE` → `SUCCESS`/`FAILED`

**Zentrale Variablen**:
- `__firstTheta`: Initialer Orientierungswinkel (beim Aktivieren aufgezeichnet)
- `__positionTheta`: Aktuelle Roboter-Orientierung
- `__angle_to_Mid_in_Rad`: Winkelabweichung zum Ziel
- `__distance_in_Meter`: Entfernung zum Ziel

**Ausgabewerte**:
- `angular_velocity_z`: Rotationsbefehl [rad/s]
- `turned_angle`: Gesamtwinkel seit Aktivierung gedreht
- `linear_velocity_x`: Nicht verwendet (None)
- `linear_velocity_y`: Nicht verwendet (None)

**Steuerlogik**:
```python
def calculate(self):
    # Berechnet gedrehten Winkel vom initialen Winkel
    turned_angle = current_theta - first_theta
    
    # Behandelt Wrap-Around bei Winkeln > π
    if abs(turned_angle) > π:
        turned_angle = turned_angle + 2 * π
    
    # Verwendet P-Regler wenn Winkelfehler Toleranz überschreitet
    if abs(angle_error) > ANGLE_TOLLERANCE_INIT and distance_in_Meter > ORIANTATION_DISTANCE:
        angular_velocity = p_regulator(angle_error, KP_INIT, MAX_ANGLE_VELOCITY_TURN_INIT)
    
    # Stoppt bei Ausrichtung innerhalb der Toleranz
    if abs(angle_error) < ANGLE_TOLLERANCE_INIT and distance_in_Meter > ORIANTATION_DISTANCE:
        angular_velocity = 0.0
```

**Erfolgsbedingung**: 
- Winkel zum Ziel < `ANGLE_TOLLERANCE_INIT` UND Entfernung > 1.0m

**Konfigurationsparameter**:
- `ANGLE_TOLLERANCE_INIT`: Winkeltoleranz für Ausrichtung
- `KP_INIT`: Proportionalverstärkung für P-Regler
- `MAX_ANGLE_VELOCITY_TURN_INIT`: Maximale Rotationsgeschwindigkeit
- `ORIANTATION_DISTANCE`: Minimale Zielentfernung für Rotation

---

### 2. DriveLogic

**Zweck**: Fährt den Roboter vorwärts zum Ziel, während die Orientierung korrigiert wird.

**Zustände**:
- `INIT` → `IDLE` → `READY` → `DRIVEMOVE` → `SUCCESS`/`FAILED`

**Zentrale Variablen**:
- `__first_theta`: Initiale Orientierung (als Referenz erfasst)
- `__angle_to_mid_in_Rad`: Winkelabweichung zum Ziel
- `__distance_in_meter`: Verbleibende Entfernung zum Ziel
- `__schpieth`: Fahrtgeschwindigkeit (vom Bediner setzbar via `setSchpieth()`)

**Ausgabewerte**:
- `linear_velocity_x`: Vorwärtsgeschwindigkeitsbefehl [m/s]
- `angular_velocity_z`: Rotationsgeschwindigkeitsbefehl [rad/s]
- `distance_remaining`: Aktuelle Entfernung zum Ziel [m]
- `linear_velocity_y`: Nicht verwendet (None)

**Steuerlogik**:
```python
def calculate(self):
    # Wendet Winkelkorrektur an, wenn Abweichung Tolleranz überschreitet aber ziel noch nicht erreicht
    if abs(angle_error) > ANGLE_TOLLERANCE_DRIVE and distance_in_Meter > GOAL_DISTANCE:
        angular_velocity = p_regulator(angle_error, KP_DRIVE, MAX_ANGLE_VELOCITY_DRIVE)
    
    # Fährt vorwärts bis zur Zielentfernung
    if distance > GOAL_DISTANCE: 
        linear_velocity = __schpieth
    else:
        linear_velocity = 0.0
    
    # Fehlerbehandlung: Kamerafehler erkannt (-1.0)
    if distance == -1.0:
        linear_velocity = 0.0
        angular_velocity = 0.0
```

**Erfolgsbedingung**: 
- Entfernung zum Ziel < `GOAL_DISTANCE` UND Entfernung ≠ -1.0

**Konfigurationsparameter**:
- `ANGLE_TOLLERANCE_DRIVE`: Minimaler Winkelfehler für Korrektur
- `KP_DRIVE`: Proportionalverstärkung für P-Regler
- `MAX_ANGLE_VELOCITY_DRIVE`: Maximale Rotationsgeschwindigkeit
- `MAX_VELOCITY`: Maximale Vorwärtsfahrgeschwindigkeit
- `GOAL_DISTANCE`: Ziel-Distanz

**Spezielle Methode**:
- `setSchpieth(s)`: Setzt die Fahrtgeschwindigkeit

---

### 3. TurnLogic

**Zweck**: Führt Drehmanöver durch, um den Roboter zu einem neuen Ziel neu auszurichten.

**Zustände**:
- `INIT` → `IDLE` → `READY` → `TURNMOVE` → `SUCCESS`/`FAILED`

**Zentrale Variablen**:
- `__first_theta`: Initiale Orientierung bei Drehbeginn
- `__angle_to_Mid_in_Rad`: Winkelabweichung zum Ziel
- `__distance_in_meter`: Verbleibende Entfernung zum Ziel

**Ausgabewerte**:
- `angular_velocity_z`: Rotationsgeschwindigkeitsbefehl [rad/s]
- `turned_angle`: Gesamtwinkel gedreht
- `linear_velocity_x`: Nicht verwendet (None)
- `linear_velocity_y`: Nicht verwendet (None)

**Steuerlogik**:
```python
def calculate(self):
    # Berechnet gedrehten Winkel mit Wrap-Around-Behandlung
    turned_angle = current_theta - first_theta
    if abs(turned_angle) > π:
        turned_angle = turned_angle + 2 * π
    
    # P-Regler für Rotation
    if abs(angle_error) > ANGLE_TOLERANCE_TURN and distance_im_Meter > 1.0:
        angular_velocity = p_regulator(angle_error, KP_TURN, MAX_ANGLE_VELOCITY_TURN_INIT)
    
    # Stoppt bei Ausrichtung
    if abs(angle_error) < ANGLE_TOLERANCE_TURN and distance_im_Meter > 1.0:
        angular_velocity = 0.0
```

**Erfolgsbedingung**: 
- Winkel zum Ziel < `ANGLE_TOLLERANCE_TURN` UND Entfernung > `ORIANTATION_DISTANCE`

**Konfigurationsparameter**:
- `KP_TURN`: Proportionalverstärkung für P-Regler
- `ANGLE_TOLLERANCE_TURN`: Winkeltoleranz
- `MAX_ANGLE_VELOCITY_TURN_INIT`: Maximale Rotationsgeschwindigkeit
- `ORIANTATION_DISTANCE`: Minimale Zielentfernung

---

### 4. FollowLogic

**Zweck**: Verfolgt ein Ziel (einen anderen Roboter mit Aruco-Marker) mit Abstands- und Ausrichtungskontrolle.

**Zustände**:
- `INIT` → `IDLE` → `READY` → `FOLLOWMOVE` → `SUCCESS`/`FAILED`/`ABORT`

**Zentrale Variablen**:
- `__firstTheta`: Initiale Orientierung bei Verfolgungsbeginn
- `__angleToMidInRad`: Winkelabweichung zum Ziel
- `__distanceInMeter`: Aktuelle Entfernung zum verfolgten Ziel
- `__dominantArucoID`: ID des erkannten Aruco-Markers
- `__followDistance`: Sollabstand zum verfolgten Objekt (default: 0.2m)

**Ausgabewerte**:
- `linear_velocity_x`: Vorwärtsgeschwindigkeitsbefehl [m/s]
- `angular_velocity_z`: Rotationsgeschwindigkeitsbefehl [rad/s]
- `distance_remaining`: Aktuelle Entfernung zum verfolgten Ziel
- `linear_velocity_y`: Nicht verwendet (None)

**Steuerlogik**:
```python
def calculate(self):
    # Wendet Winkelkorrektur an
    if abs(angle_error) > ANGLE_TOLLERANCE_FOLLOW:
        angular_velocity = p_regulator(angle_error, KP_FOLLOW_ANGULAR, MAX_ANGLE_VELOCITY_FOLLOW)
    
    # Passt Abstand zum Sollwert an
    if distance != follow_distance:
        # Negation des Fehlers: positive Abweichung → negative Geschwindigkeit (bremsen)
        linear_velocity = p_regulator(-(distance_in_Meter - follow_distance_in_Meter), KP_FOLLOW_LINEAR, MAX_VELOCITY)
    
    # Fehlerbehandlung: Ziel verloren
    if distance < 0 or aruco_id == 9999:
        linear_velocity = 0.0
        angular_velocity = 0.0
```

**Erfolgsbedingung**: 
- `dominant_aruco_id == 0` (spezieller Success-Code)

**Fehlerbedingung**:
- `dominant_aruco_id == 9999` (Ziel verloren/Abbruch-Code)

**Konfigurationsparameter**:
- `ANGLE_TOLLERANCE_FOLLOW`: Winkeltoleranz für Ausrichtung
- `KP_FOLLOW_ANGULAR`: Proportionalverstärkung für P-Regler
- `KP_FOLLOW_LINEAR`: Proportionalverstärkung für P-Regler
- `MAX_ANGLE_VELOCITY_FOLLOW`: Maximale Rotationsgeschwindigkeit
- `MAX_VELOCITY`: Maximale Fahrtgeschwindigkeit

**Spezielle Methoden**:
- `setFollowDistance(d)`: Setzt den Sollabstand zum verfolgten Objekt
- `setArucoData(id)`: Setzt die ID des erkannten Markers
- `setSchpieth(v)`: Speichert Geschwindigkeit 
- `setSuccess()`: Erzwingt SUCCESS-Zustand

---

### 5. MainController

**Zweck**: High-Level State-Machine zur Koordination aller Verhaltensmodule und Zielsteuerung.

**Zustände**:
- `INIT` → `IDLE` → `INIT_ROBOT` → `DRIVE` ⇄ `TURN` → `FOLLOW` → `SUCCESS`/`FAILED`
- `PAUSE`: Pausenzustand mit Rückkehr zum vorherigen Zustand (TODO: Implementierung)

**Zentrale Variablen**:
- `_o_l_x`, `_o_l_y`, `_o_t`: Odometriedaten (Position und Orientierung)
- `_angle_in_rad`: Winkel zum Ziel
- `_distance_in_meters`: Entfernung zum Ziel
- `_dominant_aruco_id`: ID des erkannten Aruco-Markers (-1 = kein Marker)
- `_goal_status_fin`: Flag für Abschluss einer Zielaktion
- `_goal_success`: Flag für erfolgreiche Zielaktion
- `_goToFollow`: Flag zur Signalisierung des Übergangs zum FOLLOW-Modus

**Ausgabewerte**:
- `ASToCall`: Integer, der das zu aktivierende Subsystem angibt
  - `0`: InitLogic
  - `1`: DriveLogic
  - `2`: TurnLogic
  - `3`: FollowLogic
  - `4`: Übergang zu FollowLogic (intern)

**State-Machine-Ablauf**:

```
INIT → IDLE
    ↓
INIT_ROBOT
    ├─ Erfolg → DRIVE
    └─ Fehler → FAILED

DRIVE 
    ├─ Aruco-ID=69 erkannt → Vorbereitung für FOLLOW
    ├─ Erfolg ohne Follow → TURN
    ├─ Erfolg mit Follow-Trigger → FOLLOW
    └─ Fehler → FAILED

TURN 
    ├─ Erfolg → DRIVE
    └─ Fehler → FAILED

FOLLOW 
    ├─ Erfolg/Abschluss → DRIVE
    └─ Fehler → DRIVE

SUCCESS / FAILED
```

**State-Übergangslogik**:

1. **INIT_ROBOT**: Initialisiert den Roboter mit InitLogic
   - Wartet auf `_goal_status_fin == True` und `_goal_success == True`
   - Übergang zu DRIVE bei Erfolg
   - Übergang zu FAILED bei Fehler

2. **DRIVE**: Navigiert zum Ziel mit DriveLogic
   - Überwacht auf Aruco-ID=69 (Trigger für Follow-Modus)
   - Setzt `_goToFollow = True` bei Erkennung
   - Übergang zu TURN bei Erfolg ohne Follow-Trigger
   - Übergang zu FOLLOW bei Erfolg mit Follow-Trigger
   - Übergang zu FAILED bei Fehler

3. **TURN**: Dreht den Roboter zu nächstem Ziel mit TurnLogic
   - Übergang zu DRIVE bei Erfolg
   - Übergang zu FAILED bei Fehler
   - Erzeugt kontinuierlichen Fahr-Dreh-Fahr-Zyklus

4. **FOLLOW**: Folgt dem Ziel mit FollowLogic
   - Wartet auf `_goal_status_fin == True`
   - Übergang zu DRIVE zur Fortsetzung der Navigation
   - Interner Fehler-Feedback (ID=9999)

**Spezielle Methoden**:

```python
def calculate_estimated_goal_pose(last_odom_x, last_odom_y, last_odom_quaternion):
    # Berechnet eine Zielposition 6 Meter voraus in aktueller Richtung
    current_yaw = quaternion_to_yaw(last_odom_quaternion)
    goal_x = last_odom_x + 6.0 * cos(current_yaw)
    goal_y = last_odom_y + 6.0 * sin(current_yaw)
    return goal_x, goal_y, current_yaw
```

```python
def postInit():
    # Nach Initialisierung aufgerufen, um die State-Machine zu starten
    state_machine()
    setActive()
```

```python
def exit_to_pause():
    # Pausiert und speichert aktuellen Zustand für Rückkehr
    _memorized_return_state = current_state
    state = PAUSE
```

**Zielverwaltung**:
- Verwendet `setGoalStautusFinished(bool)` zum Empfangen des Abschlussstatus 
- Verwendet `setGoalSuccess(bool)` zum Empfangen des Erfolgs-/Fehlerstatus
- Verwendet `setArucoData(id)` zum Empfangen der Aruco-Marker-ID
- Setzt Flags nach Zustandsübergängen zurück

---

## Datenfluss

```
Sensoren (Kamera, Odometrie, Aruco)
    ↓
setCameraData() / setOdomData() / setArucoData()
    ↓
Logik-Module (Init/Drive/Turn/Follow)
    ├─ calculate() → Steuerbefehle berechnen
    └─ state_machine() → Zustandsverwaltung
    ↓
getOut() → Output mit isValid-Flag
    ↓
Roboter-Befehle (linear_velocity_x, angular_velocity_z)
```

### Eingabedaten

**Odometriedaten** (`setOdomData(x, y, t)`):
- `x`, `y`: Roboterposition in Metern
- `t`: Orientierung als Quaternion (wird zu Yaw konvertiert)

**Kameradaten** (`setCameraData(angle, distance)`):
- `angle_in_rad`: Winkelabweichung zum Ziel [rad]
- `distance_in_meters`: Entfernung zum Ziel [m]
- Spezialwert: `-1.0` = Kamerafehler/kein Ziel erkannt

**Aruco-Daten** (`setArucoData(id)`):
- `id`: ID des erkannten Markers
- Spezialwerte: 
  - `-1` = kein Marker erkannt
  - `69` = Trigger für Follow-Modus (in DRIVE)
  - `0` = Success-Code (in FollowLogic)
  - `9999` = Abbruch/Ziel verloren

### Ausgabebefehle

**Geschwindigkeitsbefehle**:
- `linear_velocity_x`: Vorwärts-/Rückwärtsgeschwindigkeit [m/s]
- `linear_velocity_y`: Seitengeschwindigkeit [m/s] (ungenutzt)
- `angular_velocity_z`: Rotationsgeschwindigkeit [rad/s]

**Statusinformationen**:
- `turned_angle`: Durchgeführte Gesamtrotation [rad]
- `distance_remaining`: Entfernung zum Ziel [m]

---

## Konfigurationsparameter

Das System basiert auf einem `config`-Modul mit folgenden Parametern:

### InitLogic
- `ANGLE_TOLLERANCE_INIT`: Winkeltoleranz für Ausrichtung
- `KP_INIT`: Proportionalverstärkung
- `MAX_ANGLE_VELOCITY_TURN_INIT`: Maximale Rotationsgeschwindigkeit
- `ORIANTATION_DISTANCE`: Minimale Zielentfernung für Rotation

### DriveLogic
- `ANGLE_TOLLERANCE_DRIVE`: Winkeltoleranz für Winkelkorrektur
- `KP_DRIVE`: Proportionalverstärkung 
- `MAX_ANGLE_VELOCITY_DRIVE`: Maximale Winkelgeschwindigkeit
- `MAX_VELOCITY`: Maximale Vorwärtsfahrgeschwindigkeit
- `GOAL_DISTANCE`: Ziel-Distanz
- `MINIMAL_SPEED_TO_SET`: Standard-Fahrtgeschwindigkeit

### TurnLogic
- `ANGLE_TOLLERANCE_TURN`: Winkeltoleranz für Ausrichtung
- `KP_TURN`: Proportionalverstärkung
- `MAX_ANGLE_VELOCITY_TURN_INIT`: Maximale Rotationsgeschwindigkeit
- `ORIANTATION_DISTANCE`: Minimale Zielentfernung

### FollowLogic
- `ANGLE_TOLLERANCE_FOLLOW`: Winkeltoleranz für Ausrichtung
- `KP_FOLLOW_ANGULAR`: Proportionalverstärkung für Winkelkontrolle
- `KP_FOLLOW_LINEAR`: Proportionalverstärkung für Abstandskontrolle
- `MAX_ANGLE_VELOCITY_FOLLOW`: Maximale Rotationsgeschwindigkeit
- `MAX_VELOCITY`: Maximale Fahrtgeschwindigkeit

---

## Steueralgorithmus

Alle Module verwenden einen Proportional-(P)-Regler über `p_regulator()`:

```python
def p_regulator(error, kp, max_output):
    # Wird zur Regelung der Geschwindigkeiten genutzt
    
    output = -kp * error
    
    if output > max_output:
        output = max_output
    elif output < -max_output:
        output = -max_output
    
    return output
```

Diese Funktion:
- Multipliziert den Fehler mit der Proportionalverstärkung (kp)
- Begrenzt das Ergebnis auf ±max_output
- Bietet sanfte, responsive Steuerung
- Verwendet Negation für intuitive Regelung

---

## Integration mit ROS2

Die Logic-Module sind für die Integration mit ROS2 Action Servern konzipiert. Jedes Modul wird von einem entsprechenden Action Server verwendet:

- **InitLogic** → verwendet von `InitActionServer`
- **DriveLogic** → verwendet von `DriveActionServer`
- **TurnLogic** → verwendet von `TurnActionServer`
- **FollowLogic** → verwendet von `FollowActionServer`
- **MainController** → verwendet von `MuriActionHandler`

Die Action Server übernehmen:
- Empfang von Sensor- und Odometriedaten über ROS2 Topics
- Weitergabe der Daten an die Logic-Module
- Publikation der berechneten Geschwindigkeitsbefehle
- Verwaltung der Action-Goals und Feedback
- Koordination der Zustandsübergänge

Siehe `ros.md` für Details zur ROS2-Integration.

---

## Bekannte Probleme und TODOs

### Design-TODOs

1. **InitLogic**: Fehlende FAILED-Zustandslogik für übermäßige Rotation
2. **MainController**: 
   - `_memorized_return_state` wird deklariert, aber nicht genutzt

### Verbesserungsmöglichkeiten

1. **Error-Handling**: Fehlerbehandlung in FAILED-Zuständen ausbaubar
2. **Logging**: Debug-Print-Statements könnten durch strukturiertes Logging ersetzt werden

---

## Abhängigkeiten

- `enum.Enum`: Für Zustandsdefinitionen
- `muri_logics.logic_interface`: Basis-Interfaces (LogicInterface, ExtendedLogicInterface)
- `muri_logics.general_funcs`: 
  - `quaternion_to_yaw()`: Konvertiert Quaternion zu Yaw-Winkel
  - `p_regulator()`: Proportionalregler
- `math`: Mathematische Operationen (sin, cos, pi)
- `time`: Für Timing (falls benötigt)
- `config`: Konfigurationsparameter

---

## Best Practices

1. **Immer `isValid` prüfen**, bevor Ausgabewerte verwendet werden

2. **Module zurücksetzen** vor Reaktivierung, um sauberen Zustand zu gewährleisten

3. **Zustandsübergänge überwachen** für Debugging

4. **Toleranzen angemessen konfigurieren** angepasst je nach situation

5. **FAILED-Zustände behandeln** im Produktionscode

6. **`getActiveState()` verwenden**, um Modulstatus zu überwachen

7. **Fehlerbehandlung bei Kameradaten**

8. **MainController Aruco-Trigger**: Aruco-ID=69 triggert automatisch 

---

## State Transition Diagramme

### InitLogic
```
INIT ──→ IDLE ──(setActive)──→ READY ──→ INITMOVE ──┬─→ SUCCESS
                                                     │
                                                     └─→ FAILED
```

### DriveLogic
```
INIT ──→ IDLE ──(setActive)──→ READY ──→ DRIVEMOVE ──┬─→ SUCCESS
                                                      │
                                                      └─→ FAILED
```

### TurnLogic
```
INIT ──→ IDLE ──(setActive)──→ READY ──→ TURNMOVE ──┬─→ SUCCESS
                                                     │
                                                     └─→ FAILED
```

### FollowLogic
```
INIT ──→ IDLE ──(setActive)──→ READY ──→ FOLLOWMOVE ──┬─→ SUCCESS
                                                      ├─→ FAILED
                                                      └─→ ABORT
```

### MainController 
```
INIT ──> IDLE ──(setActive)──> INIT_ROBOT
                                       │                             
                                       |
                                       ↓
           ┌──────────────────────> DRIVE <───┐
           |                           |      |
           |    ┌──(if ArucoID is 69)──┤      |
           |    ↓                      ↓      |    
           | FOLLOW                  TURN ────┘
           |    | 
           └────┘                   
```

## Änderungshistorie und Versioning

**Dokumentversion**: 2.0.0
**Zuletzt aktualisiert**: Dezember 2025

### Änderungen in v2.0.0
- ✅ Vollständige Dokumentation von FollowLogic hinzugefügt
- ✅ MainController-Dokumentation erweitert (Aruco-Triggerlogik)
- ✅ ExtendedLogicInterface dokumentiert
- ✅ State-Diagramme hinzugefügt
- ✅ Fehlerbehandlung (Distance=-1.0, Aruco=9999) dokumentiert
- ✅ `setSchpieth()`-Verhalten in DriveLogic dokumentiert
- ✅ Kritische Fehler kennzeichnet und Fixes vorgeschlagen
- ⚠️ Tippfehler in MainController (`setGoalStautusFinished`) dokumentiert

---

## FAQ

**F: Was ist der Unterschied zwischen `distance = -1.0` und `aruco_id = 9999`?**  
A: `-1.0` ist ein Fehler der Kamera/Vision (kein Ziel erkannt). `9999` ist ein Abbruch-Befehl im Follow-Modus (Ziel verloren oder Abbruch-Signal).

**F: Wie kann ich die Fahrtgeschwindigkeit ändern?**  
A: In DriveLogic: `drive_logic.setSchpieth(new_speed)` (nur außerhalb DRIVEMOVE)
In FollowLogic: `follow_logic.setSchpieth(new_speed)` (nur außerhalb FOLLOWMOVE)

**F: Was sind typische Werte für die Toleranzen?**  
A: Abhängig vom Roboter, typischerweise:
- `ANGLE_TOLLERANCE_*`: 0.1 - 0.3 rad (5-17°)
- `GOAL_DISTANCE`: 0.1 - 0.5 m
- `ORIANTATION_DISTANCE`: 1.0 m

**F: Wie triggert man den Follow-Modus?**  
A: Aruco-Marker mit ID `69` im DRIVE-State erkennen. MainController schaltet automatisch um.

**F: Was passiert bei Fehler in FollowLogic?**  
A: Bei `aruco_id = 9999` → FAILED → zurück zu IDLE → MainController kehrt zu DRIVE zurück.

---

## Kontakt & Support

Für Fragen oder Probleme siehe die `ros.md` für ROS2-Integration und Debugging-Hinweise.