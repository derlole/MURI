# Roboter-Steuerungslogik Dokumentation

## Übersicht

Diese Dokumentation beschreibt die Roboter-Steuerungslogik, das aus vier State-Machine-Modulen besteht, die zusammenarbeiten, um das Navigationsverhalten eines mobilen Roboters zu steuern. Das System implementiert eine hierarchische Steuerungsarchitektur mit einem Haupt-Controller, der drei spezialisierte Verhaltensmodule koordiniert: Initialisierung, Fahren und Drehen.

## Systemarchitektur

Das Steuerungssystem folgt einer modularen State-Machine-Architektur:

- **MainController**: Übergeordneter Orchestrator, der Übergänge zwischen verschiedenen Verhaltenszuständen verwaltet
- **InitLogic**: Behandelt die initiale Roboter-Ausrichtung zu einem Ziel
- **DriveLogic**: Steuert die Vorwärtsbewegung zum Ziel unter Beibehaltung der Ausrichtung
- **TurnLogic**: Verwaltet Drehmanöver zur Neuausrichtung des Roboters

Alle Module implementieren das `LogicInterface`.

## Gemeinsame Komponenten

### State Machine Pattern

Jedes Logik-Modul implementiert ein konsistentes State-Machine-Muster mit folgenden Zuständen:

- `INIT`: Initialer Zustand für Variableninitialisierung
- `IDLE`: Wartezustand, bereit zur Aktivierung
- `RAEDY`: Übergangszustand zur Vorbereitung des aktiven Betriebs (Hinweis: Rechtschreibfehler im Code)
- `*MOVE`: Aktiver Ausführungszustand (INITMOVE, DRIVEMOVE, TURNMOVE)
- `SUCCESS`: Ziel erfolgreich erreicht
- `FAILED`: Fehler- oder Fehlschlagbedingung

### Output Interface

Alle Output-Klassen (`InitOut`, `DriveOut`, `TurnOut`, `MainOut`) implementieren:

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
- `setCameraData(...)`: Setzt Kamera-/Sensordaten
- `calculate()`: Berechnet Steuerbefehle
- `state_machine()`: Führt die State-Machine-Logik aus

## Modul-Details

### 1. InitLogic

**Zweck**: Dreht den Roboter, um initial auf ein erkanntes Ziel auszurichten.

**Zustände**:
- `INIT` → `IDLE` → `RAEDY` → `INITMOVE` → `SUCCESS`/`FAILED`

**Zentrale Variablen**:
- `__firstTheta`: Initialer Orientierungswinkel (beim Aktivieren aufgezeichnet)
- `__positionTheta`: Aktuelle Roboter-Orientierung
- `__angle_to_Mid_in_Rad`: Winkelabweichung zum Ziel
- `__distance_in_Meter`: Entfernung zum Ziel

**Ausgabewerte**:
- `angular_velocity_z`: Rotationsbefehl
- `turned_angle`: Gesamtwinkel seit Aktivierung gedreht
- `linear_velocity_x`: Nicht verwendet (None)
- `linear_velocity_y`: Nicht verwendet (None)

**Steuerlogik**:
```python
def calculate(self):
    # Berechnet gedrehten Winkel vom initialen Winkel
    tuerndAngle = current_theta - first_theta
    
    # Verwendet P-Regler wenn Winkelfehler Toleranz überschreitet
    if abs(angle_error) > ANGLE_TOLERANCE_INIT:
        angular_velocity = p_regulator(angle_error, KP_INIT, MAX_VELOCITY)
    
    # Stoppt bei Ausrichtung innerhalb der Toleranz
    if abs(angle_error) < ANGLE_TOLERANCE_INIT:
        angular_velocity = 0.0
```

**Erfolgsbedingung**: 
- Winkel zum Ziel < `ANGLE_TOLLERANCE_INIT` UND Entfernung > 1.0m


### 2. DriveLogic

**Zweck**: Fährt den Roboter vorwärts zum Ziel, während die Orientierung korrigiert wird.

**Zustände**:
- `INIT` → `IDLE` → `RAEDY` → `DRIVEMOVE` → `SUCCESS`/`FAILED`

**Zentrale Variablen**:
- `__first_Theta`: Initiale Orientierung (als Referenz erfasst)
- `__angle_to_Mid_in_Rad`: Winkelabweichung zum Ziel
- `__distance_in_Meter`: Verbleibende Entfernung zum Ziel

**Ausgabewerte**:
- `linear_velocity_x`: Vorwärtsgeschwindigkeitsbefehl
- `angular_velocity_z`: Rotationsgeschwindikeitsbefehl
- `distance_remaining`: Aktuelle Entfernung zum Ziel
- `linear_velocity_y`: Nicht verwendet (None)

**Steuerlogik**:
```python
def calculate(self):
    # Wendet Winkelkorrektur an, wenn Abweichung Schwelle überschreitet
    if abs(angle_error) > ANGLE_TOLLERANCE_DRIVE:
        angular_velocity = p_regulator(angle_error, KP_DRIVE, MAX_ANGULAR_VEL)
    
    # Fährt vorwärts bis zur Zielentfernung
    if distance > GOAL_DISTANCE:
        linear_velocity = MAX_VELOCITY
    else:
        linear_velocity = 0.0
```

**Erfolgsbedingung**: 
- Entfernung zum Ziel < `GOAL_DISTANCE`

**Konfigurationsparameter**:
- `ANGLE_TOLLERANCE_DRIVE`: Minimaler Winkelfehler für Korrektur
- `KP_DRIVE`: Proportionalverstärkung für Winkelsteuerung
- `MAX_ANGLE_VELOCITY_DRIVE`: Maximale Rotationsgeschwindigkeit
- `MAX_VELOCITY`: Maximale Vorwärtsfahrgeschwindigkeit
- `GOAL_DISTANCE`: Ziel-Distanz

### 3. TurnLogic

**Zweck**: Führt Drehmanöver durch, um den Roboter zu einem neuen Ziel neu auszurichten.

**Zustände**:
- `INIT` → `IDLE` → `RAEDY` → `TURNMOVE` → `SUCCESS`/`FAILED`

**Zentrale Variablen**:
- `__first_Theta`: Initiale Orientierung bei Drehbeginn
- `__angle_to_Mid_in_Rad`: Winkelabweichung zum Ziel
- `__distance_in_meter`: Verbleibende Entfernung zum Ziel

**Ausgabewerte**:
- `angular_velocity_z`: Rotationsgeschwindikeitsbefehl
- `turened_angle`: Gesamtwinkel gedreht (Hinweis: Tippfehler im Schlüsselnamen)
- `linear_velocity_x`: Nicht verwendet (None)
- `linear_velocity_y`: Nicht verwendet (None)

**Steuerlogik**:
```python
def calculate(self):
    # Berechnet gedrehten Winkel mit Wrap-Around-Behandlung
    tuerndAngle = current_theta - first_theta
    if abs(tuerndAngle) > pi:
        tuerndAngle += 2 * pi
    
    # P-Regler für sanfte Rotation
    if abs(angle_error) > ANGLE_TOLERANCE_TURN:
        angular_velocity = p_regulator(angle_error, KP_TURN, MAX_VELOCITY)
    
    # Stoppt bei Ausrichtung
    if abs(angle_error) < ANGLE_TOLERANCE_TURN:
        angular_velocity = 0.0
```

**Erfolgsbedingung**: 
- Winkel zum Ziel < `ANGLE_TOLLERANCE_TURN` UND Entfernung > 1.0m

**Konfigurationsparameter**:
- `KP_TURN`: Proportionalverstärkung
- `ANGLE_TOLLERANCE_TURN`: Winkeltoleranz für Abschluss
- `MAX_ANGLE_VELOCITY_TURN_INIT`: Maximale Rotationsgeschwindigkeit

### 4. MainController

**Zweck**: High-Level State-Machine zur Koordination der Verhaltensmodule.

**Zustände**:
- `INIT` → `IDLE` → `INIT_ROBOT` → `DRIVE` ⇄ `TURN` → `SUCCESS`/`FAILED`
- `PAUSE`: Pausenzustand (TODO)

**Zentrale Variablen**:
- `_o_l_x`, `_o_l_y`, `_o_t`: Odometriedaten (Position und Orientierung)
- `_angle_in_rad`: Winkel zum Ziel
- `_distance_in_meters`: Entfernung zum Ziel
- `_goal_status_fin`: Flag, das anzeigt, dass die Zielaktion abgeschlossen ist
- `_goal_success`: Flag, das anzeigt, dass die Zielaktion erfolgreich war

**Ausgabewerte**:
- `ASToCall`: Ganzzahl, die angibt, welches Subsystem aktiviert werden soll
  - `0`: InitLogic
  - `1`: DriveLogic
  - `2`: TurnLogic

**State-Machine-Ablauf**:

1. **INIT_ROBOT**: Aktiviert InitLogic (ASToCall=0)
   - Wartet auf Zielabschluss
   - Übergang zu DRIVE bei Erfolg

2. **DRIVE**: Aktiviert DriveLogic (ASToCall=1)
   - Überwacht geschätzte Probleme 
   - Übergang zu TURN bei Erfolg

3. **TURN**: Aktiviert TurnLogic (ASToCall=2)
   - Übergang zurück zu DRIVE bei Erfolg
   - Erzeugt kontinuierlichen Fahr-Dreh-Fahr-Zyklus

**Spezielle Methoden**:

```python
def calculate_estimated_goal_pose(last_odom_x, last_odom_y, last_odom_quaternion):
    # Berechnet eine Zielposition 6 Meter voraus in aktueller Richtung
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

**Zielverwaltung**:
- Verwendet `setGoalStautusFinished(bool)` zum Empfangen des Abschlussstatus
- Verwendet `setGoalSuccess(bool)` zum Empfangen des Erfolgs-/Fehlerstatus
- Setzt beide Flags nach Zustandsübergängen zurück

## Datenfluss

```
Kamera/Sensoren → setCameraData()
                                ↓
Odometrie → setOdomData() → Logik-Modul → calculate() → Ausgabe
                                                            ↓
                                                   Roboter-Befehle
```

### Eingabedaten

**Odometriedaten**:
- `x`, `y`: Roboterposition in Metern
- `t`: Orientierung als Quaternion (wird zu Yaw konvertiert)

**Kameradaten**:
- `angle_in_rad`: Winkelabweichung zum Ziel
- `distance_in_meters`: Entfernung zum Ziel

### Ausgabebefehle

**Geschwindigkeitsbefehle**:
- `linear_velocity_x`: Vorwärts-/Rückwärtsgeschwindigkeit [m/s]
- `linear_velocity_y`: Seitengeschwindigkeit [m/s] (ungenutzt)
- `angular_velocity_z`: Rotationsgeschwindigkeit [rad/s]

**Statusinformationen**:
- `turned_angle`: Durchgeführte Gesamtrotation
- `distance_remaining`: Entfernung zum Ziel

## Konfigurationsparameter

Das System basiert auf einem `config`-Modul mit folgenden Parametern:

**InitLogic**:
- `ANGLE_TOLLERANCE_INIT`: Winkeltoleranz für Ausrichtung
- `KP_INIT`: Proportionalverstärkung
- `MAX_ANGLE_VELOCITY_TURN_INIT`: Maximale Rotationsgeschwindigkeit

**DriveLogic**:
- `ANGLE_TOLLERANCE_DRIVE`: Winkeltoleranz für Winkelkorrektur
- `KP_DRIVE`: Proportionalverstärkung für Winkelkorrektur
- `MAX_ANGLE_VELOCITY_DRIVE`: Maximale Winkelgeschwindigkeit
- `MAX_VELOCITY`: Maximale Vorwärtsfahrgeschwindigkeit
- `GOAL_DISTANCE`: Ziel-Distanz

**TurnLogic**:
- `ANGLE_TOLLERANCE_TURN`: Winkeltoleranz für Ausrichtung
- `KP_TURN`: Proportionalverstärkung
- `MAX_ANGLE_VELOCITY_TURN_INIT`: Maximale Rotationsgeschwindigkeit

## Steueralgorithmus

Alle Module verwenden einen Proportional-(P)-Regler über `p_regulator()`:

```python
def p_regulator(error, kp, max_output):
    # Wird zur Regelung der Winkelgeschwindigkeiten genutzt

    output = -kp * error

    if output > max_output:
        output = max_output
    elif output < -max_output:
        output = -max_output
    
    return output
```

Diese Funktion:
- Multipliziert den Fehler mit der Proportionalverstärkung (kp)
- Begrenzt das Ergebnis auf ±max_velocity
- Bietet sanfte, responsive Steuerung

## Integration mit ROS2

Die Logic-Module sind für die Integration mit ROS2 Action Servern konzipiert. Jedes Modul wird von einem entsprechenden Action Server verwendet:

- **InitLogic** → verwendet von `InitActionServer`
- **DriveLogic** → verwendet von `DriveActionServer`
- **TurnLogic** → verwendet von `TurnActionServer`
- **MainController** → verwendet von `MuriActionHandler`

Die Action Server übernehmen:
- Empfang von Sensor- und Odometriedaten über ROS2 Topics
- Weitergabe der Daten an die Logic-Module
- Publikation der berechneten Geschwindigkeitsbefehle
- Verwaltung der Action-Goals und Feedback

Siehe `ros.md` für Details zur ROS2-Integration.

## Verwendungsbeispiel (Standalone)

```python
# Main-Controller initialisieren
main_ctrl = MainController()

# Verhaltensmodule initialisieren
init_logic = InitLogic()
drive_logic = DriveLogic()
turn_logic = TurnLogic()

# In der Steuerungsschleife:
main_ctrl.setOdomData(x, y, quaternion)
main_ctrl.setCameraData(angle, distance)
main_ctrl.state_machine()

output = main_ctrl.getOut()
if output.isValid:
    active_module = output.values['ASToCall']
    
    # Entsprechendes Modul aktivieren
    if active_module == 0:
        init_logic.setActive()
        init_logic.setOdomData(x, y, quaternion)
        init_logic.setCameraData(angle, distance)
        init_logic.state_machine()
        commands = init_logic.getOut()
    
    # Ähnlich für drive_logic (1) und turn_logic (2)
```

## Bekannte Probleme und TODOs

1. **InitLogic**: Fehlende FAILED-Zustandslogik für übermäßige Rotation
2. **TurnLogic**: Tippfehler im Output-Schlüssel `'turened_angle'` (sollte `'turned_angle'` sein)
   - Auch Inkonsistenz in `resetOut()`: verwendet `'turned_angle'` statt `'turened_angle'`
3. **MainController**: 
   - `PAUSE`-Zustand nicht implementiert
   - Tippfehler in `setGoalStautusFinished` (sollte `setGoalStatusFinished` sein)
4. **Rechtschreibfehler**: `RAEDY` sollte `READY` sein (in allen Modulen)

## Abhängigkeiten

- `enum.Enum`: Für Zustandsdefinitionen
- `muri_logics.logic_interface`: Basis-Interfaces
- `muri_logics.general_funcs`: 
  - `quaternion_to_yaw()`: Konvertiert Quaternion zu Yaw-Winkel
  - `p_regulator()`: Proportionalregler
- `math`: Mathematische Operationen
- `config`: Konfigurationsparameter

## Best Practices

1. **Immer `isValid` prüfen**, bevor Ausgabewerte verwendet werden
2. **Module zurücksetzen** vor Reaktivierung, um sauberen Zustand zu gewährleisten
3. **Zustandsübergänge überwachen** für Debugging
4. **Toleranzen angemessen konfigurieren** für die Fähigkeiten Ihres Roboters
5. **FAILED-Zustände behandeln** im Produktionscode
6. **`getActiveState()` verwenden**, um Modulstatus zu überwachen
7. **Konsistenz bei Schlüsselnamen** beachten (z.B. turned_angle vs. turened_angle)