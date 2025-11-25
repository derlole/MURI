# ROS2 Action Server Architektur - Dokumentation

## Überblick

Diese Dokumentation beschreibt die Implementierung einer ROS2-basierten Action Server Architektur für die Steuerung eines autonomen Roboters namens "MURI". Das System besteht aus drei spezialisierten Action Servern (INIT, DRIVE, TURN) und einem zentralen Action Handler, der die Koordination übernimmt.

## Systemarchitektur

### Komponenten

1. **InitActionServer** - Initialisierung des Roboters
2. **DriveActionServer** - Fahrbewegungen
3. **TurnActionServer** - Drehbewegungen
4. **MuriActionHandler** - Zentrale Koordination und Steuerung

### Kommunikationsdiagramm

```
┌─────────────────────────────────────────────────────────────┐
│                    MuriActionHandler                        │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐       │
│  │ Init Client  │  │ Drive Client │  │ Turn Client  │       │
│  └──────┬───────┘  └───────┬──────┘  └────────┬─────┘       │
└─────────┼──────────────────┼──────────────────┼─────────────┘
          │                  │                  │
          ▼                  ▼                  ▼
┌─────────────────┐ ┌─────────────────┐ ┌─────────────────┐
│ InitActionServer│ │DriveActionServer│ │ TurnActionServer│
└─────────┬───────┘ └─────────┬───────┘ └─────────┬───────┘
          │                   │                   │
          └───────────────────┴───────────────────┘
                              │
                              ▼
                         /cmd_vel
                              │
                              ▼
                      Roboter Hardware
```

## Detaillierte Komponentenbeschreibung

### 1. DriveActionServer

**Zweck:** Führt Fahrbewegungen zu einer Zielposition aus.

#### Konfiguration

- **Node-Name:** `muri_drive_action_server`
- **Action-Typ:** `DRIVE`
- **Action-Name:** `muri_drive`
- **Publisher:** `/cmd_vel` (Twist)
- **Subscriber:** 
  - `/odom` (Odometry)
  - `/muri_picture_data` (PictureData)
- **Timer-Rate:** 10 Hz (0.1s)

#### Klassenstruktur

```python
class DriveActionServer(Node):
    def __init__(self, logic: LogicInterface)
```

**Parameter:**
- `logic`: LogicInterface-Implementierung für die Zustandsmaschine

#### Wichtige Methoden

##### `timer_callback_asd()`
Hauptschleife, die mit 10 Hz aufgerufen wird:
- Prüft ob ein aktives Goal vorhanden ist
- Verarbeitet Cancel-Anfragen
- Führt Zustandsmaschine aus (`state_machine()`)
- Publiziert Geschwindigkeitsbefehle an `/cmd_vel`
- Sendet Feedback zum Client
- Behandelt SUCCESS/FAILED-Zustände

##### `execute_callback(goal_handle)`
Blockierender Callback für die Goal-Ausführung:
- Setzt `_goal_handle` für den Timer
- Wartet bis Goal abgeschlossen ist
- Gibt Ergebnis zurück

##### `goal_callback(goal_request)`
Entscheidet über Annahme/Ablehnung neuer Goals:
- Lehnt ab, wenn bereits ein Goal aktiv ist
- Resettet und aktiviert Logic bei Annahme
- Gibt `GoalResponse.ACCEPT` oder `GoalResponse.REJECT` zurück

##### `cancel_callback(goal_handle)`
Behandelt Cancel-Anfragen:
- Gibt immer `CancelResponse.ACCEPT` zurück

##### `listener_callback_odom_asd(msg)`
Verarbeitet Odometry-Daten:
- Speichert Position (x, y) und Orientierung
- Übergibt Daten an Logic-Komponente

##### `listener_callback_picture_data_asd(msg)`
Verarbeitet Kamera-Daten:
- Speichert Winkel und Distanz
- Übergibt Daten an Logic-Komponente

#### Feedback

```python
DRIVE.Feedback()
    - distance_remaining: float  # Verbleibende Distanz zum Ziel
```

#### Result

```python
DRIVE.Result()
    - success: bool  # True bei Erfolg, False bei Fehler
```

#### Zustände

- **SUCCESS:** Ziel erfolgreich erreicht
- **FAILED:** Fehler bei der Ausführung

#### Besonderheiten

- Verwendet `MutuallyExclusiveCallbackGroup` für Thread-Sicherheit
- Nur ein aktives Goal gleichzeitig
- ERR_THRESHOLD = 5 (definiert, aber nicht verwendet im gezeigten Code)

---

### 2. InitActionServer

**Zweck:** Führt die Initialisierungssequenz des Roboters durch (typischerweise eine 360°-Drehung zur Umgebungserkennung).

#### Konfiguration

- **Node-Name:** `muri_init_action_server`
- **Action-Typ:** `INIT`
- **Action-Name:** `muri_init`
- **Publisher:** `/cmd_vel` (Twist)
- **Subscriber:**
  - `/odom` (Odometry)
  - `/muri_picture_data` (PictureData)
- **Timer-Rate:** 10 Hz (0.1s)

#### Klassenstruktur

```python
class InitActionServer(Node):
    def __init__(self, logic: LogicInterface)
```

#### Wichtige Methoden

##### `timer_callback_asi()`
Hauptschleife (10 Hz):
- Prüft aktives Goal
- Verarbeitet Cancel-Anfragen
- Führt Zustandsmaschine aus
- Publiziert Geschwindigkeitsbefehle
- Sendet Feedback mit gedrehtem Winkel
- Behandelt SUCCESS/FAILED-Zustände

##### `execute_callback(goal_handle)`
Blockierender Callback:
- Wartet auf `_goal_result` (nicht `_goal_exiting`)
- Gibt Ergebnis zurück

#### Feedback

```python
INIT.Feedback()
    - turned_angle: float  # Bereits gedrehter Winkel
```

#### Result

```python
INIT.Result()
    - success: bool
```

#### Zustände

- **SUCCESS:** Initialisierung erfolgreich abgeschlossen
- **FAILED:** Fehler bei der Initialisierung

---

### 3. TurnActionServer

**Zweck:** Führt Drehbewegungen um einen bestimmten Winkel aus.

#### Konfiguration

- **Node-Name:** `muri_turn_action_server`
- **Action-Typ:** `TURN`
- **Action-Name:** `muri_turn`
- **Publisher:** `/cmd_vel` (Twist)
- **Subscriber:**
  - `/odom` (Odometry)
  - `/muri_picture_data` (PictureData)
- **Timer-Rate:** 10 Hz (0.1s)

#### Klassenstruktur

```python
class TurnActionServer(Node):
    def __init__(self, locic: LogicInterface)  # Typo im Original: locic
```

#### Wichtige Methoden

##### `timer_callback_ast()`
Hauptschleife (10 Hz):
- Identisch zu DriveActionServer
- Publiziert `moved_angle` als Feedback

#### Feedback

```python
TURN.Feedback()
    - moved_angle: float  # Bereits gedrehter Winkel
```

#### Result

```python
TURN.Result()
    - success: bool
```

#### Zustände

- **SUCCESS:** Drehung erfolgreich abgeschlossen
- **FAILED:** Fehler bei der Drehung

#### Besonderheiten

- Verwendet `MultiThreadedExecutor` ohne explizite Thread-Anzahl (Standard)

---

### 4. MuriActionHandler

**Zweck:** Zentrale Steuerungseinheit, die entscheidet, welcher Action Server wann aufgerufen wird.

#### Konfiguration

- **Node-Name:** `muri_action_handler`
- **Action Clients:**
  - `muri_drive` (DRIVE)
  - `muri_turn` (TURN)
  - `muri_init` (INIT)
- **Subscriber:**
  - `/odom` (Odometry)
  - `/muri_picture_data` (PictureData)
- **Timer-Rate:** 10 Hz (0.1s)

#### Klassenstruktur

```python
class MuriActionHandler(Node):
    def __init__(self, logic: LogicInterface)
```

**Parameter:**
- `logic`: MainController-Instanz

#### Hauptschleife: `main_loop_ah()`

```python
def main_loop_ah(self):
    self.main_controller.state_machine()
    out = self.main_controller.getOut()
    
    if self.main_controller.getActiveState() == MainStates.IDLE:
        self.main_controller.postInit()
    
    if out.outValid() and out.values != {}:
        if out.values['ASToCall'] == 0:
            self.send_init_goal()
        elif out.values['ASToCall'] == 1:
            self.send_drive_goal()
        elif out.values['ASToCall'] == 2:
            self.send_turn_goal()
```

**Ablauf:**
1. Führt Zustandsmaschine des MainControllers aus
2. Prüft ob Ausgabe gültig ist
3. Ruft bei IDLE-Zustand `postInit()` auf
4. Sendet Goal an entsprechenden Action Server basierend auf `ASToCall`

#### Goal-Versendung

##### `send_init_goal()`
```python
def send_init_goal(self):
    init_goal = INIT.Goal()
    self._action_client_init.wait_for_server()
    self._init_send_promise = self._action_client_init.send_goal_async(
        init_goal, 
        feedback_callback=self.init_feedback_callback
    )
    self._init_send_promise.add_done_callback(self.init_goal_response_callback)
```

##### `send_drive_goal()`
```python
def send_drive_goal(self):
    gx, gy, gt = self.main_controller.calculate_estimated_goal_pose(
        last_odom_x=self.last_odom.pose.pose.position.x,
        last_odom_y=self.last_odom.pose.pose.position.y,
        last_odom_quaternion=self.last_odom.pose.pose.orientation
    )
    drive_goal = DRIVE.Goal()
    drive_goal.target_pose.x = float(gx)
    drive_goal.target_pose.y = float(gy)
    drive_goal.target_pose.theta = float(gt)
    
    self._action_client_drive.wait_for_server()
    self._drive_send_promise = self._action_client_drive.send_goal_async(
        drive_goal,
        feedback_callback=self.drive_feedback_callback
    )
    self._drive_send_promise.add_done_callback(self.drive_goal_response_callback)
```

##### `send_turn_goal()`
```python
def send_turn_goal(self):
    gx, gy, gt = self.main_controller.calculate_estimated_goal_pose(...)
    turn_goal = TURN.Goal()
    turn_goal.target_angle = float(gt)
    
    self._action_client_turn.wait_for_server()
    self._turn_send_promise = self._action_client_turn.send_goal_async(
        turn_goal,
        feedback_callback=self.turn_feedback_callback
    )
    self._turn_send_promise.add_done_callback(self.turn_goal_response_callback)
```

#### Callback-Struktur

Für jeden Action Client existiert eine dreistufige Callback-Struktur:

1. **Feedback Callback:** Empfängt kontinuierliches Feedback während der Ausführung
2. **Goal Response Callback:** Wird aufgerufen, wenn Server Goal akzeptiert/ablehnt
3. **Result Callback:** Wird aufgerufen, wenn Goal abgeschlossen ist

##### Beispiel: Drive Callbacks

```python
def drive_feedback_callback(self, feedback_msg):
    self.get_logger().info('Drive: ' + str(feedback_msg))

def drive_goal_response_callback(self, promise):
    goal_handle = promise.result()
    if not goal_handle.accepted:
        self.get_logger().info('Rej: drive-goal')
        return
    
    self.get_logger().info('Acc: drive-goal')
    self._drive_result_promise = goal_handle.get_result_async()
    self._drive_result_promise.add_done_callback(self.drive_result_callback)

def drive_result_callback(self, promise):
    result = promise.result().result
    self.main_controller.setGoalStautusFinished(True)  # Typo im Original
    self.main_controller.setGoalSuccess(True)
    self.get_logger().info('Drive result: {0}'.format(result))
```

#### Datenverarbeitung

```python
def listener_callback_picture_data_ah(self, msg):
    self.last_picture_data = msg
    self.main_controller.setCameraData(msg.angle_in_rad, msg.distance_in_meters)

def listener_callback_odom_ah(self, msg):
    self.last_odom = msg
    self.main_controller.setOdomData(
        msg.pose.pose.position.x,
        msg.pose.pose.position.y,
        msg.pose.pose.orientation
    )
```

---

## LogicInterface

Alle Action Server und der Action Handler nutzen eine gemeinsame Schnittstelle:

```python
class LogicInterface:
    def state_machine(self)
    def getOut(self) -> Output
    def getActiveState(self) -> State
    def reset(self)
    def setActive(self)
    def setOdomData(self, x, y, orientation)
    def setCameraData(self, angle, distance)
```

### Output-Struktur

```python
class Output:
    values: dict
    def outValid(self) -> bool
    def resetOut(self)
```

**Typische Output-Werte:**
- `linear_velocity_x`: Geschwindigkeit in x-Richtung
- `linear_velocity_y`: Geschwindigkeit in y-Richtung
- `angular_velocity_z`: Winkelgeschwindigkeit
- `distance_remaining`: Verbleibende Distanz (DRIVE)
- `turned_angle`: Gedrehter Winkel (INIT/TURN)
- `ASToCall`: Auszuführender Action Server (0=INIT, 1=DRIVE, 2=TURN)

---

## Threading und Synchronisation

### MultiThreadedExecutor

Alle Nodes nutzen `MultiThreadedExecutor` für parallele Verarbeitung:

```python
executor = MultiThreadedExecutor(num_threads=4)  # DriveActionServer, InitActionServer
executor = MultiThreadedExecutor()               # TurnActionServer (Standard-Threads)
```

### MutuallyExclusiveCallbackGroup

Action Server verwenden `MutuallyExclusiveCallbackGroup` für:
- Action Server Callbacks
- Timer Callbacks

Dies verhindert parallele Ausführung von Callbacks innerhalb derselben Gruppe.

---

## Nachrichtentypen


### PictureData (muri_dev_interfaces/PictureData)

```python
angle_in_rad: float      # Winkel zum erkannten Objekt
distance_in_meters: float # Distanz zum erkannten Objekt
```

---

## Fehlerbehandlung und Shutdown

### Graceful Shutdown

Alle Nodes implementieren sauberes Herunterfahren:

```python
try:
    executor.spin()
except (KeyboardInterrupt, ExternalShutdownException):
    node.get_logger().info('Interrupt received, shutting down.')
finally:
    node.destroy_node()
    rclpy.shutdown()
```

### Cancel-Handling

Action Server behandeln Cancel-Anfragen im Timer-Callback:

```python
if self._goal_handle.is_cancel_requested:
    self.get_logger().info('Canc: <action>-goal.')
    self._goal_handle.canceled()
    self._goal_result = <ACTION>.Result()
    self._goal_result.success = False
    self._goal_exiting = True
    self._goal_handle = None
    return
```

---

## Best Practices und Hinweise

### 1. Goal-Verwaltung

- Nur ein aktives Goal pro Action Server
- Neue Goals werden abgelehnt, wenn bereits eines aktiv ist
- Goals können jederzeit abgebrochen werden

### 2. Datenfluss

- Sensordaten werden kontinuierlich an Logic-Komponenten weitergegeben
- Logic-Komponenten entscheiden über Geschwindigkeitsbefehle
- Output-Validierung vor Publikation

### 3. Logging

Konsistente Logging-Konventionen:
- `Rec:` - Received (Goal empfangen)
- `Acc:` - Accepted (Goal akzeptiert)
- `Rej:` - Rejected (Goal abgelehnt)
- `Canc:` - Cancelled (Goal abgebrochen)
- `Exec:` - Executing (Goal wird ausgeführt)
- `succ:` - Success (Goal erfolgreich)
- `fail:` - Failed (Goal fehlgeschlagen)

### 4. Bekannte Issues

- **Typo in TurnActionServer:** Parameter heißt `locic` statt `logic`
- **Typo in MuriActionHandler:** Methode heißt `setGoalStautusFinished` statt `setGoalStatusFinished`
- **Inkonsistente Wait-Bedingung:** InitActionServer wartet auf `_goal_result`, andere auf `_goal_exiting`

---

## Abhängigkeiten

### ROS2 Packages

```
rclpy
nav_msgs
geometry_msgs
```

### Custom Packages

```
muri_dev_interfaces
    - action/DRIVE
    - action/TURN
    - action/INIT
    - msg/PictureData

muri_logics
    - logic_action_server_drive (DriveLogic, DriveStates)
    - logic_action_server_turn (TurnLogic, TurnStates)
    - logic_action_server_init (InitLogic, InitStates)
    - main_controller (MainController, MainStates)
    - logic_interface (LogicInterface)
```

---

## Deployment

### Launch-Reihenfolge

1. **Action Server starten:**
   ```bash
   ros2 run <package> init_action_server
   ros2 run <package> drive_action_server
   ros2 run <package> turn_action_server
   ```

2. **Action Handler starten:**
   ```bash
   ros2 run <package> muri_action_handler
   ```

### Voraussetzungen

- Odometry-Publisher muss aktiv sein (`/odom`)
- Picture-Data-Publisher muss aktiv sein (`/muri_picture_data`)
- Roboter-Hardware muss `/cmd_vel` verarbeiten können

---

## Zustandsdiagramm (Beispiel)

```
MainController
    │
    ├─→ IDLE ──→ postInit() ──→ send_init_goal()
    │                               │
    │                               ▼
    │                           INIT läuft
    │                               │
    │                               ▼
    │                           INIT Success
    │                               │
    ├─────────────────────────────┘
    │
    ├─→ PLANNING ──→ calculate_goal ──→ send_drive_goal() / send_turn_goal()
    │                                       │
    │                                       ▼
    │                                   Ausführung
    │                                       │
    │                                       ▼
    │                                   Success/Failed
    │                                       │
    └───────────────────────────────────────┘
```

---

## Kontakt und Support

Für Fragen zur Implementierung oder Logic-Komponenten, siehe:
- `muri_logics` Package-Dokumentation
- `muri_dev_interfaces` Interface-Definitionen

---

## Version

Dokumentiert am: 2025-11-25
ROS2 Version: Humble Hawksbill
Python Version: 3.10.12