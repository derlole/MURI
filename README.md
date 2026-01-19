# MURI - Mechanische Untergrund-Ratte für Inspektion

**Projektversion**: 2.0.0  
**Datum**: Januar 2026  
**Status**: In Entwicklung

## Überblick

MURI ist ein autonomer mobiler Roboter, der für Inspektionsaufgaben in Untergrundumgebungen entwickelt wurde. Das System basiert auf ROS2 und implementiert eine hierarchische State-Machine-Architektur für die Steuerung von Fahrmanövern, Objektverfolgung und Navigation.

### Hauptfunktionen
- **Autonome Navigation**: Fahren entlang von Rohren und Verfolgung von Zielen
- **ArUco-Marker-Erkennung**: Robuste Marker-Detektion für Zielverfolgung
- **ROS2-Integration**: Modulare Action-Server-Architektur
- **State-Machine-Steuerung**: Hierarchische Logik für verschiedene Betriebsmodi

## Schnellstart

### Voraussetzungen
- ROS2 Humble Hawksbill
- Python 3.9+
- OpenCV 4.5+
- TurtleBot3 oder kompatibler Roboter

### Installation
Für detaillierte Installationsanweisungen siehe: **[Installation Guide](docs/installation.md)**

### Erste Schritte
1. Repository klonen
2. ROS2-Workspace einrichten
3. Abhängigkeiten installieren
4. Roboter starten und testen

## Dokumentation

Die vollständige Dokumentation ist in der `docs/`-Ordner organisiert:

### Projektübersicht und Planung
**[Projektplan und Architektur](docs/documentation.md)**  
Allgemeine Projektbeschreibung, Meilensteine, Stakeholder und technische Herleitungen.

### Installation und Setup
**[Installationsanleitung](docs/installation.md)**  
Schritt-für-Schritt-Anleitung zur Installation und Konfiguration des Systems.

### Steuerungslogik
**[Roboter-Steuerungslogik](docs/logic_2.md)**  
Detaillierte Beschreibung der State-Machine-Module (Init, Drive, Turn, Follow, MainController).

### ROS2-Architektur
**[ROS2 Action Server Architektur](docs/ros.md)**  
Dokumentation der ROS2-Komponenten, Action-Server und Kommunikationsschnittstellen.

### Vision-System
**[ArUco Marker Detection](docs/vision.md)**  
Beschreibung des Bildverarbeitungssystems für Marker-Erkennung und Positionsberechnung.

## Verwendung

### Grundlegende Befehle
```bash
# Workspace sourcen
source install/setup.bash

# Action-Server starten
ros2 run muri_dev muri_drive_action_server
ros2 run muri_dev muri_action_handler

# Keyboard-Steuerung
ros2 run muri_dev muri_controll
```

### Testen
- Unit-Tests: `pytest` in `muri_logics/`
- Integrationstests: ROS2-Launch-Dateien
- Hardware-Tests: Auf TurtleBot3

## Beitragen

### Entwicklungsumgebung
- **OS**: Linux (Ubuntu 22.04)
- **ROS2**: Humble Hawksbill
- **Python**: 3.10.12
- **IDE**: VS Code mit ROS2-Extensions

### Code-Style
- PEP 8 für Python
- ROS2-Standards für C++/Python-Nodes
- Dokumentation in Markdown

## Support und Kontakt

Bei Fragen oder Problemen:
- **Dokumentation konsultieren**: Siehe `docs/`-Ordner
- **Issues**: GitHub-Issues für Bug-Reports
- **Team**: Entwicklungsteam (siehe [Projektplan](docs/documentation.md))

## Autoren und Verantwortlichkeiten (inlk. der Dokumentation und Unit-tests)

- **Vision-System (Code und Dokumentation)**: Linus Braun
- **Logikbausteine**: Louis Moser
- **ROS2-Integration**: Benjamin Keppler
- **Gemeinsame Dokumentation und Main Controller**: Gemeinsam geschrieben

### Für eine detailliertere Aufschlüsselung der geschriebenen Codezeilen pro Person können gerne die Git-Commits eingesehen werden. Dabei ist jedoch zu beachten, dass bei gemeinsamen Programmiersitzungen oft an einem Rechner gemeinsam gearbeitet wurde.

## Lizenz

Dieses Projekt ist Teil einer universitären Arbeit und nicht für kommerzielle Nutzung freigegeben.

---

**Letzte Aktualisierung**: 15 Januar 2026  
**ROS2-Version**: Humble Hawksbill  
**Python-Version**: 3.10.12

