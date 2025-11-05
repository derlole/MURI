#!/usr/bin/env python3
"""
Live‑Video‑Red‑Marker‑Detection

- Liest Frames von /dev/video0 (Linux) oder dem ersten angeschlossenen Web‑Camera.
- Detectiert rote Quadrate/Vierecke (ähnlich dem vorherigen Skript).
- Zeigt das Video mit Bounding‑Box, Mittelpunkt und einigen Messwerten an.
- Beendet sich, wenn 'q' oder ESC gedrückt wird.
"""

import cv2 as cv
import numpy as np
import sys

def main():
    # --- 1) Video‑Capture initialisieren -----------------------------------
    # Für Linux: "/dev/video0"
    # Für Windows/anderes OS: 0
    device = 0
    # Falls du Linux benutzt und ausdrücklich den Pfad willst, verwende:
    # device = "/dev/video0"
    cap = cv.VideoCapture(device)

    if not cap.isOpened():
        print(f"❌  Kamera konnte nicht geöffnet werden ({device})", file=sys.stderr)
        return

    # Optional: Bildgröße reduzieren, um die Rechenlast zu senken
    # cap.set(cv.CAP_PROP_FRAME_WIDTH, 640)
    # cap.set(cv.CAP_PROP_FRAME_HEIGHT, 480)

    # --- 2) HSV‑Farbbereich für Rot -------------------------------------------------
    lower_red = np.array([0, 50, 50])
    upper_red = np.array([10, 255, 255])
    lower_red_2 = np.array([170, 50, 50])
    upper_red_2 = np.array([180, 255, 255])

    # --- 3) Loop -------------------------------------------------------------------
    while True:
        ret, frame = cap.read()
        if not ret:
            print("❌  Kein Frame mehr verfügbar", file=sys.stderr)
            break

        # Farbraumwechsel BGR → HSV
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

        # Masken für Rot erzeugen (wegen HSV‑Wertspitzen)
        mask1 = cv.inRange(hsv, lower_red, upper_red)
        mask2 = cv.inRange(hsv, lower_red_2, upper_red_2)
        mask_red = cv.bitwise_or(mask1, mask2)

        # Konturen finden
        contours, _ = cv.findContours(mask_red, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)

        # --- 4) Größtes Viereck suchen und Infos ausgeben -------------------------
        if contours:
            largest = max(contours, key=cv.contourArea)
            x, y, w, h = cv.boundingRect(largest)

            # Infos auf der Konsole (optional, kann rausgenommen werden)
            print(f"Rotes Viereck: Pos=({x},{y})  Grz={w}x{h}")

            # Mittelpunkt berechnen
            center_x = x + w // 2
            center_y = y + h // 2

            # Distanz vom Bildmittelpunkt zur roten Fläche (horizontal)
            picture_height, picture_width = frame.shape[:2]
            pixel_to_mid = (picture_width // 2) - center_x  # positive = Marker links vom Bildmittelpunkt

            # --- 5) Visualisierung ------------------------------------------------
            # Box zeichnen
            cv.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            # Mittelpunkt als Kreis
            cv.circle(frame, (center_x, center_y), 5, (255, 0, 0), -1)

            # Textinfos
            cv.putText(frame, f"Center: ({center_x},{center_y})", (10, 20),
                       cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            cv.putText(frame, f"pixel_to_mid: {pixel_to_mid}", (10, 50),
                       cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            cv.putText(frame, f"Height: {h}", (10, 80),
                       cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        # --- 6) Ausgabe anzeigen --------------------------------------------------
        cv.imshow("Red Marker Live", frame)

        # --- 7) Abbruchbedingung --------------------------------------------------
        key = cv.waitKey(1)
        if key == 27 or key == ord('q'):  # ESC oder q
            print("🛑  Beenden …")
            break

    # --- 8) Aufräumen ----------------------------------------------------------
    cap.release()
    cv.destroyAllWindows()

if __name__ == "__main__":
    main()