import cv2 as cv
import cv2.aruco as aruco

def create_charuco_board(squares_x=7, squares_y=5, square_length=40, marker_length=30, 
                         output_path='charuco_board.png', dpi=300):
    """
    Erstellt ein ChArUco-Kalibrierungsboard als PNG
    
    Args:
        squares_x: Anzahl Schachbrettfelder in X-Richtung
        squares_y: Anzahl Schachbrettfelder in Y-Richtung
        square_length: Seitenlänge eines Schachbrettfeldes in mm
        marker_length: Seitenlänge eines ArUco-Markers in mm
        output_path: Pfad für die Ausgabedatei (muss .png oder .jpg sein!)
        dpi: Auflösung für den Druck (300 DPI empfohlen)
    """
    # ArUco Dictionary erstellen
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_1000)
    
    # ChArUco Board erstellen
    board = aruco.CharucoBoard(
        (squares_x, squares_y),
        square_length,
        marker_length,
        aruco_dict
    )
    
    # Board-Bild generieren (Größe in Pixeln basierend auf DPI)
    mm_to_inch = 0.0393701
    pixels_per_mm = dpi * mm_to_inch
    
    img_width = int(squares_x * square_length * pixels_per_mm)
    img_height = int(squares_y * square_length * pixels_per_mm)
    
    board_image = board.generateImage((img_width, img_height), marginSize=20)
    
    # Bild speichern
    cv.imwrite(output_path, board_image)
    
    board_width_mm = squares_x * square_length
    board_height_mm = squares_y * square_length
    
    print(f"✓ ChArUco Board erstellt: {output_path}")
    print(f"  Bildgröße: {img_width}x{img_height} Pixel")
    print(f"  Board-Dimensionen: {squares_x}x{squares_y} Felder")
    print(f"  Feldgröße: {square_length}mm, Marker: {marker_length}mm")
    print(f"  Gesamt: {board_width_mm}mm x {board_height_mm}mm ({board_width_mm/10}cm x {board_height_mm/10}cm)")
    print(f"\n📋 DRUCKEN:")
    print(f"  1. Bild in Bildbearbeitungsprogramm öffnen")
    print(f"  2. Größe einstellen: {board_width_mm}mm x {board_height_mm}mm bei {dpi} DPI")
    print(f"  3. Drucken ohne Skalierung")
    print(f"  4. Mit Lineal prüfen: Ein Feld = {square_length}mm x {square_length}mm")
    
    return board

if __name__ == "__main__":
    # Erstelle ein ChArUco Board
    board = create_charuco_board(
        squares_x=7,
        squares_y=5,
        square_length=35,  # 40mm pro Feld
        marker_length=25,  # 30mm Marker
        output_path='charuco_board.png',  # PNG-Datei!
        dpi=300
    )
    print("\n✓ Fertig! Datei 'charuco_board.png' wurde erstellt.")