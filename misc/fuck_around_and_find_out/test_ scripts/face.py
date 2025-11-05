import cv2 as cv

face_cascade = cv.CascadeClassifier(cv.data.haarcascades + 'haarcascade_frontalface_default.xml')
eye_cascade = cv.CascadeClassifier(cv.data.haarcascades + 'haarcascade_eye.xml')

cap = cv.VideoCapture(0)

print("Press 'q' to quit")

while True:
    ret, frame = cap.read()
    
    if not ret:
        break

    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

    # Erkenne Gesichter
    faces = face_cascade.detectMultiScale(gray, 1.1, 5, minSize=(30, 30))
    
    for (x, y, w, h) in faces:
        # Zeichne grünes Rechteck um Gesicht
        cv.rectangle(frame, (x, y), (x+w, y+h), (255, 255, 0), 2)
        
        # Suche nach Augen nur im Gesichtsbereich (Region of Interest)
        roi_gray = gray[y:y+h, x:x+w]
        roi_color = frame[y:y+h, x:x+w]
        
        eyes = eye_cascade.detectMultiScale(roi_gray, 1.1, 10)
        
        for (ex, ey, ew, eh) in eyes:
            # Zeichne blaue Kreise um die Augen
            center = (x + ex + ew//2, y + ey + eh//2)
            radius = int((ew + eh) / 4)
            cv.circle(frame, center, radius, (255, 0, 0), 2)
    
    # Zeige Statistiken
    cv.putText(frame, f"Gesichter: {len(faces)}", (10, 30), 
                cv.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

    cv.imshow('Gesichts- und Augen-Tracking', frame)

    if cv.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv.destroyAllWindows()