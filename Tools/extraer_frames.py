import cv2
import os

def extraer_frames(video_path, output_folder, salto=3):
    # Crear carpeta de salida si no existe
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    # Cargar video
    cap = cv2.VideoCapture(video_path)

    if not cap.isOpened():
        print("Error: No se pudo abrir el video.")
        return

    frame_id = 0
    saved_id = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            break  # Fin del video

        # Guardamos cada 10 frames
        if frame_id % salto == 0:
            filename = os.path.join(output_folder, f"H5_{saved_id:05d}.jpg")
            cv2.imwrite(filename, frame)
            saved_id += 1

        frame_id += 1

    cap.release()
    print(f"Proceso terminado. Se guardaron {saved_id} imágenes en {output_folder}")


# -------------------------------
# EJEMPLO DE USO
# -------------------------------
if __name__ == "__main__":
    video = "H5.mp4"                 # Ruta del video de entrada
    carpeta_salida = "screenshots_H5"         # Carpeta donde se guardarán las imágenes
    extraer_frames(video, carpeta_salida, salto=10)

