import cv2
import numpy as np
from ultralytics import YOLO
import cvzone
import paho.mqtt.client as mqtt
import time

# ===== Konfigurasi MQTT =====
MQTT_BROKER = "192.168.142.13"  # Ganti dengan broker Anda
MQTT_PORT = 1883
TOPIC_CM_X = "chili/cm_x"
TOPIC_CM_Y = "chili/cm_y"
TOPIC_JENIS = "chili/jenis"
USERNAME = "Sepsal"
PASSWORD = "salsa.279"

# Scale factor: 0.5 cm per pixel
SCALE_FACTOR = 0.106
CONFIDENCE_THRESHOLD = 0.7  # Filter deteksi minimal 80%

# Priority order for chili processing
PRIORITY_ORDER = {
    "cabai besar hijau": 1,
    "cabai besar merah": 2,
    "cabai keriting hijau": 3,
    "cabai keriting merah": 4
}

# Variabel untuk tracking akurasi pengiriman
mqtt_stats = {
    'success_count': 0,
    'total_attempts': 0,
    'last_publish_time': 0
}

# Fungsi untuk mengubah nama kelas ke integer
def class_to_int(class_name):
    class_name_lower = class_name.lower()
    if "cabai besar hijau" in class_name_lower:
        return 0
    elif "cabai besar merah" in class_name_lower:
        return 1
    elif "cabai keriting hijau" in class_name_lower:
        return 2
    elif "cabai keriting merah" in class_name_lower:
        return 3
    else:
        return -1  # Unknown


# Callback ketika berhasil terhubung ke broker MQTT
def on_connect(client, userdata, flags, rc, properties=None):
    if rc == 0:
        print("✅ Connected to MQTT Broker!")
    else:
        print(f"❌ Connection failed with code {rc}")


# Callback ketika pesan berhasil dikirim
def on_publish(client, userdata, mid, *args):
    global mqtt_stats
    mqtt_stats['success_count'] += 1
    mqtt_stats['last_publish_time'] = time.time()

# Callback kosong karena tidak ada topik koreksi yang diproses
def on_message(client, userdata, msg):
    pass

# Fungsi untuk menghitung dan menampilkan persentase keberhasilan
def print_success_rate():
    if mqtt_stats['total_attempts'] > 0:
        success_rate = (mqtt_stats['success_count'] / mqtt_stats['total_attempts']) * 100
        latency = time.time() - mqtt_stats['last_publish_time'] if mqtt_stats['last_publish_time'] > 0 else 0
        print(f"\n[STATISTIK MQTT] Keberhasilan: {success_rate:.2f}% ({mqtt_stats['success_count']}/{mqtt_stats['total_attempts']}) | Latensi: {latency:.2f}s\n")

# Inisialisasi MQTT Client
mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1)
mqtt_client.username_pw_set(USERNAME, PASSWORD)
mqtt_client.on_connect = on_connect
mqtt_client.on_message = on_message
mqtt_client.on_publish = on_publish  # Menambahkan callback untuk publish

# Menghubungkan ke broker
mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
mqtt_client.loop_start()

# Fungsi untuk menentukan warna bounding box berdasarkan jenis cabai
def get_color(class_name):
    if "cabai besar hijau" in class_name.lower():
        return (0, 150, 0)  # Hijau tua
    elif "cabai besar merah" in class_name.lower():
        return (0, 0, 200)  # Merah tua
    elif "cabai keriting hijau" in class_name.lower():
        return (0, 255, 0)  # Hijau terang
    elif "cabai keriting merah" in class_name.lower():
        return (0, 0, 255)  # Merah terang

def get_bbox_color(class_name):
    if "cabai besar hijau" in class_name.lower():
        return (0, 150, 0)  # Hijau tua
    elif "cabai besar merah" in class_name.lower():
        return (0, 0, 200)  # Merah tua
    elif "cabai keriting hijau" in class_name.lower():
        return (0, 255, 0)  # Hijau terang
    elif "cabai keriting merah" in class_name.lower():
        return (0, 0, 255)  # Merah terang

# Fungsi RGB Mouse Callback (opsional—bisa dihapus jika tidak diperlukan)
def RGB(event, x, y, flags, param):
    if event == cv2.EVENT_MOUSEMOVE:
        point = [x, y]
        print(point)

# Set up jendela OpenCV dan callback mouse (opsional)
cv2.namedWindow('RGB')
cv2.setMouseCallback('RGB', RGB)

# Muat model YOLOv11 yang sudah dilatih
model = YOLO('C:/Users/HP_ID/PycharmProjects/pythonProject2/runs/segment/my_yolov11_segmentation_model5/weights/best.pt')

# Buka stream dari DroidCam sebagai webcam
cap = cv2.VideoCapture(1)  # Sesuaikan index dengan perangkat DroidCam di sistem Anda

# Periksa apakah kamera terbuka dengan benar
if not cap.isOpened():
    print("[ERROR] Could not access DroidCam webcam! Pastikan DroidCam berjalan dan terdeteksi di sistem.")
    exit()

print("[INFO] DroidCam connected successfully.")

count = 0
last_stat_print_time = 0

while True:
    # Ambil frame dari video stream
    ret, frame = cap.read()
    if not ret:
        print("[ERROR] Failed to grab frame.")
        break

    count += 1
    if count % 2 != 0:  # Menyaring setiap 2 frame untuk mengurangi beban pemrosesan
        continue

    # Jalankan YOLOv11 untuk deteksi dan segmentasi
    results = model.track(frame, persist=True)

    # Dictionary to store detected chilis by their priority
    detected_chilis = {}

    # Pastikan ada kotak (bounding boxes) dalam hasil deteksi
    if results[0].boxes is not None:
        boxes = results[0].boxes.xyxy.int().cpu().tolist()
        class_ids = results[0].boxes.cls.int().cpu().tolist()
        confidences = results[0].boxes.conf.cpu().tolist()  # Ambil nilai confidence

        # Cek jika tracking IDs ada
        if results[0].boxes.id is not None:
            track_ids = results[0].boxes.id.int().cpu().tolist()
        else:
            track_ids = [-1] * len(boxes)  # Jika tidak ada ID pelacakan, gunakan -1

        masks = results[0].masks
        if masks is not None:
            masks = masks.xy
            overlay = frame.copy()

            for box, track_id, class_id, confidence, mask in zip(boxes, track_ids, class_ids, confidences, masks):
                if confidence < CONFIDENCE_THRESHOLD:
                    continue  # Lewati deteksi dengan confidence rendah
                # Ambil nama kelas cabai berdasarkan ID yang diberikan oleh model
                class_name = results[0].names[class_id]  # Nama kelas otomatis dari model YOLO

                x1, y1, x2, y2 = box

                # Pastikan mask tidak kosong
                if mask.size > 0:
                    mask = np.array(mask, dtype=np.int32).reshape((-1, 1, 2))  # Ubah mask menjadi format yang benar

                    # Dapatkan warna berdasarkan jenis cabai
                    seg_color = get_color(class_name)
                    bbox_color = get_bbox_color(class_name)

                    # Gambar bounding box dengan warna khusus
                    cv2.rectangle(frame, (x1, y1), (x2, y2), bbox_color, 2)

                    # Gambar segmentasi dengan warna khusus
                    cv2.fillPoly(overlay, [mask], color=seg_color)

                    # Label dengan warna yang kontras
                    label = f"{class_name} ({confidence:.2f})"
                    cvzone.putTextRect(frame, label, (x1, y1 - 10), 1, 1,
                                       colorR=bbox_color, colorB=(0, 0, 0))

                    # Konversi masker ke format biner untuk OpenCV
                    binary_mask = np.zeros(frame.shape[:2], dtype=np.uint8)
                    cv2.fillPoly(binary_mask, [mask], 255)

                    # Hitung pusat massa (centroid) dari masker
                    M = cv2.moments(binary_mask)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])  # Koordinat X pusat massa
                        cy = int(M["m01"] / M["m00"])  # Koordinat Y pusat massa

                        # Store the detected chili with its priority
                        priority = PRIORITY_ORDER.get(class_name.lower(), 999)
                        detected_chilis[priority] = {
                            "class_name": class_name,
                            "coordinates": (cx, cy),
                            "box": box,
                            "mask": mask,
                            "confidence": confidence
                        }

                        # Gambar titik pusat massa pada frame
                        cv2.circle(frame, (cx, cy), 5, (255, 255, 0), -1)
                        cv2.putText(frame, f"({cx}, {cy})", (cx + 10, cy),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                        print(f"[INFO] Kelas: {class_name}, Titik Koordinat: ({cx}, {cy})")

                # Transparansi untuk overlay masker
                alpha = 0.7
                frame = cv2.addWeighted(overlay, alpha, frame, 1 - alpha, 0)

            # Process chilis in priority order if any detected
            if detected_chilis:
                # Get the highest priority chili (lowest priority number)
                highest_priority = min(detected_chilis.keys())
                chili = detected_chilis[highest_priority]

                # Convert coordinates to cm
                cx_cm = chili["coordinates"][0] * SCALE_FACTOR
                cy_cm = chili["coordinates"][1] * SCALE_FACTOR

                # Convert class to integer
                class_int = class_to_int(chili["class_name"])

                # Publish data via MQTT
                if mqtt_client.is_connected():
                    try:
                        # Kirim data
                        mqtt_client.publish(TOPIC_CM_X, str(cx_cm))
                        mqtt_client.publish(TOPIC_CM_Y, str(cy_cm))
                        mqtt_client.publish(TOPIC_JENIS, str(class_int))
                        mqtt_stats['total_attempts'] += 3

                        print(f"[MQTT] Published - X: {cx_cm} cm, Y: {cy_cm} cm, Jenis: {class_int}")
                    except Exception as e:
                        print(f"[MQTT ERROR] Failed to publish: {e}")
                        mqtt_stats['total_attempts'] += 3
                else:
                    print("MQTT not connected!")
                    mqtt_stats['total_attempts'] += 3  # Dianggap sebagai percobaan gagal

    # Tampilkan frame dengan hasil deteksi dan segmentasi
    cv2.imshow("RGB", frame)

    # Tampilkan statistik setiap 10 detik
    current_time = cv2.getTickCount() / cv2.getTickFrequency()
    if current_time - last_stat_print_time >= 10:  # Setiap 10 detik
        print_success_rate()
        last_stat_print_time = current_time

    # Keluar dari loop jika tombol "q" ditekan
    if cv2.waitKey(1) & 0xFF == ord("q"):
        print_success_rate()  # Tampilkan statistik terakhir sebelum keluar
        break

# Cleanup
mqtt_client.loop_stop()
try:
    mqtt_client.disconnect()
    print("[MQTT] Disconnected")
except Exception as e:
    print(f"[MQTT ERROR] Disconnection failed: {e}")

cap.release()
cv2.destroyAllWindows()
