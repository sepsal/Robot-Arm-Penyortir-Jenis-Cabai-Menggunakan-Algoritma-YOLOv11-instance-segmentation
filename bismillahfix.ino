#include <WiFi.h>
#include <PubSubClient.h>
#include <ESP32Servo.h>

// ==================== KONFIGURASI HARDWARE ====================
// Pin Definitions
#define PUL_PIN 19 // Pulse pin
#define DIR_PIN 18 // Direction pin

#define SERVO_RIGHT_PIN 26
#define SERVO_LEFT_PIN 14
#define SERVO_ELBOW1_PIN 32
#define SERVO_ELBOW2_PIN 13
#define SERVO_GRIPPER_PIN 12

// Constants
const int STEPS_PER_REV = 8000;               // 8000 pulses per revolution
const float DEGREE_PER_STEP = 360.0 / STEPS_PER_REV; // Degree per step (0.045 degrees)
const float GEAR_RATIO = 3.47222222; // Stepper to base rotation ratio
const float GRIPPER_OPEN = 180; // Disesuaikan lagi
const float GRIPPER_CLOSE = 0; // Disesuaikan lagi

// Robot Base Position
const float ROBOT_X = 0.0;
const float ROBOT_Y = 0.0;
const float ROBOT_Z = 15.5;

// Drop Points for Different Chili Type (in cm)
const float DROP_POINTS[4][3] = {
  {-10.0, 19.0, 10.0},  // Cabai besar hijau
  {-13.0, 14.0, 10.0},  // Cabai besar merah
  {-16.0, 9.0, 10.0},   // Cabai keriting hijau
  {-19.0, 4.0, 10.0}    // Cabai keriting merah
};

// Arm Lengths
float l1 = 19.0;
float l2 = 12.0;
float l3 = 20.0;

// Servo Objects
Servo servoRight;
Servo servoLeft;
Servo servoElbow1;
Servo servoElbow2;
Servo servoGripper;

// Variables
float targetX = 0, targetY = 0, targetZ = 1.0;
float targetAngle_1, targetAngle_2, targetAngle_3;
float thetaAngle_3, theta_3a;
float theta_1, theta_2, theta_3, theta_4;
int direction;
long stepsToMove;
long currentStep = 0;
float lastTheta = 0; // Menyimpan sudut terakhir
int jenis_cabai = -1;  // -1 = belum terdeteksi, 0-3 sesuai jenis cabai

// Variabel untuk servo smoothing
float currentServoRightPos = 90; // Posisi awal servo kanan
float currentServoLeftPos = 90;  // Posisi awal servo kiri
float currentServoElbow1Pos = 90; // Posisi awal servo elbow 1
float currentServoElbow2Pos = 0; // Posisi awal servo elbow 2
float currentServoGripperPos = GRIPPER_OPEN; // Posisi awal servo gripper

// ========== KONFIGURASI MQTT ===========
const char* ssid = "Sepsal";
const char* password = "salsa.279";
const char* mqtt_server = "192.168.20.13";
const int mqtt_port = 1883;

// MQTT Topics
const char* TOPIC_CM_X = "chili/cm_x";
const char* TOPIC_CM_Y = "chili/cm_y";
const char* TOPIC_JENIS = "chili/jenis";

// Variabel MQTT
WiFiClient espClient;
PubSubClient client(espClient);

// Flags untuk menandai data yang diterima
bool received_cm_x = false;
bool received_cm_y = false;
bool received_jenis = false;

bool isOperating = false;

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
  // Jika sedang melakukan operasi, abaikan pesan baru
  if (isOperating) {
    Serial.println("Masih dalam proses operasi, abaikan pesan baru");
    return;
  }

  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  
  String message;
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  Serial.println(message);

  if (strcmp(topic, TOPIC_CM_X) == 0) {
    targetX = message.toFloat();
    received_cm_x = true;
    Serial.print("Received cm_x: ");
    Serial.println(targetX);
  } 
  else if (strcmp(topic, TOPIC_CM_Y) == 0) {
    targetY = message.toFloat();
    received_cm_y = true;
    Serial.print("Received cm_y: ");
    Serial.println(targetY);
  }
  else if (strcmp(topic, TOPIC_JENIS) == 0) {
    jenis_cabai = message.toInt();
    received_jenis = true;
    Serial.print("Received jenis: ");
    Serial.println(jenis_cabai);
  }

  // Jika semua data sudah diterima, lakukan perhitungan IK
  if (received_cm_x && received_cm_y && received_jenis && jenis_cabai >= 0 && jenis_cabai <= 3) {
    isOperating = true; // Set flag operasi sedang berjalan
    executePickAndPlace();
  }
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection to...");
    Serial.print(mqtt_server);
    Serial.print(":");
    Serial.print(mqtt_port);
    Serial.println("...");

    if (client.connect("ESP32Client")) {
      Serial.println("connected");
      client.subscribe(TOPIC_CM_X);
      client.subscribe(TOPIC_CM_Y);
      client.subscribe(TOPIC_JENIS);

      //client.publish(TOPIC_READY, "1");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.print(" (");
      Serial.print(getMQTTStateDescription(client.state()));
      Serial.println(") try again in 5 seconds");
      delay(5000);
    }
  }
}

String getMQTTStateDescription(int state) {
  switch(state) {
    case -4: return "Connection timeout";
    case -3: return "Connection lost";
    case -2: return "Connect failed";
    case -1: return "Disconnected";
    case 0: return "Connected";
    case 1: return "Bad protocol";
    case 2: return "Bad client ID";
    case 3: return "Unavailable";
    case 4: return "Bad credentials";
    case 5: return "Unauthorized";
    default: return "Unknown";
  }
}

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);

  pinMode(PUL_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  
  servoRight.attach(SERVO_RIGHT_PIN);
  servoLeft.attach(SERVO_LEFT_PIN);
  servoElbow1.attach(SERVO_ELBOW1_PIN);
  servoElbow2.attach(SERVO_ELBOW2_PIN);
  servoGripper.attach(SERVO_GRIPPER_PIN);

  // Set initial servo positions
  servoRight.write(currentServoRightPos);
  servoLeft.write(currentServoLeftPos);
  servoElbow1.write(currentServoElbow1Pos);
  servoElbow2.write(currentServoElbow2Pos);
  servoGripper.write(currentServoGripperPos);

  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  Serial.println("Sistem Robot Arm 4 DOF siap");
  Serial.println("Waiting for MQTT messages...");
}

// ====================== LOOP UTAMA ======================
void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
}

void printDebugInfo();

// ====================== FUNGSI KONTROL ROBOT ======================
void executePickAndPlace() {
  Serial.println("Memulai proses Pick and Place");
  printDebugInfo();
  
  // 1. PICK OPERATION
  Serial.println("=== FASE PICK ===");

  // Set targetZ konstan untuk pick
  targetZ = 1.0; // 1 cm di atas permukaan
  
  // Lakukan perhitungan IK untuk PICK
  calculateInverseKinematics(targetX, targetY, targetZ);
  printDebugInfo();
  
  // Kembali ke posisi awal servo sebelum stepper
  returnServoToOrigin();

  // Kembali ke posisi awal stepper sebelum mengeksekusi koordinat baru
  returnStepperToOrigin();

  // Gerakkan stepper ke posisi target
  if (stepsToMove > 0) moveStepper(stepsToMove, direction);
  lastTheta = theta_1;
  moveServosToTarget(true); // true = mode PICK
  printDebugInfo();

  // Lanjutkan dengan penutupan gripper setelah koreksi
  controlGripper(false);
  delay(500);

  // 2. PREPARE FOR PLACE
  Serial.println("=== PREPARE PLACE ===");

  // Kembali ke posisi awal sebelum IK 2
  returnServoToOrigin();
  returnStepperToOrigin();

   // 3. PLACE OPERATION
  Serial.println("=== FASE PLACE ===");

  // Lakukan perhitungan IK untuk PLACE berdasarkan jenis cabai
  if (jenis_cabai >= 0 && jenis_cabai <= 3) {
    placeObjectToDropPoint(jenis_cabai);
    printDebugInfo();
    
    controlGripper(true);
    delay(500);
  }

  // 4. FINISH OPERATION
  Serial.println("=== FINISHING ===");

  controlGripper(true);      // Buka gripper (safety)
  delay(500);               // Beri waktu stabilisasi

  // Kirim sinyal ready setelah menyelesaikan operasi
  //sendReadySignal();

  // Reset flags dan status operasi
  received_cm_x = false;
  received_cm_y = false;
  received_jenis = false;
  isOperating = false;

  Serial.println("Proses Pick and Place selesai");
}

// Fungsi untuk mengirim sinyal ready ke Python
//void sendReadySignal() {
//  if (client.connected()) {
//    client.publish(TOPIC_READY, "1");
//    Serial.println("Mengirim sinyal READY ke Python");
//  } else {
//    Serial.println("Gagal mengirim sinyal READY, MQTT tidak terhubung");
//  }
//}

void calculateInverseKinematics(float targetX, float targetY, float targetZ) {
  // Calculate target angle θ1
  targetAngle_1 = atan2(targetY - ROBOT_Y, targetX - ROBOT_X) * 180.0 / PI;
  
  // Hitung theta_1 sebagai sudut dari sumbu Y positif (0° menghadap depan)
  theta_1 = 90.0 - targetAngle_1; // Konversi ke sudut robot

  // Normalisasi sudut antara -180° dan 180°
  theta_1 = fmod(theta_1 + 180.0, 360.0) - 180.0;

  // Determine direction (CCW or CW)
  direction = (targetX > 0) ? 1 : -1;

  // Convert angle to steps considering gear ratio
  stepsToMove = fabs(theta_1) * GEAR_RATIO / DEGREE_PER_STEP;

  // Hitung parameter untuk lengan
  float a = sqrt(pow(targetX - ROBOT_X, 2) + pow(targetY - ROBOT_Y, 2));
  float z_rel = targetZ - ROBOT_Z;
  float b = l3 + z_rel;
  float c = sqrt(pow(a, 2) + pow(b, 2));
  
  if (c > (l1 + l2) || c < fabs(l1 - l2)) {
    Serial.println("Target di luar jangkauan lengan robot.");
  } else {
    // Calculate theta_2
    float alpha_1 = acos((pow(c, 2) + pow(l1, 2) - pow(l2, 2)) / (2 * c * l1));
    float alpha_2 = atan2(b, a);
    targetAngle_2 = (alpha_1 + alpha_2) * 180.0 / PI;
    theta_2 = targetAngle_2;
    
    // Calculate theta_3
    targetAngle_3 = acos((pow(l1, 2) + pow(l2, 2) - pow(c, 2)) / (2 * l1 * l2));
    thetaAngle_3 = targetAngle_3 * 180.0 / PI;
    theta_3a = thetaAngle_3 - 180;
    theta_3 = fabs(theta_3a);

    // Calculate theta_4
    theta_4 = (90 - theta_2 - theta_3) - 180;

    Serial.print("Sudut terhitung: ");
    Serial.print("θ1="); Serial.print(theta_1);
    Serial.print("°, θ2="); Serial.print(theta_2);
    Serial.print("°, θ3="); Serial.print(theta_3);
    Serial.print("°, θ4="); Serial.print(theta_4);
   Serial.println("°");
  }
}

void returnServoToOrigin() {
  // Gerakkan servo shoulder ke posisi awal
  smoothMoveServo(servoRight, currentServoRightPos, 90, 20); // Kembali ke 90 dengan delay 20ms
  smoothMoveServo(servoLeft, currentServoLeftPos, 90, 20);  // Kembali ke 90 dengan delay 20ms

  // Gerakkan servo elbow 1 ke posisi awal
  smoothMoveServo(servoElbow1, currentServoElbow1Pos, 90, 20); // Kembali ke 150 derajat delay 20ms

  // Gerakkan servo elbow 2 ke posisi awal
  smoothMoveServo(servoElbow2, currentServoElbow2Pos, 0, 20); // Kembali ke 150 derajat delay 20ms
}

void returnStepperToOrigin() {
  if (lastTheta != 0) {
    // Hitung langkah untuk kembali ke posisi awal
    long returnSteps = abs(lastTheta * GEAR_RATIO / DEGREE_PER_STEP);
    int returnDirection = (lastTheta > 0) ? -1 : 1; // Arah kembali ke 0 derajat

    Serial.print("Langkah kembali: ");
    Serial.println(returnSteps);

    // Gerakkan stepper motor kembali ke posisi awal
    moveStepper(returnSteps, returnDirection);

    // Reset lastTheta ke 0 setelah kembali ke posisi awal
    lastTheta = 0;
  }
}

// Move Stepper Motor
void moveStepper(long steps, int dir) {
  // Set direction
  if (dir > 0) {
    digitalWrite(DIR_PIN, LOW);
    Serial.println("DIR_PIN: LOW (CCW)");
  } else {
    digitalWrite(DIR_PIN, HIGH);
    Serial.println("DIR_PIN: HIGH (CW)");
  }

  delay(10); // Delay kecil untuk memastikan pin direction sudah set

  // Move stepper
  for (long i = 0; i < steps; i++) {
    digitalWrite(PUL_PIN, HIGH);
    delayMicroseconds(500);
    digitalWrite(PUL_PIN, LOW);
    delayMicroseconds(500);

    currentStep += dir; // Update current step position
  }
}

void controlGripper(bool open) {
  float targetPos = open ? GRIPPER_OPEN : GRIPPER_CLOSE;
  smoothMoveServo(servoGripper, currentServoGripperPos, targetPos, 20);
  Serial.println(open ? "Gripper terbuka" : "Gripper tertutup");
}

void moveServosToTarget(bool isPickOperation) {
  if (isPickOperation){
    // Mode PICK (Ambil Cabai):
    // 1. Buka gripper -> 2. Bergerak ke target -> 3. Tutup gripper
    controlGripper(true);  // Buka gripper
    delay(300);            // Beri waktu untuk membuka

    //Gerakkan servo shoulder
    smoothMoveServo(servoRight, currentServoRightPos, theta_2, 20); // Delay 20ms per langkah
    smoothMoveServo(servoLeft, currentServoLeftPos, 180 - theta_2, 20); // Delay 20ms per langkah

    // Gerakkan servo elbow 1 ke posisi target
    smoothMoveServo(servoElbow1, currentServoElbow1Pos, theta_3, 20); // Delay 20ms per langkah

    // Gerakkan servo elbow 2 ke posisi target
    smoothMoveServo(servoElbow2, currentServoElbow2Pos, theta_4, 20); // Delay 20ms per langkah

    // Tutup gripper untuk mencengkeram objek
    controlGripper(false);
    delay(500);
  } else {
    // Mode PLACE (Letakkan Cabai):
    // 1. Gripper sudah tertutup (pegang cabai) -> 2. Bergerak ke wadah -> 3. Buka gripper
    
    //Gerakkan servo shoulder
    smoothMoveServo(servoRight, currentServoRightPos, theta_2, 20); // Delay 20ms per langkah
    smoothMoveServo(servoLeft, currentServoLeftPos, 180 - theta_2, 20); // Delay 20ms per langkah

    // Gerakkan servo elbow 1 ke posisi target
    smoothMoveServo(servoElbow1, currentServoElbow1Pos, theta_3, 20); // Delay 20ms per langkah

    // Gerakkan servo elbow 2 ke posisi target
    smoothMoveServo(servoElbow2, currentServoElbow2Pos, theta_4, 20); // Delay 20ms per langkah

    controlGripper(true); // Buka untuk melepas cabai
    delay(500);
  }
}

// Fungsi untuk memindahkan objek ke wadah berdasarkan jenis cabai
void placeObjectToDropPoint(int jenis_cabai) {
  // Set target ke koordinat wadah sesuai jenis cabai
  targetX = DROP_POINTS[jenis_cabai][0];
  targetY = DROP_POINTS[jenis_cabai][1];
  targetZ = DROP_POINTS[jenis_cabai][2];

  // Hitung inverse kinematics untuk wadah
  calculateInverseKinematics(targetX, targetY, targetZ);

  if (stepsToMove > 0) moveStepper(stepsToMove, direction);
  lastTheta = theta_1;
  moveServosToTarget(false); // Parameter false = mode PLACE
}

void smoothMoveServo(Servo &servo, float &currentPos, float targetPos, int stepDelay) {  
  float stepSize = 1.0; // Besaran perubahan per langkah (bisa disesuaikan)
  if (currentPos < targetPos) {
    while (currentPos < targetPos) {
      currentPos += stepSize;
      if (currentPos > targetPos) currentPos = targetPos;
      servo.write(currentPos);
      delay(stepDelay);
    }
  } else if (currentPos > targetPos) {
    while (currentPos > targetPos) {
      currentPos -= stepSize;
      if (currentPos < targetPos) currentPos = targetPos;
      servo.write(currentPos);
      delay(stepDelay);
    }
  }
}

void printDebugInfo() {
  Serial.println("=== DEBUG INFO ===");
  Serial.print("Target Position - X: "); Serial.print(targetX);
  Serial.print(" cm, Y: "); Serial.print(targetY);
  Serial.print(" cm, Z: "); Serial.print(targetZ); Serial.println(" cm");
  
  Serial.print("Current Servo Positions - ");
  Serial.print("Shoulder 1: "); Serial.print(currentServoRightPos);
  Serial.print("Shoulder 2: "); Serial.print(currentServoLeftPos);
  Serial.print("°, Elbow1: "); Serial.print(currentServoElbow1Pos);
  Serial.print("°, Elbow2: "); Serial.print(currentServoElbow2Pos);
  
  Serial.print("Stepper Position - Steps: "); Serial.print(currentStep);
  Serial.print(", Last Theta: "); Serial.print(lastTheta); Serial.println("°");
}