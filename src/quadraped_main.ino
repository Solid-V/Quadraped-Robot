#include <WiFi.h>
#include <WiFiUdp.h>
#include <SCServo.h>
#include <math.h>
#include <Adafruit_ISM330DHCX.h>
SMS_STS st;

const char* ssid = "YOUR SSID";
const char* password = "YOUR PASSWORD";

WiFiUDP udp;
unsigned int localUdpPort = 4210;  // Port to listen on
char packetBuffer[255];            // Buffer for incoming data

Adafruit_ISM330DHCX ism330dhcx;

// Global movement states
struct RobotStates {
    bool w = false;
    bool s = false;
    bool a = false;
    bool d = false;
    bool seq = false;
} states;

#define S_RXD 18    
#define S_TXD 15

//Initial configurations for robot
float L1 = 156.0;
float L2 = 164.0;

float x = -1.0;
float y = -242.0;
float z = 0;
bool isActive = true;

float global_phase = 0.0;
float phase_speed = 0.02; 
float pitch_val = 0.0;    

float step_length = 20.0;
float step_height = 20.0;

float tx_offset = 40.0;

float current_stable_pitch = 0;
float pitch_smoothing = 0.85; 
float robot_body_length = 200.0; // Distance between front and back hip (mm)

//automation constants
bool sequenceRunning = false; // The latch that stays on
bool lastSeqButtonState = false; // To detect the moment the button is pressed
float stepCounter = 0;

const float MAX_PITCH_ANGLE = 15.0; // Degrees
const float MIN_PITCH_ANGLE = -15.0; // Degrees
const float PITCH_DEADZONE = 1.2;    // Ignore tilts less than 1.2 degrees

float theta1, theta2, theta3;

byte ID[12];
s16 Position[12];
u16 Speed[12];
byte ACC[12];

//Inverse Kinematics calculations
bool IK(float x, float y, float z) {
  float Rz = 47;
  float Ry = 20;
  float R1 = sqrt(z*z + y*y);
  
  // Safety: Prevent NaN if Ry > R1
  float cos_val = Ry / R1;
  if (cos_val > 1.0) cos_val = 1.0;
  if (cos_val < -1.0) cos_val = -1.0;

  float R23_val = sqrt(max(0.0f, R1*R1 - Ry*Ry)) - Rz;
  theta3 = PI / 2 - atan2(y, z) - acos(cos_val);

  // maintain direction
  float signY = (y >= 0) ? 1.0 : -1.0;
  float ik_y = signY * R23_val; 
  
  float L = sqrt(x*x + ik_y*ik_y);

  if (L > (L1 + L2) || L < abs(L1 - L2)) {
    Serial.println("Limit reached!");
    return false;
  }

  float a1_internal = acos((L1*L1 + L2*L2 - L*L) / (2 * L1 * L2));
  float a1 = PI - a1_internal;// original

  //float a1 = (x*x + y*y - L1*L1 - L2*L2) / (2 * L1 * L2);

  //if (a1 > 1.0) a1 = 1.0;
  //if (a1 < -1.0) a1 = -1.0;

  //a1 = acos(a1);
  
  // use ik_y here so atan2 knows the target is below the hip
  float a2 = atan2(ik_y, x) - acos((L1*L1 + L*L - L2*L2) / (2 * L1 * L)); // original
  //float a2 = atan2(x, y) - atan2(L2 * sin(a1), L1 + L2 * cos(a1));

  // Joint 2 linkage adjustment
  a1 += a2;

  theta1 = a2 * 180.0 / PI; //hip s3
  theta2 = a1 * 180.0 / PI; // knee s2
  theta3 = theta3 * 180.0 / PI;

  return true;
}

int angleToPos(float angle) {
  while (angle < 0) angle += 360.0;
  while (angle >= 360) angle -= 360.0;

  float total_position = 4096.0;
  float digital_pos = (angle / 360.0) * total_position;
  return (int)round(digital_pos);
}

void move(float x, float y, float z) {

  if (IK(x, y, z)) {
    float s1_offset = theta3 + 90; //- 6; 
    float s2_offset = -theta2 + 180;
    float s3_offset = -theta1 + 90;
  
    int back_hip_pos_fr = round(angleToPos(s1_offset));
    int knee_pos_fr = round(angleToPos(s3_offset));
    int hip_pos_fr = round(angleToPos(s2_offset));

    st.SyncWritePosEx(ID, 3, Position, Speed, ACC);
  }
}

//Initialize servos and set theri respective offsets

void moveLeg_FR(float tx, float ty, float tz) {
  if (IK(tx, ty, tz)) {
    float s2_offset = 180 - theta2 + 20; // Hip
    float s3_offset = 90 - theta1 ;  // Knee
    
    byte legIDs[3] = {1, 2, 3};
    s16 legPos[3];
    legPos[0] = 2047;
    legPos[1] = round(angleToPos(s3_offset));
    legPos[2] = round(angleToPos(s2_offset));
    st.SyncWritePosEx(legIDs, 3, legPos, Speed, ACC);
  }
  else {
    Serial.println("IK Failed");
  }
}

void moveLeg_FL(float tx, float ty, float tz) {
  if (IK(tx, ty, tz)) {
    float s11_offset = 270 + theta1; // Hip
    float s12_offset = 180 + theta2 - 20;  // Knee
    
    byte legIDs[3] = {10, 11, 12};
    s16 legPos[3];
    legPos[0] = 2047;
    legPos[1] = round(angleToPos(s11_offset));
    legPos[2] = round(angleToPos(s12_offset));
    st.SyncWritePosEx(legIDs, 3, legPos, Speed, ACC);
  }
}

void moveLeg_BR(float tx, float ty, float tz) {
  if (IK(tx, ty, tz)) {
    float s6_offset = 180 - theta2 + 20; // Hip
    float s5_offset = 90 - theta1;  // Knee
    
    byte legIDs[3] = {4, 6, 5};
    s16 legPos[3];
    legPos[0] = 2047;//round(angleToPos(90));
    legPos[1] = round(angleToPos(s6_offset)); // Hip is ID 6
    legPos[2] = round(angleToPos(s5_offset)); // Knee is ID 5
    st.SyncWritePosEx(legIDs, 3, legPos, Speed, ACC);
  }
}

void moveLeg_BL(float tx, float ty, float tz) {
  if (IK(tx, ty, tz)) {
    float s9_offset = 180 + theta2 - 20; // Hip
    float s8_offset = 270 + theta1;  // Knee
    
    byte legIDs[3] = {7, 9, 8};
    s16 legPos[3];
    legPos[0] = 2047;//round(angleToPos(90));
    legPos[1] = round(angleToPos(s9_offset)); // Hip is ID   9
    legPos[2] = round(angleToPos(s8_offset)); // Knee is ID 8
    st.SyncWritePosEx(legIDs, 3, legPos, Speed, ACC);
  }
}


//Step trajectory calculations (forward walking)
void step_FR_Square(float phase, float step_len, float step_h, float y_pitch) {
  float tx, ty;
  float ground = -280.0 + y_pitch;
  
  if (phase < 0.25) {        // QUAD 1: Lift
    tx = -step_len / 2.0;
    ty = ground + (step_h * (phase / 0.25));
  } 
  else if (phase < 0.50) {   // QUAD 2: Swing Forward
    tx = (-step_len / 2.0) + (step_len * ((phase - 0.25) / 0.25));
    ty = ground + step_h;
  } 
  else if (phase < 0.75) {   // QUAD 3: Drop
    tx = step_len / 2.0;
    ty = (ground + step_h) - (step_h * ((phase - 0.50) / 0.25));
  } 
  else {                     // QUAD 4: Stance Push
    tx = (step_len / 2.0) - (step_len * ((phase - 0.75) / 0.25));
    ty = ground;
  }
  moveLeg_FR(tx + tx_offset, ty, 0);
}

void step_FL_Square(float phase, float step_len, float step_h, float y_pitch) {
  float tx, ty;
  float ground = -280.0 + y_pitch;
  
  if (phase < 0.25) {
    tx = -step_len / 2.0;
    ty = ground + (step_h * (phase / 0.25));
  } 
  else if (phase < 0.50) {
    tx = (-step_len / 2.0) + (step_len * ((phase - 0.25) / 0.25));
    ty = ground + step_h;
  } 
  else if (phase < 0.75) {
    tx = step_len / 2.0;
    ty = (ground + step_h) - (step_h * ((phase - 0.50) / 0.25));
  } 
  else {
    tx = (step_len / 2.0) - (step_len * ((phase - 0.75) / 0.25));
    ty = ground;
  }
  moveLeg_FL(tx + tx_offset, ty, 0);
}

void step_BR_Square(float phase, float step_len, float step_h, float y_pitch) {
  float tx, ty;
  float ground = -280.0 + y_pitch;
  
  if (phase < 0.25) {
    tx = -step_len / 2.0;
    ty = ground + (step_h * (phase / 0.25));
  } 
  else if (phase < 0.50) {
    tx = (-step_len / 2.0) + (step_len * ((phase - 0.25) / 0.25));
    ty = ground + step_h;
  } 
  else if (phase < 0.75) {
    tx = step_len / 2.0;
    ty = (ground + step_h) - (step_h * ((phase - 0.50) / 0.25));
  } 
  else {
    tx = (step_len / 2.0) - (step_len * ((phase - 0.75) / 0.25));
    ty = ground;
  }
  moveLeg_BR(tx + 20, ty, 0);
}

void step_BL_Square(float phase, float step_len, float step_h, float y_pitch) {
  float tx, ty;
  float ground = -280.0 + y_pitch;
  
  if (phase < 0.25) {
    tx = -step_len / 2.0;
    ty = ground + (step_h * (phase / 0.25));
  } 
  else if (phase < 0.50) {
    tx = (-step_len / 2.0) + (step_len * ((phase - 0.25) / 0.25));
    ty = ground + step_h;
  } 
  else if (phase < 0.75) {
    tx = step_len / 2.0;
    ty = (ground + step_h) - (step_h * ((phase - 0.50) / 0.25));
  } 
  else {
    tx = (step_len / 2.0) - (step_len * ((phase - 0.75) / 0.25));
    ty = ground;
  }
  moveLeg_BL(tx + 20, ty, 0);
}

//Step trajectory (backward)

void step_FR_Back_Square(float phase, float step_len, float step_h, float y_pitch) {
  float tx, ty;
  float ground = -280.0 + y_pitch;
  
  if (phase < 0.25) {        // QUAD 1: Lift
    tx = step_len / 2.0;     // Start at front
    ty = ground + (step_h * (phase / 0.25));
  } 
  else if (phase < 0.50) {   // QUAD 2: Swing Backward
    tx = (step_len / 2.0) - (step_len * ((phase - 0.25) / 0.25));
    ty = ground + step_h;
  } 
  else if (phase < 0.75) {   // QUAD 3: Drop
    tx = -step_len / 2.0;    // Land at back
    ty = (ground + step_h) - (step_h * ((phase - 0.50) / 0.25));
  } 
  else {                     // QUAD 4: Stance Push Forward
    tx = (-step_len / 2.0) + (step_len * ((phase - 0.75) / 0.25));
    ty = ground;
  }
  moveLeg_FR(tx + tx_offset, ty, 0);
}

void step_FL_Back_Square(float phase, float step_len, float step_h, float y_pitch) {
  float tx, ty;
  float ground = -280.0 + y_pitch;
  
  if (phase < 0.25) {
    tx = step_len / 2.0;
    ty = ground + (step_h * (phase / 0.25));
  } 
  else if (phase < 0.50) {
    tx = (step_len / 2.0) - (step_len * ((phase - 0.25) / 0.25));
    ty = ground + step_h;
  } 
  else if (phase < 0.75) {
    tx = -step_len / 2.0;
    ty = (ground + step_h) - (step_h * ((phase - 0.50) / 0.25));
  } 
  else {
    tx = (-step_len / 2.0) + (step_len * ((phase - 0.75) / 0.25));
    ty = ground;
  }
  moveLeg_FL(tx + tx_offset, ty, 0);
}

void step_BR_Back_Square(float phase, float step_len, float step_h, float y_pitch) {
  float tx, ty;
  float ground = -280.0 + y_pitch;
  
  if (phase < 0.25) {
    tx = step_len / 2.0;
    ty = ground + (step_h * (phase / 0.25));
  } 
  else if (phase < 0.50) {
    tx = (step_len / 2.0) - (step_len * ((phase - 0.25) / 0.25));
    ty = ground + step_h;
  } 
  else if (phase < 0.75) {
    tx = -step_len / 2.0;
    ty = (ground + step_h) - (step_h * ((phase - 0.50) / 0.25));
  } 
  else {
    tx = (-step_len / 2.0) + (step_len * ((phase - 0.75) / 0.25));
    ty = ground;
  }
  moveLeg_BR(tx + 20, ty, 0);
}

void step_BL_Back_Square(float phase, float step_len, float step_h, float y_pitch) {
  float tx, ty;
  float ground = -280.0 + y_pitch;
  
  if (phase < 0.25) {
    tx = step_len / 2.0;
    ty = ground + (step_h * (phase / 0.25));
  } 
  else if (phase < 0.50) {
    tx = (step_len / 2.0) - (step_len * ((phase - 0.25) / 0.25));
    ty = ground + step_h;
  } 
  else if (phase < 0.75) {
    tx = -step_len / 2.0;
    ty = (ground + step_h) - (step_h * ((phase - 0.50) / 0.25));
  } 
  else {
    tx = (-step_len / 2.0) + (step_len * ((phase - 0.75) / 0.25));
    ty = ground;
  }
  moveLeg_BL(tx + 20, ty, 0);
}

//Rotation part is handeled after this
// step trajectory (rotatiojn)

void RotateLeg_FR(float tx, float ty, float tz) {
  if (IK(tx, ty, tz)) {
    float s2_offset = 180 - theta2 + 20; // Hip
    float s3_offset = -theta1 + 90;  // Knee
    float s1_offset = 270 - theta3;//90 + theta3;
    
    byte legIDs[3] = {1, 2, 3};
    s16 legPos[3];
    legPos[0] = round(angleToPos(s1_offset)); 
    legPos[1] = round(angleToPos(s2_offset));
    legPos[2] = round(angleToPos(s3_offset));
    st.SyncWritePosEx(legIDs, 3, legPos, Speed, ACC);
  }
}

// Front Left: s10=BackHip, s11=Hip, s12=Knee
void RotateLeg_FL(float tx, float ty, float tz) {
  if (IK(tx, ty, tz)) {
    float s11_offset = 270 + theta1; // Hip
    float s12_offset = 180 + theta2 - 20;  // Knee
    float s10_offset = 270 - theta3;//90 + theta3;
    
    byte legIDs[3] = {10, 11, 12};
    s16 legPos[3];
    legPos[0] = round(angleToPos(s10_offset));
    legPos[1] = round(angleToPos(s11_offset));
    legPos[2] = round(angleToPos(s12_offset));
    st.SyncWritePosEx(legIDs, 3, legPos, Speed, ACC);
  }
}

// Back Right: s4=BackHip, s6=Hip, s5=Knee (Note the swap!)
void RotateLeg_BR(float tx, float ty, float tz) {
  if (IK(tx, ty, tz)) {
    float s6_offset = 180 - theta2 + 20; // Hip
    float s5_offset = 90 - theta1;  // Knee
    float s4_offset = 90 + theta3;//270 - theta3;
    
    byte legIDs[3] = {4, 6, 5};
    s16 legPos[3];
    legPos[0] = round(angleToPos(s4_offset));
    legPos[1] = round(angleToPos(s6_offset)); // Hip is ID 6
    legPos[2] = round(angleToPos(s5_offset)); // Knee is ID 5
    st.SyncWritePosEx(legIDs, 3, legPos, Speed, ACC);
  }
}

// Back Left: s7=BackHip, s9=Hip, s8=Knee (Note the swap!)
void RotateLeg_BL(float tx, float ty, float tz) {
  if (IK(tx, ty, tz)) {
    float s9_offset = 180 + theta2 - 20; // Hip
    float s8_offset = 270 + theta1;  // Knee
    float s7_offset = 90 + theta3;//270 - theta3;
    
    byte legIDs[3] = {7, 9, 8};
    s16 legPos[3];
    legPos[0] = round(angleToPos(s7_offset));
    legPos[1] = round(angleToPos(s9_offset)); // Hip is ID 9
    legPos[2] = round(angleToPos(s8_offset)); // Knee is ID 8
    st.SyncWritePosEx(legIDs, 3, legPos, Speed, ACC);
  }
}

//rotation part
void rotate_FR_Square(float phase, float turn_amt, float step_h) {
  float ty, tz;
  float tx = 0; // Keeping X neutral for pure rotation
  float ground = -260.0;
  
  if (phase < 0.25) {
    // QUADRANT 1: Vertical Lift
    float p = phase / 0.25; 
    tz = -turn_amt / 2.0;
    ty = ground + (step_h * p);
  } 
  else if (phase < 0.50) {
    // QUADRANT 2: Sideways Swing (In the air)
    float p = (phase - 0.25) / 0.25; 
    tz = (-turn_amt / 2.0) + (turn_amt * p);
    ty = ground + step_h;
  } 
  else if (phase < 0.75) {
    // QUADRANT 3: Vertical Drop
    float p = (phase - 0.50) / 0.25; 
    tz = turn_amt / 2.0;
    ty = (ground + step_h) - (step_h * p);
  } 
  else {
    // QUADRANT 4: Stance Push (On the ground)
    float p = (phase - 0.75) / 0.25; 
    tz = (turn_amt / 2.0) - (turn_amt * p);
    ty = ground;
  }

  RotateLeg_FR(tx + tx_offset, ty, tz);
}

void rotate_FL_Square(float phase, float turn_amt, float step_h) {
  float ty, tz;
  float tx = 0; 
  float ground = -260.0;
  
  float local_turn = -turn_amt; 

  if (phase < 0.25) {
    float p = phase / 0.25; 
    tz = -local_turn / 2.0;
    ty = ground + (step_h * p);
  } 
  else if (phase < 0.50) {
    float p = (phase - 0.25) / 0.25; 
    tz = (-local_turn / 2.0) + (local_turn * p);
    ty = ground + step_h;
  } 
  else if (phase < 0.75) {
    float p = (phase - 0.50) / 0.25; 
    tz = local_turn / 2.0;
    ty = (ground + step_h) - (step_h * p);
  } 
  else {
    float p = (phase - 0.75) / 0.25; 
    tz = (local_turn / 2.0) - (local_turn * p);
    ty = ground;
  }

  RotateLeg_FL(tx + tx_offset, ty, tz);
}

//rotation part
void rotate_BR_Square(float phase, float turn_amt, float step_h) {
  float ty, tz;
  float tx = 0; // Keeping X neutral for pure rotation
  float ground = -260.0;
  
  if (phase < 0.25) {
    float p = phase / 0.25; 
    tz = -turn_amt / 2.0;
    ty = ground + (step_h * p);
  } 
  else if (phase < 0.50) {
    float p = (phase - 0.25) / 0.25; 
    tz = (-turn_amt / 2.0) + (turn_amt * p);
    ty = ground + step_h;
  } 
  else if (phase < 0.75) {
    float p = (phase - 0.50) / 0.25; 
    tz = turn_amt / 2.0;
    ty = (ground + step_h) - (step_h * p);
  } 
  else {
    float p = (phase - 0.75) / 0.25; 
    tz = (turn_amt / 2.0) - (turn_amt * p);
    ty = ground;
  }

  RotateLeg_BR(tx + 20, ty, tz);
}

void rotate_BL_Square(float phase, float turn_amt, float step_h) {
  float ty, tz;
  float tx = 0; // Keeping X neutral for pure rotation
  float ground = -260.0;
  
  if (phase < 0.25) {
    float p = phase / 0.25; 
    tz = -turn_amt / 2.0;
    ty = ground + (step_h * p);
  } 
  else if (phase < 0.50) {
    float p = (phase - 0.25) / 0.25; 
    tz = (-turn_amt / 2.0) + (turn_amt * p);
    ty = ground + step_h;
  } 
  else if (phase < 0.75) {
    float p = (phase - 0.50) / 0.25; 
    tz = turn_amt / 2.0;
    ty = (ground + step_h) - (step_h * p);
  } 
  else {
    float p = (phase - 0.75) / 0.25; 
    tz = (turn_amt / 2.0) - (turn_amt * p);
    ty = ground;
  }

  RotateLeg_BL(tx + 20, ty, tz);
}


//automation part
void executeStepSequence() {
    // 1. WHILE WALKING (Steps 0 to 9)
    if (stepCounter < 30) {
        float phase1 = global_phase;
        float phase2 = fmodf(global_phase + 0.5f, 1.0f);

        step_FL_Square(phase1, step_length, step_height, pitch_val);
        step_BR_Square(phase1, step_length, step_height, pitch_val);
        step_FR_Square(phase2, step_length, step_height, pitch_val);
        step_BL_Square(phase2, step_length, step_height, pitch_val);

        global_phase += phase_speed;

        if (global_phase >= 1.0f) {
            global_phase = 0.0f;
            stepCounter++;
        }
    } 
    // 2. HOLD THE CROUCH (After Step 10)
    else {

        moveLeg_FL(49, -192, 0);
        moveLeg_FR(29, -192, 0);
        moveLeg_BL(49, -192, 0);
        moveLeg_BR(29, -192, 0);
    }
}

void setup() {
  Serial.begin(9600);
    
  // Connect to WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
  }
  Serial.println("\nConnected!");
  Serial.println(WiFi.localIP());

  // Start UDP
  udp.begin(localUdpPort);
  Serial.printf("UDP Listening on port %d\n", localUdpPort);

  // Servo setup
  Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
  st.pSerial = &Serial1;  delay(1000);

  // Initialize all 12 servos
  for (int i = 0; i < 12; i++) {
    ID[i] = i + 1;       
    Speed[i] = 3400;     // Set initial speed
    ACC[i] = 120;         // Set acceleration
  }

  Serial.println("Servo IDs 1-12 mapped successfully.");

  Wire.begin(5, 4);
  if (!ism330dhcx.begin_I2C(0x6A)) {
    ism330dhcx.begin_I2C(0x6B); 
  }

}

// main loop
void loop() {

  //IN this setup i am not using the height_offset to modify the pitch of the robot during walking or stabilizing as my current approach is very unstable
  //Might update this in future 

    sensors_event_t accel, gyro, temp;
    ism330dhcx.getEvent(&accel, &gyro, &temp);

// Calculate Pitch
    float raw_pitch = atan2(accel.acceleration.x, sqrt(accel.acceleration.y * accel.acceleration.y + accel.acceleration.z * accel.acceleration.z)) * 180 / PI;
    
    // Smooth the value (Low Pass Filter)
    current_stable_pitch = (current_stable_pitch * pitch_smoothing) + (raw_pitch * (1.0 - pitch_smoothing));
    //  clamping the value
    float gated_pitch = current_stable_pitch;

    //  if the tilt is tiny, treat it as 0 to stop servo jitter
    if (abs(gated_pitch) < PITCH_DEADZONE) {
        gated_pitch = 0;
    }

    if (gated_pitch > MAX_PITCH_ANGLE) gated_pitch = MAX_PITCH_ANGLE;
    if (gated_pitch < MIN_PITCH_ANGLE) gated_pitch = MIN_PITCH_ANGLE;

    float pitch_rad = gated_pitch * PI / 180.0;

    float height_offset = (robot_body_length / 2.0) * tan(-pitch_rad);


    int packetSize = udp.parsePacket();
    if (packetSize) {
        int len = udp.read(packetBuffer, 255);
        if (len > 0) packetBuffer[len] = 0;
        String command = String(packetBuffer);
        
        if (command == "W_ON")  states.w = true;
        if (command == "W_OFF") states.w = false;
        
        if (command == "S_ON")  states.s = true;
        if (command == "S_OFF") states.s = false;

        if (command == "D_ON")  states.d = true;
        if (command == "D_OFF") states.d = false;

        if (command == "A_ON") states.a = true;
        if (command == "A_OFF") states.a = false;

        if (command == "SEQ_ON")  states.seq = true;
        if (command == "SEQ_OFF") states.seq = false;
    }

    if (states.seq && !lastSeqButtonState) {
        sequenceRunning = !sequenceRunning; // Flip the state
        
        if (sequenceRunning) {
            stepCounter = 0;   // Start the walk from step 0
            global_phase = 0;  // Sync the phase
        }
    }
    lastSeqButtonState = states.seq; // Update the memory for the next loop cycle
    
    if (sequenceRunning) {
        executeStepSequence(); 
    }
    else if (states.w || states.s || states.d || states.a ) { 
        
        float phase1 = global_phase; 
        float phase2 = fmodf(global_phase + 0.5f, 1.0f);
        float turn_magnitude = 60.0; 

        if (states.w) {
            // Forward Gait
            step_FL_Square(phase1, step_length, step_height, pitch_val);
            step_BR_Square(phase1, step_length, step_height, pitch_val);
            step_FR_Square(phase2, step_length, step_height, pitch_val);
            step_BL_Square(phase2, step_length, step_height, pitch_val);
        } 
        else if (states.s) {
            // Backward Gait
            step_FL_Back_Square(phase1, step_length, step_height, pitch_val);
            step_BR_Back_Square(phase1, step_length, step_height, pitch_val);
            step_FR_Back_Square(phase2, step_length, step_height, pitch_val);
            step_BL_Back_Square(phase2, step_length, step_height, pitch_val);
        }
        else if (states.d) {
            rotate_FR_Square(phase1, turn_magnitude, step_height);
            moveLeg_FL(49, -222, 0);
            moveLeg_BR(49, -222, 0);
            rotate_BL_Square(phase1, turn_magnitude, step_height);
        }

        else if (states.a) {
          rotate_FL_Square(phase1, turn_magnitude, step_height);
          moveLeg_FR(49, -222, 0);
          moveLeg_BL(49, -222, 0);
          rotate_BR_Square(phase1, turn_magnitude, step_height);
        }

        global_phase += phase_speed;

        if (global_phase >= 0.99f) {
            global_phase = 0.0f; 
            
            // Return to neutral stance only if all movement buttons are released
            if (!states.w && !states.s && !states.d && !states.a) {
                // Using RotateLeg functions for reset ensures Z is centered
                moveLeg_FL(49, -242, 0);
                moveLeg_BR(29, -242, 0);
                moveLeg_FR(49, -242, 0);
                moveLeg_BL(29, -242, 0);
                Serial.println("Movement Cycle Finished. Stance Reset.");
            }
        }
    } 
    else {
        // IDLE STATE
        moveLeg_FL(49, -242, 0);
        moveLeg_BR(29, -242, 0);
        moveLeg_FR(49, -242, 0);
        moveLeg_BL(29, -242, 0);
    }

    delay(10);
}
