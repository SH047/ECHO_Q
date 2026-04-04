/**
 * ECHO-Q  –  Arduino IMU Firmware
 * ================================
 * Runs on the Arduino Nano co-processor.
 *
 * Responsibilities
 * ----------------
 *  1. Read orientation (roll, pitch, yaw) from the BNO055 IMU over I2C.
 *  2. Read calibration status from BNO055.
 *  3. Publish three Float32 topics via rosserial at 50 Hz:
 *       /echo_q/imu/roll   (radians)
 *       /echo_q/imu/pitch  (radians)
 *       /echo_q/imu/yaw    (radians)
 *  4. Publish calibration status as a UInt8 bitmask:
 *       /echo_q/imu/cal    (sys | gyro | accel | mag – each 2 bits, 0-3)
 *  5. Blink the onboard LED at a rate indicating status:
 *       Fast (100 ms)  – calibrating
 *       Slow (500 ms)  – nominal / calibrated
 *       SOS  pattern   – IMU hardware fault
 *
 * Wiring
 * ------
 *  BNO055 VCC  → Arduino 3.3V
 *  BNO055 GND  → Arduino GND
 *  BNO055 SDA  → Arduino A4  (I2C SDA)
 *  BNO055 SCL  → Arduino A5  (I2C SCL)
 *  Arduino USB → Raspberry Pi USB (rosserial)
 *
 * Dependencies (install via Arduino Library Manager)
 * --------------------------------------------------
 *  • Adafruit BNO055
 *  • Adafruit Unified Sensor
 *  • rosserial_arduino
 *
 * Compile settings
 * ----------------
 *  Board: Arduino Nano
 *  Processor: ATmega328P (Old Bootloader)  ← use if upload fails
 *  Baud: 115200
 */

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8.h>

// ── Constants ────────────────────────────────────────────────────────────────
#define LED_PIN        13
#define PUBLISH_HZ     50                        // Target publish rate
#define PUBLISH_PERIOD (1000 / PUBLISH_HZ)       // ms per tick

#define DEG_TO_RAD     (PI / 180.0f)

// BNO055 calibration thresholds (0-3 per axis; 3 = fully calibrated)
#define CAL_THRESHOLD  2

// ── ROS node handle + publishers ────────────────────────────────────────────
ros::NodeHandle nh;

std_msgs::Float32 roll_msg;
std_msgs::Float32 pitch_msg;
std_msgs::Float32 yaw_msg;
std_msgs::UInt8   cal_msg;

ros::Publisher pub_roll ("/echo_q/imu/roll",  &roll_msg);
ros::Publisher pub_pitch("/echo_q/imu/pitch", &pitch_msg);
ros::Publisher pub_yaw  ("/echo_q/imu/yaw",   &yaw_msg);
ros::Publisher pub_cal  ("/echo_q/imu/cal",   &cal_msg);

// ── IMU ──────────────────────────────────────────────────────────────────────
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

bool     imu_ok         = false;
uint32_t last_publish   = 0;
bool     led_state      = false;
uint32_t last_led       = 0;
uint8_t  blink_period   = 100;   // ms – updated based on calibration state

// ── Helpers ──────────────────────────────────────────────────────────────────

/**
 * Pack the four BNO055 calibration scores (0-3 each) into one byte.
 *   Bits [7:6] = sys
 *   Bits [5:4] = gyro
 *   Bits [3:2] = accel
 *   Bits [1:0] = mag
 */
uint8_t pack_calibration(uint8_t sys, uint8_t gyro, uint8_t accel, uint8_t mag) {
  return ((sys   & 0x03) << 6)
       | ((gyro  & 0x03) << 4)
       | ((accel & 0x03) << 2)
       | ((mag   & 0x03) << 0);
}

bool is_calibrated(uint8_t sys, uint8_t gyro, uint8_t accel, uint8_t mag) {
  return (sys   >= CAL_THRESHOLD &&
          gyro  >= CAL_THRESHOLD &&
          accel >= CAL_THRESHOLD &&
          mag   >= CAL_THRESHOLD);
}

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
  pinMode(LED_PIN, OUTPUT);

  // Init rosserial at 115200 baud
  nh.getHardware()->setBaud(115200);
  nh.initNode();

  nh.advertise(pub_roll);
  nh.advertise(pub_pitch);
  nh.advertise(pub_yaw);
  nh.advertise(pub_cal);

  // Init BNO055
  if (!bno.begin()) {
    // IMU fault – spin and signal SOS
    while (true) {
      // S: ... (3 short)
      for (int i = 0; i < 3; i++) {
        digitalWrite(LED_PIN, HIGH); delay(150);
        digitalWrite(LED_PIN, LOW);  delay(150);
      }
      delay(300);
      // O: --- (3 long)
      for (int i = 0; i < 3; i++) {
        digitalWrite(LED_PIN, HIGH); delay(450);
        digitalWrite(LED_PIN, LOW);  delay(150);
      }
      delay(300);
      // S again
      for (int i = 0; i < 3; i++) {
        digitalWrite(LED_PIN, HIGH); delay(150);
        digitalWrite(LED_PIN, LOW);  delay(150);
      }
      delay(700);
      nh.spinOnce();   // keep rosserial alive even in fault state
    }
  }

  // Use NDOF fusion mode (all sensors active, absolute orientation)
  bno.setMode(OPERATION_MODE_NDOF);
  delay(100);

  imu_ok = true;
}

// ── Loop ─────────────────────────────────────────────────────────────────────
void loop() {
  uint32_t now = millis();

  // ── Publish at target rate ─────────────────────────────────────────────
  if (now - last_publish >= PUBLISH_PERIOD) {
    last_publish = now;

    // Get Euler angles (degrees) from BNO055 fusion engine
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

    // BNO055 Euler convention: X=yaw, Y=roll, Z=pitch  (Tait-Bryan)
    float yaw_rad   =  euler.x() * DEG_TO_RAD;
    float roll_rad  =  euler.y() * DEG_TO_RAD;
    float pitch_rad = -euler.z() * DEG_TO_RAD;   // negate: BNO pitch sign

    roll_msg.data  = roll_rad;
    pitch_msg.data = pitch_rad;
    yaw_msg.data   = yaw_rad;

    pub_roll.publish(&roll_msg);
    pub_pitch.publish(&pitch_msg);
    pub_yaw.publish(&yaw_msg);

    // Calibration status
    uint8_t sys_cal = 0, gyro_cal = 0, accel_cal = 0, mag_cal = 0;
    bno.getCalibration(&sys_cal, &gyro_cal, &accel_cal, &mag_cal);
    cal_msg.data = pack_calibration(sys_cal, gyro_cal, accel_cal, mag_cal);
    pub_cal.publish(&cal_msg);

    // Update LED blink rate based on calibration
    blink_period = is_calibrated(sys_cal, gyro_cal, accel_cal, mag_cal)
                   ? 500    // slow blink = good
                   : 100;   // fast blink = still calibrating
  }

  // ── LED heartbeat ──────────────────────────────────────────────────────
  if (now - last_led >= (uint32_t)blink_period) {
    last_led  = now;
    led_state = !led_state;
    digitalWrite(LED_PIN, led_state ? HIGH : LOW);
  }

  nh.spinOnce();
}
