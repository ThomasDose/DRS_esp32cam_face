/*
  DRS_ESP32CAM_VGA_DRV8833_FACE_ADAPTIVE_BRAKE_CONTINUOUS_0DEG.ino

  - AI-Thinker ESP32-CAM (OV2640)
  - VGA 640x480, RGB565
  - Face detect MSR01 (full frame)
  - DRV8833 motor driver
  - CONTINUOUS motor drive
  - Adaptive ACTIVE BRAKE on stop / direction change (stronger near target / larger error)
  - Anti-pendulum: hysteresis + minimum hold time (H + V)
  - LED (GPIO4): VERY DIM ON if face detected, OFF otherwise
  - NO SERIAL OUTPUT

  Wiring:
    Vertical motor:
      UP    -> GPIO13
      DOWN  -> GPIO12
    Horizontal motor:
      LEFT  -> GPIO14
      RIGHT -> GPIO15
*/

#include <Arduino.h>
#include <list>
#include "esp_camera.h"
#include "human_face_detect_msr01.hpp"

/* ===== Camera pins (AI Thinker ESP32-CAM) ===== */
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

/* ===== Motor & LED pins ===== */
static const int PIN_UP    = 13;
static const int PIN_DOWN  = 12;
static const int PIN_LEFT  = 14;
static const int PIN_RIGHT = 15;
static const int PIN_LED   = 4;

/* ===== LED PWM ===== */
static const int LEDC_CH   = 7;
static const int LEDC_FREQ = 5000;
static const int LEDC_RES  = 8;
static const uint8_t LED_DIM = 1;   // very dim (0..255)

/* ===== Camera config ===== */
static const framesize_t FRAME_SIZE = FRAMESIZE_VGA;   // 640x480
static const pixformat_t PIXFORMAT  = PIXFORMAT_RGB565;
static const int         XCLK_FREQ  = 20000000;

/* ===== Face detector ===== */
static HumanFaceDetectMSR01 detector(0.15F, 0.5F, 10, 0.2F);

/* ===== Speed & robustness ===== */
static const uint32_t DETECT_MIN_MS = 30;   // fast reaction (try 25..35)
static const uint8_t  LOSE_AFTER_MISSES = 2;

/* ===== Deadzone ===== */
static const int DEADZONE_X = 40;
static const int DEADZONE_Y = 40;

/* ===== Anti-pendulum (both axes) ===== */
static const int      HYST_X = 45;         // pixels beyond deadzone to allow H motion
static const int      HYST_Y = 45;         // pixels beyond deadzone to allow V motion
static const uint32_t HOLD_DIR_MS_H = 240; // minimum hold against L<->R flip
static const uint32_t HOLD_DIR_MS_V = 240; // minimum hold against U<->D flip

/* ===== Adaptive brake =====
   Brake time is mapped from (effective error) -> [MIN..MAX] per axis.
   effective error = max(0, abs(error) - deadzone)
*/
static const uint32_t BRAKE_MIN_H = 18;
static const uint32_t BRAKE_MAX_H = 120;
static const uint32_t BRAKE_MIN_V = 18;
static const uint32_t BRAKE_MAX_V = 120;

// scale factor: how much error corresponds to "full brake"
static const int ERR_FULL_H = 140;   // ~max useful horizontal error beyond deadzone
static const int ERR_FULL_V = 180;

enum Dir : int8_t {
  DIR_NONE  = 0,
  DIR_LEFT  = -1,
  DIR_RIGHT = +1,
  DIR_UP    = -2,
  DIR_DOWN  = +2
};

/* ===== State ===== */
static uint32_t lastDetectMs = 0;
static uint8_t  missCount   = 0;
static int imgW = 640, imgH = 480;

static Dir lastDir = DIR_NONE;

// axis memory for anti-pendulum
static Dir lastHDir = DIR_NONE;
static uint32_t lastHChangeMs = 0;
static Dir lastVDir = DIR_NONE;
static uint32_t lastVChangeMs = 0;

// last measured errors (for adaptive braking)
static int lastErrH = 0;  // abs(dx)
static int lastErrV = 0;  // abs(dy)

/* ===== LED ===== */
static void led_pwm_init() {
  ledcSetup(LEDC_CH, LEDC_FREQ, LEDC_RES);
  ledcAttachPin(PIN_LED, LEDC_CH);
  ledcWrite(LEDC_CH, 0);
}
static inline void led_on()  { ledcWrite(LEDC_CH, LED_DIM); }
static inline void led_off() { ledcWrite(LEDC_CH, 0); }

/* ===== Motor helpers ===== */
static inline void motor_coast() {
  digitalWrite(PIN_UP, LOW);
  digitalWrite(PIN_DOWN, LOW);
  digitalWrite(PIN_LEFT, LOW);
  digitalWrite(PIN_RIGHT, LOW);
}

static inline void brake_vertical() {
  digitalWrite(PIN_UP, HIGH);
  digitalWrite(PIN_DOWN, HIGH);
}

static inline void brake_horizontal() {
  digitalWrite(PIN_LEFT, HIGH);
  digitalWrite(PIN_RIGHT, HIGH);
}

static inline void motor_drive(Dir d) {
  motor_coast();
  switch(d) {
    case DIR_UP:    digitalWrite(PIN_UP, HIGH);    break;
    case DIR_DOWN:  digitalWrite(PIN_DOWN, HIGH);  break;
    case DIR_LEFT:  digitalWrite(PIN_LEFT, HIGH);  break;
    case DIR_RIGHT: digitalWrite(PIN_RIGHT, HIGH); break;
    default: break;
  }
}

/* ===== Utils ===== */
static inline int clampi(int v, int lo, int hi){ return (v<lo)?lo:((v>hi)?hi:v); }

static uint32_t map_brake_ms(int errEff, bool horizontal){
  if(horizontal){
    int e = clampi(errEff, 0, ERR_FULL_H);
    return BRAKE_MIN_H + (uint32_t)((BRAKE_MAX_H - BRAKE_MIN_H) * (uint32_t)e / (uint32_t)ERR_FULL_H);
  }else{
    int e = clampi(errEff, 0, ERR_FULL_V);
    return BRAKE_MIN_V + (uint32_t)((BRAKE_MAX_V - BRAKE_MIN_V) * (uint32_t)e / (uint32_t)ERR_FULL_V);
  }
}

/* ===== Camera init ===== */
static bool init_camera() {
  camera_config_t c;
  c.ledc_channel = LEDC_CHANNEL_0;
  c.ledc_timer   = LEDC_TIMER_0;

  c.pin_d0 = Y2_GPIO_NUM; c.pin_d1 = Y3_GPIO_NUM;
  c.pin_d2 = Y4_GPIO_NUM; c.pin_d3 = Y5_GPIO_NUM;
  c.pin_d4 = Y6_GPIO_NUM; c.pin_d5 = Y7_GPIO_NUM;
  c.pin_d6 = Y8_GPIO_NUM; c.pin_d7 = Y9_GPIO_NUM;

  c.pin_xclk = XCLK_GPIO_NUM;
  c.pin_pclk = PCLK_GPIO_NUM;
  c.pin_vsync = VSYNC_GPIO_NUM;
  c.pin_href = HREF_GPIO_NUM;
  c.pin_sscb_sda = SIOD_GPIO_NUM;
  c.pin_sscb_scl = SIOC_GPIO_NUM;
  c.pin_pwdn = PWDN_GPIO_NUM;
  c.pin_reset = RESET_GPIO_NUM;

  c.xclk_freq_hz = XCLK_FREQ;
  c.pixel_format = PIXFORMAT;
  c.frame_size = FRAME_SIZE;
  c.jpeg_quality = 12;
  c.fb_location = CAMERA_FB_IN_PSRAM;
  c.grab_mode = CAMERA_GRAB_LATEST;
  c.fb_count = 2;

  if (esp_camera_init(&c) != ESP_OK) return false;

  sensor_t *s = esp_camera_sensor_get();
  if (s) s->set_framesize(s, FRAME_SIZE);

  return true;
}

/* ===== Face selection ===== */
static bool bestFace(std::list<dl::detect::result_t>& r, int& cx, int& cy) {
  int best = 0; bool found = false;
  for (auto &f : r) {
    int x=f.box[0], y=f.box[1], w=f.box[2]-f.box[0], h=f.box[3]-f.box[1];
    int a=w*h;
    if (a > best) {
      best = a;
      cx = x + w/2;
      cy = y + h/2;
      found = true;
    }
  }
  return found;
}

/* ===== Direction decision (0°) + anti-pendulum both axes ===== */
static Dir decide_dir(int cx, int cy) {
  const int dx  = cx - imgW/2;
  const int dy  = cy - imgH/2;
  const int adx = abs(dx);
  const int ady = abs(dy);

  lastErrH = adx;
  lastErrV = ady;

  if (adx <= DEADZONE_X && ady <= DEADZONE_Y) return DIR_NONE;

  const uint32_t now = millis();

  if (adx >= ady) {
    // HORIZONTAL
    if (adx < (DEADZONE_X + HYST_X)) return DIR_NONE;

    Dir want = (dx < 0) ? DIR_LEFT : DIR_RIGHT;

    if ((lastHDir == DIR_LEFT && want == DIR_RIGHT) || (lastHDir == DIR_RIGHT && want == DIR_LEFT)) {
      if (now - lastHChangeMs < HOLD_DIR_MS_H) return lastHDir;
    }

    if (want != lastHDir) { lastHDir = want; lastHChangeMs = now; }
    return want;

  } else {
    // VERTICAL
    if (ady < (DEADZONE_Y + HYST_Y)) return DIR_NONE;

    Dir want = (dy < 0) ? DIR_UP : DIR_DOWN;

    if ((lastVDir == DIR_UP && want == DIR_DOWN) || (lastVDir == DIR_DOWN && want == DIR_UP)) {
      if (now - lastVChangeMs < HOLD_DIR_MS_V) return lastVDir;
    }

    if (want != lastVDir) { lastVDir = want; lastVChangeMs = now; }
    return want;
  }
}

/* ===== Apply with adaptive axis-specific brake ===== */
static void apply_dir(Dir d) {
  if (d == lastDir) {
    if (d == DIR_NONE) motor_coast();
    else motor_drive(d);
    return;
  }

  bool lastH = (lastDir==DIR_LEFT || lastDir==DIR_RIGHT);
  bool lastV = (lastDir==DIR_UP   || lastDir==DIR_DOWN);
  bool newH  = (d==DIR_LEFT || d==DIR_RIGHT);
  bool newV  = (d==DIR_UP   || d==DIR_DOWN);

  // effective errors beyond deadzone (0..)
  int errEffH = max(0, lastErrH - DEADZONE_X);
  int errEffV = max(0, lastErrV - DEADZONE_Y);

  // brake duration: choose based on axis involved
  uint32_t tH = map_brake_ms(errEffH, true);
  uint32_t tV = map_brake_ms(errEffV, false);

  motor_coast();

  if (lastH || newH) brake_horizontal();
  if (lastV || newV) brake_vertical();

  // If both axes involved, brake for the longer of the two demands
  uint32_t t = 0;
  if ((lastH||newH) && (lastV||newV)) t = (tH > tV) ? tH : tV;
  else if (lastH||newH)              t = tH;
  else if (lastV||newV)              t = tV;

  // short, adaptive brake
  if (t > 0) delay(t);

  motor_coast();

  if (d != DIR_NONE) motor_drive(d);
  lastDir = d;
}

/* ===== SETUP ===== */
void setup() {
  setCpuFrequencyMhz(240);

  pinMode(PIN_UP, OUTPUT);
  pinMode(PIN_DOWN, OUTPUT);
  pinMode(PIN_LEFT, OUTPUT);
  pinMode(PIN_RIGHT, OUTPUT);

  motor_coast();
  led_pwm_init();
  led_off();

  if (!init_camera()) while(true) delay(1000);

  lastDetectMs = millis();
}

/* ===== LOOP ===== */
void loop() {
  uint32_t now = millis();
  if (now - lastDetectMs < DETECT_MIN_MS) return;

  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) return;

  lastDetectMs = now;
  imgW = fb->width;
  imgH = fb->height;

  auto &res = detector.infer((uint16_t*)fb->buf, {imgH, imgW, 3});
  esp_camera_fb_return(fb);

  int cx=0, cy=0;
  if (bestFace(res, cx, cy)) {
    missCount = 0;
    led_on();
    apply_dir(decide_dir(cx, cy));
  } else {
    if (++missCount >= LOSE_AFTER_MISSES) {
      led_off();
      lastHDir = DIR_NONE;
      lastVDir = DIR_NONE;
      apply_dir(DIR_NONE);
    }
  }
}