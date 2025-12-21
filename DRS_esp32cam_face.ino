#include <Arduino.h>
#include <list>

#include "esp_camera.h"
#include "human_face_detect_msr01.hpp"

// ---------------------------
// AI-Thinker ESP32-CAM pin map (OV2640)
// ---------------------------
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

// ---------------------------
// Motor GPIOs (your wiring)
// 13=UP, 12=DOWN, 14/15 horizontal
// ---------------------------
static const int PIN_UP    = 13;
static const int PIN_DOWN  = 12;
static const int PIN_LEFT  = 15;
static const int PIN_RIGHT = 14;

// ---------------------------
// Camera / performance
// ---------------------------
static const framesize_t FRAME_SIZE = FRAMESIZE_QVGA;     // 320x240 (good compromise)
static const pixformat_t PIXFORMAT  = PIXFORMAT_RGB565;   // direct for detector
static const int         XCLK_FREQ  = 20000000;
static const int         DETECT_EVERY_N = 2;              // detect every Nth frame

// Detector strictness (less false positives)
static HumanFaceDetectMSR01 detector(0.15F, 0.5F, 10, 0.2F);

// ---------------------------
// Control tuning (THIS is what fixes "too long")
// ---------------------------
static const int DEADZONE_X = 22;           // pixels around center => no horizontal pulses
static const int DEADZONE_Y = 22;           // pixels around center => no vertical pulses

static const uint32_t PULSE_MS        = 20; // motor ON-time per step (try 10..30)
static const uint32_t MIN_INTERVAL_MS = 120; // min time between pulses (try 80..200)

// ---------------------------
// State
// ---------------------------
enum Dir : int8_t { DIR_NONE=0, DIR_LEFT=-1, DIR_RIGHT=+1, DIR_UP=-2, DIR_DOWN=+2 };

static uint32_t frameNo = 0;
static bool     lastHadFace = false;
static int      lastCx = 0, lastCy = 0;
static uint32_t lastPulseMs = 0;     // for MIN_INTERVAL_MS gating
static uint32_t lastSignalMs = 0;    // for DT measurement

// ---------------------------
// Helpers
// ---------------------------
static void motor_all_stop() {
  digitalWrite(PIN_UP,    LOW);
  digitalWrite(PIN_DOWN,  LOW);
  digitalWrite(PIN_LEFT,  LOW);
  digitalWrite(PIN_RIGHT, LOW);
}

static void motor_pulse(Dir d) {
  // direction pins are mutually exclusive per axis
  motor_all_stop();

  switch (d) {
    case DIR_UP:
      digitalWrite(PIN_UP, HIGH);
      digitalWrite(PIN_DOWN, LOW);
      break;
    case DIR_DOWN:
      digitalWrite(PIN_UP, LOW);
      digitalWrite(PIN_DOWN, HIGH);
      break;
    case DIR_LEFT:
      digitalWrite(PIN_LEFT, HIGH);
      digitalWrite(PIN_RIGHT, LOW);
      break;
    case DIR_RIGHT:
      digitalWrite(PIN_LEFT, LOW);
      digitalWrite(PIN_RIGHT, HIGH);
      break;
    default:
      return;
  }

  delay(PULSE_MS);
  motor_all_stop();
}

static const char* dir_to_str(Dir d) {
  switch (d) {
    case DIR_UP: return "UP";
    case DIR_DOWN: return "DOWN";
    case DIR_LEFT: return "LEFT";
    case DIR_RIGHT: return "RIGHT";
    default: return "NONE";
  }
}

static bool init_camera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;

  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;

  config.pin_xclk  = XCLK_GPIO_NUM;
  config.pin_pclk  = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href  = HREF_GPIO_NUM;

  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;

  config.pin_pwdn  = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;

  config.xclk_freq_hz = XCLK_FREQ;
  config.pixel_format = PIXFORMAT;
  config.frame_size   = FRAME_SIZE;

  config.jpeg_quality = 12;                 // ignored for RGB565
  config.fb_location  = CAMERA_FB_IN_PSRAM;
  config.grab_mode    = CAMERA_GRAB_LATEST;
  config.fb_count     = 1;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("CAM init failed: 0x%x\n", err);
    return false;
  }

  sensor_t *s = esp_camera_sensor_get();
  if (s) {
    s->set_framesize(s, FRAME_SIZE);
    s->set_pixformat(s, PIXFORMAT);
    s->set_brightness(s, 0);
    s->set_contrast(s, 0);
    s->set_saturation(s, 0);
    s->set_gain_ctrl(s, 1);
    s->set_exposure_ctrl(s, 1);
    s->set_awb_gain(s, 1);
  }
  return true;
}

static void bestFace(std::list<dl::detect::result_t> &results,
                     bool &hasFace, int &x, int &y, int &w, int &h) {
  hasFace = false;
  int bestArea = 0;

  for (auto &r : results) {
    int b0=r.box[0], b1=r.box[1], b2=r.box[2], b3=r.box[3];
    int xx, yy, ww, hh;

    if (b2 > b0 && b3 > b1) {        // x1,y1,x2,y2
      xx=b0; yy=b1; ww=b2-b0; hh=b3-b1;
    } else {                         // x,y,w,h
      xx=b0; yy=b1; ww=b2; hh=b3;
    }
    if (ww<=0 || hh<=0) continue;

    int area = ww*hh;
    if (area > bestArea) {
      bestArea = area;
      x=xx; y=yy; w=ww; h=hh;
      hasFace = true;
    }
  }
}

static Dir decide_direction(int cx, int cy, int w, int h) {
  // choose ONE axis per pulse: whichever error is bigger (prevents diagonals)
  int centerX = w / 2;
  int centerY = h / 2;

  int dx = cx - centerX; // + => face right
  int dy = cy - centerY; // + => face down

  int adx = abs(dx);
  int ady = abs(dy);

  bool needH = adx > DEADZONE_X;
  bool needV = ady > DEADZONE_Y;

  if (!needH && !needV) return DIR_NONE;

  if (needH && (!needV || adx >= ady)) {
    return (dx < 0) ? DIR_LEFT : DIR_RIGHT;
  } else {
    return (dy < 0) ? DIR_UP : DIR_DOWN;
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  setCpuFrequencyMhz(240);

  pinMode(PIN_UP, OUTPUT);
  pinMode(PIN_DOWN, OUTPUT);
  pinMode(PIN_LEFT, OUTPUT);
  pinMode(PIN_RIGHT, OUTPUT);
  motor_all_stop();

  Serial.printf("PSRAM: %s size=%u free=%u\n",
                psramFound() ? "YES" : "NO",
                ESP.getPsramSize(),
                ESP.getFreePsram());

  if (!init_camera()) {
    Serial.println("Camera init FAILED.");
    while (true) delay(1000);
  }

  lastPulseMs  = millis();
  lastSignalMs = lastPulseMs;
  Serial.println("OK. PULSE control active. Serial prints ONLY when a pulse is emitted.");
}

void loop() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) return;

  frameNo++;
  bool doDetect = ((frameNo % DETECT_EVERY_N) == 0);

  if (doDetect) {
    bool hasFace=false;
    int x=0,y=0, bw=0,bh=0;

    std::list<dl::detect::result_t> &results =
      detector.infer((uint16_t*)fb->buf, {(int)fb->height, (int)fb->width, 3});

    bestFace(results, hasFace, x, y, bw, bh);

    if (hasFace) {
      lastCx = x + bw/2;
      lastCy = y + bh/2;
      lastHadFace = true;
    } else {
      lastHadFace = false;
    }
  }

  int w = fb->width;
  int h = fb->height;

  esp_camera_fb_return(fb);

  // Only consider pulsing if we have a face
  if (!lastHadFace) return;

  // Rate limit pulses
  uint32_t now = millis();
  if (now - lastPulseMs < MIN_INTERVAL_MS) return;

  Dir d = decide_direction(lastCx, lastCy, w, h);
  if (d == DIR_NONE) return;

  // Emit pulse + timing
  uint32_t dt = now - lastSignalMs;
  lastSignalMs = now;
  lastPulseMs  = now;

  motor_pulse(d);

  // Serial ONLY when we pulsed
  Serial.printf("DT %lu  PULSE=%s  FACE %d %d  IMG %dx%d\n",
                (unsigned long)dt, dir_to_str(d),
                lastCx, lastCy, w, h);
}