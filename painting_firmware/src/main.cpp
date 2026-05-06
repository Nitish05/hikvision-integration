#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <QuadEncoder.h>

// BNO055 on Teensy 4.1 default I2C (Wire): SDA=18, SCL=19
// I2C address: 0x28 (ADR low, default) or 0x29 (ADR high)
constexpr uint8_t BNO055_ADDR = 0x28;
constexpr int32_t BNO055_SENSOR_ID = 55;
constexpr uint32_t SAMPLE_INTERVAL_MS = 8;    // 125 Hz (BNO055 fusion caps ~100 Hz)
constexpr uint32_t BOOT_ZERO_DELAY_MS = 2000; // wait this long after begin() before capturing zero

// Three CALT CESI-S2000 draw-wire encoders via HW-221 interface boards.
// 2000 PPR × ×4 quadrature = 8000 counts/rev; 0.1 mm/pulse → 200 mm/rev → 0.025 mm/count.
constexpr double ENC_COUNTS_PER_REV = 8000.0;
constexpr double ENC_MM_PER_REV     = 200.0;
constexpr double ENC_MM_PER_COUNT   = ENC_MM_PER_REV / ENC_COUNTS_PER_REV;
// Per-encoder sign so positive enc*MM = string extending (cable getting longer).
constexpr double ENC_SIGN[3] = { -1.0, +1.0, -1.0 };

// Momentary push-button -> FR5 solenoid trigger.
// Wire: switch between SWITCH_PIN and GND. INPUT_PULLUP -> idle HIGH, pressed LOW.
constexpr uint8_t SWITCH_PIN = 6;
constexpr uint32_t SWITCH_DEBOUNCE_MS = 20;

Adafruit_BNO055 bno(BNO055_SENSOR_ID, BNO055_ADDR, &Wire);

// Teensy 4.1 hardware quadrature decoders (ENC1..ENC3). Pins must be on XBAR:
// valid set = {0,1,2,3,4,5,7,30,31,33}. Pullups enabled (last arg = 1).
QuadEncoder encoder1(1,  2,  3, 1);
QuadEncoder encoder2(2,  4,  5, 1);
QuadEncoder encoder3(3, 30, 31, 1);

static uint32_t bootMs = 0;
static uint32_t lastSampleMs = 0;
static bool zeroed = false;

// Debounce state for SWITCH_PIN. switchStateLogical is the last-emitted state.
static bool switchStateLogical = false;
static bool switchRawLast = false;
static uint32_t switchLastEdgeMs = 0;

// Reference quaternion representing "zero" pose.
static double qRefW = 1, qRefX = 0, qRefY = 0, qRefZ = 0;

// Continuous (unwrapped) angles — accumulate full turns instead of jumping at ±180°.
static double rollCont = 0, yawCont = 0;
static double rollPrev = 0, yawPrev = 0;
static bool unwrapPrimed = false;

static inline double unwrapStep(double prev, double curr) {
  double d = curr - prev;
  while (d >  180.0) d -= 360.0;
  while (d < -180.0) d += 360.0;
  return d;
}

static inline void quatConjugate(double w, double x, double y, double z,
                                 double &ow, double &ox, double &oy, double &oz) {
  ow = w; ox = -x; oy = -y; oz = -z;
}

static inline void quatMul(double aw, double ax, double ay, double az,
                           double bw, double bx, double by, double bz,
                           double &ow, double &ox, double &oy, double &oz) {
  ow = aw*bw - ax*bx - ay*by - az*bz;
  ox = aw*bx + ax*bw + ay*bz - az*by;
  oy = aw*by - ax*bz + ay*bw + az*bx;
  oz = aw*bz + ax*by - ay*bx + az*bw;
}

// Convert quaternion → Tait-Bryan (roll about X, pitch about Y, yaw about Z) in degrees.
static inline void quatToEulerDeg(double w, double x, double y, double z,
                                  double &rollDeg, double &pitchDeg, double &yawDeg) {
  const double sinp = 2.0 * (w*y - z*x);
  const double pitch = (fabs(sinp) >= 1.0) ? copysign(M_PI / 2.0, sinp) : asin(sinp);
  const double roll  = atan2(2.0 * (w*x + y*z), 1.0 - 2.0 * (x*x + y*y));
  const double yaw   = atan2(2.0 * (w*z + x*y), 1.0 - 2.0 * (y*y + z*z));
  rollDeg  = roll  * 180.0 / M_PI;
  pitchDeg = pitch * 180.0 / M_PI;
  yawDeg   = yaw   * 180.0 / M_PI;
}

// ---------------- Trilateration ----------------
// Three-sphere intersection: solve |P - Ai|^2 = Li^2 for i=1,2,3.
// Anchor placeholders — replace with surveyed world-frame coordinates (mm).
struct Vec3 { double x, y, z; };
// Anchor triangle side lengths (mm): |A1-A2|=251, |A2-A3|=308, |A3-A1|=310.
// Coordinates below place the triangle in z=0 with its centroid at the origin.
constexpr Vec3 ANCHOR1 = { +126.321,  -94.120,   0.0};   // right
constexpr Vec3 ANCHOR2 = { -124.679,  -94.120,   0.0};   // left
constexpr Vec3 ANCHOR3 = {   -1.641, +188.240,   0.0};   // front
// Zero-reference position of the handle when the user presses 'z'.
constexpr Vec3 P0 = {  0.0,   0.0,    85.0};

static inline Vec3   vsub(const Vec3 &a, const Vec3 &b) { return {a.x-b.x, a.y-b.y, a.z-b.z}; }
static inline Vec3   vadd(const Vec3 &a, const Vec3 &b) { return {a.x+b.x, a.y+b.y, a.z+b.z}; }
static inline Vec3   vscale(const Vec3 &a, double s)    { return {a.x*s,   a.y*s,   a.z*s};   }
static inline double vdot(const Vec3 &a, const Vec3 &b) { return a.x*b.x + a.y*b.y + a.z*b.z; }
static inline Vec3   vcross(const Vec3 &a, const Vec3 &b) {
  return { a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x };
}
static inline double vnorm(const Vec3 &a) { return sqrt(vdot(a, a)); }

// Baselines (string lengths at P0) populated on captureZero().
static double L1_baseline = 0, L2_baseline = 0, L3_baseline = 0;
static Vec3 lastPose = P0;
static uint32_t trilatFail = 0;

// Returns true on success. Disambiguates the two possible intersections by
// picking the one closest to lastP.
static bool trilaterate(const Vec3 &a1, const Vec3 &a2, const Vec3 &a3,
                        double L1, double L2, double L3,
                        const Vec3 &lastP, Vec3 &outP) {
  const Vec3 u = vsub(a2, a1);
  const Vec3 v = vsub(a3, a1);
  const Vec3 w = vcross(u, v);
  const double w2 = vdot(w, w);
  if (w2 < 1e-9) return false;  // collinear anchors

  const double du = 0.5 * (vdot(a2, a2) - vdot(a1, a1) + L1*L1 - L2*L2);
  const double dv = 0.5 * (vdot(a3, a3) - vdot(a1, a1) + L1*L1 - L3*L3);

  // Least-norm solution to u·P = du, v·P = dv  (lives in the plane ⊥ w).
  const Vec3 P_star = vscale(vadd(vscale(vcross(v, w), du),
                                  vscale(vcross(w, u), dv)), 1.0 / w2);

  // Parameterize along w: P(t) = P_star + t*w. Substitute into |P - a1|^2 = L1^2.
  const Vec3 r = vsub(P_star, a1);
  const double b = vdot(r, w);
  const double c = vdot(r, r) - L1*L1;
  const double disc = b*b - w2*c;
  if (disc < 0) return false;  // spheres don't intersect

  const double sq = sqrt(disc);
  const Vec3 Pplus  = vadd(P_star, vscale(w, (-b + sq) / w2));
  const Vec3 Pminus = vadd(P_star, vscale(w, (-b - sq) / w2));

  const Vec3 ep = vsub(Pplus,  lastP);
  const Vec3 em = vsub(Pminus, lastP);
  outP = (vdot(ep, ep) <= vdot(em, em)) ? Pplus : Pminus;
  return true;
}

static void captureZero() {
  imu::Quaternion q = bno.getQuat();
  qRefW = q.w(); qRefX = q.x(); qRefY = q.y(); qRefZ = q.z();
  zeroed = true;
  rollCont = 0; yawCont = 0;
  rollPrev = 0; yawPrev = 0;
  unwrapPrimed = true;
  encoder1.write(0);
  encoder2.write(0);
  encoder3.write(0);
  // Baseline string lengths when the handle sits at the zero-reference P0.
  L1_baseline = vnorm(vsub(ANCHOR1, P0));
  L2_baseline = vnorm(vsub(ANCHOR2, P0));
  L3_baseline = vnorm(vsub(ANCHOR3, P0));
  lastPose = P0;
  Serial.println(">rezero:1");
  Serial.println(">>> zeroed: handle assumed at P0, baselines recorded");
}

static void handleSerialCommand() {
  while (Serial.available() > 0) {
    const int c = Serial.read();
    if (c == 'z' || c == 'Z') {
      captureZero();
    } else if (c == 'r' || c == 'R') {
      zeroed = false;
      Serial.println(">>> zero cleared: showing absolute orientation");
    } else if (c == 'e' || c == 'E') {
      encoder1.write(0);
      encoder2.write(0);
      encoder3.write(0);
      Serial.println(">>> all encoder counts reset to 0");
    }
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}

  Wire.begin();

  if (!bno.begin()) {
    Serial.println("ERROR: BNO055 not detected. Check wiring / I2C address.");
    while (true) { delay(1000); }
  }

  delay(1000);
  bno.setExtCrystalUse(true);

  // Hardware quadrature decoders — counts in silicon, immune to ISR latency.
  encoder1.setInitConfig();
  encoder1.EncConfig.IndexTrigger      = DISABLE;
  encoder1.EncConfig.filterCount       = 5;
  encoder1.EncConfig.filterSamplePeriod = 5;
  encoder1.init();

  encoder2.setInitConfig();
  encoder2.EncConfig.IndexTrigger      = DISABLE;
  encoder2.EncConfig.filterCount       = 5;
  encoder2.EncConfig.filterSamplePeriod = 5;
  encoder2.init();

  encoder3.setInitConfig();
  encoder3.EncConfig.IndexTrigger      = DISABLE;
  encoder3.EncConfig.filterCount       = 5;
  encoder3.EncConfig.filterSamplePeriod = 5;
  encoder3.init();

  pinMode(SWITCH_PIN, INPUT_PULLUP);

  // Collinearity sanity check on anchor geometry.
  {
    const Vec3 u0 = vsub(ANCHOR2, ANCHOR1);
    const Vec3 v0 = vsub(ANCHOR3, ANCHOR1);
    const Vec3 w0 = vcross(u0, v0);
    if (vdot(w0, w0) < 1.0) {
      Serial.println("WARNING: anchors ANCHOR1/ANCHOR2/ANCHOR3 are near-collinear — trilateration ill-conditioned");
    }
  }

  bootMs = millis();
  Serial.println(">>> hold sensor still — boot-zero in 2 s");
  Serial.println(">>> send 'z' to re-zero all, 'r' to clear IMU zero, 'e' to zero encoder only");
}

void loop() {
  handleSerialCommand();

  const uint32_t now = millis();

  // Capture the boot zero once, after the sensor has settled.
  if (!zeroed && (now - bootMs) >= BOOT_ZERO_DELAY_MS) {
    captureZero();
  }

  if (now - lastSampleMs < SAMPLE_INTERVAL_MS) return;
  const uint32_t dt = now - lastSampleMs;
  lastSampleMs = now;

  imu::Quaternion q = bno.getQuat();
  const double enc1MM = encoder1.read() * ENC_MM_PER_COUNT * ENC_SIGN[0];
  const double enc2MM = encoder2.read() * ENC_MM_PER_COUNT * ENC_SIGN[1];
  const double enc3MM = encoder3.read() * ENC_MM_PER_COUNT * ENC_SIGN[2];

  // Relative quaternion: qRel = qRef^-1 * qCurrent  →  rotation from reference to current.
  double cw, cx, cy, cz;
  if (zeroed) {
    double irw, irx, iry, irz;
    quatConjugate(qRefW, qRefX, qRefY, qRefZ, irw, irx, iry, irz);
    quatMul(irw, irx, iry, irz, q.w(), q.x(), q.y(), q.z(), cw, cx, cy, cz);
  } else {
    cw = q.w(); cx = q.x(); cy = q.y(); cz = q.z();
  }

  double roll, pitch, yaw;
  quatToEulerDeg(cw, cx, cy, cz, roll, pitch, yaw);

  if (!unwrapPrimed) {
    rollPrev = roll; yawPrev = yaw;
    rollCont = roll; yawCont = yaw;
    unwrapPrimed = true;
  } else {
    rollCont += unwrapStep(rollPrev, roll);
    yawCont  += unwrapStep(yawPrev,  yaw);
    rollPrev = roll; yawPrev = yaw;
  }

  // Absolute string lengths = baseline at P0 + incremental change from encoders.
  const double L1 = L1_baseline + enc1MM;
  const double L2 = L2_baseline + enc2MM;
  const double L3 = L3_baseline + enc3MM;

  Vec3 pos;
  if (trilaterate(ANCHOR1, ANCHOR2, ANCHOR3, L1, L2, L3, lastPose, pos)) {
    lastPose = pos;
  } else {
    trilatFail++;
    pos = lastPose;
  }

  Serial.print(">enc1_mm:"); Serial.println(enc1MM, 3);
  Serial.print(">enc2_mm:"); Serial.println(enc2MM, 3);
  Serial.print(">enc3_mm:"); Serial.println(enc3MM, 3);
  Serial.print(">x:");  Serial.println(pos.x, 2);
  Serial.print(">y:");  Serial.println(pos.y, 2);
  Serial.print(">z:");  Serial.println(pos.z, 2);
  // Telemetry-only Tait-Bryan (rx=roll about world X, ry=pitch about world Y,
  // rz=yaw about world Z) at zero pose IMU body axes match world: X=right,
  // Y=front, Z=up. Bridge consumes the qw/qx/qy/qz lines below; the rx/ry/rz
  // emits exist for Teleplot debugging.
  Serial.print(">rx:"); Serial.println(rollCont, 2);
  Serial.print(">ry:"); Serial.println(pitch, 2);
  Serial.print(">rz:"); Serial.println(yawCont, 2);
  // qRel = qRef^-1 * qCurrent — body-frame rotation from zero pose to now.
  // Bridge composes this onto q_tcp_anchor for body-frame TCP rotation.
  Serial.print(">qw:"); Serial.println(cw, 6);
  Serial.print(">qx:"); Serial.println(cx, 6);
  Serial.print(">qy:"); Serial.println(cy, 6);
  Serial.print(">qz:"); Serial.println(cz, 6);
  Serial.print(">dt_ms:"); Serial.println(dt);

  static uint32_t lastFailPrint = 0;
  if (now - lastFailPrint >= 1000) {
    lastFailPrint = now;
    Serial.print(">trilatFail:"); Serial.println(trilatFail);
  }

  // Debounced switch -> emit on state change only. Idle HIGH, pressed LOW.
  const bool rawPressed = (digitalRead(SWITCH_PIN) == LOW);
  if (rawPressed != switchRawLast) {
    switchLastEdgeMs = now;
    switchRawLast = rawPressed;
  }
  if (rawPressed != switchStateLogical &&
      (now - switchLastEdgeMs) >= SWITCH_DEBOUNCE_MS) {
    switchStateLogical = rawPressed;
    Serial.print(">button:"); Serial.println(switchStateLogical ? 1 : 0);
  }
}
