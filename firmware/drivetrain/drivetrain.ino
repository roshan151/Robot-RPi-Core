/*
 * Drivetrain firmware v3 — framed, checksummed serial protocol.
 *
 * Board  : Arduino UNO/Nano-class (ATmega328P, FTDI or CH340 USB-serial)
 * Drivers: 2x DRV8871 breakout (one per motor, ILIM ~2.1 A via 30k)
 * Sensors: 2x TSINY-8370 dual-channel (quadrature) Hall encoders.
 *          Channel A drives the count interrupt; channel B is sampled
 *          on A's rising edge to give the TRUE rotation direction —
 *          counts are hardware-signed, not inferred from the command.
 *
 * Wiring (TSINY colours: yellow=A, white=B, blue=Vcc, green=GND,
 *         red/black = motor power -> DRV8871 MOTOR terminals only):
 *   D5  (PWM) -> LEFT  board IN1  (forward)
 *   D6  (PWM) -> LEFT  board IN2  (reverse)
 *   D9  (PWM) -> RIGHT board IN1  (forward)
 *   D10 (PWM) -> RIGHT board IN2  (reverse)
 *   D2        <- LEFT  encoder A (yellow)  INPUT_PULLUP, RISING int
 *   D4        <- LEFT  encoder B (white)   INPUT_PULLUP, sampled
 *   D3        <- RIGHT encoder A (yellow)  INPUT_PULLUP, RISING int
 *   D7        <- RIGHT encoder B (white)   INPUT_PULLUP, sampled
 *   5V / GND  -> both encoders' blue / green (NOT the 9 V rail — the
 *                encoders' internal 10k pull-up ties Vout to Vcc, so
 *                Vcc must equal the Arduino's logic voltage)
 *   Buck OUT+ (8-9 V)      -> each DRV8871 POWER+ (VM)
 *   Star point (buck OUT-) -> each DRV8871 POWER- and Arduino GND,
 *                             each on its OWN wire (no ground ring)
 *
 * ============================ PROTOCOL ============================
 * Every line, both directions:   $BODY*CS\n
 *   CS = two hex digits, XOR of every byte of BODY.
 *   Bytes outside a well-formed frame are ignored and counted, so
 *   line noise can corrupt at most the frame it touches — never
 *   inject a command or fake a reply.
 *
 * Host -> robot (BODY):
 *   P                        heartbeat; no reply (feeds link watchdog)
 *   Q,<seq>                  ping                        -> A,<seq>
 *   V,<seq>,<pwm>            set open-loop speed 0-255   -> A,<seq>
 *   I,<seq>,<dir>            open-loop drive F/B/L/R     -> A,<seq>
 *   S,<seq>                  stop (active brake)         -> A,<seq> (+D if move active)
 *   M,<seq>,<dir>,<pwm>,<ticks>  encoder-counted move    -> A,<seq> now,
 *                                                           D,<seq>,... when done
 *
 * Robot -> host:
 *   A,<seq>                  command accepted (re-sent verbatim if the
 *                            host retries the same seq — commands are
 *                            executed exactly once)
 *   N,<seq>,<reason>         command rejected (BADARG | BUSY)
 *   D,<seq>,<status>,<el>,<er>  counted move finished:
 *                            OK | TIMEOUT | NOISE | STOP | LINK
 *   E,<el>,<er>,<ol>,<or>,<ms>  telemetry every 100 ms: per-move counts,
 *                            then never-reset odometry totals and millis()
 *   W,<code>[,...]           warning: NOISE | MEMCORRUPT | RXBAD | LINK
 *   B,<hex>,<build>          boot: MCUSR reset-cause bits + build stamp
 *
 * Safety: while anything is driving, the firmware expects a valid
 * frame (any frame — P is cheapest) at least every LINK_TIMEOUT_MS.
 * If the host dies or the cable drops, the motors brake on their own.
 * ==================================================================
 */

#include <Arduino.h>

// ---------------------------------------------------------------- //
// Reset-cause capture.  MCUSR records WHY the chip last reset:
//   bit0 PORF=power-on  bit1 EXTRF=reset pin  bit2 BORF=brown-out
//   bit3 WDRF=watchdog
// Most bootloaders clear MCUSR but first copy it into r2; this
// .init0 stub saves r2 before anything else runs.
// ---------------------------------------------------------------- //
uint8_t reset_flags __attribute__((section(".noinit")));
void resetFlagsInit(void) __attribute__((naked)) __attribute__((used)) __attribute__((section(".init0")));
void resetFlagsInit(void) {
  __asm__ __volatile__ ("sts %0, r2\n" : "=m" (reset_flags));
}

// ------------------------------ Pins ---------------------------- //
#define IN1_L  5     // Left  motor forward (PWM)
#define IN2_L  6     // Left  motor reverse (PWM)
#define IN1_R  9     // Right motor forward (PWM)
#define IN2_R  10    // Right motor reverse (PWM)
#define ENC_L    2   // Left  encoder channel A (INT0)
#define ENC_R    3   // Right encoder channel A (INT1)
#define ENC_L_B  4   // Left  encoder channel B (sampled in ISR)
#define ENC_R_B  7   // Right encoder channel B (sampled in ISR)

// Per-side quadrature polarity.  The two motors are mirror-mounted, so
// one side usually needs inverting.  CALIBRATE after flashing: hand-roll
// each wheel in the robot's FORWARD direction and watch the counts
// (tests/test_encoders.py or the notebook's hand-spin cell).  Forward
// roll must count UP on both sides; if a side counts down, flip its
// invert to 1 and re-flash.
#define ENC_L_INVERT  0   // left encoder
#define ENC_R_INVERT  1   // right motor is mirror-mounted: forward travel
                          // spins it the opposite way, so its correctly-paired
                          // A/B decodes negative. Verified by hand-spin —
                          // forward roll gave L=+242, R=-187.

// --------------------------- Tunables --------------------------- //
#define PWM_MAX            255
#define RAMP_STEP          3      // PWM units per ramp tick
#define RAMP_MS            6      // -> 0.5 PWM/ms: 50% speed in ~250 ms.
                                  // Slow soft-start keeps the near-stall
                                  // inrush gentle; DRV8871 ILIM caps the rest.
// Dead-time held AFTER the brake engages, before the next move starts.
// Note where it is measured from: armDrive() sets brake_until_ms only once
// softStop()'s ramp has reached zero, so by the time this clock starts the
// wheels are already stopped and shorted.  The back-EMF dump is softStop()'s
// job and it is already finished -- what is left for this constant to cover is
// mechanical settling (gearbox backlash taking up, the chassis finishing its
// rock) so the next move starts from a genuinely still robot.
//
// Reversal gets longer because backlash has to be driven out the other side
// and the rocking is worse, not because the electrical case is different.
//
// Was 150/400.  Those were set when softStop() did not exist and the dead-time
// WAS the stop, so they had to cover the wheels spinning down too; they were
// never re-examined after the ramp took that job over.  Every step of a
// gesture pays one of these, and NO/DANCE are reversals end to end.
//
// If turns start ending with a visible lurch or the chassis is still rocking
// when the next move begins, these are the numbers to raise.  If BOOT:4
// (brown-out) frames appear, that is NOT this -- look at RAMP_MS instead.
#define BRAKE_MS_SAME      100    // dead-time, same direction
#define BRAKE_MS_REVERSE   250    // dead-time when any wheel flips direction
#define MOVE_TIMEOUT_MS    15000UL
#define LINK_TIMEOUT_MS    1000UL // stop if no valid frame while driving
#define TELEMETRY_MS       100UL
#define OPEN_LOOP_PWM      150    // default speed for I commands

// Sync trim (P-only, deliberately weak).  Kept small ON PURPOSE:
// boosting a wheel +50% pushed its DRV8871 into ILIM current-chopping,
// where MORE PWM makes the wheel SLOWER — the loop then latches at full
// correction and never recovers.  Gentle trim stays out of that region.
#define SYNC_KP            0.4f
#define SYNC_DIFF_MAX      300    // ticks of error the P term may see
#define SYNC_AUTHORITY_PCT 20     // max correction as % of base speed
#define SYNC_FLOOR_PCT     35     // never drive a wheel below this % of base

// Encoder ISR debounce: real edges are >1 ms apart at full speed; noise
// bursts from the driver MOSFET edges arrive microseconds apart.
#define MIN_PULSE_US       150UL

// Noise handling: bounce = plausible extra ticks; memory-scale = counts
// no ISR could physically reach mid-move (RAM hit by a supply transient).
#define MAX_NOISE_STRIKES  5
#define ABSURD_COUNT       100000L

// ------------------------ Encoder state ------------------------- //
// Quadrature: the ISR fires on channel A's rising edge and samples
// channel B, whose level at that instant encodes the TRUE rotation
// direction.  Counts are therefore hardware-signed:
//   * wrong-way motion (rolling back on a slope, overshoot during
//     braking) is measured, not miscounted as forward progress;
//   * a wheel dithering on an edge while stopped nets to ZERO
//     (+1/-1 alternate) instead of accumulating phantom travel.
// enc_dir_l/r hold each wheel's COMMANDED sign — no longer used for
// counting, only to convert signed counts into progress-along-command
// in the control loop.
volatile long enc_left  = 0;
volatile long enc_right = 0;
// Odometry totals: counted by the same ISRs but NEVER reset by a move
// and never touched by the noise repair, so the host can integrate
// position across move boundaries without losing the coast-down ticks
// that land between one move's D frame and the next move's reset.
volatile long odo_left  = 0;
volatile long odo_right = 0;
volatile int8_t enc_dir_l = 1;
volatile int8_t enc_dir_r = 1;
volatile unsigned long last_left_us  = 0;
volatile unsigned long last_right_us = 0;

static inline long labs32(long v) { return v < 0 ? -v : v; }

void isr_left() {
  unsigned long now = micros();
  if (now - last_left_us < MIN_PULSE_US) return;
  last_left_us = now;
  int8_t step = digitalRead(ENC_L_B) ? -1 : 1;   // B level = direction
#if ENC_L_INVERT
  step = -step;
#endif
  enc_left += step;
  odo_left += step;
}
void isr_right() {
  unsigned long now = micros();
  if (now - last_right_us < MIN_PULSE_US) return;
  last_right_us = now;
  int8_t step = digitalRead(ENC_R_B) ? -1 : 1;
#if ENC_R_INVERT
  step = -step;
#endif
  enc_right += step;
  odo_right += step;
}

// ------------------------- Motion state ------------------------- //
int  target_pwm_left   = 0;
int  target_pwm_right  = 0;
int  current_pwm_left  = 0;
int  current_pwm_right = 0;

bool          move_active   = false;   // encoder-counted move running
long          target_ticks  = 0;
uint8_t       move_seq      = 0;       // seq of the M command being served
char          current_dir   = '\0';
unsigned long move_start_ms = 0;
uint8_t       noise_strikes = 0;

bool          brake_pending  = false;  // armed move waiting out dead-time
unsigned long brake_until_ms = 0;
char          pending_dir    = '\0';
int           pending_speed  = 0;
long          pending_ticks  = 0;      // 0 = open-loop start
uint8_t       pending_seq    = 0;

int  open_loop_speed = OPEN_LOOP_PWM;
unsigned long last_ramp_ms = 0;
unsigned long last_rx_ms   = 0;        // any valid frame feeds this

// Soft-stop: slamming the active brake at full speed dumps both motors'
// kinetic energy into the supply in one transient — measured to hang
// this board (no BOD reset).  Instead every stop ramps PWM down fast
// (2x ramp rate, ~130 ms from 50% speed) and engages the brake only
// once the command reaches zero, when the energy left is small.
bool         stopping     = false;     // ramping down; brake engages at 0
unsigned int pending_hold = 0;         // dead-time to apply once braked

void softStop() {
  target_pwm_left  = 0;
  target_pwm_right = 0;
  stopping = true;                     // ramp section engages the brake
}

// --------------------------- TX side ---------------------------- //
void sendBody(const char* body) {
  uint8_t x = 0;
  for (const char* p = body; *p; ++p) x ^= (uint8_t)*p;
  Serial.print('$');
  Serial.print(body);
  Serial.print('*');
  if (x < 0x10) Serial.print('0');
  Serial.println(x, HEX);
}

void sendf(const char* fmt, ...) {
  char buf[72];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  sendBody(buf);
}

// ---------------------- Motor primitives ------------------------ //
// DRV8871 per board:  IN1=PWM,IN2=LOW forward · IN1=LOW,IN2=PWM reverse
//                     both LOW coast (auto-sleep) · both HIGH brake
void motorWrite(uint8_t in1, uint8_t in2, int pwm) {
  if (pwm == 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  } else if (pwm > 0) {
    analogWrite(in1, min(pwm, PWM_MAX));
    digitalWrite(in2, LOW);
  } else {
    digitalWrite(in1, LOW);
    analogWrite(in2, min(-pwm, PWM_MAX));
  }
}

void coastMotors() {
  target_pwm_left = target_pwm_right = 0;
  current_pwm_left = current_pwm_right = 0;
  motorWrite(IN1_L, IN2_L, 0);
  motorWrite(IN1_R, IN2_R, 0);
}

// Active brake: shorts each motor through its driver so spin energy
// burns off in the winding — fast stop, no surge into the supply, and
// a braked stationary motor draws no current.
void brakeMotors() {
  target_pwm_left = target_pwm_right = 0;
  current_pwm_left = current_pwm_right = 0;
  digitalWrite(IN1_L, HIGH);
  digitalWrite(IN2_L, HIGH);
  digitalWrite(IN1_R, HIGH);
  digitalWrite(IN2_R, HIGH);
}

// ------------------- Direction / reversal helpers ---------------- //
// NOTE: L and R are mirrored relative to the textbook mapping (which is
// L = -1,+1 and R = +1,-1).  This is deliberate and must not be "tidied".
//
// The harness has the left driver channel wired to the physical RIGHT wheel
// and vice versa.  That is invisible on F and B, whose sign pairs are
// symmetric, but it mirrors every turn -- the bug this compensates for was
// "commanded right 360, robot turned left".
//
// Compensating HERE rather than at the pin defines is safe only because the
// motor<->encoder pairing is self-consistent: channel L and enc_left refer to
// the same wheel as each other, they are just both the wheel on the right.
// The sync loop compares enc_left against enc_right and corrects channels L
// and R, so a consistent whole-robot relabel leaves it correct.  If the
// pairing is ever broken (channel L driving the wheel that enc_RIGHT
// measures) this fix is NOT sufficient -- the sync loop becomes positive
// feedback and must be fixed at the wiring or the pin defines instead.
// The tell is a sync error pinned near SYNC_AUTHORITY_PCT and always on the
// same side; drivetrain_client.py reports exactly that.
void wheelSigns(char dir, int8_t &left, int8_t &right) {
  switch (dir) {
    case 'F': left =  1; right =  1; break;
    case 'B': left = -1; right = -1; break;
    case 'L': left =  1; right = -1; break;   // mirrored -- see note above
    case 'R': left = -1; right =  1; break;   // mirrored -- see note above
    default:  left =  0; right =  0; break;
  }
}

// True if EITHER wheel must flip its rotation (covers F<->B and also
// e.g. B->L, which flips only the right wheel).
bool isReversal(char prev, char next) {
  int8_t pl, pr, nl, nr;
  wheelSigns(prev, pl, pr);
  wheelSigns(next, nl, nr);
  if (pl == 0 || nl == 0) return false;
  return (pl * nl < 0) || (pr * nr < 0);
}

// ------------------------ Move lifecycle ------------------------ //
// reset_counts: true for encoder-counted moves (each move measures its
// own travel from zero); false for open-loop intents, whose counters
// keep accumulating so the host can diff them across steps.
void beginDrive(char dir, int speed, long ticks, bool reset_counts) {
  stopping = false;                 // a new drive cancels any stop-in-progress
  coastMotors();                    // release brake; loop() ramps from 0
  current_dir = dir;

  int8_t ls, rs;
  wheelSigns(dir, ls, rs);
  noInterrupts();
  enc_dir_l = ls;
  enc_dir_r = rs;
  if (reset_counts) {
    enc_left  = 0;
    enc_right = 0;
    last_left_us  = 0;              // don't let a stale debounce stamp
    last_right_us = 0;              // eat the first real edge
  }
  interrupts();

  target_ticks  = ticks;
  move_active   = (ticks > 0);
  move_start_ms = millis();
  noise_strikes = 0;

  target_pwm_left  = ls * speed;
  target_pwm_right = rs * speed;
  last_ramp_ms = millis();
}

// End an encoder-counted move: soft-stop, then report D with counts.
void finishMove(const char* status) {
  move_active = false;
  softStop();
  long el, er;
  noInterrupts();
  el = enc_left;
  er = enc_right;
  interrupts();
  sendf("D,%u,%s,%ld,%ld", move_seq, status, el, er);
}

// Arm a move/open-loop start, inserting brake dead-time if the wheels
// may still be turning.  NON-BLOCKING: a delay() here would leave the
// serial port unread and starve the link watchdog.
void armDrive(char dir, int speed, long ticks, uint8_t seq) {
  bool driving   = move_active || brake_pending ||
                   current_pwm_left != 0 || current_pwm_right != 0 ||
                   target_pwm_left != 0 || target_pwm_right != 0;
  bool reversing = isReversal(current_dir, dir);

  // Streaming open-loop intents in the SAME direction just retarget —
  // no brake, no ramp restart — so brain_loop can repeat F at a few Hz
  // without stuttering.
  if (ticks == 0 && !reversing && driving && dir == current_dir &&
      !brake_pending && !move_active) {
    int8_t ls, rs;
    wheelSigns(dir, ls, rs);
    target_pwm_left  = ls * speed;
    target_pwm_right = rs * speed;
    move_seq = seq;
    return;
  }

  // Counted moves take the dead-time whenever anything is still turning;
  // open-loop starts only need it for a true direction reversal.
  bool need_hold = reversing || (ticks > 0 && driving);
  if (need_hold) {
    softStop();                     // ramp down; brake engages at zero
    pending_hold  = reversing ? BRAKE_MS_REVERSE : BRAKE_MS_SAME;
    brake_until_ms = 0;             // set once the brake actually engages
    pending_dir   = dir;
    pending_speed = speed;
    pending_ticks = ticks;
    pending_seq   = seq;
    brake_pending = true;
    move_active   = false;          // counting starts in beginDrive()
  } else {
    move_seq = seq;
    beginDrive(dir, speed, ticks, ticks > 0);
  }
}

// --------------------------- RX side ---------------------------- //
#define RX_BODY_MAX 48
enum RxState { RX_IDLE, RX_BODY, RX_CS1, RX_CS2 };
RxState  rx_state = RX_IDLE;
char     rx_body[RX_BODY_MAX + 1];
uint8_t  rx_len = 0;
uint8_t  rx_xor = 0;
uint8_t  rx_cs  = 0;
uint16_t rx_bad = 0;                 // dropped frames (checksum/framing)
uint16_t rx_bad_reported = 0;

// Exactly-once execution under host retries: if the same seq arrives
// again (our A was lost on the wire), re-send the cached reply without
// re-executing the command.
uint8_t  last_seq = 0xFF;
bool     have_last_reply = false;
char     last_reply[24];

void reply(uint8_t seq, const char* body) {
  strncpy(last_reply, body, sizeof(last_reply) - 1);
  last_reply[sizeof(last_reply) - 1] = '\0';
  last_seq = seq;
  have_last_reply = true;
  sendBody(body);
}

static int8_t hexVal(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}

void handleFrame(char* body) {
  char kind = body[0];
  if (kind == 'P' && body[1] == '\0') return;   // heartbeat: watchdog only

  unsigned int seq;
  char rbuf[24];

  if (kind == 'Q' || kind == 'S') {
    if (sscanf(body + 1, ",%u", &seq) != 1 || seq > 255) return;
    if (have_last_reply && (uint8_t)seq == last_seq) { sendBody(last_reply); return; }
    if (kind == 'S') {
      brake_pending = false;
      if (move_active) finishMove("STOP");
      else softStop();
      // current_dir is kept: the wheels may still be spinning down, so
      // the NEXT move must still get its reversal dead-time if needed.
    }
    snprintf(rbuf, sizeof(rbuf), "A,%u", seq);
    reply(seq, rbuf);
    return;
  }

  if (kind == 'V') {
    int v;
    if (sscanf(body + 1, ",%u,%d", &seq, &v) != 2 || seq > 255) return;
    if (have_last_reply && (uint8_t)seq == last_seq) { sendBody(last_reply); return; }
    v = constrain(v, 0, PWM_MAX);
    open_loop_speed = v;
    if (brake_pending && pending_ticks == 0) pending_speed = v;
    if (target_pwm_left  > 0) target_pwm_left  =  v;
    if (target_pwm_left  < 0) target_pwm_left  = -v;
    if (target_pwm_right > 0) target_pwm_right =  v;
    if (target_pwm_right < 0) target_pwm_right = -v;
    snprintf(rbuf, sizeof(rbuf), "A,%u", seq);
    reply(seq, rbuf);
    return;
  }

  if (kind == 'I') {
    char dir;
    if (sscanf(body + 1, ",%u,%c", &seq, &dir) != 2 || seq > 255) return;
    if (have_last_reply && (uint8_t)seq == last_seq) { sendBody(last_reply); return; }
    if (dir != 'F' && dir != 'B' && dir != 'L' && dir != 'R') {
      snprintf(rbuf, sizeof(rbuf), "N,%u,BADARG", seq);
      reply(seq, rbuf);
      return;
    }
    if (move_active) finishMove("STOP");   // intent overrides a counted move
    armDrive(dir, open_loop_speed, 0, (uint8_t)seq);
    snprintf(rbuf, sizeof(rbuf), "A,%u", seq);
    reply(seq, rbuf);
    return;
  }

  if (kind == 'M') {
    char dir;
    int speed;
    long ticks;
    if (sscanf(body + 1, ",%u,%c,%d,%ld", &seq, &dir, &speed, &ticks) != 4 ||
        seq > 255) return;
    if (have_last_reply && (uint8_t)seq == last_seq) { sendBody(last_reply); return; }
    if ((dir != 'F' && dir != 'B' && dir != 'L' && dir != 'R') ||
        speed <= 0 || speed > PWM_MAX || ticks <= 0) {
      snprintf(rbuf, sizeof(rbuf), "N,%u,BADARG", seq);
      reply(seq, rbuf);
      return;
    }
    if (move_active || (brake_pending && pending_ticks > 0)) {
      snprintf(rbuf, sizeof(rbuf), "N,%u,BUSY", seq);
      reply(seq, rbuf);
      return;
    }
    armDrive(dir, speed, ticks, (uint8_t)seq);
    snprintf(rbuf, sizeof(rbuf), "A,%u", seq);
    reply(seq, rbuf);
    return;
  }
  // Unknown kind on a VALID frame: host is newer than firmware.  Stay
  // silent (no seq to N against reliably) — the host will time out.
}

void pollSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '$') {                 // '$' anywhere restarts a frame:
      rx_state = RX_BODY;           // recovers instantly from garbage
      rx_len = 0;
      rx_xor = 0;
      continue;
    }
    switch (rx_state) {
      case RX_IDLE:
        break;                      // discard inter-frame noise
      case RX_BODY:
        if (c == '*') {
          rx_state = RX_CS1;
        } else if (c == '\r' || c == '\n') {
          rx_state = RX_IDLE;       // frame never finished
          rx_bad++;
        } else if (rx_len < RX_BODY_MAX) {
          rx_body[rx_len++] = c;
          rx_xor ^= (uint8_t)c;
        } else {
          rx_state = RX_IDLE;       // oversize — corrupt
          rx_bad++;
        }
        break;
      case RX_CS1: {
        int8_t v = hexVal(c);
        if (v < 0) { rx_state = RX_IDLE; rx_bad++; }
        else       { rx_cs = (uint8_t)v << 4; rx_state = RX_CS2; }
        break;
      }
      case RX_CS2: {
        int8_t v = hexVal(c);
        rx_state = RX_IDLE;
        if (v < 0) { rx_bad++; break; }
        if ((uint8_t)(rx_cs | v) == rx_xor) {
          rx_body[rx_len] = '\0';
          last_rx_ms = millis();    // ANY valid frame feeds the watchdog
          handleFrame(rx_body);
        } else {
          rx_bad++;
        }
        break;
      }
    }
  }
}

// ---------------------------- Setup ----------------------------- //
void setup() {
  uint8_t cause = MCUSR ? MCUSR : reset_flags;
  cause &= 0x0F;
  MCUSR = 0;

  Serial.begin(115200);

  pinMode(IN1_L, OUTPUT);
  pinMode(IN2_L, OUTPUT);
  pinMode(IN1_R, OUTPUT);
  pinMode(IN2_R, OUTPUT);
  pinMode(ENC_L,   INPUT_PULLUP);
  pinMode(ENC_R,   INPUT_PULLUP);
  pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_L), isr_left,  RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R), isr_right, RISING);

  coastMotors();
  delay(200);

  // Reset cause + build stamp.  __DATE__/__TIME__ come from the compiler,
  // so every fresh build announces itself — a stale flash can never
  // masquerade as current source.
  sendf("B,%X,drv8871-v5-odo built " __DATE__ " " __TIME__, cause);
  last_rx_ms = millis();
}

// ----------------------------- Loop ----------------------------- //
void loop() {
  pollSerial();

  unsigned long now = millis();

  // ---- Link watchdog: never drive without a live host ------------ //
  bool driving = move_active || brake_pending ||
                 current_pwm_left != 0 || current_pwm_right != 0 ||
                 target_pwm_left != 0 || target_pwm_right != 0;
  if (driving && !stopping && (now - last_rx_ms) > LINK_TIMEOUT_MS) {
    brake_pending = false;
    if (move_active) finishMove("LINK");
    else softStop();
    sendBody("W,LINK");
  }

  // ---- Brake dead-time expiry: start the armed move -------------- //
  // brake_until_ms stays 0 until the soft-stop finishes and the brake
  // actually engages — the dead-time is measured from wheels-stopped,
  // not from when the command arrived.
  if (brake_pending && brake_until_ms != 0 &&
      (long)(now - brake_until_ms) >= 0) {
    brake_pending = false;
    move_seq = pending_seq;
    if (pending_ticks > 0) {
      beginDrive(pending_dir, pending_speed, pending_ticks, true);
    } else {
      open_loop_speed = pending_speed;
      beginDrive(pending_dir, pending_speed, 0, false);  // open loop: no target
    }
  }

  // ---- PWM ramp + bounded sync trim ------------------------------ //
  if (now - last_ramp_ms >= RAMP_MS) {
    last_ramp_ms = now;

    int tl = target_pwm_left;
    int tr = target_pwm_right;

    if (move_active && tl != 0 && tr != 0) {
      long el, er;
      noInterrupts();
      el = enc_left;
      er = enc_right;
      interrupts();

      // Progress along each wheel's own commanded direction (>= 0),
      // comparable between wheels for F, B, L and R alike.
      long pl = el * (long)enc_dir_l;
      long pr = er * (long)enc_dir_r;

      long diff = pl - pr;                       // >0 = left ahead
      if (diff >  SYNC_DIFF_MAX) diff =  SYNC_DIFF_MAX;
      if (diff < -SYNC_DIFF_MAX) diff = -SYNC_DIFF_MAX;

      int base = (target_pwm_left < 0) ? -target_pwm_left : target_pwm_left;
      int lim  = (int)((long)base * SYNC_AUTHORITY_PCT / 100);
      int corr = (int)(SYNC_KP * (float)diff);
      if (corr >  lim) corr =  lim;
      if (corr < -lim) corr = -lim;

      int floor_pwm = (int)((long)base * SYNC_FLOOR_PCT / 100);
      int mag_l = base - corr;                   // left ahead -> slow left
      int mag_r = base + corr;
      if (mag_l < floor_pwm) mag_l = floor_pwm;  // a wheel commanded to 0
      if (mag_r < floor_pwm) mag_r = floor_pwm;  // stops counting and opens
      if (mag_l > PWM_MAX)   mag_l = PWM_MAX;    // the loop — never allow it
      if (mag_r > PWM_MAX)   mag_r = PWM_MAX;

      tl = (target_pwm_left  < 0) ? -mag_l : mag_l;
      tr = (target_pwm_right < 0) ? -mag_r : mag_r;
    }

    // Stops ramp down at double rate: quick, but nothing like the
    // full-speed brake slam that was hanging the board.
    int step = stopping ? (RAMP_STEP * 2) : RAMP_STEP;
    int step_l = current_pwm_left;
    int step_r = current_pwm_right;
    if      (step_l < tl) step_l = min(step_l + step, tl);
    else if (step_l > tl) step_l = max(step_l - step, tl);
    if      (step_r < tr) step_r = min(step_r + step, tr);
    else if (step_r > tr) step_r = max(step_r - step, tr);

    bool changed = (step_l != current_pwm_left) || (step_r != current_pwm_right);
    current_pwm_left  = step_l;
    current_pwm_right = step_r;

    // Only touch pins when driving or changing — writing 0 blindly
    // would release an engaged active brake.
    if (changed || current_pwm_left != 0 || current_pwm_right != 0) {
      motorWrite(IN1_L, IN2_L, current_pwm_left);
      motorWrite(IN1_R, IN2_R, current_pwm_right);
    }

    // Soft-stop completion: PWM reached zero — NOW engage the brake
    // (residual speed is low, so the transient is small) and start any
    // pending dead-time from this moment.
    if (stopping && current_pwm_left == 0 && current_pwm_right == 0) {
      stopping = false;
      brakeMotors();
      if (brake_pending) brake_until_ms = millis() + pending_hold;
    }
  }

  // ---- Move completion / timeout / encoder sanity ---------------- //
  if (move_active) {
    long el, er;
    noInterrupts();
    el = enc_left;
    er = enc_right;
    interrupts();

    long pl = el * (long)enc_dir_l;
    long pr = er * (long)enc_dir_r;
    if (pl < 0) pl = 0;      // net motion AGAINST the command (real, thanks
    if (pr < 0) pr = 0;      // to quadrature) counts as zero progress

    // Runaway detection: one channel wildly ahead of the other.
    // Completion uses min() of both wheels, so a noisy channel can
    // never end a move early — but it could stall one forever, hence
    // the repair below and the MOVE_TIMEOUT safety net.
    bool runaway = false;
    if (pl > 50 && pr > 50) {
      if (pl > 10 * pr || pr > 10 * pl) runaway = true;
    } else if (pl > 10000 && pr < 10) {
      runaway = true;
    } else if (pr > 10000 && pl < 10) {
      runaway = true;
    }

    if (runaway) {
      // Memory-scale corruption (counts no ISR could reach) means the
      // variable was damaged by a supply transient — repair it but
      // don't strike: aborting can't fix RAM.
      bool absurd = labs32(el) > ABSURD_COUNT || labs32(er) > ABSURD_COUNT;
      if (!absurd) noise_strikes++;
      if (noise_strikes > MAX_NOISE_STRIKES) {
        finishMove("NOISE");
      } else {
        // With quadrature, EMI bursts random-walk rather than inflate,
        // so gross inflation on one channel still marks it as the liar:
        // trust the SMALLER progress and snap only the corrupted
        // channel to it.  Re-assert the commanded signs too — they sit
        // next to the counters in RAM and share their corruption risk.
        long good = (pl < pr) ? pl : pr;
        int8_t ls, rs;
        wheelSigns(current_dir, ls, rs);
        noInterrupts();
        enc_dir_l = ls;
        enc_dir_r = rs;
        if (pl > pr) enc_left  = good * (long)ls;
        else         enc_right = good * (long)rs;
        interrupts();
        sendf(absurd ? "W,MEMCORRUPT,%ld,%ld" : "W,NOISE,%ld,%ld", el, er);
      }
    } else if (pl >= target_ticks && pr >= target_ticks) {
      finishMove("OK");
    } else if (now - move_start_ms > MOVE_TIMEOUT_MS) {
      finishMove("TIMEOUT");
    }
  }

  // ---- Telemetry -------------------------------------------------- //
  static unsigned long last_telem = 0;
  if (now - last_telem >= TELEMETRY_MS) {
    last_telem = now;
    long el, er, ol, orr;
    noInterrupts();
    el = enc_left;
    er = enc_right;
    ol = odo_left;
    orr = odo_right;
    interrupts();
    sendf("E,%ld,%ld,%ld,%ld,%lu", el, er, ol, orr, now);
  }

  // ---- Corrupt-frame counter (rate-limited) ------------------------ //
  static unsigned long last_rxbad = 0;
  if (rx_bad != rx_bad_reported && now - last_rxbad >= 5000) {
    last_rxbad = now;
    rx_bad_reported = rx_bad;
    sendf("W,RXBAD,%u", rx_bad);
  }
}
