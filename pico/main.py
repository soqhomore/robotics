"""
토끼 로봇 테스트 펌웨어 - USB 조작판 (Raspberry Pi Pico W / MicroPython, PCA9685 없음)

사용법
  1. Thonny로 이 파일을 Pico에 main.py로 저장한 뒤 Thonny를 닫는다 (USB 포트를 놓아 줘야 함)
  2. 컴퓨터의 크롬/엣지에서 panel.html을 열고 [Pico 연결] -> 포트 선택
  Thonny 셸에서 직접 쓸 때는 h 입력 -> 명령어 목록

배선 (괄호는 Pico 물리 핀 번호)
  서보 신호  얼굴 MG996 -> GP12 (16), 왼쪽 귀 MG90S -> GP11 (15), 오른쪽 귀 MG90S -> GP10 (14)
  서보 전원  빨강 -> 5V 어댑터 +, 갈색 -> 어댑터 -,  어댑터 - 와 Pico GND (38) 연결 (공통 GND 필수)
  콘덴서     KMG 16V 전해 콘덴서를 서보 전원 + / - 줄 사이에 (긴 다리 +, 흰 띠 쪽 -)
  MAX7219    DIN GP19 (25), CLK GP18 (24), CS GP17 (22), VCC VBUS (40), GND (28)

컴퓨터와 주고받는 형식 (한 줄에 하나)
  컴퓨터 -> Pico : {"c": "servo", "id": "face", "deg": 120}  같은 JSON, 또는 셸 명령
  Pico -> 컴퓨터 : @C 설정, @S 상태, @L 로그, @R 명령 결과 (패널이 {"c":"hello"}를 보낸 뒤부터)
"""

try:
    import asyncio
except ImportError:
    import uasyncio as asyncio
import gc
import json
import random
import select
import sys
import time
import machine
from machine import Pin, PWM, SPI

# ---------------- 핀 ----------------
FACE_PIN = 12
EAR_L_PIN = 11
EAR_R_PIN = 10
EYE_SCK_PIN = 18
EYE_MOSI_PIN = 19
EYE_CS_PIN = 17

CONFIG_FILE = "config.json"

# 패널에서 바꾸고 config.json에 저장할 수 있는 값들
DEFAULTS = {
    "face_center": 90, "face_min": 45, "face_max": 135, "face_glance": 25,
    "ear_l_down": 60, "ear_l_up": 130,
    "ear_r_down": 120, "ear_r_up": 50,
    "ear_min": 10, "ear_max": 170,
    "face_speed": 120, "ear_speed": 300,          # deg/s
    "servo_min_us": 500, "servo_max_us": 2500,    # 0도 / 180도 펄스폭
    "eye_brightness": 3, "eye_rotate": False, "eye_swap": False,
    "idle_anim": True,
    "alert_hold_ms": 1500,
}

BOOT_MS = time.ticks_ms()


def now():
    return time.ticks_ms()


def uptime():
    return time.ticks_diff(now(), BOOT_MS) / 1000


def after(deadline):
    return time.ticks_diff(now(), deadline) >= 0


# ================= 설정 =================

cfg = dict(DEFAULTS)
cfg_dirty = False


def load_config():
    try:
        with open(CONFIG_FILE) as f:
            saved = json.load(f)
        for k, v in saved.items():
            if k in DEFAULTS:
                cfg[k] = v
        return True
    except (OSError, ValueError):
        return False


def save_config():
    global cfg_dirty
    with open(CONFIG_FILE, "w") as f:
        json.dump(cfg, f)
    cfg_dirty = False


def coerce(key, value):
    kind = type(DEFAULTS[key])
    if kind is bool:
        return value in (True, 1, "1", "true", "on")
    return kind(float(value))


# ================= 패널 통신 / 로그 =================

streaming = False     # 패널이 hello를 보내면 True. Thonny 셸에서는 사람이 읽는 글만 출력
log_id = 0
stats = {"alerts": 0, "actions": 0, "loop_hz": 0, "loop_max_ms": 0, "mem_free": 0}


def emit(tag, obj):
    sys.stdout.write(tag + " " + json.dumps(obj) + "\n")


def log(msg, level="info"):
    global log_id
    log_id += 1
    if streaming:
        emit("@L", [log_id, round(uptime(), 1), level, msg])
    else:
        print("[{:7.1f}] {}{}".format(uptime(), "" if level == "info" else level.upper() + ": ", msg))


# ================= 서보 (Pico PWM 직접 구동) =================

servo_power = True


class Servo:
    def __init__(self, key, pin, lo_key, hi_key, rest_key, speed_key):
        self.key = key
        self.pwm = PWM(Pin(pin))
        self.pwm.freq(50)
        self.lo_key = lo_key
        self.hi_key = hi_key
        self.speed_key = speed_key
        self.pos = float(cfg[rest_key])
        self.target = self.pos
        self.boost = 1

    def limits(self):
        a, b = cfg[self.lo_key], cfg[self.hi_key]
        return min(a, b), max(a, b)

    def set(self, deg):
        lo, hi = self.limits()
        self.target = float(max(lo, min(hi, deg)))

    def write(self):
        lo_us, hi_us = cfg["servo_min_us"], cfg["servo_max_us"]
        self.pwm.duty_ns((lo_us + int(self.pos * (hi_us - lo_us) / 180)) * 1000)

    def off(self):
        self.pwm.duty_ns(0)          # 펄스 끔 = 힘 빠짐

    def step(self, dt):
        diff = self.target - self.pos
        if diff == 0 or not servo_power:
            return
        step = cfg[self.speed_key] * self.boost * dt
        self.pos = self.target if abs(diff) <= step else self.pos + (step if diff > 0 else -step)
        self.write()

    def moving(self):
        return self.pos != self.target

    def freeze(self):
        self.target = self.pos


face = None
ear_l = None
ear_r = None
servos = ()
SERVO_BY_KEY = {}


def make_servos():
    global face, ear_l, ear_r, servos, SERVO_BY_KEY
    face = Servo("face", FACE_PIN, "face_min", "face_max", "face_center", "face_speed")
    ear_l = Servo("ear_l", EAR_L_PIN, "ear_min", "ear_max", "ear_l_down", "ear_speed")
    ear_r = Servo("ear_r", EAR_R_PIN, "ear_min", "ear_max", "ear_r_down", "ear_speed")
    servos = (face, ear_l, ear_r)
    SERVO_BY_KEY = {s.key: s for s in servos}


def set_power(on):
    global servo_power
    servo_power = on
    for s in servos:
        if on:
            s.freeze()
            s.write()
        else:
            s.off()
    log("서보 힘 " + ("켜짐" if on else "빠짐 (손으로 돌릴 수 있음)"), "info" if on else "warn")


def ensure_power():
    if not servo_power:
        set_power(True)


def ears_up():
    ear_l.set(cfg["ear_l_up"])
    ear_r.set(cfg["ear_r_up"])


def ears_down():
    ear_l.set(cfg["ear_l_down"])
    ear_r.set(cfg["ear_r_down"])


def home():
    face.set(cfg["face_center"])
    ears_down()


_last_servo_ms = now()


def update_servos():
    global _last_servo_ms
    elapsed = time.ticks_diff(now(), _last_servo_ms)
    if elapsed < 20:
        return
    _last_servo_ms = now()
    for s in servos:
        s.step(elapsed / 1000)


# ================= 눈 (MAX7219 × 2) =================

class MAX7219Chain:
    def __init__(self, spi, cs, count):
        self.spi = spi
        self.cs = cs
        self.count = count
        self.cs.value(1)
        for reg, value in ((0x0F, 0), (0x09, 0), (0x0B, 7), (0x0A, cfg["eye_brightness"]), (0x0C, 1)):
            self.write_all(reg, [value] * count)
        self.fill(0x00)

    # values[i] -> i번째 모듈. 마지막에 보낸 데이터가 첫 모듈에 남으므로 거꾸로 보낸다
    def write_all(self, reg, values):
        buf = bytearray()
        for v in reversed(values):
            buf.append(reg)
            buf.append(v)
        self.cs.value(0)
        self.spi.write(buf)
        self.cs.value(1)

    def show(self, frames):
        for row in range(8):
            self.write_all(0x01 + row, [frame[row] for frame in frames])

    def fill(self, bits):
        for row in range(8):
            self.write_all(0x01 + row, [bits] * self.count)


# 8x8 비트맵, 한 줄에 1바이트 (MSB = 왼쪽 칸)
EXPRESSIONS = {
    "normal": (0x3C, 0x7E, 0xFF, 0xE7, 0xE7, 0xFF, 0x7E, 0x3C),   # 눈동자 있는 눈
    "round":  (0x3C, 0x42, 0x99, 0xBD, 0xBD, 0x99, 0x42, 0x3C),   # 경청: 동그랗게 뜬 눈
    "blink":  (0x00, 0x00, 0x00, 0x00, 0x7E, 0x00, 0x00, 0x00),
    "sleepy": (0x00, 0x00, 0x00, 0xFF, 0xE7, 0xFF, 0x7E, 0x3C),   # 반쯤 감은 눈
    "smile":  (0x00, 0x00, 0x18, 0x3C, 0x66, 0xC3, 0x81, 0x00),   # ^ 모양
    "heart":  (0x00, 0x66, 0xFF, 0xFF, 0xFF, 0x7E, 0x3C, 0x18),
    "x":      (0x81, 0x42, 0x24, 0x18, 0x18, 0x24, 0x42, 0x81),   # 오류 표시용
}


def reverse_bits(b):
    r = 0
    for _ in range(8):
        r = (r << 1) | (b & 1)
        b >>= 1
    return r


class Eyes:
    def __init__(self, chain):
        self.chain = chain
        self.shown = None
        self.last = None
        normal = EXPRESSIONS["normal"]
        self.base = ("normal", normal, normal)

    def _orient(self, rows):
        if not cfg["eye_rotate"]:
            return rows
        return [reverse_bits(b) for b in reversed(rows)]

    def draw(self, key, left, right, force=False):
        if key == self.shown and not force:
            return
        frames = [None, None]
        li = 1 if cfg["eye_swap"] else 0
        frames[li] = self._orient(left)
        frames[1 - li] = self._orient(right)
        self.chain.show(frames)
        self.shown = key
        self.last = (key, left, right)

    def render(self, name):
        rows = EXPRESSIONS[name]
        self.draw(name, rows, rows)

    def render_base(self):
        self.draw(*self.base)

    def set_base(self, key, left, right):
        self.base = (key, left, right)
        self.draw(key, left, right, force=True)

    def redraw(self):
        if self.last:
            self.draw(*self.last, force=True)

    def brightness(self, v):
        self.chain.write_all(0x0A, [v] * self.chain.count)

    def fill(self, bits):
        self.chain.fill(bits)
        self.shown = None


eyes = None   # 설정을 읽은 뒤 boot()에서 만든다


# ================= 동작 (시퀀스) =================

glance_side = 1
current_task = None
current_name = None
run_id = 0
repeat_info = None


async def wait_still(timeout_ms=6000):
    start = now()
    while any(s.moving() for s in servos) and time.ticks_diff(now(), start) < timeout_ms:
        await asyncio.sleep_ms(10)


async def act_alert():
    global glance_side
    stats["alerts"] += 1
    ears_up()
    face.set(cfg["face_center"] + glance_side * cfg["face_glance"])
    glance_side = -glance_side
    eyes.render("round")
    await asyncio.sleep_ms(cfg["alert_hold_ms"])
    home()
    eyes.render_base()
    await wait_still()


async def _look(sign):
    face.set(cfg["face_center"] + sign * cfg["face_glance"])
    eyes.render("round")
    await wait_still()
    await asyncio.sleep_ms(800)
    face.set(cfg["face_center"])
    eyes.render_base()
    await wait_still()


async def act_look_left():
    await _look(-1)


async def act_look_right():
    await _look(1)


async def act_ear_wiggle():
    for _ in range(3):
        ears_up()
        await wait_still()
        ears_down()
        await wait_still()


async def act_ear_alt():
    for _ in range(3):
        ear_l.set(cfg["ear_l_up"])
        ear_r.set(cfg["ear_r_down"])
        await wait_still()
        ear_l.set(cfg["ear_l_down"])
        ear_r.set(cfg["ear_r_up"])
        await wait_still()
    ears_down()
    await wait_still()


async def act_blink():
    for _ in range(2):
        eyes.render("blink")
        await asyncio.sleep_ms(120)
        eyes.render_base()
        await asyncio.sleep_ms(250)


async def act_sweep():
    lo, hi = face.limits()
    for deg in (lo, hi, cfg["face_center"]):
        face.set(deg)
        await wait_still()
        await asyncio.sleep_ms(200)
    for s, down, up in ((ear_l, "ear_l_down", "ear_l_up"), (ear_r, "ear_r_down", "ear_r_up")):
        s.set(cfg[up])
        await wait_still()
        await asyncio.sleep_ms(200)
        s.set(cfg[down])
        await wait_still()


async def act_self_test():
    log("셀프 테스트: 눈 표정")
    for name in EXPRESSIONS:
        eyes.render(name)
        await asyncio.sleep_ms(600)
    eyes.render_base()
    log("셀프 테스트: 서보 개별 동작")
    await act_sweep()
    log("셀프 테스트: 동시 동작")
    for sign in (1, -1):
        ears_up()
        face.set(cfg["face_center"] + sign * cfg["face_glance"])
        eyes.render("round")
        await wait_still()
        await asyncio.sleep_ms(300)
        ears_down()
        eyes.render_base()
        await wait_still()
    home()
    await wait_still()


async def act_power_test():
    for s in servos:
        s.boost = 3
    lo, hi = face.limits()
    for _ in range(5):
        face.set(hi)
        ears_up()
        await wait_still()
        face.set(lo)
        ears_down()
        await wait_still()
    for s in servos:
        s.boost = 1
    home()
    await wait_still()
    log("전원 테스트 끝. Pico가 재부팅되지 않았다면 통과")


ACTIONS = {
    "alert":      ("반응 동작 (귀·고개·눈)", act_alert),
    "look_left":  ("고개 왼쪽 (각도 −)", act_look_left),
    "look_right": ("고개 오른쪽 (각도 +)", act_look_right),
    "ear_wiggle": ("귀 쫑긋 ×3", act_ear_wiggle),
    "ear_alt":    ("귀 번갈아 ×3", act_ear_alt),
    "blink":      ("눈 깜빡 ×2", act_blink),
    "sweep":      ("전체 범위 왕복", act_sweep),
    "self_test":  ("셀프 테스트", act_self_test),
    "power_test": ("전원 강하 테스트", act_power_test),
}
ACTION_ORDER = ("alert", "look_left", "look_right", "ear_wiggle", "ear_alt",
                "blink", "sweep", "self_test", "power_test")


async def _runner(my_id, name, count, interval):
    global current_task, current_name, repeat_info
    label, fn = ACTIONS[name]
    try:
        for i in range(count):
            if count > 1:
                repeat_info = {"name": label, "i": i + 1, "n": count}
            await fn()
            stats["actions"] += 1
            if interval and i < count - 1:
                await asyncio.sleep_ms(interval)
        if count > 1:
            log("반복 테스트 완료: {} × {}".format(label, count))
    except asyncio.CancelledError:
        log("동작 중단: " + label, "warn")
    except Exception as e:
        log("동작 오류: {} ({})".format(label, e), "error")
    finally:
        for s in servos:
            s.boost = 1
        if my_id == run_id:      # 그 사이 새 동작이 시작됐으면 건드리지 않는다
            current_task = None
            current_name = None
            repeat_info = None
            eyes.render_base()


def start_action(name, count=1, interval=0):
    global current_task, current_name, run_id
    if name not in ACTIONS:
        return "모르는 동작: {}".format(name)
    stop_action()
    ensure_power()
    run_id += 1
    current_name = name
    current_task = asyncio.create_task(_runner(run_id, name, max(1, count), max(0, interval)))
    return None


def stop_action():
    global current_task, current_name, repeat_info
    if current_task:
        current_task.cancel()
        current_task = None
        current_name = None
        repeat_info = None
        for s in servos:
            s.freeze()


# ================= 평상시 표정 =================

blink_ends_at = None
next_blink_at = now()
mood_ends_at = None
next_mood_at = now()
mood_expr = "smile"


def update_idle_face():
    global blink_ends_at, next_blink_at, mood_ends_at, next_mood_at, mood_expr
    if current_task or not cfg["idle_anim"]:
        blink_ends_at = mood_ends_at = None
        return
    t = now()
    if mood_ends_at is not None and after(mood_ends_at):
        mood_ends_at = None
        if blink_ends_at is None:
            eyes.render_base()
    if blink_ends_at is not None and after(blink_ends_at):
        blink_ends_at = None
        if mood_ends_at is not None:
            eyes.render(mood_expr)
        else:
            eyes.render_base()
    if blink_ends_at is None and after(next_blink_at):
        eyes.render("blink")
        blink_ends_at = time.ticks_add(t, 120)
        next_blink_at = time.ticks_add(t, random.randint(2500, 6000))
    if mood_ends_at is None and blink_ends_at is None and after(next_mood_at):
        mood_expr = random.choice(("smile", "sleepy"))
        eyes.render(mood_expr)
        mood_ends_at = time.ticks_add(t, 2000)
        next_mood_at = time.ticks_add(t, random.randint(12000, 20000))


led = Pin("LED", Pin.OUT)   # 보드 LED: 컴퓨터 패널이 연결되어 있으면 켜짐


# ================= 명령 처리 (패널/셸 공통) =================

def capture_servo(key):
    return {"face_center": face, "face_min": face, "face_max": face,
            "ear_l_down": ear_l, "ear_l_up": ear_l, "ear_r_down": ear_r, "ear_r_up": ear_r}.get(key)


def apply_setting(key):
    if key == "eye_brightness":
        eyes.brightness(cfg[key])
    elif key in ("eye_rotate", "eye_swap"):
        eyes.redraw()
    elif key in ("servo_min_us", "servo_max_us") and servo_power:
        for s in servos:
            s.write()
    for s in servos:
        s.set(s.target)       # 범위가 바뀌었으면 목표를 다시 제한


def ok(**extra):
    extra["ok"] = True
    return extra


def fail(msg):
    return {"ok": False, "err": msg}


def do(c):
    global cfg, cfg_dirty, streaming
    name = c.get("c")

    # ---- 패널 연결 ----
    if name == "hello":
        streaming = True
        emit("@C", {"cfg": cfg, "exprs": EXPRESSIONS,
                    "actions": [[k, ACTIONS[k][0]] for k in ACTION_ORDER],
                    "reset": reset_cause, "boot": BOOT_ID})
        led.value(1)
        log("패널 연결됨")
        return ok()
    if name == "bye":
        streaming = False
        led.value(0)
        return ok()

    # ---- 서보 ----
    if name in ("servo", "nudge"):
        s = SERVO_BY_KEY.get(c.get("id"))
        if not s:
            return fail("모르는 서보")
        stop_action()
        ensure_power()
        s.set(s.target + float(c["delta"]) if name == "nudge" else float(c["deg"]))
        return ok()
    if name == "ears":
        stop_action()
        ensure_power()
        if c.get("up"):
            ears_up()
        else:
            ears_down()
        return ok()
    if name == "home":
        stop_action()
        ensure_power()
        home()
        return ok()
    if name == "relax":
        stop_action()
        set_power(False)
        return ok()
    if name == "enable":
        set_power(True)
        return ok()
    if name == "stop":
        stop_action()
        return ok()

    # ---- 눈 ----
    if name == "expr":
        e = c.get("name")
        if e not in EXPRESSIONS:
            return fail("모르는 표정")
        rows = EXPRESSIONS[e]
        eyes.set_base(e, rows, rows)
        return ok()
    if name == "pixels":
        _, base_l, base_r = eyes.base
        left = c.get("left") or base_l
        right = c.get("right") or base_r
        if len(left) != 8 or len(right) != 8:
            return fail("픽셀은 8줄이어야 함")
        eyes.set_base("custom", [int(b) & 0xFF for b in left], [int(b) & 0xFF for b in right])
        return ok()
    if name == "eye_test":
        eyes.fill(0xFF)
        time.sleep_ms(700)
        eyes.redraw()
        return ok()

    # ---- 동작 ----
    if name == "action":
        err = start_action(c.get("name"), int(c.get("count", 1)), int(c.get("interval", 0)))
        return fail(err) if err else ok()

    # ---- 설정 ----
    if name == "set":
        key = c.get("key")
        if key not in DEFAULTS:
            return fail("모르는 설정: {}".format(key))
        try:
            cfg[key] = coerce(key, c.get("value"))
        except (TypeError, ValueError):
            return fail("값 형식 오류")
        cfg_dirty = True
        apply_setting(key)
        return ok(cfg=cfg)
    if name == "capture":
        key = c.get("key")
        s = capture_servo(key)
        if not s:
            return fail("현재 위치를 저장할 수 없는 항목")
        cfg[key] = int(round(s.pos))
        cfg_dirty = True
        apply_setting(key)
        log("보정: {} = {}".format(key, cfg[key]))
        return ok(cfg=cfg)
    if name == "save":
        save_config()
        log("설정 저장됨 (config.json)")
        return ok(cfg=cfg)
    if name == "defaults":
        cfg = dict(DEFAULTS)
        cfg_dirty = True
        for key in ("eye_brightness", "eye_rotate"):
            apply_setting(key)
        log("기본값으로 되돌림 (저장 전)", "warn")
        return ok(cfg=cfg)

    # ---- 진단 ----
    if name == "reset_stats":
        for k in ("alerts", "actions"):
            stats[k] = 0
        log("통계 초기화")
        return ok()
    if name == "reboot":
        log("재부팅", "warn")
        asyncio.create_task(_reboot())
        return ok()
    return fail("모르는 명령: {}".format(name))


async def _reboot():
    await asyncio.sleep_ms(300)
    machine.reset()


# ================= 입력 (USB 시리얼) =================

SHELL_HELP = """--- 셸 명령 (전체 조작은 컴퓨터에서 panel.html) ---
f/l/r <각도>     얼굴 / 왼쪽 귀 / 오른쪽 귀
up / down        귀 세우기 / 내리기       c  기본 자세
e <표정>         {exprs}
a <동작> [횟수]  {acts}
x  동작 정지     relax  힘 빼기
s  상태          save  설정 저장"""


def print_status():
    print("face={:.0f} earL={:.0f} earR={:.0f} 힘={} 동작={}".format(
        face.pos, ear_l.pos, ear_r.pos, "ON" if servo_power else "OFF", current_name or "-"))


def handle_text(line):
    p = line.split()
    c, a = p[0], (p[1] if len(p) > 1 else None)
    keys = {"f": "face", "l": "ear_l", "r": "ear_r"}
    try:
        if c in keys and a:
            res = do({"c": "servo", "id": keys[c], "deg": int(a)})
        elif c in ("up", "down"):
            res = do({"c": "ears", "up": c == "up"})
        elif c == "c":
            res = do({"c": "home"})
        elif c == "e" and a:
            res = do({"c": "expr", "name": a})
        elif c == "a" and a:
            res = do({"c": "action", "name": a, "count": int(p[2]) if len(p) > 2 else 1, "interval": 500})
        elif c == "x":
            res = do({"c": "stop"})
        elif c == "relax":
            res = do({"c": "relax"})
        elif c == "save":
            res = do({"c": "save"})
        elif c == "s":
            print_status()
            return
        else:
            print(SHELL_HELP.format(exprs=" ".join(EXPRESSIONS), acts=" ".join(ACTION_ORDER)))
            return
    except ValueError:
        print("숫자를 확인하세요 (예: f 120)")
        return
    if not res.get("ok"):
        print("오류:", res.get("err"))


def handle_line(line):
    if line.startswith("{"):
        try:
            res = do(json.loads(line))
        except (ValueError, KeyError, TypeError) as e:
            res = fail("잘못된 명령 ({})".format(e))
        if streaming:
            emit("@R", res)
        elif not res.get("ok"):
            print("오류:", res.get("err"))
    else:
        handle_text(line)


_stdin = select.poll()
_stdin.register(sys.stdin, select.POLLIN)
_line = []


def check_input():
    # 한 글자씩 모아서 줄이 끝나면 처리 (줄이 덜 도착해도 멈추지 않음)
    while _stdin.poll(0):
        ch = sys.stdin.read(1)
        if ch in ("\n", "\r"):
            if _line:
                line = "".join(_line).strip()
                del _line[:]
                if line:
                    handle_line(line)
        elif len(_line) < 2000:
            _line.append(ch)


# ================= 상태 전송 =================

def send_state():
    emit("@S", {
        "t": round(uptime(), 1),
        "mode": current_name or "idle",
        "power": servo_power,
        "dirty": cfg_dirty,
        "servos": {s.key: {"pos": round(s.pos, 1), "target": round(s.target, 1),
                           "min": s.limits()[0], "max": s.limits()[1]} for s in servos},
        "expr": eyes.shown,
        "base": eyes.base[0],
        "rep": repeat_info,
        "stats": stats,
    })


# ================= 메인 루프 =================

async def control_loop():
    last = now()
    window_start = now()
    last_state = now()
    loops = 0
    worst = 0
    last_lag_log = 0
    while True:
        t = now()
        gap = time.ticks_diff(t, last)
        last = t
        loops += 1
        if gap > worst:
            worst = gap
        if gap > 500 and time.ticks_diff(t, last_lag_log) > 5000:
            last_lag_log = t
            log("제어 루프 {}ms 멈춤 (서보 움직임이 끊길 수 있음)".format(gap), "warn")

        check_input()
        update_servos()
        update_idle_face()

        if streaming and time.ticks_diff(t, last_state) >= 150:
            last_state = t
            send_state()

        span = time.ticks_diff(t, window_start)
        if span >= 2000:
            stats["loop_hz"] = int(loops * 1000 / span)
            stats["loop_max_ms"] = worst
            gc.collect()
            stats["mem_free"] = gc.mem_free()
            loops = 0
            worst = 0
            window_start = t
        await asyncio.sleep_ms(0)


RESET_NAMES = {}
for _name in ("PWRON_RESET", "WDT_RESET", "HARD_RESET", "SOFT_RESET", "DEEPSLEEP_RESET"):
    if hasattr(machine, _name):
        RESET_NAMES[getattr(machine, _name)] = _name
reset_cause = RESET_NAMES.get(machine.reset_cause(), str(machine.reset_cause()))
BOOT_ID = random.getrandbits(16)


def boot():
    global eyes
    loaded = load_config()
    log("부팅 ({}), 설정 {}".format(reset_cause, "불러옴" if loaded else "기본값"))

    make_servos()
    for s in servos:
        s.write()

    spi = SPI(0, baudrate=1_000_000, polarity=0, phase=0, sck=Pin(EYE_SCK_PIN), mosi=Pin(EYE_MOSI_PIN))
    eyes = Eyes(MAX7219Chain(spi, Pin(EYE_CS_PIN, Pin.OUT), 2))
    eyes.fill(0xFF)          # MAX7219는 응답이 없으므로 전부 켜서 눈으로 확인
    time.sleep_ms(500)
    eyes.fill(0x00)
    eyes.render_base()

    led.value(0)
    print("준비 완료. 컴퓨터에서 panel.html로 연결하거나, 여기서 h 입력")


async def main():
    asyncio.create_task(control_loop())
    while True:
        await asyncio.sleep(3600)


boot()
try:
    asyncio.run(main())
finally:
    for s in servos:
        s.off()
    asyncio.new_event_loop()
