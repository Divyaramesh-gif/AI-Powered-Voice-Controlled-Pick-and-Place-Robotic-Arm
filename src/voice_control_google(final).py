#!/usr/bin/env python3
import os
import time
import shutil
from typing import Optional

# ==================== FIX FLAC PATH & PATCH SR ====================
FLAC_DIR = r"C:\Users\teja8\anaconda3\envs\robo\Library\bin"
os.environ["PATH"] = FLAC_DIR + os.pathsep + os.environ.get("PATH", "")

print("Python sees flac at:", shutil.which("flac"))

import speech_recognition as sr
from speech_recognition import audio as sraudio

def _force_flac_converter():
    path = shutil.which("flac")
    if path is None:
        raise OSError("FLAC not found even though PATH was set.")
    return path

sraudio.get_flac_converter = _force_flac_converter


# ========================= ROBOT IMPORT =========================
from xarm.wrapper import XArmAPI


# ========================= USER SETTINGS =========================
ARM_IP = "192.168.1.155"

STEP_X = 30.0
STEP_Y = 30.0
STEP_Z = 20.0

MOVE_SPEED = 60  # slightly lower for stability

MIN_Z = 0.0
MAX_Z = 350.0

# Optional soft clamp (avoid runaway)
MIN_X, MAX_X = -400.0, 400.0
MIN_Y, MAX_Y = -400.0, 400.0

LANGUAGE = "en-IN"

FIXED_ENERGY_THRESHOLD = 120.0

LISTEN_TIMEOUT = 4.0
PHRASE_TIME_LIMIT = 3.0


# ======================= SIMPLE LEVENSHTEIN ======================
def edit_distance(a: str, b: str) -> int:
    a = a.lower()
    b = b.lower()
    n, m = len(a), len(b)
    dp = [[0] * (m + 1) for _ in range(n + 1)]
    for i in range(n + 1):
        dp[i][0] = i
    for j in range(m + 1):
        dp[0][j] = j
    for i in range(1, n + 1):
        for j in range(1, m + 1):
            cost = 0 if a[i - 1] == b[j - 1] else 1
            dp[i][j] = min(
                dp[i - 1][j] + 1,
                dp[i][j - 1] + 1,
                dp[i - 1][j - 1] + cost
            )
    return dp[n][m]


def normalize_text(text: str) -> str:
    t = text.lower().strip()

    # handle spaced letters: "o p e n"
    if " " in t and all(len(x) == 1 for x in t.split()):
        t = "".join(t.split())

    # common ASR confusions
    replacements = {
        "forwards": "forward",
        "backwards": "back",
        "lift": "left",
        "rite": "right",
        "write": "right",
        "go home": "home",
        "open it": "open",
        "close it": "close",
        "shut": "close",
    }
    for k, v in replacements.items():
        if k in t:
            t = t.replace(k, v)

    return t


def fuzzy_match_command(text: str) -> Optional[str]:
    text = normalize_text(text)

    # ---- Strong phrase checks ----
    if any(kw in text for kw in ["go forward", "move forward", "forward"]):
        return "forward"
    if any(kw in text for kw in ["go back", "move back", "back", "backward"]):
        return "back"
    if any(kw in text for kw in ["move left", "go left", "left"]):
        return "left"
    if any(kw in text for kw in ["move right", "go right", "right"]):
        return "right"
    if any(kw in text for kw in ["go up", "move up", "up"]):
        return "up"
    if any(kw in text for kw in ["go down", "move down", "down"]):
        return "down"

    # vacuum naming per your request
    if any(kw in text for kw in ["close", "pick", "grab", "hold"]):
        return "close"
    if any(kw in text for kw in ["open", "release", "drop"]):
        return "open"

    if "home" in text:
        return "home"
    if any(kw in text for kw in ["stop", "quit", "exit"]):
        return "stop"

    # ---- Fuzzy token checks ----
    tokens = text.split()
    if not tokens:
        return None

    candidates = ["forward", "back", "left", "right", "up", "down", "open", "close", "home", "stop"]

    best_cmd = None
    best_dist = 999
    best_tok = None

    for tok in tokens:
        for cmd in candidates:
            if abs(len(tok) - len(cmd)) > 2:
                continue
            d = edit_distance(tok, cmd)
            if d < best_dist:
                best_dist = d
                best_cmd = cmd
                best_tok = tok

    if best_cmd:
        max_allowed = 1 if len(best_cmd) <= 4 else 2
        if best_dist <= max_allowed:
            print(f"[FUZZY] Interpreting '{best_tok}' as '{best_cmd}' (distance={best_dist})")
            return best_cmd

    return None


# ======================= ROBOT HELPERS ===========================
class ArmWrapper:
    def __init__(self, ip: str):
        self.ip = ip
        self.arm = XArmAPI(ip)
        self.connected = False

    def connect(self):
        # XArmAPI may auto-connect; still safe to call connect
        try:
            self.arm.connect()
        except Exception:
            pass
        self.connected = True

    def ensure_ready(self):
        """
        Auto-recover to reduce code=1 errors.
        """
        if not self.connected:
            return

        # Clean warn/error if APIs exist
        for fn in ["clean_warn", "clean_error"]:
            if hasattr(self.arm, fn):
                try:
                    getattr(self.arm, fn)()
                except Exception:
                    pass

        # Re-enable motion + set mode/state
        try:
            self.arm.motion_enable(True)
        except Exception:
            pass

        try:
            self.arm.set_mode(0)
        except Exception:
            pass

        try:
            self.arm.set_state(0)
        except Exception:
            pass

        time.sleep(0.05)

    def connect_and_home(self):
        self.connect()
        self.ensure_ready()
        time.sleep(0.2)
        try:
            self.arm.move_gohome(wait=True)
        except Exception as e:
            print("[WARN] gohome error:", e)
        print("Arm homed and ready for voice control.")

    def safe_get_position(self):
        if not self.connected:
            return -1, None
        self.ensure_ready()
        try:
            return self.arm.get_position(is_radian=False)
        except Exception:
            return -1, None

    def safe_set_position(self, **kwargs):
        if not self.connected:
            return -1

        # First attempt
        self.ensure_ready()
        code = self.arm.set_position(**kwargs)

        if code == 0:
            return 0

        # Retry once after recovery
        print("[WARN] set_position failed once, retrying after recovery...")
        self.ensure_ready()
        time.sleep(0.1)
        code = self.arm.set_position(**kwargs)
        return code

    def safe_vacuum(self, on: bool):
        if not self.connected:
            return -1
        self.ensure_ready()
        try:
            return self.arm.set_suction_cup(on, wait=True)
        except Exception:
            return -1

    def safe_gohome(self):
        if not self.connected:
            return
        self.ensure_ready()
        try:
            self.arm.move_gohome(wait=True)
        except Exception:
            pass

    def disconnect(self):
        try:
            if self.connected:
                self.arm.disconnect()
        except Exception:
            pass
        self.connected = False
        print("Arm disconnected. Bye!")


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def move_relative(aw: ArmWrapper, dx=0.0, dy=0.0, dz=0.0, speed=MOVE_SPEED):
    code, pos = aw.safe_get_position()
    if code != 0 or pos is None:
        print("[ERROR] get_position failed, code =", code)
        return

    x, y, z, roll, pitch, yaw = pos

    tx = clamp(x + dx, MIN_X, MAX_X)
    ty = clamp(y + dy, MIN_Y, MAX_Y)
    tz = clamp(z + dz, MIN_Z, MAX_Z)

    print(f"[MOVE] From ({x:.1f}, {y:.1f}, {z:.1f}) to ({tx:.1f}, {ty:.1f}, {tz:.1f})")

    code = aw.safe_set_position(
        x=tx, y=ty, z=tz,
        roll=roll, pitch=pitch, yaw=yaw,
        speed=speed, wait=True
    )

    if code != 0:
        print("[ERROR] set_position failed, code =", code)
        print("[HINT] If this continues, check UFactory Studio: Real mode + Enable Robot.")


# ======================= VACUUM OPEN/CLOSE ========================
def vacuum_close(aw: ArmWrapper):
    print("[VACUUM] CLOSE (ON)")
    code = aw.safe_vacuum(True)

    if code == 0:
        return

    if code == 41:
        print("[WARN] Vacuum not detected (code=41).")
        print("[HINT] In UFactory Studio → End Effector → select Vacuum Gripper and enable robot.")
        return

    print("[WARN] vacuum close failed, code =", code)


def vacuum_open(aw: ArmWrapper):
    print("[VACUUM] OPEN (OFF)")
    code = aw.safe_vacuum(False)

    if code == 0:
        return

    if code == 41:
        print("[WARN] Vacuum not detected (code=41).")
        print("[HINT] In UFactory Studio → End Effector → select Vacuum Gripper and enable robot.")
        return

    print("[WARN] vacuum open failed, code =", code)


# ===================== VOICE CONTROL LOOP ========================
def main():
    aw = ArmWrapper(ARM_IP)
    aw.connect_and_home()

    recognizer = sr.Recognizer()
    recognizer.dynamic_energy_threshold = False
    recognizer.energy_threshold = FIXED_ENERGY_THRESHOLD

    # Better short-command behavior
    recognizer.pause_threshold = 0.6
    recognizer.phrase_threshold = 0.2
    recognizer.non_speaking_duration = 0.2

    mic = sr.Microphone()
    print("Using microphone:", mic)

    print("\n=== VOICE CONTROL COMMANDS ===")
    print("  forward / go forward    -> +X")
    print("  back / go back          -> -X")
    print("  left / move left        -> +Y")
    print("  right / move right      -> -Y")
    print("  up / go up              -> +Z")
    print("  down / go down          -> -Z")
    print("  close                   -> Vacuum ON")
    print("  open                    -> Vacuum OFF")
    print("  home                    -> go home pose")
    print("  stop / quit / exit      -> stop voice control\n")

    print(f"Using FIXED energy_threshold: {recognizer.energy_threshold}")
    print("Now you can start speaking commands.\n")

    running = True
    while running:
        try:
            with mic as source:
                print("[LISTENING] Speak a command...")
                try:
                    audio = recognizer.listen(
                        source,
                        timeout=LISTEN_TIMEOUT,
                        phrase_time_limit=PHRASE_TIME_LIMIT
                    )
                except sr.WaitTimeoutError:
                    print("[ERROR] listening timed out while waiting for phrase to start")
                    continue

            try:
                raw = recognizer.recognize_google(audio, language=LANGUAGE)
            except sr.UnknownValueError:
                print("[ASR] Could not understand audio. (Try closer & speak clearly)")
                continue
            except sr.RequestError as e:
                print(f"[ASR] Google error: {e}")
                continue

            norm = normalize_text(raw)
            print(f"[HEARD RAW] '{raw}'")
            print(f"[HEARD NORM] '{norm}'")

            command = fuzzy_match_command(norm)
            if command is None:
                print("[CMD] No matching command found.")
                continue

            print(f"[CMD] Interpreted command: {command}")

            if command == "forward":
                move_relative(aw, dx=+STEP_X)
            elif command == "back":
                move_relative(aw, dx=-STEP_X)
            elif command == "left":
                move_relative(aw, dy=+STEP_Y)
            elif command == "right":
                move_relative(aw, dy=-STEP_Y)
            elif command == "up":
                move_relative(aw, dz=+STEP_Z)
            elif command == "down":
                move_relative(aw, dz=-STEP_Z)
            elif command == "close":
                vacuum_close(aw)
            elif command == "open":
                vacuum_open(aw)
            elif command == "home":
                print("[CMD] Going home...")
                aw.safe_gohome()
            elif command == "stop":
                print("[CMD] Stopping voice control.")
                running = False

        except KeyboardInterrupt:
            print("\n[CMD] Keyboard interrupt, exiting.")
            break
        except Exception as e:
            print("[ERROR] Unexpected error:", e)

    try:
        aw.safe_gohome()
    except Exception:
        pass

    aw.disconnect()


if __name__ == "__main__":
    main()
