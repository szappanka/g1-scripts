"""
G1 mozgás-utánzás -- webkamerás MediaPipe Pose alapján a robot karja(i) ÉS dereka
folyamatosan követik a felhasználó felsőtest-mozgását, az "arm_sdk" (rt/arm_sdk)
alacsony szintű felsőtest-felülírással (ld. g1_arm7_sdk_dds_example.py /
g1_handshake_grab_demo.py).

FONTOS HATÁRVONAL -- ez NEM egész testes (láb/egyensúly) utánzás: a lábak és az
egyensúly VÉGIG a robot saját beépített vezérlőjén maradnak (nem nyúlunk hozzájuk),
ezért a demó emiatt nem dőlhet el. Ha a felsőtest (kar+derék) mozgása miatt
elmozdul a robot súlypontja, azt a beépített egyensúly-FSM a lábakkal kompenzálja,
ugyanúgy, mintha valaki csak a felsőtestét mozgatná állás közben -- valódi
guggolás/lépés/súlyáthelyezés-utánzáshoz a lábízületeket kellene átvenni, ami
saját egyensúly-szabályzót igényelne, és NEM ennek a szkriptnek a feladata.

Karonként 3 szabadsági fokot követ: ShoulderPitch (előre-hátra emelés), ShoulderRoll
(oldalra emelés), Elbow (könyökhajlítás). A csukló és a ShoulderYaw a kiinduló
pózban rögzítve marad. A derék 3 szabadsági foka (WaistYaw/Roll/Pitch) alapból BE
van kapcsolva (--no-waist-szal kikapcsolhatod), és a torso előre-hátra dőlését,
oldalra dőlését és elfordulását követi.

DEADZONE (nincs külön kalibrációs lépés): egy relaxált álló ember könyöke/válla a
nyers geometria szerint SOSEM pontosan (0,0,0) -- a --deadzone (alap 0.12 rad) az
ennél kisebb szögeket egyszerűen 0-nak veszi, folytonosan afölött. Így nyugodt
álláskor semmi sem mozdul, anélkül hogy neked bármilyen "alaphelyzetet" fel kéne
venned indításkor.

FONTOS -- ELSŐ FUTTATÁS ELŐTT OLVASD EL:
- A váll-előjelek (--pitch-sign, --roll-sign-left/right), a derék-előjelek
  (--waist-*-sign) és a könyök-előjel (--elbow-sign) ROBOT-SPECIFIKUS kalibrációs
  becslések. A könyök előjele ezen a konkrét robotnál már ellenőrzött (negatív =
  helyes hajlítás, pozitív = veszélyes túlfeszítés -- ld. g1_handshake_grab_demo.py).
  A váll-előjelek a hivatalos g1_arm7_sdk_dds_example.py alapján vett BECSLÉSEK, a
  derék-előjelek pedig teljesen BECSÜLTEK -- egyik sincs ezen a robotnál letesztelve.
- KÜLÖN FIGYELEM A DEREKRA: a G1JointIndex kommentjei szerint egyes G1 hardver-
  változatoknál (23dof/29dof, "rögzített derék") a WaistRoll/WaistPitch ízület
  FIZIKAILAG NINCS motorizálva, csak a WaistYaw (elfordulás). Ha ez igaz erre a
  robotra, a dőlés-parancsok valószínűleg hatástalanok lesznek (a szkript ettől
  még nem veszélyes, csak nem fog látszani rajta semmi) -- ELSŐ TESZTKOR figyeld
  meg külön-külön (--no-waist / kis --waist-scale-lel), hogy ténylegesen mozog-e a
  derék előre-hátra és oldalra, vagy csak elfordulni tud.
- MÉG KORÁBBI, ROBOTOS FUTTATÁS ELŐTT: a --freeze kapcsolóval külön tesztelhető,
  hogy maga az arm_sdk aktiválása (a kar/derék "befagyasztása" a jelenlegi pózban)
  nyugodtan történik-e -- kamera/pózkövetés ekkor el sem indul, garantáltan semmi
  sem fog mozogni ezután.
- ELSŐ FUTTATÁS (mozgáskövetéssel): használd a --dry-run kapcsolót (nem csatlakozik a robothoz, csak
  kiírja a célszögeket) -- ehhez add hozzá a --show-t is, hogy kameraablakban lásd
  a vázat + a végleges (előjel/skála/clamp utáni) ízület-parancsokat egyszerre,
  mielőtt bármi robothoz kapcsolódna. Ha az irányok jónak tűnnek, robotmal is
  indíts kis --scale/--waist-scale értékkel (pl. 0.2-0.3), és lassan, akadálymentes
  területen figyeld, jó irányba mozog-e minden ízület, mielőtt feljebb vennéd. Ha
  bármelyik ízület rossz irányba mozdul, állítsd meg (Ctrl+C) és fordíts az adott
  előjelen -- egyszerre csak egy paramétert változtass (ld. korábbi tapasztalat a
  kézfogás demónál).
- Az arm_sdk a TELJES felsőtestet átveszi -- ha a --no-waist van megadva, vagy csak
  az egyik kart mozgatod, a nem aktív kar/derék minden ciklusban a kiinduló pózban
  van tartva (kp/kd-vel), mint a kézfogás demóban.
- Ha a kamera egy ideig nem lát embert (--lost-retract-seconds), a kar(ok)/derék
  automatikusan, lassan (a --max-rate/--waist-max-rate szerint) visszaengednek a
  szkript indulási pózába, majd onnan folytatják, ha a látás visszatér.
- Ctrl+C: minden aktív ízület visszamegy a kiinduló pózba, majd az arm_sdk
  fokozatosan visszaadja az irányítást a magas szintű vezérlőnek.

Telepítés:
    pip install mediapipe opencv-python

Használat:
    python3 g1_pose_mimic.py --dry-run --show   # csak kamera+szamitas, robot nelkul, videoval
    python3 g1_pose_mimic.py --dry-run          # ua., csak terminal-kiiras (nincs kameraablak)
    python3 g1_pose_mimic.py enp2s0 --freeze    # robot, DE csak arm_sdk teszt, kovetes nelkul
    python3 g1_pose_mimic.py enp2s0 --arm both --scale 0.3 --waist-scale 0.3
    python3 g1_pose_mimic.py enp2s0 --arm right --scale 0.5 --mirror --no-waist
"""

import sys
import time
import math
import argparse
import threading
import traceback
import statistics

import cv2
import mediapipe as mp

from pose_arm_mapping import compute_pose_targets

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC


class PeriodicThread:
    """Platformfüggetlen csere a unitree_sdk2py RecurrentThread-jére (az Linux-only
    timerfd-t használ, macOS-en elszáll) -- sima threading + time.sleep, drift-korrekcióval."""

    def __init__(self, interval, target):
        self._interval = interval
        self._target = target
        self._quit = False
        self._thread = threading.Thread(target=self._run, daemon=True)

    def Start(self):
        self._thread.start()

    def stop(self):
        self._quit = True

    def _run(self):
        next_t = time.time()
        while not self._quit:
            try:
                self._target()
            except Exception:
                traceback.print_exc()
            next_t += self._interval
            sleep_time = next_t - time.time()
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                next_t = time.time()


class G1JointIndex:
    WaistYaw = 12
    WaistRoll = 13
    WaistPitch = 14
    LeftShoulderPitch = 15
    LeftShoulderRoll = 16
    LeftShoulderYaw = 17
    LeftElbow = 18
    LeftWristRoll = 19
    LeftWristPitch = 20
    LeftWristYaw = 21
    RightShoulderPitch = 22
    RightShoulderRoll = 23
    RightShoulderYaw = 24
    RightElbow = 25
    RightWristRoll = 26
    RightWristPitch = 27
    RightWristYaw = 28
    kNotUsedJoint = 29  # ezen keresztül kapcsoljuk be/ki az arm_sdk súlyt


ARM_JOINT_NAMES = ["ShoulderPitch", "ShoulderRoll", "ShoulderYaw", "Elbow", "WristRoll", "WristPitch", "WristYaw"]


class PoseVisionThread:
    """Kamera + MediaPipe Pose külön szálon -- nem szabad blokkolnia az 50Hz-es
    kar-vezérlő ciklust (egy pose-becslés simán 10-30ms is lehet CPU-n)."""

    # Induláskor (kamera-fókusz/expozíció meg nem állt be, MediaPipe meg nem "allt
    # ra" a kovetesre) az elso 1-2 kockat gyakran zajosabb/pontatlanabb -- ha ezt
    # azonnal, simitas nelkul elhinnenk, egy egyetlen rossz kocka (pl. a konyok
    # depth-becslese elcsuszik) rogton egy latvanyos, "veletlenszeru" konyokhajlast
    # kuldene a robotnak. Ehelyett osszegyujtunk WARMUP_SAMPLES darab nyers mintat,
    # es a MEDIANJUKKAL indul a simitas -- ez mar egy kiugro rossz kockat kiszur.
    WARMUP_SAMPLES = 5

    def __init__(self, camera_index, smoothing, capture_debug=False):
        self.camera_index = camera_index
        self.alpha = smoothing
        self.capture_debug = capture_debug
        self._lock = threading.Lock()
        self._smoothed = {"left": None, "right": None, "torso": None}
        self._last_seen = {"left": None, "right": None, "torso": None}
        self._warmup_buffer = {"left": [], "right": [], "torso": []}
        self._debug_frame = None
        self._debug_landmarks = None
        self._quit = False
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._thread.start()

    def stop(self):
        self._quit = True
        self._thread.join(timeout=2.0)

    def get(self, side):
        """Visszaadja az (angles_dict_vagy_None, kor_masodpercben_vagy_None) part."""
        with self._lock:
            angles = self._smoothed[side]
            last_seen = self._last_seen[side]
        if angles is None or last_seen is None:
            return None, None
        return dict(angles), time.time() - last_seen

    def get_debug_frame(self):
        """Csak --show módban hasznos: az utolsó nyers kamerakép + a rárajzolható
        MediaPipe landmarkok (2D, kép-tér), vagy (None, None), ha meg nincs kép."""
        with self._lock:
            if self._debug_frame is None:
                return None, None
            return self._debug_frame.copy(), self._debug_landmarks

    def _run(self):
        cap = cv2.VideoCapture(self.camera_index)
        if not cap.isOpened():
            print(f"HIBA: nem sikerult megnyitni a(z) {self.camera_index} kamerat.")
            return

        mp_pose = mp.solutions.pose
        with mp_pose.Pose(model_complexity=1, min_detection_confidence=0.5, min_tracking_confidence=0.5) as pose:
            while not self._quit:
                ok, frame = cap.read()
                if not ok:
                    time.sleep(0.05)
                    continue

                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                results = pose.process(rgb)

                if self.capture_debug:
                    with self._lock:
                        self._debug_frame = frame
                        self._debug_landmarks = results.pose_landmarks

                if not results.pose_world_landmarks:
                    continue

                raw = compute_pose_targets(results.pose_world_landmarks.landmark)
                now = time.time()
                with self._lock:
                    for side in ("left", "right", "torso"):
                        if raw[side] is None:
                            continue
                        if self._smoothed[side] is None:
                            buf = self._warmup_buffer[side]
                            buf.append(raw[side])
                            if len(buf) < self.WARMUP_SAMPLES:
                                continue
                            self._smoothed[side] = {k: statistics.median(s[k] for s in buf) for k in raw[side]}
                        else:
                            for k, v in raw[side].items():
                                self._smoothed[side][k] = self.alpha * v + (1.0 - self.alpha) * self._smoothed[side][k]
                        self._last_seen[side] = now

        cap.release()


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def apply_deadzone(angles, deadzone):
    """Egy relaxalt allo ember konyoke/valla SOHA nem pontosan 0 a nyers geometria
    szerint (termeszetes enyhe hajlas, elore-csuszas) -- ezert a --deadzone alatti
    ertekeket 0-ra veszi, afolott pedig folytonosan (ugras nelkul) indul onnan.
    Igy nincs szukseg kulon kalibracios lepesre: nyugodt allasnal semmi sem mozog.

    A `deadzone` vagy egy szam (minden kulcsra ugyanaz), vagy egy {kulcs: szam}
    dict (kulcsonkent mas) -- utobbi kell a konyoknek, aminek NAGYOBB a
    rendszeres torzitasa, mint a vall pitch/roll-janak (ket izulet-szegmens
    (felkar+alkar) melyseg-becslesenek hibaja adodik ossze egy szamba, a
    vall-szogeknel csak egy szegmense)."""
    def dz(v, threshold):
        if v > threshold:
            return v - threshold
        if v < -threshold:
            return v + threshold
        return 0.0
    return {k: dz(v, deadzone.get(k, 0.0) if isinstance(deadzone, dict) else deadzone) for k, v in angles.items()}


def arm_deadzones(args):
    return {"pitch": args.deadzone, "roll": args.deadzone, "elbow_bend": args.elbow_deadzone}


def compute_joint_commands(side, angles, args):
    """Ember-szog -> vegleges (elojel+skala+clamp utani) izulet-cel, PONTOSAN
    ugyanaz a kepletet hasznalja, mint amit a robot majd megkapna -- a
    dry-run/--show ezert 1:1-ben a valos viselkedest mutatja."""
    pitch_cmd = clamp(args.pitch_sign * args.scale * angles["pitch"], -args.pitch_limit, args.pitch_limit)
    roll_sign = args.roll_sign_left if side == "left" else args.roll_sign_right
    roll_cmd = clamp(roll_sign * args.scale * angles["roll"], -args.roll_limit, args.roll_limit)
    elbow_cmd = clamp(args.elbow_sign * args.scale * angles["elbow_bend"], args.elbow_min, args.elbow_max)
    return {"ShoulderPitch": pitch_cmd, "ShoulderRoll": roll_cmd, "Elbow": elbow_cmd}


def compute_waist_commands(angles, args):
    """Ua., mint compute_joint_commands, csak a derékra -- lasd ArmMimicController
    fenti megjegyzeset a keplet 1:1 megegyezeserol a robotnak kuldott parancesal."""
    yaw_cmd = clamp(args.waist_yaw_sign * args.waist_scale * angles["yaw"], -args.waist_yaw_limit, args.waist_yaw_limit)
    roll_cmd = clamp(args.waist_roll_sign * args.waist_scale * angles["roll"], -args.waist_roll_limit, args.waist_roll_limit)
    pitch_cmd = clamp(args.waist_pitch_sign * args.waist_scale * angles["pitch"], -args.waist_pitch_limit, args.waist_pitch_limit)
    return {"WaistYaw": yaw_cmd, "WaistRoll": roll_cmd, "WaistPitch": pitch_cmd}


class ArmMimicController:
    """Egy karhoz tartozó cél-szög számítás + slew-rate limitálás.
    Nincs kulon 'ramp-in' fazis -- a slew-rate limiter magatol lassan indul,
    mert a prev_cmd a szkript inditasi pozajabol indul."""

    def __init__(self, joint_ids, start_q, pitch_sign, roll_sign, elbow_sign,
                 scale, pitch_limit, roll_limit, elbow_min, elbow_max, max_rate, control_dt):
        # joint_ids sorrend: [ShoulderPitch, ShoulderRoll, ShoulderYaw, Elbow, WristRoll, WristPitch, WristYaw]
        self.pitch_id, self.roll_id, self.yaw_id, self.elbow_id = joint_ids[0], joint_ids[1], joint_ids[2], joint_ids[3]
        self.wrist_ids = joint_ids[4:]
        self.start_q = start_q
        self.pitch_sign = pitch_sign
        self.roll_sign = roll_sign
        self.elbow_sign = elbow_sign
        self.scale = scale
        self.pitch_limit = pitch_limit
        self.roll_limit = roll_limit
        self.elbow_min = elbow_min
        self.elbow_max = elbow_max
        self.max_step = max_rate * control_dt

        self.prev_pitch = start_q[self.pitch_id]
        self.prev_roll = start_q[self.roll_id]
        self.prev_elbow = start_q[self.elbow_id]

    def _slew(self, prev, target):
        return prev + clamp(target - prev, -self.max_step, self.max_step)

    def tick(self, human_angles):
        """human_angles: dict {"pitch","roll","elbow_bend"} vagy None (ilyenkor a
        kiindulasi pozahoz all vissza -- lassan, a slew-rate miatt)."""
        if human_angles is None:
            target_pitch = self.start_q[self.pitch_id]
            target_roll = self.start_q[self.roll_id]
            target_elbow = self.start_q[self.elbow_id]
        else:
            target_pitch = clamp(self.pitch_sign * self.scale * human_angles["pitch"],
                                  -self.pitch_limit, self.pitch_limit)
            target_roll = clamp(self.roll_sign * self.scale * human_angles["roll"],
                                 -self.roll_limit, self.roll_limit)
            target_elbow = clamp(self.elbow_sign * self.scale * human_angles["elbow_bend"],
                                  self.elbow_min, self.elbow_max)

        self.prev_pitch = self._slew(self.prev_pitch, target_pitch)
        self.prev_roll = self._slew(self.prev_roll, target_roll)
        self.prev_elbow = self._slew(self.prev_elbow, target_elbow)

        out = {
            self.pitch_id: self.prev_pitch,
            self.roll_id: self.prev_roll,
            self.yaw_id: self.start_q[self.yaw_id],
            self.elbow_id: self.prev_elbow,
        }
        for idx in self.wrist_ids:
            out[idx] = self.start_q[idx]
        return out


class WaistMimicController:
    """Ua. a felepites, mint ArmMimicController-nel (slew-rate limit, nincs kulon
    ramp-in), csak a 3 derek-izuletre (Yaw/Roll/Pitch)."""

    def __init__(self, yaw_id, roll_id, pitch_id, start_q, yaw_sign, roll_sign, pitch_sign,
                 scale, yaw_limit, roll_limit, pitch_limit, max_rate, control_dt):
        self.yaw_id, self.roll_id, self.pitch_id = yaw_id, roll_id, pitch_id
        self.start_q = start_q
        self.yaw_sign, self.roll_sign, self.pitch_sign = yaw_sign, roll_sign, pitch_sign
        self.scale = scale
        self.yaw_limit, self.roll_limit, self.pitch_limit = yaw_limit, roll_limit, pitch_limit
        self.max_step = max_rate * control_dt

        self.prev_yaw = start_q[yaw_id]
        self.prev_roll = start_q[roll_id]
        self.prev_pitch = start_q[pitch_id]

    def _slew(self, prev, target):
        return prev + clamp(target - prev, -self.max_step, self.max_step)

    def tick(self, human_angles):
        """human_angles: dict {"yaw","roll","pitch"} vagy None (ilyenkor a
        kiindulasi pozahoz all vissza -- lassan, a slew-rate miatt)."""
        if human_angles is None:
            target_yaw = self.start_q[self.yaw_id]
            target_roll = self.start_q[self.roll_id]
            target_pitch = self.start_q[self.pitch_id]
        else:
            target_yaw = clamp(self.yaw_sign * self.scale * human_angles["yaw"], -self.yaw_limit, self.yaw_limit)
            target_roll = clamp(self.roll_sign * self.scale * human_angles["roll"], -self.roll_limit, self.roll_limit)
            target_pitch = clamp(self.pitch_sign * self.scale * human_angles["pitch"], -self.pitch_limit, self.pitch_limit)

        self.prev_yaw = self._slew(self.prev_yaw, target_yaw)
        self.prev_roll = self._slew(self.prev_roll, target_roll)
        self.prev_pitch = self._slew(self.prev_pitch, target_pitch)

        return {self.yaw_id: self.prev_yaw, self.roll_id: self.prev_roll, self.pitch_id: self.prev_pitch}


class PoseMimicDemo:
    RETRACT_SECONDS = 2.0
    RELEASE_SECONDS = 1.0

    def __init__(self, args):
        self.args = args
        self.arms = ["left", "right"] if args.arm == "both" else [args.arm]
        self.waist_enabled = not args.no_waist
        self.waist_joint_ids = [G1JointIndex.WaistYaw, G1JointIndex.WaistRoll, G1JointIndex.WaistPitch]

        self.active_joint_ids = []
        for side in self.arms:
            prefix = "Left" if side == "left" else "Right"
            self.active_joint_ids += [getattr(G1JointIndex, f"{prefix}{name}") for name in ARM_JOINT_NAMES]
        if self.waist_enabled:
            self.active_joint_ids += self.waist_joint_ids

        other_sides = [s for s in ("left", "right") if s not in self.arms]
        self.passive_joint_ids = [] if self.waist_enabled else list(self.waist_joint_ids)
        for side in other_sides:
            prefix = "Left" if side == "left" else "Right"
            self.passive_joint_ids += [getattr(G1JointIndex, f"{prefix}{name}") for name in ARM_JOINT_NAMES]

        self.all_joint_ids = self.active_joint_ids + self.passive_joint_ids

        self.control_dt = 0.02
        self.kp = args.kp
        self.kd = args.kd

        self.crc = CRC()
        self.low_cmd = unitree_hg_msg_dds__LowCmd_()
        self.low_state = None
        self.start_q = None
        self.controllers = {}  # side -> ArmMimicController
        self.waist_controller = None

        self.vision = PoseVisionThread(args.camera, args.smoothing)

        self.stage = "WARMUP"
        self.stage_t0 = None
        self.retract_from_q = None
        self.done = False
        self.run_t0 = None
        self._last_print = 0.0

    def init_channels(self):
        self.publisher = ChannelPublisher("rt/arm_sdk", LowCmd_)
        self.publisher.Init()
        self.subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.subscriber.Init(self.on_lowstate, 10)

    def on_lowstate(self, msg: LowState_):
        self.low_state = msg

    def wait_for_state(self):
        while self.low_state is None:
            time.sleep(0.05)

    def write_joint(self, idx, q):
        self.low_cmd.motor_cmd[idx].tau = 0.0
        self.low_cmd.motor_cmd[idx].q = float(q)
        self.low_cmd.motor_cmd[idx].dq = 0.0
        self.low_cmd.motor_cmd[idx].kp = self.kp
        self.low_cmd.motor_cmd[idx].kd = self.kd

    def side_source(self, side):
        """A kamerán észlelt melyik oldala legyen a robot melyik karja -- --mirror
        felcseréli, mert ez tapasztalatilag változó, hogy melyik érzi 'természetesnek'."""
        if self.args.mirror:
            return "right" if side == "left" else "left"
        return side

    def human_angles_for(self, side):
        vision_side = self.side_source(side)
        angles, age = self.vision.get(vision_side)
        if angles is None:
            return None
        if age > self.args.lost_retract_seconds:
            return None
        return apply_deadzone(angles, arm_deadzones(self.args))

    def human_torso_angles(self):
        angles, age = self.vision.get("torso")
        if angles is None:
            return None
        if age > self.args.lost_retract_seconds:
            return None
        return apply_deadzone(angles, self.args.deadzone)

    def enter_stage(self, stage):
        self.stage = stage
        self.stage_t0 = time.time()
        if stage == "RETRACT":
            self.retract_from_q = {idx: self.low_state.motor_state[idx].q for idx in self.active_joint_ids}
        print(f"[fazis] {stage}")

    def force_release(self):
        """Azonnal elengedi az arm_sdk-t (kihagyva a sima RETRACT/RELEASE rampat) --
        arra az esetre, ha a felhasznalo egy MASODIK Ctrl+C-vel megszakitja a sima
        visszavonulast: igy legalabb nem marad a kar felig arm_sdk alatt fagyva."""
        self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q = 0.0
        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        for _ in range(5):
            self.publisher.Write(self.low_cmd)
            time.sleep(0.02)
        self.stage = "DONE"
        self.done = True

    def control_tick(self):
        if self.low_state is None:
            return

        if self.stage == "WARMUP":
            self.start_q = {idx: self.low_state.motor_state[idx].q for idx in self.all_joint_ids}
            self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q = 1.0  # arm_sdk bekapcsolva
            for side in self.arms:
                prefix = "Left" if side == "left" else "Right"
                joint_ids = [getattr(G1JointIndex, f"{prefix}{name}") for name in ARM_JOINT_NAMES]
                self.controllers[side] = ArmMimicController(
                    joint_ids, self.start_q,
                    pitch_sign=self.args.pitch_sign,
                    roll_sign=(self.args.roll_sign_left if side == "left" else self.args.roll_sign_right),
                    elbow_sign=self.args.elbow_sign,
                    scale=self.args.scale,
                    pitch_limit=self.args.pitch_limit,
                    roll_limit=self.args.roll_limit,
                    elbow_min=self.args.elbow_min,
                    elbow_max=self.args.elbow_max,
                    max_rate=self.args.max_rate,
                    control_dt=self.control_dt,
                )
            if self.waist_enabled:
                self.waist_controller = WaistMimicController(
                    *self.waist_joint_ids, self.start_q,
                    yaw_sign=self.args.waist_yaw_sign,
                    roll_sign=self.args.waist_roll_sign,
                    pitch_sign=self.args.waist_pitch_sign,
                    scale=self.args.waist_scale,
                    yaw_limit=self.args.waist_yaw_limit,
                    roll_limit=self.args.waist_roll_limit,
                    pitch_limit=self.args.waist_pitch_limit,
                    max_rate=self.args.waist_max_rate,
                    control_dt=self.control_dt,
                )
            self.run_t0 = time.time()
            self.enter_stage("RUN")

        elapsed = time.time() - self.stage_t0

        for idx in self.all_joint_ids:
            self.write_joint(idx, self.start_q[idx])

        if self.stage == "RUN":
            if self.args.freeze:
                # --freeze: csak a fenti alap-iras (start_q tartas) fut -- szandekosan
                # nincs se vision, se controller.tick() hivas, hogy tisztan az
                # "arm_sdk bekapcsol + tart" viselkedes tesztelheto legyen, mozgaskoveto
                # logika nelkul.
                if self.args.duration and (time.time() - self.run_t0) >= self.args.duration:
                    print(f"Lejart a --duration ({self.args.duration:.0f} mp), visszavonulas...")
                    self.enter_stage("RETRACT")
                self.low_cmd.crc = self.crc.Crc(self.low_cmd)
                self.publisher.Write(self.low_cmd)
                return

            for side in self.arms:
                human = self.human_angles_for(side)
                cmd = self.controllers[side].tick(human)
                for idx, q in cmd.items():
                    self.write_joint(idx, q)

            if self.waist_enabled:
                torso_human = self.human_torso_angles()
                waist_cmd = self.waist_controller.tick(torso_human)
                for idx, q in waist_cmd.items():
                    self.write_joint(idx, q)

            if self.args.duration and (time.time() - self.run_t0) >= self.args.duration:
                print(f"Lejart a --duration ({self.args.duration:.0f} mp), visszavonulas...")
                self.enter_stage("RETRACT")

            now = time.time()
            if now - self._last_print >= 1.0:
                self._last_print = now
                parts = []
                for side in self.arms:
                    angles, age = self.vision.get(self.side_source(side))
                    if angles is None:
                        parts.append(f"{side}=nincs jel")
                    else:
                        parts.append(f"{side}: pitch={angles['pitch']:+.2f} roll={angles['roll']:+.2f} "
                                     f"elbow={angles['elbow_bend']:+.2f} (kor={age:.1f}s)")
                if self.waist_enabled:
                    torso, torso_age = self.vision.get("torso")
                    if torso is None:
                        parts.append("torso=nincs jel")
                    else:
                        parts.append(f"torso: pitch={torso['pitch']:+.2f} roll={torso['roll']:+.2f} "
                                     f"yaw={torso['yaw']:+.2f} (kor={torso_age:.1f}s)")
                print("  " + " | ".join(parts))

        elif self.stage == "RETRACT":
            ratio = min(elapsed / self.RETRACT_SECONDS, 1.0)
            for idx in self.active_joint_ids:
                q0 = self.retract_from_q[idx]
                target = self.start_q[idx]
                self.write_joint(idx, (1.0 - ratio) * q0 + ratio * target)
            if ratio >= 1.0:
                self.enter_stage("RELEASE")

        elif self.stage == "RELEASE":
            weight = max(1.0 - elapsed / self.RELEASE_SECONDS, 0.0)
            self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q = weight
            if weight <= 0.0:
                self.stage = "DONE"
                self.done = True

        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.publisher.Write(self.low_cmd)


def _dry_run_text(args, vision, arms):
    print("Dry-run -- nincs robot kapcsolat. Ctrl+C a kilepeshez.")
    while True:
        time.sleep(0.5)
        parts = []
        for side in arms:
            vision_side = ("right" if side == "left" else "left") if args.mirror else side
            angles, age = vision.get(vision_side)
            if angles is None:
                parts.append(f"{side}=nincs jel")
                continue
            cmd = compute_joint_commands(side, apply_deadzone(angles, arm_deadzones(args)), args)
            parts.append(f"{side}: ShoulderPitch={cmd['ShoulderPitch']:+.2f} ShoulderRoll={cmd['ShoulderRoll']:+.2f} "
                         f"Elbow={cmd['Elbow']:+.2f} (kor={age:.1f}s)")
        if not args.no_waist:
            torso, age = vision.get("torso")
            if torso is None:
                parts.append("torso=nincs jel")
            else:
                cmd = compute_waist_commands(apply_deadzone(torso, args.deadzone), args)
                parts.append(f"torso: WaistYaw={cmd['WaistYaw']:+.2f} WaistRoll={cmd['WaistRoll']:+.2f} "
                             f"WaistPitch={cmd['WaistPitch']:+.2f} (kor={age:.1f}s)")
        print(" | ".join(parts))


def _dry_run_show(args, vision, arms):
    """--dry-run --show: kameraablak a vazzal + a VEGLEGES (elojel/skala/clamp
    utani) izulet-parancsokkal kiirva -- igy latvanyból ellenorizheto, hogy pl.
    a --mirror vagy egy elojel jo iranyba all-e, meg mielott a robot csatlakozna."""
    mp_pose = mp.solutions.pose
    mp_drawing = mp.solutions.drawing_utils
    print("Dry-run + video -- nincs robot kapcsolat. 'q' vagy Ctrl+C a kilepeshez.")
    print("Az ertekek masodpercenkent a terminalba is kiirodnak -- nem kell egyszerre "
          "neznie a kepernyot es csinalni a mozdulatot, utolag is visszaolvashato.")
    t0 = time.time()
    last_print = 0.0
    try:
        while True:
            frame, landmarks = vision.get_debug_frame()
            if frame is None:
                if cv2.waitKey(30) & 0xFF == ord("q"):
                    break
                continue

            if landmarks:
                mp_drawing.draw_landmarks(frame, landmarks, mp_pose.POSE_CONNECTIONS)

            texts = []
            for side in arms:
                vision_side = ("right" if side == "left" else "left") if args.mirror else side
                angles, age = vision.get(vision_side)
                if angles is None:
                    texts.append(f"{side}: nincs jel")
                else:
                    cmd = compute_joint_commands(side, apply_deadzone(angles, arm_deadzones(args)), args)
                    texts.append(f"{side}: Pitch={cmd['ShoulderPitch']:+.2f} Roll={cmd['ShoulderRoll']:+.2f} "
                                 f"Elbow={cmd['Elbow']:+.2f} (kor={age:.1f}s)")

            if not args.no_waist:
                torso, age = vision.get("torso")
                if torso is None:
                    texts.append("torso: nincs jel")
                else:
                    cmd = compute_waist_commands(apply_deadzone(torso, args.deadzone), args)
                    texts.append(f"torso: Yaw={cmd['WaistYaw']:+.2f} Roll={cmd['WaistRoll']:+.2f} "
                                 f"Pitch={cmd['WaistPitch']:+.2f} (kor={age:.1f}s)")

            y = 30
            for i, text in enumerate(texts):
                color = (0, 255, 0) if i < len(arms) else (0, 200, 255)
                cv2.putText(frame, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                y += 25

            now = time.time()
            if now - last_print >= 1.0:
                last_print = now
                print(f"[{now - t0:5.1f}s] " + " | ".join(texts))

            cv2.imshow("G1 pose mimic dry-run (q = kilepes)", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        cv2.destroyAllWindows()


def _dry_run_guided(args, vision, arms):
    """Vezetett teszt: TE indítod a visszaszámlálást (SZÓKÖZ), miután már
    felkészültél (nincs fix várakozás) -- utána egy rövid visszaszámlálás, majd
    egy fix ideig tartó "TARTSD MOST!" mérési periódus, aminek végén a
    terminálba írja, mit mért PONTOSAN az alatt az idő alatt (átlag/min/max)."""
    COUNTDOWN_SECONDS = 3.0
    HOLD_SECONDS = 5.0
    mp_pose = mp.solutions.pose
    mp_drawing = mp.solutions.drawing_utils
    print("Vezetett teszt: allj be a kivant pozba (pl. mindket kar egyenesen kinyujtva), majd "
          "kattints a kamera-ablakra es nyomj SZOKOZT, ha keszen allsz -- addig nem indul semmi.")

    stage = "WAITING"
    stage_t0 = None
    samples = {"left": [], "right": [], "torso": []}
    try:
        while True:
            frame, landmarks = vision.get_debug_frame()
            if frame is None:
                if cv2.waitKey(30) & 0xFF == ord("q"):
                    return
                continue
            if landmarks:
                mp_drawing.draw_landmarks(frame, landmarks, mp_pose.POSE_CONNECTIONS)

            h = frame.shape[0]
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                return

            if stage == "WAITING":
                cv2.putText(frame, "Allj be pozba, majd nyomj SZOKOZT", (20, h // 2),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 200, 255), 3)
                if key == ord(" "):
                    stage, stage_t0 = "COUNTDOWN", time.time()

            elif stage == "COUNTDOWN":
                remaining = COUNTDOWN_SECONDS - (time.time() - stage_t0)
                if remaining <= 0:
                    stage, stage_t0 = "HOLD", time.time()
                    samples = {"left": [], "right": [], "torso": []}
                else:
                    cv2.putText(frame, f"Kesz allj! {remaining:0.0f}", (30, h // 2),
                                cv2.FONT_HERSHEY_SIMPLEX, 1.3, (0, 0, 255), 3)

            elif stage == "HOLD":
                remaining = HOLD_SECONDS - (time.time() - stage_t0)
                for side in arms:
                    vision_side = ("right" if side == "left" else "left") if args.mirror else side
                    angles, _age = vision.get(vision_side)
                    if angles is not None:
                        samples[side].append(angles)
                if not args.no_waist:
                    torso, _age = vision.get("torso")
                    if torso is not None:
                        samples["torso"].append(torso)
                if remaining <= 0:
                    break
                cv2.putText(frame, f"TARTSD MOST! {remaining:0.1f}", (30, h // 2),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.3, (0, 200, 0), 3)

            cv2.imshow("G1 pose mimic - vezetett teszt (q = kilepes)", frame)
    finally:
        cv2.destroyAllWindows()

    print(f"\n=== Eredmeny (a {HOLD_SECONDS:.0f} mp-es TARTSD MOST periodus alatt mert atlag/min/max) ===")
    for side in ["left", "right"] + ([] if args.no_waist else ["torso"]):
        s = samples[side]
        if not s:
            print(f"  {side}: nem volt lathato a meres alatt")
            continue
        for k in s[0]:
            vals = [x[k] for x in s]
            print(f"  {side}.{k}: atlag={statistics.mean(vals):+.3f}  min={min(vals):+.3f}  "
                  f"max={max(vals):+.3f}  (n={len(vals)})")


def run_dry(args):
    """--dry-run: robot nelkul, csak a szamitott celszogeket irja ki (vagy --show
    eseten meg is mutatja videon, --guided eseten idozitett meressel), hogy a
    latas + geometria + elojel/skala lanc tesztelheto legyen hardver nelkul."""
    arms = ["left", "right"] if args.arm == "both" else [args.arm]
    vision = PoseVisionThread(args.camera, args.smoothing, capture_debug=args.show or args.guided)
    vision.start()
    try:
        if args.guided:
            _dry_run_guided(args, vision, arms)
        elif args.show:
            _dry_run_show(args, vision, arms)
        else:
            _dry_run_text(args, vision, arms)
    except KeyboardInterrupt:
        pass
    finally:
        vision.stop()


def main():
    parser = argparse.ArgumentParser(description="G1 mozgas-utanzas -- webkamerás pose kovetes arm_sdk-val")
    parser.add_argument("net", nargs="?", default=None, help="halozati interfesz, pl. enp2s0 (--dry-run eseten nem kell)")
    parser.add_argument("--dry-run", action="store_true", help="nem csatlakozik a robothoz, csak kiirja a celszogeket")
    parser.add_argument("--show", action="store_true",
                         help="--dry-run-nal kameraablakot is nyit a vazzal + a vegleges izulet-parancsokkal kiirva")
    parser.add_argument("--guided", action="store_true",
                         help="--dry-run-nal vezetett teszt: visszaszamlalas a kepen, majd 5 mp-es 'TARTSD MOST!' "
                              "periodus, aminek vegen a terminalba irja a mert atlag/min/max erteket -- igy nem "
                              "kell egyszerre nezni a kepernyot es idozsíteni a pozt")
    parser.add_argument("--arm", choices=["left", "right", "both"], default="both")
    parser.add_argument("--freeze", action="store_true",
                         help="csak bekapcsolja az arm_sdk-t es tartja a kiindulo pozat -- kamera/pozekoves EL SEM "
                              "INDUL. Arra jo, hogy kulon lehessen tesztelni, hogy az arm_sdk aktivalasa maga "
                              "nyugodtan tortenik-e, mozgaskovetes nelkul.")
    parser.add_argument("--camera", type=int, default=0, help="kamera index (alap: 0)")
    parser.add_argument("--mirror", action="store_true",
                         help="felcsereli bal/jobb kart -- tesztelj mindket modot, amelyik termeszetesebb")
    parser.add_argument("--deadzone", type=float, default=0.12,
                         help="ennyi radian alatti nyers szoget 0-nak vesz a vall pitch/roll-jan -- igy a termeszetes, "
                              "sosem tokeletesen nyujtott allo poz nem tunik mar 'nyujtott kar'-nak, kulon "
                              "kalibracio nelkul (alap: 0.12 rad ~ 7 fok)")
    parser.add_argument("--elbow-deadzone", type=float, default=0.55,
                         help="ua., mint --deadzone, de kulon a konyoknek -- a konyok-szog ket izulet-szegmens "
                              "(felkar+alkar) melyseg-becsleset adja ossze, ezert nagyobb a rendszeres torzitasa: "
                              "nyujtott kar is behajlitottnak tunhet, ha ez tul kicsi (alap: 0.55 rad ~ 31 fok -- "
                              "vezetett teszttel mert ertek alapjan, novelheto tovabb, ha meg mindig latszik behajlas)")
    parser.add_argument("--scale", type=float, default=0.6,
                         help="mozgas-amplitudo szorzo -- ELSO teszthez hasznalj kicsit (0.2-0.3) (alap: 0.6)")
    parser.add_argument("--pitch-sign", type=float, default=-1.0, choices=[-1.0, 1.0],
                         help="ShoulderPitch elojel-becsles, NINCS letesztelve ezen a roboton (alap: -1.0)")
    parser.add_argument("--roll-sign-left", type=float, default=1.0, choices=[-1.0, 1.0],
                         help="bal ShoulderRoll elojel (alap: 1.0, a hivatalos arm7 pelda alapjan)")
    parser.add_argument("--roll-sign-right", type=float, default=-1.0, choices=[-1.0, 1.0],
                         help="jobb ShoulderRoll elojel (alap: -1.0, a hivatalos arm7 pelda alapjan)")
    parser.add_argument("--elbow-sign", type=float, default=-1.0, choices=[-1.0, 1.0],
                         help="Elbow elojel -- NE valtoztasd pozitivra, ezen a roboton veszelyes tulfeszitest okoz (alap: -1.0)")
    parser.add_argument("--pitch-limit", type=float, default=1.0, help="ShoulderPitch max |szog| radianban (alap: 1.0)")
    parser.add_argument("--roll-limit", type=float, default=1.4, help="ShoulderRoll max |szog| radianban (alap: 1.4)")
    parser.add_argument("--elbow-min", type=float, default=-1.3,
                         help="Elbow legnegativabb (legjobban behajlitott) megengedett szoge radianban (alap: -1.3)")
    parser.add_argument("--elbow-max", type=float, default=0.0,
                         help="Elbow legpozitivabb (legkevesbe behajlitott / legnyujtottabb) megengedett szoge "
                              "radianban -- VIGYAZAT: ezen a roboton pozitiv ertek (tesztelve: 1.9 rad) veszelyes "
                              "tulfeszitest okozott a kezfogas demonal, csak NAGYON OVATOSAN, apro lepesekben "
                              "(pl. 0.05-onkent) noveld, es azonnal allj le, ha a konyok tulfeszitettnek tunik "
                              "(alap: 0.0 -- ha ezzel sem er el teljesen nyujtott kart a robot, lehet, hogy a 0.0 "
                              "motorszog maga sem 'egyenes' ezen a konkret roboton)")
    parser.add_argument("--max-rate", type=float, default=1.5, help="max izuletsebesseg rad/s-ban (alap: 1.5)")
    parser.add_argument("--no-waist", action="store_true",
                         help="derek kikapcsolasa -- csak a kar(oka)t mozgatja, a derek a kiindulo pozaban rogzitve marad")
    parser.add_argument("--waist-scale", type=float, default=0.4,
                         help="derek mozgas-amplitudo szorzo -- ELSO teszthez hasznalj kicsit (0.15-0.2) (alap: 0.4)")
    parser.add_argument("--waist-yaw-sign", type=float, default=1.0, choices=[-1.0, 1.0],
                         help="WaistYaw (elfordulas) elojel-becsles, NINCS letesztelve (alap: 1.0)")
    parser.add_argument("--waist-roll-sign", type=float, default=1.0, choices=[-1.0, 1.0],
                         help="WaistRoll (oldalra doles) elojel-becsles, NINCS letesztelve (alap: 1.0)")
    parser.add_argument("--waist-pitch-sign", type=float, default=1.0, choices=[-1.0, 1.0],
                         help="WaistPitch (elore-hatra hajlas) elojel-becsles, NINCS letesztelve (alap: 1.0)")
    parser.add_argument("--waist-yaw-limit", type=float, default=0.6, help="WaistYaw max |szog| radianban (alap: 0.6)")
    parser.add_argument("--waist-roll-limit", type=float, default=0.3, help="WaistRoll max |szog| radianban (alap: 0.3)")
    parser.add_argument("--waist-pitch-limit", type=float, default=0.3, help="WaistPitch max |szog| radianban (alap: 0.3)")
    parser.add_argument("--waist-max-rate", type=float, default=1.0,
                         help="max derek-izuletsebesseg rad/s-ban -- ovatosabb, mint a kar alapertelmezese (alap: 1.0)")
    parser.add_argument("--smoothing", type=float, default=0.4,
                         help="EMA simitasi tenyezo a nyers pose-szogeken, 0..1, kisebb = simabb de lassabb (alap: 0.4)")
    parser.add_argument("--lost-retract-seconds", type=float, default=4.0,
                         help="ha ennel tovabb nincs latas, lassan visszaall a kiindulasi pozaba (alap: 4.0)")
    parser.add_argument("--duration", type=float, default=0.0, help="ha > 0, ennyi mp utan automatikusan visszavonul (alap: 0 = vegtelen)")
    parser.add_argument("--kp", type=float, default=60.0, help="pozicio-szabalyzo merevsege (alap: 60.0)")
    parser.add_argument("--kd", type=float, default=1.5, help="pozicio-szabalyzo csillapitasa (alap: 1.5)")
    args = parser.parse_args()

    if args.dry_run:
        run_dry(args)
        return

    if not args.net:
        parser.error("hianyzik a halozati interfesz (vagy hasznald a --dry-run kapcsolot)")

    if args.freeze:
        print("FIGYELEM (--freeze mod): csak az arm_sdk aktivalasat es a kiindulo poz tartasat teszteljuk --")
        print("kamera/pozekoves NEM indul el, a kar(ok) nem fognak mozogni. Ctrl+C-re visszaenged.")
    else:
        print("FIGYELEM: ez folyamatos, elo pozekovetes -- gyozodj meg rola, hogy nincs akadaly a kar(ok) korul.")
        print("Eloszor teszteld --dry-run-nal, majd kis --scale ertekkel, mielott feljebb vennéd.")
    input("Nyomj Entert a folytatashoz...")

    ChannelFactoryInitialize(0, args.net)

    demo = PoseMimicDemo(args)
    demo.init_channels()
    demo.wait_for_state()
    if not args.freeze:
        demo.vision.start()
        print("Varakozas a kamera/pose jelre...")

    thread = PeriodicThread(interval=demo.control_dt, target=demo.control_tick)
    thread.Start()

    try:
        while not demo.done:
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nMegszakitva, kar(ok) visszaengedese... (meg egy Ctrl+C az azonnali elengedeshez)")
        demo.enter_stage("RETRACT")
        try:
            while not demo.done:
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\nAzonnali elengedes...")
            demo.force_release()
    finally:
        thread.stop()
        demo.vision.stop()

    print("Demo kesz.")


if __name__ == "__main__":
    main()
