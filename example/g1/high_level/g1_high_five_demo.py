"""
G1 pacsi (high five) demo -- ugyanaz az "arm_sdk" (rt/arm_sdk) alacsony szintű
kar-felülírás, mint a g1_handshake_grab_demo.py-ban, de itt a kar egy "pacsira
nyújtott" pózban vár, és nem tartós szorítást, hanem egy RÖVID ÜTÉST (tau_est
kiugrás) keres -- azaz belecsaptak-e a kézbe.

Két póz-opció (--style):
  top    -- "felső" pacsi: kar felemelve kb. váll/fej magasságba, mintha lentről
            csapnának bele
  bottom -- "alsó" pacsi: kar lent, tenyérrel felfelé, mintha fentről csapnának
            bele (ún. "low five")

FONTOS -- A LENTI PÓZOK NINCSENEK LETESZTELVE A ROBOTON:
  A g1_handshake_grab_demo.py REACH_POSE-ja már ki volt próbálva/hangolva ezen
  a robotnál. Ez a script viszont ÚJ pózokat vezet be (kar fel/le), amikhez
  még nem volt élő teszt -- tehát a HIGH_FIVE_POSE lenti értékei csak egy
  KIINDULÓ BECSLÉS, akárcsak eredetileg a kézfogásé volt.

  Ezért az első futtatáskor:
  1) Legyen valaki a kar mellett, akadálymentes területen.
  2) Nézd meg lassan (REACH fázis, 2 mp), milyen irányba mozdul a kar.
  3) Ha rossz irányba megy vagy túl nagyot mozdul, NE a fájlban írd át a
     HIGH_FIVE_POSE-t egyszerre több számjegyen -- inkább a parancssorból,
     EGYENKÉNT hangold a --shoulder-pitch / --shoulder-roll / --elbow
     kapcsolókkal, amíg jó nem lesz a póz. Csak ha megvan a jó érték, érdemes
     visszaírni alapértéknek a HIGH_FIVE_POSE-ba.
  4) Ctrl+C esetén a script megpróbálja RETRACT+RELEASE-elni a kart, de
     fizikai vészleállítót/damp módot is tarts készenlétben.

Menete:
  1) REACH    -- a kart a start-pózból a pacsi-pózba mozgatja (--style szerint)
  2) HOLD     -- tartja a pózt, méri a könyök/csukló tau_est nyugalmi
                 zajszintjét (automatikusan, nincs külön "állj nyugodtan"
                 lépés), majd figyeli a RÖVID kiugrást (= belecsaptak)
  3) REACT    -- ha megcsapták (vagy lejárt az időzítő és --react-on-timeout
                 van megadva): egy gyors "visszarezdülést" csinál a könyökkel,
                 mintha az ütés meglökte volna a kart. Ha meg van adva --say,
                 ekkor indul (külön szálon) a beszéd is.
  4) RETRACT  -- visszaviszi a kart pontosan a kiindulási pózba
  5) SETTLE   -- rövid, mozdulatlan tartás, hogy a visszahúzás lendülete leüljön
  6) RELEASE  -- fokozatosan visszaadja az irányítást a magas szintű vezérlőnek

A detektálás a kézfogásnál használt tau_est-alapú módszerre épül, de két
csatornát figyel: a becsült nyomatékot (tau_est) ÉS a mért szögsebességet
(dq) is, mind a négy figyelt ízületen (ShoulderPitch, ShoulderRoll, Elbow,
WristRoll) -- amelyik csatorna előbb kiugrik a nyugalmi szintjéhez képest,
az számít találatnak. A dq gyakran élesebb/gyorsabb jelet ad egy rövid
ütésre, mint a (becsült, simított) nyomaték. Az alapértékek is érzékenyebbre
vannak hangolva (rövidebb --min-hold), mert egy pacsi ütés sokkal rövidebb,
mint egy tartós szorítás.

Beszéd (--say): mint a kézfogás demóban, edge-tts-szel szintetizál (magyarul
is tud), a robot hangszóróján játssza le. Telepítés:
    pip install edge-tts miniaudio

Használat:
    python3 g1_high_five_demo.py <networkInterface>
    python3 g1_high_five_demo.py en6 --style top
    python3 g1_high_five_demo.py en6 --style bottom --arm left
    python3 g1_high_five_demo.py en6 --shoulder-pitch -0.9 --shoulder-roll 0.3
    python3 g1_high_five_demo.py en6 --say "Ez jó volt!"

Kilépés: Ctrl+C
"""

import os
import sys
import time
import math
import argparse
import statistics
import threading
import traceback
from collections import deque

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.g1.audio.g1_audio_client import AudioClient

# a hu_tts/wav modulok a ../audio mappában vannak, nem a sajátunkban
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "audio"))
from hu_tts import synthesize_pcm, DEFAULT_VOICE  # noqa: E402
from wav import play_pcm_stream  # noqa: E402


class PeriodicThread:
    """A unitree_sdk2py.utils.thread.RecurrentThread helyettesítője -- az eredeti Linux-only
    timerfd syscallt használ, ami macOS-en nem elérhető. Ez sima threading + time.sleep,
    drift-korrekcióval, platformfüggetlen."""

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
    WaistRoll = 13    # NOTE: INVALID g1 23dof/29dof-nál, rögzített derék esetén
    WaistPitch = 14   # NOTE: INVALID g1 23dof/29dof-nál, rögzített derék esetén
    LeftShoulderPitch = 15
    LeftShoulderRoll = 16
    LeftShoulderYaw = 17
    LeftElbow = 18
    LeftWristRoll = 19
    LeftWristPitch = 20   # NOTE: INVALID g1 23dof-nál
    LeftWristYaw = 21     # NOTE: INVALID g1 23dof-nál
    RightShoulderPitch = 22
    RightShoulderRoll = 23
    RightShoulderYaw = 24
    RightElbow = 25
    RightWristRoll = 26
    RightWristPitch = 27  # NOTE: INVALID g1 23dof-nál
    RightWristYaw = 28    # NOTE: INVALID g1 23dof-nál
    kNotUsedJoint = 29    # ezen keresztül kapcsoljuk be/ki az arm_sdk súlyt


ARM_JOINT_NAMES = ["ShoulderPitch", "ShoulderRoll", "ShoulderYaw", "Elbow", "WristRoll", "WristPitch", "WristYaw"]

# Pacsira nyújtott pózok.
# Sorrend: ShoulderPitch, ShoulderRoll, ShoulderYaw, Elbow, WristRoll, WristPitch, WristYaw  [rad]
# FONTOS: a kézfogás-demón mért tapasztalat szerint az Elbow NEGATÍV értéke
# hajlít helyesen -- pozitív érték veszélyesen HÁTRAFELÉ (túlfeszítve) hajlíthatja!
HIGH_FIVE_POSE = {
    "top": {
        # "left" MÉG NINCS LETESZTELVE -- csak becslés, amíg a g1_teach_pose.py-jal
        # (--arm left) be nem tanítod, ahogy a "right"-ot is.
        "left":  [-0.9,  0.35, 0.0, -0.2, 0.0, 0.0, 0.0],
        # "right" a g1_teach_pose.py-jal kézzel betanított, leellenőrzött póz.
        "right": [-1.080, -0.553, -0.341, -0.240, -0.699, -0.210, -0.191],
    },
    "bottom": {
        # "left" MÉG NINCS LETESZTELVE -- csak becslés, amíg a g1_teach_pose.py-jal
        # (--arm left) be nem tanítod, ahogy a "right"-ot is.
        "left":  [0.3,  0.25, 0.0, -0.5, 0.0, 0.0, 0.0],
        # "right" a g1_teach_pose.py-jal kézzel betanított póz. FIGYELEM: itt az
        # Elbow POZITÍV (+0.646) -- a fenti FONTOS-figyelmeztetés a kézfogás-pózból
        # származik, ahol a pozitív érték veszélyes hátrafeszítést jelentett. Ez a
        # tenyér-felfelé csuklóállás (nagy WristRoll/WristYaw) miatt más geometria,
        # de mivel kézzel, TEACH módban lett felvéve (nem a motor hajtotta oda),
        # NINCS még ellenőrizve, hogy a script által VEZÉRELT REACH-mozgás (pozíció-
        # szabályzással, kp=60) ugyanígy biztonságos-e -- első futtatáskor nézd meg
        # kiemelten lassan/óvatosan ezt a pózt.
        "right": [-0.345, 0.164, -0.080, 0.646, 1.426, 0.160, 0.539],
    },
}

# A "top" pózban a kar majdnem teljesen nyújtott (Elbow csak ~-0.24 rad), így egy
# tenyérre mért ütés nyomatéka nem az Elbow-nál/WristRoll-nál jelentkezik erősen,
# hanem a VÁLLNÁL -- ott a legnagyobb az emelőkar (kb. a teljes karhossz). Ezért a
# vállízületeket is figyeljük, nemcsak a könyököt/csuklót (ami a kézfogásnál, be-
# hajlított karnál elég volt). ShoulderYaw kihagyva: az a kar hossztengelye körüli
# forgás, egy lapos tenyérre mért ütés (ami a karra nagyjából merőlegesen hat) ezt
# várhatóan alig terheli -- ha a teszt mást mutat, simán felvehető ide is.
WATCH_JOINT_NAMES = ["ShoulderPitch", "ShoulderRoll", "Elbow", "WristRoll"]


class SlapDetector:
    """A megadott ízületek tau_est-jét (becsült nyomaték) ÉS dq-ját (mért szögsebesség)
    is gyűjti, mindkettőn külön nyugalmi alapszintet (átlag/szórás) számol, és eldönti,
    kiugrott-e valamelyik. Egy ütés a szögsebességben gyakran élesebb/gyorsabb kiugrást
    ad, mint a (becsült, simított) nyomatékban -- ezért mindkettőt figyeljük, amelyik
    előbb túllépi a saját küszöbét, az számít találatnak.

    A rövidség/tartósság közti különbséget a hívó fél a min_hold paraméterrel
    szabályozza (itt jóval rövidebb alapértékkel, mert egy pacsi ütés gyors)."""

    SIGNALS = ("tau", "dq")

    def __init__(self, joint_indices, history_len=1000):
        self.joint_indices = joint_indices
        self.history_len = history_len
        self.latest = {}    # (idx, signal) -> érték
        self.baseline = {}  # (idx, signal) -> (mean, std)
        self.history = {(idx, sig): deque(maxlen=history_len) for idx in joint_indices for sig in self.SIGNALS}

    def reset_history(self):
        self.history = {(idx, sig): deque(maxlen=self.history_len) for idx in self.joint_indices for sig in self.SIGNALS}

    def on_lowstate(self, msg: LowState_):
        for idx in self.joint_indices:
            motor = msg.motor_state[idx]
            for sig, value in (("tau", motor.tau_est), ("dq", motor.dq)):
                key = (idx, sig)
                self.latest[key] = value
                self.history[key].append(value)

    def calibrate(self):
        for key, samples in self.history.items():
            samples = list(samples)
            if len(samples) < 10:
                self.baseline[key] = (0.0, 0.02)
                continue
            mean = statistics.mean(samples)
            std = statistics.pstdev(samples) or 0.01
            self.baseline[key] = (mean, std)

    def channel_threshold(self, key, z_threshold, margin, max_threshold):
        """Az adott csatorna (ízület+jel) tényleges trigger-küszöbe -- felülről korlátozva,
        hogy egy zajos (pl. rezgő) csatorna kalibrált nagy szórása ne tegye gyakorlatilag
        érzékelhetetlenné."""
        _, std = self.baseline.get(key, (0.0, 0.0))
        return min(max(z_threshold * std, margin), max_threshold)

    def check_hit(self, z_threshold, tau_margin, tau_max, dq_margin, dq_max):
        bounds = {"tau": (tau_margin, tau_max), "dq": (dq_margin, dq_max)}
        best_key, best_dev = None, 0.0
        for key, value in self.latest.items():
            if key not in self.baseline:
                continue
            _, sig = key
            mean, _ = self.baseline[key]
            deviation = abs(value - mean)
            margin, max_threshold = bounds[sig]
            threshold = self.channel_threshold(key, z_threshold, margin, max_threshold)
            if deviation > threshold and deviation > best_dev:
                best_key, best_dev = key, deviation
        return best_key, best_dev

    def max_deviations(self):
        """A jelenlegi legnagyobb eltérés jelenként (tau_est Nm-ben, dq rad/s-ban) --
        diagnosztikához, küszöbtől függetlenül."""
        best = {sig: 0.0 for sig in self.SIGNALS}
        for key, value in self.latest.items():
            if key not in self.baseline:
                continue
            _, sig = key
            mean, _ = self.baseline[key]
            dev = abs(value - mean)
            if dev > best[sig]:
                best[sig] = dev
        return best


class HighFiveDemo:
    STAGE_REACH = "REACH"
    STAGE_HOLD = "HOLD"
    STAGE_REACT = "REACT"
    STAGE_RETRACT = "RETRACT"
    STAGE_SETTLE = "SETTLE"
    STAGE_RELEASE = "RELEASE"
    STAGE_DONE = "DONE"

    REACH_SECONDS = 2.0
    CALIB_SECONDS = 1.5
    RETRACT_SECONDS = 2.0
    # SETTLE: rövid, teljesen mozdulatlan tartás RETRACT után, teljes kp/kd-vel,
    # MIELŐTT a RELEASE elkezdené leengedni a súlyt -- ez hagyja leülni a
    # visszahúzás lendületét, mielőtt a magas szintű vezérlő visszakapja az
    # irányítást. Enélkül előfordulhat, hogy a kar RELEASE közben hirtelen
    # hátrarándul, ha a robot közben (pl. a kinyújtott kar miatt) kicsit
    # megdőlt, és a háttérben futó magas szintű vezérlőnek felgyűlt korrekciós
    # szándéka RELEASE-kor egyszerre "kiszabadul".
    SETTLE_SECONDS = 0.4
    RELEASE_SECONDS = 2.5

    def __init__(self, arm, style, pose_override, timeout, react_on_timeout,
                 z_threshold, min_margin, min_hold, hit_gap, max_threshold, dq_margin, dq_max,
                 recoil_amplitude, recoil_hz, recoil_cycles, kp, kd, say_text, say_voice):
        prefix = "Left" if arm == "left" else "Right"
        other_prefix = "Right" if arm == "left" else "Left"

        self.active_joint_ids = [getattr(G1JointIndex, f"{prefix}{name}") for name in ARM_JOINT_NAMES]
        # A másik kar + a derék -- az arm_sdk a TELJES felsőtestet átveszi a
        # magas szintű vezérlőtől, tehát ha ezeket nem tartjuk explicit
        # pozícióban (kp/kd-vel), tartóerő nélkül maradnak és a robot megdől.
        self.passive_joint_ids = (
            [getattr(G1JointIndex, f"{other_prefix}{name}") for name in ARM_JOINT_NAMES]
            + [G1JointIndex.WaistYaw, G1JointIndex.WaistRoll, G1JointIndex.WaistPitch]
        )
        self.all_joint_ids = self.active_joint_ids + self.passive_joint_ids

        self.reach_pose = list(HIGH_FIVE_POSE[style][arm])
        for i, override in pose_override.items():
            if override is not None:
                self.reach_pose[i] = override

        self._elbow_pos = ARM_JOINT_NAMES.index("Elbow")
        self._elbow_id = self.active_joint_ids[self._elbow_pos]

        watch_ids = [getattr(G1JointIndex, f"{prefix}{name}") for name in WATCH_JOINT_NAMES]
        self.detector = SlapDetector(watch_ids)

        self.timeout = timeout
        self.react_on_timeout = react_on_timeout
        self.z_threshold = z_threshold
        self.min_margin = min_margin
        self.min_hold = min_hold
        self.hit_gap = hit_gap
        self.max_threshold = max_threshold
        self.dq_margin = dq_margin
        self.dq_max = dq_max
        self.recoil_amplitude = recoil_amplitude
        self.recoil_hz = recoil_hz
        self.recoil_cycles = recoil_cycles

        self.control_dt = 0.02
        self.kp = kp
        self.kd = kd

        self.say_text = say_text
        self.say_voice = say_voice
        self.audio_client = None
        self.said = False

        self.crc = CRC()
        self.low_cmd = unitree_hg_msg_dds__LowCmd_()
        self.low_state = None

        self.stage = None
        self.stage_t0 = None
        self.start_q = None
        self.retract_from_q = None
        self.hold_since = None
        self.hit_since = None
        self.last_hit_time = None
        self.last_debug_print = None
        self.calibrated = False
        self.hit_detected = False
        self.last_deviation = 0.0
        self.last_hit_signal = None
        self.done = False

    def init_channels(self):
        self.publisher = ChannelPublisher("rt/arm_sdk", LowCmd_)
        self.publisher.Init()
        self.subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.subscriber.Init(self.on_lowstate, 10)

        if self.say_text:
            self.audio_client = AudioClient()
            self.audio_client.SetTimeout(10.0)
            self.audio_client.Init()

    def speak_async(self):
        """A megadott szöveget külön szálon mondja ki, hogy ne akassza meg a 20ms-es kar-vezérlő ciklust."""
        if not self.say_text or self.audio_client is None or self.said:
            return
        self.said = True

        def _run():
            try:
                pcm_bytes, _sample_rate = synthesize_pcm(self.say_text, voice=self.say_voice)
                play_pcm_stream(self.audio_client, list(pcm_bytes), "high_five_greeting")
                self.audio_client.PlayStop("high_five_greeting")
            except Exception:
                traceback.print_exc()

        threading.Thread(target=_run, daemon=True).start()

    def on_lowstate(self, msg: LowState_):
        self.low_state = msg
        self.detector.on_lowstate(msg)

    def wait_for_state(self):
        while self.low_state is None:
            time.sleep(0.05)

    def write_joint(self, idx, q):
        self.low_cmd.motor_cmd[idx].tau = 0.0
        self.low_cmd.motor_cmd[idx].q = float(q)
        self.low_cmd.motor_cmd[idx].dq = 0.0
        self.low_cmd.motor_cmd[idx].kp = self.kp
        self.low_cmd.motor_cmd[idx].kd = self.kd

    def enter_stage(self, stage):
        self.stage = stage
        self.stage_t0 = time.time()
        if stage == self.STAGE_HOLD:
            self.hold_since = self.stage_t0
            self.hit_since = None
            self.last_hit_time = None
            self.last_debug_print = None
            self.detector.reset_history()
            self.calibrated = False
        elif stage == self.STAGE_RETRACT:
            self.retract_from_q = {idx: self.low_state.motor_state[idx].q for idx in self.active_joint_ids}
        elif stage == self.STAGE_REACT:
            self.speak_async()
        print(f"[fázis] {stage}")

    def control_tick(self):
        if self.low_state is None:
            return

        if self.stage is None:
            self.start_q = {idx: self.low_state.motor_state[idx].q for idx in self.all_joint_ids}
            self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q = 1.0  # arm_sdk bekapcsolva
            self.enter_stage(self.STAGE_REACH)

        elapsed = time.time() - self.stage_t0

        # Alapból MINDEN ízület (passzív kar + derék, és -- amíg egy fázis
        # felül nem írja -- az aktív kar is) a kiindulási pózban marad,
        # rögzítve. Ez tartja a deréknak, hogy ne dőljön meg.
        for idx in self.all_joint_ids:
            self.write_joint(idx, self.start_q[idx])

        if self.stage == self.STAGE_REACH:
            ratio = min(elapsed / self.REACH_SECONDS, 1.0)
            for idx, target in zip(self.active_joint_ids, self.reach_pose):
                q0 = self.start_q[idx]
                self.write_joint(idx, (1.0 - ratio) * q0 + ratio * target)
            if ratio >= 1.0:
                self.enter_stage(self.STAGE_HOLD)

        elif self.stage == self.STAGE_HOLD:
            for idx, target in zip(self.active_joint_ids, self.reach_pose):
                self.write_joint(idx, target)

            held_for = time.time() - self.hold_since
            if not self.calibrated:
                if held_for >= self.CALIB_SECONDS:
                    self.detector.calibrate()
                    self.calibrated = True
                    units = {"tau": "Nm", "dq": "rad/s"}
                    for name, idx in zip(WATCH_JOINT_NAMES, self.detector.joint_indices):
                        for sig in SlapDetector.SIGNALS:
                            mean, std = self.detector.baseline[(idx, sig)]
                            margin, max_t = (self.min_margin, self.max_threshold) if sig == "tau" else (self.dq_margin, self.dq_max)
                            threshold = self.detector.channel_threshold((idx, sig), self.z_threshold, margin, max_t)
                            print(f"  {name} [{sig}]: baseline={mean:+.3f} {units[sig]}, std={std:.3f}  -> küszöb={threshold:.2f} {units[sig]}")
                    print("Várakozás pacsira...")
            else:
                key_hit, deviation = self.detector.check_hit(
                    self.z_threshold, self.min_margin, self.max_threshold, self.dq_margin, self.dq_max)
                now = time.time()
                if key_hit is not None:
                    if self.hit_since is None:
                        self.hit_since = now
                    self.last_hit_time = now
                    if now - self.hit_since >= self.min_hold:
                        self.hit_detected = True
                        self.last_deviation = deviation
                        self.last_hit_signal = key_hit[1]
                elif self.hit_since is not None and (now - self.last_hit_time) > self.hit_gap:
                    self.hit_since = None

                if self.last_debug_print is None or now - self.last_debug_print >= 0.5:
                    self.last_debug_print = now
                    devs = self.detector.max_deviations()
                    print(f"  ... aktuális kiugrás -> tau: {devs['tau']:.2f} Nm | dq: {devs['dq']:.2f} rad/s")

                timed_out = held_for >= (self.CALIB_SECONDS + self.timeout)
                if self.hit_detected or timed_out:
                    if self.hit_detected:
                        unit = "Nm" if self.last_hit_signal == "tau" else "rad/s"
                        print(f"Pacsi érzékelve! (eltérés: {self.last_deviation:.2f} {unit}, jel: {self.last_hit_signal})")
                        self.enter_stage(self.STAGE_REACT)
                    elif self.react_on_timeout:
                        print(f"Nem éreztem pacsit {self.timeout:.0f} mp után, automatikusan reagál...")
                        self.enter_stage(self.STAGE_REACT)
                    else:
                        print(f"Nem éreztem pacsit {self.timeout:.0f} mp után, visszahúzom a kart...")
                        self.enter_stage(self.STAGE_RETRACT)

        elif self.stage == self.STAGE_REACT:
            for idx, target in zip(self.active_joint_ids, self.reach_pose):
                self.write_joint(idx, target)
            offset = self.recoil_amplitude * math.sin(2.0 * math.pi * self.recoil_hz * elapsed)
            self.write_joint(self._elbow_id, self.reach_pose[self._elbow_pos] + offset)

            duration = self.recoil_cycles / self.recoil_hz
            if elapsed >= duration:
                self.enter_stage(self.STAGE_RETRACT)

        elif self.stage == self.STAGE_RETRACT:
            ratio = min(elapsed / self.RETRACT_SECONDS, 1.0)
            for idx in self.active_joint_ids:
                q0 = self.retract_from_q[idx]
                target = self.start_q[idx]  # pontosan oda megy vissza, ahonnan indultunk
                self.write_joint(idx, (1.0 - ratio) * q0 + ratio * target)
            if ratio >= 1.0:
                self.enter_stage(self.STAGE_SETTLE)

        elif self.stage == self.STAGE_SETTLE:
            # Nincs teendő -- a ciklus elején mindenki már start_q-ra van írva.
            if elapsed >= self.SETTLE_SECONDS:
                self.enter_stage(self.STAGE_RELEASE)

        elif self.stage == self.STAGE_RELEASE:
            weight = max(1.0 - elapsed / self.RELEASE_SECONDS, 0.0)
            self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q = weight
            # a pozíciók már a start pózban vannak a ciklus elején végzett alap-írás miatt
            if weight <= 0.0:
                self.stage = self.STAGE_DONE
                self.done = True

        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.publisher.Write(self.low_cmd)


def main():
    parser = argparse.ArgumentParser(description="G1 pacsi (high five) demo -- arm_sdk + tau_est/dq-alapú ütés-érzékelés")
    parser.add_argument("net", help="hálózati interfész, pl. en6")
    parser.add_argument("--arm", choices=["left", "right"], default="right",
                        help="melyik kart mozgassa (alap: right)")
    parser.add_argument("--style", choices=["top", "bottom"], default="top",
                        help="'top' = felső pacsi (kar felemelve), 'bottom' = alsó pacsi (kar lent, tenyér felfelé) (alap: top)")
    parser.add_argument("--shoulder-pitch", type=float, default=None,
                        help="felülírja a HIGH_FIVE_POSE ShoulderPitch értékét -- élő hangoláshoz, fájlszerkesztés nélkül")
    parser.add_argument("--shoulder-roll", type=float, default=None,
                        help="felülírja a HIGH_FIVE_POSE ShoulderRoll értékét")
    parser.add_argument("--elbow", type=float, default=None,
                        help="felülírja a HIGH_FIVE_POSE Elbow értékét")
    parser.add_argument("--timeout", type=float, default=10.0,
                        help="max. várakozás a pacsira a kalibráció után (alap: 10.0)")
    parser.add_argument("--react-on-timeout", action="store_true",
                        help="ha megadva, időtúllépéskor is lejátssza a REACT mozdulatot (alap: nem, csak visszahúz)")
    parser.add_argument("--z-threshold", type=float, default=4.0,
                        help="hány szórásnyi kiugrás számít ütésnek (alap: 4.0)")
    parser.add_argument("--min-margin", type=float, default=0.3,
                        help="minimum nyomaték-eltérés (tau_est) Nm-ben, zajszint alsó korlátja (alap: 0.3)")
    parser.add_argument("--min-hold", type=float, default=0.03,
                        help="a kiugrásnak legalább ennyi másodpercig kell fennállnia -- rövidebb, mint kézfogásnál, mert egy pacsi gyors (alap: 0.03)")
    parser.add_argument("--hit-gap", type=float, default=0.15,
                        help="ennyi másodpercnél rövidebb kiesés nem törli a folyamatos kiugrás számlálását (alap: 0.15)")
    parser.add_argument("--max-threshold", type=float, default=2.0,
                        help="a nyomaték-küszöb (tau_est) felső korlátja Nm-ben -- ha egy ízület zajos/rezeg, ne legyen érzékelhetetlen (alap: 2.0)")
    parser.add_argument("--dq-margin", type=float, default=0.1,
                        help="minimum szögsebesség-eltérés (dq) rad/s-ban, zajszint alsó korlátja -- egy ütés itt is kiugrást okoz, gyakran élesebbet, mint a nyomatékban (alap: 0.1)")
    parser.add_argument("--dq-max", type=float, default=2.0,
                        help="a szögsebesség-küszöb (dq) felső korlátja rad/s-ban (alap: 2.0)")
    parser.add_argument("--recoil-amplitude", type=float, default=0.2,
                        help="a REACT fázis könyök-kilengésének amplitúdója radiánban (alap: 0.2)")
    parser.add_argument("--recoil-hz", type=float, default=4.0,
                        help="a REACT fázis frekvenciája Hz-ben -- gyors, egyetlen 'visszarezdülés' érzethez (alap: 4.0)")
    parser.add_argument("--recoil-cycles", type=int, default=1,
                        help="hány kilengést csináljon a REACT fázisban (alap: 1)")
    parser.add_argument("--kp", type=float, default=60.0,
                        help="pozíció-szabályzó merevsége (alap: 60.0)")
    parser.add_argument("--kd", type=float, default=1.5,
                        help="pozíció-szabályzó csillapítása -- ha az ízület rezeg tartás közben, növeld (alap: 1.5)")
    parser.add_argument("--say", default=None,
                        help="ha meg van adva, ezt a magyar szöveget mondja ki a robot, amikor a pacsit érzékeli")
    parser.add_argument("--voice", default=DEFAULT_VOICE,
                        help=f"edge-tts hang a --say szöveghez (alap: {DEFAULT_VOICE}, másik opció: hu-HU-TamasNeural)")
    args = parser.parse_args()

    print("FIGYELEM: győződj meg róla, hogy nincs akadály a kar körül -- ez low-level kar-vezérlés.")
    print("FIGYELEM: a --style pózok NINCSENEK letesztelve ezen a roboton, első futtatáskor figyeld a kart!")
    input("Nyomj Entert a folytatáshoz...")

    ChannelFactoryInitialize(0, args.net)

    pose_override = {
        0: args.shoulder_pitch,  # ShoulderPitch
        1: args.shoulder_roll,   # ShoulderRoll
        3: args.elbow,           # Elbow
    }

    demo = HighFiveDemo(
        arm=args.arm, style=args.style, pose_override=pose_override,
        timeout=args.timeout, react_on_timeout=args.react_on_timeout,
        z_threshold=args.z_threshold, min_margin=args.min_margin, min_hold=args.min_hold,
        hit_gap=args.hit_gap, max_threshold=args.max_threshold,
        dq_margin=args.dq_margin, dq_max=args.dq_max,
        recoil_amplitude=args.recoil_amplitude, recoil_hz=args.recoil_hz, recoil_cycles=args.recoil_cycles,
        kp=args.kp, kd=args.kd, say_text=args.say, say_voice=args.voice,
    )
    demo.init_channels()
    demo.wait_for_state()

    thread = PeriodicThread(interval=demo.control_dt, target=demo.control_tick)
    thread.Start()

    try:
        while not demo.done:
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nMegszakítva, kar visszaengedése...")
        demo.enter_stage(HighFiveDemo.STAGE_RETRACT)
        while not demo.done:
            time.sleep(0.1)

    print("Demo kész.")


if __name__ == "__main__":
    main()
