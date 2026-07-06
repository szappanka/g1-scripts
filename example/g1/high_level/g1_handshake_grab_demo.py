"""
G1 kézfogás demo -- SAJÁT, vezérelt fel-le mozgással a könyökből, az "arm_sdk"
(rt/arm_sdk) alacsony szintű kar-felülírás segítségével, tau_est-alapú
"megfogták a kezét" érzékeléssel.

Az előző verzió a beépített LocoClient.ShakeHand() feladatra épült, de annak
a mozgása nem volt látható/kontrollálható (nem tudni, pontosan mit csinál a
firmware-ben). Ez a verzió helyette a hivatalos "arm_sdk" interfészt hasz-
nálja (ld. g1_arm7_sdk_dds_example.py mintája), ami a lábakat/egyensúlyt
nem érinti (az továbbra is a robot beépített, magas szintű vezérlőjén marad).

FONTOS: az arm_sdk a TELJES felsőtestet (mindkét kar + derék) egyben veszi
át a magas szintű vezérlőtől, nem ízületenként külön-külön. Ezért a szkript
minden cikluson (50 Hz) tartja a MÁSIK kart és a derekat is a kiindulási
pózban (kp/kd-vel rögzítve) -- ha ezeket nem tartanánk explicit módon,
tartóerő nélkül maradnának, és a robot előre/oldalra dőlhetne.

Menete:
  1) REACH     -- a kart a start-pózból a "kézfogásra nyújtott" pózba mozgatja
  2) HOLD      -- tartja a pózt, közben méri a könyök/csukló tau_est nyugalmi
                  zajszintjét, majd figyeli, mikor ugrik ki belőle (= megfogták)
  3) SHAKE     -- ha megfogták (vagy lejárt az időzítő): a könyököt fel-le
                  mozgatja N ciklusban -- ez a tényleges "kezet ráz" mozdulat.
                  Ha meg van adva --say, ekkor indul (külön szálon) a beszéd is.
  4) RETRACT   -- visszaviszi a kart pontosan a kiindulási pózba
  5) SETTLE    -- rövid, mozdulatlan tartás, hogy a visszahúzás lendülete leüljön
  6) RELEASE   -- fokozatosan visszaadja az irányítást a magas szintű vezérlőnek

Beszéd (--say): a beépített TtsMaker nem tud magyarul (csak kínai/angol), ezért
ez edge-tts-szel (ingyenes, nem hivatalos MS Edge felolvasó, nem kell API-kulcs)
szintetizál, majd a robot hangszóróján játssza le. Telepítés:
    pip install edge-tts miniaudio

FONTOS -- ELSŐ FUTTATÁS ELŐTT OLVASD EL:
- Ez már low-level jellegű kar-vezérlés, nem a kész gyári akció-kliens. A lenti
  REACH_POSE szögei csak egy KIINDULÓ BECSLÉS -- első futtatáskor lassan,
  akadálymentes területen, valaki figyelje a kart. Ha a REACH fázisban rossz
  irányba mozdul a kar, állítsd a REACH_POSE értékeit (előjelek/nagyságok).
- Ctrl+C esetén a szkript megpróbálja RETRACT+RELEASE-elni a kart, de fizikai
  vészleállítót/damp módot is tarts készenlétben, mint minden low-level
  tesztnél.
- A --z-threshold / --min-margin / --min-hold / --hit-gap kalibrálandó a
  konkrét robotnál -- a HOLD fázis elején kiírt baseline tau_est/std, majd a
  másodpercenként kiírt "aktuális kiugrás" érték alapján állítsd, ha túl
  érzékeny vagy túl érzéketlen (esetleg csak időnként érzékel) a "megfogás".

Használat:
    python3 g1_handshake_grab_demo.py <networkInterface>
    python3 g1_handshake_grab_demo.py enp2s0 --arm left --timeout 20
    python3 g1_handshake_grab_demo.py enp2s0 --shake-amplitude 0.2 --shake-hz 2
    python3 g1_handshake_grab_demo.py enp2s0 --say "Örülök, hogy találkoztunk!"

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

# Kézfogásra nyújtott póz -- a robotnál letesztelve/hangolva.
# Sorrend: ShoulderPitch, ShoulderRoll, ShoulderYaw, Elbow, WristRoll, WristPitch, WristYaw  [rad]
# FONTOS: ezen a robotnál az Elbow NEGATÍV értéke hajlít helyesen -- pozitív
# érték (tesztelve: 1.9) a könyököt veszélyesen HÁTRAFELÉ (túlfeszítve) hajlítja!
REACH_POSE = {
    "left":  [-0.3,  0.25, 0.0, -0.3, 0.0, 0.0, 0.0],
    "right": [-0.3, -0.25, 0.0, -0.3, 0.0, 0.0, 0.0],
}

# Elbow + WristRoll mindkét karváltozaton (5-DOF/7-DOF) létezik -- ezeket figyeljük.
WATCH_JOINT_NAMES = ["Elbow", "WristRoll"]


class GrabDetector:
    """A megadott ízületek tau_est-jét gyűjti és dönt arról, kiugrott-e a nyugalmi szinthez képest."""

    def __init__(self, joint_indices, history_len=1000):
        self.joint_indices = joint_indices
        self.history_len = history_len
        self.latest_tau = {}
        self.baseline = {}  # idx -> (mean, std)
        self.history = {idx: deque(maxlen=history_len) for idx in joint_indices}

    def reset_history(self):
        self.history = {idx: deque(maxlen=self.history_len) for idx in self.joint_indices}

    def on_lowstate(self, msg: LowState_):
        for idx in self.joint_indices:
            tau = msg.motor_state[idx].tau_est
            self.latest_tau[idx] = tau
            self.history[idx].append(tau)

    def calibrate(self):
        for idx in self.joint_indices:
            samples = list(self.history[idx])
            if len(samples) < 10:
                self.baseline[idx] = (0.0, 0.05)
                continue
            mean = statistics.mean(samples)
            std = statistics.pstdev(samples) or 0.02
            self.baseline[idx] = (mean, std)

    def joint_threshold(self, idx, z_threshold, min_margin, max_threshold):
        """Az adott ízület tényleges trigger-küszöbe -- felülről korlátozva, hogy egy zajos
        (pl. rezgő) ízület kalibrált nagy szórása ne tegye gyakorlatilag érzékelhetetlenné."""
        _, std = self.baseline.get(idx, (0.0, 0.0))
        return min(max(z_threshold * std, min_margin), max_threshold)

    def check_grab(self, z_threshold, min_margin, max_threshold):
        best_idx, best_dev = None, 0.0
        for idx in self.joint_indices:
            if idx not in self.latest_tau or idx not in self.baseline:
                continue
            mean, _ = self.baseline[idx]
            deviation = abs(self.latest_tau[idx] - mean)
            threshold = self.joint_threshold(idx, z_threshold, min_margin, max_threshold)
            if deviation > threshold and deviation > best_dev:
                best_idx, best_dev = idx, deviation
        return best_idx, best_dev

    def max_deviation(self):
        """A jelenlegi legnagyobb eltérés a nyugalmi szinthez képest -- diagnosztikához, küszöbtől függetlenül."""
        best = 0.0
        for idx in self.joint_indices:
            if idx not in self.latest_tau or idx not in self.baseline:
                continue
            mean, _ = self.baseline[idx]
            dev = abs(self.latest_tau[idx] - mean)
            if dev > best:
                best = dev
        return best


class HandshakeDemo:
    STAGE_REACH = "REACH"
    STAGE_HOLD = "HOLD"
    STAGE_SHAKE = "SHAKE"
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

    def __init__(self, arm, timeout, z_threshold, min_margin, min_hold, hit_gap, max_threshold,
                 shake_amplitude, shake_hz, shake_cycles, kp, kd, say_text, say_voice):
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

        self.reach_pose = REACH_POSE[arm]
        self._elbow_pos = ARM_JOINT_NAMES.index("Elbow")
        self._elbow_id = self.active_joint_ids[self._elbow_pos]

        watch_ids = [getattr(G1JointIndex, f"{prefix}{name}") for name in WATCH_JOINT_NAMES]
        self.detector = GrabDetector(watch_ids)

        self.timeout = timeout
        self.z_threshold = z_threshold
        self.min_margin = min_margin
        self.min_hold = min_hold
        self.hit_gap = hit_gap
        self.max_threshold = max_threshold
        self.shake_amplitude = shake_amplitude
        self.shake_hz = shake_hz
        self.shake_cycles = shake_cycles

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
        self.grabbed = False
        self.last_deviation = 0.0
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
                play_pcm_stream(self.audio_client, list(pcm_bytes), "handshake_greeting")
                self.audio_client.PlayStop("handshake_greeting")
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
        elif stage == self.STAGE_SHAKE:
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
                    for name, idx in zip(WATCH_JOINT_NAMES, self.detector.joint_indices):
                        mean, std = self.detector.baseline[idx]
                        threshold = self.detector.joint_threshold(idx, self.z_threshold, self.min_margin, self.max_threshold)
                        print(f"  {name}: baseline tau_est={mean:+.3f} Nm, std={std:.3f}  -> küszöb={threshold:.2f} Nm")
                    print("Várakozás megfogásra...")
            else:
                idx_hit, deviation = self.detector.check_grab(self.z_threshold, self.min_margin, self.max_threshold)
                now = time.time()
                if idx_hit is not None:
                    if self.hit_since is None:
                        self.hit_since = now
                    self.last_hit_time = now
                    if now - self.hit_since >= self.min_hold:
                        self.grabbed = True
                        self.last_deviation = deviation
                elif self.hit_since is not None and (now - self.last_hit_time) > self.hit_gap:
                    # csak akkor nullázzuk a számlálót, ha a kiugrás egy rövid
                    # türelmi időnél (--hit-gap) tovább szünetel -- egy valódi
                    # kézfogás nem tökéletesen egyenletes erejű, apró, egy-két
                    # mintányi kiesések nem szabad, hogy törjék a detektálást.
                    self.hit_since = None

                if self.last_debug_print is None or now - self.last_debug_print >= 0.5:
                    self.last_debug_print = now
                    print(f"  ... aktuális kiugrás: {self.detector.max_deviation():.2f} Nm")

                timed_out = held_for >= (self.CALIB_SECONDS + self.timeout)
                if self.grabbed or timed_out:
                    if self.grabbed:
                        print(f"Megfogva! (eltérés: {self.last_deviation:.2f} Nm) Kezet ráz...")
                    else:
                        print(f"Nem érzékeltem megfogást {self.timeout:.0f} mp után, automatikusan ráz...")
                    self.enter_stage(self.STAGE_SHAKE)

        elif self.stage == self.STAGE_SHAKE:
            for idx, target in zip(self.active_joint_ids, self.reach_pose):
                self.write_joint(idx, target)
            offset = self.shake_amplitude * math.sin(2.0 * math.pi * self.shake_hz * elapsed)
            self.write_joint(self._elbow_id, self.reach_pose[self._elbow_pos] + offset)

            duration = self.shake_cycles / self.shake_hz
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
    parser = argparse.ArgumentParser(description="G1 kézfogás demo -- arm_sdk + tau_est-alapú megfogás-érzékelés")
    parser.add_argument("net", help="hálózati interfész, pl. enp2s0")
    parser.add_argument("--arm", choices=["left", "right"], default="right",
                        help="melyik kart mozgassa (alap: right)")
    parser.add_argument("--timeout", type=float, default=10.0,
                        help="max. várakozás a kézfogásra a kalibráció után (alap: 10.0)")
    parser.add_argument("--z-threshold", type=float, default=6.0,
                        help="hány szórásnyi kiugrás számít megfogásnak (alap: 6.0)")
    parser.add_argument("--min-margin", type=float, default=0.5,
                        help="minimum nyomaték-eltérés Nm-ben, zajszint alsó korlátja (alap: 0.5)")
    parser.add_argument("--min-hold", type=float, default=0.1,
                        help="a kiugrásnak legalább ennyi másodpercig (összesítve) fenn kell állnia (alap: 0.1)")
    parser.add_argument("--hit-gap", type=float, default=0.15,
                        help="ennyi másodpercnél rövidebb kiesés nem törli a folyamatos kiugrás számlálását (alap: 0.15)")
    parser.add_argument("--max-threshold", type=float, default=3.0,
                        help="a küszöb felső korlátja Nm-ben -- ha egy ízület zajos/rezeg, ne legyen érzékelhetetlen (alap: 3.0)")
    parser.add_argument("--shake-amplitude", type=float, default=0.15,
                        help="a könyök fel-le mozgásának amplitúdója radiánban (alap: 0.15)")
    parser.add_argument("--shake-hz", type=float, default=1.5,
                        help="a rázás frekvenciája Hz-ben (alap: 1.5)")
    parser.add_argument("--shake-cycles", type=int, default=3,
                        help="hány fel-le ciklust rázzon (alap: 3)")
    parser.add_argument("--kp", type=float, default=60.0,
                        help="pozíció-szabályzó merevsége (alap: 60.0)")
    parser.add_argument("--kd", type=float, default=1.5,
                        help="pozíció-szabályzó csillapítása -- ha az ízület rezeg tartás közben, növeld (alap: 1.5)")
    parser.add_argument("--say", default=None,
                        help="ha meg van adva, ezt a magyar szöveget mondja ki a robot, amikor rázni kezdi a kezet")
    parser.add_argument("--voice", default=DEFAULT_VOICE,
                        help=f"edge-tts hang a --say szöveghez (alap: {DEFAULT_VOICE}, másik opció: hu-HU-TamasNeural)")
    args = parser.parse_args()

    print("FIGYELEM: győződj meg róla, hogy nincs akadály a kar körül -- ez low-level kar-vezérlés.")
    input("Nyomj Entert a folytatáshoz...")

    ChannelFactoryInitialize(0, args.net)

    demo = HandshakeDemo(
        arm=args.arm, timeout=args.timeout, z_threshold=args.z_threshold,
        min_margin=args.min_margin, min_hold=args.min_hold, hit_gap=args.hit_gap,
        max_threshold=args.max_threshold,
        shake_amplitude=args.shake_amplitude, shake_hz=args.shake_hz, shake_cycles=args.shake_cycles,
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
        demo.enter_stage(HandshakeDemo.STAGE_RETRACT)
        while not demo.done:
            time.sleep(0.1)

    print("Demo kész.")


if __name__ == "__main__":
    main()
