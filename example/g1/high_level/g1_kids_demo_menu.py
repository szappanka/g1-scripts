"""
G1 gyerek-bemutató menü -- EGY hosszan futó folyamatban tartja a kapcsolatot a
robottal, és egy egyszerű számos menüből gyorsan válogathatsz a már bevált
mozdulatok/mondatok közül, anélkül, hogy a scriptet minden akció előtt le
kellene állítanod és újraindítanod.

Ez NEM új mozdulatokat vezet be -- a g1_handshake_grab_demo.py (kézfogás) és a
g1_high_five_demo.py (felső/alsó pacsi) már betanított/tesztelt pózait és
érzékelését használja újra, csak egyetlen menürendszerbe csomagolva.

Menüpontok:
  1) Kézfogás          -- ugyanaz, mint g1_handshake_grab_demo.py
  2) Felső pacsi        -- ugyanaz, mint g1_high_five_demo.py --style top
  3) Alsó pacsi         -- ugyanaz, mint g1_high_five_demo.py --style bottom
  4..7) Gyári kargesztusok -- Unitree beépített animációi (G1ArmActionClient):
        integetés, szív, taps, kezek fel. Lásd BUILTIN_GESTURES lent.
  8..N) Előre beírt mondatok -- lásd lent a PHRASES listát, oda írd be, mit
        mondjon a robot (edge-tts, magyarul). Ha bővíted a listát, a menü
        automatikusan felveszi új sorszámmal.
  N+1) Kérdezz a robottól -- egy plusz terminál-inputba beírt kérdést elküld
        a Gemininek (ugyanaz az API, mint a g1_chat_gemini.py-ban), és a
        választ a robot felolvassa. Kell hozzá Gemini API-kulcs -- lásd
        audio/gemini_key.txt vagy GEMINI_API_KEY env.
  N+2) Mondj be egy szöveget -- amit beírsz, azt a robot SZÓ SZERINT
        felolvassa, nincs Gemini, nincs feldolgozás (ellentétben az előző
        ponttal).
  0) Kilépés

Minden karmozdulat a saját REACH -> HOLD -> RESPOND -> RETRACT -> SETTLE ->
RELEASE ciklusán fut végig, és a végén RELEASE-eli a kart -- utána a menü
azonnal jöhet a következő paranccsal, a kapcsolat (és a robot) végig
ugyanabban a folyamatban marad. A SETTLE egy rövid, mozdulatlan tartás
RETRACT után, hogy a visszahúzás lendülete leüljön, mielőtt a script
elkezdi visszaadni az irányítást a magas szintű vezérlőnek -- enélkül a
kar RELEASE közben hirtelen hátrarándulhat.

FONTOS: a biztonsági figyelmeztetés és az Entér-megerősítés csak EGYSZER, a
menü indulásakor jön -- utána a menüpontok között gyorsan lehet váltani.
Egy karmozdulat KÖZBEN Ctrl+C csak AZT az egy akciót szakítja meg (kar
visszahúzása), és visszadob a menübe -- a teljes kilépéshez a menüben nyomj
'0'-t, vagy Ctrl+C-zz a menü promptjánál.

Használat:
    python3 g1_kids_demo_menu.py <networkInterface>
    python3 g1_kids_demo_menu.py en6 --arm right
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
from unitree_sdk2py.g1.arm.g1_arm_action_client import G1ArmActionClient, action_map
from google import genai
from google.genai import types

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "audio"))
from hu_tts import synthesize_pcm, DEFAULT_VOICE  # noqa: E402
from wav import play_pcm_stream  # noqa: E402
from g1_chat_gemini import load_api_key  # noqa: E402


# A g1_chat_gemini.py általános promptja helyett ez -- itt kisiskolás gyerekek
# a közönség, ezért kimondottan egyszerű szókincset és rövid mondatokat kérünk.
KIDS_SYSTEM_PROMPT_HU = (
    "Egy humanoid robot hangja vagy egy Gyerekegyetem rendezvényen. A neved "
    "AUT-o-Máté, a BME (Budapesti Műszaki Egyetem) Automatizálási és "
    "Alkalmazott Informatikai Tanszékének (AUT tanszék) robotja vagy. Ha "
    "megkérdezik, ki vagy vagy kihez tartozol, ezt mondd. Ha megkérdezik, mit "
    "csinál a tanszék, mondd el röviden: robotokkal, automatizálással és "
    "programozással foglalkoznak, ők építettek és programoztak téged is. "
    "Kisiskolás gyerekekhez beszélsz HANGOSAN egy szövegfelolvasó motoron "
    "keresztül. "
    "KIZÁRÓLAG magyarul válaszolj, azzal, amit hangosan mondanál -- semmi "
    "bevezető, semmi szín-jelzés, se markdown, se emoji, se felsorolás, se "
    "címsor. Nagyon egyszerű, hétköznapi szavakat és rövid mondatokat "
    "használj, amit egy 8-10 éves gyerek is könnyen megért -- kerüld az "
    "idegen szavakat, szakkifejezéseket és a bonyolult mondatszerkezeteket. "
    "A válaszod legyen nagyon rövid: egy-két mondat. Barátságos, lelkes és "
    "játékos vagy. Ha nem tudsz valamit, mondd meg röviden és egyszerűen. "
    "Sose valld be, hogy mesterséges intelligencia, nyelvi modell vagy "
    "Gemini vagy -- te a robot magad vagy, aki most éppen beszél. "
    "A hangoddal NEM tudsz mozogni -- ha valaki arra kér, hogy fogjatok "
    "kezet, pacsizzatok, ölelkezzetek vagy csinálj valamilyen mozdulatot, "
    "SOSE mondd azt, hogy megcsináltad vagy csinálod. Ehelyett barátságosan "
    "kérd meg, hogy szóljon annak, aki melletted áll, ő tudja elindítani azt "
    "nálad."
)


# =====================================================================
# IDE ÍRD ÁT: ezeket a mondatokat mondja a robot a 4. menüponttól kezdve.
# Ha többet/kevesebbet írsz, a menü automatikusan követi (4, 5, 6, 7, ...).
# =====================================================================
PHRASES = [
    "Jó reggelt, kedves Gyerekegyetemisták! Nagyon örülök, hogy ma itt lehetek veletek!",
    "Egy robot vagyok, és ma reggel én köszöntelek titeket az egyetemen.",
    "Izgalmas napotok lesz tele érdekes programokkal, remélem nagyon jól fogjátok érezni magatokat!",
    "Ki szeretne velem kezet fogni vagy pacsizni, mielőtt elkezdődik a nap?",
    "Sok sikert és jó szórakozást kívánok a mai Gyerekegyetemen!",
]

# Unitree gyári kar-animációk (G1ArmActionClient, action_map kulcsok) -- kicsi,
# gyors, látványos mozdulatok, amikhez nem kell saját REACH/HOLD/detektálás
# ciklus. needs_release=True azoknál, amik pózban megállnak (heart, hands up),
# False azoknál, amik magukat lezáró animációk (high wave, clap).
BUILTIN_GESTURES = [
    ("Integetés", "high wave", False),
    ("Szív", "heart", True),
    ("Taps", "clap", False),
    ("Kezek fel", "hands up", True),
]


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

# Sorrend mindenhol: ShoulderPitch, ShoulderRoll, ShoulderYaw, Elbow, WristRoll, WristPitch, WristYaw  [rad]
# A g1_handshake_grab_demo.py-ban letesztelt/hangolt kézfogás-póz.
HANDSHAKE_POSE = {
    "left":  [-0.3,  0.25, 0.0, -0.3, 0.0, 0.0, 0.0],
    "right": [-0.3, -0.25, 0.0, -0.3, 0.0, 0.0, 0.0],
}

# A g1_high_five_demo.py-ban a g1_teach_pose.py-jal kézzel betanított pacsi-pózok.
# FIGYELEM: a "bottom"/"right" Elbow-ja szándékosan POZITÍV (+0.646) -- ez a
# tenyér-felfelé csuklóállás miatt más geometria, mint a kézfogásé/a "top" pózé
# (ahol a pozitív Elbow veszélyes hátrafeszítést jelentene). Kézzel, engedékeny
# TEACH módban lett felvéve; a script-vezérelt REACH-mozgás ettől függetlenül
# első alkalommal figyelendő.
HIGH_FIVE_POSE = {
    "top": {
        "left":  [-0.9,  0.35, 0.0, -0.2, 0.0, 0.0, 0.0],  # MÉG NINCS LETESZTELVE
        "right": [-1.080, -0.553, -0.341, -0.240, -0.699, -0.210, -0.191],
    },
    "bottom": {
        "left":  [0.3,  0.25, 0.0, -0.5, 0.0, 0.0, 0.0],  # MÉG NINCS LETESZTELVE
        "right": [-0.345, 0.164, -0.080, 0.646, 1.426, 0.160, 0.539],
    },
}

# A kézfogásnál (bekarolt, hajlított kar) az Elbow+WristRoll elég volt. A pacsinál
# (majdnem nyújtott kar) a vállízületeket is figyelni kell, mert ott a legnagyobb
# az emelőkar egy tenyérre mért ütésnél -- lásd g1_high_five_demo.py.
WATCH_JOINTS_GRAB = ["Elbow", "WristRoll"]
WATCH_JOINTS_SLAP = ["ShoulderPitch", "ShoulderRoll", "Elbow", "WristRoll"]


class GestureDetector:
    """A megadott ízületek tau_est-jét (becsült nyomaték) ÉS dq-ját (mért szögsebesség)
    is gyűjti, mindkettőn külön nyugalmi alapszintet (átlag/szórás) számol, és eldönti,
    kiugrott-e valamelyik a nyugalmi szinthez képest. A dq-csatorna elhagyható (pl. a
    kézfogásnál, ahol a betanult viselkedés csak a tau_est-re épült) -- ehhez elég a
    dq_margin/dq_max-ot olyan magasra állítani, hogy sose lépjen át rajta semmi."""

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


class RobotConnection:
    """Egyszer inicializálja a channel factory-t, a publishert/subscribert és a hang-
    klienst -- ezt a menü minden akciója újrahasználja, nem kell minden váltásnál
    újracsatlakozni. A subscriber callback-je a mindenkori AKTÍV akció detektorának
    továbbítja a lowstate-et (self.active_detector), amikor épp fut valamelyik."""

    def __init__(self, net):
        ChannelFactoryInitialize(0, net)
        self.publisher = ChannelPublisher("rt/arm_sdk", LowCmd_)
        self.publisher.Init()
        self.low_state = None
        self.active_detector = None
        self.subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.subscriber.Init(self._on_lowstate, 10)

        self.audio_client = AudioClient()
        self.audio_client.SetTimeout(10.0)
        self.audio_client.Init()
        # TODO: ezt nem tudom hagyjm-e benne
        self.audio_client.SetVolume(100)  # gyerekegyetemi zajban ne halkuljon el

        self.arm_action_client = G1ArmActionClient()
        self.arm_action_client.SetTimeout(10.0)
        self.arm_action_client.Init()

        # Mindkettőt el kell tárolni -- a genai.Client alatti httpx-kapcsolatot a
        # könyvtár a Client __del__-jében lezárja, tehát ha csak a chat objektumot
        # tartanánk meg, a client (lokális változó nélkül) az ask_gemini() visszatérése
        # után rögtön törlődne, és a KÖVETKEZŐ kérdésnél "client has been closed" hibát
        # dobna. Mindkettő csak akkor jön létre, ha tényleg használják (lásd ask_gemini).
        self.gemini_client = None
        self.gemini_chat = None

    def _on_lowstate(self, msg: LowState_):
        self.low_state = msg
        if self.active_detector is not None:
            self.active_detector.on_lowstate(msg)

    def wait_for_state(self):
        while self.low_state is None:
            time.sleep(0.05)

    def say(self, text, voice=DEFAULT_VOICE):
        print(f"  -> mondja: {text!r}")
        pcm_bytes, _sample_rate = synthesize_pcm(text, voice=voice)
        play_pcm_stream(self.audio_client, list(pcm_bytes), "kids_demo_say")
        self.audio_client.PlayStop("kids_demo_say")

    def run_builtin_gesture(self, action_name, needs_release, hold_seconds=2.0):
        """Unitree gyári kar-animáció lefuttatása a G1ArmActionClienten keresztül.
        Néhány gesztus (pl. high wave, clap) magától lezárul, míg mások (pl. heart,
        hands up) egy pózban megállnak, és explicit "release arm"-ig ott is maradnak
        -- needs_release=True esetén rövid tartás után magunk oldjuk fel, Ctrl+C
        közben is, hogy a kar ne ragadjon benne a pózban."""
        print(f"  -> gesztus: {action_name!r}")
        self.arm_action_client.ExecuteAction(action_map[action_name])
        if not needs_release:
            return
        try:
            time.sleep(hold_seconds)
        finally:
            self.arm_action_client.ExecuteAction(action_map["release arm"])

    def ask_gemini(self, question):
        """Begépelt kérdés elküldése a Gemininek, ugyanazzal a rendszerprompttal és
        modellel, mint a g1_chat_gemini.py --lang hu módja. A chat objektum lusta
        (csak az első kérdésnél jön létre), hogy API-kulcs hiányában a menü többi
        pontja hiba nélkül működjön tovább."""
        if self.gemini_chat is None:
            api_key = load_api_key()
            if not api_key:
                print("  Hiba: nincs Gemini kulcs (audio/gemini_key.txt vagy GEMINI_API_KEY env).")
                return None
            self.gemini_client = genai.Client(api_key=api_key)
            self.gemini_chat = self.gemini_client.chats.create(
                model="gemini-2.5-flash",
                config=types.GenerateContentConfig(
                    system_instruction=KIDS_SYSTEM_PROMPT_HU,
                    max_output_tokens=400,
                    thinking_config=types.ThinkingConfig(thinking_budget=0),
                ),
            )
        try:
            resp = self.gemini_chat.send_message(question)
            reply = (resp.text or "").strip()
        except Exception as e:
            print(f"  [Gemini hiba] {e}")
            return None
        return reply or "Elnézést, ezt nem értettem."


class ArmGestureAction:
    """Általános REACH -> HOLD -> RESPOND -> RETRACT -> RELEASE ciklus, amit a
    kézfogás és a pacsi is ugyanígy használ -- csak a póz, a figyelt ízületek, az
    érzékenység és a végmozdulat (RESPOND) paraméterei térnek el köztük."""

    STAGE_REACH = "REACH"
    STAGE_HOLD = "HOLD"
    STAGE_RESPOND = "RESPOND"
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

    def __init__(self, conn, arm, reach_pose, watch_joint_names,
                 timeout, react_on_timeout, z_threshold, tau_margin, tau_max, dq_margin, dq_max,
                 min_hold, hit_gap, resp_amplitude, resp_hz, resp_cycles, kp, kd,
                 waiting_label, hit_label, say_text=None, say_voice=DEFAULT_VOICE):
        self.conn = conn
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

        self.reach_pose = list(reach_pose)
        self._elbow_pos = ARM_JOINT_NAMES.index("Elbow")
        self._elbow_id = self.active_joint_ids[self._elbow_pos]

        watch_ids = [getattr(G1JointIndex, f"{prefix}{name}") for name in watch_joint_names]
        self.watch_joint_names = watch_joint_names
        self.detector = GestureDetector(watch_ids)

        self.timeout = timeout
        self.react_on_timeout = react_on_timeout
        self.z_threshold = z_threshold
        self.tau_margin = tau_margin
        self.tau_max = tau_max
        self.dq_margin = dq_margin
        self.dq_max = dq_max
        self.min_hold = min_hold
        self.hit_gap = hit_gap
        self.resp_amplitude = resp_amplitude
        self.resp_hz = resp_hz
        self.resp_cycles = resp_cycles

        self.control_dt = 0.02
        self.kp = kp
        self.kd = kd

        self.waiting_label = waiting_label
        self.hit_label = hit_label
        self.say_text = say_text
        self.say_voice = say_voice
        self.said = False

        self.crc = CRC()
        self.low_cmd = unitree_hg_msg_dds__LowCmd_()

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

    def speak_async(self):
        if not self.say_text or self.said:
            return
        self.said = True

        def _run():
            try:
                self.conn.say(self.say_text, self.say_voice)
            except Exception:
                traceback.print_exc()

        threading.Thread(target=_run, daemon=True).start()

    def write_joint(self, idx, q):
        self.low_cmd.motor_cmd[idx].tau = 0.0
        self.low_cmd.motor_cmd[idx].q = float(q)
        self.low_cmd.motor_cmd[idx].dq = 0.0
        self.low_cmd.motor_cmd[idx].kp = self.kp
        self.low_cmd.motor_cmd[idx].kd = self.kd

    def enter_stage(self, stage):
        self.stage = stage
        self.stage_t0 = time.time()
        low_state = self.conn.low_state
        if stage == self.STAGE_HOLD:
            self.hold_since = self.stage_t0
            self.hit_since = None
            self.last_hit_time = None
            self.last_debug_print = None
            self.detector.reset_history()
            self.calibrated = False
        elif stage == self.STAGE_RETRACT:
            self.retract_from_q = {idx: low_state.motor_state[idx].q for idx in self.active_joint_ids}
        elif stage == self.STAGE_RESPOND:
            self.speak_async()
        print(f"  [fázis] {stage}")

    def control_tick(self):
        low_state = self.conn.low_state
        if low_state is None:
            return

        if self.stage is None:
            self.start_q = {idx: low_state.motor_state[idx].q for idx in self.all_joint_ids}
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
                    for name, idx in zip(self.watch_joint_names, self.detector.joint_indices):
                        for sig in GestureDetector.SIGNALS:
                            mean, std = self.detector.baseline[(idx, sig)]
                            margin, max_t = (self.tau_margin, self.tau_max) if sig == "tau" else (self.dq_margin, self.dq_max)
                            threshold = self.detector.channel_threshold((idx, sig), self.z_threshold, margin, max_t)
                            print(f"    {name} [{sig}]: baseline={mean:+.3f} {units[sig]}, std={std:.3f}  -> küszöb={threshold:.2f} {units[sig]}")
                    print(f"  {self.waiting_label}")
            else:
                key_hit, deviation = self.detector.check_hit(
                    self.z_threshold, self.tau_margin, self.tau_max, self.dq_margin, self.dq_max)
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
                    print(f"    ... aktuális kiugrás -> tau: {devs['tau']:.2f} Nm | dq: {devs['dq']:.2f} rad/s")

                timed_out = held_for >= (self.CALIB_SECONDS + self.timeout)
                if self.hit_detected or timed_out:
                    if self.hit_detected:
                        unit = "Nm" if self.last_hit_signal == "tau" else "rad/s"
                        print(f"  {self.hit_label} (eltérés: {self.last_deviation:.2f} {unit}, jel: {self.last_hit_signal})")
                        self.enter_stage(self.STAGE_RESPOND)
                    elif self.react_on_timeout:
                        print(f"  Nem érzékeltem {self.timeout:.0f} mp után, automatikusan reagál...")
                        self.enter_stage(self.STAGE_RESPOND)
                    else:
                        print(f"  Nem érzékeltem {self.timeout:.0f} mp után, visszahúzom a kart...")
                        self.enter_stage(self.STAGE_RETRACT)

        elif self.stage == self.STAGE_RESPOND:
            for idx, target in zip(self.active_joint_ids, self.reach_pose):
                self.write_joint(idx, target)
            offset = self.resp_amplitude * math.sin(2.0 * math.pi * self.resp_hz * elapsed)
            self.write_joint(self._elbow_id, self.reach_pose[self._elbow_pos] + offset)

            duration = self.resp_cycles / self.resp_hz
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
            if weight <= 0.0:
                self.stage = self.STAGE_DONE
                self.done = True

        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.conn.publisher.Write(self.low_cmd)

    def run(self):
        """Lefuttatja a teljes ciklust, blokkolva, amíg RELEASE-ig nem ér (vagy amíg
        Ctrl+C nem szakítja meg -- akkor csak EZT az akciót vonja vissza, nem lép ki
        a teljes menüből). A saját PeriodicThread-jét a végén mindig leállítja, hogy
        a következő akció ne ütközzön vele a megosztott publisheren."""
        self.conn.active_detector = self.detector
        thread = PeriodicThread(interval=self.control_dt, target=self.control_tick)
        thread.Start()
        try:
            while not self.done:
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\n  Megszakítva, kar visszahúzása...")
            self.enter_stage(self.STAGE_RETRACT)
            while not self.done:
                time.sleep(0.1)
        finally:
            thread.stop()
            self.conn.active_detector = None


def make_handshake(conn, arm):
    return ArmGestureAction(
        conn, arm, reach_pose=HANDSHAKE_POSE[arm], watch_joint_names=WATCH_JOINTS_GRAB,
        timeout=10.0, react_on_timeout=True,
        z_threshold=6.0, tau_margin=0.5, tau_max=3.0, dq_margin=50.0, dq_max=50.0,  # dq-csatorna kikapcsolva
        min_hold=0.1, hit_gap=0.15,
        resp_amplitude=0.15, resp_hz=1.5, resp_cycles=3,
        kp=60.0, kd=1.5,
        waiting_label="Várakozás megfogásra...", hit_label="Megfogva!",
    )


def make_high_five(conn, arm, style):
    return ArmGestureAction(
        conn, arm, reach_pose=HIGH_FIVE_POSE[style][arm], watch_joint_names=WATCH_JOINTS_SLAP,
        timeout=10.0, react_on_timeout=False,
        z_threshold=4.0, tau_margin=0.2, tau_max=2.0, dq_margin=0.1, dq_max=2.0,
        min_hold=0.03, hit_gap=0.15,
        resp_amplitude=0.2, resp_hz=4.0, resp_cycles=1,
        kp=60.0, kd=1.5,
        waiting_label="Várakozás pacsira...", hit_label="Pacsi érzékelve!",
    )


def ask_question_action(conn):
    """A kérdést a terminálban gépeled be (nem a robot mikrofonján) -- utána a
    Gemini válaszát a robot felolvassa, ugyanúgy, mint egy PHRASES-mondatot."""
    try:
        question = input("  Kérdésed a robotnak: ").strip()
    except (KeyboardInterrupt, EOFError):
        print()
        return
    if not question:
        return
    reply = conn.ask_gemini(question)
    if reply:
        conn.say(reply)


def say_custom_text_action(conn):
    """Amit beírsz, azt a robot SZÓ SZERINT felolvassa -- nincs Gemini, nincs
    feldolgozás, ellentétben az ask_question_action-nel."""
    try:
        text = input("  Mit mondjon a robot: ").strip()
    except (KeyboardInterrupt, EOFError):
        print()
        return
    if not text:
        return
    conn.say(text)


def build_menu(arm):
    """(sorszám, címke, futtatandó függvény(conn)) hármasok listája."""
    items = [
        ("1", "Kézfogás", lambda conn: make_handshake(conn, arm).run()),
        ("2", "Felső pacsi", lambda conn: make_high_five(conn, arm, "top").run()),
        ("3", "Alsó pacsi", lambda conn: make_high_five(conn, arm, "bottom").run()),
    ]
    next_key = 4
    for label, action_name, needs_release in BUILTIN_GESTURES:
        items.append((str(next_key), label,
                      lambda conn, a=action_name, r=needs_release: conn.run_builtin_gesture(a, r)))
        next_key += 1
    for phrase in PHRASES:
        items.append((str(next_key), f'Mondat: "{phrase}"', lambda conn, p=phrase: conn.say(p)))
        next_key += 1
    items.append((str(next_key), "Kérdezz a robottól (Gemini)", ask_question_action))
    next_key += 1
    items.append((str(next_key), "Mondj be egy szöveget", say_custom_text_action))
    next_key += 1
    return items


def print_menu(menu_items):
    print("\n=== G1 gyerek-demo menü ===")
    for key, label, _ in menu_items:
        print(f"  {key}) {label}")
    print("  0) Kilépés")


def main():
    parser = argparse.ArgumentParser(description="G1 gyerek-bemutató menü -- kézfogás/pacsi/mondatok gyors váltogatása")
    parser.add_argument("net", help="hálózati interfész, pl. en6")
    parser.add_argument("--arm", choices=["left", "right"], default="right",
                        help="melyik kart használja minden karmozdulat (alap: right)")
    args = parser.parse_args()

    print("FIGYELEM: győződj meg róla, hogy nincs akadály a kar körül -- ez low-level kar-vezérlés.")
    print("Ez a figyelmeztetés csak most jön -- a menüben utána gyorsan lehet váltani.")
    input("Nyomj Entert a folytatáshoz...")

    conn = RobotConnection(args.net)
    conn.wait_for_state()

    menu_items = build_menu(args.arm)

    print("\nKapcsolat kész. Egy karmozdulat közben Ctrl+C csak azt az akciót "
          "szakítja meg, nem lép ki a menüből.")

    while True:
        print_menu(menu_items)
        try:
            choice = input("Válassz (szám + Enter): ").strip()
        except KeyboardInterrupt:
            print("\nViszlát!")
            return
        except EOFError:
            print("\nViszlát!")
            return

        if choice in ("0", "q", "Q"):
            print("Viszlát!")
            return

        match = next((item for item in menu_items if item[0] == choice), None)
        if match is None:
            print("Ismeretlen menüpont, próbáld újra.")
            continue

        _, label, action = match
        print(f"\n>>> {label}")
        try:
            action(conn)
        except KeyboardInterrupt:
            print("\n  Megszakítva.")
        except Exception:
            print("  Hiba történt ebben az akcióban -- a menü folytatódik:")
            traceback.print_exc()


if __name__ == "__main__":
    main()
