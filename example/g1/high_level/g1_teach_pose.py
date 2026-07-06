"""
G1 póz-tanító segédeszköz -- az "arm_sdk" (rt/arm_sdk) alacsony szintű kar-
felülírással a megadott kart ENGEDÉKENNYÉ (kézzel szabadon mozgatható) teszi,
és másodpercenként kiírja az aktuális ízületszögeket -- pontosan olyan
formában, ahogy a demo scriptek REACH_POSE / HIGH_FIVE_POSE szótáraiba
bemásolható.

Így nem kell találgatva, --shoulder-pitch / --shoulder-roll / --elbow
kapcsolókkal próbálgatni a jó pózt: idehozod a kart kézzel a kívánt
helyzetbe, megnézed a kiírt sort, és bemásolod a demo scriptbe.

Hogyan engedékeny a kar: minden 20ms-es cikluson a célszög = az ÉPPEN mért
aktuális szög, alacsony kp/kd-vel. Így a kar nem húz vissza semmilyen fix
pozícióhoz -- csak egy kis csillapítást (kd) ad, hogy ne lengjen ki, amikor
mozgatod vagy elengeded.

FONTOS:
- A kar a TEACH fázisban engedékeny, de nem súlytalan -- óvatosan mozgasd,
  ne rántsd meg. Ha túl merevnek/lazának érzed, hangold a --teach-kp /
  --teach-kd kapcsolókkal (alacsonyabb kp = engedékenyebb, magasabb kd =
  simább, kevésbé "élénk" mozgás).
- A MÁSIK kar és a derék -- mint minden arm_sdk demónál -- rögzítve marad a
  kiindulási pózban normál kp/kd-vel, különben a robot előre/oldalra dőlhetne.
- Ctrl+C-re a script kiírja a végső pózt, majd RETRACT-tal visszaviszi a kart
  a kiindulási pózba, végül RELEASE-eli (visszaadja az irányítást a magas
  szintű vezérlőnek). Eddig tartsd készenlétben a vészleállítót is, mint
  minden low-level tesztnél.

Használat:
    python3 g1_teach_pose.py <networkInterface> --arm right
    python3 g1_teach_pose.py en6 --arm left --teach-kp 5 --teach-kd 1.5

Kilépés: Ctrl+C (kiírja a végső pózt, majd visszahúz)
"""

import sys
import time
import argparse
import threading
import traceback

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC


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


class TeachPose:
    STAGE_TEACH = "TEACH"
    STAGE_RETRACT = "RETRACT"
    STAGE_RELEASE = "RELEASE"
    STAGE_DONE = "DONE"

    RETRACT_SECONDS = 2.0
    RELEASE_SECONDS = 1.0
    PRINT_INTERVAL = 0.5

    def __init__(self, arm, teach_kp, teach_kd, kp, kd):
        prefix = "Left" if arm == "left" else "Right"
        other_prefix = "Right" if arm == "left" else "Left"

        self.arm = arm
        self.active_joint_ids = [getattr(G1JointIndex, f"{prefix}{name}") for name in ARM_JOINT_NAMES]
        # A másik kar + a derék -- az arm_sdk a TELJES felsőtestet átveszi a
        # magas szintű vezérlőtől, tehát ha ezeket nem tartjuk explicit
        # pozícióban (kp/kd-vel), tartóerő nélkül maradnak és a robot megdől.
        self.passive_joint_ids = (
            [getattr(G1JointIndex, f"{other_prefix}{name}") for name in ARM_JOINT_NAMES]
            + [G1JointIndex.WaistYaw, G1JointIndex.WaistRoll, G1JointIndex.WaistPitch]
        )
        self.all_joint_ids = self.active_joint_ids + self.passive_joint_ids

        self.teach_kp = teach_kp
        self.teach_kd = teach_kd
        self.control_dt = 0.02
        self.kp = kp
        self.kd = kd

        self.crc = CRC()
        self.low_cmd = unitree_hg_msg_dds__LowCmd_()
        self.low_state = None

        self.stage = None
        self.stage_t0 = None
        self.start_q = None
        self.retract_from_q = None
        self.last_print = None
        self.done = False

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

    def write_joint(self, idx, q, kp, kd):
        self.low_cmd.motor_cmd[idx].tau = 0.0
        self.low_cmd.motor_cmd[idx].q = float(q)
        self.low_cmd.motor_cmd[idx].dq = 0.0
        self.low_cmd.motor_cmd[idx].kp = kp
        self.low_cmd.motor_cmd[idx].kd = kd

    def current_pose_line(self):
        values = [self.low_state.motor_state[idx].q for idx in self.active_joint_ids]
        formatted = ", ".join(f"{v:+.3f}" for v in values)
        return f'    "{self.arm}": [{formatted}],'

    def enter_stage(self, stage):
        self.stage = stage
        self.stage_t0 = time.time()
        if stage == self.STAGE_RETRACT:
            self.retract_from_q = {idx: self.low_state.motor_state[idx].q for idx in self.active_joint_ids}
        print(f"[fázis] {stage}")

    def control_tick(self):
        if self.low_state is None:
            return

        if self.stage is None:
            self.start_q = {idx: self.low_state.motor_state[idx].q for idx in self.all_joint_ids}
            self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q = 1.0  # arm_sdk bekapcsolva
            self.enter_stage(self.STAGE_TEACH)
            print("Mozgasd a kart kézzel a kívánt pózba. A jelenlegi szögek "
                  f"{self.PRINT_INTERVAL:.1f} mp-enként kiíródnak. Ctrl+C: rögzítés + visszahúzás.\n")

        elapsed = time.time() - self.stage_t0

        # A passzív kar + derék mindig a kiindulási pózban rögzítve, normál kp/kd-vel.
        for idx in self.passive_joint_ids:
            self.write_joint(idx, self.start_q[idx], self.kp, self.kd)

        if self.stage == self.STAGE_TEACH:
            # A cél mindig az ÉPPEN mért szög -- ettől nem húz vissza sehova,
            # csak a kd csillapítja a hirtelen mozdulatokat.
            for idx in self.active_joint_ids:
                self.write_joint(idx, self.low_state.motor_state[idx].q, self.teach_kp, self.teach_kd)

            now = time.time()
            if self.last_print is None or now - self.last_print >= self.PRINT_INTERVAL:
                self.last_print = now
                print(self.current_pose_line())

        elif self.stage == self.STAGE_RETRACT:
            ratio = min(elapsed / self.RETRACT_SECONDS, 1.0)
            for idx in self.active_joint_ids:
                q0 = self.retract_from_q[idx]
                target = self.start_q[idx]
                self.write_joint(idx, (1.0 - ratio) * q0 + ratio * target, self.kp, self.kd)
            if ratio >= 1.0:
                self.enter_stage(self.STAGE_RELEASE)

        elif self.stage == self.STAGE_RELEASE:
            for idx in self.active_joint_ids:
                self.write_joint(idx, self.start_q[idx], self.kp, self.kd)
            weight = max(1.0 - elapsed / self.RELEASE_SECONDS, 0.0)
            self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q = weight
            if weight <= 0.0:
                self.stage = self.STAGE_DONE
                self.done = True

        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.publisher.Write(self.low_cmd)


def main():
    parser = argparse.ArgumentParser(description="G1 póz-tanító -- kézzel mozgatható kar, élő szög-kiírással")
    parser.add_argument("net", help="hálózati interfész, pl. en6")
    parser.add_argument("--arm", choices=["left", "right"], default="right",
                        help="melyik kart tegye engedékennyé (alap: right)")
    parser.add_argument("--teach-kp", type=float, default=3.0,
                        help="a tanított kar merevsége -- alacsonyabb = könnyebb kézzel mozgatni (alap: 3.0)")
    parser.add_argument("--teach-kd", type=float, default=1.0,
                        help="a tanított kar csillapítása -- magasabb = simább, kevésbé élénk mozgás (alap: 1.0)")
    parser.add_argument("--kp", type=float, default=60.0,
                        help="a MÁSIK kar + derék, illetve a RETRACT/RELEASE fázis merevsége (alap: 60.0)")
    parser.add_argument("--kd", type=float, default=1.5,
                        help="a MÁSIK kar + derék, illetve a RETRACT/RELEASE fázis csillapítása (alap: 1.5)")
    args = parser.parse_args()

    print("FIGYELEM: győződj meg róla, hogy nincs akadály a kar körül -- ez low-level kar-vezérlés.")
    print("FIGYELEM: a kar a TEACH fázisban engedékeny lesz -- óvatosan mozgasd, ne rántsd meg.")
    input("Nyomj Entert a folytatáshoz...")

    ChannelFactoryInitialize(0, args.net)

    teach = TeachPose(arm=args.arm, teach_kp=args.teach_kp, teach_kd=args.teach_kd, kp=args.kp, kd=args.kd)
    teach.init_channels()
    teach.wait_for_state()

    thread = PeriodicThread(interval=teach.control_dt, target=teach.control_tick)
    thread.Start()

    try:
        while not teach.done:
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nVégső póz:")
        print(teach.current_pose_line())
        print("\nEzt másold be a demo script pózt tároló szótárába (pl. HIGH_FIVE_POSE).\n")
        print("Visszahúzás...")
        teach.enter_stage(TeachPose.STAGE_RETRACT)
        while not teach.done:
            time.sleep(0.1)

    print("Kész.")


if __name__ == "__main__":
    main()
