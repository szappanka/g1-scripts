"""
G1 seta demo -- a robot BEEPITETT egyensulyozo rendszerevel jar, a LocoClient
("sport" szolgaltatas) magas szintu API-jan keresztul.

Ellentetben a low_level/g1_low_level_example.py-vel (ahol MI adjuk minden
izulet pozicio-parancsat 500 Hz-en, egyensulyozas nelkul), itt csak sebesseg-
parancsokat (vx/vy/vyaw) kuldunk -- a labmozgast es az egyensulyt a robot
sajat firmware-je (a beepitett "sport" vezerlo) vegzi. Ugyanez a mechanizmus
all a g1_loco_client_example.py "move forward/lateral/rotate" opcioi mogott is.

Menete:
  1) LocoClient inicializalasa
  2) aktualis FSM id + balance mode lekerdezese es kiirasa (a G1 LocoClient
     osztaly csak a Set*-eket adja kesz metodusban, a Get*-eket a regisztralt
     API ID-n keresztul, kozvetlenul kell hivni -- ugyanugy, ahogy pl. a H2
     LocoClient teszi belul)
  3) Start() -- biztositja, hogy a robot "fo uzemi vezerles" (FSM 500) modban
     legyen, mert csak ebben fogadja el a sebesseg-parancsokat a beepitett
     egyensulyozoval egyutt
  4) Move(vx, vy, vyaw) -- elindul a seta, a robot maga tartja az egyensulyt
  5) a megadott ido utan (vagy Ctrl+C-re) StopMove() -- megallas

FONTOS -- ELSO FUTTATAS ELOTT:
- A robotnak ALLNIA kell a talajon (nem felfuggesztve!), mert a beepitett
  egyensulyozo valos talajerintkezesre es sajat IMU/labadat-visszacsatolasra
  epit -- ellentetben a low-level peldaval, ami allvanyon is biztonsagos.
- Legyen korulotte szabad terulet -- seta kozben tenylegesen elmozdul.
- Veszhelyzetben Ctrl+C -- ez megprobalja leallitani a setat (StopMove),
  de fizikai veszleallitot/tavirandzsalot is tarts keznel.

Hasznalat:
    python3 g1_loco_walk_example.py en6
    python3 g1_loco_walk_example.py en6 --vx 0.3 --duration 3
    python3 g1_loco_walk_example.py en6 --vx 0 --vyaw 0.3 --duration 4   # helyben forgas
    python3 g1_loco_walk_example.py en6 --check-only                    # csak lekerdezes, seta nelkul
"""

import time
import json
import argparse

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
from unitree_sdk2py.g1.loco.g1_loco_api import (
    ROBOT_API_ID_LOCO_GET_FSM_ID,
    ROBOT_API_ID_LOCO_GET_BALANCE_MODE,
)


def get_fsm_id(sport_client: LocoClient):
    code, data = sport_client._Call(ROBOT_API_ID_LOCO_GET_FSM_ID, "{}")
    if code != 0:
        return code, None
    return code, json.loads(data).get("data")


def get_balance_mode(sport_client: LocoClient):
    code, data = sport_client._Call(ROBOT_API_ID_LOCO_GET_BALANCE_MODE, "{}")
    if code != 0:
        return code, None
    return code, json.loads(data).get("data")


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("network_interface", help="pl. en6 (Mac) vagy enp2s0 (Linux)")
    parser.add_argument("--vx", type=float, default=0.3, help="elore/hatra sebesseg [m/s]")
    parser.add_argument("--vy", type=float, default=0.0, help="oldaliranyu sebesseg [m/s]")
    parser.add_argument("--vyaw", type=float, default=0.0, help="forgasi sebesseg [rad/s]")
    parser.add_argument("--duration", type=float, default=3.0, help="seta idotartama [s]")
    parser.add_argument(
        "--check-only", action="store_true",
        help="csak FSM id + balance mode kiirasa, seta inditasa nelkul",
    )
    return parser.parse_args()


def main():
    args = parse_args()

    print("WARNING: Please ensure there are no obstacles around the robot while running this example.")
    input("Press Enter to continue...")

    ChannelFactoryInitialize(0, args.network_interface)

    sport_client = LocoClient()
    sport_client.SetTimeout(10.0)
    sport_client.Init()

    code, fsm_id = get_fsm_id(sport_client)
    print(f"Jelenlegi FSM id: {fsm_id} (code={code})")

    code, balance_mode = get_balance_mode(sport_client)
    print(f"Jelenlegi balance mode: {balance_mode} (code={code})")

    if args.check_only:
        return

    # "Fo uzemi vezerles" mod (FSM 500) -- csak ebben fogadja el a robot a
    # sebesseg-parancsokat a beepitett egyensulyozoval egyutt.
    sport_client.Start()
    time.sleep(1.0)

    try:
        print(f"Seta indul: vx={args.vx} vy={args.vy} vyaw={args.vyaw} ({args.duration}s)")
        sport_client.Move(args.vx, args.vy, args.vyaw, continous_move=True)
        time.sleep(args.duration)
    except KeyboardInterrupt:
        print("\nMegszakitva.")
    finally:
        print("Megallas (StopMove)...")
        sport_client.StopMove()


if __name__ == "__main__":
    main()
