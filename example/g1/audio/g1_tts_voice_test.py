"""
G1 TTS hang/nyelv teszt: végigmegy a megadott speaker_id értékeken, és
mindegyiknél bemond egy rövid angol mondatot, hogy halld, melyik az angol hang.

A nyelvet/hangot a TtsMaker(text, speaker_id) MÁSODIK paramétere (speaker_id) dönti el,
NEM a szöveg. A 0 alapból egy kínai hang.

Használat:
    python3 g1_tts_voice_test.py <networkInterface> [--ids 0 1 2 3 4 5] [--volume 90]

Pl.:
    python3 g1_tts_voice_test.py en8
    python3 g1_tts_voice_test.py en8 --ids 0 1 2 --volume 100

Figyeld a terminált is: minden hang elhangzása ELŐTT kiírja az aktuális speaker_id-t,
így a hallott angol mondathoz hozzá tudod rendelni a számot.
"""

import sys
import time
import argparse

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.audio.g1_audio_client import AudioClient


def main():
    parser = argparse.ArgumentParser(description="G1 TTS hang/nyelv teszt")
    parser.add_argument("net", help="hálózati interfész, pl. en8")
    parser.add_argument("--ids", type=int, nargs="+", default=[0, 1, 2, 3, 4, 5],
                        help="kipróbálandó speaker_id-k (alap: 0 1 2 3 4 5)")
    parser.add_argument("--volume", type=int, default=90, help="hangerő 0-100 (alap: 90)")
    parser.add_argument("--gap", type=float, default=5.0, help="szünet hangonként mp-ben (alap: 5)")
    args = parser.parse_args()

    ChannelFactoryInitialize(0, args.net)

    audio = AudioClient()
    audio.SetTimeout(10.0)
    audio.Init()
    audio.SetVolume(args.volume)

    for sid in args.ids:
        print(f"\n>>> speaker_id = {sid}  (figyeld: ez angolul szól-e)")
        text = f"This is voice number {sid}. Hello, I am a Unitree G one robot."
        audio.tts_index += 1          # SDK-quirk megkerülése (lásd a demó scriptet)
        code = audio.TtsMaker(text, sid)
        if code != 0:
            print(f"    [figyelem] hibakód: {code} (lehet, hogy ez a speaker_id nem létezik)")
        time.sleep(args.gap)

    print("\nKész. Jegyezd meg, melyik speaker_id volt angol — azt használd a demókban.")


if __name__ == "__main__":
    main()
