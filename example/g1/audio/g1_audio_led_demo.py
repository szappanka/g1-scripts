"""
Egyszerű, látványos G1 demó: a robot beszél (TTS) és közben színátmenetes
LED-animációt játszik (szivárvány-pásztázás + "lélegző" effekt).

Csak a hangmodult (voice service) és a fej-LED-et használja, a robot NEM mozdul,
így biztonságos első kísérletnek.

Használat:
    python3 g1_audio_led_demo.py <networkInterface> [--text "..."] [--speaker 0] [--volume 85]

Pl.:
    python3 g1_audio_led_demo.py enp2s0
    python3 g1_audio_led_demo.py enp2s0 --text "Hello, I am a Unitree G1 robot."

A <networkInterface> annak a hálózati interfésznek a neve, amelyre a robot
csatlakozik (pl. enp2s0). Listázás: `ip addr` vagy `ifconfig`.

Kilépés: Ctrl+C  (a LED kikapcsol kilépéskor).
"""

import sys
import time
import math
import argparse

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.audio.g1_audio_client import AudioClient


def hsv_to_rgb(h, s, v):
    """HSV -> (R, G, B), mindegyik 0..255. h, s, v a [0,1] tartományban."""
    i = int(h * 6.0)
    f = h * 6.0 - i
    p = v * (1.0 - s)
    q = v * (1.0 - f * s)
    t = v * (1.0 - (1.0 - f) * s)
    i = i % 6
    r, g, b = [
        (v, t, p),
        (q, v, p),
        (p, v, t),
        (p, q, v),
        (t, p, v),
        (v, p, q),
    ][i]
    return int(r * 255), int(g * 255), int(b * 255)


def rainbow_breathing(audio_client, duration_s, fps=20):
    """duration_s másodpercig szivárvány színátmenet 'lélegző' fényerővel."""
    period = 1.0 / fps
    steps = int(duration_s * fps)
    for n in range(steps):
        t = n / fps
        hue = (t * 0.15) % 1.0                 # lassan körbeforgó színárnyalat
        bright = 0.5 + 0.5 * math.sin(t * 2.0) # 0..1 közötti "lélegzés"
        r, g, b = hsv_to_rgb(hue, 1.0, bright)
        audio_client.LedControl(r, g, b)
        time.sleep(period)


def say(audio_client, text, speaker_id):
    """Egy mondat felmondása. Kézzel lépteti a tts_index-et (SDK-quirk miatt)."""
    audio_client.tts_index += 1
    code = audio_client.TtsMaker(text, speaker_id)
    if code != 0:
        print(f"[figyelem] TtsMaker hibakód: {code}  (szöveg: {text!r})")
    return code


def main():
    parser = argparse.ArgumentParser(description="G1 hang + LED demó")
    parser.add_argument("net", help="hálózati interfész, pl. enp2s0")
    parser.add_argument("--text", default="Hello! I am a Unitree G1 robot. The audio and LED demo is running.",
                        help="felmondandó szöveg")
    parser.add_argument("--speaker", type=int, default=1,
                        help="hang azonosító: 0=kínai, 1-5=angol hangok (alap: 1)")
    parser.add_argument("--volume", type=int, default=85, help="hangerő 0-100 (alap: 85)")
    args = parser.parse_args()

    # DDS inicializálás a megadott interfészen
    ChannelFactoryInitialize(0, args.net)

    audio_client = AudioClient()
    audio_client.SetTimeout(10.0)
    audio_client.Init()

    # Hangerő beállítása + visszaolvasás
    audio_client.SetVolume(args.volume)
    code, vol = audio_client.GetVolume()
    print(f"Beállított hangerő: {vol}")

    try:
        # 1) Köszönés + közben szivárvány-animáció
        say(audio_client, args.text, args.speaker)
        rainbow_breathing(audio_client, duration_s=6.0)

        # 2) Néhány tiszta alapszín bemutatása
        for name, (r, g, b) in [("piros", (255, 0, 0)),
                                ("zold", (0, 255, 0)),
                                ("kek", (0, 0, 255))]:
            print(f"LED: {name}")
            audio_client.LedControl(r, g, b)
            time.sleep(1.0)

        # 3) Záró mondat + még egy kis fényjáték
        say(audio_client, "Demo finished. Thank you!", args.speaker)
        rainbow_breathing(audio_client, duration_s=4.0)

    except KeyboardInterrupt:
        print("\nMegszakítva.")
    finally:
        # LED kikapcsolása kilépéskor
        audio_client.LedControl(0, 0, 0)
        print("Kész. LED kikapcsolva.")


if __name__ == "__main__":
    main()
