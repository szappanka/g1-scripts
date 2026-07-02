"""
G1 magyar beszéd -- bármilyen magyar szöveget kimond a robot hangszóróján.

A beépített TtsMaker nem tud magyarul (csak kínai/angol), ezért ez a szkript
a hu_tts modullal (edge-tts + miniaudio) szintetizálja a hangot, majd a
meglévő PlayStream mechanizmuson keresztül lejátssza.

Telepítés:
    pip install edge-tts miniaudio

Használat:
    python3 g1_speak_hu.py <networkInterface> "Szia, én egy G1 robot vagyok!"
    python3 g1_speak_hu.py enp2s0 "Örülök, hogy találkoztunk!" --voice hu-HU-TamasNeural
"""

import sys
import argparse

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.audio.g1_audio_client import AudioClient

from hu_tts import synthesize_pcm, DEFAULT_VOICE
from wav import play_pcm_stream


def speak(audio_client, text, voice=DEFAULT_VOICE):
    print(f"Szintetizálás ({voice})...")
    pcm_bytes, sample_rate = synthesize_pcm(text, voice=voice)
    print(f"Lejátszás ({len(pcm_bytes)} byte, {sample_rate} Hz)...")
    play_pcm_stream(audio_client, list(pcm_bytes), "hu_speak")
    audio_client.PlayStop("hu_speak")


def main():
    parser = argparse.ArgumentParser(description="G1 magyar beszéd (edge-tts)")
    parser.add_argument("net", help="hálózati interfész, pl. enp2s0")
    parser.add_argument("text", help="a kimondandó magyar szöveg")
    parser.add_argument("--voice", default=DEFAULT_VOICE,
                        help="edge-tts hang (alap: hu-HU-NoemiNeural, másik opció: hu-HU-TamasNeural)")
    args = parser.parse_args()

    ChannelFactoryInitialize(0, args.net)
    audio_client = AudioClient()
    audio_client.SetTimeout(10.0)
    audio_client.Init()

    speak(audio_client, args.text, voice=args.voice)


if __name__ == "__main__":
    main()
