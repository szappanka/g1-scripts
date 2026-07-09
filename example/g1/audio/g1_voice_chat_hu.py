"""
G1 hangos beszélgetés magyarul -- te BESZÉLSZ (nem gépelsz), a robot hallja
(Whisper, helyi, ingyenes STT), a Gemini kitalálja a választ, a robot pedig
hangosan válaszol (edge-tts, ingyenes TTS). Kétirányú hangos beszélgetés.

Lánc:  mikrofon -> Whisper (helyi, magyar) -> Gemini (magyarul válaszol)
       -> edge-tts+miniaudio -> a robot beszél

Ez a g1_chat_gemini.py "gépelt MVP"-jének hangos verziója -- a Gemini-résztvevő
és a magyar TTS ugyanaz (importálva onnan / a hu_tts modulból), csak a gépelt
bemenetet váltja ki mikrofonos felvétel + helyi Whisper felismerés.

Telepítés:
    pip install google-genai edge-tts miniaudio openai-whisper sounddevice

Ingyenes Gemini API-kulcs az aistudio.google.com oldalon ("Get API key"), majd:
    export GEMINI_API_KEY=ide-a-kulcsod
    (vagy: echo "a-kulcsod" > gemini_key.txt ebben a mappában)

Használat:
    python3 g1_voice_chat_hu.py en6
    python3 g1_voice_chat_hu.py en6 --whisper-model small --voice hu-HU-TamasNeural

Menete: a robot válasza után rögtön hallgat -- ha beszélni kezdesz, rögzíti,
és ha 5 mp-ig csend van (a beszéd után), leállítja a felvételt és elküldi.
Nincs Enter-nyomás, nincs kézi kalibrálás: a háttérzajt minden körben
automatikusan méri egy röpke pillanat alatt.
Kilépés: Ctrl+C.
"""

import sys
import time
import argparse

import numpy as np
import sounddevice as sd
import whisper
from google import genai
from google.genai import types

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.audio.g1_audio_client import AudioClient

from hu_tts import synthesize_pcm, DEFAULT_VOICE
from wav import play_pcm_stream
from g1_chat_gemini import SYSTEM_PROMPT_HU, load_api_key

SAMPLE_RATE = 16000
BLOCK_SECONDS = 0.1
NOISE_CALIBRATION_SECONDS = 0.3
SILENCE_TIMEOUT = 5.0
MAX_RECORD_SECONDS = 30.0


def record_until_silence(device=None, silence_timeout=SILENCE_TIMEOUT, max_seconds=MAX_RECORD_SECONDS):
    """Automatikusan hallgatja a mikrofont: nincs Enter, nincs kézi kalibrálás.

    Elsőként pár tized másodpercig méri a háttérzajt (ez nem egy "állj
    csendben" lépés a felhasználónak, csak a küszöb automatikus beállítása).
    Onnantól a küszöb fölötti hangerőt tekinti beszédnek, és ha a beszéd
    kezdete után `silence_timeout` másodpercig csend van, lezárja a felvételt.
    16kHz mono float32 tömbként adja vissza (üres tömb, ha nem történt beszéd).
    """
    block_size = int(SAMPLE_RATE * BLOCK_SECONDS)
    frames = []
    noise_samples = []
    noise_floor = None
    speaking = False
    silence_since = None
    started_at = time.monotonic()

    print("Hallgatom... (ha végeztél a beszéddel, csak hagyd abba -- 5 mp csend után elküldöm)")
    with sd.InputStream(samplerate=SAMPLE_RATE, channels=1, dtype="float32",
                        device=device) as stream:
        while True:
            block, _ = stream.read(block_size)
            block = block.flatten()
            rms = float(np.sqrt(np.mean(np.square(block)))) if block.size else 0.0

            if noise_floor is None:
                noise_samples.append(rms)
                if len(noise_samples) * BLOCK_SECONDS >= NOISE_CALIBRATION_SECONDS:
                    noise_floor = max(float(np.mean(noise_samples)), 0.001)
                continue

            threshold = noise_floor * 3.0 + 0.01
            frames.append(block)

            if rms > threshold:
                if not speaking:
                    print("(beszéd érzékelve...)")
                speaking = True
                silence_since = None
            elif speaking:
                if silence_since is None:
                    silence_since = time.monotonic()
                elif time.monotonic() - silence_since >= silence_timeout:
                    break

            if time.monotonic() - started_at >= max_seconds:
                print("(elért a max. felvételi idő)")
                break

    if not speaking or not frames:
        return np.zeros(0, dtype=np.float32)
    audio = np.concatenate(frames, axis=0).flatten()

    rms = float(np.sqrt(np.mean(np.square(audio)))) if audio.size else 0.0
    print(f"(felvétel szintje: {rms:.4f} -- ha ez ~0, a mikrofon nem vesz fel semmit)")
    return audio


def main():
    parser = argparse.ArgumentParser(description="G1 hangos beszélgetés magyarul (Whisper + Gemini + edge-tts)")
    parser.add_argument("net", nargs="?", help="hálózati interfész, pl. en6")
    parser.add_argument("--device", default=None,
                        help="mikrofon eszköz neve vagy indexe (alap: rendszer alapértelmezett). "
                             "Listázás: python3 g1_voice_chat_hu.py --list-devices")
    parser.add_argument("--list-devices", action="store_true",
                        help="kilistázza az elérhető hangeszközöket, majd kilép")
    parser.add_argument("--whisper-model", default="small",
                        help="Whisper modell mérete: tiny/base/small/medium/large (alap: small)")
    parser.add_argument("--gemini-model", default="gemini-2.5-flash",
                        help="Gemini modell (alap: gemini-2.5-flash, ingyenes kereten)")
    parser.add_argument("--voice", default=DEFAULT_VOICE,
                        help=f"edge-tts hang (alap: {DEFAULT_VOICE}, másik opció: hu-HU-TamasNeural)")
    parser.add_argument("--volume", type=int, default=90, help="hangerő 0-100 (alap: 90)")
    parser.add_argument("--silence-timeout", type=float, default=SILENCE_TIMEOUT,
                        help=f"hány mp csend után zárja le a felvételt beszéd után (alap: {SILENCE_TIMEOUT})")
    args = parser.parse_args()

    if args.list_devices:
        print(sd.query_devices())
        return

    if not args.net:
        parser.error("a 'net' argumentum kötelező, ha nem --list-devices-t futtatsz")

    # a --device lehet index ("3") vagy névrészlet ("MacBook") -- a sounddevice
    # csak akkor keresi indexként, ha valódi int-et kap, nem szám-stringet
    if args.device is not None and args.device.isdigit():
        args.device = int(args.device)

    api_key = load_api_key()
    if not api_key:
        print("Hiba: nincs Gemini kulcs.\n"
              "Tedd a kulcsot a gemini_key.txt fájlba a script mellé,\n"
              "vagy állítsd be: export GEMINI_API_KEY=ide-a-kulcsod")
        sys.exit(1)

    print(f"Whisper modell betöltése ({args.whisper_model})...")
    whisper_model = whisper.load_model(args.whisper_model)

    # --- Robot hang init ---
    ChannelFactoryInitialize(0, args.net)
    audio = AudioClient()
    audio.SetTimeout(10.0)
    audio.Init()
    audio.SetVolume(args.volume)

    # --- Gemini chat init (a chat objektum magától viszi a beszélgetés-előzményt) ---
    client = genai.Client(api_key=api_key)
    chat = client.chats.create(
        model=args.gemini_model,
        config=types.GenerateContentConfig(
            system_instruction=SYSTEM_PROMPT_HU,
            max_output_tokens=400,
            thinking_config=types.ThinkingConfig(thinking_budget=0),
        ),
    )

    print("Hangos beszélgetés a G1-gyel (magyarul). Kilépés: Ctrl+C.\n")
    try:
        while True:
            recording = record_until_silence(device=args.device, silence_timeout=args.silence_timeout)
            if recording.size == 0:
                continue

            print("Felismerés (Whisper)...")
            result = whisper_model.transcribe(recording, language="hu")
            user_text = result["text"].strip()
            if not user_text:
                print("(nem értettem semmit, próbáld újra)\n")
                continue
            print(f"Te: {user_text}")

            if user_text.lower() in {"kilep", "kilép", "kilépés", "viszlát"}:
                break

            try:
                resp = chat.send_message(user_text)
                reply = (resp.text or "").strip()
            except Exception as e:
                print(f"[Gemini hiba] {e}")
                continue

            if not reply:
                reply = "Elnézést, ezt nem értettem."

            print(f"G1: {reply}\n")

            pcm_bytes, _sample_rate = synthesize_pcm(reply, voice=args.voice)
            play_pcm_stream(audio, list(pcm_bytes), "voice_chat_reply")
            audio.PlayStop("voice_chat_reply")

    except KeyboardInterrupt:
        print("\nViszlát!")
    finally:
        audio.LedControl(0, 0, 0)


if __name__ == "__main__":
    main()
