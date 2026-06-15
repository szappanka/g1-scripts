"""
G1 hangasszisztens (gépelt MVP, Gemini) — te gépelsz, a Gemini válaszol,
a G1 hangosan elmondja a választ. A robot MAGA találja ki a választ.

Lánc:  te (gépelt szöveg) -> Gemini (gemini-2.5-flash) -> G1 TtsMaker -> a robot beszél

Telepítés:
    pip install google-genai

Ingyenes API-kulcs az aistudio.google.com oldalon ("Get API key"), majd:
    export GEMINI_API_KEY=ide-a-kulcsod

Használat:
    python3 g1_chat_gemini.py en8 [--speaker 1] [--volume 90] [--model gemini-2.5-flash]

Írj egy üzenetet és nyomj Entert; a robot angolul válaszol hangosan.
Kilépés: 'quit' / 'exit' vagy Ctrl+C.
"""

import os
import sys
import argparse

from google import genai
from google.genai import types

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.audio.g1_audio_client import AudioClient


# A robot "személyisége" + a TTS-hez szabott stílus.
SYSTEM_PROMPT = (
    "You are the voice of a Unitree G1 humanoid robot. You speak OUT LOUD through a "
    "text-to-speech engine to a person standing in front of you. "
    "Reply ONLY with what you would say aloud in English — no preamble, no stage directions, "
    "no markdown, no emoji, no bullet lists, no headings. "
    "Keep replies short and conversational: usually one or two sentences, rarely more. "
    "You are friendly, curious and a little playful, but never long-winded. "
    "If you don't know something, say so briefly."
)


def load_api_key():
    """Kulcs forrása: 1) GEMINI_API_KEY / GOOGLE_API_KEY env, 2) gitignore-olt
    gemini_key.txt a script mellett. Így nem kerül a git-be."""
    key = os.environ.get("GEMINI_API_KEY") or os.environ.get("GOOGLE_API_KEY")
    if key:
        return key.strip()
    keyfile = os.path.join(os.path.dirname(os.path.abspath(__file__)), "gemini_key.txt")
    if os.path.exists(keyfile):
        with open(keyfile) as f:
            return f.read().strip()
    return None


def main():
    parser = argparse.ArgumentParser(description="G1 + Gemini beszélgető (gépelt)")
    parser.add_argument("net", help="hálózati interfész, pl. en8")
    parser.add_argument("--speaker", type=int, default=1,
                        help="hang: 0=kínai, 1-5=angol (alap: 1)")
    parser.add_argument("--volume", type=int, default=90, help="hangerő 0-100 (alap: 90)")
    parser.add_argument("--model", default="gemini-2.5-flash",
                        help="Gemini modell (alap: gemini-2.5-flash, ingyenes kereten)")
    args = parser.parse_args()

    api_key = load_api_key()
    if not api_key:
        print("Hiba: nincs Gemini kulcs.\n"
              "Tedd a kulcsot a gemini_key.txt fájlba a script mellé,\n"
              "vagy állítsd be: export GEMINI_API_KEY=ide-a-kulcsod")
        sys.exit(1)

    # --- Robot hang init ---
    ChannelFactoryInitialize(0, args.net)
    audio = AudioClient()
    audio.SetTimeout(10.0)
    audio.Init()
    audio.SetVolume(args.volume)

    # --- Gemini chat init (a chat objektum magától viszi a beszélgetés-előzményt) ---
    client = genai.Client(api_key=api_key)
    chat = client.chats.create(
        model=args.model,
        config=types.GenerateContentConfig(
            system_instruction=SYSTEM_PROMPT,
            max_output_tokens=400,
            # 'thinking' kikapcsolása a gyorsabb válaszért (2.5 Flash-en alapból bekapcsolt)
            thinking_config=types.ThinkingConfig(thinking_budget=0),
        ),
    )

    print("Beszélgetés a G1-gyel (Gemini). Írj valamit. Kilépés: 'quit' vagy Ctrl+C.\n")
    try:
        while True:
            user = input("Te: ").strip()
            if user.lower() in {"quit", "exit", "kilep", "kilép"}:
                break
            if not user:
                continue

            try:
                resp = chat.send_message(user)
                reply = (resp.text or "").strip()
            except Exception as e:
                print(f"[Gemini hiba] {e}")
                continue

            if not reply:
                reply = "Sorry, I didn't catch that."

            print(f"G1: {reply}\n")

            # A robot felmondja. (Kézzel léptetjük a tts_index-et az SDK-quirk miatt.)
            audio.tts_index += 1
            audio.TtsMaker(reply, args.speaker)

    except KeyboardInterrupt:
        print("\nViszlát!")
    finally:
        audio.LedControl(0, 0, 0)


if __name__ == "__main__":
    main()
