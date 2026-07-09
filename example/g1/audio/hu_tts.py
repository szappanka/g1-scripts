"""
Magyar szöveg -> beszéd, a G1 hangszóróján keresztül lejátszható PCM-mé alakítva.

A G1 beépített TtsMaker-je csak kínai/angol hangokat tud (ld. g1_tts_voice_test.py),
magyarul nem. Ez a modul helyette a Microsoft Edge felolvasó motorját használja
(edge-tts csomag, nem hivatalos, de ingyenes és nem kell hozzá API-kulcs), majd a
kapott MP3-at a robot hangszórójához szükséges 16 kHz, mono, 16-bit PCM formátumra
alakítja (miniaudio -- nem igényel külön ffmpeg telepítést).

Telepítés:
    pip install edge-tts miniaudio

Magyar hangok: hu-HU-NoemiNeural (nő), hu-HU-TamasNeural (férfi).
"""

import asyncio

import edge_tts
import miniaudio

from wav import write_wave

DEFAULT_VOICE = "hu-HU-TamasNeural"
TARGET_SAMPLE_RATE = 16000  # a robot AudioClient.PlayStream ezt várja


async def _synthesize_mp3_bytes(text, voice):
    chunks = []
    communicate = edge_tts.Communicate(text, voice)
    async for chunk in communicate.stream():
        if chunk["type"] == "audio":
            chunks.append(chunk["data"])
    return b"".join(chunks)


def synthesize_wav(text, wav_path, voice=DEFAULT_VOICE):
    """Magyar (vagy bármilyen, az edge-tts hang nyelvén értelmezhető) szöveget
    szintetizál, és a robot számára megfelelő formátumú WAV fájlba ír."""
    mp3_bytes = asyncio.run(_synthesize_mp3_bytes(text, voice))
    decoded = miniaudio.decode(
        mp3_bytes,
        output_format=miniaudio.SampleFormat.SIGNED16,
        nchannels=1,
        sample_rate=TARGET_SAMPLE_RATE,
    )
    if not write_wave(wav_path, TARGET_SAMPLE_RATE, decoded.samples, num_channels=1):
        raise RuntimeError(f"Nem sikerült kiírni a WAV fájlt: {wav_path}")
    return wav_path


def synthesize_pcm(text, voice=DEFAULT_VOICE):
    """Mint synthesize_wav, de fájl nélkül, közvetlenül a nyers PCM byte-okat adja vissza."""
    mp3_bytes = asyncio.run(_synthesize_mp3_bytes(text, voice))
    decoded = miniaudio.decode(
        mp3_bytes,
        output_format=miniaudio.SampleFormat.SIGNED16,
        nchannels=1,
        sample_rate=TARGET_SAMPLE_RATE,
    )
    return bytes(decoded.samples), TARGET_SAMPLE_RATE
