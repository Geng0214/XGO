"""edge-tts语音合成 + 播放"""

import edge_tts
import asyncio
import subprocess
import os
import hashlib
from config import TTS_VOICE, TTS_CACHE_DIR


class Speaker:
    def __init__(self, voice=TTS_VOICE, cache_dir=TTS_CACHE_DIR):
        self.voice = voice
        self.cache_dir = cache_dir
        os.makedirs(cache_dir, exist_ok=True)

    def speak(self, text):
        """合成并播放语音"""
        text_hash = hashlib.md5(text.encode()).hexdigest()[:8]
        filename = os.path.join(self.cache_dir, f"speech_{text_hash}.mp3")

        if not os.path.exists(filename):
            asyncio.run(self._generate(text, filename))

        try:
            subprocess.run(["aplay", filename],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=10)
        except FileNotFoundError:
            print(f"[TTS] aplay not found, skipping playback: {text}")
        except subprocess.TimeoutExpired:
            print(f"[TTS] playback timeout: {text}")

    async def _generate(self, text, filename):
        try:
            communicate = edge_tts.Communicate(text, self.voice)
            await communicate.save(filename)
        except Exception as e:
            print(f"[TTS] generate failed: {e}")
