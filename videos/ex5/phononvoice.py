"""A manimgx voice that speaks with Phonon TTS (pip package `ptts`), offline, on the CPU.

Uses the default English model (Phonon 31m, Apache 2.0) and its default voice.
The model is loaded once per render; manimgx keeps every spoken line in voice/,
so each line is synthesised only once.

Setup (into manimgx's own environment):
    uv pip install ptts --python <uv tools dir>/manimgx/Scripts/python.exe
The model (~65 MB) downloads from Hugging Face on first use; set HF_HOME to choose where it goes.
"""
import os
import tempfile
from pathlib import Path

import manimgx as m

if os.path.isdir(r"E:\tools\hf"):  # where the model was downloaded on the original machine
    os.environ.setdefault("HF_HOME", r"E:\tools\hf")

_tts = None


def _model():
    global _tts
    if _tts is None:
        import ptts
        _tts = ptts.TTS(lang="en")
    return _tts


def phonon_voice(text: str) -> m.Speech:
    """Speak `text` with Phonon's default English voice and return it as manimgx Speech."""
    fd, name = tempfile.mkstemp(prefix="phonon_", suffix=".wav")
    os.close(fd)
    wav = Path(name)
    try:
        _model().save(str(wav), text)
        data = wav.read_bytes()
    finally:
        wav.unlink(missing_ok=True)
    return m.Speech(data, text=text)
