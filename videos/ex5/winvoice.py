"""A manimgx voice that speaks with the voice built into Windows (SAPI), offline and free.

manimgx accepts any function from text to Speech as a voice. This one asks PowerShell's
System.Speech to write a WAV file and hands it to manimgx, which keeps it in voice/ beside
the scene, so each line is synthesised once.
"""
import os
import subprocess
import tempfile
from pathlib import Path

import manimgx as m

VOICE_NAME = "Microsoft David Desktop"  # en-US; "Microsoft Zira Desktop" is the other one
RATE = 0  # -10 (slow) .. 10 (fast)

_PS = r"""
Add-Type -AssemblyName System.Speech
$s = New-Object System.Speech.Synthesis.SpeechSynthesizer
$s.SelectVoice('{voice}')
$s.Rate = {rate}
$s.SetOutputToWaveFile('{wav}')
$s.Speak([IO.File]::ReadAllText('{txt}', [Text.Encoding]::UTF8))
$s.Dispose()
"""


def windows_voice(text: str) -> m.Speech:
    """Speak `text` with the Windows voice and return it as manimgx Speech."""
    tmp = Path(tempfile.mkdtemp(prefix="winvoice_"))
    txt, wav = tmp / "line.txt", tmp / "line.wav"
    txt.write_text(text, encoding="utf-8")
    script = _PS.format(voice=VOICE_NAME, rate=RATE, wav=wav, txt=txt)
    subprocess.run(["powershell.exe", "-NoProfile", "-NonInteractive", "-Command", script],
                   check=True, capture_output=True)
    data = wav.read_bytes()
    for f in (txt, wav):
        f.unlink(missing_ok=True)
    os.rmdir(tmp)
    return m.Speech(data, text=text)
