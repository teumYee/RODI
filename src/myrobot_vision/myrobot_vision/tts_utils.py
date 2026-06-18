import os
import subprocess
import tempfile

try:
    from gtts import gTTS
    GTTS_OK = True
except ImportError:
    GTTS_OK = False


def speak_ko(logger, text: str, timeout: float = 30.0) -> bool:
    logger.info(f'[TTS] {text}')
    if not GTTS_OK:
        logger.warn(f'[TTS 미설치] {text}')
        return False

    path = None
    try:
        tts = gTTS(text=text, lang='ko')
        with tempfile.NamedTemporaryFile(suffix='.mp3', delete=False) as f:
            path = f.name
        tts.save(path)

        for cmd in (
            ['mpg123', '-q', path],
            ['ffplay', '-nodisp', '-autoexit', '-loglevel', 'quiet', path],
        ):
            try:
                if subprocess.run(
                    cmd,
                    timeout=timeout,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                ).returncode == 0:
                    return True
            except (FileNotFoundError, subprocess.TimeoutExpired):
                continue

        logger.error('오디오 재생 실패: mpg123/ffplay 실행 실패')
        return False
    except Exception as e:
        logger.error(f'TTS 오류: {e}')
        return False
    finally:
        if path and os.path.exists(path):
            os.unlink(path)
