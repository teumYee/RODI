"""
tts_node.py

/child_alert (std_msgs/String, JSON)를 수신하면
gTTS로 한국어 음성 파일(.mp3)을 생성해 mpg123으로 재생.

mpg123이 없을 경우 aplay + sox로 대체 재생 시도.
"""

import json
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .tts_utils import speak_ko


# 경보 메시지 목록 (순서대로 재생)
ALERT_MESSAGES = [
    '경고! 보호자 없는 어린이가 감지되었습니다.',
    '어린이 보호 구역입니다. 즉시 확인해 주세요.',
]


class TTSNode(Node):
    def __init__(self):
        super().__init__('tts_speaker')

        self._lock = threading.Lock()
        self._speaking = False
        self._msg_index = 0

        self.create_subscription(String, '/child_alert', self._alert_cb, 10)
        self.get_logger().info('TTS 노드 시작. /child_alert 수신 대기 중')

    def _alert_cb(self, msg: String):
        data = json.loads(msg.data)
        if not data.get('alert', False):
            return

        with self._lock:
            if self._speaking:
                return
            self._speaking = True

        text = ALERT_MESSAGES[self._msg_index % len(ALERT_MESSAGES)]
        self._msg_index += 1

        t = threading.Thread(target=self._speak, args=(text,), daemon=True)
        t.start()
        self.get_logger().warn(f'TTS 재생: "{text}"')

    def _speak(self, text: str):
        try:
            speak_ko(self.get_logger(), text)
        except Exception as e:
            self.get_logger().error(f'TTS 오류: {e}')
        finally:
            with self._lock:
                self._speaking = False


def main(args=None):
    rclpy.init(args=args)
    node = TTSNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
