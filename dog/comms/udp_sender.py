"""UDP双机通信 - 发送巡检结果给狗2"""

import socket
import json
from config import UDP_PORT, UDP_BROADCAST_IP


class DogCommunicator:
    def __init__(self, port=UDP_PORT):
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    def send_inspection_result(self, letter, gauge_status, digit_status):
        """发送单个巡检结果"""
        data = json.dumps({
            "type": "inspection",
            "letter": letter,
            "gauge": gauge_status,
            "digit": digit_status,
        }).encode()
        self.sock.sendto(data, (UDP_BROADCAST_IP, self.port))
        print(f"[UDP] Sent: {letter} gauge={gauge_status} digit={digit_status}")

    def send_all_done(self, results):
        """发送所有巡检结果"""
        data = json.dumps({
            "type": "inspection_done",
            "results": results,
        }).encode()
        self.sock.sendto(data, (UDP_BROADCAST_IP, self.port))
        print(f"[UDP] Sent all results: {results}")

    def close(self):
        self.sock.close()
