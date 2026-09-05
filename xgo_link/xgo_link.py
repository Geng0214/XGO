"""两个 XGO 之间的 UDP 通信模块（同一局域网）

协议：JSON over UDP，默认端口 6001。
消息格式：{"type": "letter", "letter": "A"}     # letter 取值 A/B/C/D

发送端（xgo1）：
    link = XGOLink()
    link.send_letter("A")

接收端（xgo2）：
    link = XGOLink()
    link.bind()
    letter = link.receive_letter(timeout=10)   # 返回 "A"，超时返回 None

说明：
- 默认广播 255.255.255.255，同网段内所有机器人（和本机）都能收到；
- 也可通过 peer_ip 指定目标单播，更可靠；
- 发送端无需 bind；接收端需先 bind 监听端口。
"""
import json
import socket
import time

DEFAULT_PORT = 6001
BROADCAST_IP = "255.255.255.255"


class XGOLink:
    def __init__(self, port=DEFAULT_PORT, peer_ip=None):
        self.port = port
        self.peer_ip = peer_ip
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    # ----------------- 发送 -----------------
    def send_letter(self, letter):
        """发送一个字母（A/B/C/D），自动转大写。

        指定了 peer_ip 时单播给该地址；否则广播到整个网段。
        二选一，避免同一消息被接收端收到两份。
        """
        letter = letter.strip().upper()
        data = json.dumps({"type": "letter", "letter": letter}).encode("utf-8")
        if self.peer_ip:
            self.sock.sendto(data, (self.peer_ip, self.port))
        else:
            self.sock.sendto(data, (BROADCAST_IP, self.port))
        print(f"[link] 已发送字母: {letter}")

    def send_all(self, letters=("A", "B", "C", "D"), interval=1.0):
        """依次发送多个字母，每个间隔 interval 秒"""
        for ch in letters:
            self.send_letter(ch)
            time.sleep(interval)

    # ----------------- 接收 -----------------
    def bind(self):
        """接收端绑定监听端口（发送端不需要调用）"""
        self.sock.bind(("", self.port))

    def receive_letter(self, timeout=10.0):
        """等待接收一个字母，超时返回 None"""
        self.sock.settimeout(timeout)
        try:
            data, addr = self.sock.recvfrom(1024)
        except socket.timeout:
            return None
        except OSError:
            return None
        try:
            msg = json.loads(data.decode("utf-8"))
            if msg.get("type") == "letter":
                return msg.get("letter")
        except (ValueError, KeyError):
            pass
        return None

    def receive_loop(self, on_letter=None, timeout=10.0):
        """持续接收；每收到一个字母调用 on_letter(letter)。Ctrl+C 退出。"""
        while True:
            letter = self.receive_letter(timeout)
            if letter:
                if on_letter:
                    on_letter(letter)
                else:
                    print(f"[link] 收到字母: {letter}")

    def close(self):
        self.sock.close()
