"""xgo1 发送端示例：循环发送 A、B、C、D 四个字母给 xgo2

用法（在 xgo1 上运行）：
    python3 send_demo.py                # 同网段广播，无需任何配置
    python3 send_demo.py --peer 192.168.1.20   # 指定 xgo2 的 IP 单播，更可靠
"""
import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from xgo_link import XGOLink


def main():
    parser = argparse.ArgumentParser(description="xgo1 字母发送端")
    parser.add_argument("--peer", default=None,
                        help="xgo2 的 IP（可选，指定后同时单播，更可靠）")
    parser.add_argument("--interval", type=float, default=1.0,
                        help="每个字母发送间隔(秒)，默认 1.0")
    args = parser.parse_args()

    link = XGOLink(peer_ip=args.peer)
    print("xgo1 开始发送 A/B/C/D（按 Ctrl+C 停止）...")
    if args.peer:
        print(f"目标单播地址: {args.peer}")
    try:
        while True:
            link.send_all(("A", "B", "C", "D"), interval=args.interval)
            print("--- 一轮发送完成，休息 2 秒 ---")
            time.sleep(2)
    except KeyboardInterrupt:
        print("\n停止发送")
    finally:
        link.close()


if __name__ == "__main__":
    main()
