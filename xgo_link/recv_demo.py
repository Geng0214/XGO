"""xgo2 接收端示例：接收并处理 xgo1 发来的 A/B/C/D 字母

用法（在 xgo2 上运行）：
    python3 recv_demo.py                # 监听 6001 端口，等待接收

在 on_letter 回调里根据字母触发机器狗动作即可。
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from xgo_link import XGOLink


def on_letter(letter):
    """每收到一个字母调用一次。在这里编写机器狗的动作响应。"""
    print(f"[xgo2] 收到指令字母: {letter}")
    # TODO: 根据字母触发动作，例如：
    # if letter == "A":
    #     dog.forward(10)
    # elif letter == "B":
    #     dog.turnleft(10)
    # ...


def main():
    link = XGOLink()
    link.bind()
    print("xgo2 正在监听 6001 端口，等待 xgo1 发送 A/B/C/D...（Ctrl+C 退出）")
    try:
        link.receive_loop(on_letter=on_letter)
    except KeyboardInterrupt:
        print("\n停止接收")
    finally:
        link.close()


if __name__ == "__main__":
    main()
