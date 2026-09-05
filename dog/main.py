"""机器狗1 巡检识别 - 主入口"""

import sys
import os
import time
import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from config import (LINE_SPEED, PID_KP, PID_KI, PID_KD,
                    SCAN_CONFIRM_FRAMES, INSPECTION_TARGETS)
from dog_control import DogController
from vision.camera import Camera
from vision.line_detect import LineDetector, PID
from vision.letter_detect import LetterDetector
from vision.gauge_detect import GaugeDetector
from vision.digit_detect import DigitDetector
from speech.tts import Speaker
from comms.udp_sender import DogCommunicator


DOG_OK = False
dog = None
try:
    from xgolib import XGO
    dog = XGO(port="/dev/ttyAMA0", version="xgolite")
    dog.reset()
    time.sleep(2)
    DOG_OK = True
    print("Machine dog initialized")
except Exception as e:
    print(f"Dog connection failed: {e}")


class State:
    START = "START"
    LINE_FOLLOW = "LINE_FOLLOW"
    APPROACH = "APPROACH"
    SCAN = "SCAN"
    SPEAK = "SPEAK"
    SEND_RESULT = "SEND_RESULT"
    NEXT_TARGET = "NEXT_TARGET"
    DONE = "DONE"


def format_speech(letter, info):
    """格式化语音播报文本"""
    gauge = info.get("gauge", "")
    digit = info.get("digit", "")

    if gauge:
        if gauge == "正常":
            return f"{letter}区域仪表盘显示正常，状态正常"
        else:
            return f"{letter}区域仪表盘显示{gauge}，状态异常"
    elif digit:
        if digit == "正常":
            return f"{letter}区域数显表计显示正常，状态正常"
        else:
            return f"{letter}区域数显表计显示{digit}，状态异常"
    return ""


def main():
    print("===== Dog1 Inspection Mission =====")

    controller = DogController(dog) if DOG_OK else None
    camera = Camera()
    line_detector = LineDetector()
    pid = PID(PID_KP, PID_KI, PID_KD)
    letter_detector = LetterDetector()
    gauge_detector = GaugeDetector()
    digit_detector = DigitDetector()
    speaker = Speaker()
    comm = DogCommunicator()

    state = State.START
    results = {}
    current_target_idx = 0
    confirm_count = 0
    last_detection = None
    line_follow_timer = 0.0
    arrived = False

    try:
        while state != State.DONE:
            frame = camera.read()
            if frame is None:
                continue

            disp = frame.copy()

            if state == State.START:
                if DOG_OK:
                    dog.reset()
                    time.sleep(1)
                    controller.set_height(80)
                    controller.set_attitude(pitch=15)
                    time.sleep(0.5)
                print("Starting inspection mission")
                state = State.LINE_FOLLOW
                line_follow_timer = time.time()

            elif state == State.LINE_FOLLOW:
                offset = line_detector.detect(frame)
                if offset is not None:
                    correction = pid.update(offset)
                    if DOG_OK:
                        controller.turn_left(int(np.clip(correction, -30, 30)))
                        controller.walk_forward(LINE_SPEED)
                    cv2.putText(disp, f"Line follow: offset={offset:.2f}",
                                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                else:
                    if DOG_OK:
                        controller.walk_forward(LINE_SPEED)
                    cv2.putText(disp, "No line detected, going straight",
                                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

                elapsed = time.time() - line_follow_timer
                if elapsed > 15.0:
                    if DOG_OK:
                        controller.stop()
                    pid.reset()
                    print("Arrived at inspection zone")
                    state = State.APPROACH

            elif state == State.APPROACH:
                letter = letter_detector.detect(frame)
                cv2.putText(disp, f"Approaching target {letter or '?'}",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
                if letter:
                    confirm_count += 1
                    if confirm_count >= SCAN_CONFIRM_FRAMES:
                        confirm_count = 0
                        state = State.SCAN
                else:
                    confirm_count = 0
                    if DOG_OK:
                        controller.turn_left(8)
                        time.sleep(0.3)
                        controller.stop()

            elif state == State.SCAN:
                letter = letter_detector.detect(frame)
                gauge_info = gauge_detector.detect(frame)
                digit_info = digit_detector.detect(frame)

                detection = None
                if gauge_info:
                    detection = {"gauge": gauge_info["status"]}
                    label = f"Gauge: {gauge_info['zone']} -> {gauge_info['status']}"
                elif digit_info:
                    detection = {"digit": digit_info["status"]}
                    label = f"Digit: {digit_info['value']} -> {digit_info['status']}"
                else:
                    label = "Scanning..."

                cv2.putText(disp, f"Scan [{letter or '?'}] {label}",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

                if detection and detection == last_detection:
                    confirm_count += 1
                    if confirm_count >= SCAN_CONFIRM_FRAMES:
                        target = letter or INSPECTION_TARGETS[current_target_idx]
                        results[target] = detection
                        print(f"Confirmed: {target} -> {detection}")
                        confirm_count = 0
                        last_detection = None
                        state = State.SPEAK
                else:
                    confirm_count = 0
                    last_detection = detection

            elif state == State.SPEAK:
                target = INSPECTION_TARGETS[current_target_idx]
                info = results.get(target, {})
                text = format_speech(target, info)
                if text:
                    cv2.putText(disp, f"Speaking: {text}",
                                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                    cv2.imshow("Dog1 Inspector", disp)
                    cv2.waitKey(1)
                    speaker.speak(text)
                state = State.SEND_RESULT

            elif state == State.SEND_RESULT:
                target = INSPECTION_TARGETS[current_target_idx]
                info = results.get(target, {})
                gauge_status = info.get("gauge", "正常")
                digit_status = info.get("digit", "正常")
                comm.send_inspection_result(target, gauge_status, digit_status)
                state = State.NEXT_TARGET

            elif state == State.NEXT_TARGET:
                current_target_idx += 1
                if current_target_idx >= len(INSPECTION_TARGETS):
                    comm.send_all_done(results)
                    print("All targets inspected!")
                    state = State.DONE
                else:
                    if DOG_OK:
                        controller.turn_left(20)
                        time.sleep(1.0)
                        controller.stop()
                    print(f"Moving to target {INSPECTION_TARGETS[current_target_idx]}")
                    state = State.APPROACH

            cv2.putText(disp, f"State: {state}  Target: {INSPECTION_TARGETS[min(current_target_idx, len(INSPECTION_TARGETS)-1)]}",
                        (10, disp.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
            cv2.imshow("Dog1 Inspector", disp)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        if DOG_OK and dog:
            controller.stop()
            controller.set_attitude()
            controller.set_height(90)
        camera.release()
        comm.close()
        cv2.destroyAllWindows()
        print("Exiting...")
        print(f"Results: {results}")


if __name__ == "__main__":
    main()
