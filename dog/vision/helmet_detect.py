import cv2
import os
import time
import numpy as np
from ultralytics import YOLO
import argparse

DEFAULT_MODEL = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                             "models", "best.onnx")


def compute_iou(box_a, box_b):
    x1 = max(box_a[0], box_b[0])
    y1 = max(box_a[1], box_b[1])
    x2 = min(box_a[2], box_b[2])
    y2 = min(box_a[3], box_b[3])
    inter = max(0, x2 - x1) * max(0, y2 - y1)
    area_a = (box_a[2] - box_a[0]) * (box_a[3] - box_a[1])
    area_b = (box_b[2] - box_b[0]) * (box_b[3] - box_b[1])
    union = area_a + area_b - inter
    return inter / union if union > 0 else 0


class PersonTracker:
    def __init__(self, iou_thresh=0.3, disappear_timeout=1.5):
        self.tracks = {}          # track_id -> {'box', 'helmet', 'last_seen', 'age'}
        self.next_id = 0
        self.iou_thresh = iou_thresh
        self.disappear_timeout = disappear_timeout

    def update(self, detections, now):
        """detections: [{'bbox': [x1,y1,x2,y2], 'helmet': bool}, ...]"""
        if not detections:
            self._tick(now)
            return []

        matched = []
        unmatched_dets = list(range(len(detections)))
        unmatched_tracks = set(self.tracks.keys())

        # 匹配：按 IoU 从大到小贪心匹配
        iou_pairs = []
        for di in unmatched_dets:
            for tid in unmatched_tracks:
                iou = compute_iou(detections[di]['bbox'], self.tracks[tid]['box'])
                if iou >= self.iou_thresh:
                    iou_pairs.append((iou, di, tid))
        iou_pairs.sort(key=lambda x: -x[0])

        used_dets = set()
        used_tracks = set()
        for iou, di, tid in iou_pairs:
            if di in used_dets or tid in used_tracks:
                continue
            det = detections[di]
            self.tracks[tid]['box'] = det['bbox']
            self.tracks[tid]['helmet'] = det['helmet']
            self.tracks[tid]['last_seen'] = now
            matched.append((tid, det))
            used_dets.add(di)
            used_tracks.add(tid)

        # 未匹配的检测 → 新 track
        for di in unmatched_dets:
            if di in used_dets:
                continue
            det = detections[di]
            tid = self.next_id
            self.next_id += 1
            self.tracks[tid] = {
                'box': det['bbox'],
                'helmet': det['helmet'],
                'last_seen': now,
                'age': now,
            }
            matched.append((tid, det))

        # 删除超时的 track
        self._tick(now)
        return matched

    def _tick(self, now):
        dead = [tid for tid, t in self.tracks.items()
                if now - t['last_seen'] > self.disappear_timeout]
        for tid in dead:
            del self.tracks[tid]

    def get_all_seen(self):
        return {tid: {'helmet': t['helmet']} for tid, t in self.tracks.items()}


class HelmetDetector:
    def __init__(self, model_path, conf=0.5, target_count=5, skip_frames=4):
        self.model = YOLO(model_path, task="detect")
        self.conf = conf
        self.target_count = target_count
        self.tracker = PersonTracker(iou_thresh=0.3, disappear_timeout=1.5)
        self.skip_frames = skip_frames

        # 历史去重：同一个 track_id 只记录一次
        self.seen_persons = {}  # tid -> {'helmet': bool}
        self.reached_target = False
        self.saved = False

        # 跳帧状态
        self._frame_idx = 0
        self._last_dets = []

    def process_frame(self, frame):
        self._frame_idx += 1

        # 跳帧：非推理帧复用上次结果
        if self.skip_frames > 0 and self._frame_idx % (self.skip_frames + 1) != 1:
            detections = self._last_dets
        else:
            results = self.model(frame, conf=self.conf, verbose=False)
            detections = []
            for r in results:
                if r.boxes is None:
                    continue
                for box in r.boxes:
                    x1, y1, x2, y2 = box.xyxy[0].tolist()
                    cls = int(box.cls[0].item())
                    is_helmet = cls in [0, 2]
                    detections.append({'bbox': [x1, y1, x2, y2], 'helmet': is_helmet})
            self._last_dets = detections

        now = time.time()
        matched = self.tracker.update(detections, now)

        for tid, det in matched:
            if tid not in self.seen_persons:
                if not self.reached_target:
                    self.seen_persons[tid] = {'helmet': det['helmet']}
                    status = "Helmet" if det['helmet'] else "No Helmet"
                    print(f"[+] ID={tid}, {status}, counted {len(self.seen_persons)}/{self.target_count}")
                    if len(self.seen_persons) >= self.target_count:
                        self.reached_target = True
                        print(f"*** Reached target {self.target_count}, counting stopped ***")

        # 画框
        for tid, det in self.tracker.tracks.items():
            x1, y1, x2, y2 = [int(v) for v in det['box']]
            is_helmet = det['helmet']
            color = (0, 255, 0) if is_helmet else (0, 0, 255)
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            label = f"ID{tid} {'Helmet' if is_helmet else 'No Helmet'}"
            cv2.putText(frame, label, (x1, y1 - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        # 顶部统计
        total = len(self.seen_persons)
        helmet_count = sum(1 for p in self.seen_persons.values() if p['helmet'])
        no_helmet_count = total - helmet_count
        cv2.putText(frame, f"Total: {total}/{self.target_count}  Helmet: {helmet_count}  No Helmet: {no_helmet_count}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        if self.reached_target:
            text = f"{helmet_count} have, {no_helmet_count} no have"
            cv2.putText(frame, text, (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 255), 3)
            if not self.saved:
                cv2.imwrite('result.jpg', frame)
                self.saved = True

        return frame

    def print_summary(self):
        total = len(self.seen_persons)
        helmet_count = sum(1 for p in self.seen_persons.values() if p['helmet'])
        no_helmet_count = total - helmet_count

        cn_nums = {0: "零", 1: "一", 2: "两", 3: "三", 4: "四", 5: "五",
                   6: "六", 7: "七", 8: "八", 9: "九", 10: "十"}
        def to_cn(n):
            if n <= 10:
                return cn_nums[n]
            return str(n)

        print(f"\n【{to_cn(helmet_count)}人佩戴安全帽，{to_cn(no_helmet_count)}人没有佩戴安全帽】")


def main():
    parser = argparse.ArgumentParser(description="Helmet detection")
    parser.add_argument("--model", type=str, default=DEFAULT_MODEL)
    parser.add_argument("--source", type=str, default="0")
    parser.add_argument("--conf", type=float, default=0.85)
    parser.add_argument("--target", type=int, default=5)
    args = parser.parse_args()

    source = int(args.source) if args.source.isdigit() else args.source
    detector = HelmetDetector(args.model, conf=args.conf, target_count=args.target)

    cap = cv2.VideoCapture(source)
    if not cap.isOpened():
        print(f"Cannot open source: {args.source}")
        return

    print(f"Source={source}, Conf={args.conf}, Target={args.target}. Press 'q' to quit.")

    while True:
        ret, frame = cap.read()
        if not ret:
            break
        frame = detector.process_frame(frame)
        cv2.imshow("Helmet Detection", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        if detector.reached_target:
            break

    detector.print_summary()
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
