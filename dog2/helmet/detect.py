import cv2
import os
import time
import numpy as np
import argparse

# ultralytics 为可选依赖：树莓派上装它会拖入 torch，比较重。
# 模型目录有同名的 best.onnx，可用已安装的 onnxruntime 推理回退。
try:
    from ultralytics import YOLO
    ULTRALYTICS_AVAILABLE = True
except ImportError:
    YOLO = None
    ULTRALYTICS_AVAILABLE = False

DEFAULT_MODEL = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                             "models", "best.pt")


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


class _SimpleBox:
    """模拟 ultralytics 单框接口：xyxy / cls / conf 为 1xN 数组"""
    def __init__(self, xyxy, cls, conf):
        self.xyxy = np.array([xyxy], dtype=np.float32)
        self.cls = np.array([cls], dtype=np.float32)
        self.conf = np.array([conf], dtype=np.float32)


class _SimpleBoxes:
    """模拟 ultralytics 的 result.boxes 接口"""
    def __init__(self, dets):
        # dets: list of (x1, y1, x2, y2, cls, conf)
        self._boxes = [_SimpleBox(d[:4], d[4], d[5]) for d in dets]
        self.xyxy = np.array([d[:4] for d in dets], dtype=np.float32).reshape(-1, 4)
        self.cls = np.array([d[4] for d in dets], dtype=np.float32).reshape(-1)
        self.conf = np.array([d[5] for d in dets], dtype=np.float32).reshape(-1)

    def __len__(self):
        return len(self._boxes)

    def __iter__(self):
        return iter(self._boxes)


class _SimpleResult:
    def __init__(self, dets):
        self.boxes = _SimpleBoxes(dets) if dets else None


class ONNXYOLO:
    """ultralytics 不可用时的 ONNX 推理回退（树莓派免装 torch）。
    接口与 ultralytics 的 model(frame, conf=...) 兼容，输出同样式检测结果。"""

    def __init__(self, model_path, names=None):
        import onnxruntime as ort
        self.session = ort.InferenceSession(
            model_path, providers=["CPUExecutionProvider"])
        inp = self.session.get_inputs()[0]
        self.input_name = inp.name
        self.input_size = inp.shape[-1] if isinstance(inp.shape[-1], int) else 640
        if names is None:
            # 与 best.pt 训练一致：3 类，0/2=戴帽、1=未戴帽
            names = {0: "helmet", 1: "no_helmet", 2: "helmet"}
        self.names = names

    def __call__(self, img, conf=0.5, verbose=False):
        dets = self._predict(img, conf)
        return [_SimpleResult(dets)]

    @staticmethod
    def _nms(boxes, scores, iou_thr=0.45):
        """简单 NMS，boxes: [N,4] xyxy"""
        x1, y1, x2, y2 = boxes[:, 0], boxes[:, 1], boxes[:, 2], boxes[:, 3]
        areas = (x2 - x1).clip(min=0) * (y2 - y1).clip(min=0)
        order = scores.argsort()[::-1]
        keep = []
        while order.size > 0:
            i = order[0]
            keep.append(int(i))
            xx1 = np.maximum(x1[i], x1[order[1:]])
            yy1 = np.maximum(y1[i], y1[order[1:]])
            xx2 = np.minimum(x2[i], x2[order[1:]])
            yy2 = np.minimum(y2[i], y2[order[1:]])
            inter = np.maximum(0, xx2 - xx1) * np.maximum(0, yy2 - yy1)
            iou = inter / (areas[i] + areas[order[1:]] - inter + 1e-9)
            order = order[1:][iou <= iou_thr]
        return keep

    def _predict(self, img_bgr, conf):
        s = self.input_size
        h, w = img_bgr.shape[:2]
        scale = min(s / w, s / h)
        nw, nh = max(1, int(round(w * scale))), max(1, int(round(h * scale)))
        dx, dy = (s - nw) // 2, (s - nh) // 2
        canvas = np.full((s, s, 3), 114, dtype=np.uint8)
        canvas[dy:dy + nh, dx:dx + nw] = cv2.resize(img_bgr, (nw, nh))

        blob = canvas[:, :, ::-1].transpose(2, 0, 1).astype(np.float32) / 255.0
        out = self.session.run(None, {self.input_name: blob[None]})[0]  # [1, C, 8400]
        pred = out[0]                                  # [C, 8400]
        xywh = pred[:4]                                # letterbox 像素坐标
        scores = pred[4:]                              # [nc, 8400]

        cls_ids = np.argmax(scores, axis=0)
        confs = np.max(scores, axis=0)
        keep = confs >= conf
        if not keep.any():
            return []

        cx = (xywh[0][keep] - dx) / scale
        cy = (xywh[1][keep] - dy) / scale
        cw = xywh[2][keep] / scale
        ch = xywh[3][keep] / scale
        boxes = np.stack([cx - cw / 2, cy - ch / 2, cx + cw / 2, cy + ch / 2],
                         axis=1).astype(np.float32)
        cls_keep = cls_ids[keep].astype(int)
        conf_keep = confs[keep].astype(float)

        idx = self._nms(boxes, conf_keep)
        return [(float(boxes[i][0]), float(boxes[i][1]), float(boxes[i][2]),
                 float(boxes[i][3]), int(cls_keep[i]), float(conf_keep[i]))
                for i in idx]


class HelmetDetector:
    def __init__(self, model_path, conf=0.5, target_count=5):
        if ULTRALYTICS_AVAILABLE:
            self.model = YOLO(model_path)
        else:
            # 回退：用同目录 best.onnx + onnxruntime（免装 torch）
            onnx_path = os.path.splitext(model_path)[0] + ".onnx"
            if not os.path.exists(onnx_path):
                raise RuntimeError(
                    f"ultralytics 不可用，且找不到 ONNX 模型: {onnx_path}")
            print(f"[视觉] 未检测到 ultralytics，改用 ONNX 推理: {onnx_path}")
            self.model = ONNXYOLO(onnx_path)
        self.conf = conf
        self.target_count = target_count
        self.tracker = PersonTracker(iou_thresh=0.3, disappear_timeout=1.5)

        # 历史去重：同一个 track_id 只记录一次
        self.seen_persons = {}  # tid -> {'helmet': bool}
        self.reached_target = False
        self.saved = False

    def process_frame(self, frame, debug=False):
        results = self.model(frame, conf=self.conf, verbose=False)
        detections = []
        for r in results:
            if r.boxes is None:
                continue
            for box in r.boxes:
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                cls = int(box.cls[0].item())
                conf = float(box.conf[0].item())
                is_helmet = cls in [0, 2]
                name = self.model.names.get(cls, str(cls))
                detections.append({'bbox': [x1, y1, x2, y2], 'helmet': is_helmet})
                if debug:
                    print(f"  det: cls={cls}({name}) conf={conf:.3f} bbox=[{int(x1)},{int(y1)},{int(x2)},{int(y2)}]")

        now = time.time()
        matched = self.tracker.update(detections, now)

        for tid, det in matched:
            if tid not in self.seen_persons:
                if not self.reached_target:
                    self.seen_persons[tid] = {'helmet': det['helmet']}
                    status = "戴帽" if det['helmet'] else "未戴帽"
                    print(f"[+] ID={tid}, {status}, 累计 {len(self.seen_persons)}/{self.target_count}")
                    if len(self.seen_persons) >= self.target_count:
                        self.reached_target = True
                        print(f"*** 已达到目标人数 {self.target_count}，停止计数 ***")

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
            cn_nums = {0: "零", 1: "一", 2: "二", 3: "三", 4: "四", 5: "五",
                       6: "六", 7: "七", 8: "八", 9: "九", 10: "十"}
            def to_cn(n):
                return cn_nums.get(n, str(n))
            text = f"【{to_cn(helmet_count)}人佩戴安全帽，{to_cn(no_helmet_count)}人没有佩戴安全帽】"
            cv2.putText(frame, text, (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 255), 3)
            if not self.saved:
                cv2.imwrite('result.jpg', frame)
                print(f"\n{text}")
                self.saved = True

        return frame

    def print_summary(self):
        total = len(self.seen_persons)
        helmet_count = sum(1 for p in self.seen_persons.values() if p['helmet'])
        no_helmet_count = total - helmet_count

        cn_nums = {0: "零", 1: "一", 2: "二", 3: "三", 4: "四", 5: "五",
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

    frame_count = 0
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        frame_count += 1
        if frame_count % 30 == 1:  # 每30帧打印一次调试信息
            print(f"\n--- Frame {frame_count} ---")
            frame = detector.process_frame(frame, debug=True)
        else:
            frame = detector.process_frame(frame)
        cv2.imshow("Helmet Detection", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    detector.print_summary()
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
