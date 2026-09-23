"""Isolated COCO inference; input is RGB bytes only, never simulator labels."""
import contextlib
import json
import struct
import sys
import time


class InferenceBridge:
    """Single in-flight RGB request; physics never waits on the worker pipe."""
    def __init__(self, process):
        import queue
        self.process = process
        self.results = queue.Queue(maxsize=1)
        self.busy = False
        self.thread = None

    def submit(self, rgb, frame_id, sim_time, context):
        if self.busy:
            return False
        import threading
        self.busy = True
        def exchange():
            try:
                metadata = json.dumps({'frame_id':frame_id,'sim_time':sim_time}).encode()
                payload = struct.pack('<I',len(metadata))+metadata+rgb.tobytes()
                self.process.stdin.write(struct.pack('<I',len(payload))+payload)
                self.process.stdin.flush()
                result = json.loads(self.process.stdout.readline())
                assert result['frame_id']==frame_id and result['sim_time']==sim_time
                self.results.put((result,context))
            except Exception as exc:
                self.results.put(exc)
        self.thread = threading.Thread(target=exchange,daemon=True)
        self.thread.start()
        return True

    def poll(self):
        import queue
        try:
            result = self.results.get_nowait()
        except queue.Empty:
            return None
        self.busy = False
        if isinstance(result,Exception):
            raise result
        return result

    def close(self):
        if self.thread is not None:
            self.thread.join(timeout=10)
            if self.thread.is_alive():
                raise TimeoutError('YOLO pipe thread did not finish')

def main():
    with contextlib.redirect_stdout(sys.stderr):
        import numpy as np
        import torch
        import ultralytics
        from ultralytics import YOLO
        model = YOLO(sys.argv[1])
        person = next(k for k,v in model.names.items() if v == 'person')
        model.predict(np.zeros((360,640,3), dtype=np.uint8), classes=[person],
                      device=0, imgsz=640, conf=.25, verbose=False)
    print(json.dumps({'ready': True, 'person_class': person, 'checkpoint': sys.argv[1],
                      'model_task': model.task, 'ultralytics_version': ultralytics.__version__,
                      'device': str(model.predictor.device),
                      'confidence': .25, 'imgsz': 640,
                      'class_count': len(model.names),
                      'training_data_metadata': str(model.ckpt.get('train_args',{}).get('data'))}), flush=True)
    while True:
        header = sys.stdin.buffer.read(4)
        if len(header) != 4:
            break
        length, = struct.unpack('<I', header)
        payload = sys.stdin.buffer.read(length)
        if len(payload) != length:
            break
        metadata = {}
        if len(payload) != 360*640*3:
            meta_size, = struct.unpack('<I', payload[:4])
            metadata = json.loads(payload[4:4+meta_size])
            payload = payload[4+meta_size:]
        rgb = np.frombuffer(payload, dtype=np.uint8).reshape(360,640,3)
        start = time.perf_counter()
        with contextlib.redirect_stdout(sys.stderr):
            result = model.predict(rgb[:,:,::-1].copy(), classes=[person], device=0,
                                   imgsz=640, conf=.25, verbose=False)[0]
            torch.cuda.synchronize()
        response = {**metadata, 'ms': (time.perf_counter()-start)*1000,
                    'boxes': result.boxes.xyxy.cpu().tolist(),
                    'conf': result.boxes.conf.cpu().tolist()}
        print(json.dumps(response), flush=True)

if __name__ == '__main__':
    main()
