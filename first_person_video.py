"""Recording-only consumer of the existing RGB exposure and perception results."""
import json
import subprocess
from pathlib import Path

import numpy as np


class FirstPersonVideo:
    def __init__(self, output, filename='A300_BLIND_CORNER_FIRST_PERSON.mp4', human_count=1):
        self.output = Path(output)
        self.filename = filename
        self.human_count = human_count
        self.frames = []
        self.results = {}

    def capture(self, rgb, frame_id, stamp, elapsed, state, speed, selected=None):
        assert rgb.shape == (360, 640, 3)
        if self.frames:
            assert stamp > self.frames[-1]['stamp']
        self.frames.append(dict(rgb=rgb.copy(), frame_id=frame_id, stamp=stamp,
                                elapsed=elapsed, state=state, speed=float(speed), selected=selected))

    def detection(self, response):
        # Deliberate allowlist: evaluation/GT fields cannot reach the renderer.
        self.results[response['frame_id']] = {k: response.get(k, []) for k in
            ('sim_time', 'boxes', 'conf', 'observations', 'tracks')}

    def finish(self, control_rows, source_run):
        import cv2
        self.output.mkdir(parents=True, exist_ok=True)
        target = self.output/self.filename
        if target.exists():
            raise FileExistsError(target)
        command = ['ffmpeg', '-hide_banner', '-loglevel', 'error', '-f', 'rawvideo',
                   '-pix_fmt', 'rgb24', '-s', '640x360', '-r', '10', '-i', '-',
                   '-vf', 'scale=1280:720:flags=lanczos', '-c:v', 'libx264',
                   '-preset', 'fast', '-crf', '18', '-pix_fmt', 'yuv420p',
                   '-movflags', '+faststart', str(target)]
        proc = subprocess.Popen(command, stdin=subprocess.PIPE,
                                creationflags=subprocess.CREATE_NO_WINDOW)
        manifest = []
        for f in self.frames:
            rgb = f['rgb'].copy()
            r = self.results.get(f['frame_id'])
            if r is not None:
                assert abs(r['sim_time']-f['stamp']) < 1e-8
            boxes = r['boxes'] if r else []
            def text(value, xy, color=(255, 255, 255), size=.43):
                cv2.putText(rgb, value, xy, cv2.FONT_HERSHEY_SIMPLEX, size, (0, 0, 0), 3, cv2.LINE_AA)
                cv2.putText(rgb, value, xy, cv2.FONT_HERSHEY_SIMPLEX, size, color, 1, cv2.LINE_AA)
            used = set()
            labels = []
            for box, conf in zip(boxes, r['conf'] if r else []):
                x1, y1, x2, y2 = map(int, box)
                obs = next((o for o in r['observations'] if np.allclose(o['bbox'], box)), None)
                track_id = '--'
                if obs:
                    candidates = [t for t in r['tracks'] if t['track_id'] not in used
                                  and abs(t['last_observed_time']-f['stamp']) < 1e-8]
                    if candidates:
                        t = min(candidates, key=lambda t: np.linalg.norm(np.array(t['state'][:2])-obs['world_xyz'][:2]))
                        if np.linalg.norm(np.array(t['state'][:2])-obs['world_xyz'][:2]) < .5:
                            track_id = str(t['track_id'])
                            used.add(t['track_id'])
                cv2.rectangle(rgb, (x1, y1), (x2, y2), (55, 235, 130), 2)
                distance = f"{obs['depth_m']:.2f} m" if obs else 'unavailable'
                tx = min(max(x1, 4), 450)
                ty = min(max(y1+15, 88), 315)
                # Recording layout only: avoid overlapping multi-person text.
                candidates = [(tx, ty)] + [(tx, y) for y in range(88, 316, 54)] + [(x, y) for x in (4, 218, 450) for y in range(88, 316, 54)]
                for lx, ly in candidates:
                    rect = (lx, ly-12, lx+170, ly+36)
                    if not any(rect[0]<r[2] and rect[2]>r[0] and rect[1]<r[3] and rect[3]>r[1] for r in labels):
                        tx, ty = lx, ly
                        labels.append(rect)
                        break
                if abs(tx-x1)>25 or abs(ty-y1-15)>25:
                    cv2.line(rgb,(tx,ty-7),(max(0,x1),max(0,y1)),(55,235,130),1)
                for j, line in enumerate((f'Person {track_id}', f'Conf: {conf:.2f}', f'Dist: {distance}')):
                    text(line, (tx, ty+j*16), (55, 235, 130))
            # Small HUD leaves the native camera geometry untouched.
            cv2.rectangle(rgb, (0, 0), (238, 68), (12, 22, 30), -1)
            text('A300  |  ROBOT CAMERA', (9, 17))
            text(f"State: {f['state']}", (9, 36))
            text(f"Speed: {f['speed']:.2f} m/s   Detected: {len(boxes)}", (9, 55))
            if f.get('selected'):
                cv2.rectangle(rgb,(270,0),(639,26),(12,22,30),-1)
                text(f"Selected: {f['selected']}",(280,18))
            proc.stdin.write(rgb.tobytes())
            manifest.append({k: v for k, v in f.items() if k != 'rgb'} |
                            {'yolo_returned': r is not None, 'person_count': len(boxes)})
        proc.stdin.close()
        if proc.wait(timeout=120):
            raise RuntimeError('First-person ffmpeg encoding failed')
        def first(predicate):
            return next((r['time'] for r in control_rows if predicate(r)), None)
        slow = first(lambda r: r['state'] == 'SLOW')
        events = dict(first_detection=next((f['elapsed'] for f in manifest if f['person_count']), None),
                      first_multi_person_detection=next((f['elapsed'] for f in manifest if f['person_count']>=2), None),
                      first_slow=slow, first_avoid=first(lambda r: r['state'].startswith('AVOID')),
                      first_stop=first(lambda r: r['state'] == 'STOP' and not r['route_done']),
                      resume_cruise=first(lambda r: slow is not None and r['time'] > slow and r['state'] == 'CRUISE'))
        deltas = np.diff([f['stamp'] for f in self.frames])
        summary = dict(source_run=str(source_run), source_camera='/World/RobotCamera',
                       human_count=self.human_count, max_simultaneous_detections=max(f['person_count'] for f in manifest),
                       source_resolution=[640, 360], output_resolution=[1280, 720], fps=10,
                       frames=len(manifest), duration_s=len(manifest)/10,
                       frame_detection_timestamp_aligned=True, gt_shown=False,
                       distance_definition='Estimated optical depth from existing person-depth pipeline; meters',
                       id_display='Nearest same-exposure observed KF state within 0.5 m, display only; -- if unavailable',
                       camera_interval_min_s=float(deltas.min()), camera_interval_max_s=float(deltas.max()),
                       interaction_timestamps_s=events, timestamp_origin='simulation start after warmup',
                       yolo_result_frames=sum(f['yolo_returned'] for f in manifest),
                       no_result_policy='No bounding boxes; never reuse an older detection', frames_manifest=manifest)
        (self.output/'summary.json').write_text(json.dumps(summary, indent=2))
        print('FIRST_PERSON_VIDEO', str(target), flush=True)
