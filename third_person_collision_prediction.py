"""Fixed-camera, open-loop robot–pedestrian collision-prediction experiment.

Human actor poses and routes are recorded for post-run evaluation only. The
predictor receives camera detections, depth, CV-KF tracks, and robot ego state.
"""
import argparse
import json
import math
import struct
import subprocess
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent
YOLO_PYTHON = Path(r'D:\detection\robot_human_avoidance\.venv\python.exe')
YOLO_WEIGHTS_DIR = Path(r'D:\detection\robot_human_avoidance')
CAMERA = {'path': '/World/CollisionPredictionCamera', 'eye': [5., -5., 7.],
          'target': [5., 0., 0.], 'resolution': [640, 360], 'rate_hz': 10.,
          'focal_mm': 18., 'horizontal_aperture_mm': 24.,
          'vertical_aperture_mm': 13.5, 'clipping_m': [.05, 100.]}
ROBOT_SPEED = .30
HUMAN_RADIUS = .30
WARNING_MARGIN = .20
HORIZON = 5.
DT = .1


def scenario_spec(name):
    # Crossing and diagonal timing are chosen before running the predictor.
    # Near-misses retain the same spatial path and change only the delay.
    specs = {
        'headon_collision': ([7., 0.], [[.5, 0.]], .65, 0.),
        'perpendicular_collision': ([4.5, -2.5], [[4.5, 2.5]], .65, 15. - 2.5/.65),
        'diagonal_collision': ([3.5, -2.3], [[6.5, 2.3]], .60,
                               5./ROBOT_SPEED - math.hypot(1.5, 2.3)/.60),
        'cutin_collision': ([2.5, -2.0], [[5.5, .2], [10.5, .2]], .50, 12.),
        'rear_end_collision': ([1.8, 0.], [[10.5, 0.]], .20, 0.),
        'perpendicular_nearmiss': ([4.5, -2.5], [[4.5, 2.5]], .65, 6.0),
        'diagonal_nearmiss': ([3.5, -2.3], [[6.5, 2.3]], .60, 4.0),
        'cutin_nearmiss': ([2.5, -2.0], [[5.5, .2], [10.5, .2]], .50, 4.0),
        'multi_person_collision': None,
    }
    if name not in specs:
        raise ValueError(name)
    if name == 'multi_person_collision':
        return [
            dict(start=[7., 0., 0.], goals=[[.5, 0., 0.]], speed=.65, delay=0.),
            dict(start=[4.5, 2.2, 0.], goals=[[8., 2.2, 0.]], speed=.45, delay=0.),
            dict(start=[4.5, -2.5, 0.], goals=[[4.5, 2.5, 0.]], speed=.65, delay=6.),
        ]
    start, goals, speed, delay = specs[name]
    return [dict(start=[*start, 0.], goals=[[ *p, 0.] for p in goals], speed=speed, delay=delay)]


class PrescribedPeople:
    def __init__(self, agents, specs):
        import carb
        self.agents, self.specs = agents, specs
        self.started = [False]*len(agents)
        self.goal_index = [0]*len(agents)
        for agent, spec in zip(agents, specs):
            agent.set_auto_avoidance_enabled(False)
            agent.set_obstacle_avoidance_enabled(False)
            if not agent.teleport(carb.Float3(*spec['start'])):
                raise RuntimeError('Human teleport failed')
            agent.set_speed(max(spec['speed'], .01))

    def step(self, elapsed):
        import carb
        for i, (agent, spec) in enumerate(zip(self.agents, self.specs)):
            if not self.started[i] and elapsed >= spec['delay']:
                if agent.move_to(carb.Float3(*spec['goals'][0]), auto_brake=True) == -1:
                    raise RuntimeError('Human waypoint rejected by navmesh')
                self.started[i] = True
            elif self.started[i] and self.goal_index[i]+1 < len(spec['goals']):
                p = agent.get_world_translation()
                if np.linalg.norm(np.array([p.x, p.y])-spec['goals'][self.goal_index[i]][:2]) < .25:
                    self.goal_index[i] += 1
                    if agent.move_to(carb.Float3(*spec['goals'][self.goal_index[i]]), auto_brake=True) == -1:
                        raise RuntimeError('Human waypoint rejected by navmesh')


def predictions(state, robot_xy, robot_v, collision_radius):
    """CPA and independent 0.1 s straight-line rollout, with no GT access."""
    h = np.asarray(state, dtype=float)
    relative = h[:2]-robot_xy
    velocity = h[2:]-robot_v
    vv = float(velocity @ velocity)
    raw_cpa = -float(relative @ velocity)/vv if vv > 1e-9 else math.inf
    cpa_valid = 0. < raw_cpa <= HORIZON
    d_cpa = float(np.linalg.norm(relative+velocity*raw_cpa)) if cpa_valid else math.inf
    ts = np.arange(0., HORIZON+DT/2, DT)
    displacement = relative[None, :]+ts[:, None]*velocity[None, :]
    distances = np.linalg.norm(displacement, axis=1)
    k = int(np.argmin(distances))
    first_hit = np.flatnonzero(distances < collision_radius)
    first_warning = np.flatnonzero(distances < collision_radius+WARNING_MARGIN)
    return dict(t_cpa_s=float(raw_cpa) if cpa_valid else None,
                d_cpa_m=d_cpa if cpa_valid else None,
                rollout_min_distance_m=float(distances[k]),
                rollout_min_time_s=float(ts[k]),
                predicted_ttc_s=float(ts[first_hit[0]]) if len(first_hit) else None,
                warning_ttc_s=float(ts[first_warning[0]]) if len(first_warning) else None,
                cpa_collision=bool(cpa_valid and d_cpa < collision_radius),
                rollout_collision=bool(len(first_hit)),
                risk=('COLLISION_RISK' if len(first_hit) else
                      'WARNING' if len(first_warning) else 'SAFE'))


def project_people(response, depth, params, depth_config):
    """Same torso depth and USD row-vector backprojection as stage_probe.py."""
    from tracker import person_depth
    aperture = np.asarray(params['cameraAperture'])
    offsets = np.asarray(params['cameraApertureOffset'])
    resolution = np.asarray(params['renderProductResolution'])
    focal = float(params['cameraFocalLength'])
    fx, fy = focal*resolution/aperture
    cx, cy = resolution/2 + np.array([-1, 1])*offsets*resolution/aperture
    world_from_camera_row = np.linalg.inv(np.asarray(params['cameraViewTransform']).reshape(4, 4))
    measurements, observations = [], []
    for box, confidence in zip(response['boxes'], response['conf']):
        diagnostic = person_depth(depth, box, depth_config, (fx, fy, cx, cy))
        observation = dict(bbox=box, confidence=confidence, depth_diagnostic=diagnostic)
        if diagnostic.get('accepted'):
            z, u, v = (diagnostic[key] for key in ('depth_m', 'u', 'v'))
            point = np.array([(u-cx)*z/fx, -(v-cy)*z/fy, -z, 1.])
            xyz = (point @ world_from_camera_row)[:3]
            observation['estimated_world_xyz'] = xyz.tolist()
            measurements.append((xyz[:2], confidence))
        observations.append(observation)
    return measurements, observations


def draw_frame(rgb, response, observations, tracks, risks, elapsed, writer):
    import cv2
    image = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    cv2.rectangle(image, (0, 0), (640, 49), (16, 24, 35), -1)
    overall = 'COLLISION PREDICTED' if any(r['risk']=='COLLISION_RISK' for r in risks) else (
        'WARNING' if any(r['risk']=='WARNING' for r in risks) else 'SAFE')
    cv2.putText(image, f'Fixed third-person RGB-D | t={elapsed:.1f}s', (9, 19),
                cv2.FONT_HERSHEY_SIMPLEX, .47, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(image, overall, (9, 40), cv2.FONT_HERSHEY_SIMPLEX, .61,
                (30, 80, 255) if overall != 'SAFE' else (120, 240, 120), 2, cv2.LINE_AA)
    for box, observation in zip(response['boxes'], observations):
        x1, y1, x2, y2 = map(int, box)
        cv2.rectangle(image, (x1, y1), (x2, y2), (25, 230, 25), 2)
        label = f"person {observation['confidence']:.2f}"
        xyz = observation.get('estimated_world_xyz')
        if xyz:
            candidates = [t for t in tracks if t['miss_count']==0]
            if candidates:
                nearest = min(candidates, key=lambda t: np.linalg.norm(np.asarray(t['state'][:2])-xyz[:2]))
                label += f" ID {nearest['track_id']}"
        cv2.putText(image, label, (max(0, x1), max(64, y1-5)),
                    cv2.FONT_HERSHEY_SIMPLEX, .39, (25, 230, 25), 1, cv2.LINE_AA)
    for j, item in enumerate(risks[:3]):
        track = next(t for t in tracks if t['track_id']==item['track_id'])
        x, y, vx, vy = track['state']
        ttc = item['predicted_ttc_s']
        label = f"ID {item['track_id']} XY {x:.1f},{y:.1f} V {vx:.1f},{vy:.1f} TTC {ttc:.1f}s" if ttc is not None else (
            f"ID {item['track_id']} XY {x:.1f},{y:.1f} V {vx:.1f},{vy:.1f} {item['risk']}")
        cv2.putText(image, label, (8, 340-j*19), cv2.FONT_HERSHEY_SIMPLEX,
                    .38, (255, 255, 255), 1, cv2.LINE_AA)
    writer.write(image)


def run(args):
    from isaacsim import SimulationApp
    app = SimulationApp({'headless': args.headless, 'renderer': 'RayTracedLighting',
                         'width': 960, 'height': 540, 'multi_gpu': False,
                         'active_gpu': 0, 'physics_gpu': 0,
                         'extra_args': ['--enable', 'isaacsim.sensors.experimental.rtx',
                                        '--enable', 'isaacsim.replicator.agent.core']})
    worker, writer, worker_log = None, None, None
    out = (ROOT / 'outputs' / f'third_person_collision_prediction_{Path(args.detector_model).stem}'
           / args.scenario / f'seed_{args.seed}')
    out.mkdir(parents=True, exist_ok=True)
    try:
        import cv2
        import omni.replicator.core as rep
        import omni.anim.behavior.core as behavior
        import NavSchema
        from pxr import Usd, UsdLux
        from scipy.spatial.transform import Rotation
        from isaacsim.core.api import World
        from isaacsim.sensors.experimental.rtx import RtxCamera, CameraSensor
        from omni.kit.async_engine import run_coroutine
        from classic_single_scene import build
        from walking_actor import load_actor
        from robot_backend import settings, configure_a300, KinematicA300, WheelAdapter
        from tracker import Tracker

        np.random.seed(args.seed)
        config = json.loads((ROOT/'config.yaml').read_text())
        specs = scenario_spec(args.scenario)
        robot_config = settings('a300')
        radius = robot_config['conservative_radius_m'] + HUMAN_RADIUS
        world = World(stage_units_in_meters=1., physics_dt=1/60, rendering_dt=1/60)
        world.scene.add_default_ground_plane()
        scene_config = dict(blind_wall=None, human_navigation=dict(blocked_speed_threshold=5.))
        build(world.stage, ROOT/'classic_single_scene.usd', scene_config)
        world.stage.DefinePrim(robot_config['prim_path'], 'Xform').GetReferences().AddReference(robot_config['asset'])
        configure_a300(world.stage, robot_config, True)
        NavSchema.NavMeshExcludeAPI.Apply(world.stage.GetPrimAtPath(robot_config['prim_path']))
        robot = KinematicA300(world.stage, robot_config)
        wheel = WheelAdapter(robot_config, [12.]*4)
        UsdLux.DomeLight.Define(world.stage, '/World/Light').CreateIntensityAttr(500.)

        eye = np.asarray(CAMERA['eye']); forward = np.asarray(CAMERA['target'])-eye
        forward /= np.linalg.norm(forward)
        right = np.cross(forward, [0., 0., 1.]); right /= np.linalg.norm(right)
        up = np.cross(right, forward)
        q = Rotation.from_matrix(np.column_stack((right, up, -forward))).as_quat()
        camera = RtxCamera(CAMERA['path'], tick_rate=CAMERA['rate_hz'],
                           translations=eye, orientations=np.array([q[3], *q[:3]]))
        prim = camera.prims[0]
        for key, attr in [('focal_mm', 'focalLength'),
                          ('horizontal_aperture_mm', 'horizontalAperture'),
                          ('vertical_aperture_mm', 'verticalAperture')]:
            prim.GetAttribute(attr).Set(CAMERA[key])
        camera.camera.set_clipping_ranges(*CAMERA['clipping_m'])

        task = run_coroutine(load_actor(world.stage, len(specs), False, 'classic'))
        deadline = time.perf_counter()+240.
        while not task.done():
            app.update()
            if time.perf_counter() > deadline:
                task.cancel(); raise TimeoutError('Actor setup exceeded 240 seconds')
        task.result()
        world.reset()
        for _ in range(30):
            world.step(render=True)
        paths = [str(p.GetPath()) for p in Usd.PrimRange(world.stage.GetPrimAtPath('/World/Characters'))
                 if p.GetTypeName()=='SkelRoot']
        agents = [behavior.acquire_interface().get_agent(p) for p in paths]
        if len(agents)!=len(specs) or any(a is None for a in agents):
            raise RuntimeError(f'Actor count mismatch: {len(agents)} != {len(specs)}')
        people = PrescribedPeople(agents, specs)
        sensor = CameraSensor(camera, resolution=(360, 640),
                              annotators=['rgb', 'distance_to_image_plane'])
        reference = rep.annotators.get('ReferenceTime')
        parameters = rep.annotators.get('CameraParams')
        reference.attach(sensor._hydra_texture.path)
        parameters.attach(sensor._hydra_texture.path)
        writer = cv2.VideoWriter(str(out/'THIRD_PERSON_COLLISION_PREDICTION.mp4'),
                                 cv2.VideoWriter_fourcc(*'mp4v'), 10., (640, 360))
        if not writer.isOpened():
            raise RuntimeError('Video writer failed')
        worker_log = (out/'yolo_worker.log').open('w')
        worker = subprocess.Popen([str(YOLO_PYTHON), str(ROOT/'yolo_worker.py'),
                                   str(YOLO_WEIGHTS_DIR/args.detector_model)],
                                  stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                  stderr=worker_log, creationflags=subprocess.CREATE_NO_WINDOW)
        ready = json.loads(worker.stdout.readline())
        if not ready.get('ready') or ready.get('person_class') != 0:
            raise RuntimeError(f'YOLO person initialization failed: {ready}')
        tracker = Tracker(config['tracking'])
        initial_time = world.current_time
        last_exposure = -math.inf
        first_detection = first_risk = first_collision = None
        frame_id = 0
        rows, gt_rows = [], []
        print('THIRD_PERSON_RUN', args.scenario, str(out), flush=True)
        start_wall = time.perf_counter()
        for count in range(round(args.seconds*60)):
            elapsed = world.current_time-initial_time
            people.step(elapsed)
            command = [ROBOT_SPEED if robot.position[0] < 9. else 0., 0.]
            robot.apply_wheel_actions(wheel.forward(command=np.asarray(command)))
            robot.advance(command, 1/60)
            world.step(render=True)
            elapsed = world.current_time-initial_time
            human_xy = [[p.x, p.y] for p in (a.get_world_translation() for a in agents)]
            robot_xy = robot.position[:2].copy()
            distances = [float(np.linalg.norm(robot_xy-h)) for h in human_xy]
            if min(distances) < radius and first_collision is None:
                first_collision = elapsed
            gt_rows.append(dict(time=elapsed, robot_xy=robot_xy.tolist(),
                                human_xy=human_xy, distances_m=distances))
            if count % 6:
                continue
            raw, _ = sensor.get_data('rgb')
            depth_raw, _ = sensor.get_data('distance_to_image_plane')
            if raw is None or depth_raw is None:
                continue
            ref = reference.get_data()
            stamp = float(ref['referenceTimeNumerator'])/float(ref['referenceTimeDenominator'])
            if stamp <= initial_time+1e-7 or stamp <= last_exposure+1e-9:
                continue
            last_exposure = stamp
            frame_id += 1
            rgb = raw.numpy()[:, :, :3].astype(np.uint8).copy()
            depth = depth_raw.numpy().squeeze().copy()
            params = {k: (v.copy() if hasattr(v, 'copy') else v) for k, v in parameters.get_data().items()}
            if frame_id == 1:
                (out/'camera_params.json').write_text(json.dumps({
                    'fixed_setup': CAMERA, 'sensor_params': {k: (v.tolist() if hasattr(v, 'tolist') else v)
                                                         for k, v in params.items()}}, indent=2))
            meta = json.dumps(dict(frame_id=frame_id, sim_time=stamp)).encode()
            payload = struct.pack('<I', len(meta))+meta+rgb.tobytes()
            worker.stdin.write(struct.pack('<I', len(payload))+payload)
            worker.stdin.flush()
            response = json.loads(worker.stdout.readline())
            if response['frame_id'] != frame_id or response['sim_time'] != stamp:
                raise RuntimeError('YOLO exposure mismatch')
            if response['boxes'] and first_detection is None:
                first_detection = elapsed
            measurements, observations = project_people(response, depth, params, config['depth'])
            tracks = tracker.update(stamp, measurements)
            ego_velocity = robot.get_linear_velocity()[:2]
            # Exposure-aligned KF state is propagated causally to the current
            # response/ego-state time; the lag is usually one or two physics ticks.
            age = max(0., world.current_time-stamp)
            risks = []
            for track in tracks:
                if track['miss_count'] or track['age'] < 3:
                    continue
                current_state = np.asarray(track['state'], dtype=float).copy()
                current_state[:2] += current_state[2:]*age
                risks.append(dict(track_id=track['track_id'],
                                  **predictions(current_state, robot_xy, ego_velocity, radius)))
            if any(r['risk']=='COLLISION_RISK' for r in risks) and first_risk is None:
                first_risk = elapsed
            row = dict(frame_id=frame_id, exposure_time=stamp-initial_time,
                       response_time=elapsed, boxes=response['boxes'], confidence=response['conf'],
                       observations=observations, tracks=tracks, risks=risks,
                       robot_ego_xy=robot_xy.tolist(), robot_ego_velocity=ego_velocity.tolist(),
                       predictor_time=elapsed, exposure_to_predictor_s=age,
                       yolo_ms=response['ms'])
            rows.append(row)
            draw_frame(rgb, response, observations, tracks, risks, elapsed, writer)
            if frame_id in (1, 30, 80):
                cv2.imwrite(str(out/f'camera_{frame_id:04d}.png'), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
            if count % 600 == 0:
                print('PROGRESS', args.scenario, round(elapsed, 2), 'frames', frame_id, flush=True)
        writer.release(); writer = None
        (out/'perception.json').write_text(json.dumps(rows))
        (out/'evaluation_gt.json').write_text(json.dumps(gt_rows))
        wall_seconds = time.perf_counter()-start_wall
        summary = dict(scenario=args.scenario, seed=args.seed, camera=CAMERA,
                       yolo=ready, robot_speed_m_s=ROBOT_SPEED,
                       collision_threshold_m=radius,
                       warning_threshold_m=radius+WARNING_MARGIN,
                       human_specs=specs, first_detection_time=first_detection,
                       first_risk_time=first_risk, actual_collision_time=first_collision,
                       predicted_collision=first_risk is not None,
                       actual_collision=first_collision is not None,
                       lead_time_s=(first_collision-first_risk if first_collision is not None and first_risk is not None else None),
                       min_actual_gt_distance_m=min(min(r['distances_m']) for r in gt_rows),
                       exposure_count=len(rows), wall_seconds=wall_seconds,
                       yolo_responses_per_wall_s=len(rows)/wall_seconds,
                       predictor_inputs=['YOLO person boxes', 'third-person depth', 'CV-KF tracks', 'robot ego state'],
                       gt_in_predictor=False, video=str(out/'THIRD_PERSON_COLLISION_PREDICTION.mp4'))
        (out/'summary.json').write_text(json.dumps(summary, indent=2))
        print('RESULT', json.dumps({k: summary[k] for k in ('scenario', 'exposure_count',
              'actual_collision', 'predicted_collision', 'lead_time_s', 'min_actual_gt_distance_m')}), flush=True)
        return summary
    finally:
        if writer is not None:
            writer.release()
        if worker is not None:
            worker.stdin.close()
            worker.terminate()
            try: worker.wait(timeout=10)
            except subprocess.TimeoutExpired: worker.kill()
        if worker_log is not None:
            worker_log.close()
        app.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--scenario', required=True, choices=[
        'headon_collision', 'perpendicular_collision', 'diagonal_collision',
        'cutin_collision', 'rear_end_collision', 'perpendicular_nearmiss',
        'diagonal_nearmiss', 'cutin_nearmiss', 'multi_person_collision'])
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--seed', type=int, default=17)
    parser.add_argument('--seconds', type=float, default=35.)
    parser.add_argument('--detector-model', choices=['yolo11n.pt', 'yolo26n.pt'],
                        default='yolo26n.pt')
    run(parser.parse_args())
