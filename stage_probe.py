"""Isaac 6 staged RGB-D tests and opt-in causal tracking/brake baseline."""
import argparse
import csv
import ctypes
import json
import subprocess
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent


class MemoryStatus(ctypes.Structure):
    _fields_ = [("length", ctypes.c_ulong), ("load", ctypes.c_ulong)] + [
        (name, ctypes.c_ulonglong) for name in
        ("total_phys", "avail_phys", "total_page", "avail_page", "total_virtual", "avail_virtual", "avail_extended")]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--stage', type=int, choices=range(8), required=True)
    parser.add_argument('--robot-model',choices=['jetbot','a300'])
    parser.add_argument('--motion-mode',choices=['physical','kinematic'])
    parser.add_argument('--demo-scene',action='store_true')
    parser.add_argument('--demo-controller',action='store_true')
    parser.add_argument('--navwareset-scene',action='store_true')
    parser.add_argument('--classic-scene',action='store_true')
    parser.add_argument('--classic-scenario',choices=['headon','crossing','static_obstruction','overtaking','blind_corner'],default='headon')
    parser.add_argument('--classic-diagnostic',action='store_true')
    parser.add_argument('--navwareset-scenario',choices=['frontal','obstruction','blind_corner','perpendicular','circular'],default='frontal')
    parser.add_argument('--robot-behavior',choices=['social','non-social','social_nav','active_passing'],default='social')
    parser.add_argument('--social-experiment',action='store_true')
    parser.add_argument('--seed',type=int,default=17)
    parser.add_argument('--run-phase',choices=['development','benchmark','regression'],default='development')
    parser.add_argument('--motion-test',action='store_true')
    parser.add_argument('--motion-matrix',action='store_true')
    parser.add_argument('--curve-demo',action='store_true')
    parser.add_argument('--physics-audit',action='store_true')
    parser.add_argument('--wheel-damping',type=float)
    parser.add_argument('--tire-friction',type=float,nargs=2)
    parser.add_argument('--tire-restitution-min',action='store_true')
    parser.add_argument('--solver-iterations',type=int,nargs=2)
    parser.add_argument('--stationary',action='store_true')
    parser.add_argument('--seconds', type=float, default=180)
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--minimal', action='store_true')
    parser.add_argument('--label', default='')
    parser.add_argument('--distance-check', action='store_true')
    parser.add_argument('--humans', type=int, choices=(1,2), default=1)
    parser.add_argument('--fractional-opacity', action='store_true')
    parser.add_argument('--construction-actor', action='store_true')
    parser.add_argument('--position-check', action='store_true')
    parser.add_argument('--tracking-check', action='store_true')
    parser.add_argument('--scenario',choices=['crossing','headon'])
    parser.add_argument('--avoidance',action='store_true')
    parser.add_argument('--sim-seconds',type=float,default=None)
    parser.add_argument('--depth-audit',action='store_true')
    parser.add_argument('--observation-audit',action='store_true')
    parser.add_argument('--hfov',type=float,default=None)
    parser.add_argument('--second-view',action='store_true')
    parser.add_argument('--record-demo',action='store_true')
    parser.add_argument('--record-first-person',action='store_true')
    parser.add_argument('--robust-depth',action='store_true')
    parser.add_argument('--lateral-avoidance',action='store_true')
    args, _ = parser.parse_known_args()
    arena=args.navwareset_scene or args.classic_scene
    if args.record_first_person and not (args.classic_scene or (args.navwareset_scene and args.navwareset_scenario in ('blind_corner','circular') and args.robot_behavior in ('social','social_nav'))):
        parser.error('First-person recording targets NavWareSet blind_corner or circular, social only')
    if args.sim_seconds is None:args.sim_seconds=70. if args.classic_scene else 55. if args.navwareset_scene else 40. if args.demo_scene else 20.
    configuration = json.loads((ROOT/'config.yaml').read_text())
    if args.robot_behavior=='social_nav' or args.classic_scene:args.social_experiment=True
    if args.social_experiment:
        if not arena:parser.error('--social-experiment requires a benchmark scene')
        import random
        import numpy as np
        random.seed(args.seed);np.random.seed(args.seed)
    from robot_backend import settings,WheelAdapter
    robot_config = settings(args.robot_model or configuration.get('robot_model','jetbot'))
    motion_mode=args.motion_mode or configuration.get('motion_mode','physical')
    if robot_config['name']=='jetbot':motion_mode='physical'
    robot_config['motion_mode']=motion_mode
    if args.demo_scene:
        args.humans=2;args.tracking_check=True;args.robust_depth=True
        args.demo_controller=args.demo_controller or configuration.get('controller_mode')=='demo'
        args.record_demo=args.record_demo or configuration.get('record_demo',False)
    if args.navwareset_scene:
        if args.demo_scene or args.scenario or args.hfov is not None or args.second_view:parser.error('NavWareSet scene cannot mix old scene or camera overrides')
        from navwareset_scene import preset
        nav_config=preset(args.navwareset_scenario)
        if args.social_experiment:
            nav_config['repeat_seed']=args.seed
            nav_config['seed_scope']='Python/NumPy repeat seed; frozen character Randomizer(17), routes and assets unchanged'
        args.humans=len(nav_config['humans']);args.tracking_check=True;args.robust_depth=True;args.demo_controller=True
        if robot_config['name']!='a300' or motion_mode!='kinematic':parser.error('NavWareSet requires kinematic A300')
    if args.classic_scene:
        if args.navwareset_scene or args.demo_scene or args.scenario or args.hfov is not None or args.second_view:parser.error('Classic cannot mix scenes or camera overrides')
        if args.robot_behavior not in ('social','social_nav','active_passing'):parser.error('Unsupported classic controller')
        from classic_single_scene import preset
        nav_config=preset(args.classic_scenario)
        args.humans=1;args.tracking_check=True;args.robust_depth=True;args.demo_controller=True
        if robot_config['name']!='a300' or motion_mode!='kinematic':parser.error('Classic requires kinematic A300')
    if args.demo_controller and not (args.demo_scene or arena):parser.error('--demo-controller requires a demo scene')
    if args.demo_controller and motion_mode!='kinematic':parser.error('Demo controller requires A300 kinematic mode; physical diagnostics remain available without --demo-controller.')
    if args.wheel_damping is not None: robot_config['wheel_damping']=args.wheel_damping
    if args.tire_friction is not None: robot_config['tire_friction']=args.tire_friction
    if args.tire_restitution_min: robot_config['tire_restitution_min']=True
    if args.solver_iterations: robot_config['solver_iterations']=args.solver_iterations
    from a300_motion_validation import SHORT,MATRIX,DEMO,PhysicsAudit
    motion_commands=MATRIX if args.motion_matrix else DEMO if args.curve_demo else SHORT
    physical_audit=None
    configuration['control']['robot_radius_m'] = robot_config['conservative_radius_m']
    configuration['control']['wheel_radius_m'] = robot_config['wheel_radius_m']
    configuration['control']['track_width_m'] = robot_config['track_width_m']
    if args.lateral_avoidance and not (args.avoidance and args.robust_depth):
        parser.error('--lateral-avoidance requires --avoidance --robust-depth')
    if args.scenario:
        args.tracking_check = True
    if args.tracking_check:
        args.position_check = True
    if args.position_check and args.stage != 7:
        parser.error('Position/tracking validation uses the moving-camera Stage 7 setup.')
    if args.avoidance and not args.scenario:
        parser.error('--avoidance requires --scenario crossing or headon.')
    if args.minimal and args.stage >= 3:
        parser.error('The trimmed experience failed camera validation. Omit --minimal for stages 3-7.')
    output = ROOT / 'outputs' / (f'stage{args.stage}' + ('_minimal' if args.minimal else '') + args.label)
    if args.navwareset_scene:
        parent=ROOT/'outputs'/'navwareset_style'/args.navwareset_scenario/args.robot_behavior
        index=1
        while (parent/f'run_{index:02d}').exists():index+=1
        output=parent/f'run_{index:02d}'
        if args.social_experiment:
            name='baseline' if args.robot_behavior=='social' else args.robot_behavior
            parent=ROOT/'outputs'/'social_navigation'/args.run_phase/args.navwareset_scenario/name/f'seed_{args.seed}'
            index=1
            while (parent/f'run_{index:02d}').exists():index+=1
            output=parent/f'run_{index:02d}'
        print('NAVWARESET_OUTPUT',str(output),flush=True)
    if args.classic_scene:
        name='diagnostic' if args.classic_diagnostic else 'baseline' if args.robot_behavior=='social' else args.robot_behavior
        experiment='active_passing' if args.robot_behavior=='active_passing' else 'classic_single_pedestrian'
        parent=ROOT/'outputs'/experiment/args.run_phase/args.classic_scenario/name/f'seed_{args.seed}'
        index=1
        while (parent/f'run_{index:02d}').exists():index+=1
        output=parent/f'run_{index:02d}'
        print('CLASSIC_OUTPUT',str(output),flush=True)
    output.mkdir(parents=True, exist_ok=True)
    samples, stop = [], threading.Event()
    phase = ['startup']

    def monitor():
        with (output / 'resources.csv').open('w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['unix_time', 'phase', 'gpu_memory_used_mib_total', 'gpu_util_percent', 'system_ram_used_gib'])
            while not stop.is_set():
                try:
                    result = subprocess.check_output([
                        'nvidia-smi', '--query-gpu=memory.used,utilization.gpu',
                        '--format=csv,noheader,nounits'], text=True,
                        creationflags=subprocess.CREATE_NO_WINDOW, timeout=8)
                    gpu = [float(v.strip()) for v in result.splitlines()[0].split(',')]
                    mem = MemoryStatus()
                    mem.length = ctypes.sizeof(mem)
                    if not ctypes.windll.kernel32.GlobalMemoryStatusEx(ctypes.byref(mem)):
                        raise OSError('GlobalMemoryStatusEx failed')
                    row = [time.time(), phase[0], *gpu, (mem.total_phys-mem.avail_phys)/2**30]
                    samples.append(row)
                    writer.writerow(row)
                    f.flush()
                except Exception as exc:
                    print('MONITOR_ERROR', repr(exc), flush=True)
                stop.wait(1)

    thread = threading.Thread(target=monitor, daemon=True)
    thread.start()
    from isaacsim import SimulationApp
    app = SimulationApp({'headless': args.headless, 'width': 960, 'height': 540,
                         'window_width': 960, 'window_height': 540,
                         'renderer': 'RayTracedLighting', 'multi_gpu': False,
                         'active_gpu': 0, 'physics_gpu': 0,
                         'extra_args': (['--enable', 'isaacsim.sensors.experimental.rtx'] if args.stage >= 3 else [])
                           + (['--enable', 'isaacsim.replicator.agent.core'] if args.stage >= 5 else [])},
                         experience=str(ROOT/'minimal.kit') if args.minimal else '')
    worker = None
    bridge = None
    demo_writer = None
    first_person = None
    if args.record_first_person:
        from first_person_video import FirstPersonVideo
        filename = 'A300_THREE_PEOPLE_FIRST_PERSON.mp4' if args.navwareset_scenario=='circular' else 'A300_BLIND_CORNER_FIRST_PERSON.mp4'
        if args.classic_scene:filename=f'CLASSIC_{args.classic_scenario.upper()}_FIRST_PERSON.mp4'
        first_person = FirstPersonVideo((output/'first_person') if args.social_experiment or args.classic_scene else output.parent/'first_person',filename,args.humans)
    try:
        if args.fractional_opacity:
            import carb
            carb.settings.get_settings().set('/rtx/raytracing/fractionalCutoutOpacity', True)
        from omni.kit.viewport.utility import get_active_viewport
        viewport = get_active_viewport()
        if viewport is None:
            raise RuntimeError('No active viewport for renderer validation')
        viewport.set_texture_resolution((640, 360))
        if arena and args.headless:
            viewport.updates_enabled=False  # Unused UI only; sensor render products remain enabled.
            nav_config['unused_viewport_disabled']=True
        world, robot, controller, camera = None, None, None, None
        camera_frames = 0
        depth_checks = []
        motion = []
        backend_rows = []
        detections = []
        position_rows = []
        gt_history = []
        people_history = []
        if args.tracking_check:
            from tracker import Tracker,BrakeController,LateralController,person_depth
            configuration['control']['measurement_sigma_m'] = configuration['tracking']['measurement_sigma_m']
            configuration['control']['association_gate_m'] = configuration['tracking']['association_gate_m']
            if not args.robust_depth:
                configuration['tracking'].pop('innovation_gate_chi2',None)
            tracker = Tracker(configuration['tracking'])
            brake = (LateralController if args.lateral_avoidance else BrakeController)(configuration['control'])
            depth_ok,depth_confidence,measurement_rejected = True,None,False
            current_tracks,measurement_time = [],None
            risk_rows = []
        actor_asset = None
        if args.stage >= 1:
            from isaacsim.core.api import World
            world = World(stage_units_in_meters=1.0, physics_dt=1/60, rendering_dt=1/60)
            world.scene.add_default_ground_plane()
            if args.demo_scene:
                from demo_scene import build,DemoController
                if (ROOT/'indoor_demo_scene.usd').exists():
                    from pxr import UsdGeom
                    UsdGeom.Imageable(world.stage.GetPrimAtPath('/World/defaultGroundPlane')).MakeInvisible()
                    world.stage.DefinePrim('/Indoor','Xform').GetReferences().AddReference(str(ROOT/'indoor_demo_scene.usd'))
                else:build(world.stage,ROOT/'indoor_demo_scene.usd')
                demo_control=DemoController();demo_rows=[]
            if args.navwareset_scene:
                from navwareset_scene import build,controller as nav_controller
                nav_geometry=build(world.stage,ROOT/'navwareset_style_scene.usd')
                if args.robot_behavior=='social_nav':
                    from social_controller import SocialController
                    demo_control=SocialController(configuration['social_navigation'],nav_config['robot_waypoints'],nav_geometry,
                        robot_config['conservative_radius_m'],{'linear_accel_m_s2':.5,'linear_decel_m_s2':.8,'angular_accel_rad_s2':1.2})
                else:demo_control=nav_controller(nav_config,args.robot_behavior)
                demo_rows=[]
            if args.classic_scene:
                from classic_single_scene import build,make_controller
                nav_geometry=build(world.stage,ROOT/'classic_single_scene.usd',nav_config)
                if args.robot_behavior=='active_passing':
                    from active_passing_controller import ActivePassingController
                    demo_control=ActivePassingController(configuration['social_navigation'],nav_config['robot_waypoints'],nav_geometry,
                        robot_config['conservative_radius_m'],dict(linear_accel_m_s2=.5,linear_decel_m_s2=.8,angular_accel_rad_s2=1.2))
                    import inspect
                    (output/'active_controller_source.py').write_text(inspect.getsource(__import__('active_passing_controller')),encoding='utf8')
                else:demo_control=make_controller(nav_config,nav_geometry,configuration,robot_config,args.robot_behavior,args.classic_diagnostic)
                demo_rows=[]
            if args.stage >= 2:
                import numpy as np
                from isaacsim.core.utils.extensions import enable_extension
                enable_extension('isaacsim.robot.wheeled_robots')
                from isaacsim.robot.wheeled_robots.robots import WheeledRobot
                from isaacsim.robot.wheeled_robots.controllers.differential_controller import DifferentialController
                from isaacsim.storage.native import get_assets_root_path
                from pxr import UsdLux
                asset = robot_config.get('asset') or get_assets_root_path() + '/Isaac/Robots/NVIDIA/Jetbot/jetbot.usd'
                print('ROBOT_ASSET', asset, flush=True)
                if motion_mode=='kinematic':
                    world.stage.DefinePrim(robot_config['prim_path'],'Xform').GetReferences().AddReference(asset)
                else:
                    robot = world.scene.add(WheeledRobot(robot_config['prim_path'], name=robot_config['name'],
                        wheel_dof_names=robot_config['joints'],
                        create_robot=True, usd_path=asset, position=np.array([0., 0., robot_config['spawn_z_m']])))
                if robot_config['name']=='a300':
                    from robot_backend import configure_a300
                    configure_a300(world.stage,robot_config,args.stage>=3)
                    if motion_mode=='kinematic':
                        from robot_backend import KinematicA300
                        robot=KinematicA300(world.stage,robot_config)
                UsdLux.DomeLight.Define(world.stage, '/World/Light').CreateIntensityAttr(500.)
            if args.stage >= 3:
                from isaacsim.sensors.experimental.rtx import RtxCamera, CameraSensor
                from isaacsim.core.api.objects import FixedCuboid
                # Known static calibration props, never stand-ins for the human test.
                for name, pos, size, color in ([] if args.stage >= 5 else [
                    ('Near', [3., -.8, .5], [.5,.5,1.], [1.,.15,.1]),
                    ('Far', [5., .8, .75], [.5,.5,1.5], [.1,.3,1.])]):
                    world.scene.add(FixedCuboid('/World/'+name, name=name,
                        position=np.array(pos), scale=np.array(size), color=np.array(color)))
                camera = RtxCamera('/World/RobotCamera', tick_rate=10.,
                    translations=np.array(robot_config['camera_mount_m'])+np.array([0.,0.,robot_config['spawn_z_m']]), orientations=np.array([.5,.5,-.5,-.5]))
                prim = camera.prims[0]
                prim.GetAttribute('focalLength').Set(9.)
                prim.GetAttribute('horizontalAperture').Set(24.)
                prim.GetAttribute('verticalAperture').Set(13.5)
                if args.hfov is not None:
                    if not 106.<=args.hfov<=130.:
                        raise ValueError('Perspective camera HFOV must be 106–130 degrees')
                    prim.GetAttribute('horizontalAperture').Set(float(18.*np.tan(np.radians(args.hfov/2))))
                    if args.tracking_check:
                        configuration['control']['camera_half_fov_rad'] = float(np.radians(args.hfov/2))
                print('CAMERA_ORIGINAL_CLIPPING', str(prim.GetAttribute('clippingRange').Get()), flush=True)
                camera.camera.set_clipping_ranges(.05, 100.)
                cameras = [camera]
                if args.second_view:
                    side_camera = RtxCamera('/World/SideCamera',tick_rate=10.,translations=np.array([.2,0.,1.]),orientations=np.array([.5,.5,-.5,-.5]))
                    for attr in ('focalLength','horizontalAperture','verticalAperture'):
                        side_camera.prims[0].GetAttribute(attr).Set(prim.GetAttribute(attr).Get())
                    side_camera.camera.set_clipping_ranges(.05,100.)
                    cameras.append(side_camera)
                    configuration['control']['camera_yaw_offsets_rad'] = [0.,float(np.pi/2)]
            if args.stage >= 5:
                from walking_actor import load_actor
                from omni.kit.async_engine import run_coroutine
                if args.classic_scene:
                    import NavSchema
                    # Dynamic robot must not become a permanent hole in the human's static navmesh.
                    NavSchema.NavMeshExcludeAPI.Apply(world.stage.GetPrimAtPath(robot_config['prim_path']))
                task = run_coroutine(load_actor(world.stage, args.humans, args.construction_actor,'navwareset' if arena else 'demo' if args.demo_scene else args.scenario))
                deadline = time.perf_counter()+240
                while not task.done():
                    app.update()
                    if time.perf_counter() > deadline:
                        task.cancel()
                        raise TimeoutError('Official walking actor setup exceeded 240 seconds')
                actor_path = task.result()
                actor_asset = str(world.stage.GetPrimAtPath(actor_path).GetMetadata('payload'))
                print('ACTOR_LOADED', actor_path, flush=True)
            if args.physics_audit:
                physical_audit=PhysicsAudit(world.stage,robot_config['prim_path'])
            world.reset()
            if robot is not None:
                if args.solver_iterations and robot_config['name']=='a300':
                    robot_config['runtime_solver_iterations']=[int(robot.get_solver_position_iteration_count()),int(robot.get_solver_velocity_iteration_count())]
                print('ROBOT_DOF_LIMITS',robot.dof_names,robot.dof_properties['maxVelocity'].tolist(),flush=True)
                limits=[robot.dof_properties['maxVelocity'][robot.get_dof_index(j)] for j in robot_config['joints']]
                controller=WheelAdapter(robot_config,limits)
                robot_config['actual_wheel_velocity_limits_rad_s']=list(map(float,limits))
                configuration['control']['wheel_velocity_limit_rad_s']=float(min(limits))
            if camera:
                annotators = ['rgb'] + (['distance_to_image_plane'] if args.stage >= 4 else [])
                sensor = CameraSensor(camera, resolution=(360,640), annotators=annotators)
                if args.position_check:
                    import omni.replicator.core as rep
                    reference_time = rep.annotators.get('ReferenceTime')
                    camera_params = rep.annotators.get('CameraParams')
                    reference_time.attach(sensor._hydra_texture.path)
                    camera_params.attach(sensor._hydra_texture.path)
                sensor_sets = [(sensor,reference_time,camera_params)] if args.position_check else []
                if args.second_view:
                    side_sensor = CameraSensor(cameras[1],resolution=(360,640),annotators=annotators)
                    side_reference = rep.annotators.get('ReferenceTime');side_params = rep.annotators.get('CameraParams')
                    side_reference.attach(side_sensor._hydra_texture.path);side_params.attach(side_sensor._hydra_texture.path)
                    sensor_sets.append((side_sensor,side_reference,side_params))
                if not args.headless:
                    viewport.camera_path = '/World/RobotCamera'
            if args.record_demo:
                from scipy.spatial.transform import Rotation
                import cv2
                eye = np.array([5.,-7.,4.5]) if args.demo_scene else np.array([3.,-6.,4.])
                forward = (np.array([4.,0.,.5]) if args.demo_scene else np.array([1.5,0.,.5]))-eye
                if arena:
                    eye=np.array(nav_config['grs_camera']['eye']);forward=np.array(nav_config['grs_camera']['target'])-eye
                forward /= np.linalg.norm(forward)
                right = np.cross(forward,[0.,0.,1.]); right /= np.linalg.norm(right)
                up = np.cross(right,forward)
                q = Rotation.from_matrix(np.column_stack((right,up,-forward))).as_quat()
                demo_camera = RtxCamera('/World/DemoCamera',tick_rate=10.,translations=eye,orientations=np.array([q[3],*q[:3]]))
                demo_camera.prims[0].GetAttribute('focalLength').Set(18.)
                if arena:
                    demo_camera.prims[0].GetAttribute('focalLength').Set(9.)
                    nav_config['grs_camera']['focal_length_mm']=9.
                    nav_config['grs_camera']['horizontal_aperture_mm']=24.
                    nav_config['grs_camera']['render_schedule']='continuous rendering; sampled at 10 Hz'
                demo_camera.prims[0].GetAttribute('horizontalAperture').Set(24.)
                demo_camera.prims[0].GetAttribute('verticalAperture').Set(13.5)
                demo_camera.camera.set_clipping_ranges(.05,100.)
                demo_size=(960,540) if (args.demo_scene or arena) else (1280,720)
                demo_sensor = CameraSensor(demo_camera,resolution=(demo_size[1],demo_size[0]),annotators=['rgb'])
                demo_writer = cv2.VideoWriter(str(output/('curve_demo_raw.mp4' if args.curve_demo else 'avoidance_demo_raw.mp4')),cv2.VideoWriter_fourcc(*'mp4v'),10.,demo_size)
                if not demo_writer.isOpened():
                    raise RuntimeError('Demo video writer failed')
                viewport.camera_path = '/World/DemoCamera'
            for _ in range(30):
                world.step(render=True)
            if args.position_check:
                from pxr import Usd
                import omni.anim.behavior.core as bh_core
                skelroots = [p for p in Usd.PrimRange(world.stage.GetPrimAtPath(actor_path)) if p.GetTypeName() == 'SkelRoot']
                evaluation_agent = bh_core.acquire_interface().get_agent(str(skelroots[0].GetPath()))
                if evaluation_agent is None:
                    raise RuntimeError('No live behavior agent for evaluation')
                evaluation_paths = [str(p.GetPath()) for p in world.stage.Traverse() if str(p.GetPath()).startswith('/World/Characters/') and p.GetTypeName()=='SkelRoot']
                evaluation_agents = [bh_core.acquire_interface().get_agent(p) for p in evaluation_paths]
                if len(evaluation_agents)!=args.humans or any(a is None for a in evaluation_agents):
                    raise RuntimeError('Evaluation actor count mismatch')
                if args.demo_scene:
                    import carb
                    for agent,spawn,target in zip(evaluation_agents,[[2.,-3.,0.],[7.,1.25,0.]],[[2.,3.,0.],[-1.,1.25,0.]]):
                        if not agent.teleport(carb.Float3(*spawn)):raise RuntimeError('Demo actor teleport failed')
                        agent.set_speed(.65);agent.move_to(carb.Float3(*target),auto_brake=True)
                if args.navwareset_scene:
                    from navwareset_scene import ScriptedPeople
                    scripted_people=ScriptedPeople(evaluation_agents,nav_config)
                if args.classic_scene:
                    from classic_single_scene import ScriptedPeople
                    scripted_people=ScriptedPeople(evaluation_agents,nav_config)
                if args.scenario:
                    import carb
                    spawn,target = (([2.,-4.,0.],[2.,4.,0.]) if args.scenario=='crossing' else ([4.,0.,0.],[0.,0.,0.]))
                    import omni.anim.navigation.core as navigation
                    mesh = navigation.acquire_interface().get_navmesh()
                    print('SCENARIO_NAVMESH_TARGET',target,str(mesh.query_closest_point(carb.Float3(*target))),flush=True)
                    if not evaluation_agent.teleport(carb.Float3(*spawn)):
                        raise RuntimeError('Scenario initial teleport failed')
                    evaluation_agent.set_speed(.65)
                    evaluation_agent.move_to(carb.Float3(*target),auto_brake=True)
                    if args.humans==2:
                        second_agent = evaluation_agents[next(i for i,p in enumerate(evaluation_paths) if p!=str(skelroots[0].GetPath()))]
                        if not second_agent.teleport(carb.Float3(4.,-2.,0.)):
                            raise RuntimeError('Second pedestrian teleport failed')
                        second_agent.set_speed(.65)
                        second_agent.move_to(carb.Float3(0.,2.,0.),auto_brake=True)
                gp = evaluation_agent.get_world_translation()
                gt_history.append((world.current_time,np.array([gp.x,gp.y,gp.z])))
                people_history.append((world.current_time,np.array([[p.x,p.y,p.z] for p in [a.get_world_translation() for a in evaluation_agents]])))
        if args.stage >= 6:
            worker_log = (output/'yolo_worker.log').open('w')
            worker = subprocess.Popen([
                r'D:\detection\robot_human_avoidance\.venv\python.exe',
                str(ROOT/'yolo_worker.py'), r'D:\detection\robot_human_avoidance\yolo11n.pt'],
                stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=worker_log,
                creationflags=subprocess.CREATE_NO_WINDOW)
            ready = worker.stdout.readline()
            if not ready or not json.loads(ready)['ready']:
                raise RuntimeError('YOLO worker did not initialize')
            print('YOLO_READY', ready.decode().strip(), flush=True)
            if args.tracking_check:
                from yolo_worker import InferenceBridge
                bridge = InferenceBridge(worker)
        phase[0] = 'measurement'
        start = time.perf_counter()
        initial_sim = world.current_time if world else None
        count = 0
        last_exposure_stamp = {}
        repeated_exposures = 0
        annotated_demo_states=set()
        demo_finished_at=None
        while time.perf_counter()-start < args.seconds:
            if not app.is_running():
                raise RuntimeError('Application closed before measurement completed')
            if world:
                if arena:scripted_people.step(world.current_time-initial_sim)
                if args.distance_check:
                    distance = [1.5,2.,3.,4.,5.][min(int((world.current_time-initial_sim)/3),4)]
                    camera.set_local_poses(translations=np.array([3.-distance,0.,1.]))
                if robot is not None:
                    cycle = (world.current_time-initial_sim) % 7
                    command = ([.3, 0.] if cycle < 3 else ([0., 0.] if cycle < 5 else [0., .5])) if args.stage == 2 else [0.,0.]
                    if args.stage == 7:
                        command = [.3 if (world.current_time-initial_sim)%12 < 6 else -.3, 0.]
                        if args.scenario:
                            pos,robot_q = robot.get_world_pose()
                            heading = float(np.arctan2(2*(robot_q[0]*robot_q[3]+robot_q[1]*robot_q[2]),1-2*(robot_q[2]**2+robot_q[3]**2)))
                            speed,risks = brake.step(world.current_time,1/60,current_tracks,measurement_time,
                                pos[:2],np.asarray(robot.get_linear_velocity())[:2],
                                **({'heading':heading,'depth_ok':depth_ok} if args.lateral_avoidance else {}))
                            command = [speed if args.avoidance else configuration['control']['nominal_speed_m_s'],brake.omega if args.lateral_avoidance else 0.]
                    if camera is not None and not args.distance_check:
                        pos, q = robot.get_world_pose()
                        from scipy.spatial.transform import Rotation
                        rotation = Rotation.from_quat([q[1],q[2],q[3],q[0]])
                        mount = Rotation.from_quat([.5,-.5,-.5,.5])
                        optical = (rotation*mount).as_quat()
                        camera.set_world_poses(positions=pos+rotation.apply(robot_config['camera_mount_m']),
                            orientations=np.array([optical[3],*optical[:3]]))
                        if args.second_view:
                            side_optical = (rotation*Rotation.from_euler('z',np.pi/2)*mount).as_quat()
                            cameras[1].set_world_poses(positions=pos+rotation.apply(robot_config['camera_mount_m']),orientations=np.array([side_optical[3],*side_optical[:3]]))
                    if args.stationary:
                        command=[0.,0.]
                    if args.motion_test or args.motion_matrix or args.curve_demo:
                        segment=min(int((world.current_time-initial_sim)/4),len(motion_commands)-1)
                        command=motion_commands[segment]
                    if args.demo_controller:
                        if args.robot_behavior in ('social_nav','active_passing') and not args.classic_diagnostic:
                            command=demo_control.step(world.current_time,current_tracks,measurement_time,robot.get_world_pose()[0],robot.yaw,robot.v,robot.w)
                        else:command=demo_control.step(world.current_time,current_tracks,measurement_time,robot.get_world_pose()[0],robot.yaw)
                    robot.apply_wheel_actions(controller.forward(command=np.array(command)))
                    if motion_mode=='kinematic':
                        robot.advance(command,world.get_physics_dt())
                        if camera is not None:
                            pos,q=robot.get_world_pose()
                            rotation=Rotation.from_quat([q[1],q[2],q[3],q[0]])
                            optical=(rotation*mount).as_quat()
                            camera.set_world_poses(positions=pos+rotation.apply(robot_config['camera_mount_m']),orientations=np.array([optical[3],*optical[:3]]))
                world.step(render=True)
                if demo_writer is not None and count % 6 == 0:
                    demo_data,_ = demo_sensor.get_data('rgb')
                    if demo_data is not None:
                        frame = cv2.cvtColor(demo_data.numpy()[:,:,:3].astype(np.uint8),cv2.COLOR_RGB2BGR)
                        cv2.rectangle(frame,(0,0),(1280,76),(27,35,43),-1)
                        title='Isaac Sim | A300 open-loop curve test' if args.curve_demo else 'Isaac Sim | Robot-human avoidance demonstration'
                        detail=f'v={command[0]:.2f} m/s  omega={command[1]:.2f} rad/s | No avoidance' if args.curve_demo else f'State: {brake.state} | Development demo - not safety certification'
                        if args.demo_controller:
                            title='Husky A300 | RGB-D human avoidance'
                            detail=f'Kinematic | {demo_control.state} | v={robot.v:.2f} m/s | Tracks: {len(current_tracks)}'
                        if args.navwareset_scene:title=f'NavWareSet-style | {args.navwareset_scenario} | {args.robot_behavior}'
                        if args.classic_scene:title=f'Classic | {args.classic_scenario} | {"OPEN LOOP" if args.classic_diagnostic else args.robot_behavior}'
                        cv2.putText(frame,title,(24,30),cv2.FONT_HERSHEY_SIMPLEX,.8,(255,255,255),2)
                        cv2.putText(frame,f'Simulation time: {world.current_time-initial_sim:.1f}s   {detail}',(24,61),cv2.FONT_HERSHEY_SIMPLEX,.55,(220,220,220),1)
                        demo_writer.write(frame)
                if args.position_check:
                    gp = evaluation_agent.get_world_translation()
                    gt_history.append((world.current_time,np.array([gp.x,gp.y,gp.z])))
                    people_history.append((world.current_time,np.array([[p.x,p.y,p.z] for p in [a.get_world_translation() for a in evaluation_agents]])))
                    if args.demo_controller:
                        rp,_=robot.get_world_pose()
                        demo_rows.append({'time':world.current_time-initial_sim,'state':demo_control.state,'command':command,'robot_position':rp.tolist(),'speed':robot.v,'omega':robot.w,'estimated_risks':demo_control.risks,'evaluation_people':people_history[-1][1].tolist(),'evaluation_min_distance':min(float(np.linalg.norm(rp[:2]-p[:2])) for p in people_history[-1][1]),'route_done':demo_control.done})
                        if arena:demo_rows[-1]['robot_yaw']=robot.yaw
                        if args.classic_scene:demo_rows[-1]['estimated_tracks']=current_tracks
                        if args.robot_behavior in ('social_nav','active_passing') and not args.classic_diagnostic:demo_rows[-1]['selected_primitive']=demo_control.selected
                    if args.scenario:
                        rp,rq = robot.get_world_pose()
                        risk_rows.append({'sim_time':world.current_time,'robot_position':rp.tolist(),
                            'robot_velocity':robot.get_linear_velocity().tolist(),'robot_angular_velocity':robot.get_angular_velocity().tolist(),
                            'actual_v':float(np.linalg.norm(robot.get_linear_velocity()[:2])),
                            'actual_omega':float(robot.get_angular_velocity()[2]),
                            'requested_wheel_rad_s':controller.requested.tolist(),
                            'clipped_wheel_rad_s':controller.clipped.tolist(),
                            'actual_joint_velocity_rad_s':robot.get_joint_velocities().tolist(),
                            'evaluation_gt_human_xyz':[gp.x,gp.y,gp.z],
                            'evaluation_gt_distance_m':min(float(np.linalg.norm(rp[:2]-np.array([p.x,p.y]))) for p in [a.get_world_translation() for a in evaluation_agents]),
                            'controller_state':brake.state if args.avoidance else 'DISABLED',
                            'command':command,'risks':risks,'depth_confidence':depth_confidence,
                            'measurement_rejected':measurement_rejected,**getattr(brake,'diagnostics',{})})
                if robot is not None and count % 6 == 0:
                    pos, quat = robot.read_usd_pose() if motion_mode=='kinematic' else robot.get_world_pose()
                    motion.append([world.current_time-initial_sim, *command, *pos.tolist(), *quat.tolist()])
                    yaw=float(np.arctan2(2*(quat[0]*quat[3]+quat[1]*quat[2]),1-2*(quat[2]**2+quat[3]**2)))
                    velocity=robot.get_linear_velocity()
                    backend_rows.append({'time':world.current_time-initial_sim,'command':list(command),'position':pos.tolist(),
                      'yaw':yaw,'actual_forward_m_s':float(velocity[0]*np.cos(yaw)+velocity[1]*np.sin(yaw)),
                      'actual_yaw_rate':float(robot.get_angular_velocity()[2]),'linear_velocity':velocity.tolist(),
                      'requested_wheel_rad_s':controller.requested.tolist(),'clipped_wheel_rad_s':controller.clipped.tolist(),
                      'actual_joint_velocity':robot.get_joint_velocities().tolist()})
                    if args.physics_audit or args.motion_test or args.motion_matrix or args.curve_demo:
                        from scipy.spatial.transform import Rotation
                        rp=Rotation.from_quat([quat[1],quat[2],quat[3],quat[0]]).as_euler('xyz')
                        backend_rows[-1].update(lateral_velocity_m_s=float(-velocity[0]*np.sin(yaw)+velocity[1]*np.cos(yaw)),roll=float(rp[0]),pitch=float(rp[1]))
                        try:
                            backend_rows[-1]['measured_joint_efforts']=robot.get_measured_joint_efforts().tolist()
                        except (AttributeError,RuntimeError) as exc:
                            backend_rows[-1]['measured_joint_efforts']=str(exc)
                if camera is not None and count % 6 == 0:
                    view_index = (count//6)%len(sensor_sets) if args.second_view else 0
                    if args.second_view and current_tracks:
                        rp,rq = robot.get_world_pose()
                        yaw = float(np.arctan2(2*(rq[0]*rq[3]+rq[1]*rq[2]),1-2*(rq[2]**2+rq[3]**2)))
                        target_track = next((t for t in current_tracks if t['track_id']==brake.active_id),current_tracks[0])
                        target_xy = np.asarray(target_track['state'][:2])+np.asarray(target_track['state'][2:])*max(0.,world.current_time-measurement_time)
                        bearing = np.arctan2(target_xy[1]-rp[1],target_xy[0]-rp[0])-yaw
                        view_index = min(range(2),key=lambda j:abs(np.arctan2(np.sin(bearing-j*np.pi/2),np.cos(bearing-j*np.pi/2))))
                    if args.second_view:
                        sensor,reference_time,camera_params = sensor_sets[view_index]
                    data, _ = sensor.get_data('rgb')
                    rgba = data.numpy() if data is not None else None
                    if rgba is not None and rgba.size:
                        camera_frames += 1
                        if worker is not None:
                            import struct
                            rgb = rgba[:,:,:3].astype(np.uint8)
                            # Snapshot both channels BEFORE inference; no world.step between reads.
                            depth_data, _ = sensor.get_data('distance_to_image_plane')
                            depth_values = depth_data.numpy().squeeze().copy() if depth_data is not None else None
                            stamp = world.current_time
                            if args.position_check:
                                rt = reference_time.get_data()
                                stamp = float(rt['referenceTimeNumerator'])/float(rt['referenceTimeDenominator'])
                                # Reject the pre-reset/pre-teleport render still held by the sensor.
                                if stamp <= initial_sim+1e-7:
                                    count += 1
                                    continue
                                if stamp <= last_exposure_stamp.get(view_index,-float('inf'))+1e-9:
                                    repeated_exposures += 1
                                    count += 1
                                    continue
                                last_exposure_stamp[view_index]=stamp
                                params = {k:(v.copy() if hasattr(v,'copy') else v) for k,v in camera_params.get_data().items()}
                                if args.observation_audit and 2.8<=stamp<=4.4:
                                    np.savez_compressed(output/f'exposure_{camera_frames:04d}.npz',rgb=rgb,depth=depth_values,
                                        sim_time=stamp,robot_pose=robot.get_world_pose()[0],
                                        camera_view=np.asarray(params['cameraViewTransform']),
                                        aperture=np.asarray(params['cameraAperture']),focal=params['cameraFocalLength'])
                                if not (output/'camera_params.json').exists():
                                    (output/'camera_params.json').write_text(json.dumps({k:(v.tolist() if hasattr(v,'tolist') else v) for k,v in params.items()},indent=2))
                                from pxr import UsdGeom, Usd
                                # Evaluation-only snapshot; never supplied to detector or backprojection.
                                # Evaluation interpolates only its own runtime GT history to exposure time.
                                times = np.array([g[0] for g in gt_history])
                                positions = np.array([g[1] for g in gt_history])
                                gt_xyz = np.array([np.interp(stamp,times,positions[:,j]) for j in range(3)])
                            if first_person is not None:
                                exposure_row = next((r for r in reversed(demo_rows) if r['time'] <= stamp-initial_sim+1e-7), demo_rows[0])
                                first_person.capture(rgb,camera_frames,stamp,stamp-initial_sim,exposure_row['state'],exposure_row['speed'],exposure_row.get('selected_primitive'))
                            if args.classic_scene and args.classic_scenario=='blind_corner':
                                from PIL import Image
                                (output/'visibility_rgb').mkdir(exist_ok=True)
                                Image.fromarray(rgb).save(output/'visibility_rgb'/f'{stamp-initial_sim:09.4f}.png')
                            if bridge is not None:
                                completed = bridge.poll()
                                bridge.submit(rgb.copy(),camera_frames,stamp,(rgb.copy(),depth_values,params,view_index))
                                if completed is None:
                                    count += 1
                                    continue
                                response,(rgb,depth_values,params,response_view) = completed
                                response['camera_index'] = response_view
                                stamp = response['sim_time']
                                gt_xyz = np.array([np.interp(stamp,times,positions[:,j]) for j in range(3)])
                            else:
                                meta = json.dumps({'frame_id':camera_frames,'sim_time':stamp}).encode()
                                payload = struct.pack('<I',len(meta))+meta+rgb.tobytes()
                                worker.stdin.write(struct.pack('<I',len(payload))+payload)
                                worker.stdin.flush()
                                response = json.loads(worker.stdout.readline())
                                assert response['frame_id'] == camera_frames and response['sim_time'] == stamp
                            if args.position_check and depth_values is not None:
                                aperture = np.asarray(params['cameraAperture'])
                                offsets = np.asarray(params['cameraApertureOffset'])
                                resolution = np.asarray(params['renderProductResolution'])
                                focal = float(params['cameraFocalLength'])
                                fx,fy = focal*resolution/aperture
                                cx,cy = resolution/2 + np.array([-1,1])*offsets*resolution/aperture
                                view = np.asarray(params['cameraViewTransform']).reshape(4,4)
                                world_from_camera_row = np.linalg.inv(view)
                                measurements = []
                                response['observations'] = []
                                response['depth_diagnostics'] = []
                                measurement_rows = []
                                for box,confidence in zip(response['boxes'],response['conf']):
                                    x1,y1,x2,y2 = box
                                    u,v = (x1+x2)/2,(y1+y2)/2
                                    xa,xb = np.clip([int(u-.2*(x2-x1)),int(u+.2*(x2-x1))+1],0,640)
                                    ya,yb = np.clip([int(v-.2*(y2-y1)),int(v+.2*(y2-y1))+1],0,360)
                                    values = depth_values[ya:yb,xa:xb]
                                    values = values[np.isfinite(values)&(values>.05)&(values<100)]
                                    if len(values)<3:
                                        continue
                                    z = float(np.median(values))
                                    legacy_local = np.array([(u-cx)*z/fx,-(v-cy)*z/fy,-z,1.])
                                    legacy_xyz = (legacy_local @ world_from_camera_row)[:3]
                                    diagnostic = person_depth(depth_values,box,configuration['depth'],(fx,fy,cx,cy)) if args.robust_depth else {'accepted':True,'depth_confidence':1.}
                                    response['depth_diagnostics'].append(diagnostic)
                                    if args.robust_depth:
                                        if 'depth_m' not in diagnostic:
                                            continue
                                        z,u,v = diagnostic['depth_m'],diagnostic['u'],diagnostic['v']
                                    # USD camera: +X right, +Y up, -Z forward; optical: right/down/forward.
                                    local = np.array([(u-cx)*z/fx,-(v-cy)*z/fy,-z,1.])
                                    estimated = (local @ world_from_camera_row)[:3]
                                    if diagnostic['accepted']:
                                        measurements.append((estimated[:2],confidence))
                                        measurement_rows.append(len(position_rows))
                                    response['observations'].append({'bbox':box,'confidence':confidence,'depth_m':z,'world_xyz':estimated.tolist()})
                                    position_rows.append({'frame_id':response['frame_id'],'sim_time':stamp,
                                        'world_step_time':world.current_time,'confidence':confidence,'depth_m':z,
                                        'depth_diagnostic':diagnostic.copy(),'accepted':diagnostic['accepted'],
                                        'legacy_xy_error_m':float(np.linalg.norm(legacy_xyz[:2]-gt_xyz[:2])) if args.humans==1 else None,
                                        'estimated_world_xyz':estimated.tolist(),'gt_root_world_xyz':gt_xyz.tolist(),
                                        'xy_error_m':float(np.linalg.norm(estimated[:2]-gt_xyz[:2])) if args.humans==1 else None})
                                if args.tracking_check:
                                    tracks = tracker.update(stamp,measurements)
                                    response['measurement_rejected'] = tracker.rejected
                                    for rejected in tracker.rejected:
                                        row = position_rows[measurement_rows[rejected['measurement_index']]]
                                        row['accepted'] = False
                                        row['tracking_rejection'] = rejected
                                    ds = response['depth_diagnostics']
                                    depth_confidence = min((d['depth_confidence'] for d in ds),default=None)
                                    measurement_rejected = bool(tracker.rejected) or any(not d['accepted'] for d in ds)
                                    depth_ok = not ds or any(d['accepted'] for d in ds)
                                    response['tracks'] = tracks
                                    response['evaluation_gt_xyz'] = gt_xyz.tolist()
                                    people_values = np.array([g[1] for g in people_history])
                                    response['evaluation_gt_people_xyz'] = [[float(np.interp(stamp,times,people_values[:,i,j])) for j in range(3)] for i in range(args.humans)]
                                    current_tracks,measurement_time = tracks,stamp
                            response['depth_available'] = bool(depth_values is not None and np.isfinite(depth_values).any())
                            if first_person is not None:
                                first_person.detection(response)
                            if args.observation_audit:
                                response['camera_view'] = np.asarray(params['cameraViewTransform']).tolist()
                                response['controller_at_response'] = getattr(brake,'diagnostics',{}).copy()
                                response['response_world_time'] = world.current_time
                            if args.depth_audit and response['boxes'] and depth_values is not None:
                                np.savez_compressed(output/f"depth_frame_{response['frame_id']:04d}.npz",
                                    rgb=rgb,depth=depth_values,boxes=np.asarray(response['boxes']),
                                    camera_view=np.asarray(params['cameraViewTransform']),
                                    gt_evaluation_only=gt_xyz,sim_time=stamp)
                            sample = {'sim_time':world.current_time-initial_sim,
                                'nominal_depth_m':distance if args.distance_check else None, **response}
                            detections.append(sample)
                            if arena:
                                sample['controller_state']=demo_control.state
                                sample['response_elapsed_s']=world.current_time-initial_sim
                                sample['exposure_elapsed_s']=stamp-initial_sim
                            save_frame = ((args.observation_audit and 2.8<=stamp<=4.4) or (args.distance_check and camera_frames % 30 == 20)
                                or camera_frames in (20,60,100,200)
                                or (not response['boxes'] and sum(not d['boxes'] for d in detections) <= 20))
                            if args.demo_controller and response['boxes'] and demo_control.state not in annotated_demo_states:
                                save_frame=True;annotated_demo_states.add(demo_control.state)
                            if arena and camera_frames % 10 == 0:save_frame=True
                            if save_frame:
                                from PIL import Image, ImageDraw
                                im = Image.fromarray(rgb)
                                stem = f'detection_{camera_frames:04d}'
                                if arena:sample['evidence_frame']=stem
                                im.save(output/(stem+'_rgb.png'))
                                if args.navwareset_scene and depth_values is not None:
                                    np.savez_compressed(output/(stem+'_depth.npz'),depth_m=depth_values,exposure_sim_time=stamp,
                                        camera_view=np.asarray(params['cameraViewTransform']),camera_aperture=np.asarray(params['cameraAperture']),
                                        camera_focal_length=np.asarray(params['cameraFocalLength']))
                                draw = ImageDraw.Draw(im)
                                for box,confidence in zip(response['boxes'],response['conf']):
                                    draw.rectangle(box,outline='lime',width=2)
                                    draw.text((box[0],max(0,box[1]-12)),f'person {confidence:.3f}',fill='lime')
                                if args.tracking_check:
                                    state_label = demo_control.state if args.demo_controller else brake.state if args.avoidance else ('DISABLED' if args.scenario else 'TRACKING')
                                    text_lines = [f'EST {state_label} RGB t={stamp:.2f}s now={world.current_time:.2f}s']
                                    for t in response.get('tracks',[]):
                                        x,y,vx,vy = t['state']
                                        text_lines.append(f"ID {t['track_id']} XY {x:.2f},{y:.2f}m V {vx:.2f},{vy:.2f}m/s")
                                        if args.demo_controller:text_lines.append(f"Distance {np.linalg.norm(np.array([x,y])-robot.get_world_pose()[0][:2]):.2f} m | {demo_control.state}")
                                    if args.scenario:
                                        for r in risks:
                                            text_lines.append(f"ID {r['track_id']} D={r['distance_m']:.2f}m tCPA={r['t_cpa_s']:.2f}s dCPA={r['d_cpa_m']:.2f}m TTC={r['ttc_s']:.2f}s")
                                    for obs in response.get('observations',[]):
                                        text_lines.append(f"Conf={obs['confidence']:.2f} optical depth={obs['depth_m']:.2f}m")
                                    draw.rectangle((0,0,640,15*len(text_lines)+4),fill='black')
                                    draw.multiline_text((4,2),'\n'.join(text_lines),fill='white',spacing=3)
                                im.save(output/(stem+'_bbox.png'))
                        if args.stage >= 5 and camera_frames in (20, 30, 40, 60, 100):
                            from PIL import Image
                            Image.fromarray(rgba[:,:,:3].astype(np.uint8)).save(output/f'actor_{camera_frames:04d}.png')
                        if camera_frames == 20:
                            from PIL import Image
                            Image.fromarray(rgba[:,:,:3].astype(np.uint8)).save(output/'camera_rgb.png')
                            if args.stage == 4:
                                depth_data, _ = sensor.get_data('distance_to_image_plane')
                                depth = depth_data.numpy().squeeze()
                                np.save(output/'depth_m.npy', depth)
                                xyz = np.array([[3.,-.8,.5],[5.,.8,.75]])-np.array([.2,0.,1.])
                                uv = np.column_stack([320-240*xyz[:,1]/xyz[:,0],180-240*xyz[:,2]/xyz[:,0]])
                                for label, pixel, expected in zip(['near','far'],uv,[2.55,4.55]):
                                    u,v = np.rint(pixel).astype(int)
                                    measured_depth = float(np.median(depth[v-2:v+3,u-2:u+3]))
                                    depth_checks.append({'prop':label,'pixel':[int(u),int(v)],'expected_front_z_m':expected,
                                        'measured_z_m':measured_depth,'abs_error_m':abs(measured_depth-expected)})
            else:
                app.update()
            count += 1
            if (args.scenario or args.stationary) and world.current_time-initial_sim>=args.sim_seconds:
                break
            if args.demo_controller and world.current_time-initial_sim>=args.sim_seconds:break
            if args.demo_controller and demo_control.done:
                if demo_finished_at is None:demo_finished_at=world.current_time
                if world.current_time-demo_finished_at>=2.:break
            if (args.motion_test or args.motion_matrix or args.curve_demo) and world.current_time-initial_sim>=4.*len(motion_commands):
                break
            if args.distance_check and world.current_time-initial_sim >= 15:
                break
        elapsed = time.perf_counter()-start
        measured = [r for r in samples if r[1] == 'measurement']
        result = {'stage': args.stage, 'status': 'COMPLETED_REQUIRES_LOG_REVIEW',
                  'wall_seconds': elapsed, 'steps': count, 'loop_fps': count/elapsed,
                  'real_time_factor': (world.current_time-initial_sim)/elapsed if world else None,
                  'gpu_total_memory_peak_mib': max((r[2] for r in measured), default=None),
                  'gpu_total_memory_mean_mib': sum(r[2] for r in measured)/len(measured) if measured else None,
                  'system_ram_peak_gib': max((r[4] for r in measured), default=None),
                  'gpu_memory_scope': 'whole GPU, includes unchanged desktop applications',
                  'camera_yolo_human_tested': bool(detections),
                  'headless': args.headless,
                  'camera_tick_rate_hz': 10 if camera else None}
        result['rgb_frames'] = camera_frames
        result['repeated_exposures_skipped'] = repeated_exposures
        result['camera_count'] = 2 if args.second_view else 1
        result['robust_depth'] = args.robust_depth
        result['robot_backend'] = robot_config
        if args.demo_controller:
            (output/'demo_evaluation.json').write_text(json.dumps(demo_rows,indent=2))
            if args.robot_behavior in ('social_nav','active_passing') and not args.classic_diagnostic:
                (output/'social_planning.json').write_text(json.dumps(demo_control.logs))
            result['demo']={'route_done':demo_control.done,'min_gt_distance':min(r['evaluation_min_distance'] for r in demo_rows),'states':sorted(set(r['state'] for r in demo_rows)),'geometric_collision_threshold':robot_config['conservative_radius_m']+.3}
        if args.navwareset_scene:
            from navwareset_scene import save_dataset
            nav_config['initial_sim_time']=initial_sim
            nav_config['human_asset_payloads']=[str(world.stage.GetPrimAtPath('/'.join(p.split('/')[:5])).GetMetadata('payload')) for p in evaluation_paths]
            save_dataset(output,nav_config,nav_geometry,demo_rows,robot_config,args.robot_behavior,evaluation_paths)
            if args.social_experiment:
                (output/'occupancy_xy_points.json').write_text(json.dumps({'frame':'world','units':'m',
                    'source':'frozen known static geometry; planning (social_nav) and post-run evaluation; no human state',
                    'polygons':nav_geometry},indent=2))
            (output/'position_evaluation.json').write_text(json.dumps(position_rows,indent=2))
            result['navwareset']={'scenario':args.navwareset_scenario,'behavior':args.robot_behavior,'exact_reconstruction':False,'gt_in_control':False}
        if args.classic_scene:
            from classic_single_scene import save_dataset
            nav_config['initial_sim_time']=initial_sim
            save_dataset(output,nav_config,nav_geometry,demo_rows,robot_config,args.robot_behavior,evaluation_paths)
            result['classic']={'scenario':args.classic_scenario,'diagnostic':args.classic_diagnostic,'gt_in_control':False}
        if backend_rows:
            (output/'robot_backend.json').write_text(json.dumps(backend_rows,indent=2))
            if args.motion_test or args.motion_matrix or args.curve_demo:
                from a300_motion_validation import summarize
                summarize(output)
        if physical_audit is not None:
            physical_audit.save(output/'physics_audit.json',robot,world)
        result['lateral_avoidance'] = args.lateral_avoidance
        if args.position_check:
            result['configuration'] = configuration
        if args.scenario and risk_rows:
            (output/'control_evaluation.json').write_text(json.dumps(risk_rows,indent=2))
            minimum = min(r['evaluation_gt_distance_m'] for r in risk_rows)
            result['scenario'] = {'name':args.scenario,'avoidance':args.avoidance,'minimum_gt_distance_m':minimum,
                'collision_proxy':minimum<configuration['control']['robot_radius_m']+configuration['control']['human_radius_m'],
                'collision_definition':'XY center-distance disk proxy; not PhysX contact',
                'controller_states':sorted(set(r['controller_state'] for r in risk_rows))}
        if position_rows:
            (output/'position_evaluation.json').write_text(json.dumps(position_rows,indent=2))
            errors = np.array([r['xy_error_m'] for r in position_rows if r['xy_error_m'] is not None])
            if len(errors):
                result['position_surface_vs_gt_root_xy'] = {'samples':len(errors), 'mean_m':float(errors.mean()),
                    'median_m':float(np.median(errors)), 'p95_m':float(np.percentile(errors,95))}
        result['camera_resolution'] = [640,360] if camera else None
        result['depth_checks'] = depth_checks
        result['human_asset_payload'] = actor_asset
        result['human_count'] = args.humans if args.stage >= 5 else 0
        result['camera_clipping_m'] = [.05,100.] if camera else None
        result['fractional_cutout_opacity_override'] = args.fractional_opacity
        if detections:
            (output/'detections.json').write_text(json.dumps(detections,indent=2))
            timings = np.array([d['ms'] for d in detections])
            result['yolo'] = {'model':'yolo11n.pt COCO', 'threshold':.25,
                'samples':len(detections), 'detected_samples':sum(bool(d['boxes']) for d in detections),
                'effective_fps':len(detections)/elapsed, 'mean_ms':float(timings.mean()),
                'p95_ms':float(np.percentile(timings,95)),
                'depth_available_samples':sum(d.get('depth_available',False) for d in detections),
                'max_confidence':max((max(d['conf'],default=0.) for d in detections),default=0.)}
        if camera is not None and camera_frames < 20:
            result['status'] = 'FAIL_CAMERA_NO_VALID_FRAMES'
        if motion:
            with (output/'robot_motion.csv').open('w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['sim_time','command_v','command_w','x','y','z','qw','qx','qy','qz'])
                writer.writerows(motion)
            result['robot_asset'] = asset
            result['robot_motion_samples'] = len(motion)
        (output / 'summary.json').write_text(json.dumps(result, indent=2))
        if first_person is not None:
            first_person.finish(demo_rows,output)
        if args.navwareset_scene:
            from navwareset_scene import evaluate
            print('NAVWARESET_EVALUATION',json.dumps(evaluate(output)),flush=True)
        print('STAGE_RESULT', json.dumps(result), flush=True)
    except Exception:
        import traceback
        failure = traceback.format_exc()
        (output/'failure.txt').write_text(failure)
        print(failure, flush=True)
    finally:
        if demo_writer is not None:
            demo_writer.release()
            if args.demo_controller:
                subprocess.run(['ffmpeg','-y','-hide_banner','-loglevel','error','-i',str(output/'avoidance_demo_raw.mp4'),
                    '-c:v','libx264','-preset','fast','-crf','20','-pix_fmt','yuv420p','-movflags','+faststart',
                    str(output/(f'CLASSIC_{args.classic_scenario.upper()}.mp4' if args.classic_scene else f'NAVWARESET_{args.navwareset_scenario.upper()}.mp4' if args.navwareset_scene else 'A300_HUMAN_AVOIDANCE_DEMO.mp4'))],check=True,timeout=120,creationflags=subprocess.CREATE_NO_WINDOW)
        if bridge is not None:
            bridge.close()
        if worker is not None:
            worker.stdin.close()
            try:
                worker.wait(timeout=10)
            except subprocess.TimeoutExpired:
                worker.terminate()
                worker.wait(timeout=10)
        stop.set()
        thread.join(timeout=10)
        app.close()


if __name__ == '__main__':
    main()
