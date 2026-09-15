"""Expand official A300 xacro without ROS; import once with shipped Isaac 6 importer."""
from pathlib import Path
import json
import sys
ROOT = Path(__file__).resolve().parent

if '--import' not in sys.argv and '--inspect' not in sys.argv:
    import xacro
    import xacro.substitution_args
    import xml.etree.ElementTree as ET
    import numpy as np
    import trimesh
    from scipy.spatial.transform import Rotation
    packages = ROOT/'clearpath_common'
    xacro.substitution_args._eval_find = lambda package: str(packages/package).replace('\\','/')
    doc = xacro.process_file(str(ROOT/'robot.urdf.xacro'),mappings={'use_platform_controllers':'false'})
    tree = ET.fromstring(doc.toxml())
    for mesh in tree.findall('.//mesh'):
        mesh.set('filename',str(packages/mesh.get('filename').removeprefix('package://')).replace('\\','/'))
    # Gazebo plugin declarations are not used by Isaac; geometry/inertias stay unchanged.
    for node in tree.findall('gazebo'):
        tree.remove(node)
    ET.indent(tree)
    ET.ElementTree(tree).write(ROOT/'husky_a300.urdf',encoding='utf-8',xml_declaration=True)
    transforms={'base_link':np.eye(4)}
    pending=list(tree.findall('joint'))
    while pending:
        progressed=False
        for joint in pending[:]:
            parent=joint.find('parent').get('link')
            if parent not in transforms: continue
            origin=joint.find('origin'); t=np.eye(4)
            if origin is not None:
                t[:3,3]=np.fromstring(origin.get('xyz','0 0 0'),sep=' ')
                t[:3,:3]=Rotation.from_euler('xyz',np.fromstring(origin.get('rpy','0 0 0'),sep=' ')).as_matrix()
            transforms[joint.find('child').get('link')]=transforms[parent]@t
            pending.remove(joint);progressed=True
        if not progressed: raise ValueError('Unresolved URDF tree')
    points=[];wheels=[]
    for link in tree.findall('link'):
        for collision in link.findall('collision'):
            geom=collision.find('geometry'); origin=collision.find('origin'); t=transforms[link.get('name')].copy()
            local=np.eye(4)
            if origin is not None:
                local[:3,3]=np.fromstring(origin.get('xyz','0 0 0'),sep=' ')
                local[:3,:3]=Rotation.from_euler('xyz',np.fromstring(origin.get('rpy','0 0 0'),sep=' ')).as_matrix()
            t=t@local
            if geom.find('mesh') is not None:
                vertices=trimesh.load(geom.find('mesh').get('filename'),force='mesh').vertices
            elif geom.find('cylinder') is not None:
                c=geom.find('cylinder');r=float(c.get('radius'));h=float(c.get('length'))
                vertices=np.array([[x,y,z] for x in (-r,r) for y in (-r,r) for z in (-h/2,h/2)])
            else:
                size=np.fromstring(geom.find('box').get('size'),sep=' ')/2
                vertices=np.array([[x,y,z] for x in (-size[0],size[0]) for y in (-size[1],size[1]) for z in (-size[2],size[2])])
            points.extend(vertices@t[:3,:3].T+t[:3,3])
    for joint in tree.findall('joint'):
        if joint.get('type')=='continuous':
            name=joint.get('name'); link=joint.find('child').get('link')
            wheels.append({'name':name,'xyz':transforms[link][:3,3].tolist(),'axis':joint.find('axis').get('xyz'),'limit':joint.find('limit').attrib if joint.find('limit') is not None else None})
    bounds=np.array([np.min(points,axis=0),np.max(points,axis=0)])
    radius=float(tree.find("link[@name='front_left_wheel_link']/collision/geometry/cylinder").get('radius'))
    d={w['name']:w for w in wheels}
    track=abs(d['front_left_wheel_joint']['xyz'][1]-d['front_right_wheel_joint']['xyz'][1])
    params={'wheel_radius_m':radius,'track_width_m':track,'wheelbase_m':abs(d['front_left_wheel_joint']['xyz'][0]-d['rear_left_wheel_joint']['xyz'][0]),
      'wheels':wheels,'collision_bounds_base_m':bounds.tolist(),'collision_dimensions_m':(bounds[1]-bounds[0]).tolist(),
      'conservative_radius_m':float(np.linalg.norm(np.max(np.abs(bounds[:,:2]),axis=0))),
      'specified_mass_kg':sum(float(m.get('value')) for m in tree.findall('link/inertial/mass')),
      'links_without_inertial':[l.get('name') for l in tree.findall('link') if l.find('inertial') is None],
      'spawn_z_m':float(-bounds[0,2]+.02),'camera_mount_m':[.37,0.,.50]}
    (ROOT/'parameters.json').write_text(json.dumps(params,indent=2))
    print(json.dumps(params,indent=2))
else:
    from isaacsim import SimulationApp
    app=SimulationApp({'headless':True,'extra_args':['--enable','isaacsim.asset.importer.urdf']})
    try:
        from isaacsim.asset.importer.urdf import URDFImporter,URDFImporterConfig
        from pxr import Usd,UsdPhysics,PhysxSchema
        if '--inspect' in sys.argv:
            from pxr import UsdGeom
            stage=Usd.Stage.Open(str(ROOT/'husky_a300.usd'))
            prims=list(Usd.PrimRange(stage.GetDefaultPrim(),Usd.TraverseInstanceProxies()))
            print('GEOMETRY_INSPECTION',json.dumps({'mesh_count':sum(p.IsA(UsdGeom.Mesh) for p in prims),
              'faces':sum(len(UsdGeom.Mesh(p).GetFaceVertexCountsAttr().Get() or []) for p in prims if p.IsA(UsdGeom.Mesh))}),flush=True)
            sys.exit(0)
        config=URDFImporterConfig(urdf_path=str(ROOT/'husky_a300.urdf'),usd_path=str(ROOT/'converted'),merge_fixed_joints=True,
          fix_base=False,allow_self_collision=False,collision_from_visuals=False,joint_drive_type='force',joint_target_type='velocity',
          override_joint_stiffness=0.,override_joint_damping=100.)
        generated=URDFImporter(config).import_urdf()
        stage=Usd.Stage.CreateNew(str(ROOT/'husky_a300.usd'))
        root=stage.DefinePrim('/husky_a300','Xform')
        root.GetReferences().AddReference(Path(generated).relative_to(ROOT).as_posix())
        stage.SetDefaultPrim(root)
        from pxr import UsdGeom
        UsdGeom.SetStageUpAxis(stage,UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage,1.)
        stage.GetRootLayer().Save()
        print('A300_IMPORTED',generated,flush=True)
    finally:
        app.close()
