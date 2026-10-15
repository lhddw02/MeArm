"""Build portable SI meshes, MJCF, SDF and URDF from one MeArm V1 spec.

Community visual transforms are baked into local meshes. Kinematics are
retargeted to official Arduino V1 (80/80/68 mm), not silently copied from ROS.
"""
from community_meshes import ROOT, visual_meshes, transform
import json, math, sys
from pathlib import Path
import numpy as np
import trimesh
import xml.etree.ElementTree as ET

PI=math.pi
BLUE=[.06,.48,.64,1]
DARK=[.08,.12,.18,1]
CFG=json.loads((ROOT/'config/mearm_v1.json').read_text())
NAME=CFG['name']
DEST=ROOT/'models'/NAME
MESH=DEST/'meshes'

def box(size,pos=(0,0,0)):
    m=trimesh.creation.box(extents=size); m.apply_translation(pos); return m

def proxy(size,pos=(0,0,0),rpy=(0,0,0)):
    return dict(type='box',size=list(size),pos=list(pos),rpy=list(rpy))

def build_spec():
    MESH.mkdir(parents=True,exist_ok=True)
    src=visual_meshes()
    links=[]
    def add(name,parent,pos=(0,0,0),rpy=(0,0,0),mesh=None,joint=None,collisions=(),color=BLUE,mass=None):
        if mesh is not None:
            mesh.remove_unreferenced_vertices()
            mesh.fix_normals(multibody=True)
            mesh.export(MESH/f'{name}.stl')
            # Uniform-density plate inertia, plus servo bodies as separate links.
            mesh.density=CFG['acrylic_density_kg_m3']
            mp=mesh.mass_properties
            real_mass=max(float(mp.mass),.0001) if mass is None else mass
            tensor=np.asarray(mp.inertia)*(real_mass/max(float(mp.mass),1e-12))
            com=np.asarray(mp.center_mass)
        else:
            real_mass=1e-5 if mass is None else mass
            tensor=np.eye(3)*1e-9; com=np.zeros(3)
        eig=np.linalg.eigvalsh(tensor)
        if eig.min()<=0: raise ValueError((name,'invalid inertia',eig))
        links.append(dict(name=name,parent=parent,pos=list(pos),rpy=list(rpy),mass=real_mass,com=com.tolist(),inertia=[float(tensor[0,0]),float(tensor[1,1]),float(tensor[2,2]),float(tensor[0,1]),float(tensor[0,2]),float(tensor[1,2])],mesh=f'models/{NAME}/meshes/{name}.stl' if mesh is not None else None,collisions=list(collisions),color=color,joint=joint))
    def hinge(name,axis,low,high,mimic=None):
        return dict(name=name,axis=axis,lower=low,upper=high,effort=CFG['servo_torque_nm'],velocity=CFG['servo_speed_rad_s'],mimic=mimic)
    def mimic(name,mult=-1): return dict(joint=name,multiplier=mult)
    def canon(key):
        m=src[key].copy(); m.apply_transform(transform(rpy=(PI/2,0,0))); return m
    L1=CFG['upper_arm_m']; L2=CFG['forearm_m']; h=CFG['shoulder_height_m']; a=CFG['shoulder_offset_m']
    base=src['base_link'].copy(); base.apply_translation((0,0,.002))
    add('base',None,mesh=base,collisions=[proxy([.145,.095,.003],[-.0304,0,.0015]),proxy([.060,.060,.003],[0,0,.0195])])
    yawz=h-.029
    add('yaw','base',(0,0,yawz),mesh=src['column'],joint=hinge('yaw_joint',[0,0,1],-PI/4,PI/4),collisions=[proxy([.058,.046,.004],[0,0,0]),proxy([.05,.003,.038],[0,.022,.020]),proxy([.05,.003,.038],[0,-.022,.020])])
    upper=canon('arm_0'); upper.vertices[:,0]*=L1/.080
    add('upper','yaw',(a,0,.029),(0,-PI/2,0),upper,hinge('shoulder_joint',[0,-1,0],-PI/4,PI/4),[proxy([L1,.028,.009],[L1/2,0,0])])
    add('elbow_comp','upper',(L1,0,0),(0,PI/2,0),canon('connector'),hinge('elbow_comp_joint',[0,-1,0],-PI/4,PI/4,mimic('shoulder_joint')),[])
    fore=canon('arm_1b'); fore.vertices[:,0]*=L2/.081
    add('forearm','elbow_comp',mesh=fore,joint=hinge('elbow_joint',[0,-1,0],-PI/4,PI/4),collisions=[proxy([L2,.030,.009],[L2/2,0,0])])
    hand=canon('gripper'); hand.apply_translation((0,.002,.003))
    add('hand','forearm',(L2,0,0),mesh=hand,joint=hinge('wrist_joint',[0,-1,0],-PI/4,PI/4,mimic('elbow_joint')),collisions=[proxy([.030,.025,.014],[.008,0,.009])])
    # Source jaws are already in a horizontal plane at neutral. Align their
    # pivots symmetrically; opening is positive for the left, negative right.
    add('left_finger','hand',(.028,.009,0),mesh=src['gripper_left'],joint=hinge('left_finger_joint',[0,0,1],0,CFG['finger_open_rad']),collisions=[proxy([.025,.003,.005],[.021,-.007,-.002])])
    add('right_finger','hand',(.028,-.009,0),mesh=src['gripper_right'],joint=hinge('right_finger_joint',[0,0,1],-CFG['finger_open_rad'],0,mimic('left_finger_joint')),collisions=[proxy([.025,.003,.005],[.021,.007,-.002])])
    # Visible parallel bars follow the same ideal kinematic relations.
    add('upper_parallel','yaw',(a-.035,-.019,.045),(0,-PI/2,0),canon('arm_0_support'),hinge('upper_parallel_joint',[0,-1,0],-PI/4,PI/4,mimic('shoulder_joint',1)),[])
    add('forearm_parallel','elbow_comp',(.012,-.014,.018),mesh=canon('gripper_support'),joint=hinge('forearm_parallel_joint',[0,-1,0],-PI/4,PI/4,mimic('elbow_joint',1)),collisions=[])
    # SG90-sized approximations; real servo variant/mass must be measured.
    add('base_servo','base',(0,0,.011),mesh=box([.023,.012,.020]),color=DARK,mass=CFG['servo_mass_kg'])
    for side,y in [('left',.029),('right',-.029)]:
        add(f'{side}_servo','yaw',(a-.006,y,.029),mesh=box([.023,.022,.012]),color=DARK,mass=CFG['servo_mass_kg'])
    add('gripper_servo','hand',(.004,0,.0065),mesh=box([.024,.012,.018]),color=DARK,mass=CFG['servo_mass_kg'])
    spec=dict(name=NAME,units='m, kg, s, rad',config=CFG,links=links,tool=dict(link='hand',pos=[CFG['wrist_to_tool_m'],0,0]))
    (ROOT/'config/model_spec.json').write_text(json.dumps(spec,indent=2)+'\n')
    return spec

def vec(x): return ' '.join(f'{v:.10g}' for v in x)

def write_xml(root,path):
    path.parent.mkdir(parents=True,exist_ok=True); ET.indent(root,space='  ')
    ET.ElementTree(root).write(path,encoding='utf-8',xml_declaration=True)

def build_mjcf(spec):
    root=ET.Element('mujoco',model=NAME)
    ET.SubElement(root,'compiler',angle='radian',eulerseq='xyz',meshdir=f'../models/{NAME}/meshes',inertiafromgeom='false')
    ET.SubElement(root,'option',timestep='0.001',integrator='implicitfast',gravity='0 0 -9.81',iterations='100',tolerance='1e-10')
    ET.SubElement(root,'statistic',center='.04 0 .08',extent='.34')
    vis=ET.SubElement(root,'visual'); ET.SubElement(vis,'global',offwidth='1440',offheight='1080',azimuth='135',elevation='-25')
    ET.SubElement(vis,'headlight',ambient='.4 .4 .4',diffuse='.7 .7 .7',specular='.2 .2 .2')
    asset=ET.SubElement(root,'asset')
    ET.SubElement(asset,'texture',name='grid',type='2d',builtin='checker',rgb1='.85 .88 .90',rgb2='.75 .80 .83',width='256',height='256')
    ET.SubElement(asset,'material',name='floor_mat',texture='grid',texrepeat='5 5',reflectance='.04')
    world=ET.SubElement(root,'worldbody'); ET.SubElement(world,'light',pos='0 -.2 .5',dir='0 0 -1',diffuse='.8 .8 .8')
    ET.SubElement(world,'geom',name='floor',type='plane',size='.5 .5 .01',material='floor_mat',contype='1',conaffinity='3')
    bodies={}
    equal=ET.Element('equality')
    for l in spec['links']:
        parent=world if l['parent'] is None else bodies[l['parent']]
        # MJCF lowercase xyz are intrinsic, so encode URDF/SDF extrinsic RPY as quaternion.
        quat=trimesh.transformations.quaternion_from_matrix(transform(rpy=l['rpy']))
        b=ET.SubElement(parent,'body',name=l['name'],pos=vec(l['pos']),quat=vec(quat)); bodies[l['name']]=b
        ET.SubElement(b,'inertial',pos=vec(l['com']),mass=str(l['mass']),fullinertia=vec(l['inertia']))
        j=l['joint']
        if j:
            ET.SubElement(b,'joint',name=j['name'],type='hinge',axis=vec(j['axis']),range=vec([j['lower'],j['upper']]),damping='.015',armature='0.00001',frictionloss='.0005')
            if j['mimic']:
                ET.SubElement(equal,'joint',name=j['name']+'_coupling',joint1=j['name'],joint2=j['mimic']['joint'],polycoef=vec([0,j['mimic']['multiplier'],0,0,0]),solref='.003 1',solimp='.999 .999 .001')
        if l['mesh']:
            ET.SubElement(asset,'mesh',name=l['name']+'_mesh',file=Path(l['mesh']).name)
            ET.SubElement(b,'geom',name=l['name']+'_visual',type='mesh',mesh=l['name']+'_mesh',rgba=vec(l['color']),contype='0',conaffinity='0',group='2')
        for i,c in enumerate(l['collisions']):
            q=trimesh.transformations.quaternion_from_matrix(transform(rpy=c['rpy']))
            ET.SubElement(b,'geom',name=f'{l["name"]}_collision_{i}',type='box',size=vec(np.array(c['size'])/2),pos=vec(c['pos']),quat=vec(q),rgba='.7 .2 .2 .2',contype='2',conaffinity='1',group='3',friction='.9 .01 .001')
    ET.SubElement(bodies['hand'],'site',name='tcp',pos=vec(spec['tool']['pos']),size='.003',rgba='1 .35 .1 1')
    root.append(equal)
    act=ET.SubElement(root,'actuator')
    for name,joint,lo,hi,kp in [('base','yaw_joint',-PI/4,PI/4,5),('shoulder','shoulder_joint',-PI/4,PI/4,8),('elbow','elbow_joint',-PI/4,PI/4,6),('gripper','left_finger_joint',0,CFG['finger_open_rad'],2)]:
        ET.SubElement(act,'position',name=name,joint=joint,kp=str(kp),kv='.12',ctrlrange=vec([lo,hi]),forcerange=vec([-CFG['servo_torque_nm'],CFG['servo_torque_nm']]))
    sensor=ET.SubElement(root,'sensor'); ET.SubElement(sensor,'framepos',name='tcp_position',objtype='site',objname='tcp')
    keys=ET.SubElement(root,'keyframe'); ET.SubElement(keys,'key',name='neutral',ctrl='0 0 0 0')
    write_xml(root,ROOT/'mujoco/mearm.xml')

def build_urdf(spec):
    robot=ET.Element('robot',name=NAME)
    ET.SubElement(robot,'link',name='world')
    for l in spec['links']:
        b=ET.SubElement(robot,'link',name=l['name'])
        inert=ET.SubElement(b,'inertial'); ET.SubElement(inert,'origin',xyz=vec(l['com'])); ET.SubElement(inert,'mass',value=str(l['mass']))
        ET.SubElement(inert,'inertia',**dict(zip(['ixx','iyy','izz','ixy','ixz','iyz'],map(str,l['inertia']))))
        if l['mesh']:
            visual=ET.SubElement(b,'visual'); geom=ET.SubElement(visual,'geometry'); ET.SubElement(geom,'mesh',filename='../'+l['mesh'])
            mat=ET.SubElement(visual,'material',name=l['name']+'_color'); ET.SubElement(mat,'color',rgba=vec(l['color']))
        for c in l['collisions']:
            collision=ET.SubElement(b,'collision'); ET.SubElement(collision,'origin',xyz=vec(c['pos']),rpy=vec(c['rpy'])); geom=ET.SubElement(collision,'geometry'); ET.SubElement(geom,'box',size=vec(c['size']))
        j=l['joint']; joint=ET.SubElement(robot,'joint',name=j['name'] if j else l['name']+'_fixed',type='revolute' if j else 'fixed')
        ET.SubElement(joint,'parent',link=l['parent'] or 'world'); ET.SubElement(joint,'child',link=l['name']); ET.SubElement(joint,'origin',xyz=vec(l['pos']),rpy=vec(l['rpy']))
        if j:
            ET.SubElement(joint,'axis',xyz=vec(j['axis'])); ET.SubElement(joint,'limit',lower=str(j['lower']),upper=str(j['upper']),effort=str(j['effort']),velocity=str(j['velocity']))
            if j['mimic']: ET.SubElement(joint,'mimic',joint=j['mimic']['joint'],multiplier=str(j['mimic']['multiplier']),offset='0')
    ET.SubElement(robot,'link',name='tool')
    tool=ET.SubElement(robot,'joint',name='tool_joint',type='fixed')
    ET.SubElement(tool,'parent',link=spec['tool']['link']); ET.SubElement(tool,'child',link='tool')
    ET.SubElement(tool,'origin',xyz=vec(spec['tool']['pos']),rpy='0 0 0')
    write_xml(robot,ROOT/'urdf/mearm.urdf')

def build_cut_meshes():
    from inspect_dxf import cut_parts
    from shapely.affinity import scale
    parts,_=cut_parts(); out=ROOT/'cad/cut_parts'; out.mkdir(parents=True,exist_ok=True)
    manifest=[]; sheet=[]
    for i,p in enumerate(parts):
        if p.area<50: continue  # discard uncontained small hole/scrap faces
        shape=scale(p,xfact=.001,yfact=.001,origin=(0,0))
        mesh=trimesh.creation.extrude_polygon(shape,height=CFG['plate_thickness_m'],engine='earcut')
        repair_mm=0
        if not mesh.is_watertight:
            # A source hole touches an outer boundary at one point. Open that
            # zero-width connection by 2 microns to avoid a nonmanifold edge.
            p=p.buffer(-.002).buffer(.002); repair_mm=.002
            shape=scale(p,xfact=.001,yfact=.001,origin=(0,0))
            mesh=trimesh.creation.extrude_polygon(shape,height=CFG['plate_thickness_m'],engine='earcut')
        mesh.fix_normals(multibody=True); sheet.append(mesh.copy())
        center=mesh.bounds.mean(axis=0); mesh.apply_translation(-center)
        mesh.export(out/f'part_{i:02d}.stl')
        assert trimesh.load_mesh(out/f'part_{i:02d}.stl').is_watertight, f'Invalid CAD mesh {i}'
        manifest.append(dict(id=i,file=f'part_{i:02d}.stl',original_center_m=center.tolist(),area_mm2=p.area,holes=len(p.interiors),boundary_opening_repair_mm=repair_mm,watertight=mesh.is_watertight))
    trimesh.util.concatenate(sheet).export(ROOT/'cad/v1_cut_sheet.obj')
    (ROOT/'cad/parts.json').write_text(json.dumps(manifest,indent=2)+'\n')
    return len(manifest)

def build_static_assembly(spec):
    """CAD-friendly neutral assembly, with transforms baked into meter units."""
    folder=ROOT/'cad'; folder.mkdir(exist_ok=True)
    scene=trimesh.Scene(); transforms={}; solids=[]
    for l in spec['links']:
        parent=transforms.get(l['parent'],np.eye(4))
        t=parent@transform(l['pos'],l['rpy']); transforms[l['name']]=t
        if not l['mesh']: continue
        mesh=trimesh.load_mesh(ROOT/l['mesh']); mesh.apply_transform(t)
        mesh.visual.vertex_colors=np.tile((np.array(l['color'])*255).astype(np.uint8),(len(mesh.vertices),1))
        solids.append(mesh); scene.add_geometry(mesh,node_name=l['name'])
    scene.export(folder/'mearm_assembled.glb')
    trimesh.util.concatenate(solids).export(folder/'mearm_assembled.stl')

if __name__=='__main__':
    spec=build_spec(); build_mjcf(spec); build_urdf(spec)
    from build_gazebo import build
    build()
    build_static_assembly(spec)
    print('Built',len(spec['links']),'links and',build_cut_meshes(),'extruded official DXF contours')
