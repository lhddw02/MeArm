"""Read the supplied ROS reference visuals without requiring ROS or xacro."""
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1]
from _runtime import configure_dependencies
configure_dependencies(ROOT)
import xml.etree.ElementTree as ET
import numpy as np
import trimesh

def transform(pos=(0,0,0),rpy=(0,0,0)):
    t=trimesh.transformations.euler_matrix(*rpy,axes='sxyz'); t[:3,3]=pos
    return t

def numbers(node,attr,default='0 0 0'):
    return [float(x) for x in node.get(attr,default).split()] if node is not None else [float(x) for x in default.split()]

def visual_meshes():
    doc=ET.parse(ROOT/'references/community/urdf/robot.urdf').getroot()
    result={}
    for link in doc.findall('link'):
        meshes=[]
        for visual in link.findall('visual'):
            shape=visual.find('geometry')[0]
            if shape.tag=='mesh':
                mesh=trimesh.load_mesh(ROOT/'references/community/meshes'/Path(shape.get('filename')).name)
                mesh.apply_scale([float(v) for v in shape.get('scale','1 1 1').split()])
            elif shape.tag=='box':
                if link.get('name')=='gripper': continue  # separate 9 g servo in the new model
                mesh=trimesh.creation.box(extents=[float(v) for v in shape.get('size').split()])
            elif shape.tag=='cylinder':
                mesh=trimesh.creation.cylinder(radius=float(shape.get('radius')),height=float(shape.get('length')),sections=24)
            else: continue
            origin=visual.find('origin')
            mesh.apply_transform(transform(numbers(origin,'xyz'),numbers(origin,'rpy')))
            meshes.append(mesh)
        if meshes: result[link.get('name')]=trimesh.util.concatenate(meshes)
    return result

if __name__=='__main__':
    for name,mesh in visual_meshes().items(): print(name,np.round(mesh.bounds,4).tolist(),mesh.is_watertight)
