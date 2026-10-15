"""Extract nested cut contours; create a numbered drawing in millimetres."""
from pathlib import Path
import sys
ROOT = Path(__file__).resolve().parents[1]
from _runtime import configure_dependencies
configure_dependencies(ROOT)
import json
import ezdxf
from ezdxf import path as ep
from shapely.geometry import LineString, Polygon
from shapely import set_precision
from shapely.ops import polygonize, unary_union
import numpy as np


def cut_parts(filename=ROOT/'references/MeArmV1.1.dxf'):
    doc = ezdxf.readfile(filename)
    # DXF INSUNITS=5 (centimetres). Never assume these are mm!
    scale = {0: 1., 4: 1., 5: 10., 6: 1000.}[doc.header.get('$INSUNITS', 0)]
    lines = []
    for ent in doc.modelspace():
        if ent.dxf.layer == 'Vector Etch':
            continue
        if ent.dxftype() in ('TEXT', 'MTEXT', 'DIMENSION', 'HATCH'):
            continue
        try:
            pts = [(round(v.x*scale,4), round(v.y*scale,4)) for v in ep.make_path(ent).flattening(0.002)]
        except (TypeError, ValueError, AttributeError):
            continue
        if len(pts) > 1:
            lines.append(pts)
    # The upstream cut drawing contains tiny export gaps and larger open gaps.
    # Repair endpoints only; never scale the contours. See fidelity notes.
    ends = np.array([p for pts in lines for p in (pts[0],pts[-1])])
    dist = np.linalg.norm(ends[:,None,:]-ends[None,:,:],axis=2)
    parents = list(range(len(ends)))
    def root(i):
        while parents[i] != i:
            parents[i] = parents[parents[i]]; i=parents[i]
        return i
    for a,b in np.argwhere(np.triu(dist < 0.12,1)):
        parents[root(int(b))]=root(int(a))
    groups = {}
    for i in range(len(ends)):
        groups.setdefault(root(i),[]).append(i)
    fixed = {i:tuple(ends[v].mean(axis=0)) for v in groups.values() for i in v}
    repaired = []
    for i,pts in enumerate(lines):
        pts[0]=fixed[2*i]; pts[-1]=fixed[2*i+1]
        repaired.append(LineString(pts))
    singles=[v[0] for v in groups.values() if len(v)==1]
    used=set()
    for i in singles:
        if i in used: continue
        candidates=[j for j in singles if j!=i and j not in used and dist[i,j]<2.2]
        if candidates:
            j=min(candidates,key=lambda j:dist[i,j])
            repaired.append(LineString([ends[i],ends[j]])); used.update((i,j))
    lines=[set_precision(line,grid_size=0.001) for line in repaired]
    polys = list(polygonize(unary_union(lines)))
    # polygonize emits each hole as its own face as well: retain outer material.
    outer = []
    for p in polys:
        if p.area < 4.:
            continue
        if any(Polygon(q.exterior).contains(p.representative_point()) and q.area > p.area for q in polys):
            continue
        outer.append(p)
    return sorted(outer, key=lambda p: (-round(p.centroid.y/10), p.centroid.x)), doc


if __name__ == '__main__':
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    parts, doc = cut_parts()
    print('DXF units:',doc.header.get('$INSUNITS'), 'parts:',len(parts))
    out = ROOT/'docs'; out.mkdir(exist_ok=True)
    fig, ax = plt.subplots(figsize=(14,10))
    for i,p in enumerate(parts):
        x,y=p.exterior.xy; ax.fill(x,y,color='#2585a8',alpha=.65)
        for h in p.interiors:
            x,y=h.xy; ax.fill(x,y,color='white')
        pt=p.representative_point(); ax.text(pt.x,pt.y,str(i),ha='center',fontsize=10)
        print(i, [round(v,2) for v in p.bounds], 'area',round(p.area,1),'holes',len(p.interiors))
    ax.set_aspect('equal'); ax.set_xlabel('mm'); ax.set_ylabel('mm')
    ax.set_title('Official MeArm V1.1 cut contours — part IDs for simulation extraction')
    fig.tight_layout(); fig.savefig(out/'cut_parts.png',dpi=150)
