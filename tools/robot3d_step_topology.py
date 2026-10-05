#!/usr/bin/env python3
"""Audit non-solid STEP topology separately from physical solid envelopes."""
import json
import numpy as np
from OCP.STEPControl import STEPControl_Reader
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_FACE,TopAbs_SOLID,TopAbs_EDGE,TopAbs_VERTEX
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.BRep import BRep_Tool
from OCP.TopoDS import TopoDS
from OCP.TopLoc import TopLoc_Location
from robot3d_extract import ROOT,OUT,bbox,render
r=STEPControl_Reader();r.ReadFile(str(ROOT/'file_3D/Xe.step'));r.TransferRoots();s=r.OneShape()
results={};extra=[]
for typ,avoid,key in [(TopAbs_FACE,TopAbs_SOLID,'faces_outside_solids'),(TopAbs_EDGE,TopAbs_FACE,'edges_outside_faces'),(TopAbs_VERTEX,TopAbs_EDGE,'vertices_outside_edges')]:
    ex=TopExp_Explorer(s,typ,avoid);bounds=[]
    while ex.More():
        shape=ex.Current();bounds.append(bbox(shape))
        if typ==TopAbs_FACE:
            BRepMesh_IncrementalMesh(shape,.35,False,.25,False).Perform();loc=TopLoc_Location();tri=BRep_Tool.Triangulation_s(TopoDS.Face(shape),loc)
            if tri:
                p=np.array([[p.X(),p.Y(),p.Z()] for p in [tri.Node(i).Transformed(loc.Transformation()) for i in range(1,tri.NbNodes()+1)]])
                ids=np.array([tri.Triangle(i).Get() for i in range(1,tri.NbTriangles()+1)])-1;extra.append(p[ids])
        ex.Next()
    results[key]=dict(count=len(bounds),bounds=bounds)
g=json.loads((OUT/'geometry.json').read_text());bs=np.array([x['bounds_mm'] for x in g['step']['solids']]);results['solid_union_bounds_mm']=np.r_[bs[:,:3].min(0),bs[:,3:].max(0)].tolist()
(OUT/'step_topology.json').write_text(json.dumps(results,indent=2))
np.savez_compressed(OUT/'cad_extra_faces.npz',**{f'face_{i}':m for i,m in enumerate(extra)})
print({k:(v.get('count') if isinstance(v,dict) else v) for k,v in results.items()},flush=True)
