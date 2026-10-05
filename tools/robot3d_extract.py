#!/usr/bin/env python3
"""Measure source STEP/STLs, tessellate real CAD, and render reproducible plates."""
import json, hashlib, math, re, sys, itertools
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import trimesh
import vtk
from vtk.util.numpy_support import numpy_to_vtk
from OCP.STEPControl import STEPControl_Reader
from OCP.IFSelect import IFSelect_RetDone
from OCP.TopAbs import TopAbs_SOLID, TopAbs_FACE, TopAbs_REVERSED
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS
from OCP.BRep import BRep_Tool
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.TopLoc import TopLoc_Location

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'docs/robot_3d_report'
FIG=OUT/'figures'
MODEL=ROOT/'src/vacuum_robot_gazebo/models/vacuum_robot'
FIG.mkdir(parents=True,exist_ok=True)

def bbox(shape):
    b=Bnd_Box(); BRepBndLib.AddOptimal_s(shape,b,False,False)
    a,z=b.CornerMin(),b.CornerMax()
    return [a.X(),a.Y(),a.Z(),z.X(),z.Y(),z.Z()]

def extract():
    step=ROOT/'file_3D/Xe.step'
    reader=STEPControl_Reader()
    assert reader.ReadFile(str(step))==IFSelect_RetDone
    reader.TransferRoots(); shape=reader.OneShape()
    print('STEP transferred',bbox(shape),flush=True)
    BRepMesh_IncrementalMesh(shape,0.35,False,0.25,True).Perform()
    solids=[]; meshes=[]
    ex=TopExp_Explorer(shape,TopAbs_SOLID)
    while ex.More():
        s=ex.Current(); prop=GProp_GProps(); BRepGProp.VolumeProperties_s(s,prop)
        surf=GProp_GProps(); BRepGProp.SurfaceProperties_s(s,surf)
        p=prop.CentreOfMass(); tris=[]
        fe=TopExp_Explorer(s,TopAbs_FACE)
        while fe.More():
            f=TopoDS.Face(fe.Current()); loc=TopLoc_Location()
            tri=BRep_Tool.Triangulation_s(f,loc)
            if tri:
                points=np.array([[p.X(),p.Y(),p.Z()] for p in
                    [tri.Node(i).Transformed(loc.Transformation()) for i in range(1,tri.NbNodes()+1)]])
                ids=np.array([tri.Triangle(i).Get() for i in range(1,tri.NbTriangles()+1)])-1
                if f.Orientation()==TopAbs_REVERSED: ids=ids[:,::-1]
                tris.append(points[ids])
            fe.Next()
        mesh=np.concatenate(tris) if tris else np.zeros((0,3,3))
        meshes.append(mesh)
        solids.append(dict(id=len(solids)+1,bounds_mm=bbox(s),volume_mm3=prop.Mass(),
                           area_mm2=surf.Mass(),centroid_mm=[p.X(),p.Y(),p.Z()],triangles=len(mesh)))
        ex.Next()
    np.savez_compressed(OUT/'cad_meshes.npz',**{f'solid_{i+1}':m for i,m in enumerate(meshes)})
    urdf=ET.parse(ROOT/'src/vacuum_robot_gazebo/urdf/vacuum_robot.urdf').getroot()
    translations={j.find('child').get('link'):np.fromstring(j.find('origin').get('xyz','0 0 0'),sep=' ') for j in urdf.findall('joint')}
    visual=[]; stl_data=[]
    for link in urdf.findall('link'):
        for v in link.findall('visual'):
            me=v.find('geometry/mesh')
            if me is None: continue
            fname=Path(me.get('filename')).name
            m=trimesh.load(MODEL/'meshes'/fname,force='mesh')
            trans=np.fromstring(v.find('origin').get('xyz'),sep=' ')+translations.get(link.get('name'),np.zeros(3))
            vertices=m.triangles*.001+trans
            visual.append(vertices)
            stl_data.append(dict(link=link.get('name'),file=fname,faces=len(m.faces),
                vertices=len(m.vertices),watertight=bool(m.is_watertight),
                bounds_base_m=[vertices.reshape(-1,3).min(0).tolist(),vertices.reshape(-1,3).max(0).tolist()],
                transform_m=trans.tolist()))
    np.savez_compressed(OUT/'urdf_meshes.npz',**{d['link']:m for d,m in zip(stl_data,visual)})
    raw=step.read_text(errors='replace')
    products=re.findall(r"#\d+=PRODUCT\((.*?)\);",raw,re.S)
    sources=[step,ROOT/'src/vacuum_robot_gazebo/urdf/vacuum_robot.urdf',MODEL/'model.sdf']
    sources+=list((ROOT/'src/vacuum_robot_gazebo/config').glob('*.yaml'))
    sources+=list((MODEL/'meshes').glob('*.stl'))
    result=dict(step=dict(path=str(step.relative_to(ROOT)),bounds_mm=bbox(shape),solid_count=len(solids),
        solids=solids,product_records=products,unit='mm (OCCT STEP reader target unit)',
        tessellation=dict(linear_deflection_mm=.35,angular_deflection_rad=.25),
        total_solid_volume_mm3=sum(s['volume_mm3'] for s in solids)),stl=stl_data,
        sources=[dict(path=str(p.relative_to(ROOT)),bytes=p.stat().st_size,sha256=hashlib.sha256(p.read_bytes()).hexdigest()) for p in sources])
    (OUT/'geometry.json').write_text(json.dumps(result,indent=2,ensure_ascii=False))
    print('solids',len(solids),'STL',stl_data,flush=True)

def actor(tri,color,alpha=1):
    points=vtk.vtkPoints(); points.SetData(numpy_to_vtk(tri.reshape(-1,3),deep=True))
    cells=vtk.vtkCellArray()
    ids=np.column_stack([np.full(len(tri),3),np.arange(len(tri)*3).reshape(-1,3)]).astype(np.int64)
    from vtk.util.numpy_support import numpy_to_vtkIdTypeArray
    cells.SetCells(len(tri),numpy_to_vtkIdTypeArray(ids.ravel(),deep=True))
    poly=vtk.vtkPolyData(); poly.SetPoints(points); poly.SetPolys(cells)
    norms=vtk.vtkPolyDataNormals(); norms.SetInputData(poly); norms.SetFeatureAngle(55)
    mapper=vtk.vtkPolyDataMapper(); mapper.SetInputConnection(norms.GetOutputPort())
    a=vtk.vtkActor(); a.SetMapper(mapper); a.GetProperty().SetColor(color)
    a.GetProperty().SetOpacity(alpha); a.GetProperty().SetSpecular(.28); a.GetProperty().SetSpecularPower(28)
    return a

def render(name,meshes,colors,direction=(1,-1,.8),explode=0,clip=False,alphas=None):
    r=vtk.vtkRenderer(); r.SetBackground(.96,.97,.98)
    win=vtk.vtkRenderWindow(); win.SetOffScreenRendering(1); win.SetSize(1800,1300); win.AddRenderer(r)
    pts=np.concatenate([m.reshape(-1,3) for m in meshes]); center=(pts.min(0)+pts.max(0))/2
    for i,m in enumerate(meshes):
        if clip:
            keep=m.mean(1)[:,2]<(center[2]+15)
            m=m[keep]
        if not len(m): continue
        shift=(m.reshape(-1,3).mean(0)-center)*explode
        r.AddActor(actor(m+shift,colors[i%len(colors)],alphas[i] if alphas else 1))
    cam=r.GetActiveCamera(); cam.SetFocalPoint(*center)
    cam.SetPosition(*(center+np.asarray(direction)*1000)); cam.SetViewUp(0,0,1 if direction[2]<2 else 0.001)
    if abs(direction[2])>2: cam.SetViewUp(1,0,0)
    cam.ParallelProjectionOn(); r.ResetCamera(); cam.Zoom(1.12)
    win.Render()
    w=vtk.vtkWindowToImageFilter(); w.SetInput(win); w.ReadFrontBufferOff(); w.Update()
    writer=vtk.vtkPNGWriter(); writer.SetFileName(str(FIG/f'{name}.png')); writer.SetInputConnection(w.GetOutputPort()); writer.Write()
    win.Finalize()

def plates():
    cad=list(np.load(OUT/'cad_meshes.npz').values())
    extra_path=OUT/'cad_extra_faces.npz'
    if extra_path.exists():
        extra=list(np.load(extra_path).values())
        if extra:cad.append(np.concatenate(extra))
    stls=list(np.load(OUT/'urdf_meshes.npz').values()); stls=[m*1000 for m in stls]
    # Axis-permutation registration: proper rotations only, no fitted scale.
    from scipy.spatial import cKDTree
    def samples(meshes,n):
        tri=np.concatenate(meshes);area=np.linalg.norm(np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0]),axis=1)
        rng=np.random.default_rng(20261005);sel=tri[rng.choice(len(tri),n,p=area/area.sum())]
        a=rng.random((n,1));b=rng.random((n,1));u=np.sqrt(a)
        return (1-u)*sel[:,0]+u*(1-b)*sel[:,1]+u*b*sel[:,2]
    src=samples(cad,60000);dst=samples(stls,120000);tree=cKDTree(dst)
    src_bounds=np.array(json.loads((OUT/'geometry.json').read_text())['step']['bounds_mm']).reshape(2,3)
    dpoints=np.concatenate([m.reshape(-1,3) for m in stls]);dc=(dpoints.min(0)+dpoints.max(0))/2
    candidates=[]
    for perm in itertools.permutations(range(3)):
        for signs in itertools.product((-1,1),repeat=3):
            rot=np.eye(3)[list(perm)]*np.array(signs)[:,None]
            if np.linalg.det(rot)<0:continue
            trans=dc-src_bounds.mean(0)@rot.T
            distances=tree.query(src[::10]@rot.T+trans)[0]
            candidates.append((np.median(distances),rot,trans))
    score,rot,trans=min(candidates,key=lambda c:c[0]);mapped=src@rot.T+trans
    distances=tree.query(mapped)[0]
    align=dict(rotation=rot.tolist(),translation_mm=trans.tolist(),scale=1.,method='24 proper signed axis permutations; bounding-box centering; minimum median nearest-sample distance',
        samples_step=60000,samples_stl=120000,seed=20261005,nearest_sample_distance_mm=dict(p50=float(np.median(distances)),p95=float(np.percentile(distances,95)),maximum=float(distances.max())),
        candidate_scores=sorted(float(c[0]) for c in candidates))
    (OUT/'registration.json').write_text(json.dumps(align,indent=2))
    cad=[m@rot.T+trans for m in cad]
    np.savez_compressed(OUT/'cad_aligned.npz',**{f'solid_{i+1}':m for i,m in enumerate(cad)})
    palette=[(.38,.57,.67),(.71,.76,.79),(.77,.52,.21),(.24,.32,.38),(.16,.47,.48),(.53,.58,.65)]
    # Neutral engineering false colors distinguish solids; not source materials.
    for name,d in [('iso_front',(1,-1,.75)),('iso_back',(-1,1,.7)),('front',(1,0,.001)),
                   ('rear',(-1,0,.001)),('left',(0,1,.001)),('right',(0,-1,.001)),('top',(0,0,3)),('bottom',(0,0,-3))]:
        render('cad_'+name,cad,palette,d)
    render('cad_exploded',cad,palette,(1,-1,.8),explode=.6)
    render('cad_open_view',cad,palette,(1,-1,1.4),clip=True)
    colors=[(.66,.71,.76),(.11,.15,.19),(.11,.15,.19)]
    for name,d in [('iso',(1,-1,.7)),('back',(-1,1,.7)),('top',(0,0,3)),('front',(1,0,.001)),('side',(0,-1,.001)),('bottom',(0,0,-3))]:
        render('urdf_'+name,stls,colors,d)
    render('urdf_exploded',stls,colors,(1,-1,.8),explode=.65)
    for i in range(3): render('mesh_'+str(i),[stls[i]],[colors[i]],(1,-1,.65))
    print('CAD and mesh plates completed',flush=True)

if __name__=='__main__':
    if '--render-only' not in sys.argv: extract()
    plates()
