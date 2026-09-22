"""Geometría vial estática del mapa oficial; no utiliza anotaciones de objetos."""
import xml.etree.ElementTree as ET
import numpy as np
from pyproj import Transformer
from shapely import intersects_xy
from shapely.geometry import Polygon, box, mapping
from shapely.ops import unary_union

ORIGIN = (11.438043, 48.771731)  # lon, lat, urbaning.data.info.gps_origins['crossing1']


def load_road(path, roi=(-45, 45, -45, 45), margin=0.35):
    root = ET.parse(path).getroot()
    project = Transformer.from_crs('EPSG:4326', 'EPSG:32632', always_xy=True)
    origin = np.array(project.transform(*ORIGIN))
    nodes = {n.get('id'): np.array(project.transform(float(n.get('lon')), float(n.get('lat')))) - origin
             for n in root.findall('node')}
    ways = {w.get('id'): np.array([nodes[n.get('ref')] for n in w.findall('nd')]) for w in root.findall('way')}
    extent = box(roi[0], roi[2], roi[1], roi[3])
    polygons = []
    for relation in root.findall('relation'):
        tags = {t.get('k'): t.get('v') for t in relation.findall('tag')}
        if tags.get('type') != 'lanelet' or tags.get('subtype') != 'road':
            continue
        boundaries = {m.get('role'): ways[m.get('ref')] for m in relation.findall('member')
                      if m.get('type') == 'way' and m.get('role') in ('left', 'right')}
        left, right = boundaries['left'], boundaries['right']
        if np.linalg.norm(left[0] - right[0]) > np.linalg.norm(left[0] - right[-1]):
            right = right[::-1]
        polygon = Polygon(np.concatenate((left, right[::-1]))).buffer(0)
        if polygon.intersects(extent):
            polygons.append(polygon)
    if not polygons:
        raise ValueError('No hay carriles viales dentro de la ROI')
    return unary_union(polygons).buffer(margin).intersection(extent)


def inside(points, road):
    points = np.asarray(points)
    return intersects_xy(road, points[:, 0], points[:, 1])


def filter_boxes(boxes, road):
    if not boxes:
        return []
    mask = inside(np.array([b['center'][:2] for b in boxes]), road)
    return [b for b, valid in zip(boxes, mask) if valid]


def box_polygon(box):
    """Huella BEV orientada de una caja métrica."""
    angle = box['yaw_rad']
    rotation = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
    corners = np.array([[-1, -1], [1, -1], [1, 1], [-1, 1]])
    corners = corners * (np.array(box['dimensions'][:2]) / 2) @ rotation.T + box['center'][:2]
    return Polygon(corners)


def draw_road(ax, road):
    polygons = list(road.geoms) if hasattr(road, 'geoms') else [road]
    for polygon in polygons:
        ax.plot(*polygon.exterior.xy, color='tab:blue', linewidth=0.9)
        for ring in polygon.interiors:
            ax.plot(*ring.xy, color='tab:blue', linewidth=0.9)

def load_lane_segments(path):
    """Segmentos de ejes de carril para orientar observaciones parciales."""
    root = ET.parse(path).getroot()
    project = Transformer.from_crs('EPSG:4326', 'EPSG:32632', always_xy=True)
    origin = np.array(project.transform(*ORIGIN))
    nodes = {n.get('id'): np.array(project.transform(float(n.get('lon')),float(n.get('lat'))))-origin for n in root.findall('node')}
    ways = {w.get('id'): np.array([nodes[n.get('ref')] for n in w.findall('nd')]) for w in root.findall('way')}
    from shapely.geometry import LineString
    segments=[]
    for rel in root.findall('relation'):
        tags={t.get('k'):t.get('v') for t in rel.findall('tag')}
        if tags.get('type')!='lanelet' or tags.get('subtype')!='road':
            continue
        bounds={m.get('role'):ways[m.get('ref')] for m in rel.findall('member') if m.get('type')=='way' and m.get('role') in ('left','right')}
        left,right=bounds['left'],bounds['right']
        if np.linalg.norm(left[0]-right[0])>np.linalg.norm(left[0]-right[-1]):right=right[::-1]
        a,b=LineString(left),LineString(right)
        count=max(2,int(max(a.length,b.length)/2)+1)
        center=np.array([(np.array(a.interpolate(t,normalized=True).coords[0])+np.array(b.interpolate(t,normalized=True).coords[0]))/2 for t in np.linspace(0,1,count)])
        for start,end in zip(center[:-1],center[1:]):
            if np.linalg.norm(end-start)>0.01 and np.linalg.norm((start+end)/2)<100:
                segments.append([*start,*end])
    return np.array(segments)


def lane_heading(xy, segments):
    start,end=segments[:,:2],segments[:,2:]
    direction=end-start
    t=np.clip(np.sum((xy-start)*direction,axis=1)/np.sum(direction**2,axis=1),0,1)
    nearest=np.argmin(np.linalg.norm(start+t[:,None]*direction-xy,axis=1))
    return np.arctan2(direction[nearest,1],direction[nearest,0])
