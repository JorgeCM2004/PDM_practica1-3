"""Detector geométrico mejorado. Solo recibe puntos del fotograma y mapa estático."""
import numpy as np
from scipy.spatial import cKDTree
from scipy.ndimage import median_filter, gaussian_filter
from shapely.geometry import MultiPoint
from road import inside, filter_boxes, box_polygon

DEFAULTS = {'eps': 0.85, 'min_points': 4, 'min_height': 0.4,
            'min_length': 1.0, 'min_width': 0.3, 'complete_boxes': True}


def local_ground(points, plane):
    """Residuos de suelo por celdas; solo cotas cercanas al plano inicial."""
    size = 3.0
    origin = np.floor(points[:, :2].min(axis=0) / size) * size
    indices = np.floor((points[:, :2] - origin) / size).astype(int)
    shape = tuple(indices.max(axis=0) + 1)
    residual = points[:, 2] - points[:, :2] @ plane[:2] - plane[2]
    grid = np.full(shape, np.inf)
    valid = (residual > -0.4) & (residual < 0.3)
    np.minimum.at(grid, tuple(indices[valid].T), residual[valid])
    known = np.isfinite(grid)
    if known.any():
        locations = np.column_stack(np.nonzero(known))
        missing = np.column_stack(np.nonzero(~known))
        if len(missing):
            nearest = cKDTree(locations).query(missing)[1]
            grid[tuple(missing.T)] = grid[tuple(locations[nearest].T)]
        grid = gaussian_filter(median_filter(grid, size=3), sigma=1)
    else:
        grid[:] = 0
    return residual - grid[tuple(indices.T)], (grid, origin, size)


def box_from_cluster(points, plane, ground, complete=True, lanes=None, yaw_override=None, placement='away'):
    hull = MultiPoint(points[:, :2]).minimum_rotated_rectangle
    if hull.geom_type != 'Polygon':
        return None
    corners = np.array(hull.exterior.coords)[:4]
    edges = np.roll(corners, -1, axis=0) - corners
    lengths = np.linalg.norm(edges, axis=1)
    edge = edges[np.argmax(lengths)]
    yaw = np.arctan2(edge[1], edge[0])
    if lanes is not None and (lengths.max() < 3.2 or lengths.min() < 0.8):
        from road import lane_heading
        yaw = lane_heading(points[:, :2].mean(axis=0), lanes)
    if yaw_override is not None:
        yaw = yaw_override
    rotation = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
    local = points[:, :2] @ rotation
    lo, hi = local.min(axis=0), local.max(axis=0)
    observed = hi - lo
    center = (lo + hi) / 2
    observed_center = center @ rotation.T
    dimensions = observed.copy()
    if complete:
        # Priorización conservadora para carrocerías parcialmente visibles.
        dimensions = np.maximum(dimensions, [4.0, 1.8])
        # Elegir el mástil más cercano; expandir alejándose de él si falta una cara.
        xy = center @ rotation.T
        sensors = np.array([[-20.27, -2.59], [18.30, 1.46]])
        viewpoint = sensors[np.argmin(np.linalg.norm(sensors - xy, axis=1))] @ rotation
        shift = np.sign(center - viewpoint) * (dimensions - observed) / 2
        if placement == 'away':
            center += shift
        elif placement == 'opposite':
            center -= shift
        elif placement != 'centered':
            raise ValueError('Colocación de caja desconocida')
    xy = center @ rotation.T
    grid, origin, size = ground
    index = np.clip(np.floor((xy-origin)/size).astype(int), 0, np.array(grid.shape)-1)
    bottom = float(np.r_[xy, 1] @ plane + grid[tuple(index)])
    height = float(np.percentile(points[:, 2], 98) - bottom)
    return {'center': [float(xy[0]), float(xy[1]), bottom + height/2],
            'dimensions': [float(dimensions[0]), float(dimensions[1]), height],
            'observed_dimensions_xy': observed.tolist(),
            'observed_center_xy': observed_center.tolist(),
            'yaw_rad': float((yaw+np.pi/2) % np.pi - np.pi/2),
            'class': 'vehicle_candidate', 'num_points': len(points)}


def resolve_fragments(groups, plane, ground, lanes, config):
    """Fusión espacial de fragmentos y completado sin invadir cajas con más evidencia.

    Solo usa puntos del instante actual. No usa etiquetas ni asociaciones temporales.
    """
    entries = []
    for group in groups:
        box = box_from_cluster(group, plane, ground, config['complete_boxes'], lanes)
        if box is not None:
            entries.append((group, box, 1))
    merges = config.get('merge_fragments', True)
    if merges:
        changed = True
        while changed:
            changed = False
            for i in range(len(entries)):
                if changed:
                    break
                for j in range(i + 1, len(entries)):
                    ga, a, na = entries[i]
                    gb, b, nb = entries[j]
                    if na + nb > 3:
                        continue
                    angle = abs(np.arctan2(np.sin(2*(a['yaw_rad']-b['yaw_rad'])), np.cos(2*(a['yaw_rad']-b['yaw_rad'])))) / 2
                    if angle > np.deg2rad(25):
                        continue
                    pa, pb = box_polygon(a), box_polygon(b)
                    if pa.intersection(pb).area / min(pa.area, pb.area) < 0.12:
                        continue
                    if cKDTree(ga[:, :2]).query(gb[:, :2])[0].min() > 1.8:
                        continue
                    combined = np.concatenate((ga, gb))
                    merged = box_from_cluster(combined, plane, ground, config['complete_boxes'], lanes)
                    if merged is None:
                        continue
                    length, width = merged['observed_dimensions_xy']
                    if length > 7.0 or width > 2.8:
                        continue
                    # La unión debe explicar un único vehículo, sin gran superficie vacía.
                    if box_polygon(merged).area > 1.3*(pa.area+pb.area):
                        continue
                    entries[i] = (combined, merged, na + nb)
                    del entries[j]
                    changed = True
                    break
    kept, polygons = [], []
    for group, original, fragments in sorted(entries, key=lambda item: item[1]['num_points'], reverse=True):
        length, width = original['observed_dimensions_xy']
        if not (config['min_length'] <= length <= 18 and config['min_width'] <= width <= 3.5
                and 0.65 <= original['dimensions'][2] <= 4.5):
            continue
        p = box_polygon(original)
        conflicts = [q for q in polygons if p.intersection(q).area > 0.15]
        chosen = original
        if conflicts and config.get('resolve_overlap', True):
            proposals = [(0.0, original)]
            yaws = [original['yaw_rad']]
            # Con poco soporte, el carril más cercano puede tener la dirección perpendicular.
            if max(original['observed_dimensions_xy']) < 3.2:
                yaws.append(original['yaw_rad'] + np.pi/2)
            for yaw in yaws:
                for placement in ('away', 'centered', 'opposite'):
                    candidate = box_from_cluster(group, plane, ground, config['complete_boxes'], lanes, yaw, placement)
                    if candidate is None or candidate['dimensions'][1] > 3.5:
                        continue
                    shift = np.linalg.norm(np.array(candidate['center'][:2])-original['center'][:2])
                    penalty = 0.05*shift + (0.12 if yaw != original['yaw_rad'] else 0)
                    proposals.append((penalty, candidate))
            def cost(proposal):
                prior, box = proposal
                polygon = box_polygon(box)
                return prior + 10*sum(polygon.intersection(q).area for q in polygons)
            chosen = min(proposals, key=cost)[1]
            p = box_polygon(chosen)
        # El solapamiento fuerte restante se considera una hipótesis duplicada.
        if any(p.intersection(q).area/min(p.area,q.area) > config.get('nms_overlap', 0.5) for q in polygons):
            continue
        chosen['fragments_merged'] = fragments
        chosen['overlap_resolved'] = chosen is not original
        kept.append(chosen)
        polygons.append(p)
    return kept


def detect_improved(points, roi, voxel, road, config=None, lanes=None):
    # Importación diferida evita dependencia circular con la CLI.
    from main import voxelize, ground_plane, clusters
    config = {**DEFAULTS, **(config or {})}
    xmin, xmax, ymin, ymax = roi
    mask = ((points[:, 0]>=xmin)&(points[:, 0]<=xmax)&(points[:, 1]>=ymin)&(points[:, 1]<=ymax))
    points = voxelize(points[mask], voxel)
    plane = ground_plane(points)
    height, ground = local_ground(points, plane)
    # Margen de contexto para no cortar la carrocería al borde de la calzada.
    foreground = points[(height>=config['min_height'])&(height<=4.5)&inside(points, road.buffer(0.5))]
    groups = list(clusters(foreground, config['eps'], config['min_points']))
    if config.get('spatial_refinement', True):
        boxes = resolve_fragments(groups, plane, ground, lanes, config)
        return points, filter_boxes(boxes, road), plane
    boxes = []
    for group in groups:
        box = box_from_cluster(group, plane, ground, config['complete_boxes'], lanes)
        if box is None:
            continue
        length, width = box['observed_dimensions_xy']
        height = box['dimensions'][2]
        if config['min_length']<=length<=18 and config['min_width']<=width<=3.5 and 0.65<=height<=4.5:
            boxes.append(box)
    boxes = filter_boxes(boxes, road)
    # Supresión de cajas duplicadas por fragmentos; no usa identidades temporales.
    kept, polygons = [], []
    for box in sorted(boxes, key=lambda b: b['num_points'], reverse=True):
        p = box_polygon(box)
        if any(p.intersection(q).area/min(p.area,q.area)>0.5 for q in polygons):
            continue
        kept.append(box)
        polygons.append(p)
    return points, kept, plane
