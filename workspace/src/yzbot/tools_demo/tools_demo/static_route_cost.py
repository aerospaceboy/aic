"""Map-aware transport distance estimates; never a motion or safety planner.

The task scheduler used straight chords between navigation anchors, even
when a wall forces Nav2 to drive around its end. This graph estimates that
detour. Dynamic obstacles and actual execution still belong to Nav2 and the
independent safety guards; a finite cost is not a collision certificate.
"""

from collections import OrderedDict
import math
from pathlib import Path

import cv2
import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import dijkstra
import yaml


class StaticRouteCost:
    """Cache obstacle-aware distances on a conservative coarse static grid."""

    def __init__(self, path, clearance=.48, stride=2):
        if not math.isfinite(clearance) or clearance < 0:
            raise ValueError('Route-cost clearance must be nonnegative')
        if not isinstance(stride, int) or stride < 1:
            raise ValueError('Route-cost stride must be a positive integer')
        path = Path(path).resolve()
        cfg = yaml.safe_load(path.read_text())
        resolution = float(cfg['resolution'])
        self.origin = tuple(float(v) for v in cfg['origin'])
        if (not math.isfinite(resolution) or resolution <= 0
                or len(self.origin) != 3
                or not all(math.isfinite(v) for v in self.origin)
                or abs(self.origin[2]) > 1e-9
                or cfg.get('mode', 'trinary') != 'trinary'):
            raise ValueError('Unsupported static route-cost map geometry')
        threshold = float(cfg['free_thresh'])
        if not 0 < threshold < float(cfg['occupied_thresh']) < 1:
            raise ValueError('Invalid static map thresholds')
        image_path = path.parent / cfg['image']
        image = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)
        if image is None:
            raise ValueError('Cannot read static route-cost map image')
        occupancy = image if int(cfg.get('negate', 0)) else 255 - image
        free = np.flipud(occupancy.astype(float) / 255. < threshold)
        self.raw_free = free
        self.raw_resolution = resolution
        # Padding treats the outside as occupied, including entirely white
        # fixtures. Pool ALL native cells so coarsening cannot open a wall.
        padded = np.pad(free.astype(np.uint8), 1)
        distance = cv2.distanceTransform(
            padded, cv2.DIST_L2, cv2.DIST_MASK_PRECISE)[1:-1, 1:-1] * resolution
        safe = free & (distance >= clearance)
        h, w = (size // stride for size in safe.shape)
        if h < 1 or w < 1:
            raise ValueError('Static map is smaller than the coarse stride')
        self.free = safe[:h*stride, :w*stride].reshape(
            h, stride, w, stride).all(axis=(1, 3))
        self.resolution = resolution * stride
        self.height, self.width = self.free.shape
        ids = np.arange(h*w).reshape(h, w)
        rows, columns, weights = [], [], []
        for dy, dx in ((1, 0), (0, 1), (1, 1), (1, -1)):
            y0, y1 = max(0, -dy), min(h, h-dy)
            x0, x1 = max(0, -dx), min(w, w-dx)
            a = (slice(y0, y1), slice(x0, x1))
            b = (slice(y0+dy, y1+dy), slice(x0+dx, x1+dx))
            valid = self.free[a] & self.free[b]
            if dx and dy:
                valid &= (self.free[y0:y1, x0+dx:x1+dx]
                          & self.free[y0+dy:y1+dy, x0:x1])
            source, target = ids[a][valid], ids[b][valid]
            cost = np.full(source.size, self.resolution * math.hypot(dx, dy))
            rows.extend((source, target))
            columns.extend((target, source))
            weights.extend((cost, cost))
        self.graph = coo_matrix((np.concatenate(weights),
                                (np.concatenate(rows), np.concatenate(columns))),
                               shape=(h*w, h*w)).tocsr()
        self.fields = OrderedDict()
        self.pairs = OrderedDict()
        self.identity = (str(path), path.stat().st_mtime_ns,
                         str(image_path.resolve()), image_path.stat().st_mtime_ns,
                         clearance, stride)

    @staticmethod
    def _line_free(a, b, free, resolution, origin):
        count = max(2, math.ceil(math.dist(a, b) / (resolution * .4)) + 1)
        t = np.linspace(0., 1., count)
        x = np.floor((a[0] + t*(b[0]-a[0])-origin[0]) / resolution).astype(int)
        y = np.floor((a[1] + t*(b[1]-a[1])-origin[1]) / resolution).astype(int)
        if (not np.all((x >= 0) & (x < free.shape[1])
                       & (y >= 0) & (y < free.shape[0]))
                or not np.all(free[y, x])):
            return False
        diagonal = (x[1:] != x[:-1]) & (y[1:] != y[:-1])
        return bool(np.all(free[y[:-1][diagonal], x[1:][diagonal]])
                    and np.all(free[y[1:][diagonal], x[:-1][diagonal]]))

    def _cell(self, point):
        ix = math.floor((point[0]-self.origin[0]) / self.resolution)
        iy = math.floor((point[1]-self.origin[1]) / self.resolution)
        candidates = []
        bound = math.ceil(.30 / self.resolution) + 1
        for dy in range(-bound, bound+1):
            for dx in range(-bound, bound+1):
                x, y = ix+dx, iy+dy
                if not (0 <= x < self.width and 0 <= y < self.height
                        and self.free[y, x]):
                    continue
                center = (self.origin[0]+(x+.5)*self.resolution,
                          self.origin[1]+(y+.5)*self.resolution)
                distance = math.dist(point, center)
                if distance <= .30 and self._line_free(
                        point, center, self.raw_free, self.raw_resolution, self.origin):
                    candidates.append((distance, y*self.width+x))
        return min(candidates) if candidates else (math.inf, None)

    def distance(self, a, b):
        """Return metres, or infinity for unavailable/disconnected endpoints."""
        a, b = tuple(map(float, a[:2])), tuple(map(float, b[:2]))
        if len(a) != 2 or len(b) != 2 or not all(map(math.isfinite, (*a, *b))):
            raise ValueError('Route-cost endpoints must be finite 2D points')
        key = tuple(sorted((a, b)))
        if key in self.pairs:
            self.pairs.move_to_end(key)
            return self.pairs[key]
        direct = math.dist(a, b)
        if self._line_free(a, b, self.free, self.resolution, self.origin):
            result = direct
        else:
            da, ia = self._cell(a)
            db, ib = self._cell(b)
            result = math.inf
            if ia is not None and ib is not None:
                if ib not in self.fields:
                    self.fields[ib] = dijkstra(self.graph, directed=False, indices=ib)
                    if len(self.fields) > 64:
                        self.fields.popitem(last=False)
                self.fields.move_to_end(ib)
                result = max(direct, da + float(self.fields[ib][ia]) + db)
        self.pairs[key] = result
        if len(self.pairs) > 8192:
            self.pairs.popitem(last=False)
        return result


def load_static_route_cost(owner, path, clearance=.48):
    """Reuse a graph between replans; reload when YAML or image changes."""
    cached = getattr(owner, '_static_route_cost', None)
    if cached is not None:
        name, yaml_stamp, image, image_stamp, margin, _ = cached.identity
        if (name == str(Path(path).resolve()) and margin == clearance
                and Path(name).stat().st_mtime_ns == yaml_stamp
                and Path(image).stat().st_mtime_ns == image_stamp):
            return cached
    cached = StaticRouteCost(path, clearance)
    owner._static_route_cost = cached
    return cached
