from __future__ import annotations
import math
from typing import Any
import cv2
import numpy as np
from geometry_msgs.msg import Point32, PolygonStamped
from nav_msgs.msg import OccupancyGrid
from frontier_explorer.geometry_utils import normalize_polygon

class MapProcessor:
    def __init__(self, logger: Any) -> None:
        """
        Initialize the MapProcessor.
        logger: The ROS2 node logger to write info/warning logs.
        """
        self._logger = logger

    def extract_boundary_and_obstacles(
        self,
        msg: OccupancyGrid,
        map_morph_close_radius: int,
        map_erode_m: float,
        map_contour_epsilon: float,
        obstacle_min_area_m2: float,
        obstacle_dilate_m: float,
        default_frame_id: str = 'map'
    ) -> tuple[PolygonStamped | None, list[list[tuple[float, float]]]]:
        """
        Extract the largest outer boundary polygon and inner obstacle polygons
        from the occupancy grid map.
        
        Returns:
            A tuple (boundary_polygon, obstacle_polygons).
            boundary_polygon: PolygonStamped or None if extraction fails.
            obstacle_polygons: List of polygons, each as a list of (x, y) coordinate pairs.
        """
        h = msg.info.height
        w = msg.info.width
        if h == 0 or w == 0:
            return None, []

        grid = np.array(msg.data, dtype=np.int8).reshape((h, w))
        free_mask = np.uint8(np.where(grid == 0, 255, 0))

        # Morphological closing fills small gaps between free cells (SLAM noise)
        if map_morph_close_radius > 0:
            kernel = cv2.getStructuringElement(
                cv2.MORPH_ELLIPSE, (2 * map_morph_close_radius + 1, 2 * map_morph_close_radius + 1)
            )
            free_mask = cv2.morphologyEx(free_mask, cv2.MORPH_CLOSE, kernel)

        # Snapshot the mask BEFORE erosion for obstacle detection
        pre_erode_mask = free_mask.copy()
        res = msg.info.resolution
        erode_r = max(0, round(map_erode_m / res))
        if erode_r > 0:
            kernel = cv2.getStructuringElement(
                cv2.MORPH_ELLIPSE, (2 * erode_r + 1, 2 * erode_r + 1)
            )
            free_mask = cv2.erode(free_mask, kernel)

        # Use RETR_CCOMP to retrieve both outer boundary AND inner holes (obstacles).
        contours, hierarchy = cv2.findContours(
            free_mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE
        )
        if not contours or hierarchy is None:
            return None, []

        # Outer contours have parent == -1 in hierarchy[0][i][3]
        outer_contours = [
            c for i, c in enumerate(contours) if hierarchy[0][i][3] == -1
        ]
        if not outer_contours:
            return None, []

        largest = max(outer_contours, key=cv2.contourArea)
        original_area = cv2.contourArea(largest)

        # Reject trivially small contours
        min_area_px = 1.0 / (res ** 2)
        if original_area < min_area_px:
            return None, []

        epsilon_px = map_contour_epsilon / res
        approx = cv2.approxPolyDP(largest, epsilon_px, closed=True)

        if len(approx) <= 4:
            approx_area = cv2.contourArea(approx)
            if approx_area > original_area * 1.05 or len(approx) < 3 or self._is_bounding_rect(approx, largest):
                for scale in (0.5, 0.25, 0.1):
                    smaller_eps = epsilon_px * scale
                    retry = cv2.approxPolyDP(largest, smaller_eps, closed=True)
                    retry_area = cv2.contourArea(retry)
                    if len(retry) >= 5 and retry_area <= original_area * 1.05 and not self._is_bounding_rect(retry, largest):
                        approx = retry
                        if self._logger:
                            self._logger.info(
                                f'approxPolyDP retry with eps={smaller_eps:.1f}px '
                                f'gave {len(approx)} points'
                            )
                        break
                else:
                    if self._logger:
                        self._logger.warn(
                            'Map contour simplification produced a degenerate polygon; '
                            'skipping polygon update. Try refreshing the map again.'
                        )
                    return None, []

        if len(approx) < 3:
            return None, []

        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y
        poly = PolygonStamped()
        poly.header.stamp = msg.header.stamp
        poly.header.frame_id = msg.header.frame_id if msg.header.frame_id else default_frame_id

        for pt in approx:
            col = float(pt[0][0])
            row = float(pt[0][1])
            poly.polygon.points.append(Point32(
                x=origin_x + col * res,
                y=origin_y + row * res,
                z=0.0,
            ))

        normalized = normalize_polygon(poly, default_frame_id)
        if normalized is None:
            return None, []

        # --- Extract inner obstacle contours ---
        obstacle_min_px = obstacle_min_area_m2 / (res ** 2)
        obs_dilate_r = max(1, round(obstacle_dilate_m / res))

        obs_mask = cv2.bitwise_not(pre_erode_mask)
        outer_fill = np.zeros_like(obs_mask)
        cv2.drawContours(outer_fill, [largest], -1, 255, thickness=cv2.FILLED)
        obs_mask = cv2.bitwise_and(obs_mask, outer_fill)

        if obs_dilate_r > 0:
            dk = cv2.getStructuringElement(
                cv2.MORPH_ELLIPSE, (2 * obs_dilate_r + 1, 2 * obs_dilate_r + 1)
            )
            obs_mask = cv2.dilate(obs_mask, dk)

        obs_contours, _ = cv2.findContours(
            obs_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        obstacle_polygons: list[list[tuple[float, float]]] = []
        for obs_c in obs_contours:
            if cv2.contourArea(obs_c) < obstacle_min_px:
                continue
            obs_approx = cv2.approxPolyDP(obs_c, epsilon_px, closed=True)
            if len(obs_approx) < 3:
                continue
            pts: list[tuple[float, float]] = []
            for pt in obs_approx:
                col = float(pt[0][0])
                row = float(pt[0][1])
                pts.append((origin_x + col * res, origin_y + row * res))
            if pts[0] != pts[-1]:
                pts.append(pts[0])
            obstacle_polygons.append(pts)

        return normalized, obstacle_polygons

    def _is_bounding_rect(self, poly_pts: np.ndarray, contour: np.ndarray) -> bool:
        """Return True if poly_pts is effectively the axis-aligned or rotated bounding rectangle of contour."""
        if len(poly_pts) != 4:
            return False
        p_area = cv2.contourArea(poly_pts)
        _x, _y, bw, bh = cv2.boundingRect(contour)
        bbox_area = float(bw * bh)
        if bbox_area > 0 and abs(p_area - bbox_area) / bbox_area < 0.05:
            return True
        (_, (rw, rh), _) = cv2.minAreaRect(contour)
        min_rect_area = float(rw * rh)
        if min_rect_area > 0 and abs(p_area - min_rect_area) / min_rect_area < 0.05:
            return True
        return False
