"""
core/poi.py
Points of Interest (survey tasks) and the PoI manager.

Lifecycle:  PENDING -> ASSIGNED -> IN_PROGRESS -> COMPLETED
                ^          |            |
                +----------+------------+   release (UAV failed / re-tasked);
                                            survey progress is kept.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Iterator, Optional

import numpy as np


class PoIStatus(str, Enum):
    PENDING = "PENDING"          # waiting for a UAV
    ASSIGNED = "ASSIGNED"        # a UAV is on its way
    IN_PROGRESS = "IN_PROGRESS"  # a UAV is on station, survey time accumulating
    COMPLETED = "COMPLETED"


@dataclass(eq=False)
class PoI:
    poi_id: str
    position: np.ndarray            # ground point (x, y, 0) in local ENU metres
    priority: int = 1               # 1 (low) .. 5 (high)
    survey_time_s: float = 30.0     # time on station required to finish
    altitude_m: Optional[float] = None
    status: PoIStatus = PoIStatus.PENDING
    assigned_uav: Optional[int] = None
    progress_s: float = 0.0
    created_at_s: float = 0.0
    first_assigned_at_s: Optional[float] = None
    completed_at_s: Optional[float] = None
    completed_by: Optional[int] = None
    completed_early: bool = False

    def __post_init__(self) -> None:
        pos = np.asarray(self.position, dtype=float).reshape(-1)
        if pos.shape == (2,):
            pos = np.append(pos, 0.0)
        if pos.shape != (3,):
            raise ValueError(f"PoI {self.poi_id}: position must be (x, y) or (x, y, z)")
        self.position = pos
        if not 1 <= self.priority <= 5:
            raise ValueError(f"PoI {self.poi_id}: priority must be 1..5")
        if self.survey_time_s <= 0:
            raise ValueError(f"PoI {self.poi_id}: survey_time_s must be > 0")

    # ------------------------------------------------------------------ status
    @property
    def is_completed(self) -> bool:
        return self.status is PoIStatus.COMPLETED

    @property
    def progress_ratio(self) -> float:
        return min(1.0, self.progress_s / self.survey_time_s)

    def survey_waypoint(self, default_altitude_m: float) -> np.ndarray:
        alt = self.altitude_m if self.altitude_m is not None else default_altitude_m
        return np.array([self.position[0], self.position[1], alt])

    # ------------------------------------------------------------- transitions
    def assign(self, uav_id: int, t_s: float) -> None:
        if self.is_completed:
            raise ValueError(f"PoI {self.poi_id} is already completed")
        if self.assigned_uav is not None and self.assigned_uav != uav_id:
            raise ValueError(f"PoI {self.poi_id} is already assigned to UAV {self.assigned_uav}")
        self.assigned_uav = uav_id
        if self.status is PoIStatus.PENDING:
            self.status = PoIStatus.ASSIGNED
        if self.first_assigned_at_s is None:
            self.first_assigned_at_s = t_s

    def release(self) -> None:
        """Unassign the UAV; the PoI goes back to PENDING and keeps its progress."""
        if self.is_completed:
            return
        self.assigned_uav = None
        self.status = PoIStatus.PENDING

    def add_survey_time(self, dt: float, t_s: float) -> bool:
        """Accumulate on-station time. Returns True if this call completed the PoI."""
        if self.status not in (PoIStatus.ASSIGNED, PoIStatus.IN_PROGRESS):
            return False
        self.status = PoIStatus.IN_PROGRESS
        self.progress_s = min(self.survey_time_s, self.progress_s + dt)
        if self.progress_s >= self.survey_time_s - 1e-9:
            self.complete(t_s)
            return True
        return False

    def complete(self, t_s: float, early: bool = False) -> None:
        if self.is_completed:
            raise ValueError(f"PoI {self.poi_id} is already completed")
        self.status = PoIStatus.COMPLETED
        self.completed_at_s = t_s
        self.completed_by = self.assigned_uav
        self.completed_early = early
        if not early:
            self.progress_s = self.survey_time_s
        self.assigned_uav = None


class PoIManager:
    """Creates and tracks PoIs, their priorities and completion status."""

    def __init__(self) -> None:
        self._pois: dict[str, PoI] = {}

    def add(self, poi: PoI) -> PoI:
        if poi.poi_id in self._pois:
            raise ValueError(f"duplicate PoI id '{poi.poi_id}'")
        self._pois[poi.poi_id] = poi
        return poi

    def get(self, poi_id: str) -> PoI:
        try:
            return self._pois[poi_id]
        except KeyError:
            raise KeyError(f"unknown PoI '{poi_id}'") from None

    def __contains__(self, poi_id: object) -> bool:
        return poi_id in self._pois

    def __iter__(self) -> Iterator[PoI]:
        return iter(self._pois.values())

    def __len__(self) -> int:
        return len(self._pois)

    def pending(self) -> list[PoI]:
        """Unassigned, unfinished PoIs - highest priority first, then by id."""
        return sorted((p for p in self._pois.values() if p.status is PoIStatus.PENDING),
                      key=lambda p: (-p.priority, p.poi_id))

    def active(self) -> list[PoI]:
        return [p for p in self._pois.values() if p.status in (PoIStatus.ASSIGNED, PoIStatus.IN_PROGRESS)]

    def completed(self) -> list[PoI]:
        return [p for p in self._pois.values() if p.is_completed]

    def assigned_to(self, uav_id: int) -> Optional[PoI]:
        return next((p for p in self._pois.values() if p.assigned_uav == uav_id), None)

    @property
    def completion_rate(self) -> float:
        return len(self.completed()) / len(self._pois) if self._pois else 0.0

    @property
    def all_completed(self) -> bool:
        return bool(self._pois) and all(p.is_completed for p in self._pois.values())
