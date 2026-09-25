"""
simulation/communication.py
Radio channel model of the *swarm research network* (UAV<->UAV and UAV<->GCS).
This is NOT the MAVLink telemetry link to Mission Planner.

Per link, every ``update_interval_s``:
    path loss   PL(d)  = PL0 + 10 * n * log10(d / 1 m)          (log-distance model)
    received    Prx    = Ptx + Gtx + Grx - PL(d) - obstacle loss + fading,  fading ~ N(0, sigma)
    SNR                = Prx - noise floor
    PDR                = logistic(SNR; mid, slope) * radio_health(a) * radio_health(b)
    estimate           = EWMA of PDR (what a real link-quality estimator would report)
    link up/down       with hysteresis (up >= link_up_pdr, down < link_down_pdr)
    latency            = hop_latency + retx_latency * (1/PDR - 1)   (expected retransmissions)
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any, Optional, Sequence

import numpy as np

from core.config import build
from core.uav import GCS_NODE_ID
from simulation.obstacles import ObstacleField

if TYPE_CHECKING:
    from core.world import World


@dataclass(frozen=True)
class CommParams:
    tx_power_dbm: float = 12.6           # UAV-UAV: 85 % PDR at 100 m, degrading beyond
    uav_antenna_gain_dbi: float = 0.0
    gcs_antenna_gain_dbi: float = 13.4   # UAV-GCS: 85 % PDR at ~328 m
    gcs_antenna_height_m: float = 10.0
    path_loss_ref_db: float = 40.0
    path_loss_exponent: float = 2.6
    noise_floor_dbm: float = -90.0
    snr_mid_db: float = 8.0
    snr_slope_db: float = 1.5
    fading_sigma_db: float = 1.0
    pdr_smoothing: float = 0.4
    link_up_pdr: float = 0.55
    link_down_pdr: float = 0.45
    hop_latency_ms: float = 5.0
    retx_latency_ms: float = 8.0
    update_interval_s: float = 0.5

    def __post_init__(self) -> None:
        if not 0 < self.pdr_smoothing <= 1:
            raise ValueError("pdr_smoothing must be in (0, 1]")
        if not 0 < self.link_down_pdr <= self.link_up_pdr < 1:
            raise ValueError("require 0 < link_down_pdr <= link_up_pdr < 1")
        if self.snr_slope_db <= 0 or self.path_loss_exponent <= 0 or self.update_interval_s <= 0:
            raise ValueError("snr_slope_db, path_loss_exponent and update_interval_s must be > 0")


@dataclass
class Link:
    a: int                 # lower node id (0 = GCS)
    b: int
    distance_m: float
    snr_db: float
    pdr: float             # smoothed estimate, 0..1
    latency_ms: float
    obstructed: bool
    up: bool

    def other(self, node: int) -> int:
        return self.b if node == self.a else self.a

    def to_dict(self) -> dict[str, Any]:
        return {"a": self.a, "b": self.b, "distance_m": round(self.distance_m, 1), "snr_db": round(self.snr_db, 1),
                "pdr": round(self.pdr, 3), "latency_ms": round(self.latency_ms, 1),
                "obstructed": self.obstructed, "up": self.up}


def link_key(a: int, b: int) -> tuple[int, int]:
    return (a, b) if a < b else (b, a)


class CommunicationModel:
    def __init__(self, params: CommParams, rng: np.random.Generator, obstacles: ObstacleField) -> None:
        self.params = params
        self.rng = rng
        self.obstacles = obstacles
        self.links: dict[tuple[int, int], Link] = {}
        self.positions: dict[int, np.ndarray] = {}
        self.last_update_s = -math.inf
        self.version = 0

    @classmethod
    def from_world(cls, world: "World", obstacles: ObstacleField) -> "CommunicationModel":
        params = build(CommParams, world.params.section("communication"), "communication")
        return cls(params, np.random.default_rng([world.seed, 101]), obstacles)

    # ------------------------------------------------------------ channel maths
    def _mean_snr_db(self, a: Sequence[float], b: Sequence[float], involves_gcs: bool) -> tuple[float, float, bool]:
        p = self.params
        d = max(1.0, math.dist(a, b))
        path_loss = p.path_loss_ref_db + 10.0 * p.path_loss_exponent * math.log10(d)
        gain = p.uav_antenna_gain_dbi + (p.gcs_antenna_gain_dbi if involves_gcs else p.uav_antenna_gain_dbi)
        obstacle_loss, obstructed = self.obstacles.attenuation_db(a, b)
        rx = p.tx_power_dbm + gain - path_loss - obstacle_loss
        return rx - p.noise_floor_dbm, d, obstructed

    def _pdr_from_snr(self, snr_db: float) -> float:
        z = (snr_db - self.params.snr_mid_db) / self.params.snr_slope_db
        return 1.0 / (1.0 + math.exp(-max(-50.0, min(50.0, z))))

    def predict_pdr(self, a: Sequence[float], b: Sequence[float], involves_gcs: bool = False) -> float:
        """Expected PDR of a healthy link (no fading) - used by planners."""
        snr, _, _ = self._mean_snr_db(a, b, involves_gcs)
        return self._pdr_from_snr(snr)

    def range_for_pdr(self, target_pdr: float, involves_gcs: bool = False) -> float:
        """Free-space (no obstacle) distance at which a healthy link reaches ``target_pdr``."""
        p = self.params
        snr_needed = p.snr_mid_db + p.snr_slope_db * math.log(target_pdr / (1.0 - target_pdr))
        gain = p.uav_antenna_gain_dbi + (p.gcs_antenna_gain_dbi if involves_gcs else p.uav_antenna_gain_dbi)
        max_path_loss = p.tx_power_dbm + gain - p.noise_floor_dbm - snr_needed
        return 10 ** ((max_path_loss - p.path_loss_ref_db) / (10.0 * p.path_loss_exponent))

    def latency_ms(self, pdr: float) -> float:
        return self.params.hop_latency_ms + self.params.retx_latency_ms * (1.0 / max(pdr, 0.05) - 1.0)

    # ------------------------------------------------------------- measurement
    def gcs_antenna_position(self, world: "World") -> np.ndarray:
        pos = world.state.gcs_position.copy()
        pos[2] += self.params.gcs_antenna_height_m
        return pos

    def update(self, world: "World", force: bool = False) -> bool:
        """Re-measure every link if the update interval elapsed. Returns True when links changed."""
        if not force and world.t - self.last_update_s < self.params.update_interval_s - 1e-9:
            return False
        self.last_update_s = world.t
        p = self.params
        positions = {GCS_NODE_ID: self.gcs_antenna_position(world)}
        health = {GCS_NODE_ID: 1.0}
        for uav in world.state.uavs.values():
            if uav.is_operational and uav.is_airborne:
                positions[uav.uav_id] = uav.position.copy()
                health[uav.uav_id] = uav.comm.radio_health

        nodes = sorted(positions)
        fresh: dict[tuple[int, int], Link] = {}
        for i, a in enumerate(nodes):
            for b in nodes[i + 1:]:
                snr, dist, obstructed = self._mean_snr_db(positions[a], positions[b], a == GCS_NODE_ID)
                if p.fading_sigma_db > 0:
                    snr += float(self.rng.normal(0.0, p.fading_sigma_db))
                measured = self._pdr_from_snr(snr) * health[a] * health[b]
                prev = self.links.get((a, b))
                if prev is None:
                    pdr, was_up = measured, measured >= p.link_up_pdr
                else:
                    pdr = p.pdr_smoothing * measured + (1.0 - p.pdr_smoothing) * prev.pdr
                    was_up = prev.up
                up = pdr >= p.link_down_pdr if was_up else pdr >= p.link_up_pdr
                fresh[(a, b)] = Link(a, b, dist, snr, pdr, self.latency_ms(pdr), obstructed, up)
        self.links = fresh
        self.positions = positions
        self.version += 1
        return True

    def link(self, a: int, b: int) -> Optional[Link]:
        return self.links.get(link_key(a, b))

    def links_of(self, node: int) -> list[Link]:
        return [lk for lk in self.links.values() if node in (lk.a, lk.b)]

    def up_links(self) -> list[Link]:
        return [lk for lk in self.links.values() if lk.up]
