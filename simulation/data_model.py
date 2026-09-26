"""
simulation/data_model.py
Imagery / situational data produced at PoIs and carried back to the GCS.

* A UAV surveying a PoI generates ``imagery_rate_mbps`` into its on-board buffer.
* Whenever the UAV has a route to the GCS, the buffer drains (FIFO) at the
  route goodput = link_capacity / ETX. ETX (expected transmissions, the sum of
  1 / PDR over the hops) charges every hop its share of the channel plus its
  link-layer retransmissions - the same per-hop retransmission assumption the
  router's cost and the latency model make, so a lossy hop slows delivery
  rather than silently dropping data. (A route given only a hop count and an
  end-to-end PDR drains at link_capacity / hop_count * PDR.)
* A disconnected UAV keeps its data (store-and-forward / data ferrying) until
  it reconnects. A failed UAV loses its buffer.

Metrics: generated, delivered, lost, delivery ratio, and delivery delay
(time from capture to arrival at the GCS).
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any

from core.config import build
from core.poi import PoIStatus
from core.uav import UAVRole

if TYPE_CHECKING:
    from core.world import World


@dataclass(frozen=True)
class DataParams:
    imagery_rate_mbps: float = 3.0     # megabits per second while surveying
    link_capacity_mbps: float = 40.0   # single-hop radio capacity
    buffer_capacity_mb: float = 4000.0
    pad_download_mbps: float = 100.0   # wired download once a UAV is back on its pad
    live_delay_s: float = 10.0         # imagery arriving within this delay counts as live

    def __post_init__(self) -> None:
        if self.imagery_rate_mbps < 0 or self.link_capacity_mbps <= 0 or self.buffer_capacity_mb <= 0:
            raise ValueError("data rates and capacities must be positive")


class DataModel:
    CHUNK_S = 1.0  # data captured within the same second is merged into one chunk

    def __init__(self, params: DataParams) -> None:
        self.params = params
        self._buffers: dict[int, deque[list]] = {}  # uav_id -> [[capture_time_s, megabits, poi_id], ...]
        self.generated_mb = 0.0
        self.delivered_mb = 0.0
        self.live_mb = 0.0     # delivered within live_delay_s of capture (real-time situational awareness)
        self.lost_mb = 0.0
        self._delay_weighted = 0.0
        self.max_delay_s = 0.0
        self.delivered_by_poi: dict[str, float] = {}

    @classmethod
    def from_world(cls, world: "World") -> "DataModel":
        return cls(build(DataParams, world.params.section("data"), "data"))

    def buffer_mb(self, uav_id: int) -> float:
        return sum(mb for _, mb, *_ in self._buffers.get(uav_id, ()))

    @property
    def buffered_mb(self) -> float:
        """Imagery still on board any UAV, not yet at the GCS."""
        return sum(self.buffer_mb(uid) for uid in self._buffers)

    def update(self, world: "World", dt: float) -> None:
        p, t = self.params, world.t
        for uav in world.state.uavs.values():
            buf = self._buffers.setdefault(uav.uav_id, deque())
            if not uav.is_operational:
                if buf:
                    self.lost_mb += sum(c[1] for c in buf)
                    buf.clear()
                continue
            # capture
            if uav.role is UAVRole.SURVEY and uav.assigned_poi is not None:
                poi = world.state.pois.get(uav.assigned_poi)
                if poi.status is PoIStatus.IN_PROGRESS and poi.assigned_uav == uav.uav_id:
                    mb = p.imagery_rate_mbps * dt
                    if self.buffer_mb(uav.uav_id) + mb > p.buffer_capacity_mb:
                        self.lost_mb += mb
                    else:
                        if buf and t - buf[-1][0] < self.CHUNK_S and buf[-1][2] == poi.poi_id:
                            buf[-1][1] += mb
                        else:
                            buf.append([t, mb, poi.poi_id])
                        self.generated_mb += mb
            # offload: over the multi-hop route in flight, or by cable on the home pad
            comm = uav.comm
            on_pad = not uav.is_airborne and uav.horizontal_distance_to(uav.home) <= 5.0
            if buf and (on_pad or (comm.connected and comm.hop_count)):
                if on_pad:
                    budget = p.pad_download_mbps * dt
                elif comm.etx:
                    budget = p.link_capacity_mbps / comm.etx * dt
                else:
                    budget = p.link_capacity_mbps / comm.hop_count * comm.pdr * dt
                while buf and budget > 1e-12:
                    chunk = buf[0]
                    sent = min(chunk[1], budget)
                    chunk[1] -= sent
                    budget -= sent
                    self._record_delivery(t - chunk[0], sent, chunk[2])
                    if chunk[1] <= 1e-9:
                        buf.popleft()

    def _record_delivery(self, delay_s: float, mb: float, poi_id: str) -> None:
        self.delivered_mb += mb
        if delay_s <= self.params.live_delay_s:
            self.live_mb += mb
        self._delay_weighted += delay_s * mb
        self.max_delay_s = max(self.max_delay_s, delay_s)
        self.delivered_by_poi[poi_id] = self.delivered_by_poi.get(poi_id, 0.0) + mb

    def stats(self) -> dict[str, Any]:
        buffered = self.buffered_mb
        return {
            "generated_mb": round(self.generated_mb, 1),
            "delivered_mb": round(self.delivered_mb, 1),
            "buffered_mb": round(buffered, 1),
            "lost_mb": round(self.lost_mb, 1),
            "delivery_ratio": round(self.delivered_mb / self.generated_mb, 4) if self.generated_mb else None,
            "live_mb": round(self.live_mb, 1),
            "live_ratio": round(self.live_mb / self.generated_mb, 4) if self.generated_mb else None,
            "mean_delay_s": round(self._delay_weighted / self.delivered_mb, 2) if self.delivered_mb else None,
            "max_delay_s": round(self.max_delay_s, 2),
        }
