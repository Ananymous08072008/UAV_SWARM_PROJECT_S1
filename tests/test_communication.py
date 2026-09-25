"""Stage 2 tests: radio channel model and obstacles."""

import numpy as np
import pytest

from simulation.communication import CommParams, CommunicationModel
from simulation.obstacles import Obstacle, ObstacleField, square
from tests.helpers import make_world


def model(obstacles: ObstacleField | None = None, **params) -> CommunicationModel:
    return CommunicationModel(CommParams(**params), np.random.default_rng(1), obstacles or ObstacleField())


def test_pdr_falls_with_distance():
    comm = model()
    a = np.array([0.0, 0.0, 50.0])
    pdrs = [comm.predict_pdr(a, np.array([d, 0.0, 50.0])) for d in (50, 150, 250, 400)]
    assert pdrs == sorted(pdrs, reverse=True)
    assert pdrs[0] > 0.99 and pdrs[-1] < 0.05


def test_range_for_pdr_is_the_inverse_of_predict():
    comm = model()
    for target in (0.5, 0.85, 0.95):
        distance = comm.range_for_pdr(target)
        assert comm.predict_pdr(np.array([0, 0, 0.0]), np.array([distance, 0, 0.0])) == pytest.approx(target, abs=1e-6)


def test_gcs_antenna_reaches_further():
    comm = model()
    assert comm.range_for_pdr(0.85, involves_gcs=True) > comm.range_for_pdr(0.85) * 1.3


def test_uav_to_uav_range_is_100_m_and_links_degrade_beyond_it():
    comm = model()
    assert comm.range_for_pdr(0.85) == pytest.approx(100.0, abs=0.5)

    def pdr(d):
        return comm.predict_pdr(np.array([0, 0, 60.0]), np.array([d, 0, 60.0]))

    assert pdr(50) > 0.99 and pdr(100) >= 0.85 - 1e-3
    assert pdr(100) > pdr(110) > pdr(120) > pdr(130)
    assert pdr(120) < 0.65                              # clearly degraded past the range
    assert pdr(140) < CommParams().link_down_pdr        # and the link drops out


def test_obstacle_blocks_a_link_but_not_a_detour():
    field = ObstacleField([square("B1", (40, 0), 24, height_m=60.0, attenuation_db=40.0)])
    comm = model(field)
    through = comm.predict_pdr(np.array([0, 0, 50.0]), np.array([80, 0, 50.0]))
    around = comm.predict_pdr(np.array([0, 0, 50.0]), np.array([80, 48, 50.0]))
    assert through < 0.05 < around
    # flying above the obstacle clears the link again
    assert comm.predict_pdr(np.array([0, 0, 80.0]), np.array([80, 0, 80.0])) > 0.5


def test_obstacle_geometry():
    obstacle = Obstacle("O", ((0, 0), (100, 0), (100, 100), (0, 100)), height_m=50.0)
    assert obstacle.contains_xy(50, 50) and not obstacle.contains_xy(150, 50)
    assert obstacle.blocks((-50, 50, 10), (150, 50, 10))
    assert not obstacle.blocks((-50, 50, 60), (150, 50, 60))      # over the top
    assert not obstacle.blocks((-50, -50, 10), (150, -50, 10))    # beside it
    assert obstacle.crosses_xy((-50, 50), (150, 50))


def test_measurement_is_smoothed_and_hysteretic():
    world = make_world()
    comm = CommunicationModel.from_world(world, ObstacleField())
    world.goto(1, (100, 0, 50), "test")
    world.goto(2, (160, 0, 50), "test")
    for _ in range(300):
        world.step()
    assert comm.update(world, force=True)
    link = comm.link(1, 2)
    assert link is not None and link.up and link.pdr > 0.8
    assert link.latency_ms >= comm.params.hop_latency_ms
    # a second measurement only moves the estimate part of the way (EWMA)
    before = link.pdr
    comm.update(world, force=True)
    assert abs(comm.link(1, 2).pdr - before) < 0.3


def test_radio_fault_scales_every_link_of_that_uav():
    world = make_world()
    comm = CommunicationModel.from_world(world, ObstacleField())
    world.goto(1, (100, 0, 50), "test")
    world.goto(2, (160, 0, 50), "test")
    for _ in range(300):
        world.step()
    comm.update(world, force=True)
    healthy = comm.link(1, 2).pdr
    world.set_radio_health(1, 0.2, "test")
    for _ in range(20):
        comm.update(world, force=True)
    assert comm.link(1, 2).pdr < healthy * 0.5
    assert not comm.link(1, 2).up


def test_only_airborne_operational_uavs_are_nodes():
    world = make_world()
    comm = CommunicationModel.from_world(world, ObstacleField())
    comm.update(world, force=True)
    assert set(comm.positions) == {0}          # everyone still on the ground
    world.goto(1, (100, 0, 50), "test")
    for _ in range(300):
        world.step()
    comm.update(world, force=True)
    assert set(comm.positions) == {0, 1}
    world.fail_uav(1, "test")
    comm.update(world, force=True)
    assert set(comm.positions) == {0}
