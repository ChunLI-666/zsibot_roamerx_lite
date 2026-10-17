"""Geometry-label regressions; production collision logic is tested by the harness."""
import math
import numpy as np
from geometry_cases import minimum_width, cell_intersects_polygon


def test_minimum_width_is_rotation_invariant_and_not_aabb_width():
    rectangle = np.array([[-.3,-.15],[.3,-.15],[.3,.15],[-.3,.15]])
    angle = .71
    rotation = np.array([[math.cos(angle),-math.sin(angle)],[math.sin(angle),math.cos(angle)]])
    assert abs(minimum_width(rectangle @ rotation.T)-.3) < 1e-12


def test_cell_inside_aabb_but_outside_diamond_is_free():
    diamond = np.array([[-.3,0],[0,-.3],[.3,0],[0,.3]])
    assert not cell_intersects_polygon(diamond, np.array([.26,.26]), .02)
    assert cell_intersects_polygon(diamond, np.array([.1,.1]), .02)


def test_cell_contact_is_collision_on_both_sides():
    rectangle = np.array([[-.25,-.25],[.25,-.25],[.25,.25],[-.25,.25]])
    assert cell_intersects_polygon(rectangle, np.array([-.275,0]), .05)
    assert cell_intersects_polygon(rectangle, np.array([.275,0]), .05)
    assert not cell_intersects_polygon(rectangle, np.array([.276,0]), .05)
