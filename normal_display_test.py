#!/usr/bin/env python3
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({
    "headless": True,
    "renderer": "RayTracedLighting",
})

from create_usd_polygon import create_usd_polygon, make_initial_stage
from pxr import Usd, UsdGeom, UsdShade, Gf, Sdf
import sys
import argparse
from common.proto import maphub_pb2
from constants import *
from usd_input import UsdInput

verts = [(1,0), (1,1), (0,1), (0,0)]
usd_input = UsdInput(verts=verts,
                    rings=[len(verts)],
                    file_name="usd_output/display_cube_test.usda",
                    color=GEOMETRY_COLOR,
                    semantic_label=OTHERS,
                    height=1)

stage = create_usd_polygon(usd_input)
simulation_app.close()
