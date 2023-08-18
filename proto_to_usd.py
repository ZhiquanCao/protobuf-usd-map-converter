#!/usr/bin/env python3
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({
    "headless": True,
    "renderer": "RayTracedLighting",
}, experience="/isaac-sim/apps/omni.isaac.sim.python.minimal.kit")

from create_usd_polygon import create_usd_polygon, make_initial_stage
from pxr import Usd, UsdGeom, UsdShade, Gf, Sdf
import sys
import argparse
from common.proto import maphub_pb2
from constants import *
from usd_input import UsdInput


def combine_usd(*args):
    merged_stage = make_initial_stage(f"{OUTPUT_DIR}{MAP}.{USD_FORMAT}")
    # merged_stage = Usd.Stage.DefineMerged([stage1, stage2])

    for arg in args:
        merged_stage.GetRootLayer().subLayerPaths.append(arg.GetRootLayer().identifier)
    merged_stage.GetRootLayer().Flatten()
    merged_stage.GetRootLayer().Export('merged_stage.usda')
    return merged_stage.GetRootLayer().ExportToString()


def object_to_usd(object):
    pass


def geometry_to_usd(geometries):
    rootLayer = make_initial_stage(f"{OUTPUT_DIR}{GEOMETRY}.{USD_FORMAT}")
    for i in range(len(geometries)):
        verts = [(point.x, point.y) for point in geometries[i].polygon.point]
        usd_input = UsdInput(verts=verts[0:-1],
                              rings=[len(verts)-1],
                              full_path=f"{OUTPUT_DIR}{GEOMETRY}/{GEOMETRY}_{i}.{USD_FORMAT}",
                              color=GEOMETRY_COLOR,
                              semantic_label=OTHERS,
                              height=GEOMETRY_HEIGHT)
        stage = create_usd_polygon(usd_input)
        rootLayer.GetRootLayer().subLayerPaths.append(stage.GetRootLayer().identifier)
    # rootLayer.GetRootLayer().Save()
    return rootLayer


def edge_info_to_usd(edge_info):
    rootLayer = make_initial_stage(f"{OUTPUT_DIR}{EDGE_INFO}.{USD_FORMAT}")
    for i in range(len(edge_info)):
        verts = []
        rings = []
        outer = [(semantic_point.point.x, semantic_point.point.y) for semantic_point in edge_info[i].outer.semantic_point]
        verts += outer
        rings.append(len(outer))
        if edge_info[i].inners:
            for inner in edge_info[i].inners:
                length = len(inner.semantic_point)
                rings.append(rings[-1]+length)
                for semantic_point in inner.semantic_point:
                    verts.append((semantic_point.point.x, semantic_point.point.y))
        usd_input = UsdInput(verts=verts,
                            rings=rings,
                            full_path=f"{OUTPUT_DIR}{EDGE_INFO}/{EDGE_INFO}_{i}.{USD_FORMAT}",
                            color=EDGE_INFO_COLOR,
                            semantic_label=GRASS)
        stage = create_usd_polygon(usd_input)
        rootLayer.GetRootLayer().subLayerPaths.append(stage.GetRootLayer().identifier)
    # rootLayer.GetRootLayer().Save()
    return rootLayer


def channels_to_usd(channels):
    rootLayer = make_initial_stage(f"{OUTPUT_DIR}{CHANNELS}.{USD_FORMAT}")
    for i in range(len(channels)):
        verts = [(point.x, point.y) for point in channels[i].outer.point]
        usd_input = UsdInput(verts=verts,
                            rings=[len(verts)],
                            full_path=f"{OUTPUT_DIR}{CHANNELS}/{CHANNELS}_{i}.{USD_FORMAT}",
                            color=CHANNELS_COLOR,
                            semantic_label=ROAD)
        stage = create_usd_polygon(usd_input)
        rootLayer.GetRootLayer().subLayerPaths.append(stage.GetRootLayer().identifier)
    # rootLayer.GetRootLayer().Save()
    return rootLayer


# Iterates though all objects in maphub, convert each of them into usd and combine the usd
def map_to_usd(map):
    geometries = geometry_to_usd(map.geometry)
    edge_info = edge_info_to_usd(map.edge_info)
    channels = channels_to_usd(map.channels)
    print(combine_usd(geometries, edge_info, channels))


parser = argparse.ArgumentParser(description='Process some integers.')
parser.add_argument('infile', type=argparse.FileType('rb'))
parser.add_argument('--usd-format', dest='usd_format', choices=['usd', 'usda'],
                    default='usd', help='either usd or usda')
parser.add_argument('--height', dest='height', type=float, default=0.5,
                    help='the height of geometry (default: 0.5)')
parser.add_argument('--out', dest='output_dir', default='usd_output/',
                    help='the directory to store usd output (default: "usd_output/")')

map = maphub_pb2.HDMap()
args = parser.parse_args()
map.ParseFromString(args.infile.read())
with open('out.txt', 'w') as f:
    print(map, file=f)
USD_FORMAT = args.usd_format
GEOMETRY_HEIGHT = args.height
OUTPUT_DIR = args.output_dir

map_to_usd(map)

simulation_app.close()
