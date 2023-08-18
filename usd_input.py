from dataclasses import dataclass


@dataclass
class UsdInput:
    """Class for packaging the arguments of create_usd_polygon function."""
    verts: 'typing.Any'
    rings: 'typing.Any'
    full_path: str
    color: 'typing.Any' = (0, 1, 0)
    semantic_label: str = "others"
    height: float = None
