from pathlib import Path

MESH_DIR = "meshes"

def get_meshdir():
    return str(Path(__file__).parent.resolve() / MESH_DIR)