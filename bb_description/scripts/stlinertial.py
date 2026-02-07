import numpy as np
from stl import mesh
from pathlib import Path
path=Path.home() / 'robot_ws' / 'src' / 'bb_description' / 'meshes' / 'base_link.STL'
# Load the STL file
your_mesh = mesh.Mesh.from_file(path)

# Get properties: Volume, COG, and Inertia Matrix (at COG)
volume, cog, inertia = your_mesh.get_mass_properties()

# If you know the density of the material (e.g., 7850 kg/m^3 for steel)
# density = 7850
# mass = volume * density

print("Volume = {0}".format(volume))
print("Center of Gravity = {0}".format(cog))
print("Inertia Matrix = \n{0}".format(inertia))
