import os
os.environ["JULIA_PROJECT"] = "."

from julia.api import Julia
# ubuntu/debian/conda static linkage to libpython workaround
# set to True if you have a different python distribution
# otherwise it significantly bloats the initial loading time.
jl = Julia(compiled_modules=False)
from julia import Main

Main.include("src/BlueMesh7.jl"); jl.using(".BlueMesh7")
BlueMesh7 = Main.BlueMesh7

import numpy as np

positions = BlueMesh7.generate_positions()

node_roles = np.ones(len(positions), dtype='i8')
mesh = BlueMesh7.initialize_mesh(positions, node_roles)

println("Running a simulation for 7 minutes (420000 steps, 1 step = 1ms)...")
stats = start(mesh, minutes=7)

print(f"Packet delivery rate: {stats[0]:.2f}, Worst delivery rate: {stats[1]:.2f}, Average delay: {delay:.2f}ms")
