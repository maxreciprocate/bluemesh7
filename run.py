import os
os.environ["JULIA_PROJECT"] = "."

from julia.api import Julia
# ubuntu/debian/conda static linkage to libpython workaround
# set to True if you have a different python distribution
# otherwise it significantly bloats the initial loading time.
jl = Julia(compiled_modules=False)
from julia import Main

Main.include("BlueMesh7.jl"); jl.using(".BlueMesh7")
BlueMesh7 = Main.BlueMesh7

import numpy as np

adjacency_matrix, positions = BlueMesh7.random_graph()

node_roles = np.ones(len(positions), dtype='i8')
model = BlueMesh7.initialize_model(node_roles, positions)

print("running the simulation for 12 minutes (1 step = 1ms)...")
produced, received = BlueMesh7.start_model(model, 12 * 60_000)
print(f"packets produced: {produced}, received: {received}, {round(100 * received / produced, 2)}%")
