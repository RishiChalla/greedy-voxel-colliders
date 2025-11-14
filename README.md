# greedy-voxel-colliders
Currently all major existing solutions for voxel meshing / colliders in Rust are either integrated into a larger voxel engine, or are used solely for meshing rather than colliders.

This library seeks to fill that gap by providing functionality to generate collision cuboids for a voxel chunk which can easily be passed to a physics API in any game engine.
