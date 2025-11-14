# greedy-voxel-colliders
Currently all major existing solutions for voxel meshing / colliders in Rust are either integrated into a larger voxel engine, or are used solely for meshing rather than colliders.

This library seeks to fill that gap by providing functionality to generate collision cuboids for a voxel chunk which can easily be passed to a physics API in any game engine. Colliders constructed from a compound of cuboids usually has more "fill" to it allowing more optimized (trimesh construction or convex decomposition is unnecessary) and robust physics (less wall clips).

## Installation
Simply add the library to your Cargo.toml
```toml
greedy_voxel_colliders = { git = "https://github.com/RishiChalla/greedy-voxel-colliders" }
```

## Usage Example

The API is centered around the `VoxelChunk` trait. You just need to implement this trait for your existing chunk structure.

```rust
use greedy_voxel_colliders::{
    generate_collision_cuboids, 
    VoxelChunk, 
    VoxelIndex, 
    CollisionCuboid
};

// --- 1. Define Your Chunk Structure ---
// The crate is generic over the chunk size
const CHUNK_SIZE: usize = 16;

pub struct MyChunk {
    // A simple 3D array for this example. 1 = Solid, 0 = Air
    data: [[[u8; CHUNK_SIZE]; CHUNK_SIZE]; CHUNK_SIZE],
}

// --- 2. Implement the VoxelChunk Trait ---
impl VoxelChunk<CHUNK_SIZE, CHUNK_SIZE, CHUNK_SIZE> for MyChunk {
    /// This function tells the algorithm if a voxel at a given index is solid.
    fn is_pos_solid(&self, pos: VoxelIndex) -> bool {
        self.data[pos.x][pos.y][pos.z] == 1
    }
}

fn main() {
    // --- 3. Create Your Chunk Data ---
    let mut data = [[[0; CHUNK_SIZE]; CHUNK_SIZE]; CHUNK_SIZE];

    // Create a 3x1x1 "|" shape on the ground
    data[0][0][0] = 1; // Stalk of the |
    data[1][0][0] = 1; // Base of the |
    data[2][0][0] = 1; // Base of the |

    let chunk = MyChunk { data };

    // --- 4. Generate the Colliders! ---
    let cuboids: Vec<CollisionCuboid> = generate_collision_cuboids(&chunk);

    // The greedy algorithm will optimally merge this "L" shape
    // into two cuboids: a 1x1x1 and a 2x1x1.
    println!("Found {} optimal cuboids:", cuboids.len());
    for cuboid in cuboids {
        println!("  - Min: ({}, {}, {}), Max: ({}, {}, {})",
            cuboid.min.x, cuboid.min.y, cuboid.min.z,
            cuboid.max.x, cuboid.max.y, cuboid.max.z
        );
    }
    
    // --- 5. Use the Cuboids ---
    // Now you can pass this `Vec<CollisionCuboid>` to your physics
    // engine to create a compound collider.
    //
    // For example, in Bevy/XPBD:
    // let shapes: Vec<(Vec3, Quat, Collider)> = cuboids.iter().map(|cuboid| {
    //     // 1. Calculate cuboid center and half-extents from min/max
    //     // 2. Convert from index-space to world-space
    //     // 3. Return (position, rotation, Collider::cuboid(...))
    // }).collect();
    //
    // commands.spawn((
    //     ...
    //     RigidBody::Static,
    //     Collider::compound(shapes),
    // ));
}
```
