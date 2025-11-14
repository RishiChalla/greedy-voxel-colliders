use itertools::{Itertools, iproduct};

/// Represents a dimension in the Chunk 3d space
#[derive(Clone, Copy)]
enum Dimension { X, Y, Z }

/// An index for a single voxel within a voxel chunk. A voxel chunk should be able to determine
/// whether a single voxel is solid or not from a VoxelIndex. VoxelIndex is used for both
/// chunk indexing, and the output collision bounding boxes as all coordinates are in index-space.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub struct VoxelIndex {
    pub x: usize, pub y: usize, pub z: usize
}

impl From<(usize, usize, usize)> for VoxelIndex {
    fn from(index: (usize, usize, usize)) -> Self {
        Self { x: index.0, y: index.1, z: index.2 }
    }
}

/// This trait needs to be implemented on your Chunk data structure to be inputted into the
/// greedy collision cuboid algorithm. This means your Chunk data structure must have a size
/// known at compile time (even if its different on all dimensions), and should be able to determine
/// if it is solid at a certain 3d index.
pub trait VoxelChunk<const SIZE_X: usize, const SIZE_Y: usize, const SIZE_Z: usize> {
    /// Should check whether a certain position / index in the chunk is solid. Should return true
    /// if the position is solid, and there is a filled voxel present at the index.
    fn is_pos_solid(&self, pos: VoxelIndex) -> bool;
}

/// A single collision cuboid bounding box in Index-space. For passing this to a Physics API, you may need to
/// transform this into the relevant coordinate system, most Physics APIs should allow for translations/transformations
/// without modifying the actual cuboids individually.
#[derive(Debug, PartialEq, Eq)]
pub struct CollisionCuboid {
    pub min: VoxelIndex,
    pub max: VoxelIndex,
}

impl CollisionCuboid {
    fn get_incremented_in_dim(&self, dim: Dimension) -> Vec<VoxelIndex> {
        match dim {
            Dimension::X => iproduct!(self.min.y..self.max.y, self.min.z..self.max.z)
                .map(|(y, z)| VoxelIndex::from((self.max.x + 1, y, z))).collect(),
            Dimension::Y => iproduct!(self.min.x..self.max.x, self.min.z..self.max.z)
                .map(|(x, z)| VoxelIndex::from((x, self.max.y + 1, z))).collect(),
            Dimension::Z => iproduct!(self.min.x..self.max.x, self.min.y..self.max.y)
                .map(|(x, y)| VoxelIndex::from((x, y, self.max.z + 1))).collect(),
        }
    }

    fn increment_in_dim(&mut self, dim: Dimension) {
        match dim {
            Dimension::X => self.max.x += 1,
            Dimension::Y => self.max.y += 1,
            Dimension::Z => self.max.z += 1,
        }
    }
}

/// This method takes a chunk as the input, and returns a list of bounding box cuboids that can be compounded to
/// generate an accurate and whole physics Collider for an inputted Voxel Chunk. This is more performant than
/// providing Triangles to Physics APIs as triangles both have more data, and need to be decompositioned.
pub fn generate_collision_cuboids<
    const SIZE_X: usize,
    const SIZE_Y: usize,
    const SIZE_Z: usize,
    Chunk: VoxelChunk<SIZE_X, SIZE_Y, SIZE_Z>,
>(chunk: &Chunk) -> Vec<CollisionCuboid> {
    // Loop through all voxels until everything has been visited
    let mut visited = [[[false; SIZE_Z]; SIZE_Y]; SIZE_X];
    iproduct!(0..SIZE_X, 0..SIZE_Y, 0..SIZE_Z).map(VoxelIndex::from).filter_map(|min| {
        // Skip if visited
        let is_visited = &mut visited[min.x][min.y][min.z];
        if *is_visited { return None; }
        *is_visited = true;
        // Skip if not solid
        if !chunk.is_pos_solid(min) { return None; }
        // Expand the bounding box on the each axis as much as possible
        let mut cuboid = CollisionCuboid { min, max: min };
        for dim in [Dimension::X, Dimension::Y, Dimension::Z] {
            loop {
                // TODO - two issues.
                // 1. We don't check against SIZE_<dim> ourselves and instead still call is_pos_solid,
                //    potentially resulting in a lookup out of bounds from the user implementation.
                // 2. get_incremented_in_dim creates a vec which is inefficient. We can optimize this by
                //    looping directly here, and avoiding the iproduct! call
                let updated = cuboid.get_incremented_in_dim(dim);
                if !updated.iter().all(|pos| chunk.is_pos_solid(*pos)) { break; }
                for pos in updated { visited[pos.x][pos.y][pos.z] = true; }
                cuboid.increment_in_dim(dim);
            }
        }
        Some(cuboid)
    }).collect_vec()
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::BTreeSet;

    // A simple chunk implementation for testing.
    // `1` is solid, `0` is air.
    struct TestChunk<const SIZE_X: usize, const SIZE_Y: usize, const SIZE_Z: usize> {
        data: [[[u8; SIZE_Z]; SIZE_Y]; SIZE_X],
    }

    impl<const SIZE_X: usize, const SIZE_Y: usize, const SIZE_Z: usize>
        VoxelChunk<SIZE_X, SIZE_Y, SIZE_Z> for TestChunk<SIZE_X, SIZE_Y, SIZE_Z>
    {
        fn is_pos_solid(&self, pos: VoxelIndex) -> bool {
            let VoxelIndex { x, y, z } = pos;
            // Check bounds just in case, though the algorithm shouldn't go OOB
            if x >= SIZE_X || y >= SIZE_Y || z >= SIZE_Z {
                return false;
            }
            self.data[x][y][z] == 1
        }
    }

    // Helper to make cuboids easier to write in tests
    fn cuboid(x1: usize, y1: usize, z1: usize, x2: usize, y2: usize, z2: usize) -> CollisionCuboid {
        CollisionCuboid {
            min: (x1, y1, z1).into(),
            max: (x2, y2, z2).into(),
        }
    }

    // Helper to compare cuboid vecs as sets
    impl Ord for CollisionCuboid {
        fn cmp(&self, other: &Self) -> std::cmp::Ordering {
            (self.min.x, self.min.y, self.min.z).cmp(&(other.min.x, other.min.y, other.min.z))
        }
    }

    impl PartialOrd for CollisionCuboid {
        fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
            Some(self.cmp(other))
        }
    }

    #[test]
    fn test_empty_chunk() {
        let chunk = TestChunk::<4, 4, 4> {
            data: [[[0; 4]; 4]; 4],
        };
        let cuboids = generate_collision_cuboids(&chunk);
        assert!(cuboids.is_empty());
    }

    #[test]
    fn test_full_chunk() {
        let chunk = TestChunk::<4, 4, 4> {
            data: [[[1; 4]; 4]; 4],
        };
        let cuboids = generate_collision_cuboids(&chunk);
        assert_eq!(cuboids.len(), 1);
        assert_eq!(cuboids[0], cuboid(0, 0, 0, 3, 3, 3));
    }

    #[test]
    fn test_two_separate_voxels() {
        let mut data = [[[0; 4]; 4]; 4];
        data[0][0][0] = 1;
        data[2][0][0] = 1;
        let chunk = TestChunk::<4, 4, 4> { data };
        let cuboids = generate_collision_cuboids(&chunk);

        let expected: BTreeSet<CollisionCuboid> = [
            cuboid(0, 0, 0, 0, 0, 0),
            cuboid(2, 0, 0, 2, 0, 0),
        ]
        .into_iter()
        .collect();
        
        let result: BTreeSet<CollisionCuboid> = cuboids.into_iter().collect();
        assert_eq!(result, expected);
    }
    
    #[test]
    fn test_2x2x1_square() {
        let mut data = [[[0; 4]; 4]; 4];
        data[0][0][0] = 1;
        data[1][0][0] = 1;
        data[0][1][0] = 1;
        data[1][1][0] = 1;
        let chunk = TestChunk::<4, 4, 4> { data };
        let cuboids = generate_collision_cuboids(&chunk);

        // The flawed algorithm would produce 4 cuboids here.
        // The correct one produces 1.
        assert_eq!(cuboids.len(), 1);
        assert_eq!(cuboids[0], cuboid(0, 0, 0, 1, 1, 0));
    }

    #[test]
    fn test_l_shape() {
        let mut data = [[[0; 4]; 4]; 4];
        data[0][0][0] = 1;
        data[1][0][0] = 1; // 2-wide base
        data[0][1][0] = 1; // 1-wide stalk
        let chunk = TestChunk::<4, 4, 4> { data };
        let cuboids = generate_collision_cuboids(&chunk);

        let expected: BTreeSet<CollisionCuboid> = [
            // The algorithm will find the 2x1 base first
            cuboid(0, 0, 0, 1, 0, 0),
            // Then it will find the 1x1 stalk
            cuboid(0, 1, 0, 0, 1, 0),
        ]
        .into_iter()
        .collect();
        
        let result: BTreeSet<CollisionCuboid> = cuboids.into_iter().collect();
        assert_eq!(result, expected);
    }
    
    #[test]
    fn test_3d_shape_with_missing_corner() {
        let mut data = [[[0; 2]; 2]; 2];
        // A 2x2x2 cube
        for (x,y,z) in iproduct!(0..2, 0..2, 0..2) {
            data[x][y][z] = 1;
        }
        // Remove one corner
        data[1][1][1] = 0;
        
        let chunk = TestChunk::<2, 2, 2> { data };
        let cuboids = generate_collision_cuboids(&chunk);
        
        // This should be optimally merged into 3 cuboids
        let expected: BTreeSet<CollisionCuboid> = [
            cuboid(0, 0, 0, 1, 1, 0), // Front face (2x2x1)
            cuboid(0, 0, 1, 0, 1, 1), // Back-left (1x2x1)
            cuboid(1, 0, 1, 1, 0, 1), // Back-right-bottom (1x1x1)
        ]
        .into_iter()
        .collect();
        
        let result: BTreeSet<CollisionCuboid> = cuboids.into_iter().collect();
        assert_eq!(result, expected);
    }
}
