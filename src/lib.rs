use itertools::{Itertools, iproduct};

/// An index for a single voxel within a voxel chunk. A voxel chunk should be able to determine
/// whether a single voxel is solid or not from a VoxelIndex. VoxelIndex is used for both
/// chunk indexing, and the output collision bounding boxes as all coordinates are in index-space.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub struct VoxelIndex {
    pub x: usize, pub y: usize, pub z: usize
}

impl VoxelIndex {
    pub fn new(x: usize, y: usize, z: usize) -> Self { 
        Self { x, y, z }
    }
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
    min: VoxelIndex, max: VoxelIndex,
}

#[derive(Clone, Copy, Debug, PartialEq, Default)]
pub struct VoxelVec3 { pub x: f32, pub y: f32, pub z: f32 }

impl CollisionCuboid {
    /// Converts this index-space cuboid into a `min` corner position and a `full size`.
    /// A single-voxel cuboid at `(5,5,5)` will return:
    /// - `position`: (5.0, 5.0, 5.0)
    /// - `size`: (1.0, 1.0, 1.0)
    pub fn to_position_size_f32(&self) -> (VoxelVec3, VoxelVec3) {
        (
            VoxelVec3 { x: self.min.x as f32, y: self.min.y as f32, z: self.min.z as f32 },
            VoxelVec3 {
                x: (self.max.x - self.min.x + 1) as f32,
                y: (self.max.y - self.min.y + 1) as f32,
                z: (self.max.z - self.min.z + 1) as f32,
            }
        )
    }

    /// Converts this index-space cuboid into a `center` position and `full size`.
    /// A single-voxel cuboid at `(5,5,5)` will return:
    /// - `center`: (5.5, 5.5, 5.5)
    /// - `size`: (1.0, 1.0, 1.0)
    pub fn to_center_size_f32(&self) -> (VoxelVec3, VoxelVec3) {
        // Calculate the full size (width, height, depth)
        let size = VoxelVec3 {
            x: (self.max.x - self.min.x + 1) as f32,
            y: (self.max.y - self.min.y + 1) as f32,
            z: (self.max.z - self.min.z + 1) as f32,
        };

        // Calculate the center position
        let center = VoxelVec3 {
            x: self.min.x as f32 + (size.x / 2.0),
            y: self.min.y as f32 + (size.y / 2.0),
            z: self.min.z as f32 + (size.z / 2.0),
        };

        (center, size)
    }
}

/// This method takes a chunk as the input, and returns a list of bounding box cuboids that can be compounded to
/// generate an accurate and whole physics Collider for an inputted Voxel Chunk. This is more performant than
/// providing Triangles to Physics APIs as triangles both have more data, and need to be decompositioned.
/// When Allow overlap is true, less cuboids will be generated, but they may overlap one-another.
pub fn generate_collision_cuboids<
    const SIZE_X: usize,
    const SIZE_Y: usize,
    const SIZE_Z: usize,
    Chunk: VoxelChunk<SIZE_X, SIZE_Y, SIZE_Z>,
>(chunk: &Chunk, allow_overlap: bool) -> Vec<CollisionCuboid> {
    // Loop through all voxels until everything has been visited
    let mut visited = [[[false; SIZE_Z]; SIZE_Y]; SIZE_X];
    iproduct!(0..SIZE_X, 0..SIZE_Y, 0..SIZE_Z).map(VoxelIndex::from).filter_map(|min| {
        // Skip if visited or isn't solid
        if visited[min.x][min.y][min.z] || !chunk.is_pos_solid(min) { return None; }
        // Expand the bounding box on the each axis as much as possible
        let mut max = min;
        
        // Checks if the bounding cuboid can be expanded at a position
        let can_expand_at = |pos: VoxelIndex| (allow_overlap || !visited[pos.x][pos.y][pos.z])
            && chunk.is_pos_solid(VoxelIndex::new(pos.x, pos.y, pos.z));

        // Expand on X axis
        while max.x + 1 < SIZE_X && can_expand_at(VoxelIndex::new(max.x + 1, max.y, max.z)) {
            max.x += 1;
        }

        // Expand on Y axis, checking all X line is solid when expanding
        while max.y + 1 < SIZE_Y && (min.x..=max.x).all(|x| can_expand_at(VoxelIndex::new(x, max.y + 1, max.z))) {
            max.y += 1;
        }

        // Expand on Z axis, checking all X/Y plane is solid when expanding
        while max.z + 1 < SIZE_Z && (min.x..=max.x).cartesian_product(min.y..=max.y)
            .all(|(x, y)| can_expand_at(VoxelIndex::new(x, y, max.z + 1))) {
            max.z += 1;
        }

        // Mark all items in the cuboid as visited
        for (x, y, z) in iproduct!(min.x..=max.x, min.y..=max.y, min.z..=max.z) {
            visited[x][y][z] = true;
        }

        Some(CollisionCuboid { min, max })
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
        let cuboids = generate_collision_cuboids(&chunk, false);
        assert!(cuboids.is_empty());
    }

    #[test]
    fn test_full_chunk() {
        let chunk = TestChunk::<4, 4, 4> {
            data: [[[1; 4]; 4]; 4],
        };
        let cuboids = generate_collision_cuboids(&chunk, false);
        assert_eq!(cuboids.len(), 1);
        assert_eq!(cuboids[0], cuboid(0, 0, 0, 3, 3, 3));
    }

    #[test]
    fn test_two_separate_voxels() {
        let mut data = [[[0; 4]; 4]; 4];
        data[0][0][0] = 1;
        data[2][0][0] = 1;
        let chunk = TestChunk::<4, 4, 4> { data };
        let cuboids = generate_collision_cuboids(&chunk, false);

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
        let cuboids = generate_collision_cuboids(&chunk, false);

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
        let cuboids = generate_collision_cuboids(&chunk, false);

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
        let cuboids = generate_collision_cuboids(&chunk, false);
        
        // This should be optimally merged into 3 cuboids
        let expected: BTreeSet<CollisionCuboid> = [
            cuboid(0, 0, 0, 1, 1, 0), // Front face (2x2x1)
            cuboid(0, 0, 1, 1, 0, 1), // Back-left (1x2x1)
            cuboid(0, 1, 1, 0, 1, 1), // Back-right-bottom (1x1x1)
        ]
        .into_iter()
        .collect();
        
        let result: BTreeSet<CollisionCuboid> = cuboids.into_iter().collect();
        assert_eq!(result, expected);
    }

    #[test]
    fn test_overlap_vs_non_overlap_logic() {
        // A 3x3x1 chunk for the '+' shape
        let mut data = [[[0; 1]; 3]; 3];
        
        // Create the '+' shape at z=0
        // [0, 1, 0]
        // [1, 1, 1]
        // [0, 1, 0]
        data[1][0][0] = 1;
        data[0][1][0] = 1;
        data[1][1][0] = 1;
        data[2][1][0] = 1;
        data[1][2][0] = 1;

        let chunk = TestChunk::<3, 3, 1> { data };

        // --- Test 1: allow_overlap = true ---
        // Should produce two large, overlapping cuboids.
        let cuboids_overlap = generate_collision_cuboids(&chunk, true);
        let expected_overlap: BTreeSet<CollisionCuboid> = [
            cuboid(0, 1, 0, 2, 1, 0), // Horizontal bar
            cuboid(1, 0, 0, 1, 2, 0), // Vertical bar
        ]
        .into_iter()
        .collect();
        
        let result_overlap: BTreeSet<CollisionCuboid> = cuboids_overlap.into_iter().collect();
        assert_eq!(result_overlap, expected_overlap);


        // --- Test 2: allow_overlap = false ---
        // Should produce 4 non-overlapping cuboids based on the X-Y-Z expansion order.
        let cuboids_no_overlap = generate_collision_cuboids(&chunk, false);
        let expected_no_overlap: BTreeSet<CollisionCuboid> = [
            cuboid(0, 1, 0, 2, 1, 0), // Horizontal bar
            cuboid(1, 0, 0, 1, 0, 0), // Top piece
            cuboid(1, 2, 0, 1, 2, 0)  // Bottom piece
        ]
        .into_iter()
        .collect();

        let result_no_overlap: BTreeSet<CollisionCuboid> = cuboids_no_overlap.into_iter().collect();
        assert_eq!(result_no_overlap, expected_no_overlap);
    }

    #[test]
    fn test_cuboid_size_for_single_voxel() {
        let cuboid = CollisionCuboid {
            min: VoxelIndex::new(5, 5, 5),
            max: VoxelIndex::new(5, 5, 5),
        };

        // Test the original function (min corner + size)
        let (pos, size) = cuboid.to_position_size_f32();
        assert_eq!(pos, VoxelVec3 { x: 5.0, y: 5.0, z: 5.0 });
        assert_eq!(size, VoxelVec3 { x: 1.0, y: 1.0, z: 1.0 });
        
        // Test the new function (center + size)
        let (center, size_b) = cuboid.to_center_size_f32();
        assert_eq!(center, VoxelVec3 { x: 5.5, y: 5.5, z: 5.5 });
        assert_eq!(size_b, VoxelVec3 { x: 1.0, y: 1.0, z: 1.0 });
    }
}
