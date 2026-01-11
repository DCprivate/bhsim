//! # Barnes–Hut Octree (3D)
//!
//! This module implements a **3D Barnes–Hut octree** for approximating
//! gravitational acceleration in an `N`-body system. The goal is to replace the
//! naive `O(N²)` all-pairs force calculation with an approximate `O(N log N)`
//! method while preserving good accuracy for distant interactions.
//!
//! ## Core Concepts
//!
//! The key idea of Barnes–Hut is to treat a group of distant bodies as a
//! single pseudo-body located at their center of mass. For sufficiently
//! far clusters, evaluating one interaction is drastically cheaper than
//! computing many individual forces.
//!
//! - The simulation space is recursively subdivided into 8 regions (octants).
//! - Each region becomes a node of the octree.
//! - Leaf nodes may hold one body, or they are subdivided further.
//! - Each node stores:
//!   - total mass of its subtree
//!   - center of mass (COM)
//!   - bounding box (for computing size and subdivision)

/// =======================================================================================================
/// References:
/// C++ implementation that I referenced https://github.com/MarcVivas/N-body/blob/main/src/Octree/Octree.cpp
/// another online reference I used https://arborjs.org/docs/barnes-hut
/// 
/// I used AI to do heavy documentation here
/// 
/// I am keeping it heavily-documented so I can remember what the heck it does
/// 
/// =======================================================================================================

use crate::simulation::states::{System3, NVec3};

/// A single octree node for the 3D Barnes–Hut algorithm.
///
/// Each node represents a cubic region of space that may contain:
/// - zero bodies (empty)
/// - exactly one body (leaf node, `body_index = Some(i)`)
/// - multiple bodies (internal node with children)
///
/// The node stores both geometric bounds and aggregate physical data
/// (total mass and center-of-mass) used to approximate distant regions.
pub struct BarnesHutNode3 {
    pub mass: f64,
    pub com: NVec3,
    pub bbox_min: NVec3,
    pub bbox_max: NVec3,
    pub children: [Option<usize>; 8],  // indices into BarnesHutTree3::nodes
    pub body_index: Option<usize>,     // Some(i) if this leaf holds body i
}

/// A complete 3D Barnes–Hut octree built over an N-body system.
///
/// This structure owns:
/// - a vector of all octree nodes (`nodes`)
/// - an index into that list representing the root (`root`)
pub struct BarnesHutTree3 {
    pub nodes: Vec<BarnesHutNode3>,
    pub root: usize,
}

impl BarnesHutTree3 {
    /// Build a 3D Barnes–Hut octree from the current state of the system.
    ///
    /// This:
    /// 1. Computes a global cubic bounding box that encloses all bodies.
    /// 2. Creates a root node covering that bounding volume.
    /// 3. Inserts each body into the tree, subdividing nodes as needed.
    /// 4. Computes total mass and center-of-mass for every node (bottom-up).
    ///
    /// The resulting tree can be used to approximate gravitational forces via
    /// [`BarnesHutTree3::force_on_body`].
    ///
    /// # Parameters
    /// - `sys`: The current 3D N-body system (`System3`) whose bodies will be
    ///   inserted into the octree.
    ///
    /// # Returns
    /// A fully constructed [`BarnesHutTree3`] with:
    /// - a root node covering all bodies,
    /// - all bodies inserted into appropriate leaf nodes,
    /// - each node's `mass` and `com` fields populated.
    pub fn build(sys: &System3) -> Self {
        let (bbox_min, bbox_max) = compute_global_bbox(sys);

        let mut nodes = Vec::new();
        let root = 0;

        nodes.push(BarnesHutNode3 {
            mass: 0.0,
            com: NVec3::zeros(),
            bbox_min,
            bbox_max,
            children: [None; 8],
            body_index: None,
        });

        let mut tree = BarnesHutTree3 { nodes, root };

        // Insert each body into the octree
        for (i, _b) in sys.bodies.iter().enumerate() {
            tree.insert_body(root, i, sys);
        }

        // Compute mass + center of mass bottom-up
        tree.compute_mass_and_com(sys, root);

        tree
    }

    /// Compute the net gravitational acceleration on a single body using the tree.
    ///
    /// This evaluates the Barnes–Hut approximation for body `i`:
    ///
    /// - Traverses the octree starting from the root.
    /// - For sufficiently distant nodes (according to `theta`), uses the node's
    ///   total mass and center-of-mass as a single “pseudo-body”.
    /// - For nearby nodes, recursively descends to finer levels (or exact
    ///   pairwise interactions in leaf nodes).
    ///
    /// The result is an acceleration vector (force per unit mass) at the
    /// position of body `i`.
    ///
    /// # Parameters
    /// - `i`     : Index of the body in `sys.bodies` for which to compute acceleration.
    /// - `sys`   : Current N-body system providing body positions and masses.
    /// - `G`     : Gravitational constant used in the force law.
    /// - `eps2`  : Softening parameter (added to distance²) to avoid singularities.
    /// - `theta` : Opening-angle threshold controlling the accuracy / speed tradeoff.
    ///
    /// # Returns
    /// A 3D vector `NVec3` giving the net acceleration on body `i` due to all
    /// other bodies, approximated via the Barnes–Hut tree.
    pub fn force_on_body(&self, i: usize, sys: &System3, G: f64, eps2: f64, theta: f64) -> NVec3 {
        let pos_i = sys.bodies[i].x;
        let mut acc = NVec3::zeros();
        self.traverse_node(self.root, i, pos_i, sys, G, eps2, theta, &mut acc);
        acc
    }

    // helpers ==============================================================================

    /// Insert a single body into the octree, starting from the given node.
    ///
    /// This function walks (and possibly modifies) the tree so that the body
    /// with index `body_idx` ends up in the correct leaf node:
    ///
    /// - If the target node is empty and has no children, it becomes a leaf
    ///   that stores this body.
    /// - If the target node is a leaf that already contains another body, the
    ///   node is subdivided into 8 children, and both the existing body and the
    ///   new body are reinserted into the appropriate child nodes.
    /// - If the target node already has children, the body is forwarded down
    ///   into the appropriate child based on its position.
    ///
    /// This insertion logic ensures:
    /// - At most one body is stored directly in any leaf node.
    /// - Internal nodes either have children or are empty.
    ///
    /// # Parameters
    /// - `node_idx`: Index of the node in `self.nodes` where insertion starts.
    /// - `body_idx`: Index of the body in `sys.bodies` to be inserted.
    /// - `sys`     : The system providing body positions used for spatial tests.
    fn insert_body(&mut self, node_idx: usize, body_idx: usize, sys: &System3) {
        // Copy the bbox out by value (NVec3 is Copy)
        let bbox_min = self.nodes[node_idx].bbox_min;
        let bbox_max = self.nodes[node_idx].bbox_max;
        let pos = sys.bodies[body_idx].x;

        // Snapshot current state (by value) so we don't hold &mut while recursing
        let body_index = self.nodes[node_idx].body_index;
        let children_all_none = self.nodes[node_idx]
            .children
            .iter()
            .all(|c| c.is_none());

        // Case 1: leaf and empty -> just store body here
        if body_index.is_none() && children_all_none {
            self.nodes[node_idx].body_index = Some(body_idx);
            return;
        }

        // Case 2: leaf with an existing body -> subdivide and reinsert existing body
        if body_index.is_some() && children_all_none {
            let existing_body_idx = body_index.unwrap();
            // clear the body from this node
            self.nodes[node_idx].body_index = None;

            // create children
            self.subdivide(node_idx, bbox_min, bbox_max);

            // reinsert existing body (no &mut node live here)
            self.insert_body(node_idx, existing_body_idx, sys);
        }

        // Case 3: node has (or now has) children -> descend into the right child
        let child_idx = child_index_for_point(&pos, &bbox_min, &bbox_max);
        let child_node_idx = match self.nodes[node_idx].children[child_idx] {
            Some(idx) => idx,
            None => {
                // create child node
                let (cmin, cmax) = child_bbox(&bbox_min, &bbox_max, child_idx);
                let new_idx = self.nodes.len();
                self.nodes.push(BarnesHutNode3 {
                    mass: 0.0,
                    com: NVec3::zeros(),
                    bbox_min: cmin,
                    bbox_max: cmax,
                    children: [None; 8],
                    body_index: None,
                });
                // now we can safely update the parent
                self.nodes[node_idx].children[child_idx] = Some(new_idx);
                new_idx
            }
        };

        // recurse into the child
        self.insert_body(child_node_idx, body_idx, sys);
    }

    /// Subdivide a node into 8 child octants covering its bounding box.
    ///
    /// This operation:
    /// - Splits the node's current axis-aligned bounding box into 8 equally sized
    ///   sub-boxes (octants).
    /// - Allocates a new `BarnesHutNode3` for each child region with:
    ///   - zero mass,
    ///   - zero center-of-mass,
    ///   - appropriate `bbox_min` / `bbox_max`,
    ///   - no children,
    ///   - no body assigned yet.
    /// - Stores the indices of these new nodes in `children[0..8]` of the
    ///   parent node.
    ///
    /// After subdivision, the parent node becomes an internal node: it no longer
    /// holds a single body directly but instead delegates storage to its children.
    ///
    /// # Parameters
    /// - `node_idx`: Index of the node in `self.nodes` to subdivide.
    /// - `bbox_min`: The minimum corner of the node's bounding box.
    /// - `bbox_max`: The maximum corner of the node's bounding box.
    fn subdivide(&mut self, node_idx: usize, bbox_min: NVec3, bbox_max: NVec3) {
        for child_idx in 0..8 {
            let (cmin, cmax) = child_bbox(&bbox_min, &bbox_max, child_idx);
            let new_node_idx = self.nodes.len();
            self.nodes.push(BarnesHutNode3 {
                mass: 0.0,
                com: NVec3::zeros(),
                bbox_min: cmin,
                bbox_max: cmax,
                children: [None; 8],
                body_index: None,
            });
            self.nodes[node_idx].children[child_idx] = Some(new_node_idx);
        }
    }

    /// Recursively compute total mass and center-of-mass for a subtree.
    ///
    /// This function performs a bottom-up pass over the octree:
    ///
    /// 1. Starts from `node_idx`.
    /// 2. Recursively visits all children.
    /// 3. Aggregates:
    ///    - the mass of any leaf body stored directly in this node, plus
    ///    - the mass and COM of all non-empty child nodes.
    /// 4. Writes the resulting total `mass` and `com` back into the node.
    ///
    /// The result is that every node's:
    /// - `mass` field stores the sum of all body masses in its subtree.
    /// - `com` field stores the mass-weighted center-of-mass for that subtree.
    ///
    /// These values are later used by the Barnes–Hut traversal to approximate
    /// distant groups of bodies as a single mass at their COM.
    ///
    /// # Parameters
    /// - `sys`      : The N-body system providing positions and masses.
    /// - `node_idx` : Index of the node in `self.nodes` to process.
    fn compute_mass_and_com(&mut self, sys: &System3, node_idx: usize) {
        let mut mass = 0.0;
        let mut com = NVec3::zeros();

        // Snapshot the info we need from this node by value
        let body_index = self.nodes[node_idx].body_index;
        let children = self.nodes[node_idx].children; // [Option<usize>; 8] is Copy

        // Leaf body contribution
        if let Some(bidx) = body_index {
            let b = &sys.bodies[bidx];
            mass += b.m;
            com += b.x * b.m;
        }

        // Children contributions
        for child in children.iter().flatten() {
            let child_idx = *child;
            // recurse first
            self.compute_mass_and_com(sys, child_idx);
            // then read child node's mass/com
            let cn = &self.nodes[child_idx];
            if cn.mass > 0.0 {
                mass += cn.mass;
                com += cn.com * cn.mass;
            }
        }

        if mass > 0.0 {
            com /= mass;
        }

        // finally write back to this node
        let node = &mut self.nodes[node_idx];
        node.mass = mass;
        node.com = com;
    }

    /// Recursively traverse a subtree to accumulate Barnes–Hut acceleration.
    ///
    /// This function decides, for each node:
    ///
    /// - **If the node is empty** (`mass == 0.0`):  
    ///   Return immediately (no contribution).
    ///
    /// - **If the node is a leaf with a single body**:  
    ///   Compute the exact pairwise interaction between that body and `body_idx`,
    ///   applying softening `eps2` and skipping self-interaction.
    ///
    /// - **If the node is internal (has children)**:  
    ///   Compute its spatial size `s` and distance `d` to the target body.
    ///   - If `s / d < theta`, approximate the entire node as a single mass
    ///     located at its center-of-mass and add that contribution to `acc`.
    ///   - Otherwise, recursively visit all non-empty child nodes.
    ///
    /// The accumulated result is the net gravitational acceleration on the body
    /// at `pos_i` due to all bodies in this subtree (exact or approximated).
    ///
    /// # Parameters
    /// - `node_idx`: Index of the current node in `self.nodes`.
    /// - `body_idx`: Index of the target body in `sys.bodies` (for self-skip).
    /// - `pos_i`   : Position of the target body.
    /// - `sys`     : N-body system providing body positions and masses.
    /// - `G`       : Gravitational constant.
    /// - `eps2`    : Softening added to distance squared.
    /// - `theta`   : Opening-angle threshold controlling approximation.
    /// - `acc`     : Mutable accumulator for the resulting acceleration.
    fn traverse_node(&self, node_idx: usize, body_idx: usize, pos_i: NVec3, sys: &System3, G: f64, eps2: f64, theta: f64, acc: &mut NVec3) {
        let node = &self.nodes[node_idx];

        // Skip empty nodes
        if node.mass == 0.0 {
            return;
        }

        // If leaf with a single body, do direct interaction
        if let Some(bidx) = node.body_index {
            if bidx == body_idx {
                return; // don't self-interact
            }

            let b = &sys.bodies[bidx];
            let r = b.x - pos_i;
            let dist2 = r.dot(&r) + eps2;
            let inv_r = dist2.sqrt().recip();
            let inv_r3 = inv_r * inv_r * inv_r;

            *acc += G * b.m * inv_r3 * r;
            return;
        }

        // Internal node: decide whether to approximate or descend
        let size_vec = node.bbox_max - node.bbox_min;
        let size = size_vec.x.max(size_vec.y).max(size_vec.z);

        let r = node.com - pos_i;
        let dist = r.norm();
        if dist == 0.0 {
            return;
        }

        let s_over_d = size / dist;

        if s_over_d < theta {
            // Far enough away: approximate this node as a single mass at COM
            let dist2 = r.dot(&r) + eps2;
            let inv_r = dist2.sqrt().recip();
            let inv_r3 = inv_r * inv_r * inv_r;

            *acc += G * node.mass * inv_r3 * r;
        } else {
            // Too close: recurse into children
            for &child in &node.children {
                if let Some(child_idx) = child {
                    self.traverse_node(child_idx, body_idx, pos_i, sys, G, eps2, theta, acc);
                }
            }
        }
    }
}

// helpers ===========================================================================

/// Compute a global cubic bounding box that encloses all bodies in the system.
///
/// This function:
/// - Scans all body positions in `sys.bodies`.
/// - Finds the axis-aligned min and max along each coordinate.
/// - Expands the resulting rectangular box into a **cube** by:
///   - computing the center,
///   - taking the largest half-extent over x/y/z,
///   - constructing a cube with equal half-extent in all directions.
///
/// Using a cube (instead of a general rectangular box) simplifies the notion
/// of node “size” used in the Barnes–Hut opening criterion.
///
/// # Parameters
/// - `sys`: The 3D N-body system containing all bodies.
///
/// # Returns
/// A pair `(bbox_min, bbox_max)`:
/// - `bbox_min`: minimum corner of the cubic bounding box.
/// - `bbox_max`: maximum corner of the cubic bounding box.
fn compute_global_bbox(sys: &System3) -> (NVec3, NVec3) {
    let mut min = NVec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
    let mut max = NVec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);

    for b in &sys.bodies {
        min.x = min.x.min(b.x.x);
        min.y = min.y.min(b.x.y);
        min.z = min.z.min(b.x.z);

        max.x = max.x.max(b.x.x);
        max.y = max.y.max(b.x.y);
        max.z = max.z.max(b.x.z);
    }

    // Expand to a cube so size is well-defined
    let center = (min + max) * 0.5;
    let mut half = (max - min) * 0.5;
    let max_half = half.x.max(half.y).max(half.z);
    half = NVec3::new(max_half, max_half, max_half);

    let bbox_min = center - half;
    let bbox_max = center + half;
    (bbox_min, bbox_max)
}

/// Compute the octant index for a point within a node's bounding box.
///
/// This function determines which of the 8 child octants a point `p`
/// belongs to, relative to the axis-aligned bounding box defined by
/// `bbox_min` and `bbox_max`.
///
/// The index is encoded using 3 bits:
///
/// - Bit 0 (value 1): X axis — 0 for left (x < center.x), 1 for right (x >= center.x)
/// - Bit 1 (value 2): Y axis — 0 for bottom (y < center.y), 1 for top (y >= center.y)
/// - Bit 2 (value 4): Z axis — 0 for back (z < center.z), 1 for front (z >= center.z)
///
/// This encoding matches the layout of `children[0..8]` in the octree nodes.
///
/// # Parameters
/// - `p`        : The point to classify.
/// - `bbox_min` : Minimum corner of the parent node's bounding box.
/// - `bbox_max` : Maximum corner of the parent node's bounding box.
///
/// # Returns
/// An integer in `0..8` giving the child index for this point.
fn child_index_for_point(p: &NVec3, bbox_min: &NVec3, bbox_max: &NVec3) -> usize {
    let center = (bbox_min + bbox_max) * 0.5;
    let mut idx = 0;

    if p.x >= center.x { idx |= 1; } // bit 0
    if p.y >= center.y { idx |= 2; } // bit 1
    if p.z >= center.z { idx |= 4; } // bit 2

    idx
}

/// Compute the axis-aligned bounding box for a given child octant.
///
/// Given a parent node's bounding box `[parent_min, parent_max]` and a
/// child index `child_idx` in `0..8`, this function returns the smaller
/// box corresponding to that child octant.
///
/// The child index uses the same 3-bit encoding as `child_index_for_point`:
///
/// - Bit 0 (value 1): X axis — 0 for left half, 1 for right half.
/// - Bit 1 (value 2): Y axis — 0 for bottom half, 1 for top half.
/// - Bit 2 (value 4): Z axis — 0 for back half, 1 for front half.
///
/// The parent box is split at its center along each axis, and the child
/// box is chosen accordingly.
///
/// # Parameters
/// - `parent_min`: Minimum corner of the parent node's bounding box.
/// - `parent_max`: Maximum corner of the parent node's bounding box.
/// - `child_idx` : Index of the child octant (0–7) using bit flags.
///
/// # Returns
/// A pair `(min, max)` defining the child octant's bounding box.
fn child_bbox(parent_min: &NVec3, parent_max: &NVec3, child_idx: usize) -> (NVec3, NVec3) {
    let center = (parent_min + parent_max) * 0.5;

    let mut min = *parent_min;
    let mut max = *parent_max;

    // x: bit 0
    if (child_idx & 1) == 0 {
        max.x = center.x;
    } else {
        min.x = center.x;
    }

    // y: bit 1
    if (child_idx & 2) == 0 {
        max.y = center.y;
    } else {
        min.y = center.y;
    }

    // z: bit 2
    if (child_idx & 4) == 0 {
        max.z = center.z;
    } else {
        min.z = center.z;
    }

    (min, max)
}