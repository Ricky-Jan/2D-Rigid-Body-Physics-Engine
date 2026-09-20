use crate::aabb::AABB;

pub const NULL_PTR: usize = usize::MAX;

pub struct TreeNode {
    pub aabb: AABB,
    pub parent: usize,
    pub left: usize,
    pub right: usize,
    pub shape_idx: usize,
    pub next_free: usize,
    pub is_leaf: bool,
    pub height: i32,
}
impl TreeNode {
    pub fn new_leaf(shape_idx: usize, aabb: AABB) -> Self {
        Self {
            aabb,
            parent: NULL_PTR,
            left: NULL_PTR,
            right: NULL_PTR,
            shape_idx,
            next_free: NULL_PTR,
            is_leaf: true,
            height: 0,
        }
    }

    pub fn new_internal() -> Self {
        Self {
            aabb: AABB::empty(),
            parent: NULL_PTR,
            left: NULL_PTR,
            right: NULL_PTR,
            shape_idx: 0,
            next_free: NULL_PTR,
            is_leaf: false,
            height: 0,
        }
    }
}

pub struct DynamicBVH {
    pub nodes: Vec<TreeNode>,
    pub root: usize,
    pub free: usize,
}

impl DynamicBVH {
    pub fn new() -> Self {
        Self {
            nodes: Vec::with_capacity(256),
            root: NULL_PTR,
            free: NULL_PTR,
        }
    }

    pub fn alloc_node(&mut self, node: TreeNode) -> usize {
        if self.free != NULL_PTR {
            let idx = self.free;
            self.free = self.nodes[idx].next_free;
            self.nodes[idx] = node;
            idx
        } else {
            let idx = self.nodes.len();
            self.nodes.push(node);
            idx
        }
    }

    pub fn free_node(&mut self, idx: usize) {
        self.nodes[idx].next_free = self.free;
        self.free = idx;
    }

    pub fn insert_leaf(&mut self, leaf: usize) {
        if self.root == NULL_PTR {
            self.root = leaf;
            self.nodes[leaf].parent = NULL_PTR;
            return;
        }

        let leaf_aabb = self.nodes[leaf].aabb;
        let sibling = self.sah(leaf_aabb);
        let old_parent = self.nodes[sibling].parent;

        let mut new_parent_node = TreeNode::new_internal();
        new_parent_node.parent = old_parent;
        new_parent_node.aabb = self.nodes[sibling].aabb.union(&leaf_aabb);

        let new_parent = self.alloc_node(new_parent_node);

        self.nodes[new_parent].left = sibling;
        self.nodes[new_parent].right = leaf;
        self.nodes[sibling].parent = new_parent;
        self.nodes[leaf].parent = new_parent;

        if old_parent != NULL_PTR {
            if self.nodes[old_parent].left == sibling {
                self.nodes[old_parent].left = new_parent;
            } else {
                self.nodes[old_parent].right = new_parent;
            }
        } else {
            self.root = new_parent;
        }

        self.refit(self.nodes[leaf].parent);
    }

    fn sah(&mut self, leaf_aabb: AABB) -> usize {
        let mut search_idx = self.root;

        while !self.nodes[search_idx].is_leaf {
            let left = self.nodes[search_idx].left;
            let right = self.nodes[search_idx].right;

            let area = self.nodes[search_idx].aabb.perimeter();
            let combined_aabb = self.nodes[search_idx].aabb.union(&leaf_aabb);
            let combined_area = combined_aabb.perimeter();

            let cost_create = 2.0 * combined_area;

            let cost_inherit = 2.0 * (combined_area - area);

            let cost_left = if self.nodes[left].is_leaf {
                self.nodes[left].aabb.union(&leaf_aabb).perimeter() + cost_inherit
            } else {
                let old_area = self.nodes[right].aabb.perimeter();
                let new_area = self.nodes[right].aabb.union(&leaf_aabb).perimeter();
                (new_area - old_area) + cost_inherit
            };

            let cost_right = if self.nodes[right].is_leaf {
                self.nodes[right].aabb.union(&leaf_aabb).perimeter() + cost_inherit
            } else {
                let old_area = self.nodes[right].aabb.perimeter();
                let new_area = self.nodes[right].aabb.union(&leaf_aabb).perimeter();
                (new_area - old_area) + cost_inherit
            };

            if cost_create < cost_left && cost_create < cost_right {
                break;
            }

            if cost_left < cost_right {
                search_idx = left;
            } else {
                search_idx = right;
            }
        }

        search_idx
    }

    fn greedy_sah(&mut self, leaf_aabb: AABB) -> usize {
        let mut search_idx = self.root;

        while !self.nodes[search_idx].is_leaf {
            let left = self.nodes[search_idx].left;
            let right = self.nodes[search_idx].right;

            let cost_left = self.nodes[left].aabb.union(&leaf_aabb).perimeter();
            let cost_right = self.nodes[right].aabb.union(&leaf_aabb).perimeter();

            if cost_left < cost_right {
                search_idx = left;
            } else {
                search_idx = right;
            }
        }

        search_idx
    }

    pub fn remove_leaf(&mut self, leaf: usize) {
        if leaf == self.root {
            self.root = NULL_PTR;
            self.free_node(leaf);
            return;
        }

        let parent = self.nodes[leaf].parent;
        let grand_parent = self.nodes[parent].parent;

        let sibling = if self.nodes[parent].left == leaf {
            self.nodes[parent].right
        } else {
            self.nodes[parent].left
        };

        if grand_parent != NULL_PTR {
            if self.nodes[grand_parent].left == parent {
                self.nodes[grand_parent].left = sibling;
            } else {
                self.nodes[grand_parent].right = sibling;
            }

            self.nodes[sibling].parent = grand_parent;
            self.free_node(parent);
            self.refit(grand_parent);
        } else {
            self.root = sibling;
            self.nodes[sibling].parent = NULL_PTR;
            self.free_node(parent);
        }
        self.free_node(leaf);
    }

    fn refit(&mut self, idx: usize) {
        let mut curr_idx = idx;
        while curr_idx != NULL_PTR {
            curr_idx = self.rotate(curr_idx);

            let left = self.nodes[curr_idx].left;
            let right = self.nodes[curr_idx].right;

            self.nodes[curr_idx].aabb = self.nodes[left].aabb.union(&self.nodes[right].aabb);
            self.nodes[curr_idx].height = self.nodes[left].height.max(self.nodes[right].height) + 1;

            curr_idx = self.nodes[curr_idx].parent;
        }
    }

    fn rotate(&mut self, i_a: usize) -> usize {
        if i_a == NULL_PTR || self.nodes[i_a].is_leaf || self.nodes[i_a].height < 2 {
            return i_a;
        }

        let i_b = self.nodes[i_a].left;
        let i_c = self.nodes[i_a].right;

        let height_b = self.nodes[i_b].height;
        let height_c = self.nodes[i_c].height;

        if height_b > height_c {
            let i_d = self.nodes[i_b].left;
            let i_e = self.nodes[i_b].right;

            let aabb_c = self.nodes[i_c].aabb;
            let aabb_d = self.nodes[i_d].aabb;
            let aabb_e = self.nodes[i_e].aabb;

            let cost_ce = aabb_c.union(&aabb_e).perimeter();
            let cost_cd = aabb_c.union(&aabb_d).perimeter();
            let cost_orig = self.nodes[i_b].aabb.perimeter();

            if cost_cd < cost_ce && cost_cd < cost_orig {
                self.nodes[i_a].right = i_d;
                self.nodes[i_d].parent = i_a;

                self.nodes[i_b].left = i_c;
                self.nodes[i_c].parent = i_b;

                self.nodes[i_b].aabb = self.nodes[i_c].aabb.union(&self.nodes[i_e].aabb);
                self.nodes[i_b].height = 1 + self.nodes[i_c].height.max(self.nodes[i_e].height);

                return i_a;
            }

            if cost_ce < cost_cd && cost_ce < cost_orig {
                self.nodes[i_a].right = i_e;
                self.nodes[i_e].parent = i_a;

                self.nodes[i_b].right = i_c;
                self.nodes[i_c].parent = i_b;

                self.nodes[i_b].aabb = self.nodes[i_d].aabb.union(&self.nodes[i_c].aabb);
                self.nodes[i_b].height = 1 + self.nodes[i_d].height.max(self.nodes[i_c].height);

                return i_a;
            }
        }

        if height_c > height_b {
            let i_f = self.nodes[i_c].left;
            let i_g = self.nodes[i_c].right;

            let aabb_b = self.nodes[i_b].aabb;
            let aabb_f = self.nodes[i_f].aabb;
            let aabb_g = self.nodes[i_g].aabb;

            let cost_bf = aabb_b.union(&aabb_f).perimeter();
            let cost_bg = aabb_b.union(&aabb_g).perimeter();
            let cost_orig = self.nodes[i_c].aabb.perimeter();

            if cost_bf < cost_bg && cost_bf < cost_orig {
                self.nodes[i_a].left = i_f;
                self.nodes[i_f].parent = i_a;

                self.nodes[i_c].left = i_b;
                self.nodes[i_b].parent = i_c;

                self.nodes[i_c].aabb = self.nodes[i_b].aabb.union(&self.nodes[i_g].aabb);
                self.nodes[i_c].height = self.nodes[i_b].height.max(self.nodes[i_g].height) + 1;

                return i_a;
            }

            if cost_bg < cost_bf && cost_bg < cost_orig {
                self.nodes[i_a].left = i_g;
                self.nodes[i_g].parent = i_a;

                self.nodes[i_c].right = i_b;
                self.nodes[i_b].parent = i_c;

                self.nodes[i_c].aabb = self.nodes[i_f].aabb.union(&self.nodes[i_b].aabb);
                self.nodes[i_c].height = self.nodes[i_f].height.max(self.nodes[i_b].height) + 1;

                return i_a;
            }
        }

        i_a
    }

    pub fn query(&self, target_aabb: &AABB) -> Vec<usize> {
        let mut result = Vec::new();

        if self.root == NULL_PTR {
            return result;
        }

        let mut stack = Vec::with_capacity(64);
        stack.push(self.root);

        while let Some(node_idx) = stack.pop() {
            let node = &self.nodes[node_idx];

            if !node.aabb.intersect(target_aabb) {
                continue;
            }

            if node.is_leaf {
                result.push(node.shape_idx);
            } else {
                stack.push(node.left);
                stack.push(node.right);
            }
        }

        result
    }
}
