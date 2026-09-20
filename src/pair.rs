use crate::{
    arbiter::Arbiter,
    math::{PRIME_A, PRIME_B},
};

pub const NULL_PTR: usize = usize::MAX;
pub const HASH_SIZE: usize = 4096;

pub struct PairNode {
    pub arbiter: Arbiter,
    pub next: usize,
    pub prev: usize,
    pub active_idx: usize,
    pub next_free: usize,
}

pub struct Pair {
    pub pool: Vec<PairNode>,
    pub hash_table: Vec<usize>,
    pub active_pairs: Vec<usize>,
    pub free: usize,
}

impl Pair {
    pub fn new() -> Self {
        Self {
            pool: Vec::with_capacity(1024),
            hash_table: vec![NULL_PTR; HASH_SIZE],
            active_pairs: Vec::with_capacity(1024),
            free: NULL_PTR,
        }
    }

    fn hash(a: usize, b: usize) -> usize {
        (a.wrapping_mul(PRIME_A) ^ b.wrapping_mul(PRIME_B)) % HASH_SIZE
    }

    fn alloc_node(&mut self) -> usize {
        if self.free != NULL_PTR {
            let idx = self.free;
            self.free = self.pool[idx].next_free;
            idx
        } else {
            self.pool.push(PairNode {
                arbiter: Arbiter::new(0, 0),
                next: NULL_PTR,
                prev: NULL_PTR,
                active_idx: NULL_PTR,
                next_free: NULL_PTR,
            });
            self.pool.len() - 1
        }
    }

    pub fn get(&mut self, body_a: usize, body_b: usize) -> usize {
        let (a, b) = if body_a < body_b {
            (body_a, body_b)
        } else {
            (body_b, body_a)
        };

        let bucket = Self::hash(a, b);

        let mut curr = self.hash_table[bucket];
        while curr != NULL_PTR {
            if self.pool[curr].arbiter.body_a_idx == a && self.pool[curr].arbiter.body_b_idx == b {
                return curr;
            }
            curr = self.pool[curr].next;
        }

        let idx = self.alloc_node();
        self.pool[idx].arbiter = Arbiter::new(a, b);

        self.pool[idx].prev = NULL_PTR;
        self.pool[idx].next = self.hash_table[bucket];
        if self.hash_table[bucket] != NULL_PTR {
            let old_head = self.hash_table[bucket];
            self.pool[old_head].prev = idx;
        }
        self.hash_table[bucket] = idx;

        self.active_pairs.push(idx);
        self.pool[idx].active_idx = self.active_pairs.len() - 1;

        idx
    }

    pub fn remove(&mut self, idx: usize) {
        let prev = self.pool[idx].prev;
        let next = self.pool[idx].next;
        let a = self.pool[idx].arbiter.body_a_idx;
        let b = self.pool[idx].arbiter.body_b_idx;

        if prev == NULL_PTR {
            let bucket = Self::hash(a, b);
            self.hash_table[bucket] = next;
        } else {
            self.pool[prev].next = next;
        }

        if next != NULL_PTR {
            self.pool[next].prev = prev;
        }

        let active_idx = self.pool[idx].active_idx;
        let last_active_idx = self.active_pairs.len() - 1;

        if active_idx != last_active_idx {
            let last_pool_idx = self.active_pairs[last_active_idx];
            self.active_pairs[active_idx] = last_pool_idx;
            self.pool[last_pool_idx].active_idx = active_idx;
        }
        self.active_pairs.pop();

        self.pool[idx].next_free = self.free;
        self.free = idx;
    }
}
