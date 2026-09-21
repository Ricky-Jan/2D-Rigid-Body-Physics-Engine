use crate::{
    math::{Complex, EPS, NEG_INF, POS_INF, Vec2},
    settings::BALANCE,
    shape::{Shape, ShapeType},
};

#[derive(Clone, Copy)]
pub struct FeatureCache {
    pub prev_edge_a: usize,
    pub prev_edge_b: usize,
    pub prev_circle_edge: usize,
}

impl FeatureCache {
    pub fn new() -> Self {
        Self {
            prev_edge_a: 0,
            prev_edge_b: 0,
            prev_circle_edge: 0,
        }
    }
}

#[derive(Clone, Copy)]
pub struct ManifoldPoint {
    pub pos: Vec2,
    pub penetration: f64,
}

#[derive(Clone, Copy)]
pub struct Manifold {
    pub points: [ManifoldPoint; 2],
    pub point_count: usize,
    pub normal: Vec2,
}

impl Manifold {
    pub fn new() -> Self {
        Self {
            points: [ManifoldPoint {
                pos: Vec2::zero(),
                penetration: 0.0,
            }; 2],
            point_count: 0,
            normal: Vec2::zero(),
        }
    }

    pub fn add_contact(&mut self, pos: Vec2, normal: Vec2, penetration: f64) {
        if self.point_count < 2 {
            self.normal = normal;
            self.points[self.point_count].pos = pos;
            self.points[self.point_count].penetration = penetration;
            self.point_count += 1;
        }
    }
}

pub fn collide(
    manifold: &mut Manifold,
    shape_a: &Shape,
    shape_b: &Shape,
    cache: &mut FeatureCache,
) -> bool {
    manifold.point_count = 0;

    let a_id = shape_a.shape_type.get_id();
    let b_id = shape_b.shape_type.get_id();

    let swap = a_id > b_id;
    let (s1, s2) = if swap {
        (shape_b, shape_a)
    } else {
        (shape_a, shape_b)
    };

    let is_hit = match (&s1.shape_type, &s2.shape_type) {
        (ShapeType::Circle(c1), ShapeType::Circle(c2)) => {
            circle_vs_circle(manifold, &s1.pos, c1.radius, &s2.pos, c2.radius)
        }
        (ShapeType::Circle(c), ShapeType::Rect(r)) => circle_vs_rect(
            manifold,
            &s1.pos,
            c.radius,
            &s2.pos,
            &s2.heading,
            r.hw,
            r.hh,
        ),
        (ShapeType::Circle(c), ShapeType::Polygon(p)) => circle_vs_poly(
            manifold,
            &s1.pos,
            c.radius,
            &p.world_vertices[0..p.count],
            &p.world_normals[0..p.count],
            cache,
        ),

        (ShapeType::Rect(r1), ShapeType::Rect(r2)) => poly_vs_poly(
            manifold,
            &r1.world_vertices,
            &r1.world_normals,
            &r2.world_vertices,
            &r2.world_normals,
            swap,
            cache,
        ),
        (ShapeType::Rect(r), ShapeType::Polygon(p)) => poly_vs_poly(
            manifold,
            &r.world_vertices,
            &r.world_normals,
            &p.world_vertices[0..p.count],
            &p.world_normals[0..p.count],
            swap,
            cache,
        ),
        (ShapeType::Polygon(p1), ShapeType::Polygon(p2)) => poly_vs_poly(
            manifold,
            &p1.world_vertices[0..p1.count],
            &p1.world_normals[0..p1.count],
            &p2.world_vertices[0..p2.count],
            &p2.world_normals[0..p2.count],
            swap,
            cache,
        ),
        _ => unreachable!("Shape order enforcement failed!"),
    };

    if is_hit && swap {
        manifold.normal = manifold.normal.reverse();
    }
    is_hit
}

fn circle_vs_circle(
    manifold: &mut Manifold,
    pos_a: &Vec2,
    radius_a: f64,
    pos_b: &Vec2,
    radius_b: f64,
) -> bool {
    let delta = pos_b.sub(pos_a);
    let dist_sq = delta.dist_sq();
    let r_sum = radius_a + radius_b;

    if dist_sq < EPS {
        let normal = Vec2::new(1.0, 0.0);
        let pos = Vec2::new(pos_a.x + radius_a, pos_a.y);
        manifold.add_contact(pos, normal, r_sum);
        return true;
    }

    if dist_sq < r_sum * r_sum {
        let dist = dist_sq.sqrt();
        let normal = delta.scale(1.0 / dist);
        let penetration = r_sum - dist;
        let pos = pos_a.add(&normal.scale(radius_a));
        manifold.add_contact(pos, normal, penetration);
        return true;
    }
    false
}

fn circle_vs_rect(
    manifold: &mut Manifold,
    c_pos: &Vec2,
    c_radius: f64,
    r_pos: &Vec2,
    r_heading: &Complex,
    r_hw: f64,
    r_hh: f64,
) -> bool {
    let d = c_pos.sub(r_pos);
    let loc = r_heading.inv_rotate(&d);

    let cls_x = loc.x.clamp(-r_hw, r_hw);
    let cls_y = loc.y.clamp(-r_hh, r_hh);

    let clx = loc.x - cls_x;
    let cly = loc.y - cls_y;
    let len_sq = clx * clx + cly * cly;

    if len_sq > c_radius * c_radius {
        return false;
    }

    let len = len_sq.sqrt();

    if len < EPS {
        let dw = r_hw - loc.x.abs();
        let dh = r_hh - loc.y.abs();
        let normal: Vec2;
        let penetration: f64;

        if dw < dh {
            penetration = dw + c_radius;
            normal = if loc.x > 0.0 {
                r_heading.rotate(&Vec2::new(-1.0, 0.0))
            } else {
                r_heading.rotate(&Vec2::new(1.0, 0.0))
            };
        } else {
            penetration = dh + c_radius;
            normal = if loc.y > 0.0 {
                r_heading.rotate(&Vec2::new(0.0, -1.0))
            } else {
                r_heading.rotate(&Vec2::new(0.0, 1.0))
            };
        }
        let pos = c_pos.add(&normal.scale(c_radius));
        manifold.add_contact(pos, normal, penetration);
        return true;
    }

    let local_n = Vec2::new(-clx / len, -cly / len);
    let normal = r_heading.rotate(&local_n);
    let pos = c_pos.add(&normal.scale(c_radius));
    manifold.add_contact(pos, normal, c_radius - len);
    true
}

fn circle_vs_poly(
    manifold: &mut Manifold,
    c_pos: &Vec2,
    c_radius: f64,
    v_poly: &[Vec2],
    n_poly: &[Vec2],
    cache: &mut FeatureCache,
) -> bool {
    let count = v_poly.len();

    let mut curr_edge = cache.prev_circle_edge;
    if curr_edge >= count {
        curr_edge = 0;
    }

    let get_dist = |idx: usize| -> f64 { c_pos.sub(&v_poly[idx]).dot(&n_poly[idx]) };

    let mut curr_dist = get_dist(curr_edge);

    let mut next_edge = (curr_edge + 1) % count;
    let mut prev_edge = (curr_edge + count - 1) % count;

    let mut next_dist = get_dist(next_edge);

    if next_dist > curr_dist {
        loop {
            curr_edge = next_edge;
            curr_dist = next_dist;
            next_edge = (curr_edge + 1) % count;
            next_dist = get_dist(next_edge);

            if !(next_dist > curr_dist) {
                break;
            }
        }
    } else {
        let mut prev_dist = get_dist(prev_edge);
        if prev_dist > curr_dist {
            loop {
                curr_edge = prev_edge;
                curr_dist = prev_dist;
                prev_edge = (curr_edge + count - 1) % count;
                prev_dist = get_dist(prev_edge);

                if !(prev_dist > curr_dist) {
                    break;
                }
            }
        }
    }

    let best_edge = curr_edge;
    let max_dist = curr_dist;

    cache.prev_circle_edge = best_edge;

    if max_dist > c_radius {
        return false;
    }

    if max_dist < EPS {
        let normal = n_poly[best_edge].reverse();
        let penetration = c_radius - max_dist;
        let pos = c_pos.add(&normal.scale(c_radius));
        manifold.add_contact(pos, normal, penetration);
        return true;
    }

    let v1 = v_poly[best_edge];
    let v2 = v_poly[(best_edge + 1) % count];
    let edge = v2.sub(&v1);

    let dot1 = c_pos.sub(&v1).dot(&edge);
    if dot1 <= 0.0 {
        let delta = c_pos.sub(&v1);
        let dist_sq = delta.dist_sq();
        if dist_sq > c_radius * c_radius {
            return false;
        }
        let dist = dist_sq.sqrt();
        if dist < EPS {
            return false;
        }

        let normal = delta.scale(-1.0 / dist);
        let pos = c_pos.add(&normal.scale(c_radius));
        manifold.add_contact(pos, normal, c_radius - dist);
        return true;
    }

    let dot2 = c_pos.sub(&v2).dot(&edge.reverse());
    if dot2 <= 0.0 {
        let delta = c_pos.sub(&v2);
        let dist_sq = delta.dist_sq();
        if dist_sq > c_radius * c_radius {
            return false;
        }
        let dist = dist_sq.sqrt();
        if dist < EPS {
            return false;
        }

        let normal = delta.scale(-1.0 / dist);
        let pos = c_pos.add(&normal.scale(c_radius));
        manifold.add_contact(pos, normal, c_radius - dist);
        return true;
    }

    let normal = n_poly[best_edge].reverse();
    let pos = c_pos.add(&normal.scale(c_radius));
    manifold.add_contact(pos, normal, c_radius - max_dist);
    true
}

fn find_max_separation(
    v_a: &[Vec2],
    n_a: &[Vec2],
    v_b: &[Vec2],
    is_shape_a: bool,
    cache: &mut FeatureCache,
) -> (f64, usize) {
    let count_a = v_a.len();
    let count_b = v_b.len();

    let mut max_sep = NEG_INF;
    let mut best_edge = 0;

    let start_edge = if is_shape_a {
        cache.prev_edge_a
    } else {
        cache.prev_edge_b
    };
    let start_edge = if start_edge >= count_a { 0 } else { start_edge };

    let mut curr_inc = 0;
    let mut is_first_edge = true;

    for i in 0..count_a {
        let edge_idx = (start_edge + i) % count_a;
        let normal = &n_a[edge_idx];
        let vertex_a = &v_a[edge_idx];

        let mut min_proj = POS_INF;

        if is_first_edge {
            for j in 0..count_b {
                let proj = v_b[j].sub(vertex_a).dot(normal);
                if proj < min_proj {
                    min_proj = proj;
                    curr_inc = j;
                }
            }
            is_first_edge = false;
        } else {
            let get_proj = |idx: usize| -> f64 { v_b[idx].sub(vertex_a).dot(normal) };

            let mut curr_dist = get_proj(curr_inc);
            let mut next_inc = (curr_inc + 1) % count_b;
            let mut prev_inc = (curr_inc + count_b - 1) % count_b;

            let mut next_dist = get_proj(next_inc);

            if next_dist < curr_dist {
                loop {
                    curr_inc = next_inc;
                    curr_dist = next_dist;
                    next_inc = (curr_inc + 1) % count_b;
                    next_dist = get_proj(next_inc);

                    if !(next_dist < curr_dist) {
                        break;
                    }
                }
                min_proj = curr_dist;
            } else {
                let mut prev_dist = get_proj(prev_inc);
                if prev_dist < curr_dist {
                    loop {
                        curr_inc = prev_inc;
                        curr_dist = prev_dist;
                        prev_inc = (curr_inc + count_b - 1) % count_b;
                        prev_dist = get_proj(prev_inc);

                        if !(prev_dist < curr_dist) {
                            break;
                        }
                    }
                }
                min_proj = curr_dist;
            }
        }

        if min_proj > max_sep {
            max_sep = min_proj;
            best_edge = edge_idx;
            if max_sep > 0.0 {
                break;
            }
        }
    }

    if is_shape_a {
        cache.prev_edge_a = best_edge;
    } else {
        cache.prev_edge_b = best_edge;
    }

    (max_sep, best_edge)
}

fn get_incident_edge(ref_normal: &Vec2, n_inc: &[Vec2]) -> usize {
    let mut min_dot = POS_INF;
    let mut inc_edge = 0;
    for i in 0..n_inc.len() {
        let dot = ref_normal.dot(&n_inc[i]);
        if dot < min_dot {
            min_dot = dot;
            inc_edge = i;
        }
    }
    inc_edge
}

fn clip_segment(in1: &Vec2, in2: &Vec2, normal: &Vec2, offset: f64) -> (usize, Vec2, Vec2) {
    let mut count = 0;
    let mut out1 = Vec2::zero();
    let mut out2 = Vec2::zero();

    let dist1 = normal.dot(in1) - offset;
    let dist2 = normal.dot(in2) - offset;

    if dist1 <= 0.0 {
        out1 = *in1;
        count += 1;
    }

    if dist2 <= 0.0 {
        if count == 1 {
            out2 = *in2;
        } else {
            out1 = *in2;
        }
        count += 1;
    }

    if dist1 * dist2 < 0.0 {
        let t = dist1 / (dist1 - dist2);
        let intersect = Vec2::new(in1.x + t * (in2.x - in1.x), in1.y + t * (in2.y - in1.y));
        if count == 1 {
            out2 = intersect;
        } else {
            out1 = intersect;
        }
        count += 1;
    }

    (count, out1, out2)
}

fn poly_vs_poly(
    manifold: &mut Manifold,
    v_a: &[Vec2],
    n_a: &[Vec2],
    v_b: &[Vec2],
    n_b: &[Vec2],
    swap: bool,
    cache: &mut FeatureCache,
) -> bool {
    let is_s1_a = !swap;

    let (sep_a, edge_a) = find_max_separation(v_a, n_a, v_b, is_s1_a, cache);
    if sep_a > 0.0 {
        return false;
    }

    let (sep_b, edge_b) = find_max_separation(v_b, n_b, v_a, !is_s1_a, cache);
    if sep_b > 0.0 {
        return false;
    }

    let flip = sep_b > sep_a + BALANCE;
    let (v_ref, n_ref, best_edge_ref, v_inc, n_inc) = if flip {
        (v_b, n_b, edge_b, v_a, n_a)
    } else {
        (v_a, n_a, edge_a, v_b, n_b)
    };

    let ref_v1 = v_ref[best_edge_ref];
    let ref_v2 = v_ref[(best_edge_ref + 1) % v_ref.len()];
    let ref_normal = n_ref[best_edge_ref];

    let inc_edge = get_incident_edge(&ref_normal, n_inc);
    let inc_v1 = v_inc[inc_edge];
    let inc_v2 = v_inc[(inc_edge + 1) % v_inc.len()];

    let tangent_vec = ref_v2.sub(&ref_v1);
    let dist = tangent_vec.dist_sq().sqrt();

    if dist < EPS {
        return false;
    }

    let ref_tangent = tangent_vec.scale(1.0 / dist);

    let offset1 = ref_tangent.dot(&ref_v1);
    let (count1, c1_v1, c1_v2) = clip_segment(&inc_v1, &inc_v2, &ref_tangent.reverse(), -offset1);
    if count1 < 2 {
        return false;
    }

    let offset2 = ref_tangent.dot(&ref_v2);
    let (count2, c2_v1, c2_v2) = clip_segment(&c1_v1, &c1_v2, &ref_tangent, offset2);
    if count2 < 2 {
        return false;
    }

    let ref_offset = ref_normal.dot(&ref_v1);
    let final_pts = [c2_v1, c2_v2];

    let final_normal = if flip {
        ref_normal.reverse()
    } else {
        ref_normal
    };

    for i in 0..2 {
        let depth = ref_offset - ref_normal.dot(&final_pts[i]);
        if depth > 0.0 {
            manifold.add_contact(final_pts[i], final_normal, depth);
        }
    }

    manifold.point_count > 0
}
