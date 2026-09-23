# Force2D - 2D Rigid-Body Physics Engine

![Rust](https://img.shields.io/badge/rust-%23000000.svg?style=for-the-badge&logo=rust&logoColor=white)
![Dependencies](https://img.shields.io/badge/dependencies-zero%20(core)-success?style=for-the-badge)

Demo: https://ricky-jan.github.io/Force2D/
<p align="center">
  <img src="assets/demo.gif" width="800" alt="FORCE2D Physics Engine Demo">
</p>

Force2D is a personal educational project aimed at understanding the core mechanics of 2D rigid-body physics engines by building one entirely from scratch in Rust. Rather than wrapping existing libraries, this engine serves as a practical exploration of Data-Oriented Design (DOD) principles, memory management, and rigid-body dynamics. The physics core is strictly decoupled from the rendering layer to maintain a clean, portable, and observable architecture.

## Engine Features

The engine supports a comprehensive set of features expected in a modern 2D physics simulation:

* **Shape Support:** Simulates Circles, Rectangles, Regular Polygons, and user-defined Custom Convex Polygons.
* **Compound Bodies:** Supports attaching multiple distinct shapes to a single rigid body to create complex concave geometries (e.g., concave bowls, multi-part vehicles).
* **Joint System:** 
  * *Distance Joint:* Constrains two bodies to maintain a specific distance, modeling springs and soft soft-body physics (e.g., ropes, soft bridges).
  * *Revolute Joint:* Pins two bodies at a shared anchor point, enabling ragdoll physics and mechanical linkages.
  * *Mouse Joint:* Allows interactive dragging and manipulation of dynamic bodies via mouse input.
* **Collision Filtering:** Implements a Box2D-style filtering system using `categoryBits`, `maskBits`, and `groupIndex` to precisely control which shapes can collide, allowing for self-collision exemptions and complex layered interactions.
* **Island Management & Sleeping:** Groups interacting bodies into isolated simulation islands and safely puts resting bodies to sleep, drastically reducing CPU cycles for static scenes.

## Technical Implementations Explored

### Data-Oriented Design and Memory Management
* **Index-Based Entity Management:** This project avoids standard OOP paradigms (such as dynamic dispatch and `Vec<Box<dyn Shape>>`) to study the effects on memory layouts. Entities, shapes, and collision pairs are managed in contiguous memory pools using raw indices, allowing for observations on L1/L2 cache coherency during simulation loops.
* **Zero-Allocation Pipeline:** Intrusive Linked Lists were implemented to attach multiple shapes to a single RigidBody, practicing how to structure complex, multi-part objects without relying on runtime heap allocations or fragmenting the cache.

### Mathematics and Kinetics
* **Green's Theorem for Arbitrary Polygons:** Applied mathematical concepts to calculate the exact geometric area, mass, and local centroid for user-defined convex polygons. This implementation inherently detects and corrects improper vertex winding orders (Clockwise to Counter-Clockwise).
* **Parallel Axis Theorem:** Used this theorem to analytically compute the precise center of mass (COM) and aggregate moment of inertia for complex compound bodies during initialization, guaranteeing physically accurate rotational dynamics.

### Collision Detection Pipeline
* **Broad-Phase (Dynamic BVH):** Constructed a Dynamic Bounding Volume Hierarchy tree utilizing the Surface Area Heuristic (SAH). This section focuses on implementing and understanding $O(\log N)$ tree operations for efficient spatial partitioning, rapid insertions, tree refitting, and pair culling.
* **Narrow-Phase (SAT and Sutherland-Hodgman):** Developed a robust Separating Axis Theorem (SAT) pipeline to resolve `Circle vs Circle`, `Circle vs Polygon`, and `Polygon vs Polygon` collisions. The Sutherland-Hodgman algorithm was integrated to practice clipping incident edges against reference edges to generate accurate multi-point contact manifolds.

### Constraint Resolution
* **Sequential Impulse Solver:** Studied and implemented iterative solving for velocity constraints to achieve realistic physical responses.
* **Warm Starting:** Implemented previous-frame impulse caching to observe its impact on solver convergence rates, providing absolute stability for tall stacks of objects.
* **Baumgarte Stabilization:** Applied this technique to resolve object overlapping and penetration by feeding a portion of the positional error back into the velocity constraint.

## Performance Observations
Through the application of DOD principles and minimal branching, this implementation served as a valuable case study in Rust's performance characteristics. The solver successfully handles over 1,000 active rigid bodies (mixing various shapes and joints) simultaneously at a stable 60 FPS on a single thread, demonstrating the practical computational benefits of cache-friendly data structures.

## Getting Started

The core physics engine is built with zero dependencies to focus purely on the physics implementation. The included demo project utilizes `macroquad` strictly for visualization, debug rendering, and web compilation.

### Build and Run the Demo Locally
```bash
git clone [https://github.com/yourusername/Force2D.git](https://github.com/yourusername/Force2D.git)
cd Force2D
cargo run --release
