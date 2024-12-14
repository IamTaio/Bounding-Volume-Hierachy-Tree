# Bounding Volume Hierarchy (BVH) Tree Implementation

## 1. Project Overview

This project implements a Bounding Volume Hierarchy (BVH) tree for efficient collision detection in 2D space. 

- **Main Purpose**: Efficiently detect collisions between axis-aligned bounding boxes (AABBs) in a 2D space
- **Core Functionality**: 
  - Dynamic insertion and removal of objects
  - Object movement tracking
  - Collision detection queries
  - Tree structure optimization for performance
- **Technologies**: C++ (standard library)
- **Unique Features**: 
  - Area-minimizing insertion strategy
  - Efficient collision detection using hierarchical structure
  - Dynamic restructuring on object movement

## 2. Problem Statement

### Problem
Efficiently detect collisions between multiple 2D objects in a scene where objects can be added, removed, or moved.

### Inputs
- **Agent File** (`agent.txt`): Defines initial objects in the scene
  ```
  component_name min_x min_y max_x max_y
  ```
  Example:
  ```
  head 10 50 30 70
  torso 0 20 40 50
  ```

- **Actions File** (`actions.txt`): Defines operations to perform
  ```
  c min_x min_y max_x max_y  // Check collisions
  m component_name min_x min_y max_x max_y  // Move component
  p  // Print tree state
  ```

### Outputs
- Collision detection results (sorted component names)
- Movement confirmation messages
- Tree structure visualization

Example:
```
Projectile (5, 15, 25, 35)
Collides with: torso
Moved the head to location (15, 55, 35, 75)
```

## 3. Implementation Details

### Main Components

1. **AABB Class**
   - Represents axis-aligned bounding boxes
   - Handles collision detection between boxes
   - Computes union areas and intersections

2. **BVHTreeNode Class**
   - Tree node containing AABB data
   - Tracks parent and child relationships
   - Maintains leaf/branch status

3. **BVHTree Class**
   - Manages the tree structure
   - Handles insertions, deletions, and updates
   - Performs collision queries

### Key Functions

```cpp
void addBVHMember(AABB objectArea, string name)  // Add new object
void removeBVHMember(string name)  // Remove existing object
void moveBVHMember(string name, AABB newLocation)  // Update object position
vector<string> getCollidingObjects(AABB object)  // Query collisions
```

### Data Structures
- Binary tree for spatial partitioning
- Unordered map for O(1) object lookup
- Vector for collision result storage

## 4. Usage Instructions

### Compilation
```bash
g++ -o bvh_tree main.cpp BVHTree.cpp -std=c++11
```

### Execution
1. Create input files:
   - `agent.txt`: Define initial objects
   - `actions.txt`: Define operations

2. Run the program:
```bash
./bvh_tree
```

### Input File Requirements

`agent.txt`:
```
component_name x1 y1 x2 y2
```

`actions.txt`:
```
c x1 y1 x2 y2    # Check collisions
m name x1 y1 x2 y2    # Move object
p    # Print tree
```

## 5. Error Handling

The system includes several error handling mechanisms:

- File not found errors for input files
  - Program exits with error code 1 if either agent.txt or actions.txt is missing
- Duplicate component name handling
  - Silently ignores attempts to add components with existing names
- Invalid component movement handling
  - Safely handles movement requests for non-existent components
- Non-existent component removal handling
  - Gracefully ignores attempts to remove components that don't exist

## 6. Example Session

Here's a complete example demonstrating the core functionality:

```
# agent.txt
head 10 50 30 70
torso 0 20 40 50
arm_left -10 30 0 45

# actions.txt
p
c 5 15 25 35
m head 15 55 35 75
p

# Output
Current tree:
+ branch || min = (0, 20), max = (40, 70), Area = 1000
  - R - leaf: head || min = (10, 50), max = (30, 70), Area = 400
  - L - leaf: torso || min = (0, 20), max = (40, 50), Area = 800

Projectile (5, 15, 25, 35)
Collides with: torso

Moved the head to location (15, 55, 35, 75)

Current tree:
+ branch || min = (0, 20), max = (40, 75), Area = 1100
  - R - leaf: head || min = (15, 55), max = (35, 75), Area = 400
  - L - leaf: torso || min = (0, 20), max = (40, 50), Area = 800
```

This example demonstrates:
1. Initial tree construction with multiple components
2. Collision detection with a projectile
3. Component movement
4. Tree structure visualization before and after movement

The tree's automatic restructuring and area optimization can be observed in the changing dimensions and structure of the branches.
