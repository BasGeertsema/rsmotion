// Geometry generation functions for RSMotion visualization

import { Vec3 } from './types';

export function createGroundPlane(size: number, divisions: number): Float32Array {
    const vertices: number[] = [];
    const halfSize = size / 2;
    const step = size / divisions;
    
    // Generate vertices for a grid of squares
    for (let z = 0; z < divisions; z++) {
        for (let x = 0; x < divisions; x++) {
            const x0 = -halfSize + x * step;
            const z0 = -halfSize + z * step;
            const x1 = x0 + step;
            const z1 = z0 + step;
            
            // Two triangles per square
            // Triangle 1
            vertices.push(x0, 0, z0);
            vertices.push(x1, 0, z0);
            vertices.push(x0, 0, z1);
            
            // Triangle 2
            vertices.push(x1, 0, z0);
            vertices.push(x1, 0, z1);
            vertices.push(x0, 0, z1);
        }
    }
    
    return new Float32Array(vertices);
}

export function createCarBody(width: number, height: number, length: number): Float32Array {
    const w = width / 2;
    
    // Car body (chassis) dimensions
    const bodyHeight = height * 0.6;  // Body is 60% of total height
    const bodyBottom = 0;
    const bodyTop = bodyBottom + bodyHeight;
    
    // Shift the car forward so rear axle is at origin
    const zFront = length;  // Front of car
    const zRear = 0;        // Rear of car at origin
    
    const vertices: number[] = [];
    
    // CAR BODY (Lower Box)
    // Front face
    vertices.push(
        -w, bodyBottom, zFront,
         w, bodyBottom, zFront,
         w, bodyTop, zFront,
        -w, bodyBottom, zFront,
         w, bodyTop, zFront,
        -w, bodyTop, zFront
    );
    
    // Back face
    vertices.push(
        -w, bodyBottom, zRear,
        -w, bodyTop, zRear,
         w, bodyTop, zRear,
        -w, bodyBottom, zRear,
         w, bodyTop, zRear,
         w, bodyBottom, zRear
    );
    
    // Top face
    vertices.push(
        -w, bodyTop, zRear,
        -w, bodyTop, zFront,
         w, bodyTop, zFront,
        -w, bodyTop, zRear,
         w, bodyTop, zFront,
         w, bodyTop, zRear
    );
    
    // Bottom face
    vertices.push(
        -w, bodyBottom, zRear,
         w, bodyBottom, zRear,
         w, bodyBottom, zFront,
        -w, bodyBottom, zRear,
         w, bodyBottom, zFront,
        -w, bodyBottom, zFront
    );
    
    // Right face
    vertices.push(
         w, bodyBottom, zRear,
         w, bodyTop, zRear,
         w, bodyTop, zFront,
         w, bodyBottom, zRear,
         w, bodyTop, zFront,
         w, bodyBottom, zFront
    );
    
    // Left face
    vertices.push(
        -w, bodyBottom, zRear,
        -w, bodyBottom, zFront,
        -w, bodyTop, zFront,
        -w, bodyBottom, zRear,
        -w, bodyTop, zFront,
        -w, bodyTop, zRear
    );
    
    return new Float32Array(vertices);
}

export function createCarCabin(width: number, height: number, length: number): Float32Array {
    // Car cabin dimensions  
    const cabinHeight = height * 0.4;  // Cabin is 40% of total height
    const cabinBottom = height * 0.6;  // Sits on top of body
    const cabinTop = cabinBottom + cabinHeight;
    const cabinWidth = width * 0.8 / 2;  // Cabin is 80% of body width
    const cabinRear = length * 0.1;  // Cabin starts 20% from rear
    const cabinFront = length * 0.5;  // Cabin ends 80% from rear
    
    const vertices: number[] = [];
    
    // Front face
    vertices.push(
        -cabinWidth, cabinBottom, cabinFront,
         cabinWidth, cabinBottom, cabinFront,
         cabinWidth, cabinTop, cabinFront,
        -cabinWidth, cabinBottom, cabinFront,
         cabinWidth, cabinTop, cabinFront,
        -cabinWidth, cabinTop, cabinFront
    );
    
    // Back face
    vertices.push(
        -cabinWidth, cabinBottom, cabinRear,
        -cabinWidth, cabinTop, cabinRear,
         cabinWidth, cabinTop, cabinRear,
        -cabinWidth, cabinBottom, cabinRear,
         cabinWidth, cabinTop, cabinRear,
         cabinWidth, cabinBottom, cabinRear
    );
    
    // Top face
    vertices.push(
        -cabinWidth, cabinTop, cabinRear,
        -cabinWidth, cabinTop, cabinFront,
         cabinWidth, cabinTop, cabinFront,
        -cabinWidth, cabinTop, cabinRear,
         cabinWidth, cabinTop, cabinFront,
         cabinWidth, cabinTop, cabinRear
    );
    
    // Right face
    vertices.push(
         cabinWidth, cabinBottom, cabinRear,
         cabinWidth, cabinTop, cabinRear,
         cabinWidth, cabinTop, cabinFront,
         cabinWidth, cabinBottom, cabinRear,
         cabinWidth, cabinTop, cabinFront,
         cabinWidth, cabinBottom, cabinFront
    );
    
    // Left face
    vertices.push(
        -cabinWidth, cabinBottom, cabinRear,
        -cabinWidth, cabinBottom, cabinFront,
        -cabinWidth, cabinTop, cabinFront,
        -cabinWidth, cabinBottom, cabinRear,
        -cabinWidth, cabinTop, cabinFront,
        -cabinWidth, cabinTop, cabinRear
    );
    
    return new Float32Array(vertices);
}

export function createPathLineStrip(points: Vec3[]): Float32Array {
    const vertices: number[] = [];
    
    for (const point of points) {
        vertices.push(point.x, point.y, point.z);
    }
    
    return new Float32Array(vertices);
}

// Helper function to determine if a square should be white or gray in the checkerboard
export function isWhiteSquare(x: number, z: number): boolean {
    return (x + z) % 2 === 0;
}