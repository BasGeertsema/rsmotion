// Type definitions for the RSMotion visualization

export interface Vec3 {
    x: number;
    y: number;
    z: number;
}

export interface CppApp {
    callMain: () => void;
    ccall: (name: string, returnType: string, argTypes: string[], args: any[]) => any;
    cwrap: (name: string, returnType: string, argTypes: string[]) => (...args: any[]) => any;
    // Direct function access
    _getCarPositionX: () => number;
    _getCarPositionY: () => number;
    _getCarPositionZ: () => number;
    _getCarOrientationX: () => number;
    _getCarOrientationY: () => number;
    _getCarOrientationZ: () => number;
    _getPathPointCount: () => number;
    _getPathPointX: (index: number) => number;
    _getPathPointY: (index: number) => number;
    _getPathPointZ: (index: number) => number;
    _getFinishPositionX: () => number;
    _getFinishPositionY: () => number;
    _getFinishPositionZ: () => number;
    _getFinishOrientationX: () => number;
    _getFinishOrientationY: () => number;
    _getFinishOrientationZ: () => number;
}

export interface CarState {
    position: Vec3;
    orientation: Vec3;
}

export interface PathPoint {
    position: Vec3;
}