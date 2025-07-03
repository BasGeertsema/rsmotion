// WebGL renderer for RSMotion visualization

import { Vec3 } from './types';
import { vertexShaderSource, fragmentShaderSource } from './shaders';
import { createGroundPlane, createCarBody, createCarCabin, createPathLineStrip, isWhiteSquare } from './geometry';

export class WebGLRenderer {
    private gl: WebGL2RenderingContext;
    private program: WebGLProgram;
    
    // Uniforms
    private modelMatrixLocation: WebGLUniformLocation;
    private viewMatrixLocation: WebGLUniformLocation;
    private projectionMatrixLocation: WebGLUniformLocation;
    private colorLocation: WebGLUniformLocation;
    
    // Attributes
    private positionAttributeLocation: number;
    
    // Buffers
    private groundBuffer: WebGLBuffer;
    private carBodyBuffer: WebGLBuffer;
    private carCabinBuffer: WebGLBuffer;
    private pathBuffer: WebGLBuffer | null = null;
    private axesBuffer: WebGLBuffer;
    
    // Vertex counts
    private groundVertexCount: number;
    private carBodyVertexCount: number;
    private carCabinVertexCount: number;
    private pathVertexCount: number = 0;
    
    // Ground plane data for colored squares
    private groundSquares: { buffer: WebGLBuffer; color: number[]; count: number }[] = [];
    
    // Cached matrices
    private viewMatrix: Float32Array;
    private projectionMatrix: Float32Array;
    private identityMatrix: Float32Array;
    
    constructor(canvas: HTMLCanvasElement) {
        const gl = canvas.getContext('webgl2');
        if (!gl) {
            throw new Error('WebGL2 not supported');
        }
        this.gl = gl;
        
        // Create shader program
        this.program = this.createShaderProgram();
        
        // Get uniform locations
        this.modelMatrixLocation = gl.getUniformLocation(this.program, 'u_modelMatrix')!;
        this.viewMatrixLocation = gl.getUniformLocation(this.program, 'u_viewMatrix')!;
        this.projectionMatrixLocation = gl.getUniformLocation(this.program, 'u_projectionMatrix')!;
        this.colorLocation = gl.getUniformLocation(this.program, 'u_color')!;
        
        // Get attribute location
        this.positionAttributeLocation = gl.getAttribLocation(this.program, 'a_position');
        
        // Create geometry buffers
        this.groundBuffer = this.createGroundBuffer();
        this.createCarBuffers();
        this.axesBuffer = this.createAxesBuffer();
        
        // Setup WebGL state
        gl.enable(gl.DEPTH_TEST);
        gl.enable(gl.BLEND);
        gl.blendFunc(gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA);
        gl.clearColor(0.529, 0.808, 0.922, 1.0); // Sky blue
        
        // Pre-calculate constant matrices
        this.viewMatrix = this.createViewMatrix();
        this.projectionMatrix = this.createProjectionMatrix();
        this.identityMatrix = new Float32Array([
            1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1
        ]);
    }
    
    private createShaderProgram(): WebGLProgram {
        const gl = this.gl;
        
        // Create vertex shader
        const vertexShader = gl.createShader(gl.VERTEX_SHADER)!;
        gl.shaderSource(vertexShader, vertexShaderSource);
        gl.compileShader(vertexShader);
        
        if (!gl.getShaderParameter(vertexShader, gl.COMPILE_STATUS)) {
            console.error('Vertex shader compilation error:', gl.getShaderInfoLog(vertexShader));
            throw new Error('Failed to compile vertex shader');
        }
        
        // Create fragment shader
        const fragmentShader = gl.createShader(gl.FRAGMENT_SHADER)!;
        gl.shaderSource(fragmentShader, fragmentShaderSource);
        gl.compileShader(fragmentShader);
        
        if (!gl.getShaderParameter(fragmentShader, gl.COMPILE_STATUS)) {
            console.error('Fragment shader compilation error:', gl.getShaderInfoLog(fragmentShader));
            throw new Error('Failed to compile fragment shader');
        }
        
        // Create program
        const program = gl.createProgram()!;
        gl.attachShader(program, vertexShader);
        gl.attachShader(program, fragmentShader);
        gl.linkProgram(program);
        
        if (!gl.getProgramParameter(program, gl.LINK_STATUS)) {
            console.error('Shader program linking error:', gl.getProgramInfoLog(program));
            throw new Error('Failed to link shader program');
        }
        
        return program;
    }
    
    private createGroundBuffer(): WebGLBuffer {
        const gl = this.gl;
        
        // Create checkered ground with individual squares
        const size = 20;
        const divisions = 20;
        const step = size / divisions;
        const halfSize = size / 2;
        
        // Create separate buffers for white and gray squares
        for (let z = 0; z < divisions; z++) {
            for (let x = 0; x < divisions; x++) {
                const x0 = -halfSize + x * step;
                const z0 = -halfSize + z * step;
                const x1 = x0 + step;
                const z1 = z0 + step;
                
                const vertices = new Float32Array([
                    // Two triangles per square, a little bit descended
                    x0, -0.02, z0,
                    x1, -0.02, z0,
                    x0, -0.02, z1,
                    
                    x1, -0.02, z0,
                    x1, -0.02, z1,
                    x0, -0.02, z1,
                ]);
                
                const buffer = gl.createBuffer()!;
                gl.bindBuffer(gl.ARRAY_BUFFER, buffer);
                gl.bufferData(gl.ARRAY_BUFFER, vertices, gl.STATIC_DRAW);
                
                const isWhite = isWhiteSquare(x, z);
                const color = isWhite ? [1.0, 1.0, 1.0, 1.0] : [0.5, 0.5, 0.5, 1.0];
                
                this.groundSquares.push({ buffer, color, count: 6 });
            }
        }
        
        // Return dummy buffer (not used)
        return gl.createBuffer()!;
    }
    
    private createCarBuffers(): void {
        const gl = this.gl;
        
        // Car dimensions: width=0.4, height=0.3, length=0.5
        const carWidth = 0.3;
        const carHeight = 0.4;
        const carLength = 0.9;
        
        // Create body buffer
        const bodyVertices = createCarBody(carWidth, carHeight, carLength);
        this.carBodyBuffer = gl.createBuffer()!;
        gl.bindBuffer(gl.ARRAY_BUFFER, this.carBodyBuffer);
        gl.bufferData(gl.ARRAY_BUFFER, bodyVertices, gl.STATIC_DRAW);
        this.carBodyVertexCount = bodyVertices.length / 3;
        
        // Create cabin buffer
        const cabinVertices = createCarCabin(carWidth, carHeight, carLength);
        this.carCabinBuffer = gl.createBuffer()!;
        gl.bindBuffer(gl.ARRAY_BUFFER, this.carCabinBuffer);
        gl.bufferData(gl.ARRAY_BUFFER, cabinVertices, gl.STATIC_DRAW);
        this.carCabinVertexCount = cabinVertices.length / 3;
    }
    
    private createAxesBuffer(): WebGLBuffer {
        const gl = this.gl;
        
        // Create lines for X, Y, Z axes
        // Each axis is 2 units long, centered at origin
        const vertices = new Float32Array([
            // X axis (red) - along X
            -1, 0, 0,
            1, 0, 0,
            
            // Y axis (green) - along Y
            0, -1, 0,
            0, 1, 0,
            
            // Z axis (blue) - along Z
            0, 0, -1,
            0, 0, 1,
        ]);
        
        const buffer = gl.createBuffer()!;
        gl.bindBuffer(gl.ARRAY_BUFFER, buffer);
        gl.bufferData(gl.ARRAY_BUFFER, vertices, gl.STATIC_DRAW);
        
        return buffer;
    }
    
    public updatePath(points: Vec3[]): void {
        const gl = this.gl;
        
        if (points.length === 0) {
            this.pathBuffer = null;
            this.pathVertexCount = 0;
            return;
        }
        
        const vertices = createPathLineStrip(points);
        
        if (!this.pathBuffer) {
            this.pathBuffer = gl.createBuffer()!;
        }
        
        gl.bindBuffer(gl.ARRAY_BUFFER, this.pathBuffer);
        gl.bufferData(gl.ARRAY_BUFFER, vertices, gl.DYNAMIC_DRAW);
        
        this.pathVertexCount = vertices.length / 3;
    }
    
    public render(carPosition: Vec3, carOrientation: Vec3, pathPoints: Vec3[], finishPosition: Vec3, finishOrientation: Vec3): void {
        const gl = this.gl;
        
        // Clear the canvas
        gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
        
        // Use shader program
        gl.useProgram(this.program);
        
        // Set up view and projection matrices (already cached)
        gl.uniformMatrix4fv(this.viewMatrixLocation, false, this.viewMatrix);
        gl.uniformMatrix4fv(this.projectionMatrixLocation, false, this.projectionMatrix);
        
        // Enable vertex attribute
        gl.enableVertexAttribArray(this.positionAttributeLocation);
        
        // Render components
        this.renderGround();
        this.renderPath(pathPoints);
        this.renderCar(carPosition, carOrientation, 0.0, 0.0, 0.8, 0.3, 0.5, 1.0, 1.0); // Blue moving car
        this.renderCar(finishPosition, finishOrientation, 0.2, 0.9, 0.2, 0.4, 0.9, 0.4, 0.8); // Red finish car
        
        // only during debug
        // this.renderAxes();
        
        // Disable vertex attribute
        gl.disableVertexAttribArray(this.positionAttributeLocation);
    }
    
    private renderGround(): void {
        const gl = this.gl;
        gl.uniformMatrix4fv(this.modelMatrixLocation, false, this.identityMatrix);
        
        for (const square of this.groundSquares) {
            gl.bindBuffer(gl.ARRAY_BUFFER, square.buffer);
            gl.vertexAttribPointer(this.positionAttributeLocation, 3, gl.FLOAT, false, 0, 0);
            gl.uniform4fv(this.colorLocation, square.color);
            gl.drawArrays(gl.TRIANGLES, 0, square.count);
        }
    }
    
    private renderPath(pathPoints: Vec3[]): void {
        const gl = this.gl;
        
        this.updatePath(pathPoints);
        if (this.pathBuffer && this.pathVertexCount > 0) {
            gl.bindBuffer(gl.ARRAY_BUFFER, this.pathBuffer);
            gl.vertexAttribPointer(this.positionAttributeLocation, 3, gl.FLOAT, false, 0, 0);
            gl.uniformMatrix4fv(this.modelMatrixLocation, false, this.identityMatrix);
            gl.uniform4f(this.colorLocation, 0.4, 0.8, 0.4, 1.0); // Green path
            gl.lineWidth(6.0);
            gl.drawArrays(gl.LINE_STRIP, 0, this.pathVertexCount);
        }
    }
    
    private renderCar(position: Vec3, orientation: Vec3, bodyR: number, bodyG: number, bodyB: number, cabinR: number, cabinG: number, cabinB: number, alpha: number): void {
        const gl = this.gl;
        
        const carModelMatrix = this.createCarModelMatrix(position, orientation);
        gl.uniformMatrix4fv(this.modelMatrixLocation, false, carModelMatrix);
        
        // Draw car body
        gl.bindBuffer(gl.ARRAY_BUFFER, this.carBodyBuffer);
        gl.vertexAttribPointer(this.positionAttributeLocation, 3, gl.FLOAT, false, 0, 0);
        gl.uniform4f(this.colorLocation, bodyR, bodyG, bodyB, alpha);
        gl.drawArrays(gl.TRIANGLES, 0, this.carBodyVertexCount);
        
        // Draw car cabin
        gl.bindBuffer(gl.ARRAY_BUFFER, this.carCabinBuffer);
        gl.vertexAttribPointer(this.positionAttributeLocation, 3, gl.FLOAT, false, 0, 0);
        gl.uniform4f(this.colorLocation, cabinR, cabinG, cabinB, alpha);
        gl.drawArrays(gl.TRIANGLES, 0, this.carCabinVertexCount);
    }
    
    private renderAxes(): void {
        const gl = this.gl;
        
        gl.bindBuffer(gl.ARRAY_BUFFER, this.axesBuffer);
        gl.vertexAttribPointer(this.positionAttributeLocation, 3, gl.FLOAT, false, 0, 0);
        gl.uniformMatrix4fv(this.modelMatrixLocation, false, this.identityMatrix);
        gl.lineWidth(8.0);
        
        // X axis - Red
        gl.uniform4f(this.colorLocation, 1.0, 0.0, 0.0, 1.0);
        gl.drawArrays(gl.LINES, 0, 2);
        
        // Y axis - Green
        gl.uniform4f(this.colorLocation, 0.0, 1.0, 0.0, 1.0);
        gl.drawArrays(gl.LINES, 2, 2);
        
        // Z axis - Blue
        gl.uniform4f(this.colorLocation, 0.0, 0.0, 1.0, 1.0);
        gl.drawArrays(gl.LINES, 4, 2);
    }
    
    private createViewMatrix(): Float32Array {
        // Camera at a better angle to see the car structure
        const eye = [3, 5, -4];
        const center = [0, 0, 0];
        const up = [0, 1, 0];
        
        return this.lookAt(eye, center, up);
    }
    
    private createProjectionMatrix(): Float32Array {
        // Orthographic projection
        const left = -4;
        const right = 4;
        const bottom = -4;
        const top = 4;
        const near = 0.1;
        const far = 100;
        
        return this.orthographic(left, right, bottom, top, near, far);
    }
    
    private createCarModelMatrix(position: Vec3, orientation: Vec3): Float32Array {
        // Create transformation matrix for the car
        // First translate to position, then rotate to match orientation
        
        // Calculate rotation angle from orientation vector
        const angle = Math.atan2(orientation.x, orientation.z); // Convert degrees to radians
        
        const c = Math.cos(angle);
        const s = Math.sin(angle);
        
        // Combined rotation and translation matrix
        return new Float32Array([
            c, 0, -s, 0,
            0, 1, 0, 0,
            s, 0, c, 0,
            position.x, position.y, position.z, 1
        ]);
    }
    
    // Matrix math helpers
    private lookAt(eye: number[], center: number[], up: number[]): Float32Array {
        const zAxis = this.normalize([
            eye[0] - center[0],
            eye[1] - center[1],
            eye[2] - center[2]
        ]);
        const xAxis = this.normalize(this.cross(up, zAxis));
        const yAxis = this.cross(zAxis, xAxis);
        
        const tx = -this.dot(xAxis, eye);
        const ty = -this.dot(yAxis, eye);
        const tz = -this.dot(zAxis, eye);
        
        return new Float32Array([
            xAxis[0], yAxis[0], zAxis[0], 0,
            xAxis[1], yAxis[1], zAxis[1], 0,
            xAxis[2], yAxis[2], zAxis[2], 0,
            tx, ty, tz, 1
        ]);
    }
    
    private dot(a: number[], b: number[]): number {
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    }
    
    private orthographic(left: number, right: number, bottom: number, top: number, near: number, far: number): Float32Array {
        return new Float32Array([
            2 / (right - left), 0, 0, 0,
            0, 2 / (top - bottom), 0, 0,
            0, 0, -2 / (far - near), 0,
            -(right + left) / (right - left), -(top + bottom) / (top - bottom), -(far + near) / (far - near), 1
        ]);
    }
    
    private normalize(v: number[]): number[] {
        const length = Math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
        return [v[0] / length, v[1] / length, v[2] / length];
    }
    
    private cross(a: number[], b: number[]): number[] {
        return [
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0]
        ];
    }
}