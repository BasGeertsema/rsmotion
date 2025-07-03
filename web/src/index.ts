import CppApp from "./cpp/app.js";
import { CppApp as CppAppType, Vec3 } from "./types";
import { WebGLRenderer } from "./renderer";

(async () => {
  const app = await CppApp() as CppAppType;

  // Get canvas element
  const canvas = document.getElementById('canvas') as HTMLCanvasElement;
  if (!canvas) {
    console.error('Canvas element not found');
    return;
  }

  // Create WebGL renderer
  let renderer: WebGLRenderer;
  try {
    renderer = new WebGLRenderer(canvas);
  } catch (e) {
    console.error('Failed to initialize WebGL:', e);
    return;
  }

  // Start the C++ simulation
  try {
    app.callMain();
  } catch (e) {
    console.error('Error starting C++ simulation:', e);
    return;
  }
   
  // Animation loop
  function animate() {
    // Query car state from WASM
    const carPosition: Vec3 = {
      x: app._getCarPositionX(),
      y: app._getCarPositionY(),
      z: app._getCarPositionZ()
    };
    
    const carOrientation: Vec3 = {
      x: app._getCarOrientationX(),
      y: app._getCarOrientationY(),
      z: app._getCarOrientationZ()
    };
    
    // Query path points
    const pathPointCount = app._getPathPointCount();
    const pathPoints: Vec3[] = [];
    
    for (let i = 0; i < pathPointCount; i++) {
      pathPoints.push({
        x: app._getPathPointX(i),
        y: app._getPathPointY(i),
        z: app._getPathPointZ(i)
      });
    }
    
    // Query finish position and orientation
    const finishPosition: Vec3 = {
      x: app._getFinishPositionX(),
      y: app._getFinishPositionY(),
      z: app._getFinishPositionZ()
    };
    
    const finishOrientation: Vec3 = {
      x: app._getFinishOrientationX(),
      y: app._getFinishOrientationY(),
      z: app._getFinishOrientationZ()
    };
    
    // Render the scene
    renderer.render(carPosition, carOrientation, pathPoints, finishPosition, finishOrientation);

    // Continue animation
    requestAnimationFrame(animate);
  }
  
  // Start animation loop
  requestAnimationFrame(animate);
})();