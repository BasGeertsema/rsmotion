#include <array>
#include <cstdint>
#include <iostream>
#include <random>

#include <GL/gl.h>
#include <emscripten/emscripten.h>
#include <emscripten/html5.h>
#include <emscripten/html5_webgl.h>

// RSMotion library
#include <rsmotion/rsmotion.h>

using namespace rsmotion;

/**
 * This class contains most of the relevant example code that uses
 * the RSMotion library. It generates a random path for a car and
 * once the car reaches the end of the path, it generates a new
 * random path.
 */
class CarPathGenerator
{
private:
    std::mt19937 gen;

public:
    CarPathGenerator()
    {
        std::random_device rd; // Will be used to obtain a seed for the random number engine
        gen = std::mt19937(rd());

        using namespace rsmotion::math;

        // set a wheelbase to 1 meter
        const float wheelbase = 1.0f;

        // set the start position to the origin
        const Vec3f startPosition{0.f, 0.f, 0.f};

        // set the orientation (yaw; around y axis) to zero degrees (i.e. no rotation)
        const Quatf startOrientation{Vec3f{0, 1, 0}, Anglef::Degrees(0)};

        // create the initial CarState
        CarState carStart{rsmotion::PointState{startPosition, startOrientation}, wheelbase};

        // generate the first path, simply drive forward

        Start = carStart;
        Finish = {Vec3f{0.f, 0.f, 3.f}, startOrientation};
        Path = SearchShortestPath(Start, Finish);
        //GenerateRandomPath(carStart);
    }

    void GenerateRandomPath(const CarState &newStart)
    {
        // the given state is the new start state
        Start = newStart;

        // setup random distributions for translation and rotation
        std::uniform_real_distribution<float> transDis{-4.0f, 4.0f};
        std::uniform_real_distribution<float> rotDis{-180.0f, 180.0f};

        // Create a new PointState to move towards to.
        // The end point is the state of the _rear_ axis.
        // The front axis is always calculated by aligning it
        // based on the rear axis.
        rsmotion::math::Vec3f newEndPoint{};
        newEndPoint[0] = transDis(gen);
        newEndPoint[1] = 0.f;
        newEndPoint[2] = transDis(gen);
        float rotation = rotDis(gen);
        Finish = PointState{newEndPoint, rsmotion::math::Quatf{{0.f, 1.f, 0.f}, rsmotion::math::Anglef::Degrees(rotation)}};

        // Search for the optimal path and store it so
        // we can visualize it and move the car along the path.
        Path = SearchShortestPath(Start, Finish);
    }

    rsmotion::CarState Start;
    rsmotion::PointState Finish;
    rsmotion::algorithm::Path Path;
};

class State {
    public:
    State() {
    }
    
    CarPathGenerator PathGenerator {};
    
    float Time { -0.2f }; // the elapsed time driving along the path
    rsmotion::math::Vec3f CarCenterPos {}; // Position of the car center
    rsmotion::math::Vec3f CarOrientation { rsmotion::CoordinateSystem::Forward() }; // Direction the car is facing (normalized vector)
    std::vector<rsmotion::math::Vec3f> PathPoints {}; // Points to visualize the current path
};

// the global state of the car and path generator
State carState;

// Our main app loop run each frame
void app_loop(void*);

int main(int, const char **)
{
    // Start the app loop
    emscripten_set_main_loop_arg(app_loop, (void*)&carState, -1, 0);

    return 0;
}

uint32_t hue = 0;

// Export functions for JavaScript to query the simulation state
extern "C" {
    EMSCRIPTEN_KEEPALIVE float getCarPositionX() {
        return carState.CarCenterPos[0];
    }
    
    EMSCRIPTEN_KEEPALIVE float getCarPositionY() {
        return carState.CarCenterPos[1];
    }
    
    EMSCRIPTEN_KEEPALIVE float getCarPositionZ() {
        return carState.CarCenterPos[2];
    }
    
    EMSCRIPTEN_KEEPALIVE float getCarOrientationX() {
        return carState.CarOrientation[0];
    }
    
    EMSCRIPTEN_KEEPALIVE float getCarOrientationY() {
        return carState.CarOrientation[1];
    }
    
    EMSCRIPTEN_KEEPALIVE float getCarOrientationZ() {
        return carState.CarOrientation[2];
    }
    
    EMSCRIPTEN_KEEPALIVE int getPathPointCount() {
        return static_cast<int>(carState.PathPoints.size());
    }
    
    EMSCRIPTEN_KEEPALIVE float getPathPointX(int index) {
        if (index >= 0 && index < static_cast<int>(carState.PathPoints.size())) {
            return carState.PathPoints[index][0];
        }
        return 0.0f;
    }
    
    EMSCRIPTEN_KEEPALIVE float getPathPointY(int index) {
        if (index >= 0 && index < static_cast<int>(carState.PathPoints.size())) {
            return carState.PathPoints[index][1];
        }
        return 0.0f;
    }
    
    EMSCRIPTEN_KEEPALIVE float getPathPointZ(int index) {
        if (index >= 0 && index < static_cast<int>(carState.PathPoints.size())) {
            return carState.PathPoints[index][2];
        }
        return 0.0f;
    }
    
    EMSCRIPTEN_KEEPALIVE float getFinishPositionX() {
        return carState.PathGenerator.Finish.Pos[0];
    }
    
    EMSCRIPTEN_KEEPALIVE float getFinishPositionY() {
        return carState.PathGenerator.Finish.Pos[1];
    }
    
    EMSCRIPTEN_KEEPALIVE float getFinishPositionZ() {
        return carState.PathGenerator.Finish.Pos[2];
    }
    
    EMSCRIPTEN_KEEPALIVE float getFinishOrientationX() {
        auto orientation = carState.PathGenerator.Finish.Orientation * CoordinateSystem::Forward();
        return orientation[0];
    }
    
    EMSCRIPTEN_KEEPALIVE float getFinishOrientationY() {
        auto orientation = carState.PathGenerator.Finish.Orientation * CoordinateSystem::Forward();
        return orientation[1];
    }
    
    EMSCRIPTEN_KEEPALIVE float getFinishOrientationZ() {
        auto orientation = carState.PathGenerator.Finish.Orientation * CoordinateSystem::Forward();
        return orientation[2];
    }
}

std::vector<rsmotion::math::Vec3f> PointsOnLine(const CarPathGenerator &pathGenerator)
{
    // generate the points on the path for visualization
    auto pointsOnLine = std::vector<rsmotion::math::Vec3f>();
    const int totalPoints = 20;
    for (int n = 0; n < totalPoints; ++n)
    {
        auto movedCar = TraversePathNormalized((1.0f / (totalPoints-1)) * n, pathGenerator.Path, pathGenerator.Start);
        const auto &point = movedCar.Rear.Pos + rsmotion::math::Vec3f { 0.f, 0.0f, 0.f };
        pointsOnLine.push_back(point);                
    }
    return pointsOnLine;
}

void app_loop(void*)
{
    // fetch the state for this frame
    State& state = carState;
    CarPathGenerator& pathGenerator = state.PathGenerator;
    
    // get the position of the car at the current time by moving the car along the path
    auto movedCar = TraversePathDistance(state.Time, pathGenerator.Path, pathGenerator.Start);

    // project center of car from rear for positioning the car in the center of the path
    state.CarCenterPos = movedCar.Rear.Pos;
    state.CarOrientation = movedCar.Rear.Orientation * CoordinateSystem::Forward();
    
    if(state.PathPoints.empty())
    {
        state.PathPoints = PointsOnLine(pathGenerator);
    }

    // check whether the car is at the end of the path
    // if so, generate a new path
    if (EuclideanDistance(movedCar.Rear.Pos, pathGenerator.Finish) < 0.02f)
    {
        // set the current car position as the new start position and generate a new path
        pathGenerator.GenerateRandomPath(movedCar);
                
        state.PathPoints = PointsOnLine(pathGenerator);

        // reset the time to -0.2f to pause the car for a while
        state.Time = -0.2f;
    }    

    // increase the time for next frame, this will move the car along the path
    state.Time += 0.02f;
}



