#include <iostream>
#include <cstring>
#include <algorithm>
#include <iterator>
#include <fstream>
#include <random>

// RSMotion library
#include <rsmotion/rsmotion.h>

#include <stdio.h>
#include <bx/bx.h>
#include <bgfx/bgfx.h>
#include <bgfx/platform.h>
#include <GLFW/glfw3.h>

#if BX_PLATFORM_LINUX
#define GLFW_EXPOSE_NATIVE_X11
#elif BX_PLATFORM_WINDOWS
#define GLFW_EXPOSE_NATIVE_WIN32
#endif
#include <GLFW/glfw3native.h>
#include "logo.h"
#include <bx/math.h>

#include "debugdraw/debugdraw.h"

using namespace rsmotion;

/**
 * This class contains most of the relevant example code that uses
 * the RSMotion library. It is used in the main procedure below.
 */
class CarPathGenerator
{
private:    
    std::mt19937 gen;

public:
    CarPathGenerator()
    {
        std::random_device rd; //Will be used to obtain a seed for the random number engine
        gen = std::mt19937(rd());
        
        using namespace rsmotion::math;

        // set a wheelbase to 1 meter
        const float wheelbase = 1.0f;

        // set the start position to the origin
        const Vec3f startPosition {0.f, 0.f, 0.f};
        
        // set the orientation (yaw; around y axis) to zero degrees (i.e. no rotation)
        const Quatf startOrientation { Vec3f{0,1,0}, Anglef::Degrees(0) };

        // create the initial CarState
        CarState carStart{rsmotion::PointState{startPosition, startOrientation}, wheelbase};

        // generate the first path
        GenerateRandomPath(carStart);
    }
    
    void GenerateRandomPath(const CarState& newStart) {
        // the given state is the new start state
        Start = newStart;

        // setup random distributions for translation and rotation
        std::uniform_real_distribution<float> transDis{-2.0f, 2.0f};
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


/*********************
 * 
 * NOTE: 
 *  The code below contains mostly code for windowing and graphics
 *  and is less relevant to the RSMotion library
 *
 ********************/

static void glfw_errorCallback(int error, const char *description)
{
    fprintf(stderr, "GLFW error %d: %s\n", error, description);
}


bgfx::ShaderHandle loadShader(const std::string& path)
{
    std::ifstream shaderFile(path, std::ios::binary);
    std::vector<uint8_t> fileContents((std::istreambuf_iterator<char>(shaderFile)),
                                        std::istreambuf_iterator<char>());


    // add an extra 0 at the end. Not sure why, but the bgfx example code
    // does it so that is why..
    const auto memory = bgfx::alloc(static_cast<uint32_t>(fileContents.size()) + 1);
    std::copy_n(fileContents.data(), fileContents.size(), memory->data);
    memory->data[memory->size - 1] = '\0';
    
    auto handle = bgfx::createShader(memory);

    return handle;
}


struct PosColorVertex
{
	float m_x;
	float m_y;
	float m_z;
	uint32_t m_abgr;

	static void init()
	{
		ms_decl
			.begin()
			.add(bgfx::Attrib::Position, 3, bgfx::AttribType::Float)
			.add(bgfx::Attrib::Color0,   4, bgfx::AttribType::Uint8, true)
			.end();
	};

	static bgfx::VertexDecl ms_decl;
};

bgfx::VertexDecl PosColorVertex::ms_decl;

static PosColorVertex s_cubeVertices[] =
{
	{-1.0f,  1.0f,  1.0f, 0xffff0000 },
	{ 1.0f,  1.0f,  1.0f, 0xffff0000 },
	{-1.0f, -1.0f,  1.0f, 0xffff0000 },
	{ 1.0f, -1.0f,  1.0f, 0xffff0000 },
	{-1.0f,  1.0f, -1.0f, 0xffaa0000 },
	{ 1.0f,  1.0f, -1.0f, 0xffaa0000 },
	{-1.0f, -1.0f, -1.0f, 0xffaa0000 },
	{ 1.0f, -1.0f, -1.0f, 0xffaa0000 },
};

static PosColorVertex s_planeVertices[] =
{
	{-1.0f,  0.0f,  1.0f, 0xff888888 },
	{ 1.0f,  0.0f,  1.0f, 0xff888888 },
	{-1.0f,  0.0f, -1.0f, 0xff888888 },
	{ 1.0f,  0.0f, -1.0f, 0xff888888 },

    {-1.0f+2.0f, 0.0f,  1.0f, 0xffaaaaaa },
    { 1.0f+2.0f, 0.0f,  1.0f, 0xffaaaaaa },
    {-1.0f+2.0f, 0.0f, -1.0f, 0xffaaaaaa },
    { 1.0f+2.0f, 0.0f, -1.0f, 0xffaaaaaa },

    {-1.0f+2.0f,  0.0f,  1.0f+2.0f, 0xff888888 },
	{ 1.0f+2.0f,  0.0f,  1.0f+2.0f, 0xff888888 },
	{-1.0f+2.0f,  0.0f, -1.0f+2.0f, 0xff888888 },
	{ 1.0f+2.0f,  0.0f, -1.0f+2.0f, 0xff888888 },

    {-1.0f,  0.0f,  1.0f+2.0f, 0xffaaaaaa },
	{ 1.0f,  0.0f,  1.0f+2.0f, 0xffaaaaaa },
	{-1.0f,  0.0f, -1.0f+2.0f, 0xffaaaaaa },
	{ 1.0f,  0.0f, -1.0f+2.0f, 0xffaaaaaa }
};

static const uint16_t s_planeTriList[] =
{
	0, 2, 1,
	2, 3, 1,

    0+4, 2+4, 1+4,
	2+4, 3+4, 1+4,

    0+8, 2+8, 1+8,
	2+8, 3+8, 1+8,

    0+12, 2+12, 1+12,
	2+12, 3+12, 1+12
};

static const uint16_t s_cubeTriList[] =
{
	0, 1, 2, // 0
	1, 3, 2,
	4, 6, 5, // 2
	5, 6, 7,
	0, 2, 4, // 4
	4, 2, 6,
	1, 5, 3, // 6
	5, 7, 3,
	0, 4, 1, // 8
	4, 5, 1,
	2, 3, 6, // 10
	6, 3, 7,
};

int main(int argc, char **argv)
{
    // Create a GLFW window without an OpenGL context.
    glfwSetErrorCallback(glfw_errorCallback);
    if (!glfwInit())
        return 1;
    glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
    GLFWwindow *window = glfwCreateWindow(1024, 768, "Reeds Shepp Car Demo", nullptr, nullptr);
    if (!window)
        return 1;

    // Call bgfx::renderFrame before bgfx::init to signal to bgfx not to create a render thread.
    // Most graphics APIs must be used on the same thread that created the window.
    bgfx::renderFrame();
    // Initialize bgfx using the native window handle and window resolution.
    bgfx::Init init;
#if BX_PLATFORM_LINUX || BX_PLATFORM_BSD
    init.platformData.ndt = glfwGetX11Display();
    init.platformData.nwh = (void *)(uintptr_t)glfwGetX11Window(window);
#elif BX_PLATFORM_OSX
    init.platformData.nwh = glfwGetCocoaWindow(window);
#elif BX_PLATFORM_WINDOWS
    init.platformData.nwh = glfwGetWin32Window(window);
#endif
    int width, height;
    glfwGetWindowSize(window, &width, &height);
    init.resolution.width = (uint32_t)width;
    init.resolution.height = (uint32_t)height;
    init.resolution.reset = BGFX_RESET_VSYNC;
    if (!bgfx::init(init))
        return 1;

    bgfx::setDebug(BGFX_DEBUG_PROFILER);    

    // Create vertex stream declaration.
    PosColorVertex::init();

    // Create static vertex buffer.
    bgfx::VertexBufferHandle vbCube = bgfx::createVertexBuffer(
        // Static data can be passed with bgfx::makeRef
        bgfx::makeRef(s_cubeVertices, sizeof(s_cubeVertices)), PosColorVertex::ms_decl);

    // Create static index buffer for triangle list rendering.
    auto ibCube = bgfx::createIndexBuffer(
        // Static data can be passed with bgfx::makeRef
        bgfx::makeRef(s_cubeTriList, sizeof(s_cubeTriList)));

    auto ibPlane = bgfx::createIndexBuffer(
        // Static data can be passed with bgfx::makeRef
        bgfx::makeRef(s_planeTriList, sizeof(s_planeTriList)));

    bgfx::VertexBufferHandle vbPlane = bgfx::createVertexBuffer(
        // Static data can be passed with bgfx::makeRef
        bgfx::makeRef(s_planeVertices, sizeof(s_planeVertices)), PosColorVertex::ms_decl);

    // Create program from shaders.
    auto vsShader = loadShader("vs_cubes.bin");
    auto fsShader = loadShader("fs_cubes.bin");
    auto m_program  = bgfx::createProgram(vsShader, fsShader, true);

    // Set view 0 to the same dimensions as the window and to clear the color buffer.
    const bgfx::ViewId kClearView = 0;
    bgfx::setViewClear(kClearView, BGFX_CLEAR_COLOR | BGFX_CLEAR_DEPTH, 0x303030ff, 1.0f, 0);
    bgfx::setViewRect(kClearView, 0, 0, bgfx::BackbufferRatio::Equal);

    ddInit();

    // initialize
    CarPathGenerator pathGenerator;
    float time = -0.2f;

    while (!glfwWindowShouldClose(window))
    {
        glfwPollEvents();
        // Handle window resize.
        int oldWidth = width, oldHeight = height;
        glfwGetWindowSize(window, &width, &height);
        if (width != oldWidth || height != oldHeight)
        {
            bgfx::reset((uint32_t)width, (uint32_t)height, BGFX_RESET_VSYNC);
            bgfx::setViewRect(kClearView, 0, 0, bgfx::BackbufferRatio::Equal);
        }

        // This dummy draw call is here to make sure that view 0 is cleared if no other draw calls are submitted to view 0.
        bgfx::touch(kClearView);

        // update
        const bx::Vec3 at = {0.0f, 0.0f, 0.0f};
        const bx::Vec3 eye = {5.0f, 7.5f, -10.0f};

        // Set view and projection matrix for view 0.
        {
            float view[16];
            bx::mtxLookAt(view, eye, at);

            float proj[16];
            bx::mtxProj(proj, 20.0f, float(width) / float(height), 0.1f, 100.0f, bgfx::getCaps()->homogeneousDepth);
            bgfx::setViewTransform(0, view, proj);

            bgfx::setViewRect(0, 0, 0, uint16_t(width), uint16_t(height));
        }

        bgfx::IndexBufferHandle ibh = ibCube;
        uint64_t state = 0 | BGFX_STATE_WRITE_R | BGFX_STATE_WRITE_G | BGFX_STATE_WRITE_B | BGFX_STATE_WRITE_A | BGFX_STATE_WRITE_Z | BGFX_STATE_DEPTH_TEST_LESS | BGFX_STATE_CULL_CW | BGFX_STATE_MSAA;

        bgfx::touch(0);

        // draw ground
        for(int x = -20; x < 20; x++) 
        {
            for (int z = -20; z < 20; z++)
            {
                float mtx[16];
                bx::mtxSRT(mtx, 0.2f,  0.2f, 0.2f, 0.f, 0.f, 0.f, x * 0.8f, 0.f, z * 0.8f );

                // Set model matrix for rendering.
                bgfx::setTransform(mtx);

                // Set vertex and index buffer.
                bgfx::setVertexBuffer(0, vbPlane);
                bgfx::setIndexBuffer(ibPlane);

                // Set render states.
                bgfx::setState(state);

                // Submit primitive for rendering to view 0.
                bgfx::submit(0, m_program);
            }
        }

        {
            // draw the car as a box
            auto movedCar = TraversePathDistance(time, pathGenerator.Path, pathGenerator.Start);

            // project center of car from rear
            auto centerPos = movedCar.Rear.Pos + (movedCar.Rear.Orientation * rsmotion::math::Vec3f { 0.f,0.f,0.2f });
            
            // Set model matrix for rendering.
            float mtx[16];
            bx::mtxSRT(mtx, 0.1f, 0.1f, 0.2f, 0.f, -RotationInXY(movedCar.Rear.Orientation).Value(), 0.f, centerPos[0], 0, centerPos[2]);
            bgfx::setTransform(mtx);

            // Set vertex and index buffer.
            bgfx::setVertexBuffer(0, vbCube);
            bgfx::setIndexBuffer(ibh);

            // Set render states.
            bgfx::setState(state);

            // Submit primitive for rendering to view 0.
            bgfx::submit(0, m_program);

            // check whether we are at the end of the path
            if(EuclideanDistance(movedCar.Rear.Pos, pathGenerator.Finish) < 0.02f) {                
                // set the moved car along the path as the new start state
                // and generate a new path
                pathGenerator.GenerateRandomPath(movedCar);

                // pause the car for a while and then continue
                time = -0.2f;
            }
        }

        {
            // draw the path as a series of lines
            DebugDrawEncoder dde;
            dde.begin(0);

            dde.push();
            
            for(int n = 0; n < 21; ++n) {                
                auto movedCar = TraversePathNormalized((1.0f / 20.0f) * n, pathGenerator.Path, pathGenerator.Start);
                const auto& p = movedCar.Rear.Pos;
                if (movedCar.InReverse) {
                    dde.setColor(0xff0000aa + (n * 2));
                } else {
                    dde.setColor(0xff00aa00 + ((n * 2) << 8));
                }
                if(n == 0 ) {
                    dde.moveTo(p[0], 0.02f, p[2]);
                }
                else {
                    dde.lineTo(p[0], 0.02f, p[2]);
                }
            }         
            dde.pop();            

            dde.end();
        }

        // advance to next frame. Process submitted rendering primitives.
        bgfx::frame();

        time += 0.02f;
    }

    bgfx::shutdown();
    glfwTerminate();

    return 0;
}