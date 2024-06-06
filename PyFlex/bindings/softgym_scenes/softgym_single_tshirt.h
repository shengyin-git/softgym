#pragma once
#include <iostream>
#include <vector>

class SoftgymSingleTshirt : public Scene
{
public:
    float cam_x;
    float cam_y;
    float cam_z;
    float cam_angle_x;
    float cam_angle_y;
    float cam_angle_z;
    int cam_width;
    int cam_height;

	SoftgymSingleTshirt(const char* name) : Scene(name) {}

    float get_param_float(py::array_t<float> scene_params, int idx)
    {
        auto ptr = (float *) scene_params.request().ptr;
        float out = ptr[idx];
        return out;
    }

    //params ordering: xpos, ypos, zpos, xsize, zsize, stretch, bend, shear
    // render_type, cam_X, cam_y, cam_z, angle_x, angle_y, angle_z, width, height
	void Initialize(py::array_t<float> scene_params, int thread_idx=0)
    {
        auto ptr = (float *) scene_params.request().ptr;

        int render_type = (int)ptr[0];

        cam_x = ptr[1];
        cam_y = ptr[2];
        cam_z = ptr[3];
        cam_angle_x = ptr[4];
        cam_angle_y = ptr[5];
        cam_angle_z = ptr[6];
        cam_width = int(ptr[7]);
        cam_height = int(ptr[8]);

        int num_points = (int)ptr[9];
		int num_triangles = (int)ptr[10];
        int num_springs = (int)ptr[11];

        auto points_ptr = (float *) ptr + 12; 
        auto triangles_ptr = (float *) ptr + 12 + num_points * 4; 
        auto springs_ptr = (float *) ptr + 12 + num_points * 4 + num_triangles * 3; 

		float radius = 0.00625f;

        // Cloth
		int phase = NvFlexMakePhase(0, eNvFlexPhaseSelfCollide | eNvFlexPhaseSelfCollideFilter);
	    CreateSingleTshirt(phase, num_points, points_ptr, num_triangles, triangles_ptr, num_springs, springs_ptr);

		g_numSubsteps = 4;
		g_params.numIterations = 30;

		g_params.dynamicFriction = 0.75f;
		g_params.particleFriction = 1.0f;
		g_params.damping = 1.0f;
		g_params.sleepThreshold = 0.02f;

		g_params.relaxationFactor = 1.0f;
		g_params.shapeCollisionMargin = 0.04f;

		g_sceneLower = Vec3(-1.0f);
		g_sceneUpper = Vec3(1.0f);
		g_drawPoints = true;
        g_drawEllipsoids = false;
        g_drawDiffuse = false;
        g_drawCloth = false;
        g_drawRopes = false;
        g_drawMesh = false;

        g_params.radius = radius; //*1.8f;
        g_params.collisionDistance = 0.005f;

        // g_drawPoints = render_type & 1;
        // g_drawCloth = (render_type & 2) >>1;
        // g_drawSprings = false;


        // table
//        NvFlexRigidShape table;
        // Half x, y, z
//        NvFlexMakeRigidBoxShape(&table, -1, 0.27f, 0.55f, 0.3f, NvFlexMakeRigidPose(Vec3(-0.04f, 0.0f, 0.0f), Quat()));
//        table.filter = 0;
//        table.material.friction = 0.95f;
//		table.user = UnionCast<void*>(AddRenderMaterial(Vec3(0.35f, 0.45f, 0.65f)));

//        float density = 1000.0f;
//        NvFlexRigidBody body;
//		NvFlexMakeRigidBody(g_flexLib, &body, Vec3(1.0f, 1.0f, 0.0f), Quat(), &table, &density, 1);
//
//        g_buffers->rigidShapes.push_back(table);
//        g_buffers->rigidBodies.push_back(body);

        // Box object
//        float scaleBox = 0.05f;
//        float densityBox = 2000000000.0f;

//        Mesh* boxMesh = ImportMesh(make_path(boxMeshPath, "/data/box.ply"));
//        boxMesh->Transform(ScaleMatrix(scaleBox));
//
//        NvFlexTriangleMeshId boxId = CreateTriangleMesh(boxMesh, 0.00125f);
//
//        NvFlexRigidShape box;
//        NvFlexMakeRigidTriangleMeshShape(&box, g_buffers->rigidBodies.size(), boxId, NvFlexMakeRigidPose(0, 0), 1.0f, 1.0f, 1.0f);
//        box.filter = 0x0;
//        box.material.friction = 1.0f;
//        box.material.torsionFriction = 0.1;
//        box.material.rollingFriction = 0.0f;
//        box.thickness = 0.00125f;
//
//        NvFlexRigidBody boxBody;
//        NvFlexMakeRigidBody(g_flexLib, &boxBody, Vec3(0.21f, 0.7f, -0.1375f), Quat(), &box, &density, 1);
//
//        g_buffers->rigidBodies.push_back(boxBody);
//        g_buffers->rigidShapes.push_back(box);

//        g_params.numPostCollisionIterations = 15;

    }

    virtual void CenterCamera(void)
    {
        g_camPos = Vec3(cam_x, cam_y, cam_z);
        g_camAngle = Vec3(cam_angle_x, cam_angle_y, cam_angle_z);
        g_screenHeight = cam_height;
        g_screenWidth = cam_width;
    }
};