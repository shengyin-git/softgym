#pragma once
#include <iostream>
#include <vector>

// inline void swap(int &a, int &b) {int tmp =a ; a = b; b=tmp;}

class Softgym3dCloth : public Scene
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

	Softgym3dCloth(const char* name) : Scene(name) {}

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
	    int render_type = ptr[0]; // 0: only points, 1: only mesh, 2: points + mesh    

        cam_x = ptr[1];
        cam_y = ptr[2];
        cam_z = ptr[3];
        cam_angle_x = ptr[4];
        cam_angle_y = ptr[5];
        cam_angle_z = ptr[6];
        cam_width = int(ptr[7]);
        cam_height = int(ptr[8]);

        // Cloth
		float radius = ptr[9];  
        float stretchStiffness = ptr[10]; //0.9f;
		float bendStiffness = ptr[11]; //1.0f;
		float shearStiffness = ptr[12]; //0.9f;

        int flip_mesh = int(ptr[13]); // Flip half
        int vertice_num = int(ptr[14]); 
        int face_num = int(ptr[15]);
        int stretch_num = int(ptr[16]);
        int bend_num = int(ptr[17]);
        int shear_num = int(ptr[18]);

		// build any cloth
		int phase = NvFlexMakePhase(0, eNvFlexPhaseSelfCollide | eNvFlexPhaseSelfCollideFilter);

		float x, y, z, inv_mass;
		int base = 19; 
		for (uint32_t i = 0; i < vertice_num; i++){
			x = float(ptr[base + i * 4]);
			y = float(ptr[base + i * 4 + 1]);
			z = float(ptr[base + i * 4 + 2]);
			inv_mass = float(ptr[base + i * 4 + 3]);
			// cout << "add point x: " << x << " y: " << y << " z: " << z << endl;
			g_buffers->positions.push_back(Vec4(x, y, z, inv_mass));
			g_buffers->velocities.push_back(Vec3(0, 0, 0));
			g_buffers->phases.push_back(phase);
		};

		base = base + vertice_num * 4;
		bool compute_normal=true;
		for (uint32_t i = 0; i < face_num; i++){
			g_buffers->triangles.push_back(ptr[base + i*3]);
			g_buffers->triangles.push_back(ptr[base + i*3+1]);
			g_buffers->triangles.push_back(ptr[base + i*3+2]);
			auto p1 = g_buffers->positions[ptr[base + i*3]];
			auto p2 = g_buffers->positions[ptr[base + i*3+1]];
			auto p3 = g_buffers->positions[ptr[base + i*3+2]];
			auto normal = Vec3(0, 0, 0);
			if (compute_normal) {
				auto U = p2 - p1;
				auto V = p3 - p1;
				normal = Vec3(
					U.y * V.z - U.z * V.y,
					U.z * V.x - U.x * V.z,
					U.x * V.y - U.y * V.x);
			} else { // face normal is already computed outside
				// cout << "using existing normal" << endl;
				normal = Vec3(
					ptr[base + i*3 + 3],
					ptr[base + i*3 + 4],
					ptr[base + i*3 + 5]);
			}
			g_buffers->triangleNormals.push_back(normal / Length(normal));
		};

		int sender, receiver;
		if (!compute_normal) {
			base = base + face_num*6;
		} else {
			base = base + face_num*3;
		}
		for (uint32_t i = 0; i < stretch_num; i++ ){
			sender = int(ptr[base + i * 2]);
			receiver = int(ptr[base + i * 2 + 1]);
			CreateSpring(sender, receiver, stretchStiffness); // assume no additional particles are added
			// cout << "create stretch spring sender: " << sender << " receiver: " << receiver << endl;
		}
		base = base + stretch_num*2;
		for (uint32_t i=0; i< bend_num; i++){
			sender = int(ptr[base + i * 2]);
			receiver = int(ptr[base + i * 2 + 1]);
			CreateSpring(sender, receiver, shearStiffness);
			// cout << "create shear spring sender: " << sender << " receiver: " << receiver << endl;
		}

		base += bend_num*2;
		for (uint32_t i = 0; i < shear_num; i++){
			sender = int(ptr[base + i * 2]);
			receiver = int(ptr[base + i * 2 + 1]);
			CreateSpring(sender, receiver, bendStiffness);
			// cout << "create bend spring sender: " << sender << " receiver: " << receiver << endl;
		}

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
		g_drawPoints = false;

        g_params.radius = radius*1.8f;
        g_params.collisionDistance = 0.005f;

        g_drawPoints = render_type & 1;
        g_drawCloth = (render_type & 2) >>1;
        g_drawSprings = false;
    }

    virtual void CenterCamera(void)
    {
        g_camPos = Vec3(cam_x, cam_y, cam_z);
        g_camAngle = Vec3(cam_angle_x, cam_angle_y, cam_angle_z);
        g_screenHeight = cam_height;
        g_screenWidth = cam_width;
    }
};