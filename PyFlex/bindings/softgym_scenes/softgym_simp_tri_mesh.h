
class SoftgymSimpTriMesh : public Scene
{
public:

    SoftgymSimpTriMesh(const char* name) : Scene(name) {}

    float cam_x;
    float cam_y;
    float cam_z;
    float cam_angle_x;
    float cam_angle_y;
    float cam_angle_z;
    int cam_width;
    int cam_height;	

    // params ordering:    
    void Initialize(py::array_t<float> scene_params, int thread_idx = 0)
    {
        auto ptr = (float *) scene_params.request().ptr;
        int num_nodes = ptr[0];
        int num_springs = ptr[1];
        float rest_length = ptr[2];

        cam_x = ptr[3];
        cam_y = ptr[4];
        cam_z = ptr[5];
        cam_angle_x = ptr[6];
        cam_angle_y = ptr[7];
        cam_angle_z = ptr[8];
        cam_width = int(ptr[9]);
        cam_height = int(ptr[10]);

        int group = 0;

        auto node_ptr = (float *) ptr + 11; 
        auto spring_ptr = (float *) ptr + 11 + num_nodes * 4; 

        Rope r;
        // here the particle size means the rest distance, which is smaller than the contact distance
        CreateSimpTriMesh(r, num_nodes, node_ptr, num_springs, spring_ptr, NvFlexMakePhase(group++, eNvFlexPhaseSelfCollide | eNvFlexPhaseSelfCollideFilter));

        g_ropes.push_back(r);   

        g_params.radius = rest_length; //rest_length*2;
		g_params.numIterations = 4;
		g_params.dynamicFriction = 1.0f;
		// g_params.staticFriction = 0.8f;
		g_params.collisionDistance = 0.001f;
		
		g_maxDiffuseParticles = 64*1024;
		g_diffuseScale = 0.25f;		
		g_diffuseShadow = false;
		g_diffuseColor = 2.5f;
		g_diffuseMotionScale = 1.5f;
		g_params.diffuseThreshold *= 0.01f;
		g_params.diffuseBallistic = 35;

		g_windStrength = 0.0f;
		g_windFrequency = 0.0f;

		g_numSubsteps = 2;

		// draw options		
		g_drawEllipsoids = false;
		g_drawPoints = false;
		g_drawDiffuse = false;
		g_drawSprings = 0;

		g_ropeScale = 0.5f;
		g_warmup = false;

        // return(node_idx);
    }

    virtual void CenterCamera(void)
    {
        g_camPos = Vec3(cam_x, cam_y, cam_z);
        g_camAngle = Vec3(cam_angle_x, cam_angle_y, cam_angle_z);
        g_screenHeight = cam_height;
        g_screenWidth = cam_width;
    }
};