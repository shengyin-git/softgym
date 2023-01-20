
class SoftgymSimpMesh : public Scene
{
public:

    SoftgymSimpMesh(const char* name) : Scene(name) {}

    float cam_x;
    float cam_y;
    float cam_z;
    float cam_angle_x;
    float cam_angle_y;
    float cam_angle_z;
    int cam_width;
    int cam_height;	

    // params ordering: node set, edge set, (!!!!!!!possibly particle radius!!!!!!!!!), stretch, bend, shear
    // render_type, cam_X, cam_y, cam_z, angle_x, angle_y, angle_z, width, height
    
    void Initialize(py::array_t<float> scene_params, int thread_idx = 0)
    {

        auto ptr = (float *) scene_params.request().ptr;
        float rest_length = ptr[0];

        // cloth
        float stretchStiffness = ptr[1]; //0.9f;
		float bendStiffness = ptr[2]; //1.0f;
		float shearStiffness = ptr[3]; //0.9f;

        float mass = float(ptr[4]);	// avg bath towel is 500-700g; the mass here is the total mass 

        cam_x = ptr[5];
        cam_y = ptr[6];
        cam_z = ptr[7];
        cam_angle_x = ptr[8];
        cam_angle_y = ptr[9];
        cam_angle_z = ptr[10];
        cam_width = int(ptr[11]);
        cam_height = int(ptr[12]);

        int num_node = ptr[13];
        int num_edge = ptr[14];

        int group = 0;

        auto node_ptr = (float *) ptr + 15; //node_set.request().ptr;
        auto edge_ptr = (float *) ptr + 15+num_node*3; //edge_set.request().ptr;

        Rope r;
        // here the particle size means the rest distance, which is smaller than the contact distance
        CreateSimpMesh(r, num_node, node_ptr, num_edge, edge_ptr, stretchStiffness, bendStiffness, shearStiffness, rest_length, 
            NvFlexMakePhase(group++, eNvFlexPhaseSelfCollide | eNvFlexPhaseSelfCollideFilter), mass);

        g_ropes.push_back(r);   

        g_params.radius = rest_length*2;
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