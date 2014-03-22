#include <irrlicht/irrlicht.h>
#include <bullet/btBulletCollisionCommon.h>
#include <bullet/btBulletDynamicsCommon.h>
#include <tokamak/tokamak.h>
#include <time.h>
#include <cstdlib>


// linking with lib and disable console window





#ifdef _IRR_WINDOWS_
#define _CRT_SECURE_NO_WARNINGS
#pragma comment(lib, "Irrlicht.lib")
#pragma comment(lib, "BulletCollision.lib")
#pragma comment(lib, "BulletDynamics.lib")
#pragma comment(lib, "LinearMath.lib")
#pragma comment(lib, "tokamak.lib")
#pragma comment(lib, "tokamak_d.lib")

#pragma comment(linker, "/subsystem:windows /ENTRY:mainCRTStartup")
#endif






// Functions
static void CreateStartScene();
static void CreateBox(const btVector3 &TPosition, const irr::core::vector3df &TScale, btScalar TMass);
static void UpdatePhysics(irr::u32 TDeltaTime);
static void UpdateRender(btRigidBody *TObject);

// Global
static bool Done = false;
static btDiscreteDynamicsWorld *World;
static irr::IrrlichtDevice *irrDevice;
static irr::video::IVideoDriver *irrDriver;
static irr::scene::ISceneManager *irrScene;
static irr::gui::IGUIEnvironment *irrGUI;
static irr::ITimer *irrTimer;
static irr::core::list<btRigidBody *> Objects;






//Bullet
class EventReceiverClass : public irr::IEventReceiver  {

public:

	virtual bool OnEvent(const irr::SEvent &TEvent) {

		if(TEvent.EventType == irr::EET_KEY_INPUT_EVENT && !TEvent.KeyInput.PressedDown) {
			switch(TEvent.KeyInput.Key) {
				case irr::KEY_ESCAPE:
					Done = true;
				break;
				default:
					return false;
				break;
			}

			return true;
		}

		return false;
	}
};


/*
int main() {

	// Initialize irrlicht
	EventReceiverClass Receiver;
	irrDevice = irr::createDevice(irr::video::EDT_OPENGL, irr::core::dimension2d<irr::u32>(800, 600),32, false, false, false,&Receiver);
	irrGUI = irrDevice->getGUIEnvironment();
	irrTimer = irrDevice->getTimer();
	irrScene = irrDevice->getSceneManager();
	irrDriver = irrDevice->getVideoDriver();
	irrDevice->getCursorControl()->setVisible(0);
	
	// Initialize bullet
	btDefaultCollisionConfiguration *CollisionConfiguration = new btDefaultCollisionConfiguration();
	btBroadphaseInterface *BroadPhase = new btAxisSweep3(btVector3(-1000, -1000, -1000), btVector3(1000, 1000, 1000));
	btCollisionDispatcher *Dispatcher = new btCollisionDispatcher(CollisionConfiguration);
	btSequentialImpulseConstraintSolver *Solver = new btSequentialImpulseConstraintSolver();
	World = new btDiscreteDynamicsWorld(Dispatcher, BroadPhase, Solver, CollisionConfiguration);
	
	// Add camera
	irr::scene::ICameraSceneNode *Camera = irrScene->addCameraSceneNodeFPS(0, 100, 10);
	Camera->setPosition(irr::core::vector3df(2.0f, 5, 5.0f));
	Camera->setTarget(irr::core::vector3df(0,0,0));

	
	// Create the initial scene
	irrScene->addLightSceneNode(0, irr::core::vector3df(2, 5, -2), irr::video::SColorf(4, 4, 4, 1));
	CreateStartScene();
	

	// Main loop
	irr::u32 TimeStamp = irrTimer->getTime(), DeltaTime = 0;
	while(!Done) {

		DeltaTime = irrTimer->getTime() - TimeStamp;
		TimeStamp = irrTimer->getTime();

		UpdatePhysics(DeltaTime);

		irrDriver->beginScene(true, true, irr::video::SColor(255, 20, 0, 0));
		irrScene->drawAll();
		irrGUI->drawAll();
		irrDriver->endScene();
		irrDevice->run();
	}

	
	delete World;
	delete Solver;
	delete Dispatcher;
	delete BroadPhase;
	delete CollisionConfiguration;
	
	irrDevice->drop();

	return 0;
}

// Runs the physics simulation.
// - TDeltaTime tells the simulation how much time has passed since the last frame so the simulation can run independently of the frame rate.
void UpdatePhysics(irr::u32 TDeltaTime){

	World->stepSimulation(TDeltaTime * 0.001f, 60);

	// Relay the object's orientation to irrlicht
	for(irr::core::list<btRigidBody *>::Iterator Iterator = Objects.begin(); Iterator != Objects.end(); ++Iterator) {

		UpdateRender(*Iterator);
	}	
}

// Creates a base box
void CreateStartScene() {

	//Floor
	CreateBox(btVector3(0.0f, 0.0f, 0.0f), irr::core::vector3df(10.0f, 0.5f, 10.0f), 0.0f);
	//Cube
	CreateBox(btVector3(0.00f, 10, 0.00f), irr::core::vector3df(1.0f, 1.0f, 1.0f), 100);
}

// Create a box rigid body
void CreateBox(const btVector3 &TPosition, const irr::core::vector3df &TScale, btScalar TMass) {
	
	irr::scene::ISceneNode *Node = irrScene->addCubeSceneNode(1.0f);
	Node->setScale(TScale);
	
	// Set the initial position of the object
	btTransform Transform;
	Transform.setIdentity();
	Transform.setOrigin(TPosition);
	
	btDefaultMotionState *MotionState = new btDefaultMotionState(Transform);

	// Create the shape
	btVector3 HalfExtents(TScale.X * 0.5f, TScale.Y * 0.5f, TScale.Z * 0.5f);
	btCollisionShape *Shape = new btBoxShape(HalfExtents);
	
	// Add mass
	btVector3 LocalInertia;
	Shape->calculateLocalInertia(TMass, LocalInertia);
	
	// Create the rigid body object
	//btRigidBody *RigidBody = new btRigidBody(TMass, MotionState, Shape, LocalInertia);

	btRigidBody::btRigidBodyConstructionInfo rbInfo(TMass, MotionState, Shape, LocalInertia);
	rbInfo.m_restitution=0.6;	
	btRigidBody* RigidBody = new btRigidBody(rbInfo);

	// Store a pointer to the irrlicht node so we can update it later
	RigidBody->setUserPointer((void *)(Node));

	// Add it to the world
	World->addRigidBody(RigidBody);
	Objects.push_back(RigidBody);
}

void QuaternionToEuler(const btQuaternion &TQuat, btVector3 &TEuler) {
	btScalar W = TQuat.getW();
	btScalar X = TQuat.getX();
	btScalar Y = TQuat.getY();
	btScalar Z = TQuat.getZ();
	float WSquared = W * W;
	float XSquared = X * X;
	float YSquared = Y * Y;
	float ZSquared = Z * Z;

	TEuler.setX(atan2f(2.0f * (Y * Z + X * W), -XSquared - YSquared + ZSquared + WSquared));
	TEuler.setY(asinf(-2.0f * (X * Z - Y * W)));
	TEuler.setZ(atan2f(2.0f * (X * Y + Z * W), XSquared - YSquared - ZSquared + WSquared));
	TEuler *= irr::core::RADTODEG;
}

// Passes bullet's orientation to irrlicht
void UpdateRender(btRigidBody *TObject) {
	irr::scene::ISceneNode *Node = static_cast<irr::scene::ISceneNode *>(TObject->getUserPointer());

	// Set position
	btVector3 Point = TObject->getCenterOfMassPosition();
	Node->setPosition(irr::core::vector3df((irr::f32)Point[0], (irr::f32)Point[1], (irr::f32)Point[2]));

	// Set rotation
	btVector3 EulerRotation;
	QuaternionToEuler(TObject->getOrientation(), EulerRotation);
	Node->setRotation(irr::core::vector3df(EulerRotation[0], EulerRotation[1], EulerRotation[2]));
	
	
}

*/








//Tokamak
#define PI 3.1415926
#define CUBECOUNT 1
#define CUBEX 5.0
#define CUBEY 5.0
#define CUBEZ 5.0
#define CUBEMASS 50000000.0f
#define FLOORSIZE 100
 
//irr::scene::ICameraSceneNode* camera = 0;
 
neSimulator *gSim = NULL;
 
neRigidBody *gCubes[CUBECOUNT];
neAnimatedBody *gFloor = NULL;
 
bool gbUseHFTimer;
int gCurrentTime;
float gfTimeScale;
 

/*
class MyEventReceiver : public irr::IEventReceiver
{
public:
    virtual bool OnEvent(const irr::SEvent& event)
    {
        if (camera)
            return camera->OnEvent(event);
 
        return false;
    }
};
 
 */

class PhysicsCubeNode: public irr::scene::ISceneNode
{
    irr::core::aabbox3d<irr::f32> Box;
    irr::video::S3DVertex Vertices[8];
    irr::video::SMaterial Material;
 
public:
 
    PhysicsCubeNode(irr::scene::ISceneNode* parent,
                    irr::scene::ISceneManager* mgr, irr::u32 id)
        : irr::scene::ISceneNode(parent, mgr, id)
    {
        Material.Wireframe = false;
        Material.Lighting = false;
 
        Vertices[0] = irr::video::S3DVertex(
                          -CUBEX/2,-CUBEY / 2,-CUBEZ/2, 1,1,0,
                          irr::video::SColor(255,0,255,255), 0, 1);
        Vertices[1] = irr::video::S3DVertex(
                          CUBEX/2,-CUBEY / 2,-CUBEZ/2, 1,0,0,
                          irr::video::SColor(255,255,0,255), 1, 1);
        Vertices[2] = irr::video::S3DVertex(
                          CUBEX/2,-CUBEY / 2, CUBEZ/2, 0,1,1,
                          irr::video::SColor(255,255,255,0), 1, 0);
        Vertices[3] = irr::video::S3DVertex(
                          -CUBEX/2,-CUBEY / 2, CUBEZ/2, 0,0,1,
                          irr::video::SColor(255,0,255,0), 0, 0);
        Vertices[4] = irr::video::S3DVertex(
                          -CUBEX/2, CUBEY / 2,-CUBEZ/2, 0,0,1,
                          irr::video::SColor(255,0,0,255), 0, 0);
        Vertices[5] = irr::video::S3DVertex(
                          CUBEX/2, CUBEY / 2,-CUBEZ/2, 0,0,1,
                          irr::video::SColor(255,255,0,0), 0, 0);
        Vertices[6] = irr::video::S3DVertex(
                          CUBEX/2, CUBEY / 2, CUBEZ/2, 0,0,1,
                          irr::video::SColor(255,0,0,0), 0, 0);
        Vertices[7] = irr::video::S3DVertex(
                          -CUBEX/2, CUBEY / 2, CUBEZ/2, 0,0,1,
                          irr::video::SColor(255,255,255,255), 0, 0);
 
        Box.reset(Vertices[0].Pos);
        for (s32 i=1; i<8; ++i)
            Box.addInternalPoint(Vertices[i].Pos);
    }
 
 
    virtual void OnRegisterSceneNode()
    {
        if (IsVisible)
            SceneManager->registerNodeForRendering(this);
 
        ISceneNode::OnRegisterSceneNode();
    }
 
    virtual void render()
    {
        irr::u16 indices[] = {0,1,2,
                              0,2,3,
                              1,6,2,
                              1,5,6,
                              0,5,1,
                              0,4,5,
                              0,3,7,
                              0,7,4,
                              2,7,3,
                              2,6,7,
                              4,6,5,
                              4,7,6
                             };
 
        irr::video::IVideoDriver* driver = SceneManager->getVideoDriver();
 
        driver->setMaterial(Material);
        driver->setTransform(irr::video::ETS_WORLD, AbsoluteTransformation);
        driver->drawIndexedTriangleList(&Vertices[0], 8, &indices[0], 12);
    }
 
    virtual const irr::core::aabbox3d<f32>& getBoundingBox() const
    {
        return Box;
    }
 
    virtual s32 getMaterialCount()
    {
        return 1;
    }
 
    virtual irr::video::SMaterial& getMaterial(s32 i)
    {
        return Material;
    }
};
 
 
class PhysicsFloorNode: public irr::scene::ISceneNode
{
private:
    irr::core::aabbox3d<irr::f32> Box;
    irr::video::S3DVertex Vertices[4];
    irr::video::SMaterial Material;
 
public:
 
    PhysicsFloorNode(irr::scene::ISceneNode* parent,
                     irr::scene::ISceneManager* mgr, irr::s32 id)
        : irr::scene::ISceneNode(parent, mgr, id)
    {
        Material.Wireframe = false;
        Material.Lighting = false;
 
        Vertices[0] = irr::video::S3DVertex(
                          -FLOORSIZE/2,0.0,-FLOORSIZE/2, 1,1,0,
                          irr::video::SColor(255,0,0,0), 0, 1);
        Vertices[1] = irr::video::S3DVertex(
                          FLOORSIZE/2,0.0,-FLOORSIZE/2, 1,0,0,
                          irr::video::SColor(255,0,0,0), 1, 1);
        Vertices[2] = irr::video::S3DVertex(
                          FLOORSIZE/2,0.0, FLOORSIZE/2, 0,1,1,
                          irr::video::SColor(255,0,0,0), 1, 0);
        Vertices[3] = irr::video::S3DVertex(
                          -FLOORSIZE/2,0.0, FLOORSIZE/2, 0,0,1,
                          irr::video::SColor(255,0,0,0), 0, 0);
 
        Box.reset(Vertices[0].Pos);
        for (s32 i=1; i<4; ++i)
            Box.addInternalPoint(Vertices[i].Pos);
    }
 
    virtual void OnRegisterSceneNode()
    {
        if (IsVisible)
            SceneManager->registerNodeForRendering(this);
 
        ISceneNode::OnRegisterSceneNode();
    }
 
    virtual void render()
    {
        irr::u16 indices[] = {0,1,2,
                              0,2,3,
                              0,2,1,
                              0,3,2
                             };
 
        irr::video::IVideoDriver* driver = SceneManager->getVideoDriver();
 
        driver->setMaterial(Material);
        driver->setTransform(irr::video::ETS_WORLD, AbsoluteTransformation);
        driver->drawIndexedTriangleList(&Vertices[0], 4, &indices[0], 4);
    }
 
    virtual const irr::core::aabbox3d<f32>& getBoundingBox() const
    {
        return Box;
    }
 
    virtual s32 getMaterialCount()
    {
        return 1;
    }
 
    virtual irr::video::SMaterial& getMaterial(s32 i)
    {
        return Material;
    }
};
 
bool InitPhysics(void)
{
    neGeometry *geom; // Pointer to a Geometry object
    // which we'll use to define the shape/size
    // of each cube
    neV3 boxSize1;    // A variable to store the length, width
    // and height of the cube
    neV3 gravity;     // A vector to store the direction and
    //intensity of gravity
    neV3 pos;         // The position of a cube
    f32 mass;         // The mass of our cubes
    neSimulatorSizeInfo sizeInfo; // SizeInfo stores data
    // about how manyobjects we are going to model
    int i;
 
    sizeInfo.rigidBodiesCount = CUBECOUNT;
 
    sizeInfo.animatedBodiesCount = 1;
 
    s32 totalBody = sizeInfo.rigidBodiesCount +
                    sizeInfo.animatedBodiesCount;
    sizeInfo.geometriesCount = totalBody;
 
    sizeInfo.overlappedPairsCount =
        totalBody * (totalBody - 1) / 2;
 
    sizeInfo.rigidParticleCount = 0;
    sizeInfo.constraintsCount = 0;
    sizeInfo.terrainNodesStartCount = 0;
 
    gravity.Set(0.0f, -10.0f, 0.0f);
 
    gSim = neSimulator::CreateSimulator(sizeInfo, NULL, &gravity);
 
    srand((unsigned int)time(NULL));
 
    for (i=0; i<CUBECOUNT; i++)
    {
        // Create a rigid body
        gCubes[i] = gSim->CreateRigidBody();
 
        // Add geometry to the body and set it to be a box
        // of dimensions 1, 1, 1
        geom = gCubes[i]->AddGeometry();
        boxSize1.Set(CUBEX, CUBEY, CUBEZ);
        geom->SetBoxSize(boxSize1[0], boxSize1[1], boxSize1[2]);
 
        // Update the bounding info of the object -- must always call this
        // after changing a body's geometry.
        gCubes[i]->UpdateBoundingInfo();
 
        // Set other properties of the object (mass, position, etc.)
        mass =CUBEMASS;
        gCubes[i]->SetInertiaTensor(
            neBoxInertiaTensor(boxSize1[0], boxSize1[1], boxSize1[2], mass));
        gCubes[i]->SetMass(mass);
 
        // Vary the position so the cubes don't all exactly
        // stack on top of each other
        // (makes for a more interesting simulation)
 
        // MC - You may need to play with the randomization
        // stuff, since your cubemay be to big to
        // be easily destabelized
        //pos.Set((irr::f32)((rand()%10) / 20.0f * CUBEX, 4.0f + i * (CUBEY + 1),
                //(rand()%10)/ 20.0f * CUBEZ));
		pos.Set(0,20,0);
        gCubes[i]->SetPos(pos);
    }
 
    gFloor = gSim->CreateAnimatedBody();
 
    geom = gFloor->AddGeometry();
    boxSize1.Set(FLOORSIZE, 0.0f, FLOORSIZE);
    geom->SetBoxSize(boxSize1[0],boxSize1[1],boxSize1[2]);
    gFloor->UpdateBoundingInfo();
 
    pos.Set(0.0f, 0.0f, 0.0f);
    gFloor->SetPos(pos);
// All done
    return true;
}
 
void KillPhysics(void)
{
    if (gSim)
    {
        // Destroy the simulator.
        // Note that this will release all related
        // resources that we've allocated.
        neSimulator::DestroySimulator(gSim);
        gSim = NULL;
    }
}
 
irr::ITimer* TIMER = 0;
bool InitTimer(void)
{
    gfTimeScale = 0.001f;
    gCurrentTime =TIMER->getTime();
    gbUseHFTimer = false;
    return true;
}
 
float GetElapsedTime()
{
    int newTime;
    float fElapsed;
 
    newTime=TIMER->getTime();
 
    // Scale accordingly
    int diff = newTime - gCurrentTime;
    fElapsed = (float)(diff * gfTimeScale);
 
    // Save the new time value for the next time we're called
    gCurrentTime = newTime;
 
    return fElapsed;
}
 
int main()
{
    //irr - variable

	EventReceiverClass Receiver;
	irrDevice = irr::createDevice(irr::video::EDT_OPENGL, irr::core::dimension2d<irr::u32>(800, 600),32, false, false, false,&Receiver);
	irrGUI = irrDevice->getGUIEnvironment();
	irrTimer = irrDevice->getTimer();
	irrScene = irrDevice->getSceneManager();
	irrDriver = irrDevice->getVideoDriver();
	irrDevice->getCursorControl()->setVisible(0);

	irr::scene::ICameraSceneNode *camera = irrScene->addCameraSceneNodeFPS(0, 100, 10);
	camera->setPosition(irr::core::vector3df(20,30,20));
	camera->setTarget(irr::core::vector3df(-20,-30,-20));

	//EventReceiverClass Receiver;
    //tok - timer stuff
    float fElapsed;
    static float fLastElapsed;
    int i;
 
    //MC - I needed a vector3df to pass to the Node movement functions
    irr::core::vector3df TempVect;
 

	/*
    irr::IrrlichtDevice *device =
        irr::createDevice(irr::video::EDT_OPENGL,
                          irr::core::dimension2d<irr::u32>(800, 600), 32,
                          false, false,NULL);
 
 */
    //TIMER = device->getTimer();

	TIMER = irrDevice ->getTimer();
 
  // irr::video::IVideoDriver* driver = device->getVideoDriver();
    //irr::scene::ISceneManager* smgr = device->getSceneManager();
 
 
    //MC - I moved the camera so the initial view is more interesting
    //     then turned it toward the origin, there the cubes fall
	/*
	camera = smgr->addCameraSceneNodeFPS(0, 100.0f, 0.5f);
    camera->setPosition(irr::core::vector3df(20,30,20));
    camera->setTarget(irr::core::vector3df(-20,-30,-20));
 */
    //device->getCursorControl()->setVisible(false);
 
 
//MC - This is a major step, in case it's not obvious.
// Initialize Tokamak
    InitPhysics();
 
//MC - Here we set up our array of cubes then create them.
// Make sure you DO NOT drop() the cubes, since we need to
// use the pointer to refer back to the
// cubes to position them
    PhysicsCubeNode *CubeNode[CUBECOUNT];
 
    for (i=0; i<CUBECOUNT; i++)
    {
        CubeNode[i] = new PhysicsCubeNode(
            irrScene->getRootSceneNode(), irrScene, 666);
    }
 
//MC - I'm one of those programmers that will fit 2
// or more commands on one line if I can.  Plus, I
// think this is a really cute way of calling and
// dropping a node without using a variable. I use
// the magic of C++ to create a new Node and drop
// that same node in the same look.  If I was doing
// this the long way I would need a PhysicsFloorNode
// pointer
    (new PhysicsFloorNode(irrScene->getRootSceneNode(),
                          irrScene, 666))->drop();
 
//tok - Initialize the timer and the variable to
// count the last number of milliseconds.
//      this makes sure there isn't a wierd jump in the program.
    InitTimer();
    fLastElapsed = 0;
 
//irr - This will give us something to reference the last FPS count
    int lastFPS = -1;
 
    while(irrDevice->run())
    {
 
        //tok - Find out how much time has elapsed
        //      since we were last called
        fElapsed = GetElapsedTime();
 
        if (fLastElapsed+fElapsed > 0.001)
        {
            if (fLastElapsed+fElapsed < 0.01)//fix the time step
                gSim->Advance(fLastElapsed+fElapsed);
            else
                gSim->Advance((irr::f32)0.01);
            fLastElapsed = 0;
        }
        else
            fLastElapsed = fLastElapsed+fElapsed;
 
        //MC - here where the real magic happens.
        //     We loop through each cube;
        for (i=0; i<CUBECOUNT; i++)
        {
 
            //First, we get the position from the Tokamak cube
 
            neV3 p = gCubes[i]->GetPos();
 
            //Now, we set up out temporary vector
            TempVect.X = p[0];
            TempVect.Y = p[1];
            TempVect.Z = p[2];
 
            //And set the position of the cube that Irrlicht is going to draw
            CubeNode[i]->setPosition(TempVect);
 
            //Last, we get the rotation from the Tokamak cube
            // This is a the rotation quaternion, I believe
            // More information is found here:
            // http://www.gamedev.net/reference/articl ... le1095.asp
            // (Thanks to unnamed forum user)
            neQ q = gCubes[i]->GetRotationQ();
 
            //Again, the temporary vector is set
            TempVect.Y = q.Y;
            TempVect.X = q.X;
            TempVect.Z = q.Z;
 
            //we convert it to degrees and set the Irrlicht cube's rotation
            TempVect *= ((irr::f32)(180 / PI));
            CubeNode[i]->setRotation(TempVect);
        }
 
        //irr - Now we draw it all
        irrDriver->beginScene(true, true, irr::video::SColor(0,100,100,100));
 
        irrScene->drawAll();
 
        irrDriver->endScene();
 
        int fps = irrDriver->getFPS();
 
        if (lastFPS != fps)
        {
            wchar_t tmp[1024];
            swprintf(tmp, 1024, L"Physics Example- Irrlicht & Tokamak"\
                     L" Engines(fps:%d)", fps);
 
            irrDevice->setWindowCaption(tmp);
            lastFPS = fps;
        }
    }
 
 
//tok - When we're one, we kill the physics engine
    KillPhysics();
 
//MC - And drop all the node pointers that we don't need anymore
    for (i=0; i<CUBECOUNT; i++)
        CubeNode[i]->drop();
 
//irr - and we finish off everything by dropping the device
    irrDevice->drop();
 
    return 0;
}

