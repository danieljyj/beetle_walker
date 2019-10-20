//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#include "chrono/physics/ChSystem.h"                    
#include "chrono/physics/ChBodyEasy.h"       
#include "chrono/assets/ChTexture.h"
#include "chrono/assets/ChColorAsset.h"
#include "chrono_irrlicht/ChIrrApp.h"
#include<array>
#include "chrono_irrlicht/ChIrrAppInterface.h"

#include "chrono/core/ChRealtimeStep.h"               //包括的头文件一般这些是必须的，一开始可以直接复制，如果有需要额外添加。

#define COG_OF_LEG 0.2561     //The obj from 3Dbuilder has a cog different from the cog calculated by hand.It may because the 3dbuilder is too simple.
                             // I have modified the obj in 3dbuilder in advance ,by give it a offset in advance. 0.2561 is the real cog calculated 
                            // by easymesh.(from cog to the center of circle)
                           //因为我是用的Windows自带的3D建模工具产生的obj文件（beetle_leg_norm.obj）.这个软件比较简陋，算出的质心和我用手算出的不一样，和chronoengine算出的也不一样。
                           //我在3Dbuilder建模的时候考虑到了这一点，事先在模型中把真正的质心放在原点，而不是把模型默认的质心放在原点（需要好好摸索。。。当时搞了好几天,如果会3dmax什么的就好了。）
                          //0.2561是半圆环形的腿的质心与圆心的距离。在下面初始化腿的位置的时候这个参数很重要，当时也是调了很久。

//Use the namespace of Chrono 
using namespace chrono;
using namespace chrono::irrlicht;

// Use the main namespaces of Irrlicht
using namespace irr;                     
using namespace irr::core;
using namespace irr::scene;
using namespace irr::video;
using namespace irr::io;
using namespace irr::gui;                            //一般开头都是用这几个命名空间，如果有额外的需要再进行改动。


//double boxW(12);
//double boxL(50);
//double boxH(2);
//double boxDrop(2 + boxH);
//double chassisL(15.0);
//double particleDiameter(0.2);

class MyBeetle{                                     //我为自己的机器人单独写了一个类，这个做法可以参考demo_forklift.	                                                
public:                                             //其实这个程序一开始就是模仿forklift的，包括和后面的MyEventReceiver也是。
	std::shared_ptr<ChBodyEasyBox> trunk;
	std::shared_ptr<ChBodyEasyMesh> leglf;              //lf represents left-front.similarly,rm->right-medium rb->right-back
	std::shared_ptr<ChBodyEasyMesh> leglm;
	std::shared_ptr<ChBodyEasyMesh> leglb;
	std::shared_ptr<ChBodyEasyMesh> legrf;
	std::shared_ptr<ChBodyEasyMesh> legrm;
	std::shared_ptr<ChBodyEasyMesh> legrb;                  
	std::shared_ptr<ChLinkEngine> my_link_f;
	std::shared_ptr<ChLinkEngine> my_link_m;
	std::shared_ptr<ChLinkEngine> my_link_b;
	std::shared_ptr<ChBodyEasyCylinder> axlef;
	std::shared_ptr<ChBodyEasyCylinder> axlem;
	std::shared_ptr<ChBodyEasyCylinder> axleb;        //用一个引擎带动两条腿，这样做最大的好处是可以让两条腿同步。我试过给每条腿分配一个引擎，这样腿很容易因为微扰而逐渐步态打乱。
	std::shared_ptr<ChLinkLockLock> axlef_leglf;
	std::shared_ptr<ChLinkLockLock> axlef_legrf;
	std::shared_ptr<ChLinkLockLock> axlem_leglm;
	std::shared_ptr<ChLinkLockLock> axlem_legrm;
	std::shared_ptr<ChLinkLockLock> axleb_leglb;
	std::shared_ptr<ChLinkLockLock> axleb_legrb;        //以上是beetle的组成部分。  

	MyBeetle(ChIrrAppInterface* app, ChVector<> offset = ChVector<>(0, 0, 0)) {             //构造函数 construction function  这是整个模型的核心。
		// 1-Create a box trunk.
		trunk = std::make_shared<ChBodyEasyBox>(4, 0.2, 1.8, 10, true, true);             //详见ChBodyEasyBox的参考说明，学会看懂类并学会如何初始化
		trunk->SetPos(ChVector<>(0, 0.85, 0));                                             //设置位置

		auto ttexture = std::make_shared<ChTexture>(GetChronoDataFile("csrc.png"));         
		trunk->AddAsset(ttexture);                                                         //这两步是添加表面图案。
		app->GetSystem()->Add(trunk);                                   //正如前面所说的，每一个部件都要加入到ChSystem中，这就是加入的方法。声明每一个新的部件最后一步都是这个。
		                                                               //这里用了指针，所以写起来难懂，简单的先看demo_crank怎么做的


/*		auto material = std::make_shared<ChMaterialSurfaceDEM>();
		material->SetYoungModulus(6e4);
		material->SetFriction(0.3f);
		material->SetRestitution(0.2f);
		material->SetAdhesion(0);             */

		// 2-Create six legs.
		legrf = std::make_shared<ChBodyEasyMesh>(GetChronoDataFile("beetle_leg_norm.obj"),  // mesh .OBJ file   //圆形腿的构建没有EasyBox这样的现成类可以调用，但是可以导入外部的obj文件。
			                                     50,                                      // density
			                                     true,                                     // compute mass, inertia & COG from the mesh (must be a closed watertight mesh!)
			                                     true,                                    // enable collision with mesh
			                                     0.001,                                   // sphere swept inflate of mesh - improves robustness of collision detection
			                                     true);                                   // enable visualization of mesh
		legrf->SetFrame_REF_to_abs(ChFrame<>(ChVector<>(2 - COG_OF_LEG, 0.45, -1.1)));      //SetFrame_REF_to_abs是设置REF参考系在绝对参考系的坐标，要看懂这一步，需要看懂ChFrame和ChAuxRef这两个类的定义。
		                                                                                    //也可以在网站上看Reference manual。
		auto ltexture = std::make_shared<ChTexture>(GetChronoDataFile("coffee.jpg"));         
		legrf->AddAsset(ltexture);                                                          //添加表面图案，这个ltexture代表leg-texture的意思。
		app->GetSystem()->Add(legrf);
		//legrf->SetMaterialSurface(material);

		leglf = std::make_shared<ChBodyEasyMesh>(GetChronoDataFile("beetle_leg_norm.obj"),  
			                                     50,                            
			                                     true,   
			                                     true,   
			                                     0.001,  
			                                     true);  
		leglf->SetFrame_REF_to_abs(ChFrame<>(ChVector<>(2 + COG_OF_LEG, 1.25, 1.1)));
		leglf->SetRot(Q_from_AngZ(CH_C_PI));
		leglf->AddAsset(ltexture);
		app->GetSystem()->Add(leglf);
		//leglf->SetMaterialSurface(material);

		legrm = std::make_shared<ChBodyEasyMesh>(GetChronoDataFile("beetle_leg_norm.obj"),  
			                                     50,    
			                                     true,   
			                                     true,   
			                                     0.001,  
			                                     true);  
		legrm->SetFrame_REF_to_abs(ChFrame<>(ChVector<>(+COG_OF_LEG, 1.25, -1.1)));
		legrm->SetRot(Q_from_AngZ(CH_C_PI));
		legrm->AddAsset(ltexture);
		app->GetSystem()->Add(legrm);
		//legrm->SetMaterialSurface(material);

		leglm = std::make_shared<ChBodyEasyMesh>(GetChronoDataFile("beetle_leg_norm.obj"),  
			                                     50,   
			                                     true,  
		                                         true,  
			                                     0.001, 
			                                     true); 
		leglm->SetFrame_REF_to_abs(ChFrame<>(ChVector<>(-COG_OF_LEG, 0.45, 1.1)));
		leglm->AddAsset(ltexture);
		app->GetSystem()->Add(leglm);
		//leglm->SetMaterialSurface(material);

		legrb = std::make_shared<ChBodyEasyMesh>(GetChronoDataFile("beetle_leg_norm.obj"), 
			                                    50,   
			                                    true,  
			                                    true,  
			                                    0.001, 
			                                    true); 
		legrb->SetFrame_REF_to_abs(ChFrame<>(ChVector<>(-2 - COG_OF_LEG, 0.45, -1.1)));
		legrb->AddAsset(ltexture);
		app->GetSystem()->Add(legrb);

		leglb = std::make_shared<ChBodyEasyMesh>(GetChronoDataFile("beetle_leg_norm.obj"), 
			                                    50,   
			                                    true,  
			                                    true,  
			                                    0.001, 
			                                    true); 
		leglb->SetFrame_REF_to_abs(ChFrame<>(ChVector<>(-2 +COG_OF_LEG, 1.25, 1.1)));
		leglb->SetRot(Q_from_AngZ(CH_C_PI));
		leglb->AddAsset(ltexture);
		app->GetSystem()->Add(leglb);


		//3_Create three cylinder axles.
		this->axlef = std::make_shared<ChBodyEasyCylinder>(0.1, 2, 50, false, true);           //用一个轴把两条腿连接起来，这样可以保证两条腿的运动始终是保持我们想要的相位差。
		axlef->SetPos(ChVector<>(2, 0.85, 0));                                                  //如果把每条腿都连接在trunk上而不是轴上,时间长了之后就会乱掉。
		axlef->SetRot(Q_from_AngX(CH_C_PI / 2.));                                               //ChBodyEasyCylinder产生的圆柱是高在
		app->GetSystem()->Add(axlef);
		auto atexture = std::make_shared<ChTexture>(GetChronoDataFile("blu.png"));
		axlef->AddAsset(atexture);

		this->axlem = std::make_shared<ChBodyEasyCylinder>(0.1, 2, 50, false, true);
		axlem->SetPos(ChVector<>(0, 0.85, 0));
		axlem->SetRot(Q_from_AngX(CH_C_PI / 2.));
		app->GetSystem()->Add(axlem);
		axlem->AddAsset(atexture);

		this->axleb = std::make_shared<ChBodyEasyCylinder>(0.1, 2, 50, false, true);
		axleb->SetPos(ChVector<>(-2, 0.85, 0));
		axleb->SetRot(Q_from_AngX(CH_C_PI / 2.));
		app->GetSystem()->Add(axleb);
		axleb->AddAsset(atexture);

		//Add legs to axles.
		this->axlef_leglf = std::make_shared<ChLinkLockLock>();                                     //添加六个关节。这个关节是不能动的关节。
		axlef_leglf->Initialize(axlef, leglf, ChCoordsys<>(ChVector<>(2.05, 0.83, 1)));
		app->GetSystem()->AddLink(axlef_leglf);

		this->axlef_legrf = std::make_shared<ChLinkLockLock>();
		axlef_legrf->Initialize(axlef, legrf, ChCoordsys<>(ChVector<>(1.95, 0.83, -1)));
		app->GetSystem()->AddLink(axlef_legrf);

		this->axlem_leglm = std::make_shared<ChLinkLockLock>();
		axlem_leglm->Initialize(axlem, leglm, ChCoordsys<>(ChVector<>(-0.05, 0.83, 1)));
		app->GetSystem()->AddLink(axlem_leglm);

		this->axlem_legrm = std::make_shared<ChLinkLockLock>();
		axlem_legrm->Initialize(axlem, legrm, ChCoordsys<>(ChVector<>(0.05, 0.83, -1)));
		app->GetSystem()->AddLink(axlem_legrm);

		this->axleb_leglb = std::make_shared<ChLinkLockLock>();
		axleb_leglb->Initialize(axleb, leglb, ChCoordsys<>(ChVector<>(-1.95, 0.83, 1)));
		app->GetSystem()->AddLink(axleb_leglb);

		this->axleb_legrb = std::make_shared<ChLinkLockLock>();
		axleb_legrb->Initialize(axleb, legrb, ChCoordsys<>(ChVector<>(-2.05, 0.83, -1)));
		app->GetSystem()->AddLink(axleb_legrb);

		// 4-Creat three engines which drive between axles and trunk.                               
		this->my_link_f = std::make_shared<ChLinkEngine>();                                            //添加三个引擎，这个引擎是让axle转。axle没有碰撞体积。
		my_link_f->Initialize(trunk, axlef, ChCoordsys<>(ChVector<>(2, 0.85, 0)));
		my_link_f->Set_shaft_mode(ChLinkEngine::ENG_SHAFT_LOCK); // also works as revolute support
		my_link_f->Set_eng_mode(ChLinkEngine::ENG_MODE_SPEED);
		app->GetSystem()->AddLink(my_link_f);

		this->my_link_m = std::make_shared<ChLinkEngine>();
		my_link_m->Initialize(trunk, axlem, ChCoordsys<>(ChVector<>(0, 0.85, 0)));
		my_link_m->Set_shaft_mode(ChLinkEngine::ENG_SHAFT_LOCK); // also works as revolute support
		my_link_m->Set_eng_mode(ChLinkEngine::ENG_MODE_SPEED);
		app->GetSystem()->AddLink(my_link_m);

		this->my_link_b = std::make_shared<ChLinkEngine>();
		my_link_b->Initialize(trunk, axleb, ChCoordsys<>(ChVector<>(-2, 0.85, 0)));
		my_link_b->Set_shaft_mode(ChLinkEngine::ENG_SHAFT_LOCK); // also works as revolute support
		my_link_b->Set_eng_mode(ChLinkEngine::ENG_MODE_SPEED);
		app->GetSystem()->AddLink(my_link_b);


		trunk->Move(offset);                                  //把所有的部件移动到初始位置。
		leglf->Move(offset);
		leglm->Move(offset);
		leglb->Move(offset);
		legrf->Move(offset);
		legrm->Move(offset);
		legrb->Move(offset);
		axlef->Move(offset);
		axlem->Move(offset);
		axleb->Move(offset);
	}


	~MyBeetle() {                                      //析构函数  destruction function
		ChSystem* mysystem = trunk->GetSystem();      // trick to get the system here

		mysystem->Remove(my_link_f);
		mysystem->Remove(my_link_m);
		mysystem->Remove(my_link_b);
		mysystem->Remove(trunk);
		mysystem->Remove(leglf);
		mysystem->Remove(leglm);
		mysystem->Remove(leglb);
		mysystem->Remove(legrf);
		mysystem->Remove(legrm);
		mysystem->Remove(legrb);
		mysystem->Remove(axlef);
		mysystem->Remove(axlem);
		mysystem->Remove(axleb);
	}
};

class MyEventReceiver : public IEventReceiver {                                                    
public:
	MyEventReceiver(ChIrrAppInterface* myapp, MyBeetle* mb) {                              //构造函数
		// store pointer to physical system & other stuff so we can tweak them by user keyboard
		app = myapp;
		beetle= mb;
	}

	bool OnEvent(const SEvent& event) {                                            //照猫画虎就可以了。
		// check if user presses keys
		if (event.EventType == irr::EET_KEY_INPUT_EVENT && !event.KeyInput.PressedDown) {
			switch (event.KeyInput.Key) {

			case irr::KEY_KEY_A:                                                          //如果接收到A键，引擎的角速度就会加0.5
				if (auto mfun = std::dynamic_pointer_cast<ChFunction_Const>(beetle->my_link_f->Get_spe_funct()))
					mfun->Set_yconst(0.5 + mfun->Get_yconst());
				if (auto mfun = std::dynamic_pointer_cast<ChFunction_Const>(beetle->my_link_m->Get_spe_funct()))
					mfun->Set_yconst(0.5 + mfun->Get_yconst());
				if (auto mfun = std::dynamic_pointer_cast<ChFunction_Const>(beetle->my_link_b->Get_spe_funct()))
					mfun->Set_yconst(0.5 + mfun->Get_yconst());
				return true;
			case irr::KEY_KEY_Z:                                                         //如果接收到Z键，引擎的角速度就会减0.5
				if (auto mfun = std::dynamic_pointer_cast<ChFunction_Const>(beetle->my_link_f->Get_spe_funct()))
					mfun->Set_yconst(-0.5 + mfun->Get_yconst());
				if (auto mfun = std::dynamic_pointer_cast<ChFunction_Const>(beetle->my_link_m->Get_spe_funct()))
					mfun->Set_yconst(-0.5 + mfun->Get_yconst());
				if (auto mfun = std::dynamic_pointer_cast<ChFunction_Const>(beetle->my_link_b->Get_spe_funct()))
					mfun->Set_yconst(-0.5 + mfun->Get_yconst());
				return true;
		
			}
		}
		return false;
	}

private:                                          
	ChIrrAppInterface* app;
	MyBeetle* beetle;
};

/*
void createTerrain(ChSystem & sys, double dia, double W, double H, double L)
{
	int x = W / dia - 3;
	int y = H / dia;
	int z = L / dia - 3;
	double mass = 0.05 * 4 / 3 * CH_C_PI*pow(dia / 2.0, 3.0);// density * volume
	double inertia = 0.4*mass*dia*dia / 4.0;
	int i = 3, j = 0, k = 3;
	
				
				std::shared_ptr<ChBody> particle;
				particle->SetPos(ChVector<>(-W / 2.0, -boxDrop, -(chassisL / 2.0) - 2.0) + ChVector<>(i + ChRandom() - 0.5, j, k + ChRandom() - 0.5) * dia);
				particle->SetMass(mass);
				particle->SetInertiaXX(ChVector<>(inertia, inertia, inertia));
				particle->GetCollisionModel()->AddSphere(dia / 2.0);
				particle->SetCollide(true);
				particle->SetPos_dt(ChVector<>(ChRandom() - 0.5, -ChRandom(), ChRandom() - 0.5) * 10);
				sys.AddBody(particle);

}*/

void createTerrain(ChIrrAppInterface* app, double dia, double x, double y,double z)
{
	auto wallfront = std::make_shared<ChBodyEasyBox>(0.2, 2*y, 2*z, 500, true, false);
	wallfront->SetPos(ChVector<>(x + 0.1, y, 0));
	wallfront->SetBodyFixed(true);
	app->GetSystem()->Add(wallfront);
	auto wallleft = std::make_shared<ChBodyEasyBox>(2 * x, 2 * y, 0.2, 500, true, false);
	wallleft ->SetPos(ChVector<>(0, y, z + 0.1));
	wallleft->SetBodyFixed(true);
	app->GetSystem()->Add(wallleft);
	auto wallright = std::make_shared<ChBodyEasyBox>(2 * x, 2 * y, 0.2, 500, true, false);
	wallright->SetPos(ChVector<>(0, y, -z - 0.1));
	wallright->SetBodyFixed(true);
	app->GetSystem()->Add(wallright);
	auto wallback = std::make_shared<ChBodyEasyBox>(0.2, 2 * y, 2 * z, 500, true, false);
	wallback->SetPos(ChVector<>(-x-0.1, y, 0));
	wallback->SetBodyFixed(true);
	app->GetSystem()->Add(wallback);

	int imax = 2*x / dia, kmax = 2*z / dia;
	int i, j, k, m=0;
	double xini = -x + dia / 2.0, yini = dia / 2.0, zini = -z + dia / 2.0;
	std::array<std::shared_ptr<ChBodyEasySphere>,5000> particle;
	for (j = 0; j < 4; j++)
	{
		for (i = 0; i < imax; i++)
		{
			for (k = 0; k < kmax; k++)
			{
				particle[m] = std::make_shared<ChBodyEasySphere>(dia / 2.0 , 5, true, true);
				particle[m]->SetPos(ChVector<>(xini + dia*i, yini+dia*j, zini + dia*k));
				app->GetSystem()->Add(particle[m]);
				m++;
			}
		}
	}

}


int main(int argc, char* argv[]) {                                       
    // Set path to Chrono data directory
    SetChronoDataPath(CHRONO_DATA_DIR);               
    // Create a Chrono physical system
    ChSystem mphysicalSystem;                                       

    // Create the Irrlicht visualization (open the Irrlicht device,
    // bind a simple user interface, etc. etc.)
    ChIrrApp application(&mphysicalSystem, L"beetle", core::dimension2d<u32>(1200, 700),false);  // screen dimensions

	// create text with info
	IGUIStaticText* textFPS = application.GetIGUIEnvironment()->addStaticText(
		L"Use keys A,Z to move the robot", rect<s32>(150, 10, 430, 40), true);                        

    // Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
    application.AddTypicalCamera(core::vector3df(2, 5, -5),
                                 core::vector3df(0, 1, 0));  // to change the position of camera                   


    ///////////////////////////////////////////////////////////////////////////////////////////
	//Create a floor that is fixed (that is used also to represent the absolute reference)
	auto floorBody = std::make_shared<ChBodyEasyBox>(60, 2, 60,  // x, y, z dimensions
		                                             3000,       // density
		                                             true,       // with collide geometry
		                                             true        // enable visualization geometry
		                                             );
	floorBody->SetPos(ChVector<>(0, -1, 0));
	floorBody->SetBodyFixed(true);
	floorBody->GetMaterialSurface()->SetSfriction(1.0);
	floorBody->GetMaterialSurface()->SetKfriction(1.0);                                                 
	auto ftexture = std::make_shared<ChTexture>(GetChronoDataFile("ground.png"));
	floorBody->AddAsset(ftexture);

	mphysicalSystem.Add(floorBody);

	// Creat the terrain. 
	double x=5; double y=2; double z=2;// The size of the terrain, xyz is half length of each side.
	double particleDiameter(0.2);
	createTerrain(&application, particleDiameter, x, y, z);



	// ..the beetle (this class - see above - is a 'set' of bodies and links, automatically added at creation)
	MyBeetle* mybeetle = new MyBeetle(&application, ChVector<>(0, particleDiameter * 4, 0));        //mybeetle添加进来,
    

	// Use this function for adding a ChIrrNodeAsset to all items
	// Otherwise use application.AssetBind(myitem); on a per-item basis.
	application.AssetBindAll();                                                            
    // Use this function for 'converting' assets into Irrlicht meshes
	application.AssetUpdateAll();

	// This is for GUI tweaking of system parameters..
	MyEventReceiver receiver(&application, mybeetle);
	// note how to add a custom event receiver to the default interface:
	application.SetUserEventReceiver(&receiver);

	////////////////////////////////////////////////////////////////////////////////////////

	mphysicalSystem.SetMaxItersSolverSpeed(20);  // the higher, the easier to keep the constraints 'mounted'. I have used this way to overcome the softness of joints.
	mphysicalSystem.SetSolverType(ChSystem::SOLVER_SOR);   //之前我把迭代速度设置成20，但是发现如果身体太重腿的关节就会脱落。当我把20改成120后，就消失了

	//mphysicalSystem.SetSolverType(ChSystem::SOLVER_BARZILAIBORWEIN);  //This is a more precise method. But a bit slower. You can see in demo_convergence.cpp
	//mphysicalSystem.SetMaxItersSolverSpeed(40);                       //我用了这个更精确的方法，但是并没有产生预期的效果。


	// Optionally, attach a RGB color asset to the floor, for better visualization
    auto color = std::make_shared<ChColorAsset>();
    color->SetColor(ChColor(0.2f, 0.25f, 0.25f));
    floorBody->AddAsset(color);

    // Adjust some settings:
	application.SetStepManage(true);
    application.SetTimestep(0.005);
    application.SetTryRealtime(true);

	
	double simTime(0);
	double timestep = application.GetTimestep();

	ChStreamOutAsciiFile mfileo("beetle_archive.txt");
	// Create a binary archive, that uses the binary file as storage.
	ChArchiveAsciiDump marchiveout(mfileo);
	ChStreamOutAsciiFile mfileo2("beetle_archiverotation.txt");
	// Create a binary archive, that uses the binary file as storage.
	ChArchiveAsciiDump marchiveout2(mfileo2);


	//Do the simulation and output the data.
	while (application.GetDevice()->run()) {                                    
		
		// Irrlicht must prepare frame to draw
		application.GetVideoDriver()->beginScene(true, true, SColor(255, 140, 161, 192));
		// Irrlicht application draws all 3D objects and all GUI items
		application.DrawAll();
		// Advance the simulation time step
		application.DoStep();
		// Irrlicht must finish drawing the frame
		application.GetVideoDriver()->endScene();

		simTime += timestep;

		//SERIALIZE TO ASCII DUMP                                
		try {                                                    
			mfileo << "Time is "<< simTime<<"\n";
			marchiveout << CHNVP(mybeetle->trunk->coord);
			marchiveout << CHNVP(mybeetle->trunk->coord_dt);
			marchiveout << CHNVP(mybeetle->trunk->coord_dtdt);

			marchiveout << CHNVP(mybeetle->leglf->coord);
			marchiveout << CHNVP(mybeetle->leglf->coord_dt);
			marchiveout << CHNVP(mybeetle->leglf->coord_dtdt);

			//marchiveout << CHNVP(mybeetle->my_link_f->Get_spe_funct());   //still a little bug for now. 输出角速度现在只能输出初始时刻的。我估计可能和键盘输入指令有关。
			//marchiveout2 << CHNVP(mybeetle->my_link_f);        

			mfileo << "\n";

		}										   
		catch (ChException myex) {
			GetLog() << "ERROR: " << myex.what() << "\n\n";
		}

	}

	if (mybeetle)
		delete mybeetle;

    return 0; 
}
