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

#include "chrono/core/ChRealtimeStep.h"               //������ͷ�ļ�һ����Щ�Ǳ���ģ�һ��ʼ����ֱ�Ӹ��ƣ��������Ҫ������ӡ�

#define COG_OF_LEG 0.2561     //The obj from 3Dbuilder has a cog different from the cog calculated by hand.It may because the 3dbuilder is too simple.
                             // I have modified the obj in 3dbuilder in advance ,by give it a offset in advance. 0.2561 is the real cog calculated 
                            // by easymesh.(from cog to the center of circle)
                           //��Ϊ�����õ�Windows�Դ���3D��ģ���߲�����obj�ļ���beetle_leg_norm.obj��.�������Ƚϼ�ª����������ĺ�����������Ĳ�һ������chronoengine�����Ҳ��һ����
                           //����3Dbuilder��ģ��ʱ���ǵ�����һ�㣬������ģ���а����������ķ���ԭ�㣬�����ǰ�ģ��Ĭ�ϵ����ķ���ԭ�㣨��Ҫ�ú�������������ʱ���˺ü���,�����3dmaxʲô�ľͺ��ˡ���
                          //0.2561�ǰ�Բ���ε��ȵ�������Բ�ĵľ��롣�������ʼ���ȵ�λ�õ�ʱ�������������Ҫ����ʱҲ�ǵ��˺ܾá�

//Use the namespace of Chrono 
using namespace chrono;
using namespace chrono::irrlicht;

// Use the main namespaces of Irrlicht
using namespace irr;                     
using namespace irr::core;
using namespace irr::scene;
using namespace irr::video;
using namespace irr::io;
using namespace irr::gui;                            //һ�㿪ͷ�������⼸�������ռ䣬����ж������Ҫ�ٽ��иĶ���


//double boxW(12);
//double boxL(50);
//double boxH(2);
//double boxDrop(2 + boxH);
//double chassisL(15.0);
//double particleDiameter(0.2);

class MyBeetle{                                     //��Ϊ�Լ��Ļ����˵���д��һ���࣬����������Բο�demo_forklift.	                                                
public:                                             //��ʵ�������һ��ʼ����ģ��forklift�ģ������ͺ����MyEventReceiverҲ�ǡ�
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
	std::shared_ptr<ChBodyEasyCylinder> axleb;        //��һ��������������ȣ����������ĺô��ǿ�����������ͬ�������Թ���ÿ���ȷ���һ�����棬�����Ⱥ�������Ϊ΢�Ŷ��𽥲�̬���ҡ�
	std::shared_ptr<ChLinkLockLock> axlef_leglf;
	std::shared_ptr<ChLinkLockLock> axlef_legrf;
	std::shared_ptr<ChLinkLockLock> axlem_leglm;
	std::shared_ptr<ChLinkLockLock> axlem_legrm;
	std::shared_ptr<ChLinkLockLock> axleb_leglb;
	std::shared_ptr<ChLinkLockLock> axleb_legrb;        //������beetle����ɲ��֡�  

	MyBeetle(ChIrrAppInterface* app, ChVector<> offset = ChVector<>(0, 0, 0)) {             //���캯�� construction function  ��������ģ�͵ĺ��ġ�
		// 1-Create a box trunk.
		trunk = std::make_shared<ChBodyEasyBox>(4, 0.2, 1.8, 10, true, true);             //���ChBodyEasyBox�Ĳο�˵����ѧ�ῴ���ಢѧ����γ�ʼ��
		trunk->SetPos(ChVector<>(0, 0.85, 0));                                             //����λ��

		auto ttexture = std::make_shared<ChTexture>(GetChronoDataFile("csrc.png"));         
		trunk->AddAsset(ttexture);                                                         //����������ӱ���ͼ����
		app->GetSystem()->Add(trunk);                                   //����ǰ����˵�ģ�ÿһ��������Ҫ���뵽ChSystem�У�����Ǽ���ķ���������ÿһ���µĲ������һ�����������
		                                                               //��������ָ�룬����д�����Ѷ����򵥵��ȿ�demo_crank��ô����


/*		auto material = std::make_shared<ChMaterialSurfaceDEM>();
		material->SetYoungModulus(6e4);
		material->SetFriction(0.3f);
		material->SetRestitution(0.2f);
		material->SetAdhesion(0);             */

		// 2-Create six legs.
		legrf = std::make_shared<ChBodyEasyMesh>(GetChronoDataFile("beetle_leg_norm.obj"),  // mesh .OBJ file   //Բ���ȵĹ���û��EasyBox�������ֳ�����Ե��ã����ǿ��Ե����ⲿ��obj�ļ���
			                                     50,                                      // density
			                                     true,                                     // compute mass, inertia & COG from the mesh (must be a closed watertight mesh!)
			                                     true,                                    // enable collision with mesh
			                                     0.001,                                   // sphere swept inflate of mesh - improves robustness of collision detection
			                                     true);                                   // enable visualization of mesh
		legrf->SetFrame_REF_to_abs(ChFrame<>(ChVector<>(2 - COG_OF_LEG, 0.45, -1.1)));      //SetFrame_REF_to_abs������REF�ο�ϵ�ھ��Բο�ϵ�����꣬Ҫ������һ������Ҫ����ChFrame��ChAuxRef��������Ķ��塣
		                                                                                    //Ҳ��������վ�Ͽ�Reference manual��
		auto ltexture = std::make_shared<ChTexture>(GetChronoDataFile("coffee.jpg"));         
		legrf->AddAsset(ltexture);                                                          //��ӱ���ͼ�������ltexture����leg-texture����˼��
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
		this->axlef = std::make_shared<ChBodyEasyCylinder>(0.1, 2, 50, false, true);           //��һ����������������������������Ա�֤�����ȵ��˶�ʼ���Ǳ���������Ҫ����λ�
		axlef->SetPos(ChVector<>(2, 0.85, 0));                                                  //�����ÿ���ȶ�������trunk�϶���������,ʱ�䳤��֮��ͻ��ҵ���
		axlef->SetRot(Q_from_AngX(CH_C_PI / 2.));                                               //ChBodyEasyCylinder������Բ���Ǹ���
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
		this->axlef_leglf = std::make_shared<ChLinkLockLock>();                                     //��������ؽڡ�����ؽ��ǲ��ܶ��Ĺؽڡ�
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
		this->my_link_f = std::make_shared<ChLinkEngine>();                                            //����������棬�����������axleת��axleû����ײ�����
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


		trunk->Move(offset);                                  //�����еĲ����ƶ�����ʼλ�á�
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


	~MyBeetle() {                                      //��������  destruction function
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
	MyEventReceiver(ChIrrAppInterface* myapp, MyBeetle* mb) {                              //���캯��
		// store pointer to physical system & other stuff so we can tweak them by user keyboard
		app = myapp;
		beetle= mb;
	}

	bool OnEvent(const SEvent& event) {                                            //��è�����Ϳ����ˡ�
		// check if user presses keys
		if (event.EventType == irr::EET_KEY_INPUT_EVENT && !event.KeyInput.PressedDown) {
			switch (event.KeyInput.Key) {

			case irr::KEY_KEY_A:                                                          //������յ�A��������Ľ��ٶȾͻ��0.5
				if (auto mfun = std::dynamic_pointer_cast<ChFunction_Const>(beetle->my_link_f->Get_spe_funct()))
					mfun->Set_yconst(0.5 + mfun->Get_yconst());
				if (auto mfun = std::dynamic_pointer_cast<ChFunction_Const>(beetle->my_link_m->Get_spe_funct()))
					mfun->Set_yconst(0.5 + mfun->Get_yconst());
				if (auto mfun = std::dynamic_pointer_cast<ChFunction_Const>(beetle->my_link_b->Get_spe_funct()))
					mfun->Set_yconst(0.5 + mfun->Get_yconst());
				return true;
			case irr::KEY_KEY_Z:                                                         //������յ�Z��������Ľ��ٶȾͻ��0.5
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
	MyBeetle* mybeetle = new MyBeetle(&application, ChVector<>(0, particleDiameter * 4, 0));        //mybeetle��ӽ���,
    

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
	mphysicalSystem.SetSolverType(ChSystem::SOLVER_SOR);   //֮ǰ�Ұѵ����ٶ����ó�20�����Ƿ����������̫���ȵĹؽھͻ����䡣���Ұ�20�ĳ�120�󣬾���ʧ��

	//mphysicalSystem.SetSolverType(ChSystem::SOLVER_BARZILAIBORWEIN);  //This is a more precise method. But a bit slower. You can see in demo_convergence.cpp
	//mphysicalSystem.SetMaxItersSolverSpeed(40);                       //�������������ȷ�ķ��������ǲ�û�в���Ԥ�ڵ�Ч����


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

			//marchiveout << CHNVP(mybeetle->my_link_f->Get_spe_funct());   //still a little bug for now. ������ٶ�����ֻ�������ʼʱ�̵ġ��ҹ��ƿ��ܺͼ�������ָ���йء�
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
