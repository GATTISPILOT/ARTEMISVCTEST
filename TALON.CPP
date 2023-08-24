// ==============================================================
//                 ORBITER MODULE: TALON
//                  Part of the ORBITER SDK
//          Copyright (C) 2002-2004 Martin Schweiger
//                   All rights reserved
//
// TALON.cpp
// Control module for TALON vessel class
//

// ==============================================================
//#define ORBITER_MODULE
#include "orbitersdk.h"
#include "TALON.h"
#include "meshres_TALON.h"
#include "TALONVCMESH.h"


#include <stdio.h>
#include <fstream>

#define LOADBMP(id) (LoadBitmap (g_Param.hDLL, MAKEINTRESOURCE (id)))

//BOOL CALLBACK ATHLETE_TEST_DlgProc(HWND, UINT, WPARAM, LPARAM);
//
//
//



static DIMENSION const MFD_BUTTONS_FN_DIM = _D(50, 28, 2, 2);
static RECT const MFD_BUTTONS_FN_RECT[] = { _R(000, 000, 100, 168), _R(200, 000, 300, 168), _R(400, 000, 500, 168) };

static DIMENSION const MFD_BUTTONS_SYS_DIM = _D(30, 30, 0, 0);
static RECT const MFD_BUTTONS_SYS_RECT[] = { _R(230, 272, 260, 302), _R(230, 272, 260, 302), _R(230, 272, 260, 302) };



PARTICLESTREAMSPEC rcs_stream = {
	0, 0.1, 60, 30.0, 0.1, 0.03, 8, 0, PARTICLESTREAMSPEC::EMISSIVE,
	PARTICLESTREAMSPEC::LVL_LIN, 0.1, 1,
	PARTICLESTREAMSPEC::ATM_FLAT, 0.2, 0.1
};

PARTICLESTREAMSPEC main_stream = {
	0, 1.0, 60, 40.0, 0.1, 0.04, 12, 0, PARTICLESTREAMSPEC::EMISSIVE,
	PARTICLESTREAMSPEC::LVL_LIN, 0.0, 0.05,
	PARTICLESTREAMSPEC::ATM_FLAT, 0.1, 0.1
};

PARTICLESTREAMSPEC land_stream = {
	0, 3, 60, 2.0, 0.1, 0.7, 6, 10, PARTICLESTREAMSPEC::EMISSIVE,
	PARTICLESTREAMSPEC::LVL_PSQRT, 0.0, 0.05,
	PARTICLESTREAMSPEC::ATM_FLAT, 0.1, 0.1
};

PARTICLESTREAMSPEC srb_stream = {
	0, 4.0, 30, 40.0, 0.4, 8.0, 12, 3.0, PARTICLESTREAMSPEC::DIFFUSE,
	PARTICLESTREAMSPEC::LVL_PSQRT, 0.5, 1,
	PARTICLESTREAMSPEC::ATM_FLAT, 1, 0.1
};





void TALON::clbkPostCreation(void)
{
	
//	SetAnimation(anim_JOINT0, 0.0);
//	SetAnimation(anim_JOINT1, 0.0);
//	SetAnimation(anim_JOINT2, .5);
//	SetAnimation(anim_JOINT3, 0.0202);
//	SetAnimation(anim_JOINT4, 0.0);
//	SetAnimation(anim_JOINT5, 0.5279);
//	SetAnimation(anim_JOINT6, -0.8352);
//	SetAnimation(anim_JOINT7, 1.0);
	//PANELEXT_proc = 1;
	//PANEL_proc = 1;
}


void VLiftCoeff(double aoa, double M, double Re, double *cl, double *cm, double *cd)
{
	static const double step = RAD*30.0;
	static const double istep = 1.0 / step;
	static const int nabsc = 13;
	//	Angle of attack					 -180	-150	-120	-90		-60		-30		0		30	 60		90		120		150	  180
	//	static const double CL[nabsc] = {	0,	-0.12,	-0.1,	  0,	  0,	  0,	0,		0,	  0,	 0,		0.1,	0.12,	0};
	//	static const double CM[nabsc] = {	0,	0.0002,	0.0004,	0.0004,	0.0003,	0.0002,	0, -0.0002,-0.0003,-0.0004,-0.0004,-0.0002,	0};
	static const double CL[nabsc] = { 0, -0.12, -0.1, 0, 0, 0, 0, 0, 0, 0, 0.1, 0.12, 0 };
	static const double CM[nabsc] = { 0, -0.0002, -0.0004, -0.0004, -0.0003, -0.0002, 0, 0.0002, 0.0003, 0.0004, 0.0004, 0.0002, 0 };
	// lift and moment coefficients from -180 to 180 in 30 degree steps.

	aoa += PI;
	int idx = max(0, min(11, (int)(aoa*istep)));
	double d = aoa*istep - idx;
	*cl = CL[idx] + (CL[idx + 1] - CL[idx])*d;
	*cm = (CM[idx] + (CM[idx + 1] - CM[idx])*d) / 10;
	*cd = 0.25 + oapiGetInducedDrag(*cl, 1.27, 0.3);
}

// 2. horizontal lift component (vertical stabiliser and body)

void HLiftCoeff(double beta, double M, double Re, double *cl, double *cm, double *cd)
{
	static const double step = RAD * 30;
	static const double istep = 1.0 / step;
	static const int nabsc = 13;
	static const double CL[nabsc] = { 0, -0.1, -0.1, 0, 0, 0, 0, 0, 0, 0, 0.1, 0.1, 0 };

	beta += PI;
	int idx = max(0, min(11, (int)(beta*istep)));
	double d = beta*istep - idx;
	*cl = CL[idx] + (CL[idx + 1] - CL[idx])*d;
	*cm = 0.0;
	*cd = 0.25 + oapiGetInducedDrag(*cl, 1.27, 0.3);
}





// Constructor


// --------------------------------------------------------------
// Destructor
// --------------------------------------------------------------
TALON::~TALON() {
	delete mfdController;

}




TALON::TALON (OBJHANDLE hObj, int fmodel)
: VESSEL3 (hObj, fmodel)


{
	mfdController = new ClassicMfd(3, g_Param.hFont[0], g_Param.col[0], g_Param.hBrush[0], g_Param.hBrush[1], &MFD_BUTTONS_FN_DIM, MFD_BUTTONS_FN_RECT, &MFD_BUTTONS_SYS_DIM, MFD_BUTTONS_SYS_RECT);

	mode = 0;
	BEACON = 0;
	ofs = _V(0, 0, 0);
	string[0] = 0;

	tex_rcs = oapiRegisterExhaustTexture("exhaust_crcs");
	tex_main = oapiRegisterExhaustTexture("exhaust_crcs");


	//cmMeshUINT = AddMesh(oapiLoadMeshGlobal("TALON\\TALON_CM"));// mesh1
	//SetMeshVisibilityMode(cmMeshUINT, MESHVIS_ALWAYS);

	//cockpitMeshUINT = AddMesh(oapiLoadMeshGlobal("Orion-MPCV\\orion-cockpit"));// mesh3
	//SetMeshVisibilityMode(cockpitMeshUINT, MESHVIS_NEVER);

	//panelMeshUINT = AddMesh(oapiLoadMeshGlobal("Orion-MPCV\\orion-panel"));// mesh4
	//SetMeshVisibilityMode(panelMeshUINT, MESHVIS_ALWAYS);


	//vcMeshUINT = AddMesh(oapiLoadMeshGlobal("TALON\\TALON_VCNEW"));// mesh3
	//SetMeshVisibilityMode(coverMeshUINT, MESHVIS_NEVER);

	//SetMeshVisibilityMode(vcMeshUINT = AddMesh(oapiLoadMeshGlobal("TALON\\TALON_VCNEW"), MESHVIS_NEVER));

//	SetMeshVisibilityMode(meshi_Vessel = AddMesh(meshhg_Vessel = oapiLoadMeshGlobal("TALON\\TALON_CM")), MESHVIS_ALWAYS );
//	SetMeshVisibilityMode(meshi_VC1 = AddMesh(meshhg_Vessel), MESHVIS_VC);
//	SetMeshVisibilityMode(meshi_VC = AddMesh(meshhg_VC = oapiLoadMeshGlobal("TALON\\TALON_VCNEW")), MESHVIS_VC);





	//SetMeshVisibilityMode(meshi_VC = AddMesh(meshhg_VC = oapiLoadMeshGlobal("TALON\\TALON_VCNEW")), MESHVIS_NEVER);

	//	SetMeshVisibilityMode(meshi_VC = AddMesh(meshhg_VC = oapiLoadMeshGlobal("INTERSTELLAR\\VESSEL\\ENDURANCE\\ENDURANCE42VC")), MESHVIS_VC);


	//cmMesh = oapiLoadMeshGlobal("TALON\\TALON_CM");
	//smMesh = oapiLoadMeshGlobal("TALON\\TALON_SM");
	//vcMesh = oapiLoadMeshGlobal("TALON\\TALON_VC");

	//cockpitMesh = oapiLoadMeshGlobal("Orion-MPCV\\orion-cockpit");
	//panelMesh = oapiLoadMeshGlobal("Orion-MPCV\\orion-panel");
	//astroMesh = oapiLoadMeshGlobal("Orion-MPCV\\orion-astro");

	//coverMesh = oapiLoadMeshGlobal("TALON\\TALON_COVER");
//	hatchMesh = oapiLoadMeshGlobal("Orion-MPCV\\orion-hatch");

	//drogueMeshUINT = AddMesh(oapiLoadMeshGlobal("Orion-MPCV\\orion-drogue"));
	//chuteMeshUINT = AddMesh(oapiLoadMeshGlobal("Orion-MPCV\\orion-chute"));

	//fair1Mesh = oapiLoadMeshGlobal("Orion-MPCV\\orion-fair1");
	//fair2Mesh = oapiLoadMeshGlobal("Orion-MPCV\\orion-fair2");
	//fair3Mesh = oapiLoadMeshGlobal("Orion-MPCV\\orion-fair3");
	//lasMesh = oapiLoadMeshGlobal("Orion-MPCV\\orion-las");

//	fair1h = fair2h = fair3h = fairLh = 0L;

	//SetConfig0_Ready();
	
//	mfds_co.ngroup = 3; //
	//huds.ngroup = 2;
	//huds.size = 0.25;
	huds.ngroup = GRP_TALON_VCNEW4_HUD;
	huds.hudcnt = _V(0, 1.668, 2.631);

LIFT_SPEED =.1;
JOINT1_proc = 0;
JOINT2_proc = 0;
CAM = 0;
ARM3_status = HATCH_DOWN;
ARM3_proc = 1.0;
do_attach = 0;
DefineAnimations ();
center_arm = false;
apd = 0;
HGA_status = HATCH_UP;
HGA_proc = 0.0;

HATCH_status = HATCH_UP;
HATCH_proc = 0.0;
EVA = 0;
ARM2_check = SPANEL_STOP;
JOINT1_proc = 0;
ANTROT_check = SPANEL_STOP;
JOINT2_proc = .5;
static VECTOR3 beaconcol[4] = { { 1, 0, 0 }, //red
{ 0, 1, 0 }, { 1, 1, 0 }, { 1, 1, 0 }

};

for (int i = 0; i < 3; i++) {
	beacon[i].shape = BEACONSHAPE_STAR;
	beacon[i].pos = beaconpos + i;
	beacon[i].col = beaconcol + i;
	beacon[i].size = 0.2;
	beacon[i].falloff = 0.3;
	beacon[i].period = 1;
	beacon[i].duration = 1;
	beacon[i].tofs = (6 - i)*0.2;
	beacon[i].active = true;
	AddBeacon(beacon + i);
}

 {
	 int (i) = 3;
	beacon[i].shape = BEACONSHAPE_STAR;
	beacon[i].pos = beaconpos + i;
	beacon[i].col = beaconcol + i;
	beacon[i].size = 0.2;
	beacon[i].falloff = 0.3;
	beacon[i].period = 1;
	beacon[i].duration = .1;
	beacon[i].tofs = (6 - i)*0.2;
	beacon[i].active = true;
	AddBeacon(beacon + i);
}



beaconpos[0].x = -2.477;
beaconpos[1].x = 2.477;
beaconpos[2].x = 0;
beaconpos[3].x = 0;


beaconpos[0].z = 1.49;
beaconpos[1].z = 1.49;
beaconpos[2].z = 1.49;
beaconpos[3].z = 1.49;


beaconpos[0].y = 0;
beaconpos[1].y = 0;
beaconpos[2].y = 2.477;
beaconpos[3].y = -2.477;







}


void TALON::clbkSetClassCaps (FILEHANDLE cfg)
//void TALON::SetTransporter()
{
	// physical specs
		SetSize (3);
	SetEmptyMass (500.0);
	SetCW (0.09, 0.09, 2, 1.4);
	SetWingAspect (0.7);
	SetWingEffectiveness (2.5);
    SetCrossSections (_V(710.54,2159.34,207.95));
    SetSurfaceFrictionCoeff (0.07, 0.3);
    SetRotDrag (_V(0.10,0.13,0.04));
    SetMaxWheelbrakeForce (5e5);
    SetPMI (_V(743.84,872.38,148.19));
	SetTrimScale (0.1);
	SetCameraOffset(_V(-0.41, 1.6, 2.21));
	
	SetTouchdownPoints  (_V(0,.0001,10), _V(-1.6,.0001,-10), _V(1.6,.0001,-10));; 
EnableTransponder (true);

//SetMeshVisibilityMode (AddMesh (oapiLoadMeshGlobal ("TALON")), MESHVIS_ALWAYS ); //Main ship mesh



	// Call clbkSetClassCaps_UMMU and select a member
	
	SetAnimation(anim_JOINT0,1.0);
	//SetAnimation(anim_JOINT2, .5);
	//SetAnimation(anim_JOINT7, 1.0);


SetMeshVisibilityMode(meshi_Vessel = AddMesh(meshhg_Vessel = oapiLoadMeshGlobal("TALON\\TALON_CM")), MESHVIS_ALWAYS);
SetMeshVisibilityMode(meshi_VC1 = AddMesh(meshhg_Vessel), MESHVIS_VC);
SetMeshVisibilityMode(meshi_VC = AddMesh(meshhg_VC = oapiLoadMeshGlobal("TALON\\TALON_VCNEW4")), MESHVIS_VC);
mfdController->HandleLoadMesh(meshi_VC, meshhg_VC);
smMeshUINT = AddMesh(oapiLoadMeshGlobal("TALON\\TALON_SM_MMU"));// mesh0
SetMeshVisibilityMode(smMeshUINT, MESHVIS_EXTERNAL);
coverMeshUINT = AddMesh(oapiLoadMeshGlobal("TALON\\TALON_COVER"));// mesh2
SetMeshVisibilityMode(coverMeshUINT, MESHVIS_EXTERNAL);
//MMUMeshUINT = AddMesh(oapiLoadMeshGlobal("TALON\\TALON_MMU"));// mesh5
//SetMeshVisibilityMode(MMUMeshUINT, MESHVIS_EXTERNAL);

astroMeshUINT = AddMesh(oapiLoadMeshGlobal("TALON\\TALON_CREW"));// mesh5
SetMeshVisibilityMode(astroMeshUINT, MESHVIS_EXTERNAL | MESHVIS_VC);

astro1MeshUINT = AddMesh(oapiLoadMeshGlobal("TALON\\TALON_CREW4"));// mesh5
SetMeshVisibilityMode(astro1MeshUINT, MESHVIS_EXTERNAL | MESHVIS_VC);

lasMeshMeshUINT = AddMesh(oapiLoadMeshGlobal("Orion-MPCV\\orion-las"));// mesh13
SetMeshVisibilityMode(lasMeshMeshUINT, MESHVIS_NEVER);



SetDockParams(_V(0, 0, 4.375), _V(0, 0, 1), _V(0, 1, 0));//NOSE DOCK0
SetDockParams(_V(0, 1.9, -1), _V(0, -1, 0), _V(0, 0, 1)); //eva DOCK1
SetConfig0_Ready();
}

// =======================================================
// UMMU Class Caps
// =======================================================


void TALON::ReloadMeshes(void)
{
	UINT meshidx;
	double dz;		// z-offset for various configs

	ClearMeshes(TRUE);

	switch (mode) {
	case 0:
	case 1:	dz = 0;	break;
	case 2:
	case 3:
	case 4:
	case 5:
	case 6:	dz = -2.06;	break;	// capsule config
	case 10:
	case 11: dz = 0;	break;
	case 12: dz = -2;	break;
	}

	ofs = _V(0, 0, 0 + dz);
	ofs_camera = _V(-0.30, 1.368, 2.3 + dz);	// left hand seat, eye-point of docking window
	ofs_hud = _V(0, 1.6, 2.5 + dz);
	SetCameraOffset(ofs_camera);

	SetMeshVisibilityMode(meshi_Vessel = AddMesh(meshhg_Vessel = oapiLoadMeshGlobal("TALON\\TALON_CM")), MESHVIS_EXTERNAL);//mesh0
	SetMeshVisibilityMode(meshi_VC1 = AddMesh(meshhg_Vessel), MESHVIS_VC);//mesh1
	SetMeshVisibilityMode(meshi_VC = AddMesh(meshhg_VC = oapiLoadMeshGlobal("TALON\\TALON_VCNEW4")), MESHVIS_VC);//mesh2
	smMeshUINT = AddMesh(oapiLoadMeshGlobal("TALON\\TALON_SM_MMU"));// mesh3
	SetMeshVisibilityMode(smMeshUINT, MESHVIS_NEVER);
	coverMeshUINT = AddMesh(oapiLoadMeshGlobal("TALON\\TALON_COVER"));// mesh4

	SetMeshVisibilityMode(coverMeshUINT, MESHVIS_NEVER);

	MMUMeshUINT = AddMesh(oapiLoadMeshGlobal("TALON\\TALON_MMU"));// mesh5
	SetMeshVisibilityMode(MMUMeshUINT, MESHVIS_EXTERNAL);

	astroMeshUINT = AddMesh(oapiLoadMeshGlobal("TALON\\TALON_CREW"));// mesh5
	SetMeshVisibilityMode(astroMeshUINT, MESHVIS_EXTERNAL | MESHVIS_VC);

	astro1MeshUINT = AddMesh(oapiLoadMeshGlobal("TALON\\TALON_CREW4"));// mesh5
	SetMeshVisibilityMode(astro1MeshUINT, MESHVIS_EXTERNAL | MESHVIS_VC);

	drogueMeshUINT = AddMesh(oapiLoadMeshGlobal("Orion-MPCV\\orion-drogue"));
	SetMeshVisibilityMode(drogueMeshUINT, MESHVIS_NEVER);

	chuteMeshUINT = AddMesh(oapiLoadMeshGlobal("Orion-MPCV\\orion-chute"));
	SetMeshVisibilityMode(chuteMeshUINT, MESHVIS_NEVER);

	BallonMeshUINT = AddMesh(oapiLoadMeshGlobal("Orion-MPCV\\orion-balloons"));
	SetMeshVisibilityMode(BallonMeshUINT, MESHVIS_NEVER);

	lasMeshMeshUINT = AddMesh(oapiLoadMeshGlobal("Orion-MPCV\\orion-las"));// mesh13
	SetMeshVisibilityMode(lasMeshMeshUINT, MESHVIS_NEVER);


	switch (mode) {
	case 0:
		//AddMesh(coverMesh, &ofs);
		SetMeshVisibilityMode(coverMeshUINT, MESHVIS_EXTERNAL | MESHVIS_VC);
		SetMeshVisibilityMode(smMeshUINT, MESHVIS_EXTERNAL | MESHVIS_VC);
		SetMeshVisibilityMode(MMUMeshUINT, MESHVIS_ALWAYS);

		//AddMesh(smMesh, &ofs);
		//	AddMesh(solar0Mesh, &ofs);
		break;

	case 1:		// Orbit config
		SetMeshVisibilityMode(coverMeshUINT, MESHVIS_EXTERNAL | MESHVIS_VC);
		SetMeshVisibilityMode(smMeshUINT, MESHVIS_EXTERNAL | MESHVIS_VC);
		SetMeshVisibilityMode(MMUMeshUINT, MESHVIS_ALWAYS);


		break;
	case 2:		// Orbit config
		SetMeshVisibilityMode(coverMeshUINT, MESHVIS_EXTERNAL | MESHVIS_VC);
		SetMeshVisibilityMode(MMUMeshUINT, MESHVIS_ALWAYS);

	//	SetMeshVisibilityMode(smMeshUINT, MESHVIS_EXTERNAL | MESHVIS_VC);


		break;
	case 3:		// Drogue config
		SetMeshVisibilityMode(drogueMeshUINT, MESHVIS_EXTERNAL | MESHVIS_VC);
		SetMeshVisibilityMode(MMUMeshUINT, MESHVIS_NEVER);
		//	ShiftCentreOfMass(ofs);
		break;
	case 4:		// not used
	case 5:		// Chute config
		SetMeshVisibilityMode(chuteMeshUINT, MESHVIS_EXTERNAL | MESHVIS_VC);
		SetMeshVisibilityMode(MMUMeshUINT, MESHVIS_NEVER);
		//	SetMeshVisibilityMode(AddMesh(chuteMesh, &ofs), MESHVIS_ALWAYS);
		//	ShiftCentreOfMass(ofs);
		break;
	case 6:		// Landed config
		//	AddMesh(balloonMesh, &ofs);
		SetMeshVisibilityMode(BallonMeshUINT, MESHVIS_EXTERNAL | MESHVIS_VC);

	//	SetMeshVisibilityMode(MMUMeshUINT, MESHVIS_NEVER);

		break;
	case 7:
		//AddMesh(coverMesh, &ofs);
		SetMeshVisibilityMode(coverMeshUINT, MESHVIS_EXTERNAL | MESHVIS_VC);
		SetMeshVisibilityMode(smMeshUINT, MESHVIS_EXTERNAL | MESHVIS_VC);

		//AddMesh(smMesh, &ofs);
		//	AddMesh(solar0Mesh, &ofs);
		break;
	case 10:		// Orbit config
		SetMeshVisibilityMode(coverMeshUINT, MESHVIS_EXTERNAL | MESHVIS_VC);
		SetMeshVisibilityMode(smMeshUINT, MESHVIS_EXTERNAL | MESHVIS_VC);
		SetMeshVisibilityMode(lasMeshMeshUINT, MESHVIS_EXTERNAL | MESHVIS_VC);

		break;
	case 11:		// Orbit config
		SetMeshVisibilityMode(coverMeshUINT, MESHVIS_EXTERNAL | MESHVIS_VC);
		SetMeshVisibilityMode(smMeshUINT, MESHVIS_EXTERNAL | MESHVIS_VC);
		SetMeshVisibilityMode(lasMeshMeshUINT, MESHVIS_EXTERNAL | MESHVIS_VC);

		break;


}



}






void TALON::clbkPostStep(double simt, double simdt, double mjd)
{


	//vcanimation
	if (GetNavmodeState(1) == true){
		SetAnimation(anim_killrot, 1);
	}
	else{
		SetAnimation(anim_killrot, 0);
	}
	
	if (GetNavmodeState(2) == true){
		SetAnimation(anim_horlevel, 1);
	}
	else{
		SetAnimation(anim_horlevel, 0);
	}
	if (GetNavmodeState(3) == true){
		SetAnimation(anim_prograde, 1);
	}
	else{
		SetAnimation(anim_prograde, 0);
	}
	if (GetNavmodeState(4) == true){
		SetAnimation(anim_retrograde, 1);
	}
	else{
		SetAnimation(anim_retrograde, 0);
	}
	if (GetNavmodeState(5) == true){
		SetAnimation(anim_NMLplus, 1);
	}
	else{
		SetAnimation(anim_NMLplus, 0);
	}
	if (GetNavmodeState(6) == true){
		SetAnimation(anim_nmlminus, 1);
	}
	else{
		SetAnimation(anim_nmlminus, 0);
	}
	if (GetNavmodeState(7) == true){
		SetAnimation(anim_althold, 1);
	}
	else{
		SetAnimation(anim_althold, 0);
	}
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	if (JOINT1_proc > 1)JOINT1_proc = (1);
	else if (JOINT1_proc < 0)JOINT1_proc = (0);
	double db = simdt * LIFT_SPEED;
	if (ARM2_check == SPANEL_DOWN)JOINT1_proc = (JOINT1_proc + db);
	else if (ARM2_check == SPANEL_UP)JOINT1_proc = (JOINT1_proc - db);
	else if (ARM2_check == SPANEL_STOP)JOINT1_proc = JOINT1_proc;

	if (JOINT1_proc >1)JOINT1_proc = (1);
	else if (JOINT1_proc < 0)JOINT1_proc = (0);


	SetAnimation(anim_JOINT1, JOINT1_proc);



	if (ARM3_status >= HATCH_RAISING) {
		double da = simdt * LIFT_SPEED;
		if (ARM3_status == HATCH_RAISING) {
			if (ARM3_proc > 0.0) ARM3_proc = max(0.0, ARM3_proc - da);
			else                ARM3_status = HATCH_UP;
		}
		else {
			if (ARM3_proc < 1.0) ARM3_proc = min(1.0, ARM3_proc + da);
			else                ARM3_status = HATCH_DOWN;
		}


		SetAnimation(anim_JOINT7, ARM3_proc);
		//SetAnimation(anim_arm_ARM3, ARM_proc);
		//sprintf(oapiDebugString(), "anim %5.5f", ARM3_proc);
	}


	if (JOINT2_proc > 1)JOINT2_proc = (0);
	else if (JOINT2_proc < 0)JOINT2_proc = (1);
	//double db = simdt * LIFT_SPEED;
	if (ANTROT_check == SPANEL_DOWN)JOINT2_proc = (JOINT2_proc + db);
	else if (ANTROT_check == SPANEL_UP)JOINT2_proc = (JOINT2_proc - db);
	else if (ANTROT_check == SPANEL_STOP)JOINT2_proc = JOINT2_proc;

	if (JOINT2_proc >1)JOINT2_proc = (0);
	else if (JOINT2_proc < 0)JOINT2_proc = (1);


	SetAnimation(anim_JOINT2, JOINT2_proc);




	if (HGA_status >= HATCH_RAISING) {
		double da = simdt * LIFT_SPEED;
		if (HGA_status == HATCH_RAISING) {
			if (HGA_proc > 0.0) HGA_proc = max(0.0, HGA_proc - da);
			else                HGA_status = HATCH_UP;
		}
		else {
			if (HGA_proc < 1.0) HGA_proc = min(1.0, HGA_proc + da);
			else                HGA_status = HATCH_DOWN;
		}


		SetAnimation(anim_HGA, HGA_proc);
		//SetAnimation(anim_arm_ARM3, ARM_proc);
		//sprintf(oapiDebugString(), "anim %5.5f", ARM3_proc);
	}
//HATCH
	if (HATCH_status >= HATCH_RAISING) {
		double da = simdt * LIFT_SPEED;
		if (HATCH_status == HATCH_RAISING) {
			if (HATCH_proc > 0.0) HATCH_proc = max(0.0, HATCH_proc - da);
			else                HATCH_status = HATCH_UP;
		}
		else {
			if (HATCH_proc < 1.0) HATCH_proc = min(1.0, HATCH_proc + da);
			else                HATCH_status = HATCH_DOWN;
		}


		SetAnimation(anim_HATCH, HATCH_proc);
		//SetAnimation(anim_arm_ARM3, ARM_proc);
		//sprintf(oapiDebugString(), "anim %5.5f", ARM3_proc);
	}



	if (BEACON == 0){
		beacon[0].active = true;
		beacon[1].active = true;
		beacon[2].active = true;
		beacon[3].active = true;
	}

	if (BEACON == 1){
		beacon[0].active = false;
		beacon[1].active = false;
		beacon[2].active = false;
		beacon[3].active = false;
	}


	if (mode > 1)
		h = GetAltitude();

	//	int t = (int) (simt - t_start);

	switch (mode) {
	case 0:
		

		
	case 1:
		return;

	case 2:
		if (h < 3000) { 	// Check if we shall deploy the drogue
			blowCover();
			SetConfig3_Drogue();
		}
		return;

	case 3:
		AddForce(_V(10000, 0, 0), _V(0, 0.05, 0.1));	// rotate while hanging on the chutes
		AddForce(_V(0, -10, 0), _V(0, 0, -100));			// trim foot down

		if (h < 800) {	// Check if we shall deploy the chute
			blowDrogue();
			SetConfig4_Chute();		// = 5
			AddForce(_V(10000, 0, 0), _V(0, 0.05, 0.1));	// rotate while hanging on the chutes
			AddForce(_V(0, -5, 0), _V(0, 0, -100));			// trim foot down,
			oapiSetTimeAcceleration(1);
		}
		return;

	case 4:
	case 5:
		if (oapiGetTimeAcceleration() >10)
			oapiSetTimeAcceleration(10);

		AddForce(_V(10000, 0, 0), _V(0, 0.05, 0.1));	// rotate while hanging on the chutes
		AddForce(_V(0, -10, 0), _V(0, 0, -100));			// trim foot down,

		if (h < 10)	{ // Check if we are close to the ground		
			oapiSetTimeAcceleration(1);
			blowChute();
			SetConfig6_Land();
		}
		return;

	case 10:
		if (GetAltitude() > 108000) {	// Check if blow the fairings
			//	if (fairing)
			//	blowFairing();
			SetConfig11_Launch1();
			ReloadMeshes();
		}
		break;
	case 11:
		if (GetAltitude() > 109000) {	// Check if blow the LAS
			blowLas();
			SetConfig0_Ready();
			ReloadMeshes();
		}
		break;
	}	// end switch
}
void TALON::SeparateMMU(void)
{
	if (HATCH_proc > .8){
	
	// Create MMU at docking port
	DOCKHANDLE hDock = GetDockHandle(1);
	if (GetDockStatus(hDock)) return; // something is already attached to this docking port

	int i;
	char name[256];
	OBJHANDLE hVessel;
	for (i = 0;; i++) {
		sprintf(name, "%s-MMU-%d", GetName(), i + 1);
		hVessel = oapiGetVesselByName(name);
		if (!hVessel) break;
	}

	VESSELSTATUS vs;
	EVA = 1;
	GetStatus(vs);
	hMMU = oapiCreateVessel(name, "TALON_EVA", vs);
	Dock(hMMU, 0, 0, 1);
	SetMeshVisibilityMode(MMUMeshUINT, MESHVIS_NEVER);

	oapiSetFocusObject(hMMU);
	
	}
}
void TALON::ENTER_MMU(void)
{
	if ((HATCH_proc > .8)&&(EVA=1)){

		// Create MMU at docking port
		DOCKHANDLE hDock = GetDockHandle(1);
		if (GetDockStatus(hDock)) return; // something is already attached to this docking port

		for (DWORD i = 0; i < oapiGetVesselCount(); i++) {
			OBJHANDLE hV = oapiGetVesselByIndex(i);    //get handle for ship
			// if (strncmp (hV->GetVesselByName(), "OBLIVIONBIKE1", 12)!=0  ){
			OBJHANDLE Focusvessel = oapiGetVesselByName("TALON-MMU-1");


			oapiDeleteVessel(Focusvessel);
			SetMeshVisibilityMode(MMUMeshUINT, MESHVIS_EXTERNAL);
			EVA = 0;
			//oapiSetFocusObject(hMMU);
		}
	}
}
void TALON::DefineAnimations(void)

{
	//vcanimation
	static UINT NAVMODE_01 = GRP_TALON_VCNEW4_KILLROT_ON;
	static MGROUP_TRANSLATE killrot(2, &NAVMODE_01, 1, _V(0, 0.001, -.009));
	anim_killrot = CreateAnimation(0);
	AddAnimationComponent(anim_killrot, 0.0f, 1.0f, &killrot);

	static UINT NAVMODE_02 = { GRP_TALON_VCNEW4_HLVL_ON };
	static MGROUP_TRANSLATE H_level(2, &NAVMODE_02, 1, _V(0, 0.001, -.009));
	anim_horlevel = CreateAnimation(0);
	AddAnimationComponent(anim_horlevel, 0.0f, 1.0f, &H_level);

	static UINT NAVMODE_03 = GRP_TALON_VCNEW4_PROGRADE_ON;
	static MGROUP_TRANSLATE prograde(2, &NAVMODE_03, 1, _V(0, 0.001, -.009));
	anim_prograde = CreateAnimation(0);
	AddAnimationComponent(anim_prograde, 0.0f, 1.0f, &prograde);

	static UINT NAVMODE_04 = GRP_TALON_VCNEW4_RETROGRADE_ON;
	static MGROUP_TRANSLATE retrograde(2, &NAVMODE_04, 1, _V(0, 0.001, -.009));
	anim_retrograde = CreateAnimation(0);
	AddAnimationComponent(anim_retrograde, 0.0f, 1.0f, &retrograde);

	static UINT NAVMODE_05 = GRP_TALON_VCNEW4_NML_ON;
	static MGROUP_TRANSLATE NMLPLUS(2, &NAVMODE_05, 1, _V(0, 0.001, -.009));
	anim_NMLplus = CreateAnimation(0);
	AddAnimationComponent(anim_NMLplus, 0.0f, 1.0f, &NMLPLUS);

	static UINT NAVMODE_06 = GRP_TALON_VCNEW4_NML_ON_0;
	static MGROUP_TRANSLATE NMLMINUS(2, &NAVMODE_06, 1, _V(0, 0.001, -.009));
	anim_nmlminus = CreateAnimation(0);
	AddAnimationComponent(anim_nmlminus, 0.0f, 1.0f, &NMLMINUS);

	static UINT NAVMODE_07 = GRP_TALON_VCNEW4_HOLDALT_ON;
	static MGROUP_TRANSLATE ALTHOLD(2, &NAVMODE_07, 1, _V(0, 0.001, -.009));
	anim_althold = CreateAnimation(0);
	AddAnimationComponent(anim_althold, 0.0f, 1.0f, &ALTHOLD);
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	anim_HATCH = CreateAnimation(0.0);
	static UINT HATCHGrp[5] = { 22,31,33,38,39 };
	static MGROUP_ROTATE HATCH1(0, HATCHGrp, 5, _V(1.975, 0, 2.448), _V(0, 1, 0), (float)(180 * RAD));

	static UINT HATCHVCGrp[5] = { 22, 31, 33, 38, 39 };
	static MGROUP_ROTATE HATCH2(1, HATCHVCGrp, 5, _V(1.975, 0, 2.448), _V(0, 1, 0), (float)(180 * RAD));

	AddAnimationComponent(anim_HATCH, 0, 1, &HATCH1);
	AddAnimationComponent(anim_HATCH, 0, 1, &HATCH2);

	
	
	
	
	ANIMATIONCOMPONENT_HANDLE parentHGA;

	anim_HGA = CreateAnimation(0.0);
	static UINT HGAGrp[1] = { GRP_HGAPORT };
	static MGROUP_ROTATE HGA(3, HGAGrp, 1, _V(1.082399, 2.219181, .2751346), _V(.894, -.44, 0), (float)(90 * RAD));
	parentHGA = AddAnimationComponent(anim_HGA, 0, .25, &HGA);



	static UINT HGA2Grp[1] = { GRP_HGA2 };
	static MGROUP_ROTATE HGA2(3, HGA2Grp, 1, _V(1.14, 2.22, -.705), _V(.894, -.44, 0), (float)(180 * RAD));
	parentHGA = AddAnimationComponent(anim_HGA, .25, .5, &HGA2, parentHGA);

	static UINT HGA1Grp[2] = { GRP_HGA3B, GRP_HGA3A };
	static MGROUP_ROTATE HGA1(3, HGA1Grp, 2, _V(1.14, 1.65, -.728), _V(.894, -.44, 0), (float)(90 * RAD));
	AddAnimationComponent(anim_HGA, .5, 1, &HGA1, parentHGA);





	ANIMATIONCOMPONENT_HANDLE  parent19, parent18, parent20, parent21, parent22, parent23, parent1, parent8, parent2, parent3, parent4, parent5, parent6, parent7, parent9, parent10, parent11, parent12, parent13, parent14, parent15, parent16, parent17;
	ANIMATIONCOMPONENT_HANDLE parent24, parent25, parent26, parent, parent27, LEFTMAINparent, RIGHTMAINparent;
	ANIMATIONCOMPONENT_HANDLE  parent19A, parent18A, parent20A, parent21A, parent22A, parent23A, parent8A, parent2A, parent3A, parent4A, parent5A, parent6A, parent7A, parent9A, parent10A, parent11A, parent12A, parent13A, parent14A, parent15A, parent16A, parent17A;


	//anim_ANTFOLD = CreateAnimation(0);
	anim_JOINT2 = CreateAnimation(0.5);
	anim_JOINT1 = CreateAnimation(0.0);
	anim_JOINT7 = CreateAnimation(0.0);
	anim_JOINT0 = CreateAnimation(0.0);



	static UINT RMSShoulderYaw1Grp[1] = { GRP_arrayswivel };//
	SOLAR1 = new MGROUP_ROTATE(3, RMSShoulderYaw1Grp, 1, _V(0, 0, 0), _V(0, 0, 1), (float)(-20 * RAD)); // -2 .. +145
	LEFTMAINparent = AddAnimationComponent(anim_JOINT0, 0, 1, SOLAR1);



	static UINT RMSShoulderYaw2Grp[1] = { GRP_Stbd_Array };//
	SOLAR2 = new MGROUP_ROTATE(3, RMSShoulderYaw2Grp, 1, _V(2.433, 0, .259), _V(0, 1, 0), (float)(90 * RAD)); // -2 .. +145
	parent = AddAnimationComponent(anim_JOINT1, 0, 1, SOLAR2, LEFTMAINparent);

	static UINT RMSShoulderYaw3Grp[2] = { GRP_Stbd_SAsup, GRP_Stbd_Arra0 };//
	SOLAR3 = new MGROUP_ROTATE(3, RMSShoulderYaw3Grp, 2, _V(2.642, -.008, .259), _V(1, 0, 0), (float)(360 * RAD)); // -2 .. +145
	parent2 = AddAnimationComponent(anim_JOINT2, 0, 1, SOLAR3, parent);


	//SOLAR SAIL DEPLOY  4.847, -.01, .367   5.36, -.01, .376

	static UINT sail1[1] = { GRP_SA_S1 };//sa_s1
	SOLAR4 = new MGROUP_ROTATE(3, sail1, 1, _V(4.847, -.01, .367), _V(1, 0, 0), (float)(90 * RAD)); // -2 .. +145
	parent3 = AddAnimationComponent(anim_JOINT7, 0, .4, SOLAR4, parent2);

	static UINT sail2[1] = { GRP_SA_S2 };//sa_s2
	SOLAR5 = new MGROUP_ROTATE(3, sail2, 1, _V(4.847, -.01, .367), _V(-0.951042, -0.309061, 0), (float)(-180 * RAD)); // -2 .. +145
	parent4 = AddAnimationComponent(anim_JOINT7, 0, 1, SOLAR5, parent3);

	static UINT sail3[1] = { GRP_SA_S3 };//sa_s3
	SOLAR6 = new MGROUP_ROTATE(3, sail3, 1, _V(4.847, -.01, .367), _V(-0.809003, -0.587804, 0), (float)(180 * RAD)); // -2 .. +145
	parent5 = AddAnimationComponent(anim_JOINT7, 0, 1, SOLAR6, parent4);

	static UINT sail4[1] = { GRP_SA_S4 };//sa_s4
	SOLAR7 = new MGROUP_ROTATE(3, sail4, 1, _V(4.847, -.01, .367), _V(-0.587787, -0.809016, 0), (float)(-180 * RAD)); // -2 .. +145
	parent6 = AddAnimationComponent(anim_JOINT7, 0, 1, SOLAR7, parent5);

	static UINT sail5[1] = { GRP_SA_S5 };//sa_s5
	SOLAR8 = new MGROUP_ROTATE(3, sail5, 1, _V(4.847, -.01, .367), _V(-0.309053, -0.951045, 0), (float)(180 * RAD)); // -2 .. +145
	parent7 = AddAnimationComponent(anim_JOINT7, 0, 1, SOLAR8, parent6);

	static UINT sail6[1] = { GRP_SA_S6 };//sa_s6
	SOLAR9 = new MGROUP_ROTATE(3, sail6, 1, _V(4.847, -.01, .367), _V(0, -1, 0), (float)(-180 * RAD)); // -2 .. +145
	parent8 = AddAnimationComponent(anim_JOINT7, 0, 1, SOLAR9, parent7);

	static UINT sail7[1] = { GRP_SA_S7 };//sa_s7
	SOLAR10 = new MGROUP_ROTATE(3, sail7, 1, _V(4.847, -.01, .367), _V(-0.309053, 0.951045, 0), (float)(180 * RAD));
	parent9 = AddAnimationComponent(anim_JOINT7, 0, 1, SOLAR10, parent8);


	static UINT sail8[1] = { GRP_SA_S8 };//sa_s8
	SOLAR11 = new MGROUP_ROTATE(3, sail8, 1, _V(4.847, -.01, .367), _V(-0.587787, 0.809016, 0), (float)(-180 * RAD)); // -2 .. +145
	parent10 = AddAnimationComponent(anim_JOINT7, 0, 1, SOLAR11, parent9);

	static UINT sail9[1] = { GRP_SA_S9 };//sa_s9
	SOLAR12 = new MGROUP_ROTATE(3, sail9, 1, _V(4.847, -.01, .367), _V(-0.809003, 0.587804, 0), (float)(180 * RAD)); // -2 .. +145
	parent11 = AddAnimationComponent(anim_JOINT7, 0, 1, SOLAR12, parent10);

	static UINT sail10[1] = { GRP_SA_S10 };//sa_s10
	SOLAR13 = new MGROUP_ROTATE(3, sail10, 1, _V(4.847, -.01, .367), _V(-0.951042, 0.309061, 0), (float)(-180 * RAD)); // -2 .. +145
	parent12 = AddAnimationComponent(anim_JOINT7, 0, 1, SOLAR13, parent11);

	static UINT sail11[1] = { GRP_SA_S11 };//sa_s11
	SOLAR14 = new MGROUP_ROTATE(3, sail11, 1, _V(4.847, -.01, .367), _V(1, 0, 0), (float)(180 * RAD)); // -2 .. +145
	parent13 = AddAnimationComponent(anim_JOINT7, 0, 1, SOLAR14, parent12);



	static UINT sail12[1] = { GRP_SA_S12 };//sa_s12
	SOLAR15 = new MGROUP_ROTATE(3, sail12, 1, _V(4.847, -.01, .367), _V(0.951042, 0.309061, 0), (float)(-180 * RAD)); // -2 .. +145
	parent14 = AddAnimationComponent(anim_JOINT7, 0, 1, SOLAR15, parent13);

	static UINT sail13[1] = { GRP_SA_S13 };//sa_s13
	SOLAR16 = new MGROUP_ROTATE(3, sail13, 1, _V(4.847, -.01, .367), _V(0.809003, 0.587804, 0), (float)(180 * RAD)); // -2 .. +145
	parent15 = AddAnimationComponent(anim_JOINT7, 0, 1, SOLAR16, parent14);

	static UINT sail14[1] = { GRP_SA_S14 };//sa_s14
	SOLAR17 = new MGROUP_ROTATE(3, sail14, 1, _V(4.847, -.01, .367), _V(0.587787, 0.809016, 0), (float)(-180 * RAD)); // -2 .. +145
	parent16 = AddAnimationComponent(anim_JOINT7, 0, 1, SOLAR17, parent15);

	static UINT sail15[1] = { GRP_SA_S15 };//sa_s15
	SOLAR18 = new MGROUP_ROTATE(3, sail15, 1, _V(4.847, -.01, .367), _V(0.309053, 0.951045, 0), (float)(180 * RAD)); // -2 .. +145
	parent17 = AddAnimationComponent(anim_JOINT7, 0, 1, SOLAR18, parent16);

	static UINT sail16[1] = { GRP_SA_S16 };//sa_s16
	SOLAR19 = new MGROUP_ROTATE(3, sail16, 1, _V(4.847, -.01, .367), _V(0, -1, 0), (float)(-180 * RAD)); // -2 .. +145
	parent18 = AddAnimationComponent(anim_JOINT7, 0, 1, SOLAR19, parent17);

	static UINT sail17[1] = { GRP_SA_S17 };//sa_s17
	SOLAR20 = new MGROUP_ROTATE(3, sail17, 1, _V(4.847, -.01, .367), _V(0.309053, -0.951045, 0), (float)(180 * RAD)); // -2 .. +145
	parent19 = AddAnimationComponent(anim_JOINT7, 0, 1, SOLAR20, parent18);

	static UINT sail18[1] = { GRP_SA_S18 };//sa_s18
	SOLAR21 = new MGROUP_ROTATE(3, sail18, 1, _V(4.847, -.01, .367), _V(0.587787, -0.809016, 0), (float)(-180 * RAD)); // -2 .. +145
	parent20 = AddAnimationComponent(anim_JOINT7, 0, 1, SOLAR21, parent19);

	static UINT sail19[1] = { GRP_SA_S19 };//sa_s19
	SOLAR22 = new MGROUP_ROTATE(3, sail19, 1, _V(4.847, -.01, .367), _V(0.809003, -0.587804, 0), (float)(180 * RAD)); // -2 .. +145
	parent21 = AddAnimationComponent(anim_JOINT7, 0, 1, SOLAR22, parent20);

	static UINT sail20[1] = { GRP_SA_S20 };//sa_s20
	SOLAR23 = new MGROUP_ROTATE(3, sail20, 1, _V(4.847, -.01, .367), _V(0.951042, -0.309061, 0), (float)(-180 * RAD)); // -2 .. +145
	parent22 = AddAnimationComponent(anim_JOINT7, 0, 1, SOLAR23, parent21);

	//PORT SIDE

	static UINT RMSShoulderYawP1Grp[1] = { GRP_arrayswivel1 };//
	SOLAR1A = new MGROUP_ROTATE(3, RMSShoulderYawP1Grp, 1, _V(0, 0, 0), _V(0, 0, 1), (float)(-20 * RAD)); // -2 .. +145
	RIGHTMAINparent = AddAnimationComponent(anim_JOINT0, 0, 1, SOLAR1A);



	static UINT RMSShoulderYawP2Grp[1] = { GRP_Port_Array };//
	SOLAR2A = new MGROUP_ROTATE(3, RMSShoulderYawP2Grp, 1, _V(-2.416, -.04, .26), _V(0, 1, 0), (float)(-90 * RAD)); // -2 .. +145
	parent1 = AddAnimationComponent(anim_JOINT1, 0, 1, SOLAR2A, RIGHTMAINparent);

	static UINT RMSShoulderYawP3Grp[2] = { GRP_Port_SAsup, GRP_Port_Arra0 };//
	SOLAR3A = new MGROUP_ROTATE(3, RMSShoulderYawP3Grp, 2, _V(-2.626, -0.003, .26), _V(1, 0, 0), (float)(360 * RAD)); // -2 .. +145
	parent2A = AddAnimationComponent(anim_JOINT2, 0, 1, SOLAR3A, parent1);

	//SOLAR SAIL DEPLOY  5.426  -5.392, -.038, .376  -5.346, -.001, .381   4.831.368

	static UINT sail1A[1] = { GRP_SA_P1 };//sa_s1
	SOLAR4A = new MGROUP_ROTATE(3, sail1A, 1, _V(-4.831, -.001, .368), _V(1, 0, 0), (float)(90 * RAD)); // -2 .. +145
	parent3A = AddAnimationComponent(anim_JOINT7, 0, .4, SOLAR4A, parent2A);

	static UINT sail2A[1] = { GRP_SA_P2 };//sa_s2
	SOLAR5A = new MGROUP_ROTATE(3, sail2A, 1, _V(-4.831, -.001, .368), _V(-0.951042, 0.309061, 0), (float)(-180 * RAD)); // -2 .. +145
	parent4A = AddAnimationComponent(anim_JOINT7, 0, 1, SOLAR5A, parent3A);

	static UINT sail3A[1] = { GRP_SA_P3 };//sa_s3
	SOLAR6A = new MGROUP_ROTATE(3, sail3A, 1, _V(-4.831, -.001, .368), _V(-0.809003, 0.587804, 0), (float)(180 * RAD)); // -2 .. +145
	parent5A = AddAnimationComponent(anim_JOINT7, 0, 1, SOLAR6A, parent4A);

	static UINT sail4A[1] = { GRP_SA_P4 };//sa_s4
	SOLAR7A = new MGROUP_ROTATE(3, sail4A, 1, _V(-4.831, -.001, .368), _V(-0.587787, 0.809016, 0), (float)(-180 * RAD)); // -2 .. +145
	parent6A = AddAnimationComponent(anim_JOINT7, 0, 1, SOLAR7A, parent5A);

	static UINT sail5A[1] = { GRP_SA_P5 };//sa_s5
	SOLAR8A = new MGROUP_ROTATE(3, sail5A, 1, _V(-4.831, -.001, .368), _V(-0.309053, 0.951045, 0), (float)(180 * RAD)); // -2 .. +145
	parent7A = AddAnimationComponent(anim_JOINT7, 0, 1, SOLAR8A, parent6A);

	static UINT sail6A[1] = { GRP_SA_P6 };//sa_s6
	SOLAR9A = new MGROUP_ROTATE(3, sail6A, 1, _V(-4.831, -.001, .368), _V(0, 1, 0), (float)(-180 * RAD)); // -2 .. +145
	parent8A = AddAnimationComponent(anim_JOINT7, 0, 1, SOLAR9A, parent7A);

	static UINT sail7A[1] = { GRP_SA_P7 };//sa_s7
	SOLAR10A = new MGROUP_ROTATE(3, sail7A, 1, _V(-4.831, -.001, .368), _V(0.309053, 0.951045, 0), (float)(180 * RAD));
	parent9A = AddAnimationComponent(anim_JOINT7, 0, 1, SOLAR10A, parent8A);

	static UINT sail8A[1] = { GRP_SA_P8 };//sa_s8
	SOLAR11A = new MGROUP_ROTATE(3, sail8A, 1, _V(-4.831, -.001, .368), _V(0.587787, 0.809016, 0), (float)(-180 * RAD)); // -2 .. +145
	parent10A = AddAnimationComponent(anim_JOINT7, 0, 1, SOLAR11A, parent9A);

	static UINT sail9A[1] = { GRP_SA_P9 };//sa_s9
	SOLAR12A = new MGROUP_ROTATE(3, sail9A, 1, _V(-4.831, -.001, .368), _V(0.809003, 0.587804, 0), (float)(180 * RAD)); // -2 .. +145
	parent11A = AddAnimationComponent(anim_JOINT7, 0, 1, SOLAR12A, parent10A);

	static UINT sail10A[1] = { GRP_SA_P10 };//sa_s10
	SOLAR13A = new MGROUP_ROTATE(3, sail10A, 1, _V(-4.831, -.001, .368), _V(0.951042, 0.309061, 0), (float)(-180 * RAD)); // -2 .. +145
	parent12A = AddAnimationComponent(anim_JOINT7, 0, 1, SOLAR13A, parent11A);

	static UINT sail11A[1] = { GRP_SA_P11 };//sa_s11
	SOLAR14A = new MGROUP_ROTATE(3, sail11A, 1, _V(-4.831, -.001, .368), _V(1, 0, 0), (float)(180 * RAD)); // -2 .. +145
	parent13A = AddAnimationComponent(anim_JOINT7, 0, 1, SOLAR14A, parent12A);



	static UINT sail12A[1] = { GRP_SA_P12 };//sa_s12
	SOLAR15A = new MGROUP_ROTATE(3, sail12A, 1, _V(-4.831, -.001, .368), _V(0.951042, -0.309061, -0), (float)(-180 * RAD)); // -2 .. +145
	parent14A = AddAnimationComponent(anim_JOINT7, 0, 1, SOLAR15A, parent13A);

	static UINT sail13A[1] = { GRP_SA_P13 };//sa_s13
	SOLAR16A = new MGROUP_ROTATE(3, sail13A, 1, _V(-4.831, -.001, .368), _V(0.809003, -0.587804, 0), (float)(180 * RAD)); // -2 .. +145
	parent15A = AddAnimationComponent(anim_JOINT7, 0, 1, SOLAR16A, parent14A);

	static UINT sail14A[1] = { GRP_SA_P14 };//sa_s14
	SOLAR17A = new MGROUP_ROTATE(3, sail14A, 1, _V(-4.831, -.001, .368), _V(0.587787, -0.809016, 0), (float)(-180 * RAD)); // -2 .. +145
	parent16A = AddAnimationComponent(anim_JOINT7, 0, 1, SOLAR17A, parent15A);

	static UINT sail15A[1] = { GRP_SA_P15 };//sa_s15
	SOLAR18A = new MGROUP_ROTATE(3, sail15A, 1, _V(-4.831, -.001, .368), _V(0.309053, -0.951045, 0), (float)(180 * RAD)); // -2 .. +145
	parent17A = AddAnimationComponent(anim_JOINT7, 0, 1, SOLAR18A, parent16A);

	static UINT sail16A[1] = { GRP_SA_P16 };//sa_s16
	SOLAR19A = new MGROUP_ROTATE(3, sail16A, 1, _V(-4.831, -.001, .368), _V(0, -1, 0), (float)(-180 * RAD)); // -2 .. +145
	parent18A = AddAnimationComponent(anim_JOINT7, 0, 1, SOLAR19A, parent17A);

	static UINT sail17A[1] = { GRP_SA_P17 };//sa_s17
	SOLAR20A = new MGROUP_ROTATE(3, sail17A, 1, _V(-4.831, -.001, .368), _V(-0.309053, -0.951045, 0), (float)(180 * RAD)); // -2 .. +145
	parent19A = AddAnimationComponent(anim_JOINT7, 0, 1, SOLAR20A, parent18A);

	static UINT sail18A[1] = { GRP_SA_P18 };//sa_s18
	SOLAR21A = new MGROUP_ROTATE(3, sail18A, 1, _V(-4.831, -.001, .368), _V(-0.587787, -0.809016, 0), (float)(-180 * RAD)); // -2 .. +145
	parent20A = AddAnimationComponent(anim_JOINT7, 0, 1, SOLAR21A, parent19A);

	static UINT sail19A[1] = { GRP_SA_P19 };//sa_s19
	SOLAR22A = new MGROUP_ROTATE(3, sail19A, 1, _V(-4.831, -.001, .368), _V(-0.809003, -0.587804, 0), (float)(180 * RAD)); // -2 .. +145
	parent21A = AddAnimationComponent(anim_JOINT7, 0, 1, SOLAR22A, parent20A);

	static UINT sail20A[1] = { GRP_SA_P20 };//sa_s20
	SOLAR23A = new MGROUP_ROTATE(3, sail20A, 1, _V(-4.831, -.001, .368), _V(-0.951042, -0.309061, 0), (float)(-180 * RAD)); // -2 .. +145
	parent22A = AddAnimationComponent(anim_JOINT7, 0, 1, SOLAR23A, parent21A);




	
}



// --------------------------------------------------------------
// Keyboard interface handler (buffered key events)
// --------------------------------------------------------------
int TALON::clbkConsumeBufferedKey(DWORD key, bool down, char *kstate)
{
	
	if (!down) return 0; 
	if (key == OAPI_KEY_5)
	{
		if (BEACON == 0)BEACON = 1;
		else (BEACON = 0);

	}

	if (key == OAPI_KEY_4)
	{
		RevertARM3();

	}

	if (key == OAPI_KEY_3)
	{
		RevertHGA();

	}
	if (key == OAPI_KEY_G)
	{
		RevertHATCH();

	}
	if (key == OAPI_KEY_E)
	{
		SeparateMMU();

	}
	if (key == OAPI_KEY_K)
	{
		ENTER_MMU();

	}
	if (key == OAPI_KEY_1)
	{
		mode = 0;
		SetConfig7_Ready();
		return 1;

	}
	if (key == OAPI_KEY_J)
	{			// Jettison service module  SeparateMMU ();
		if (mode < 2)			// && (door_status == DOOR_CLOSED))
			blowSM();
		return 1;
	}
	if (key == OAPI_KEY_6 &&!KEYMOD_SHIFT(kstate))
	{ // operate dish
		if (ARM2_check == SPANEL_STOP)ARM2_check = SPANEL_UP;
		else if (ARM2_check == SPANEL_UP)ARM2_check = SPANEL_STOP;
		else if (ARM2_check == SPANEL_DOWN)ARM2_check = SPANEL_UP;
		return 1;
	}
	if (key == OAPI_KEY_7&&!KEYMOD_SHIFT(kstate))
	{ // operate dish
		if (ARM2_check == SPANEL_STOP)ARM2_check = SPANEL_DOWN;
		else if (ARM2_check == SPANEL_DOWN)ARM2_check = SPANEL_STOP;
		else if (ARM2_check == SPANEL_UP)ARM2_check = SPANEL_DOWN;
		return 1;
	}
	if (key == OAPI_KEY_6 &&KEYMOD_SHIFT(kstate))
	{ // operate dish
		if (ANTROT_check == SPANEL_STOP)ANTROT_check = SPANEL_UP;
		else if (ANTROT_check == SPANEL_UP)ANTROT_check = SPANEL_STOP;
		else if (ANTROT_check == SPANEL_DOWN)ANTROT_check = SPANEL_UP;
		return 1;
	}
	if (key == OAPI_KEY_7&&KEYMOD_SHIFT(kstate))
	{ // operate dish
		if (ANTROT_check == SPANEL_STOP)ANTROT_check = SPANEL_DOWN;
		else if (ANTROT_check == SPANEL_DOWN)ANTROT_check = SPANEL_STOP;
		else if (ANTROT_check == SPANEL_UP)ANTROT_check = SPANEL_DOWN;
		return 1;
	}
	if (key == OAPI_KEY_C)
	{// Deploy drogue and chute
		double speed = GetAirspeed();

		if (GetAltitude() > 40000)
			return 1;		// do nothing above 40 km

		if ((mode == 2) && (speed < 1000)) {
			blowCover();
			SetConfig3_Drogue();
			return 1;
		}

		if ((mode == 3) && (speed < 120)) {
			blowDrogue();
			SetConfig4_Chute();
			return 1;
		}

		if ((mode == 5) && (GetAltitude() < 200))  { // Create the chute as an individual object
			blowChute();
			SetConfig6_Land();
			return 1;
		}
	}
	return 0;
}
// ====================================================================
// clbkVisualCreated used to display UMMU initialisation message 
// because oapiDebugString() doesn't work in clbkSetClassCap
// ====================================================================
void TALON::clbkVisualCreated(VISHANDLE vis, int refcount)
{
	//MainExternalMeshVisual = GetMesh(vis, 0);

}
// ==============================================================
// Visual destroyed
// ==============================================================
void TALON::clbkVisualDestroyed(VISHANDLE vis, int refcount)
{
	//MainExternalMeshVisual = 0;
}

void TALON::RevertARM3(void)
{
	ARM3_status = ((ARM3_status == HATCH_UP || ARM3_status == HATCH_RAISING) ?
	HATCH_LOWERING : HATCH_RAISING);
}
void TALON::RevertHGA(void)
{
	HGA_status = ((HGA_status == HATCH_UP || HGA_status == HATCH_RAISING) ?
	HATCH_LOWERING : HATCH_RAISING);
}

void TALON::RevertHATCH(void)
{
	HATCH_status = ((HATCH_status == HATCH_UP || HATCH_status == HATCH_RAISING) ?
	HATCH_LOWERING : HATCH_RAISING);
}

void TALON::clbkSaveState(FILEHANDLE scn)
{
	{
		char cbuf[256];


		sprintf(cbuf, "%0.4f", JOINT1_proc);
		oapiWriteScenario_string(scn, "JOINT1", cbuf);
		sprintf(cbuf, "%0.4f", JOINT2_proc);
		oapiWriteScenario_string(scn, "JOINT2", cbuf);
		sprintf(cbuf, "%0.4f", JOINT3_proc);
		sprintf(cbuf, "%0.4f", ARM3_proc);
		oapiWriteScenario_string(scn, "JOINT7", cbuf);
		oapiWriteScenario_int(scn, "MODE", mode);

		sprintf(cbuf, "%d %0.4f", HGA_status, HGA_proc);
		oapiWriteScenario_string(scn, "HGA", cbuf);

		sprintf(cbuf, "%d %0.4f", HATCH_status, HATCH_proc);
		oapiWriteScenario_string(scn, "HATCH", cbuf);

		sprintf(cbuf, "%d", BEACON);
		oapiWriteScenario_string(scn, "BEACON", cbuf);

		sprintf(cbuf, "%d", EVA);
		oapiWriteScenario_string(scn, "EVA", cbuf);

		if (water)
			oapiWriteScenario_int(scn, "WATER", 0);

	
		// ORBITER, default vessel parameters
		SaveDefaultState(scn);

	}

}
void TALON::clbkLoadStateEx (FILEHANDLE scn, void *status)
{
	char *line;
	int value;
	while (oapiReadScenario_nextline(scn, line))
	{


		if (!_strnicmp(line, "JOINT1", 6)) {
			sscanf(line + 6, "%lf", &JOINT1_proc);
		}

		if (!_strnicmp(line, "JOINT2", 6)) {
			sscanf(line + 6, "%lf", &JOINT2_proc);
		}

		if (!_strnicmp(line, "JOINT7", 6)) {
			sscanf(line + 6, "%lf", &ARM3_proc);
		}
		if (!_strnicmp(line, "HGA", 3)) {
			sscanf(line + 3, "%d%lf", &HGA_status, &HGA_proc);
		}


		if (!_strnicmp(line, "HATCH", 5)) {
			sscanf(line + 5, "%d%lf", &HATCH_status, &HATCH_proc);
		}


		// call this method to let PayloadManager to load its internal parameters

		if (!_strnicmp(line, "MODE", 4)) {
			sscanf_s(line + 4, "%d", &value);
			mode = (int)value;
		}

		if (!_strnicmp(line, "WATER", 5)) {
			water = 1;
		}

		if (!_strnicmp(line, "BEACON", 6)) {
			sscanf(line + 6, "%d ", &BEACON);
		}

		if (!_strnicmp(line, "EVA",3)) {
			sscanf(line + 3, "%d ", &EVA);
		}


		switch (mode) {
		case 0:
			SetConfig0_Ready();
			break;
		case 1:
			SetConfig1_Orbit();
			break;
		case 2:
			SetConfig2_Reentry();
			break;
		case 3:
			SetConfig3_Drogue();
			break;
		case 4:
		case 5:
			SetConfig4_Chute();
			break;
		case 6:
			SetConfig6_Land();
			break;

		case 7:
			SetConfig0_Surface();
			break;



		case 10:
			SetConfig10_Launch0();
			break;
		case 11:
			SetConfig11_Launch1();
			break;
		}




		// UMMU, Load all saved member from scenario. 
		// Return TRUE when a "UMMUCREW" line is encountered 
		// FALSE when it must pass this line to you or default parser

		// ORBITER, unrecognised option - pass to Orbiter's generic parser


		// ORBITER, unrecognised option - pass to Orbiter's generic parser



		ParseScenarioLineEx(line, status);
	}

	SetAnimation(anim_JOINT2, JOINT2_proc);
	SetAnimation(anim_JOINT1, JOINT1_proc);
	SetAnimation(anim_JOINT7, ARM3_proc);
	SetAnimation(anim_HGA, HGA_proc);

	SetAnimation(anim_HATCH, HATCH_proc);
}





	

bool TALON::clbkLoadGenericCockpit() {
	return true;
}

bool TALON::clbkLoadVC(int id) { // ID is the Preset Camera Position
	huds.size = 0.3;
	huds.nmesh = 2;

	mfdController->HandleLoadVC(id);
	oapiVCRegisterHUD(&huds);

	oapiVCRegisterArea(AID_NAVMODE1, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN);
	oapiVCRegisterArea(AID_NAVMODE2, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN);
	oapiVCRegisterArea(AID_NAVMODE3, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN);
	oapiVCRegisterArea(AID_NAVMODE4, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN);
	oapiVCRegisterArea(AID_NAVMODE5, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN);
	oapiVCRegisterArea(AID_NAVMODE6, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN);
	oapiVCRegisterArea(AID_NAVMODE7, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN);

	
	oapiVCSetAreaClickmode_Spherical(AID_NAVMODE1, _V(-0.1264551, 1.500134, 2.812949), 0.015);
	
	oapiVCSetAreaClickmode_Spherical(AID_NAVMODE2, _V(-.08476054, 1.500178, 2.822194), 0.015);
	oapiVCSetAreaClickmode_Spherical(AID_NAVMODE3, _V(-.04344729, 1.500378, 2.822194), 0.015);
	oapiVCSetAreaClickmode_Spherical(AID_NAVMODE4, _V(0, 1.500178, 2.822194), 0.015);
	oapiVCSetAreaClickmode_Spherical(AID_NAVMODE5, _V(.04249217, 1.500178, 2.822194), 0.015);
	oapiVCSetAreaClickmode_Spherical(AID_NAVMODE6, _V(.08443221, 1.500178, 2.822194), 0.015);
	oapiVCSetAreaClickmode_Spherical(AID_NAVMODE7, _V(.1269527, 1.500178, 2.822194), 0.015);
	
	switch (id) {
	case 0: // commander position
		SetCameraOffset(_V(-0.41, 1.6, 2.241));
		SetCameraDefaultDirection(_V(0, 0, 1));
		SetCameraMovement(_V(0, 0, 0.2), 0, 0, _V(-0.3, 0, 0), 75 * RAD, -5 * RAD, _V(0.3, 0, 0), -20 * RAD, -27 * RAD);
		oapiVCSetNeighbours(-1, 1, 2, 3);

		
		break;
	case 1: // pilot position
		SetCameraOffset(_V(0.41, 1.6, 2.241));
		SetCameraDefaultDirection(_V(0, 0, 1));
		SetCameraMovement(_V(0, 0, 0.3), 0, 0, _V(-0.3, 0, 0), 20 * RAD, -27 * RAD, _V(0.3, 0, 0), -75 * RAD, -5 * RAD);
		oapiVCSetNeighbours(0, 1, 2, 3);

		
		break;
	case 2: // pilot position
		SetCameraOffset(_V(0, 1.7, 2.21));
		SetCameraDefaultDirection(_V(0, 0, 1));
		SetCameraMovement(_V(0, 0, 0.3), 0, 0, _V(-0.3, 0, 0), 20 * RAD, -27 * RAD, _V(0.3, 0, 0), -75 * RAD, -5 * RAD);
		oapiVCSetNeighbours(0, 1, 2, 3);


		break;
	case 3: // pilot position
		SetCameraOffset(_V(0, .2, 2.64));
		SetCameraDefaultDirection(_V(0, 0, 1));
		SetCameraMovement(_V(0, 0, 0.3), 0, 0, _V(-0.3, 0, 0), 20 * RAD, -27 * RAD, _V(0.3, 0, 0), -75 * RAD, -5 * RAD);
		oapiVCSetNeighbours(0, 1, 2, 3);

		break;
	}

	return HandleLoadVC(id);

}

bool TALON::HandleLoadVC(UINT const id)
{



	return true;
}

	bool TALON::clbkVCMouseEvent(int id, int event, VECTOR3 &p)
	{
		bool result = false;
		result = result || mfdController->HandleMouseEvent(id, event, p);
		result = result || VCMouseEvent(id, event, p);
		return result;
	}

	bool TALON::VCMouseEvent(int const id, int const event, VECTOR3& const p)
	{

		switch (id) {


		case AID_NAVMODE1:
			ToggleNavmode(NAVMODE_KILLROT);
			return true;
		case AID_NAVMODE2:
			ToggleNavmode(NAVMODE_HLEVEL);
			return true;
		case AID_NAVMODE3:
			ToggleNavmode(NAVMODE_PROGRADE);
			return true;
		case AID_NAVMODE4:
			ToggleNavmode(NAVMODE_RETROGRADE);
			return true;
		case AID_NAVMODE5:
			ToggleNavmode(NAVMODE_NORMAL);
			return true;
		case AID_NAVMODE6:
			ToggleNavmode(NAVMODE_ANTINORMAL);
			return true;
		case AID_NAVMODE7:
			ToggleNavmode(NAVMODE_HOLDALT);
			return true;

		}
		return false;
	}



	bool TALON::clbkVCRedrawEvent(int id, int event, SURFHANDLE surf) {
		bool result = false;
		result = result || mfdController->HandleRedrawEvent(id, event, surf);


		result = result || VCRedrawEvent(id, event, surf);
		{}

	
	return result;
}

	bool TALON::VCRedrawEvent(int const id, int const event, SURFHANDLE const surf) {
		return false;
	}

	void TALON::clbkMFDMode(int mfd, int mode)
	{
		mfdController->HandleMFDMode(mfd, mode);
	}

	
	void TALON::SetThrusters1_Orbit()
	{
		ClearThrusterDefinitions();
		ClearPropellantResources();
		ph_RM = CreatePropellantResource(SM_FUEL, SM_FUEL, 1);
		SetDefaultPropellantResource(ph_RM);
		THRUSTER_HANDLE th_rcs[28], th_group[4];




		th_rcs[0] = CreateThruster(_V(6, 0, 10), _V(0, -1, 0), RCS_THRUST, ph_RM, RCS_ISP);
		AddExhaust(th_rcs[0], 1.5, .16, _V(1.768703, 1.925852, .781), _V(-.7, .7, 0), tex_rcs);
		// AddExhaust(th_rcs[0], 1, 0.1, _V(1.2234, 1.1706, .782), _V(0, 1, 0));
		// Delantero derecho inferior
		th_rcs[1] = CreateThruster(_V(6, 0, 10), _V(0, 1, 0), RCS_THRUST, ph_RM, RCS_ISP);
		AddExhaust(th_rcs[1], 1.5, .16, _V(1.933633, 1.756633, .781), _V(.7, -.7, 0), tex_rcs);
		// Delantero derecho proa
		th_rcs[2] = CreateThruster(_V(6, 0, 10), _V(0, 0, -1), RCS_THRUST, ph_RM, RCS_ISP);
		AddExhaust(th_rcs[2], 1.5, 0.16, _V(1.854029, 1.835438, .984), _V(0, 0, 1), tex_rcs);
		//AddExhaust(th_rcs[2], .5, .01, _V(1.1555, 1.2106, .766), _V(-0.464, 0, .8858));
		//AddExhaust(th_rcs[2], .5, 0.01, _V(1.148, 1.2179, .782), _V(0, 0, 1));
		// Delantero derecho popa
		th_rcs[3] = CreateThruster(_V(6, 0, 10), _V(0, 0, 1), RCS_THRUST, ph_RM, RCS_ISP);
		AddExhaust(th_rcs[3], 1.5, 0.16, _V(1.854029, 1.835438, .574), _V(0, 0, -1), tex_rcs);
		// 1.854029  1.835438
		//	AddExhaust(th_rcs[3], 1, .10, _V(0.922, .046, -7.5872), _V(0, 0, -1));
		// Delantero derecho exterior
		th_rcs[4] = CreateThruster(_V(6, 0, 10), _V(-1, 0, 0), RCS_THRUST, ph_RM, RCS_ISP);
		//	AddExhaust(th_rcs[4], 1, 0.1, _V(2.1046, 1.0445, 8.739), _V(1, 0, 0));
		//	AddExhaust(th_rcs[4], 1, 0.1, _V(2.1044, 1.0706, 9.2092), _V(1, 0, 0));
		AddExhaust(th_rcs[4], 2.0, 0.16, _V(1.903803, 1.884322, .781), _V(.7, .7, 0), tex_rcs);
		// Grupo delantero izquierdo

		// Delantero izquierdo superior
		th_rcs[5] = CreateThruster(_V(-6, 0, 10), _V(0, -1, 0), RCS_THRUST, ph_RM, RCS_ISP);
		AddExhaust(th_rcs[5], 1.5, .16, _V(-1.749317, 1.937592, .781), _V(.7, .7, 0), tex_rcs);

		//AddExhaust(th_rcs[5], 1, 0.1, _V(-1.2234, 1.1706, .782), _V(0, 1, 0));
		// Delantero izquierdo inferior
		th_rcs[6] = CreateThruster(_V(-6, 0, 10), _V(0, 1, 0), RCS_THRUST, ph_RM, RCS_ISP);
		AddExhaust(th_rcs[6], 1.5, .16, _V(-1.926829, 1.758356, .781), _V(-.7, -.7, 0), tex_rcs);
		// Delantero izquierdo proa
		th_rcs[7] = CreateThruster(_V(-6, 0, 10), _V(0, 0, -1), RCS_THRUST, ph_RM, RCS_ISP);
		//	AddExhaust(th_rcs[7], 1, .10, _V(-1.7589, .991, 9.6925), _V(0, 0, 1));
		AddExhaust(th_rcs[7], 1.5, 0.16, _V(-1.84152, 1.846496, .984), _V(0, 0, 1), tex_rcs);

		//AddExhaust(th_rcs[7], .5, 0.01, _V(-1.148, 1.2179, .782), _V(0, 0, 1));
		// Delantero izquierdo popa
		th_rcs[8] = CreateThruster(_V(-6, 0, 10), _V(0, 0, 1), RCS_THRUST, ph_RM, RCS_ISP);
		//	AddExhaust(th_rcs[8], 1, .10, _V(-0.922, .046, -7.5872), _V(0, 0, -1));
		AddExhaust(th_rcs[8], 1.5, 0.16, _V(-1.84152, 1.846496, .574), _V(0, 0, -1), tex_rcs);

		//AddExhaust(th_rcs[8], .5, 0.01, _V(-1.148, 1.2179, .782), _V(0, 0, -1));
		// Delantero izquierdo exterior
		th_rcs[9] = CreateThruster(_V(-6, 0, 10), _V(1, 0, 0), RCS_THRUST, ph_RM, RCS_ISP);
		//	AddExhaust(th_rcs[9], 1, 0.1, _V(-2.1046, 1.0445, 8.739), _V(-1, 0, 0));
		//	AddExhaust(th_rcs[9], 1, 0.1, _V(-2.1044, 1.0706, 9.2092), _V(-1, 0, 0));
		AddExhaust(th_rcs[9], 1.5, 0.16, _V(-1.839897, 1.846103, .781), _V(-.7, .7, 0), tex_rcs);


		// Grupo trasero derecho

		// Trasero derecho superior
		th_rcs[10] = CreateThruster(_V(6, 0, -10), _V(0, -1, 0), RCS_THRUST, ph_RM, RCS_ISP);
		//	AddExhaust(th_rcs[10], 1, .10, _V(3.4827, .3, -7.4698), _V(0, 1, 0));

		AddExhaust(th_rcs[10], 1.5, 0.16, _V(1.917134, -1.784073, .781), _V(.7, .7, 0), tex_rcs);
		//AddExhaust(th_rcs[10], 1, 0.1, _V(1.2234, 1.1706, .782), _V(0, 1, 0));
		// Trasero derecho inferior
		th_rcs[11] = CreateThruster(_V(6, 0, -10), _V(0, 1, 0), RCS_THRUST, ph_RM, RCS_ISP);

		AddExhaust(th_rcs[11], 1.5, 0.16, _V(1.764258, -1.938728, .781), _V(-.7, -.7, 0), tex_rcs);
		//	AddExhaust(th_rcs[11], 1, .10, _V(3.2594, -0.092, -7.6734), _V(0, -1, 0));
		// Trasero derecho proa
		th_rcs[12] = CreateThruster(_V(6, 0, -10), _V(0, 0, -1), RCS_THRUST, ph_RM, RCS_ISP);
		//AddExhaust(th_rcs[12], 1, .10, _V(1.68, -1.67, -.575), _V(.577, -.577, -.577));
		AddExhaust(th_rcs[12], 1.5, 0.16, _V(1.841586, -1.8576, .984), _V(0, 0, 1), tex_rcs);

		//AddExhaust(th_rcs[12], .5, 0.01, _V(1.148, 1.2179, -.782), _V(0, 0, 1));
		// Trasero derecho popa
		th_rcs[13] = CreateThruster(_V(6, 0, -10), _V(0, 0, 1), RCS_THRUST, ph_RM, RCS_ISP);
		//	AddExhaust(th_rcs[13], 1, .10, _V(1.1948, -.120, -9.6909), _V(0, 0, -1));
		AddExhaust(th_rcs[13], 1.5, 0.16, _V(1.841586, -1.8576, .574), _V(0, 0, -1), tex_rcs);

		//AddExhaust(th_rcs[13], .5, 0.01, _V(1.148, 1.2179, -.782), _V(0, 0, -1));
		// Trasero derecho exterior
		th_rcs[14] = CreateThruster(_V(6, 0, -10), _V(-1, 0, 0), RCS_THRUST, ph_RM, RCS_ISP);
		AddExhaust(th_rcs[14], 1.5, 0.16, _V(1.888693, -1.909152, .781), _V(.7, -.7, 0), tex_rcs);
		//	AddExhaust(th_rcs[14], 1, 0.1, _V(3.5573, .195, -7.2458), _V(1, 0, 0));

		// Grupo trasero izquierdo

		// Trasero izquierdo superior
		th_rcs[15] = CreateThruster(_V(-6, 0, -10), _V(0, -1, 0), RCS_THRUST, ph_RM, RCS_ISP);
		AddExhaust(th_rcs[15], 1.5, 0.16, _V(-1.928779, -1.766297, .781), _V(-.7, .7, 0), tex_rcs);
		// Trasero izquierdo inferior
		th_rcs[16] = CreateThruster(_V(-6, 0, -10), _V(0, 1, 0), RCS_THRUST, ph_RM, RCS_ISP);
		//	AddExhaust(th_rcs[16], 1, .10, _V(-3.2594, -0.092, -7.6734), _V(0, -1, 0));
		AddExhaust(th_rcs[16], 1.5, 0.16, _V(-1.767014, -1.929839, .781), _V(.7, -.7, 0), tex_rcs);

		// Trasero izquierdo proa
		th_rcs[17] = CreateThruster(_V(-6, 0, -10), _V(0, 0, -1), RCS_THRUST, ph_RM, RCS_ISP);
		AddExhaust(th_rcs[17], 1.5, 0.16, _V(-1.851452, -1.845157, .984), _V(0, 0, 1), tex_rcs);

		//AddExhaust(th_rcs[17], .5, 0.01, _V(-1.148, 1.2179, -.782), _V(0, 0, 1));
		// Trasero izquierdo popa
		th_rcs[18] = CreateThruster(_V(-6, 0, -10), _V(0, 0, 1), RCS_THRUST, ph_RM, RCS_ISP);
		//	AddExhaust(th_rcs[18], 1, .10, _V(-1.1948, -.120, -9.6909), _V(0, 0, -1));
		AddExhaust(th_rcs[18], 1.5, 0.16, _V(-1.851452, -1.845157, .574), _V(0, 0, -1), tex_rcs);

		// Trasero izquierdo exterior
		th_rcs[19] = CreateThruster(_V(-6, 0, -10), _V(1, 0, 0), RCS_THRUST, ph_RM, RCS_ISP);
		//	AddExhaust(th_rcs[19], 1, 0.1, _V(-3.5573, .195, -6.9846), _V(-1, 0, 0));
		//	AddExhaust(th_rcs[19], 1, 0.1, _V(-3.5573, .195, -7.2458), _V(-1, 0, 0));
		AddExhaust(th_rcs[19], 1.5, 0.16, _V(-1.851452, -1.846046, .781), _V(-.7, -.7, 0), tex_rcs);
		//pitchrcs
		th_rcs[20] = CreateThruster(_V(6, 0, 10), _V(0, -1, 0), RCS_THRUST, ph_RM, RCS_ISP);//0
		AddExhaust(th_rcs[20], 1.5, 0.16, _V(1.854029, 1.835438, .574), _V(0, 0, -1), tex_rcs);

		th_rcs[21] = CreateThruster(_V(6, 0, 10), _V(0, 1, 0), RCS_THRUST, ph_RM, RCS_ISP);//1
		AddExhaust(th_rcs[21], 1.5, 0.16, _V(1.854029, 1.835438, .984), _V(0, 0, 1), tex_rcs);

		th_rcs[22] = CreateThruster(_V(-6, 0, 10), _V(0, -1, 0), RCS_THRUST, ph_RM, RCS_ISP);//5
		AddExhaust(th_rcs[22], 1.5, 0.16, _V(-1.84152, 1.846496, .574), _V(0, 0, -1), tex_rcs);

		th_rcs[23] = CreateThruster(_V(-6, 0, 10), _V(0, 1, 0), RCS_THRUST, ph_RM, RCS_ISP);//6
		AddExhaust(th_rcs[23], 1.5, 0.16, _V(-1.84152, 1.846496, .984), _V(0, 0, 1), tex_rcs);

		th_rcs[24] = CreateThruster(_V(6, 0, -10), _V(0, -1, 0), RCS_THRUST, ph_RM, RCS_ISP);//10
		AddExhaust(th_rcs[24], 1.5, 0.16, _V(1.841586, -1.8576, .574), _V(0, 0, -1), tex_rcs);

		th_rcs[25] = CreateThruster(_V(6, 0, -10), _V(0, 1, 0), RCS_THRUST, ph_RM, RCS_ISP);//11
		AddExhaust(th_rcs[25], 1.5, 0.16, _V(1.841586, -1.8576, .984), _V(0, 0, 1), tex_rcs);

		th_rcs[26] = CreateThruster(_V(-6, 0, -10), _V(0, -1, 0), RCS_THRUST, ph_RM, RCS_ISP);//15
		AddExhaust(th_rcs[26], 1.5, 0.16, _V(-1.851452, -1.845157, .574), _V(0, 0, -1), tex_rcs);

		// Trasero izquierdo inferior
		th_rcs[27] = CreateThruster(_V(-6, 0, -10), _V(0, 1, 0), RCS_THRUST, ph_RM, RCS_ISP);//16
		AddExhaust(th_rcs[27], 1.5, 0.16, _V(-1.851452, -1.845157, .984), _V(0, 0, 1), tex_rcs);






		////visual rcs

		// Grupo motor de cabeceo hacia arriba "PITCH-UP"1,6,10,15
		th_group[0] = th_rcs[21];
		th_group[1] = th_rcs[23];
		th_group[2] = th_rcs[24];
		th_group[3] = th_rcs[26];
		//th_group[0] = th_rcs[3];
		//th_group[1] = th_rcs[8];
		//th_group[2] = th_rcs[12];
		//th_group[3] = th_rcs[17];
		CreateThrusterGroup(th_group, 4, THGROUP_ATT_PITCHUP);

		// Grupo motor de cabeceo hacia abajo "PITCH-DOWN" 0,5,11,16
		th_group[0] = th_rcs[20];

		th_group[1] = th_rcs[22];
		th_group[2] = th_rcs[25];
		th_group[3] = th_rcs[27];
		//th_group[4] = th_rcs[21];
		//th_group[0] = th_rcs[2];
		//th_group[1] = th_rcs[7];
		//th_group[2] = th_rcs[13];
		//th_group[3] = th_rcs[18];
		CreateThrusterGroup(th_group, 4, THGROUP_ATT_PITCHDOWN);

		// Grupo motor de balanceo izquierda "YAW-RIGHT"
		th_group[0] = th_rcs[8];
		th_group[1] = th_rcs[18];
		th_group[2] = th_rcs[2];
		th_group[3] = th_rcs[12];
		CreateThrusterGroup(th_group, 4, THGROUP_ATT_YAWRIGHT);

		// Grupo motor de balanceo derecha "YAW-LEFT"
		th_group[0] = th_rcs[3];
		th_group[1] = th_rcs[13];
		th_group[2] = th_rcs[7];
		th_group[3] = th_rcs[17];
		CreateThrusterGroup(th_group, 4, THGROUP_ATT_YAWLEFT);

		// Grupo motor de alabeo derecho "BANK-RIGHT"
		th_group[0] = th_rcs[0];
		th_group[1] = th_rcs[6];
		th_group[2] = th_rcs[10];
		th_group[3] = th_rcs[16];
		CreateThrusterGroup(th_group, 4, THGROUP_ATT_BANKRIGHT);

		// Grupo motor de alabeo izquierdo "BANK-LEFT"
		th_group[0] = th_rcs[1];
		th_group[1] = th_rcs[5];
		th_group[2] = th_rcs[11];
		th_group[3] = th_rcs[15];
		CreateThrusterGroup(th_group, 4, THGROUP_ATT_BANKLEFT);

		// 2-Movimientos traslacionales

		// Grupo motor de traslación derecha "RIGHT"
		// Este grupo lo podían formar los impulsores izquierdos delantero de proa y trasero de popa
		th_group[0] = th_rcs[9];
		th_group[1] = th_rcs[19];
		CreateThrusterGroup(th_group, 2, THGROUP_ATT_RIGHT);

		// Grupo motor de traslación izquierda "LEFT"
		// Este grupo lo podían formar los impulsores derechos delantero de proa y trasero de popa
		th_group[0] = th_rcs[4];
		th_group[1] = th_rcs[14];
		CreateThrusterGroup(th_group, 2, THGROUP_ATT_LEFT);

		// Grupo motor de traslación arriba "DOWN"
		th_group[0] = th_rcs[0];
		th_group[1] = th_rcs[5];
		th_group[2] = th_rcs[10];
		th_group[3] = th_rcs[15];
		CreateThrusterGroup(th_group, 4, THGROUP_ATT_DOWN);

		// Grupo motor de traslación abajo "UP"
		th_group[0] = th_rcs[1];
		th_group[1] = th_rcs[6];
		th_group[2] = th_rcs[11];
		th_group[3] = th_rcs[16];
		CreateThrusterGroup(th_group, 4, THGROUP_ATT_UP);

		// Grupo motor de traslación adelante "FORWARD"
		th_group[0] = th_rcs[3];
		th_group[1] = th_rcs[8];
		th_group[2] = th_rcs[13];
		th_group[3] = th_rcs[18];
		CreateThrusterGroup(th_group, 4, THGROUP_ATT_FORWARD);

		// Grupo motor de traslación atrás "BACK"
		th_group[0] = th_rcs[2];
		th_group[1] = th_rcs[7];
		th_group[2] = th_rcs[12];
		th_group[3] = th_rcs[17];
		CreateThrusterGroup(th_group, 4, THGROUP_ATT_BACK);
		THRUSTER_HANDLE  th_att_lin[2];

		/*

		THRUSTER_HANDLE th_att_rot[16], th_att_lin[16];
		// Pitchup
		th_att_rot[0] = th_att_lin[8] = CreateThruster(_V(0.06, 2.10, -1.49), _V(0, -0.7, -0.7), RCS_THRUST, ph_RM, RCS_ISP);
		th_att_rot[1] = th_att_lin[9] = CreateThruster(_V(-0.06, 2.10, -1.49), _V(0, -0.7, -0.7), RCS_THRUST, ph_RM, RCS_ISP);

		th_att_rot[2] = th_att_lin[0] = CreateThruster(_V(0.06, -2.10, .582), _V(0, 0.7, 0.7), RCS_THRUST, ph_RM, RCS_ISP);
		th_att_rot[3] = th_att_lin[1] = CreateThruster(_V(-0.06, -2.10, -1.82), _V(0, 0.7, 0.7), RCS_THRUST, ph_RM, RCS_ISP);

		// Pitchdown
		th_att_rot[4] = th_att_lin[2] = CreateThruster(_V(0.06, 2.10, -1.82), _V(0, -0.7, 0.7), RCS_THRUST, ph_RM, RCS_ISP);
		th_att_rot[5] = th_att_lin[3] = CreateThruster(_V(-0.06, 2.10, -1.82), _V(0, -0.7, 0.7), RCS_THRUST, ph_RM, RCS_ISP);

		th_att_rot[6] = th_att_lin[10] = CreateThruster(_V(0.06, -2.10, -1.49), _V(0, 0.7, -0.7), RCS_THRUST, ph_RM, RCS_ISP);
		th_att_rot[7] = th_att_lin[11] = CreateThruster(_V(-0.06, -2.10, -1.49), _V(0, 0.7, -0.7), RCS_THRUST, ph_RM, RCS_ISP);

		// Yawleft
		th_att_rot[8] = th_att_lin[4] = CreateThruster(_V(2.10, 0.06, -1.82), _V(-0.7, 0, 0.7), RCS_THRUST, ph_RM, RCS_ISP);
		th_att_rot[9] = th_att_lin[5] = CreateThruster(_V(2.10, -0.06, -1.82), _V(-0.7, 0, 0.7), RCS_THRUST, ph_RM, RCS_ISP);

		th_att_rot[10] = th_att_lin[12] = CreateThruster(_V(-2.10, 0.06, -1.49), _V(0.7, 0, -0.7), RCS_THRUST, ph_RM, RCS_ISP);
		th_att_rot[11] = th_att_lin[13] = CreateThruster(_V(-2.10, -0.06, -1.49), _V(0.7, 0, -0.7), RCS_THRUST, ph_RM, RCS_ISP);

		// Yawright
		th_att_rot[12] = th_att_lin[14] = CreateThruster(_V(2.10, 0.06, -1.49), _V(-0.7, 0, -0.7), RCS_THRUST, ph_RM, RCS_ISP);
		th_att_rot[13] = th_att_lin[15] = CreateThruster(_V(2.10, -0.06, -1.49), _V(-0.7, 0, -0.7), RCS_THRUST, ph_RM, RCS_ISP);

		th_att_rot[14] = th_att_lin[6] = CreateThruster(_V(-2.10, 0.06, -1.82), _V(0.7, 0, 0.7), RCS_THRUST, ph_RM, RCS_ISP);
		th_att_rot[15] = th_att_lin[7] = CreateThruster(_V(-2.10, -0.06, -1.82), _V(0.7, 0, 0.7), RCS_THRUST, ph_RM, RCS_ISP);

		CreateThrusterGroup(th_att_rot, 4, THGROUP_ATT_PITCHUP);
		CreateThrusterGroup(th_att_rot + 4, 4, THGROUP_ATT_PITCHDOWN);

		CreateThrusterGroup(th_att_rot + 8, 4, THGROUP_ATT_YAWLEFT);
		CreateThrusterGroup(th_att_rot + 12, 4, THGROUP_ATT_YAWRIGHT);

		CreateThrusterGroup(th_att_lin, 8, THGROUP_ATT_FORWARD);
		CreateThrusterGroup(th_att_lin + 8, 8, THGROUP_ATT_BACK);

		for (int i = 0; i<16; i++) {
		AddExhaust(th_att_rot[i], R_SIZE, tex_rcs);
		AddExhaustStream(th_att_rot[i], &rcs_stream);
		}

		// Bankleft
		th_att_rot[0] = th_att_lin[0] = th_att_lin[12] = CreateThruster(_V(-0.18, -2.10, 0.06), _V(0.7, 0.7, 0), RCS_THRUST, ph_RM, RCS_ISP);
		th_att_rot[1] = th_att_lin[1] = th_att_lin[13] = CreateThruster(_V(-0.18, -2.10, -0.06), _V(0.7, 0.7, 0), RCS_THRUST, ph_RM, RCS_ISP);

		th_att_rot[2] = th_att_lin[4] = th_att_lin[8] = CreateThruster(_V(0.18, 2.10, 0.06), _V(-0.7, -0.7, 0), RCS_THRUST, ph_RM, RCS_ISP);
		th_att_rot[3] = th_att_lin[5] = th_att_lin[9] = CreateThruster(_V(0.18, 2.10, -0.06), _V(-0.7, -0.7, 0), RCS_THRUST, ph_RM, RCS_ISP);

		// Bankright
		th_att_rot[4] = th_att_lin[2] = th_att_lin[10] = CreateThruster(_V(0.18, -2.10, 0.06), _V(-0.7, 0.7, 0), RCS_THRUST, ph_RM, RCS_ISP);
		th_att_rot[5] = th_att_lin[3] = th_att_lin[11] = CreateThruster(_V(0.18, -2.10, -0.06), _V(-0.7, 0.7, 0), RCS_THRUST, ph_RM, RCS_ISP);

		th_att_rot[6] = th_att_lin[6] = th_att_lin[14] = CreateThruster(_V(-0.18, 2.10, 0.06), _V(0.7, -0.7, 0), RCS_THRUST, ph_RM, RCS_ISP);
		th_att_rot[7] = th_att_lin[7] = th_att_lin[15] = CreateThruster(_V(-0.18, 2.10, -0.06), _V(0.7, -0.7, 0), RCS_THRUST, ph_RM, RCS_ISP);

		CreateThrusterGroup(th_att_rot, 4, THGROUP_ATT_BANKLEFT);
		CreateThrusterGroup(th_att_rot + 4, 4, THGROUP_ATT_BANKRIGHT);

		CreateThrusterGroup(th_att_lin, 4, THGROUP_ATT_UP);
		CreateThrusterGroup(th_att_lin + 4, 4, THGROUP_ATT_DOWN);
		CreateThrusterGroup(th_att_lin + 8, 4, THGROUP_ATT_LEFT);
		CreateThrusterGroup(th_att_lin + 12, 4, THGROUP_ATT_RIGHT);

		int i;
		for (i = 0; i<8; i++) {
		AddExhaust(th_att_rot[i], R_SIZE, tex_rcs);
		AddExhaustStream(th_att_rot[i], &rcs_stream);
		}
		*/
		// main
		th_att_lin[0] = CreateThruster(_V(0, 0, -3.5), _V(0, 0, 1), MAIN_THRUST, ph_RM, MAIN_ISP);
		CreateThrusterGroup(th_att_lin, 1, THGROUP_MAIN);
		AddExhaust(th_att_lin[0], MAIN_ENGSIZE, tex_main);

		// main dummy for exhaust stream
		th_att_lin[1] = CreateThruster(_V(0, 0, -4.5), _V(0, 0, 1), 0, ph_RM, MAIN_ISP);
		CreateThrusterGroup(th_att_lin, 2, THGROUP_MAIN);
		AddExhaustStream(th_att_lin[1], &main_stream);


		ph_descent = CreatePropellantResource(LEM_DescentFUEL);	// Descent stage propellant tank
		// Define Thrusters: Main Engines 
		SetDefaultPropellantResource(ph_descent);// Set descent tanks as default propellant resource
		th_descent[0] = CreateThruster(_V(0.0, 0, 0), _V(0, 1, 0), LEM_DescentTHRUST, ph_descent, LEM_DescentISP); // Orbiral stage engine
		SetThrusterResource(th_descent[0], ph_descent);

		th_group[0] = th_descent[0];

		CreateThrusterGroup(th_group, 1, THGROUP_HOVER);

		AddExhaust(th_descent[0], MAIN_ENGSIZE, tex_main);





		/*
		// aux
		th_aux[0] = CreateThruster(_V(1.3, 0.16, -2.4), _V(0, 0, 1), AUX_THRUST, ph_RM, RCS_ISP);
		th_aux[1] = CreateThruster(_V(1.3, -0.16, -2.4), _V(0, 0, 1), AUX_THRUST, ph_RM, RCS_ISP);
		th_aux[2] = CreateThruster(_V(-1.3, 0.16, -2.4), _V(0, 0, 1), AUX_THRUST, ph_RM, RCS_ISP);
		th_aux[3] = CreateThruster(_V(-1.3, -0.16, -2.4), _V(0, 0, 1), AUX_THRUST, ph_RM, RCS_ISP);
		th_aux[4] = CreateThruster(_V(0.16, 1.3, -2.4), _V(0, 0, 1), AUX_THRUST, ph_RM, RCS_ISP);
		th_aux[5] = CreateThruster(_V(-0.16, 1.3, -2.4), _V(0, 0, 1), AUX_THRUST, ph_RM, RCS_ISP);
		th_aux[6] = CreateThruster(_V(0.16, -1.3, -2.4), _V(0, 0, 1), AUX_THRUST, ph_RM, RCS_ISP);
		th_aux[7] = CreateThruster(_V(-0.16, -1.3, -2.4), _V(0, 0, 1), AUX_THRUST, ph_RM, RCS_ISP);

		CreateThrusterGroup(th_aux, 1, THGROUP_USER);

		for (i = 0; i<8; i++) {
		AddExhaust(th_aux[i], AUX_SIZE, tex_rcs);
		AddExhaustStream(th_aux[i], &rcs_stream);
		}

		*/
	}

	//LAUNCHMOON
	void TALON::SetThrusters1_Surface()
	{
		ClearThrusterDefinitions();
		ClearPropellantResources();
		ph_RM = CreatePropellantResource(SM_FUEL, SM_FUEL, 1);
		SetDefaultPropellantResource(ph_RM);
		THRUSTER_HANDLE th_rcs[28], th_group[4];




		th_rcs[0] = CreateThruster(_V(6, 0, 10), _V(0, -1, 0), RCS_THRUST_LIFT_OFF, ph_RM, RCS_ISP);
		AddExhaust(th_rcs[0], 1.5, .16, _V(1.768703, 1.925852, .781), _V(-.7, .7, 0), tex_rcs);
		// AddExhaust(th_rcs[0], 1, 0.1, _V(1.2234, 1.1706, .782), _V(0, 1, 0));
		// Delantero derecho inferior
		th_rcs[1] = CreateThruster(_V(6, 0, 10), _V(0, 1, 0), RCS_THRUST_LIFT_OFF, ph_RM, RCS_ISP);
		AddExhaust(th_rcs[1], 1.5, .16, _V(1.933633, 1.756633, .781), _V(.7, -.7, 0), tex_rcs);
		// Delantero derecho proa
		th_rcs[2] = CreateThruster(_V(6, 0, 10), _V(0, 0, -1), RCS_THRUST_LIFT_OFF, ph_RM, RCS_ISP);
		AddExhaust(th_rcs[2], 1.5, 0.16, _V(1.854029, 1.835438, .984), _V(0, 0, 1), tex_rcs);
		//AddExhaust(th_rcs[2], .5, .01, _V(1.1555, 1.2106, .766), _V(-0.464, 0, .8858));
		//AddExhaust(th_rcs[2], .5, 0.01, _V(1.148, 1.2179, .782), _V(0, 0, 1));
		// Delantero derecho popa
		th_rcs[3] = CreateThruster(_V(6, 0, 10), _V(0, 0, 1), RCS_THRUST_LIFT_OFF, ph_RM, RCS_ISP);
		AddExhaust(th_rcs[3], 1.5, 0.16, _V(1.854029, 1.835438, .574), _V(0, 0, -1), tex_rcs);
		// 1.854029  1.835438
		//	AddExhaust(th_rcs[3], 1, .10, _V(0.922, .046, -7.5872), _V(0, 0, -1));
		// Delantero derecho exterior
		th_rcs[4] = CreateThruster(_V(6, 0, 10), _V(-1, 0, 0), RCS_THRUST_LIFT_OFF, ph_RM, RCS_ISP);
		//	AddExhaust(th_rcs[4], 1, 0.1, _V(2.1046, 1.0445, 8.739), _V(1, 0, 0));
		//	AddExhaust(th_rcs[4], 1, 0.1, _V(2.1044, 1.0706, 9.2092), _V(1, 0, 0));
		AddExhaust(th_rcs[4], 2.0, 0.16, _V(1.903803, 1.884322, .781), _V(.7, .7, 0), tex_rcs);
		// Grupo delantero izquierdo

		// Delantero izquierdo superior
		th_rcs[5] = CreateThruster(_V(-6, 0, 10), _V(0, -1, 0), RCS_THRUST_LIFT_OFF, ph_RM, RCS_ISP);
		AddExhaust(th_rcs[5], 1.5, .16, _V(-1.749317, 1.937592, .781), _V(.7, .7, 0), tex_rcs);

		//AddExhaust(th_rcs[5], 1, 0.1, _V(-1.2234, 1.1706, .782), _V(0, 1, 0));
		// Delantero izquierdo inferior
		th_rcs[6] = CreateThruster(_V(-6, 0, 10), _V(0, 1, 0), RCS_THRUST_LIFT_OFF, ph_RM, RCS_ISP);
		AddExhaust(th_rcs[6], 1.5, .16, _V(-1.926829, 1.758356, .781), _V(-.7, -.7, 0), tex_rcs);
		// Delantero izquierdo proa
		th_rcs[7] = CreateThruster(_V(-6, 0, 10), _V(0, 0, -1), RCS_THRUST_LIFT_OFF, ph_RM, RCS_ISP);
		//	AddExhaust(th_rcs[7], 1, .10, _V(-1.7589, .991, 9.6925), _V(0, 0, 1));
		AddExhaust(th_rcs[7], 1.5, 0.16, _V(-1.84152, 1.846496, .984), _V(0, 0, 1), tex_rcs);

		//AddExhaust(th_rcs[7], .5, 0.01, _V(-1.148, 1.2179, .782), _V(0, 0, 1));
		// Delantero izquierdo popa
		th_rcs[8] = CreateThruster(_V(-6, 0, 10), _V(0, 0, 1), RCS_THRUST_LIFT_OFF, ph_RM, RCS_ISP);
		//	AddExhaust(th_rcs[8], 1, .10, _V(-0.922, .046, -7.5872), _V(0, 0, -1));
		AddExhaust(th_rcs[8], 1.5, 0.16, _V(-1.84152, 1.846496, .574), _V(0, 0, -1), tex_rcs);

		//AddExhaust(th_rcs[8], .5, 0.01, _V(-1.148, 1.2179, .782), _V(0, 0, -1));
		// Delantero izquierdo exterior
		th_rcs[9] = CreateThruster(_V(-6, 0, 10), _V(1, 0, 0), RCS_THRUST_LIFT_OFF, ph_RM, RCS_ISP);
		//	AddExhaust(th_rcs[9], 1, 0.1, _V(-2.1046, 1.0445, 8.739), _V(-1, 0, 0));
		//	AddExhaust(th_rcs[9], 1, 0.1, _V(-2.1044, 1.0706, 9.2092), _V(-1, 0, 0));
		AddExhaust(th_rcs[9], 1.5, 0.16, _V(-1.839897, 1.846103, .781), _V(-.7, .7, 0), tex_rcs);


		// Grupo trasero derecho

		// Trasero derecho superior
		th_rcs[10] = CreateThruster(_V(6, 0, -10), _V(0, -1, 0), RCS_THRUST_LIFT_OFF, ph_RM, RCS_ISP);
		//	AddExhaust(th_rcs[10], 1, .10, _V(3.4827, .3, -7.4698), _V(0, 1, 0));

		AddExhaust(th_rcs[10], 1.5, 0.16, _V(1.917134, -1.784073, .781), _V(.7, .7, 0), tex_rcs);
		//AddExhaust(th_rcs[10], 1, 0.1, _V(1.2234, 1.1706, .782), _V(0, 1, 0));
		// Trasero derecho inferior
		th_rcs[11] = CreateThruster(_V(6, 0, -10), _V(0, 1, 0), RCS_THRUST_LIFT_OFF, ph_RM, RCS_ISP);

		AddExhaust(th_rcs[11], 1.5, 0.16, _V(1.764258, -1.938728, .781), _V(-.7, -.7, 0), tex_rcs);
		//	AddExhaust(th_rcs[11], 1, .10, _V(3.2594, -0.092, -7.6734), _V(0, -1, 0));
		// Trasero derecho proa
		th_rcs[12] = CreateThruster(_V(6, 0, -10), _V(0, 0, -1), RCS_THRUST_LIFT_OFF, ph_RM, RCS_ISP);
		//AddExhaust(th_rcs[12], 1, .10, _V(1.68, -1.67, -.575), _V(.577, -.577, -.577));
		AddExhaust(th_rcs[12], 1.5, 0.16, _V(1.841586, -1.8576, .984), _V(0, 0, 1), tex_rcs);

		//AddExhaust(th_rcs[12], .5, 0.01, _V(1.148, 1.2179, -.782), _V(0, 0, 1));
		// Trasero derecho popa
		th_rcs[13] = CreateThruster(_V(6, 0, -10), _V(0, 0, 1), RCS_THRUST_LIFT_OFF, ph_RM, RCS_ISP);
		//	AddExhaust(th_rcs[13], 1, .10, _V(1.1948, -.120, -9.6909), _V(0, 0, -1));
		AddExhaust(th_rcs[13], 1.5, 0.16, _V(1.841586, -1.8576, .574), _V(0, 0, -1), tex_rcs);

		//AddExhaust(th_rcs[13], .5, 0.01, _V(1.148, 1.2179, -.782), _V(0, 0, -1));
		// Trasero derecho exterior
		th_rcs[14] = CreateThruster(_V(6, 0, -10), _V(-1, 0, 0), RCS_THRUST_LIFT_OFF, ph_RM, RCS_ISP);
		AddExhaust(th_rcs[14], 1.5, 0.16, _V(1.888693, -1.909152, .781), _V(.7, -.7, 0), tex_rcs);
		//	AddExhaust(th_rcs[14], 1, 0.1, _V(3.5573, .195, -7.2458), _V(1, 0, 0));

		// Grupo trasero izquierdo

		// Trasero izquierdo superior
		th_rcs[15] = CreateThruster(_V(-6, 0, -10), _V(0, -1, 0), RCS_THRUST_LIFT_OFF, ph_RM, RCS_ISP);
		AddExhaust(th_rcs[15], 1.5, 0.16, _V(-1.928779, -1.766297, .781), _V(-.7, .7, 0), tex_rcs);
		// Trasero izquierdo inferior
		th_rcs[16] = CreateThruster(_V(-6, 0, -10), _V(0, 1, 0), RCS_THRUST_LIFT_OFF, ph_RM, RCS_ISP);
		//	AddExhaust(th_rcs[16], 1, .10, _V(-3.2594, -0.092, -7.6734), _V(0, -1, 0));
		AddExhaust(th_rcs[16], 1.5, 0.16, _V(-1.767014, -1.929839, .781), _V(.7, -.7, 0), tex_rcs);

		// Trasero izquierdo proa
		th_rcs[17] = CreateThruster(_V(-6, 0, -10), _V(0, 0, -1), RCS_THRUST_LIFT_OFF, ph_RM, RCS_ISP);
		AddExhaust(th_rcs[17], 1.5, 0.16, _V(-1.851452, -1.845157, .984), _V(0, 0, 1), tex_rcs);

		//AddExhaust(th_rcs[17], .5, 0.01, _V(-1.148, 1.2179, -.782), _V(0, 0, 1));
		// Trasero izquierdo popa
		th_rcs[18] = CreateThruster(_V(-6, 0, -10), _V(0, 0, 1), RCS_THRUST_LIFT_OFF, ph_RM, RCS_ISP);
		//	AddExhaust(th_rcs[18], 1, .10, _V(-1.1948, -.120, -9.6909), _V(0, 0, -1));
		AddExhaust(th_rcs[18], 1.5, 0.16, _V(-1.851452, -1.845157, .574), _V(0, 0, -1), tex_rcs);

		// Trasero izquierdo exterior
		th_rcs[19] = CreateThruster(_V(-6, 0, -10), _V(1, 0, 0), RCS_THRUST_LIFT_OFF, ph_RM, RCS_ISP);
		//	AddExhaust(th_rcs[19], 1, 0.1, _V(-3.5573, .195, -6.9846), _V(-1, 0, 0));
		//	AddExhaust(th_rcs[19], 1, 0.1, _V(-3.5573, .195, -7.2458), _V(-1, 0, 0));
		AddExhaust(th_rcs[19], 1.5, 0.16, _V(-1.851452, -1.846046, .781), _V(-.7, -.7, 0), tex_rcs);
		//pitchrcs
		th_rcs[20] = CreateThruster(_V(6, 0, 10), _V(0, -1, 0), RCS_THRUST_LIFT_OFF, ph_RM, RCS_ISP);//0
		AddExhaust(th_rcs[20], 1.5, 0.16, _V(1.854029, 1.835438, .574), _V(0, 0, -1), tex_rcs);

		th_rcs[21] = CreateThruster(_V(6, 0, 10), _V(0, 1, 0), RCS_THRUST_LIFT_OFF, ph_RM, RCS_ISP);//1
		AddExhaust(th_rcs[21], 1.5, 0.16, _V(1.854029, 1.835438, .984), _V(0, 0, 1), tex_rcs);

		th_rcs[22] = CreateThruster(_V(-6, 0, 10), _V(0, -1, 0), RCS_THRUST_LIFT_OFF, ph_RM, RCS_ISP);//5
		AddExhaust(th_rcs[22], 1.5, 0.16, _V(-1.84152, 1.846496, .574), _V(0, 0, -1), tex_rcs);

		th_rcs[23] = CreateThruster(_V(-6, 0, 10), _V(0, 1, 0), RCS_THRUST_LIFT_OFF, ph_RM, RCS_ISP);//6
		AddExhaust(th_rcs[23], 1.5, 0.16, _V(-1.84152, 1.846496, .984), _V(0, 0, 1), tex_rcs);

		th_rcs[24] = CreateThruster(_V(6, 0, -10), _V(0, -1, 0), RCS_THRUST_LIFT_OFF, ph_RM, RCS_ISP);//10
		AddExhaust(th_rcs[24], 1.5, 0.16, _V(1.841586, -1.8576, .574), _V(0, 0, -1), tex_rcs);

		th_rcs[25] = CreateThruster(_V(6, 0, -10), _V(0, 1, 0), RCS_THRUST_LIFT_OFF, ph_RM, RCS_ISP);//11
		AddExhaust(th_rcs[25], 1.5, 0.16, _V(1.841586, -1.8576, .984), _V(0, 0, 1), tex_rcs);

		th_rcs[26] = CreateThruster(_V(-6, 0, -10), _V(0, -1, 0), RCS_THRUST_LIFT_OFF, ph_RM, RCS_ISP);//15
		AddExhaust(th_rcs[26], 1.5, 0.16, _V(-1.851452, -1.845157, .574), _V(0, 0, -1), tex_rcs);

		// Trasero izquierdo inferior
		th_rcs[27] = CreateThruster(_V(-6, 0, -10), _V(0, 1, 0), RCS_THRUST_LIFT_OFF, ph_RM, RCS_ISP);//16
		AddExhaust(th_rcs[27], 1.5, 0.16, _V(-1.851452, -1.845157, .984), _V(0, 0, 1), tex_rcs);






		////visual rcs

		// Grupo motor de cabeceo hacia arriba "PITCH-UP"1,6,10,15
		th_group[0] = th_rcs[21];
		th_group[1] = th_rcs[23];
		th_group[2] = th_rcs[24];
		th_group[3] = th_rcs[26];
		//th_group[0] = th_rcs[3];
		//th_group[1] = th_rcs[8];
		//th_group[2] = th_rcs[12];
		//th_group[3] = th_rcs[17];
		CreateThrusterGroup(th_group, 4, THGROUP_ATT_PITCHUP);

		// Grupo motor de cabeceo hacia abajo "PITCH-DOWN" 0,5,11,16
		th_group[0] = th_rcs[20];

		th_group[1] = th_rcs[22];
		th_group[2] = th_rcs[25];
		th_group[3] = th_rcs[27];
		//th_group[4] = th_rcs[21];
		//th_group[0] = th_rcs[2];
		//th_group[1] = th_rcs[7];
		//th_group[2] = th_rcs[13];
		//th_group[3] = th_rcs[18];
		CreateThrusterGroup(th_group, 4, THGROUP_ATT_PITCHDOWN);

		// Grupo motor de balanceo izquierda "YAW-RIGHT"
		th_group[0] = th_rcs[8];
		th_group[1] = th_rcs[18];
		th_group[2] = th_rcs[2];
		th_group[3] = th_rcs[12];
		CreateThrusterGroup(th_group, 4, THGROUP_ATT_YAWRIGHT);

		// Grupo motor de balanceo derecha "YAW-LEFT"
		th_group[0] = th_rcs[3];
		th_group[1] = th_rcs[13];
		th_group[2] = th_rcs[7];
		th_group[3] = th_rcs[17];
		CreateThrusterGroup(th_group, 4, THGROUP_ATT_YAWLEFT);

		// Grupo motor de alabeo derecho "BANK-RIGHT"
		th_group[0] = th_rcs[0];
		th_group[1] = th_rcs[6];
		th_group[2] = th_rcs[10];
		th_group[3] = th_rcs[16];
		CreateThrusterGroup(th_group, 4, THGROUP_ATT_BANKRIGHT);

		// Grupo motor de alabeo izquierdo "BANK-LEFT"
		th_group[0] = th_rcs[1];
		th_group[1] = th_rcs[5];
		th_group[2] = th_rcs[11];
		th_group[3] = th_rcs[15];
		CreateThrusterGroup(th_group, 4, THGROUP_ATT_BANKLEFT);

		// 2-Movimientos traslacionales

		// Grupo motor de traslación derecha "RIGHT"
		// Este grupo lo podían formar los impulsores izquierdos delantero de proa y trasero de popa
		th_group[0] = th_rcs[9];
		th_group[1] = th_rcs[19];
		CreateThrusterGroup(th_group, 2, THGROUP_ATT_RIGHT);

		// Grupo motor de traslación izquierda "LEFT"
		// Este grupo lo podían formar los impulsores derechos delantero de proa y trasero de popa
		th_group[0] = th_rcs[4];
		th_group[1] = th_rcs[14];
		CreateThrusterGroup(th_group, 2, THGROUP_ATT_LEFT);

		// Grupo motor de traslación arriba "DOWN"
		th_group[0] = th_rcs[0];
		th_group[1] = th_rcs[5];
		th_group[2] = th_rcs[10];
		th_group[3] = th_rcs[15];
		CreateThrusterGroup(th_group, 4, THGROUP_ATT_DOWN);

		// Grupo motor de traslación abajo "UP"
		th_group[0] = th_rcs[1];
		th_group[1] = th_rcs[6];
		th_group[2] = th_rcs[11];
		th_group[3] = th_rcs[16];
		CreateThrusterGroup(th_group, 4, THGROUP_ATT_UP);

		// Grupo motor de traslación adelante "FORWARD"
		th_group[0] = th_rcs[3];
		th_group[1] = th_rcs[8];
		th_group[2] = th_rcs[13];
		th_group[3] = th_rcs[18];
		CreateThrusterGroup(th_group, 4, THGROUP_ATT_FORWARD);

		// Grupo motor de traslación atrás "BACK"
		th_group[0] = th_rcs[2];
		th_group[1] = th_rcs[7];
		th_group[2] = th_rcs[12];
		th_group[3] = th_rcs[17];
		CreateThrusterGroup(th_group, 4, THGROUP_ATT_BACK);
		THRUSTER_HANDLE  th_att_lin[2];

		/*

		THRUSTER_HANDLE th_att_rot[16], th_att_lin[16];
		// Pitchup
		th_att_rot[0] = th_att_lin[8] = CreateThruster(_V(0.06, 2.10, -1.49), _V(0, -0.7, -0.7), RCS_THRUST, ph_RM, RCS_ISP);
		th_att_rot[1] = th_att_lin[9] = CreateThruster(_V(-0.06, 2.10, -1.49), _V(0, -0.7, -0.7), RCS_THRUST, ph_RM, RCS_ISP);

		th_att_rot[2] = th_att_lin[0] = CreateThruster(_V(0.06, -2.10, .582), _V(0, 0.7, 0.7), RCS_THRUST, ph_RM, RCS_ISP);
		th_att_rot[3] = th_att_lin[1] = CreateThruster(_V(-0.06, -2.10, -1.82), _V(0, 0.7, 0.7), RCS_THRUST, ph_RM, RCS_ISP);

		// Pitchdown
		th_att_rot[4] = th_att_lin[2] = CreateThruster(_V(0.06, 2.10, -1.82), _V(0, -0.7, 0.7), RCS_THRUST, ph_RM, RCS_ISP);
		th_att_rot[5] = th_att_lin[3] = CreateThruster(_V(-0.06, 2.10, -1.82), _V(0, -0.7, 0.7), RCS_THRUST, ph_RM, RCS_ISP);

		th_att_rot[6] = th_att_lin[10] = CreateThruster(_V(0.06, -2.10, -1.49), _V(0, 0.7, -0.7), RCS_THRUST, ph_RM, RCS_ISP);
		th_att_rot[7] = th_att_lin[11] = CreateThruster(_V(-0.06, -2.10, -1.49), _V(0, 0.7, -0.7), RCS_THRUST, ph_RM, RCS_ISP);

		// Yawleft
		th_att_rot[8] = th_att_lin[4] = CreateThruster(_V(2.10, 0.06, -1.82), _V(-0.7, 0, 0.7), RCS_THRUST, ph_RM, RCS_ISP);
		th_att_rot[9] = th_att_lin[5] = CreateThruster(_V(2.10, -0.06, -1.82), _V(-0.7, 0, 0.7), RCS_THRUST, ph_RM, RCS_ISP);

		th_att_rot[10] = th_att_lin[12] = CreateThruster(_V(-2.10, 0.06, -1.49), _V(0.7, 0, -0.7), RCS_THRUST, ph_RM, RCS_ISP);
		th_att_rot[11] = th_att_lin[13] = CreateThruster(_V(-2.10, -0.06, -1.49), _V(0.7, 0, -0.7), RCS_THRUST, ph_RM, RCS_ISP);

		// Yawright
		th_att_rot[12] = th_att_lin[14] = CreateThruster(_V(2.10, 0.06, -1.49), _V(-0.7, 0, -0.7), RCS_THRUST, ph_RM, RCS_ISP);
		th_att_rot[13] = th_att_lin[15] = CreateThruster(_V(2.10, -0.06, -1.49), _V(-0.7, 0, -0.7), RCS_THRUST, ph_RM, RCS_ISP);

		th_att_rot[14] = th_att_lin[6] = CreateThruster(_V(-2.10, 0.06, -1.82), _V(0.7, 0, 0.7), RCS_THRUST, ph_RM, RCS_ISP);
		th_att_rot[15] = th_att_lin[7] = CreateThruster(_V(-2.10, -0.06, -1.82), _V(0.7, 0, 0.7), RCS_THRUST, ph_RM, RCS_ISP);

		CreateThrusterGroup(th_att_rot, 4, THGROUP_ATT_PITCHUP);
		CreateThrusterGroup(th_att_rot + 4, 4, THGROUP_ATT_PITCHDOWN);

		CreateThrusterGroup(th_att_rot + 8, 4, THGROUP_ATT_YAWLEFT);
		CreateThrusterGroup(th_att_rot + 12, 4, THGROUP_ATT_YAWRIGHT);

		CreateThrusterGroup(th_att_lin, 8, THGROUP_ATT_FORWARD);
		CreateThrusterGroup(th_att_lin + 8, 8, THGROUP_ATT_BACK);

		for (int i = 0; i<16; i++) {
		AddExhaust(th_att_rot[i], R_SIZE, tex_rcs);
		AddExhaustStream(th_att_rot[i], &rcs_stream);
		}

		// Bankleft
		th_att_rot[0] = th_att_lin[0] = th_att_lin[12] = CreateThruster(_V(-0.18, -2.10, 0.06), _V(0.7, 0.7, 0), RCS_THRUST, ph_RM, RCS_ISP);
		th_att_rot[1] = th_att_lin[1] = th_att_lin[13] = CreateThruster(_V(-0.18, -2.10, -0.06), _V(0.7, 0.7, 0), RCS_THRUST, ph_RM, RCS_ISP);

		th_att_rot[2] = th_att_lin[4] = th_att_lin[8] = CreateThruster(_V(0.18, 2.10, 0.06), _V(-0.7, -0.7, 0), RCS_THRUST, ph_RM, RCS_ISP);
		th_att_rot[3] = th_att_lin[5] = th_att_lin[9] = CreateThruster(_V(0.18, 2.10, -0.06), _V(-0.7, -0.7, 0), RCS_THRUST, ph_RM, RCS_ISP);

		// Bankright
		th_att_rot[4] = th_att_lin[2] = th_att_lin[10] = CreateThruster(_V(0.18, -2.10, 0.06), _V(-0.7, 0.7, 0), RCS_THRUST, ph_RM, RCS_ISP);
		th_att_rot[5] = th_att_lin[3] = th_att_lin[11] = CreateThruster(_V(0.18, -2.10, -0.06), _V(-0.7, 0.7, 0), RCS_THRUST, ph_RM, RCS_ISP);

		th_att_rot[6] = th_att_lin[6] = th_att_lin[14] = CreateThruster(_V(-0.18, 2.10, 0.06), _V(0.7, -0.7, 0), RCS_THRUST, ph_RM, RCS_ISP);
		th_att_rot[7] = th_att_lin[7] = th_att_lin[15] = CreateThruster(_V(-0.18, 2.10, -0.06), _V(0.7, -0.7, 0), RCS_THRUST, ph_RM, RCS_ISP);

		CreateThrusterGroup(th_att_rot, 4, THGROUP_ATT_BANKLEFT);
		CreateThrusterGroup(th_att_rot + 4, 4, THGROUP_ATT_BANKRIGHT);

		CreateThrusterGroup(th_att_lin, 4, THGROUP_ATT_UP);
		CreateThrusterGroup(th_att_lin + 4, 4, THGROUP_ATT_DOWN);
		CreateThrusterGroup(th_att_lin + 8, 4, THGROUP_ATT_LEFT);
		CreateThrusterGroup(th_att_lin + 12, 4, THGROUP_ATT_RIGHT);

		int i;
		for (i = 0; i<8; i++) {
		AddExhaust(th_att_rot[i], R_SIZE, tex_rcs);
		AddExhaustStream(th_att_rot[i], &rcs_stream);
		}
		*/
		// main
		th_att_lin[0] = CreateThruster(_V(0, 0, -3.5), _V(0, 0, 1), MAIN_THRUST_LIFT_OFF, ph_RM, MAIN_ISP);
		CreateThrusterGroup(th_att_lin, 1, THGROUP_MAIN);
		AddExhaust(th_att_lin[0], MAIN_ENGSIZE, tex_main);

		// main dummy for exhaust stream
		th_att_lin[1] = CreateThruster(_V(0, 0, -4.5), _V(0, 0, 1), 0, ph_RM, MAIN_ISP);
		CreateThrusterGroup(th_att_lin, 2, THGROUP_MAIN);
		AddExhaustStream(th_att_lin[1], &main_stream);


		ph_descent = CreatePropellantResource(LEM_DescentFUEL);	// Descent stage propellant tank
		// Define Thrusters: Main Engines 
		SetDefaultPropellantResource(ph_descent);// Set descent tanks as default propellant resource
		th_descent[0] = CreateThruster(_V(0.0, 0, 0), _V(0, 1, 0), LEM_DescentTHRUST, ph_descent, LEM_DescentISP); // Orbiral stage engine
		SetThrusterResource(th_descent[0], ph_descent);

		th_group[0] = th_descent[0];

		CreateThrusterGroup(th_group, 1, THGROUP_HOVER);

		AddExhaust(th_descent[0], MAIN_ENGSIZE, tex_main);





		/*
		// aux
		th_aux[0] = CreateThruster(_V(1.3, 0.16, -2.4), _V(0, 0, 1), AUX_THRUST, ph_RM, RCS_ISP);
		th_aux[1] = CreateThruster(_V(1.3, -0.16, -2.4), _V(0, 0, 1), AUX_THRUST, ph_RM, RCS_ISP);
		th_aux[2] = CreateThruster(_V(-1.3, 0.16, -2.4), _V(0, 0, 1), AUX_THRUST, ph_RM, RCS_ISP);
		th_aux[3] = CreateThruster(_V(-1.3, -0.16, -2.4), _V(0, 0, 1), AUX_THRUST, ph_RM, RCS_ISP);
		th_aux[4] = CreateThruster(_V(0.16, 1.3, -2.4), _V(0, 0, 1), AUX_THRUST, ph_RM, RCS_ISP);
		th_aux[5] = CreateThruster(_V(-0.16, 1.3, -2.4), _V(0, 0, 1), AUX_THRUST, ph_RM, RCS_ISP);
		th_aux[6] = CreateThruster(_V(0.16, -1.3, -2.4), _V(0, 0, 1), AUX_THRUST, ph_RM, RCS_ISP);
		th_aux[7] = CreateThruster(_V(-0.16, -1.3, -2.4), _V(0, 0, 1), AUX_THRUST, ph_RM, RCS_ISP);

		CreateThrusterGroup(th_aux, 1, THGROUP_USER);

		for (i = 0; i<8; i++) {
		AddExhaust(th_aux[i], AUX_SIZE, tex_rcs);
		AddExhaustStream(th_aux[i], &rcs_stream);
		}

		*/
	}




	void TALON::SetThrusters2_Reentry()
	{
		ClearThrusterDefinitions();
		ClearPropellantResources();

		ph_CM = CreatePropellantResource(CM_FUEL);	 // CM propellant
		SetDefaultPropellantResource(ph_CM);

		THRUSTER_HANDLE th_att_rot[16], th_att_lin[16];
		// Pitchup
		th_att_rot[0] = th_att_lin[2] = CreateThruster(_V(0.275, 2.0347, 2.2634), _V(0, -0.1, -1), CRCS_THRUST, ph_CM, RCS_ISP);
		th_att_rot[1] = th_att_lin[3] = CreateThruster(_V(-0.275, 2.0347, 2.2634), _V(0, -0.1, -1), CRCS_THRUST, ph_CM, RCS_ISP);

		// Pitchdown	
		th_att_rot[2] = th_att_lin[0] = CreateThruster(_V(0.275, 1.5089, 3.267), _V(0, -1, -0.1), CRCS_THRUST, ph_CM, RCS_ISP);
		th_att_rot[3] = th_att_lin[1] = CreateThruster(_V(-0.275, 1.5089, 3.267), _V(0, -1, -0.1), CRCS_THRUST, ph_CM, RCS_ISP);

		// Yawleft
		th_att_rot[4] = th_att_lin[4] = CreateThruster(_V(-2.1991, 0.247, 2.0075), _V(-0.1, 0, -1), CRCS_THRUST, ph_CM, RCS_ISP);
		th_att_rot[5] = th_att_lin[5] = CreateThruster(_V(-2.1991, -0.247, 2.0075), _V(-0.1, 0, -1), CRCS_THRUST, ph_CM, RCS_ISP);

		// Yawright
		th_att_rot[6] = th_att_lin[6] = CreateThruster(_V(2.1991, 0.247, 2.0075), _V(0.1, 0, -1), CRCS_THRUST, ph_CM, RCS_ISP);
		th_att_rot[7] = th_att_lin[7] = CreateThruster(_V(2.1991, -0.247, 2.0075), _V(0.1, 0, -1), CRCS_THRUST, ph_CM, RCS_ISP);

		CreateThrusterGroup(th_att_rot, 2, THGROUP_ATT_PITCHUP);
		CreateThrusterGroup(th_att_rot + 2, 2, THGROUP_ATT_PITCHDOWN);
		CreateThrusterGroup(th_att_rot + 4, 2, THGROUP_ATT_YAWLEFT);
		CreateThrusterGroup(th_att_rot + 6, 2, THGROUP_ATT_YAWRIGHT);

		CreateThrusterGroup(th_att_lin + 4, 4, THGROUP_ATT_BACK);

		for (int i = 0; i<8; i++) {
			AddExhaust(th_att_rot[i], FR_SIZE, tex_rcs);
			AddExhaustStream(th_att_rot[i], &rcs_stream);
		}

		// Bankleft
		th_att_rot[0] = th_att_lin[2] = CreateThruster(_V(-1.5032, 1.6792, 1.9087), _V(-0.2, -0.8, 0), 3 * CRCS_THRUST, ph_CM, RCS_ISP);
		th_att_rot[1] = th_att_lin[3] = CreateThruster(_V(-1.328, 1.8339, 1.8892), _V(-0.2, -0.8, 0), 3 * CRCS_THRUST, ph_CM, RCS_ISP);

		// Bankright
		th_att_rot[2] = th_att_lin[4] = CreateThruster(_V(1.5032, 1.6792, 1.9087), _V(0.2, -0.8, 0), 3 * CRCS_THRUST, ph_CM, RCS_ISP);
		th_att_rot[3] = th_att_lin[5] = CreateThruster(_V(1.328, 1.8339, 1.8892), _V(0.2, -0.8, 0), 3 * CRCS_THRUST, ph_CM, RCS_ISP);

		CreateThrusterGroup(th_att_rot, 2, THGROUP_ATT_BANKLEFT);
		CreateThrusterGroup(th_att_rot + 2, 2, THGROUP_ATT_BANKRIGHT);

		CreateThrusterGroup(th_att_lin, 6, THGROUP_ATT_DOWN);

		//	int i;
		for (int i = 0; i<4; i++) {
			AddExhaust(th_att_rot[i], FR_SIZE, tex_rcs);
			AddExhaustStream(th_att_rot[i], &rcs_stream);
		}
	}
	void TALON::SetThrusters3_Land()
	{
		ph_CM = CreatePropellantResource(10);	 // ground braking propellant

		THRUSTER_HANDLE th_att_lin[16];

		th_att_lin[0] = CreateThruster(_V(2, 0, -5), _V(-1, 0, 1), 40000, ph_CM, RCS_ISP);
		th_att_lin[1] = CreateThruster(_V(-2, 0, -5), _V(1, 0, 1), 40000, ph_CM, RCS_ISP);
		th_att_lin[2] = CreateThruster(_V(0, 2, -5), _V(0, -1, 1), 40000, ph_CM, RCS_ISP);
		th_att_lin[3] = CreateThruster(_V(0, -2, -5), _V(0, 1, 1), 40000, ph_CM, RCS_ISP);

		CreateThrusterGroup(th_att_lin, 4, THGROUP_ATT_UP);
		SetThrusterGroupLevel(THGROUP_ATT_UP, 1);

		for (int i = 0; i<4; i++)
			AddExhaustStream(th_att_lin[i], &land_stream);
	}

	void TALON::SetConfig0_Ready(void)
	{
		SetSize(6);
		SetEmptyMass(CM_DRYMASS + CM_FUEL + SM_DRYMASS);
		SetPMI(_V(2.3, 2.3, 2));

		SetCW(0.5, 0.5, 1, 1);
		SetCrossSections(_V(35, 35, 24));
		SetRotDrag(_V(0.5, 0.5, 0.3));

		SetCOG_elev(2.5);

		SetPitchMomentScale(1e-4);
		SetBankMomentScale(1e-4);


		SetThrusters1_Orbit();
		mode = 0;
		ReloadMeshes();
	}
	void TALON::SetConfig1_Orbit(void)
	{
		SetSize(6);
		SetEmptyMass(CM_DRYMASS + CM_FUEL + SM_DRYMASS);
		SetPMI(_V(2.3, 2.3, 2));

		SetCW(0.5, 0.5, 1, 1);
		SetCrossSections(_V(35, 35, 24));
		SetRotDrag(_V(0.5, 0.5, 0.3));

		SetCOG_elev(2.5);

		SetPitchMomentScale(1e-4);
		SetBankMomentScale(1e-4);


		SetThrusters1_Orbit();
		mode = 1;
		ReloadMeshes();

	}
	void TALON::SetConfig2_Reentry(void)
	{
		SetSize(6);
		SetEmptyMass(CM_DRYMASS);
		SetPMI(_V(1, 1, 1));

		//	SetCW (0.5, 0.5, 1, 1);
		SetCrossSections(_V(24, 28, 19));
		SetWingAspect(1);
		SetWingEffectiveness(1);

		SetPitchMomentScale(-1e-4);

		SetBankMomentScale(-1e-4);
		SetRotDrag(_V(0.01, 0.01, 0.005));

		SetCameraOffset(_V(0, 0, -1.8));
		SetTouchdownPoints(_V(0, -2, 0), _V(-2, 1, 0), _V(2, 1, 0));
		SetSurfaceFrictionCoeff(0.2, 0.2);

//		SetDockParams(_V(0, 0, 2.1), _V(0, 0, 1), _V(0, 1, 0));

		CreateAirfoil(LIFT_VERTICAL, _V(0, 0.01, 0.1), VLiftCoeff, 5.5, 0, 1.27);
		CreateAirfoil(LIFT_HORIZONTAL, _V(0, 0, 0.01), HLiftCoeff, 5.5, 0, 1.27);

		mode = 2;
		ReloadMeshes();
		SetThrusters2_Reentry();
	}
	void TALON::SetConfig3_Drogue(void)
	{
		SetSize(20);
		SetEmptyMass(CM_DRYMASS);
		SetTrimScale(0);
		ClearAirfoilDefinitions();

		SetCW(1.5, 1.5, 1, 1);
		SetCrossSections(_V(29, 35, 150));
		SetRotDrag(_V(0.1, 0.1, 0.1));

		SetPitchMomentScale(-2e-4);
		SetBankMomentScale(-2e-4);

		SetPMI(_V(9, 9, 3));

		SetTouchdownPoints(_V(0, -5, -8), _V(-3, -2, -8), _V(3, -2, -8));
		SetSurfaceFrictionCoeff(0.3, 0.3);

		ClearThrusterDefinitions();
		mode = 3;
		ReloadMeshes();
	}

	void TALON::SetConfig4_Chute(void)
	{
		SetSize(30);
		SetEmptyMass(CM_DRYMASS);
		SetTrimScale(0);
		ClearAirfoilDefinitions();

		SetCW(2, 2, 1, 1);
		SetCrossSections(_V(26, 26, 2000));
		SetRotDrag(_V(0.5, 0.5, 0.3));

		SetPitchMomentScale(-2e-4);
		SetBankMomentScale(-2e-4);

		SetPMI(_V(9, 9, 3));

		//	SetTouchdownPoints(_V(0,-5,-6.9),_V(-3,-2,-6.9),_V(3,-2,-6.9));
		SetTouchdownPoints(_V(0, -5, -7), _V(-5, 3, -7), _V(5, 3, -7));
		SetSurfaceFrictionCoeff(0.3, 0.3);

		ClearThrusterDefinitions();
		mode = 4;
		ReloadMeshes();
	}

	void TALON::SetConfig5_Chute_deployed(void)
	{
		SetConfig4_Chute();
		mode = 5;
		ReloadMeshes();
	}

	void TALON::SetConfig6_Land(void)
	{
		SetSize(6);
		SetEmptyMass(CM_DRYMASS);
		SetPMI(_V(9, 9, 3));
		ClearAirfoilDefinitions();

		//	SetTouchdownPoints(_V(0,-5,-6.9),_V(-3,-2,-6.9),_V(3,-2,-6.9));
		SetTouchdownPoints(_V(0, -5, -4), _V(-5, 3, -4), _V(5, 3, -4));
		SetSurfaceFrictionCoeff(0.3, 0.3);

		SetThrusters3_Land();

		mode = 6;
		balloonMesh = oapiLoadMeshGlobal("Orion-MPCV\\orion-balloons");
		ReloadMeshes();
	}
	void TALON::SetConfig7_Ready(void)
	{
		SetSize(6);
		SetEmptyMass(CM_DRYMASS + CM_FUEL + SM_DRYMASS);
		SetPMI(_V(2.3, 2.3, 2));

		SetCW(0.5, 0.5, 1, 1);
		SetCrossSections(_V(35, 35, 24));
		SetRotDrag(_V(0.5, 0.5, 0.3));

		SetCOG_elev(2.5);

		SetPitchMomentScale(1e-4);
		SetBankMomentScale(1e-4);


		SetThrusters1_Orbit();
		mode = 0;


	//	SetThrusterLevel(th_att_lin, 1);
		//ReloadMeshes();
	}
	void TALON::SetConfig0_Surface(void)
	{
		SetSize(6);
		SetEmptyMass(CM_DRYMASS + CM_FUEL + SM_DRYMASS);
		SetPMI(_V(2.3, 2.3, 2));

		SetCW(0.5, 0.5, 1, 1);
		SetCrossSections(_V(35, 35, 24));
		SetRotDrag(_V(0.5, 0.5, 0.3));

		SetCOG_elev(2.5);

		SetPitchMomentScale(1e-4);
		SetBankMomentScale(1e-4);


		SetThrusters1_Surface();
		mode = 7;
		ReloadMeshes();
	}
	void TALON::SetConfig10_Launch0(void)
	{
		SetSize(12);
		SetEmptyMass(LAS_DRYMASS + LAS_FUEL + CM_DRYMASS + CM_FUEL + SM_DRYMASS + SM_FUEL + FAIR_MASS + ADAPT_MASS);
		SetPMI(_V(5, 5, 4));

		SetCW(0.5, 0.5, 1, 1);
		SetCrossSections(_V(35, 35, 24));
		SetRotDrag(_V(0.5, 0.5, 0.3));

		SetCOG_elev(2.5);

		SetPitchMomentScale(1e-4);
		SetBankMomentScale(1e-4);

		mode = 10;
		ReloadMeshes();
	}

	void TALON::SetConfig11_Launch1(void)
	{
		SetConfig10_Launch0();
		SetEmptyMass(CM_DRYMASS + CM_FUEL + SM_DRYMASS + SM_FUEL + ADAPT_MASS);	// reduce mass of blown fair + LAS
		SetSize(12);
		mode = 11;
		ReloadMeshes();
	}
	void TALON::blowSM() {
		int j = 0;
		SetMeshVisibilityMode(smMeshUINT, MESHVIS_NEVER);

		if (oapiGetTimeAcceleration() > 10)		// Slow down
			oapiSetTimeAcceleration(10);
		BEACON = 1;

		if (mode > 9) j++;
		SetConfig2_Reentry();

		VESSELSTATUS vs;
		char name[256] = { "" };
		THRUSTER_HANDLE th_att_lin[4];
		OBJHANDLE h;
		VESSEL *v;

		GetStatus(vs);
		vs.flag[0] = 3;
		//	VECTOR3 ofs = {0,0,-3};
		//VECTOR3 ofs = { 0, 0, -2.06 };
		Local2Rel(ofs, vs.rpos);

		VECTOR3 vel = _V(0, 0, -0.02);
		VECTOR3 rofs, rvel = { vs.rvel.x, vs.rvel.y, vs.rvel.z };
		Local2Rel(ofs, vs.rpos);
		GlobalRot(vel, rofs);
		vs.rvel.x = rvel.x + rofs.x;
		vs.rvel.y = rvel.y + rofs.y;
		vs.rvel.z = rvel.z + rofs.z;

		strcpy(name, GetName()); strcat(name, "-SM");
		h = oapiCreateVessel(name, "TALON-SM", vs);
		v = oapiGetVesselInterface(h);


		//		v->SetAnimation(anim_JOINT2, JOINT2_proc);
		//		v->SetAnimation(anim_JOINT1, JOINT1_proc);
		//		v->SetAnimation(anim_JOINT7, ARM3_proc);
		//		v->SetAnimation(anim_HGA, HGA_proc);



		v->SetEmptyMass(SM_DRYMASS);
		ph_RM = v->CreatePropellantResource(10.0, 10.0, 1.0);
		//  CreateThruster (_V( 0.06, 2.10, -1.49), _V(0,-0.7,-0.7), RCS_THRUST, ph_RM, RCS_ISP);

		th_att_lin[8] = v->CreateThruster(_V(6, 0, 10), _V(0, 0, -1), RCS_THRUST / 3, ph_RM, RCS_ISP);
		v->AddExhaust(th_att_lin[8], 1.5, 0.16, _V(1.854029, 1.835438, .984), _V(0, 0, 1), tex_rcs);
		th_att_lin[9] = v->CreateThruster(_V(-6, 0, 10), _V(0, 0, -1), RCS_THRUST / 3, ph_RM, RCS_ISP);
		v->AddExhaust(th_att_lin[9], 1.5, 0.16, _V(-1.84152, 1.846496, .984), _V(0, 0, 1), tex_rcs);

		th_att_lin[10] = v->CreateThruster(_V(6, 0, -10), _V(0, 0, -1), RCS_THRUST / 3, ph_RM, RCS_ISP);
		//AddExhaust(th_rcs[12], 1, .10, _V(1.68, -1.67, -.575), _V(.577, -.577, -.577));
		v->AddExhaust(th_att_lin[10], 1.5, 0.16, _V(1.841586, -1.8576, .984), _V(0, 0, 1), tex_rcs);

		th_att_lin[11] = v->CreateThruster(_V(-6, 0, -10), _V(0, 0, -1), RCS_THRUST / 3, ph_RM, RCS_ISP);
		v->AddExhaust(th_att_lin[11], 1.5, 0.16, _V(-1.851452, -1.845157, .984), _V(0, 0, 1), tex_rcs);

		v->SetThrusterLevel(th_att_lin[8], 1.0);
		v->SetThrusterLevel(th_att_lin[9], 1.0);
		v->SetThrusterLevel(th_att_lin[10], 1.0);
		v->SetThrusterLevel(th_att_lin[11], 1.0);




		v->SetSize(6);
		v->SetCrossSections(_V(24, 28, 19));
		v->SetCW(0.5, 0.5, 1, 1);
		if (j)  return;

		// Hack to display the solar panel around the separated RM
		GetStatus(vs);
		vs.flag[0] = 3;
		ofs = _V(0, 0, -1.87);

		Local2Rel(ofs, vs.rpos);

		vel = _V(0, 0, -0.02);
		rofs, rvel = _V(vs.rvel.x, vs.rvel.y, vs.rvel.z);
		Local2Rel(ofs, vs.rpos);
		GlobalRot(vel, rofs);
		vs.rvel.x = rvel.x + rofs.x;
		vs.rvel.y = rvel.y + rofs.y;
		vs.rvel.z = rvel.z + rofs.z;

		//strcpy(name, "S");
		//h = oapiCreateVessel(name, "Orion-MPCV\\Orion-SMS", vs); // config file of created solar panel
		//v = oapiGetVesselInterface(h);

		v->SetEmptyMass(SM_DRYMASS);
		ph_RM = v->CreatePropellantResource(7.06, 7.06, 1.0);
		th_att_lin[0] = v->CreateThruster(_V(0, 0, 0), _V(0, 0, -1), 0.703 * 8 * RCS_THRUST / 3, ph_RM, RCS_ISP);

		v->SetThrusterLevel(th_att_lin[0], 1.0);

		v->SetSize(6);
		v->SetCrossSections(_V(24, 28, 19));
		v->SetCW(0.5, 0.5, 1, 1);
		//v->mode = 99;

		v->SetEnableFocus(false);

	}
	void TALON::blowCover() {
		SetMeshVisibilityMode(coverMeshUINT, MESHVIS_NEVER);
		// Create the upper cover as an individual object
		VESSELSTATUS vs;
		GetStatus(vs);
		vs.flag[0] = 3;
		VECTOR3 ofs = { 0, 0, 0 };
		Local2Rel(ofs, vs.rpos);

		VECTOR3 vel = _V(0, 0.5, -1);
		VECTOR3 rofs, rvel = { vs.rvel.x, vs.rvel.y, vs.rvel.z };
		Local2Rel(ofs, vs.rpos);
		GlobalRot(vel, rofs);
		vs.rvel.x = rvel.x + rofs.x;
		vs.rvel.y = rvel.y + rofs.y;
		vs.rvel.z = rvel.z + rofs.z;

		char name[256] = { "" };
		strcpy(name, GetName()); strcpy(name, "-cover");
		oapiCreateVessel(name, "TALON_DOCKCOVER", vs);

		if (oapiGetTimeAcceleration() >10)
			oapiSetTimeAcceleration(10);
	}
	void TALON::blowDrogue() {		// Create the drogue as an individual object
		VESSELSTATUS vs;
		GetStatus(vs);
		vs.flag[0] = 3;
		VECTOR3 ofs = { 0, 0, -2 };
		Local2Rel(ofs, vs.rpos);

		VECTOR3 vel = _V(0, 0, -2);
		VECTOR3 rofs, rvel = { vs.rvel.x, vs.rvel.y, vs.rvel.z };
		Local2Rel(ofs, vs.rpos);
		GlobalRot(vel, rofs);
		vs.rvel.x = rvel.x + rofs.x;
		vs.rvel.y = rvel.y + rofs.y;
		vs.rvel.z = rvel.z + rofs.z;

		char name[256];
		strcpy(name, GetName()); strcpy(name, "-Drogue");
		oapiCreateVessel(name, "TALON_DROGUE", vs);
	}

	void TALON::blowChute() {		// Create the chute as an individual object
		oapiSetTimeAcceleration(1);

		VESSELSTATUS vs;
		GetStatus(vs);
		vs.flag[0] = 3;
		VECTOR3 ofs = { 0, 0, 2 };
		Local2Rel(ofs, vs.rpos);

		VECTOR3 vel = _V(0, -30, 3);
		VECTOR3 rofs, rvel = { vs.rvel.x, vs.rvel.y, vs.rvel.z };
		Local2Rel(ofs, vs.rpos);
		GlobalRot(vel, rofs);
		vs.rvel.x = rvel.x + rofs.x;
		vs.rvel.y = rvel.y + rofs.y;
		vs.rvel.z = rvel.z + rofs.z;

		char name[256];
		strcpy(name, GetName()); strcpy(name, "-Chute");
		oapiCreateVessel(name, "TALON_CHUTE", vs);
	}
	void TALON::blowLas() {
		VESSELSTATUS vs;
		char name[256];
		PROPELLANT_HANDLE ph_las_aux;
		THRUSTER_HANDLE th_las_aux;
		//	OBJHANDLE h;
		VESSEL *v;

		if (oapiGetTimeAcceleration() > 10)		// Slow down
			oapiSetTimeAcceleration(10);

		//	SetEmptyMass (CM_DRYMASS+CM_FUEL+SM_DRYMASS+SM_FUEL);
		//ReloadMeshes();

		GetStatus(vs);
		vs.flag[0] = 3;
		VECTOR3 ofs = { 0, 0, mode == 12 ? -8 : 0 };
		Local2Rel(ofs, vs.rpos);

		VECTOR3 vel = _V(0, 0, 0.02);
		VECTOR3 rofs, rvel = { vs.rvel.x, vs.rvel.y, vs.rvel.z };
		GlobalRot(vel, rofs);
		vs.rvel.x = rvel.x + rofs.x;
		vs.rvel.y = rvel.y + rofs.y;
		vs.rvel.z = rvel.z + rofs.z;

		strcpy(name, GetName()); strcat(name, "-LAS");
		fairLh = oapiCreateVessel(name, "Orion-MPCV\\Orion-MPCV-LAS", vs);
		v = oapiGetVesselInterface(fairLh);

		v->SetEmptyMass(3696);
		ph_las_aux = v->CreatePropellantResource(480, 480, 1.0);
		th_las_aux = v->CreateThruster(_V(0.0, 0.0, 9.5), _V(0.0, 0.001, 1), 563250.0, ph_las_aux, LAS_ISP);
		v->SetThrusterLevel(th_las_aux, 1.0);
		v->SetEnableFocus(false);

		v->AddExhaust(th_las_aux, 8.0, 0.6, _V(0.0, 0.4, 12.3), _V(0, 0.4, -0.5), tex_rcs);
		v->AddExhaust(th_las_aux, 8.0, 0.6, _V(0.4, 0.0, 12.3), _V(0.4, 0, -0.5), tex_rcs);
		v->AddExhaust(th_las_aux, 8.0, 0.6, _V(0.0, -0.4, 12.3), _V(-0, -0.4, -0.5), tex_rcs);
		v->AddExhaust(th_las_aux, 8.0, 0.6, _V(-0.4, 0.0, 12.3), _V(-0.4, 0, -0.5), tex_rcs);

		v->SetPitchMomentScale(1e-5);
		v->SetBankMomentScale(1e-5);

		SetMeshVisibilityMode(lasMeshMeshUINT, MESHVIS_NEVER);
	}