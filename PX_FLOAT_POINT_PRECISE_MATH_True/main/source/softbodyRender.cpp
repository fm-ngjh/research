// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Copyright (c) 2008-2023 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#include "render_snippet.h"

#ifdef RENDER_SNIPPET

#include <vector>

#include "PxPhysicsAPI.h"

#include "SnippetRender.h"
#include "SnippetCamera.h"
#include "SnippetSoftBody.h"

using namespace physx;

#include "myFunc.h"
#include <Windows.h>
#include <iostream>
using namespace myFunc;
using namespace std;

extern void initPhysics(bool interactive);
extern void stepPhysics(bool interactive);
extern void cleanupPhysics(bool interactive);
extern std::vector<SoftBody> gSoftBodies;



#include	<windows.h>
#include	<stdio.h>
#include	<mmsystem.h>	//winmm.libが必要

static int	_count = 0;


namespace
{
	Snippets::Camera* sCamera;

	vector<PxRigidActor*> actors;
	void renderCallback()
	{
		stepPhysics(true);

		Snippets::startRender(sCamera);

		//const PxVec3 dynColor(1.0f, 0.5f, 0.25f); //剛体の色
		const PxVec3 dynColor(1.0f, 1.0f, 1.0f);
		const PxVec3 rcaColor(0.6f * 0.75f, 0.8f * 0.75f, 1.0f * 0.75f);	//変形物体の色
		//const PxVec3 rcaColor(1.0f, 1.0f, 1.0f);	//白

		PxScene* scene;
		PxGetPhysics().getScenes(&scene, 1);
		PxU32 nbActors = scene->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC | PxActorTypeFlag::eRIGID_STATIC);
		if (nbActors)
		{
			//std::vector<PxRigidActor*> actors(nbActors);
			//scene->getActors(PxActorTypeFlag::eRIGID_DYNAMIC | PxActorTypeFlag::eRIGID_STATIC, reinterpret_cast<PxActor**>(&actors[0]), nbActors);
			//Snippets::renderActors(&actors[0], static_cast<PxU32>(actors.size()), true, dynColor);

			std::vector<PxRigidActor*> tmp(nbActors);
			scene->getActors(PxActorTypeFlag::eRIGID_DYNAMIC | PxActorTypeFlag::eRIGID_STATIC, reinterpret_cast<PxActor**>(&tmp[0]), nbActors);
			actors = tmp;
			Snippets::renderActors(&actors[0], static_cast<PxU32>(actors.size()), true, dynColor);
		}
		for (PxU32 i = 0; i < gSoftBodies.size(); i++)
		{
			SoftBody* sb = &gSoftBodies[i];
			Snippets::renderSoftBody(sb->mSoftBody, sb->mPositionsInvMass, true, rcaColor);
		}

		Snippets::showFPS(30, NULL);	//描画FPSの表示(ウィンドウ枠)
		
		glLoadIdentity();										//文字の描画
		render_string(0.4, 0.5, -1, "PhysXのTimeStepPesSecond : " + get_physxTSPS());	
		

		Snippets::finishRender();
	}

	void cleanup()
	{
		delete sCamera;
		cleanupPhysics(true);
	}

	void exitCallback(void)
	{
#if PX_WINDOWS
		cleanup();
#endif
	}
}


void CALLBACK TimerProc(UINT uID, UINT uMsg, DWORD_PTR dwUser, DWORD_PTR dw1, DWORD_PTR dw2) {
	stepPhysics(true);
	/*if(get_renderOK()) glutPostRedisplay();*/
}

void simuLoop() {
	initPhysics(true);	
	MMRESULT timerID = timeSetEvent(1000/timeStepPerSecond, 0, TimerProc, (DWORD)&_count, TIME_PERIODIC /*| TIME_CALLBACK_FUNCTION*/); 
	if (!timerID) {
		printf("タイマー登録失敗");
		exit(-1);
	}
}

void renderLoop()
{
	initPhysics(true);

	//sCamera = new Snippets::Camera(PxVec3(10.0f, 10.0f, 10.0f), PxVec3(-0.6f, -0.2f, -0.7f));				//カメラの初期位置？
	sCamera = new Snippets::Camera(PxVec3(1.0f, 1.0f, 1.0f), PxVec3(-0.6f, -0.2f, -0.7f));

	Snippets::setupDefault("PhysX Snippet Softbody", sCamera, NULL, renderCallback, exitCallback);
	

	Snippets::initFPS();	//描画FPS表示の初期化

	glutMainLoop();

	

#if PX_LINUX_FAMILY
	cleanup();
#endif
}

#endif

