
#include "render_snippet.h"

#include <vector>

#include "PxPhysicsAPI.h"

#include "SnippetRender.h"
#include "SnippetCamera.h"
#include "SnippetSoftBody.h"

using namespace physx;

#include "myFunc.h"
#include <Windows.h>
#include <iostream>
#pragma warning(disable : 4996)
using namespace myFunc;
using namespace std;

extern void initPhysics(bool interactive);
extern void stepPhysics(bool interactive);
extern void cleanupPhysics(bool interactive);
extern std::vector<SoftBody> gSoftBodies;

extern struct rigidBodyBuf* rigidBodyBuffer;
extern void renderRigidBody(rigidBodyBuf* buffer, PxVec3 dynColor);
extern bool rigidBufFlag;

extern struct softBodyBuf* softBodyBuffer;
extern void renderSoftBody(softBodyBuf* buffer, PxVec3 dynColor);
extern bool softBufFlag;

static int	_count = 0;

Snippets::Camera* sCamera;

// 従来の描画関数
void display() {

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
        std::vector<PxRigidActor*> actors(nbActors);
        scene->getActors(PxActorTypeFlag::eRIGID_DYNAMIC | PxActorTypeFlag::eRIGID_STATIC, reinterpret_cast<PxActor**>(&actors[0]), nbActors);
        Snippets::renderActors(&actors[0], static_cast<PxU32>(actors.size()), true, dynColor);
    }
    for (PxU32 i = 0; i < gSoftBodies.size(); i++)
    {
        SoftBody* sb = &gSoftBodies[i];
        Snippets::renderSoftBody(sb->mSoftBody, sb->mPositionsInvMass, true, rcaColor);
    }

    Snippets::showFPS(30, NULL);	//描画FPSの表示(ウィンドウ枠)

    glLoadIdentity();										//文字の描画
    render_string(0.4, 0.5, -1, "PhysXのTimeStepPesSecond : " + to_string(get_physxTSPS()));


    Snippets::finishRender();

}

// ダブルバッファを使用した描画関数 物理シミュレーションと非同期
void newDisplay() {

    Snippets::startRender(sCamera);

    const PxVec3 dynColor(1.0f, 1.0f, 1.0f);
    const PxVec3 rcaColor(0.6f * 0.75f, 0.8f * 0.75f, 1.0f * 0.75f);	//変形物体の色

    // 剛体のレンダリング
    if (rigidBodyBuffer != nullptr){
        renderRigidBody(rigidBodyBuffer, dynColor);
    }

    // 変形物体のレンダリング
    if (softBodyBuffer != nullptr) {
        renderSoftBody(softBodyBuffer, rcaColor);
    }

    // 描画のFPSをウィンドウ枠に表示する
    Snippets::showFPS(30, NULL);
     
    // 文字の描画
    glLoadIdentity();										
    render_string(0.4, 0.5, -1, "PhysXのTimeStepPesSecond : " + to_string(get_physxTSPS()));
    //render_string(0.4, 0.475, -1, "Write : " + to_string(w));
    //render_string(0.4, 0.45, -1, "Load : " + to_string(l));
    //render_string(0.4, 0.425, -1, "Write during Loading : " + to_string(wl));
    //render_string(0.4, 0.4, -1, "Load during Writing : " + to_string(lw));

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


// 物理シミュレーションのコールバック関数
void CALLBACK physicsTimer(UINT uID, UINT uMsg, DWORD_PTR dwUser, DWORD_PTR dw1, DWORD_PTR dw2) {
    stepPhysics(true);
}

void physicsLoop() {
    MMRESULT timerID = timeSetEvent(1000 / timeStepPerSecond, 0, physicsTimer, (DWORD)&_count, TIME_PERIODIC /*| TIME_CALLBACK_FUNCTION*/);
    if (!timerID) {
        printf("タイマー登録失敗");
        exit(-1);
    }
}



// レンダリングのコールバック関数  現在はglutの垂直同期を利用しているので使っていない
void CALLBACK renderingTimer(UINT uID, UINT uMsg, DWORD_PTR dwUser, DWORD_PTR dw1, DWORD_PTR dw2) {
    //glutPostRedisplay();
}

void renderingLoop() {
    MMRESULT timerID = timeSetEvent(1000 / 60, 0, renderingTimer, (DWORD)&_count, TIME_PERIODIC /*| TIME_CALLBACK_FUNCTION*/);
    if (!timerID) {
        printf("タイマー登録失敗");
        exit(-1);
    }
}

void loopMain() {

    // 物理シミュレーションの初期化
    initPhysics(true);

    // 描画に関する初期化
    sCamera = new Snippets::Camera(PxVec3(0, 1.0f, 3.0f), PxVec3(-0, -1.0f, -3.0f));
    Snippets::setupDefault("aaaa", sCamera, NULL, newDisplay, exitCallback);    // glutの垂直同期を用いた描画
    //Snippets::setupDefault2("aaaa", sCamera, NULL, newDisplay, exitCallback);                 // glutの垂直同期を用いない描画 どうにも上手くいかない

    // 物理シミュレーションのコールバック関数の登録
    physicsLoop();

    // 描画のコールバック関数の登録   現在は使っていない
    //renderingLoop();

    // 描画ループの開始
    glutMainLoop();
}

