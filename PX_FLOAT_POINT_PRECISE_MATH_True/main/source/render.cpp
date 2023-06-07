
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

// �]���̕`��֐�
void display() {

    Snippets::startRender(sCamera);

    //const PxVec3 dynColor(1.0f, 0.5f, 0.25f); //���̂̐F
    const PxVec3 dynColor(1.0f, 1.0f, 1.0f);
    const PxVec3 rcaColor(0.6f * 0.75f, 0.8f * 0.75f, 1.0f * 0.75f);	//�ό`���̂̐F
    //const PxVec3 rcaColor(1.0f, 1.0f, 1.0f);	//��

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

    Snippets::showFPS(30, NULL);	//�`��FPS�̕\��(�E�B���h�E�g)

    glLoadIdentity();										//�����̕`��
    render_string(0.4, 0.5, -1, "PhysX��TimeStepPesSecond : " + to_string(get_physxTSPS()));


    Snippets::finishRender();

}

// �_�u���o�b�t�@���g�p�����`��֐� �����V�~�����[�V�����Ɣ񓯊�
void newDisplay() {

    Snippets::startRender(sCamera);

    const PxVec3 dynColor(1.0f, 1.0f, 1.0f);
    const PxVec3 rcaColor(0.6f * 0.75f, 0.8f * 0.75f, 1.0f * 0.75f);	//�ό`���̂̐F

    // ���̂̃����_�����O
    if (rigidBodyBuffer != nullptr){
        renderRigidBody(rigidBodyBuffer, dynColor);
    }

    // �ό`���̂̃����_�����O
    if (softBodyBuffer != nullptr) {
        renderSoftBody(softBodyBuffer, rcaColor);
    }

    // �`���FPS���E�B���h�E�g�ɕ\������
    Snippets::showFPS(30, NULL);
     
    // �����̕`��
    glLoadIdentity();										
    render_string(0.4, 0.5, -1, "PhysX��TimeStepPesSecond : " + to_string(get_physxTSPS()));
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


// �����V�~�����[�V�����̃R�[���o�b�N�֐�
void CALLBACK physicsTimer(UINT uID, UINT uMsg, DWORD_PTR dwUser, DWORD_PTR dw1, DWORD_PTR dw2) {
    stepPhysics(true);
}

void physicsLoop() {
    MMRESULT timerID = timeSetEvent(1000 / timeStepPerSecond, 0, physicsTimer, (DWORD)&_count, TIME_PERIODIC /*| TIME_CALLBACK_FUNCTION*/);
    if (!timerID) {
        printf("�^�C�}�[�o�^���s");
        exit(-1);
    }
}



// �����_�����O�̃R�[���o�b�N�֐�  ���݂�glut�̐��������𗘗p���Ă���̂Ŏg���Ă��Ȃ�
void CALLBACK renderingTimer(UINT uID, UINT uMsg, DWORD_PTR dwUser, DWORD_PTR dw1, DWORD_PTR dw2) {
    //glutPostRedisplay();
}

void renderingLoop() {
    MMRESULT timerID = timeSetEvent(1000 / 60, 0, renderingTimer, (DWORD)&_count, TIME_PERIODIC /*| TIME_CALLBACK_FUNCTION*/);
    if (!timerID) {
        printf("�^�C�}�[�o�^���s");
        exit(-1);
    }
}

void loopMain() {

    // �����V�~�����[�V�����̏�����
    initPhysics(true);

    // �`��Ɋւ��鏉����
    sCamera = new Snippets::Camera(PxVec3(0, 1.0f, 3.0f), PxVec3(-0, -1.0f, -3.0f));
    Snippets::setupDefault("aaaa", sCamera, NULL, newDisplay, exitCallback);    // glut�̐���������p�����`��
    //Snippets::setupDefault2("aaaa", sCamera, NULL, newDisplay, exitCallback);                 // glut�̐���������p���Ȃ��`�� �ǂ��ɂ���肭�����Ȃ�

    // �����V�~�����[�V�����̃R�[���o�b�N�֐��̓o�^
    physicsLoop();

    // �`��̃R�[���o�b�N�֐��̓o�^   ���݂͎g���Ă��Ȃ�
    //renderingLoop();

    // �`�惋�[�v�̊J�n
    glutMainLoop();
}

