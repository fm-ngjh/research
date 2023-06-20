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
// Copyright (c) 2023 fm-ngjh. All rights reserved.

#include <ctype.h>
#include "PxPhysicsAPI.h"
#include "snippetcommon/SnippetPrint.h"
#include "snippetcommon/SnippetPVD.h"
#include "snippetutils/SnippetUtils.h"
#include "snippetsoftbody/SnippetSoftBody.h"
#include "snippetsoftbody/MeshGenerator.h"
#include "extensions/PxTetMakerExt.h"
#include "extensions/PxSoftBodyExt.h"

using namespace physx;
using namespace meshgenerator;

// 自分で追加した初期化系の諸々 ###################################################################### 
#include "myFunc.h"
#include "render_snippet.h"
#include <iostream>
#include <iomanip>
#include <time.h>
#include <fstream>
#include <sstream>
#include <direct.h>

using namespace std;
using namespace myFunc;


// 諸定数
const int solverIterCount = 32;
const float floorStaticFriction = 0.0f;
const float floorDynamicFriction = 0.0f;
const float floorCoefficient = 0.0f;
const float softBodyDynamicFriction = 1000000000.0f;
const float softBodyDensity = 5000.0f;
PxReal maxEdgeLength = 0.25;	// これによって発散するkrの値変わる 0.5ならkr=200000でも大丈夫

const double defRigidDynamicFriction = 100.0;
const double defRigidStaticFriction = 100.0;
const double defRigidCoefficient = 0.0;

const float targetRigidBodyDensity = 1000.0f;
const float kp = 200000.0f, bp = 1000.0f, kr = 100000.0f, br = 100.0f;	// VCの係数

const PxVec3 gravity = PxVec3(0.0, -9.81, 0.0);	// 通常
//const PxVec3 gravity = PxVec3(0.0, 0.00, 0.0);	// 無重力


// CSV出力に必要な変数
string output_csv_file_path_rigidbody;
string output_csv_file_path_softbody;
double csvTime = 0;
float dt;
float TSPS;

int nbDistanceFromPointToSurface = 25;	// 変形物体とある頂点と剛体のある面の距離 何個の頂点について調べるのか
vector<int> p_index = { 25, 20, 13, 8, 16, 33, 21, 3, 2, 7, 29, 24, 15, 1, 14, 54, 48, 38, 34, 35, 57, 50, 45, 42, 39 };	// 25個の場合の頂点のindex
vector<float> distanceFromPointToSurface(nbDistanceFromPointToSurface); // 変形物体とある頂点と剛体のある面の距離


// フラグの管理
const bool isCSV = true;			// CSVを出力する
const bool isVC = true;				// 並進VC力を加える
const bool isVCRot = true;			// 回転VC力を加える
const bool isMakeSoftBody = true;	// 変形物体を1つでも生成する
const bool isAttatchment = true;	// 変形物体と剛体を接続する
const bool isMoveFinger = true;		// 指を動かす
const bool isCalDistance = true;


// PhysXの初期化
static PxDefaultAllocator		gAllocator;
static PxDefaultErrorCallback	gErrorCallback;
static PxFoundation*			gFoundation = NULL;
static PxPhysics*				gPhysics = NULL;
static PxCudaContextManager*	gCudaContextManager = NULL;
static PxDefaultCpuDispatcher*	gDispatcher = NULL;
static PxScene*					gScene = NULL;
static PxMaterial*				gMaterial = NULL;
static PxPvd*					gPvd = NULL;
std::vector<SoftBody>			gSoftBodies;


// 剛体生成関数の前方宣言
static PxRigidDynamic* createRigidCube(const PxTransform& t, PxReal halfExtent, double density, PxMaterial* material, PxFilterData data = PxFilterData());
static PxRigidDynamic* createRigidCuboid(const PxTransform& t, PxReal halfWidth, PxReal halfHeight, PxReal halfDepth, double density, PxMaterial* material);
static PxRigidDynamic* createRigidCapsule(const PxTransform& t, PxReal rad, PxReal halfHeight, double density, PxMaterial* material);
static PxRigidDynamic* createRigidSphere(const PxTransform& t, PxReal rad, double density, PxMaterial* material);


// 剛体の構造体
struct CubeParam {
	double wdh;
	double density;
	double sf;
	double df;
	double cr;

	PxRigidDynamic* body;
	PxVec3 pos;
	PxVec3 prevPos;
	PxQuat quat;
	PxQuat prevQuat;
	PxVec3 vel;
	PxVec3 prevVel;
	PxVec3 angVel;
	PxVec3 prevAngVel;
	PxMaterial* rigidMaterial;
	PxFilterData filterData;

	CubeParam() {
		wdh = 0.1; density = 10.0;
		sf = defRigidStaticFriction; df = defRigidDynamicFriction; cr = defRigidCoefficient;
		pos = PxVec3(0.0, 0.0, 0.0); prevPos = PxVec3(0.0, 0.0, 0.0);
		quat = PxIdentity; prevQuat = PxIdentity;
		rigidMaterial = gPhysics->createMaterial(sf, df, cr);
	}

	void make() {
		body = createRigidCube(PxTransform(pos), wdh / 2.0, density, rigidMaterial, filterData);
	}

	void reParam() {
		pos = body->getGlobalPose().p;
		quat = body->getGlobalPose().q;
		vel = body->getLinearVelocity();
		angVel = body->getAngularVelocity();
	}

	void rePrevParam() {
		prevPos = pos;
		prevQuat = quat;
		prevVel = vel;
		prevAngVel = angVel;
	}
};

struct CuboidParam {
	double w;
	double h;
	double d;
	double density;
	double sf;
	double df;
	double cr;

	PxRigidDynamic* body;
	PxVec3 pos;
	PxVec3 prevPos;
	PxQuat quat;
	PxQuat prevQuat;
	PxVec3 vel;
	PxVec3 prevVel;
	PxVec3 angVel;
	PxVec3 prevAngVel;
	PxMaterial* rigidMaterial;// = gPhysics->createMaterial(0, 0, 0);

	CuboidParam() {
		w = 0.1; h = 0.1; d = 0.1; density = 10.0;
		sf = defRigidStaticFriction; df = defRigidDynamicFriction; cr = defRigidCoefficient;
		pos = PxVec3(0.0, 0.0, 0.0); prevPos = PxVec3(0.0, 0.0, 0.0);
		quat = PxQuat(PxIdentity); prevQuat = PxQuat(PxIdentity);
		rigidMaterial = gPhysics->createMaterial(sf, df, cr);
	}

	void make() {
		body = createRigidCuboid(PxTransform(pos), w / 2.0, h / 2.0, d / 2.0, density, rigidMaterial);
	}

	void reParam() {
		pos = body->getGlobalPose().p;
		quat = body->getGlobalPose().q;
		vel = body->getLinearVelocity();
		angVel = body->getAngularVelocity();
	}

	void rePrevParam() {
		prevPos = pos;
		prevQuat = quat;
		prevVel = vel;
		prevAngVel = angVel;
	}
};

struct CapsuleParam {
	double r;		//半径
	double h;		//高さ
	double density;	//密度
	double sf;		//静止摩擦係数
	double df;		//動摩擦係数
	double cr;		//反発係数

	PxRigidDynamic* body;
	PxVec3 pos;
	PxVec3 prevPos;
	PxQuat quat;
	PxQuat prevQuat;
	PxVec3 vel;
	PxVec3 prevVel;
	PxVec3 angVel;
	PxVec3 prevAngVel;
	PxMaterial* rigidMaterial;
	PxVec3 VC;
	PxVec3 VCR;

	CapsuleParam() {
		r = 1.0; h = 1.0; density = 100.0;
		sf = defRigidStaticFriction; df = defRigidDynamicFriction; cr = defRigidCoefficient;
		pos = PxVec3(0.0, 0.0, 0.0); prevPos = PxVec3(0.0, 0.0, 0.0);
		quat = PxQuat(PxIdentity); prevQuat = PxQuat(PxIdentity);
		rigidMaterial = gPhysics->createMaterial(sf, df, cr);
	}

	void make() {
		body = createRigidCapsule(PxTransform(pos), r, h / 2.0, density, rigidMaterial);
	}

	void reParam() {
		pos = body->getGlobalPose().p;
		quat = body->getGlobalPose().q;
		vel = body->getLinearVelocity();
		angVel = body->getAngularVelocity();
	}

	void rePrevParam() {
		prevPos = pos;
		prevQuat = quat;
		prevVel = vel;
		prevAngVel = angVel;
	}
};

struct SphereParam {
	double r;		//半径
	double density;	//密度
	double sf;		//静止摩擦係数
	double df;		//動摩擦係数
	double cr;		//反発係数

	PxRigidDynamic* body;
	PxVec3 pos;
	PxVec3 prevPos;
	PxQuat quat;
	PxQuat prevQuat;
	PxVec3 vel;
	PxVec3 prevVel;
	PxVec3 angVel;
	PxVec3 prevAngVel;
	PxMaterial* rigidMaterial;
	PxVec3 VC;
	PxVec3 VCR;

	SphereParam() {
		r = 1.0; density = 100.0;
		sf = defRigidStaticFriction; df = defRigidDynamicFriction; cr = defRigidCoefficient;
		pos = PxVec3(0.0, 0.0, 0.0); prevPos = PxVec3(0.0, 0.0, 0.0);
		quat = PxQuat(PxIdentity); prevQuat = PxQuat(PxIdentity);
		rigidMaterial = gPhysics->createMaterial(sf, df, cr);
	}

	void make() {
		body = createRigidSphere(PxTransform(pos), r, density, rigidMaterial);
	}

	void reParam() {
		pos = body->getGlobalPose().p;
		quat = body->getGlobalPose().q;
		vel = body->getLinearVelocity();
		angVel = body->getAngularVelocity();
	}

	void rePrevParam() {
		prevPos = pos;
		prevQuat = quat;
		prevVel = vel;
		prevAngVel = angVel;
	}
};


// 剛体オブジェクトのパラメータを格納する配列
vector<CubeParam> rigidCube;
vector<CuboidParam> rigidCuboid;
vector<CapsuleParam> rigidCapsule;
vector<SphereParam> rigidSphere;

// 変形物体のbodyを格納する配列　そのうち↑の形式に直す
vector<PxSoftBody*> softBody;

// VC用
vector<PxVec3> rPos(2), rPrevPos(2);
vector<PxQuat> rQuat(2), rPrevQuat(2);


// レンダリングのための変数保存---------------
#include "SnippetRender.h"

#define MAX_NUM_ACTOR_SHAPES	128

Snippets::TriggerRender* cb = 0;
bool changeColorForSleepingActors = true;
bool wireframePass = true;

// 剛体のバッファ
struct rigidBodyBuf {
	PxU32 numActors;
	vector<PxRigidActor*> actors;
	vector<PxU32> nbShapes;
	vector<bool> sleeping;
	vector<vector<PxMat44>> shapePose;
	vector<vector<geomParam>> geom;
	vector<vector<bool>> isTrigger;

	void renew(PxU32 _numActors, vector<PxRigidActor*> _actors, vector<PxU32> _nbShapes, vector<bool> _sleeping, vector<vector<PxMat44>> _shapePose,
		vector<vector<geomParam>> _geom, vector<vector<bool>> _isTrigger) {
		numActors = _numActors;
		actors = _actors;
		nbShapes = _nbShapes;
		sleeping = _sleeping;
		shapePose = _shapePose;
		geom = _geom;
		isTrigger = _isTrigger;
	}
};

// 剛体レンダリングのためのダブルバッファ
rigidBodyBuf rigidBodyBuf0;
rigidBodyBuf rigidBodyBuf1;
rigidBodyBuf* rigidBodyBuffer;

bool rigidBufFlag = 0;
extern void prepareVertexBuffer();
extern void pushVertex(const PxVec3& v0, const PxVec3& v1, const PxVec3& v2, const PxVec3& n);
extern const PxVec3* getVertexBuffer();
extern void releaseVertexBuffer();

// 変形物体のバッファ
struct softBodyBuf {
	vector<SoftBody> softBodies;
	PxU32 softBodiesSize;
	vector<SoftBody*> sb;
	vector<PxArray<PxVec4>> deformedPositionsInvMass;
	vector<PxShape*> sbShape;
	vector<PxMat44> sbShapePose;
	vector<geomParam> sbGeom;


	void renew(vector<SoftBody> _softBodies, PxU32 _softBodiesSize, vector<SoftBody*> _sb, vector<PxArray<PxVec4>> _deformedPositionsInvMass, vector<PxShape*> _sbShape, vector<PxMat44> _sbShapePose, vector<geomParam> _sbGeom) {
		softBodies = _softBodies;
		softBodiesSize = _softBodiesSize;
		sb = _sb;
		deformedPositionsInvMass = _deformedPositionsInvMass;
		sbShape = _sbShape;
		sbShapePose = _sbShapePose;
		sbGeom = _sbGeom;
	}
};

// 変形物体レンダリングのためのダブルバッファ
softBodyBuf softBodyBuf0;
softBodyBuf softBodyBuf1;
softBodyBuf* softBodyBuffer;

bool softBufFlag = 0;

void writeSoftBodyBuffer(softBodyBuf& buf); // 変形物体の頂点をcsvにとるための都合で前方宣言してる

// -----------------------------------------

// ######################################################################



void addSoftBody(PxSoftBody* softBody, const PxFEMParameters& femParams, const PxFEMMaterial& /*femMaterial*/,
	const PxTransform& transform, const PxReal density, const PxReal scale, const PxU32 iterCount/*, PxMaterial* tetMeshMaterial*/)
{
	PxShape* shape = softBody->getShape();
	PxTetrahedronMeshGeometry tetMeshGeom;
	shape->getTetrahedronMeshGeometry(tetMeshGeom);
	PxTetrahedronMesh* colTetMesh = tetMeshGeom.tetrahedronMesh;
	const PxU32 numVerts = colTetMesh->getNbVertices();

	PxBuffer* positionInvMassBuf = gPhysics->createBuffer(numVerts * sizeof(PxVec4), PxBufferType::eHOST, gCudaContextManager);

	const PxReal maxInvMassRatio = 50.f;

	softBody->setParameter(femParams);
	//softBody->setMaterial(femMaterial);
	softBody->setSolverIterationCounts(iterCount);

	PxSoftBodyExt::transform(*softBody, transform, scale);
	PxSoftBodyExt::updateMass(*softBody, density, maxInvMassRatio);
	PxSoftBodyExt::commit(*softBody, PxSoftBodyData::eALL);

	SoftBody sBody(softBody, positionInvMassBuf);

	gSoftBodies.push_back(sBody);
}

static PxSoftBody* createSoftBody(const PxCookingParams& params, const PxArray<PxVec3>& triVerts, const PxArray<PxU32>& triIndices, bool useCollisionMeshForSimulation = false)
{
	PxFEMSoftBodyMaterial* material = PxGetPhysics().createFEMSoftBodyMaterial(1e+9f, 0.45f, softBodyDynamicFriction);				//ヤング率，ポアソン比，動摩擦係数
	material->setDamping(0.005f);																									//ダンパ

	PxSoftBodyMesh* softBodyMesh;																									//-----------四面体の生成？ただし最初の2分割？----------

	PxU32 numVoxelsAlongLongestAABBAxis = 4 /*def 8*/;																				//useCollisionMeshForSimulationがfalseの時の分割数？
	PxSimpleTriangleMesh surfaceMesh;
	surfaceMesh.points.count = triVerts.size();
	surfaceMesh.points.data = triVerts.begin();
	surfaceMesh.triangles.count = triIndices.size() / 3;
	surfaceMesh.triangles.data = triIndices.begin();

	if (useCollisionMeshForSimulation)
	{
		softBodyMesh = PxSoftBodyExt::createSoftBodyMeshNoVoxels(params, surfaceMesh, gPhysics->getPhysicsInsertionCallback());		//FEMも干渉検出も同じ四面体メッシュ？
	}
	else
	{
		softBodyMesh = PxSoftBodyExt::createSoftBodyMesh(params, surfaceMesh, numVoxelsAlongLongestAABBAxis, gPhysics->getPhysicsInsertionCallback());	//FEMはボクセルメッシュを使う？
	}
	//---------------------------------------------------
	//Alternatively one can cook a softbody mesh in a single step
	//tetMesh = cooking.createSoftBodyMesh(simulationMeshDesc, collisionMeshDesc, softbodyDesc, physics.getPhysicsInsertionCallback());
	PX_ASSERT(softBodyMesh);

	if (!gCudaContextManager)
		return NULL;
	PxSoftBody* softBody = gPhysics->createSoftBody(*gCudaContextManager);
	if (softBody)
	{
		PxShapeFlags shapeFlags = PxShapeFlag::eVISUALIZATION | PxShapeFlag::eSCENE_QUERY_SHAPE | PxShapeFlag::eSIMULATION_SHAPE;

		PxFEMSoftBodyMaterial* materialPtr = PxGetPhysics().createFEMSoftBodyMaterial(1e+9f, 0.45f, softBodyDynamicFriction);		//ヤング率，ポアソン比，動摩擦係数2 こっちが本体？
		PxTetrahedronMeshGeometry geometry(softBodyMesh->getCollisionMesh());
		PxShape* shape = gPhysics->createShape(geometry, &materialPtr, 1, true, shapeFlags);

		shape->setSimulationFilterData(PxFilterData(0, 0, 1, 0));	// 干渉しない物体を設定するためのフィルタデータ とりあえず変形物体はこの値にした　剛体において３つ目の値を２以上にすると干渉しなくなる
		shape->setRestOffset(0.00);
		shape->setContactOffset(0.00);

		if (shape)
		{
			softBody->attachShape(*shape);
		}
		softBody->attachSimulationMesh(*softBodyMesh->getSimulationMesh(), *softBodyMesh->getSoftBodyAuxData());

		gScene->addActor(*softBody);

		PxFEMParameters femParams;
		//femParams.sleepDamping = 1000;
		addSoftBody(softBody, femParams, *material, PxTransform(PxVec3(0.f, 0.f, 0.f), PxQuat(PxIdentity)), softBodyDensity, 1.0f, solverIterCount);	//密度，スケール，反復計算回数
		softBody->setSoftBodyFlag(PxSoftBodyFlag::eDISABLE_SELF_COLLISION, true);
	}
	return softBody;
}

static void createSoftbodies(const PxCookingParams& params)
{
	PxArray<PxVec3> triVerts;
	PxArray<PxU32> triIndices;

	//PxReal maxEdgeLength = 0.5 /*def 1*/;													// 表面の三角形の辺の最大値(斜辺)がこの値未満になるように分割していく

	createCube(triVerts, triIndices, PxVec3(-1.875, 0.3, 0/*0, 1, 0*/), 0.5f /*def 2.5*/);					//箱を作ってる
	PxRemeshingExt::limitMaxEdgeLength(triIndices, triVerts, maxEdgeLength);				//箱の四面体を細かくしていく．干渉検出の方？
	softBody.push_back(createSoftBody(params, triVerts, triIndices, true));

	createCube(triVerts, triIndices, PxVec3(1.875, 0.3, 0), 0.5f /*def 2.5*/);					
	PxRemeshingExt::limitMaxEdgeLength(triIndices, triVerts, maxEdgeLength);				
	softBody.push_back(createSoftBody(params, triVerts, triIndices, true));

	//createSphere(triVerts, triIndices, PxVec3(-1.8, 0.3, 0), 0.25f, maxEdgeLength);
	//softBody.push_back(createSoftBody(params, triVerts, triIndices, true));
	////PxSoftBodyExt::transform(*softBody[0], PxTransform(PxVec3(-1.8, 0.3, 0)), 0.5f);
	////PxSoftBodyExt::commit(*softBody[0], PxSoftBodyData::eALL);

	//createSphere(triVerts, triIndices, PxVec3(1.8, 0.3, 0), 0.25f, maxEdgeLength);
	//softBody.push_back(createSoftBody(params, triVerts, triIndices, true));
	////PxSoftBodyExt::transform(*softBody[1], PxTransform(PxVec3(1.8, 0.3, 0)), 0.5f);
	////PxSoftBodyExt::commit(*softBody[1], PxSoftBodyData::eALL);
}

static PxRigidDynamic* createRigidCube(const PxTransform& t, PxReal halfExtent, double density, PxMaterial* material, PxFilterData data)
{
	PxShape* shape = gPhysics->createShape(PxBoxGeometry(halfExtent, halfExtent, halfExtent), *material);
	shape->setSimulationFilterData(data);

	PxTransform localTm(PxVec3(0.0f, 0.0f, 0.0f) * halfExtent);
	PxRigidDynamic* body = gPhysics->createRigidDynamic(t.transform(localTm));
	body->attachShape(*shape);
	PxRigidBodyExt::updateMassAndInertia(*body, density);
	gScene->addActor(*body);

	shape->release();
	return body;
}

static PxRigidDynamic* createRigidCuboid(const PxTransform& t, PxReal halfWidth, PxReal halfHeight, PxReal halfDepth, double density, PxMaterial* material)
{
	PxShape* shape = gPhysics->createShape(PxBoxGeometry(halfWidth, halfHeight, halfDepth), *material);

	PxTransform localTm(PxVec3(0.0f, 0.0f, 0.0f));
	PxRigidDynamic* body = gPhysics->createRigidDynamic(t.transform(localTm));
	body->attachShape(*shape);
	PxRigidBodyExt::updateMassAndInertia(*body, density);
	gScene->addActor(*body);

	shape->release();
	return body;
}

static PxRigidDynamic* createRigidCapsule(const PxTransform& t, PxReal rad, PxReal halfHeight, double density, PxMaterial* material)
{
	PxShape* shape = gPhysics->createShape(PxCapsuleGeometry(rad, halfHeight), *material);

	PxTransform localTm(PxVec3(0.0f, 0.0f, 0.0f) * halfHeight);
	PxRigidDynamic* body = gPhysics->createRigidDynamic(t.transform(localTm));
	body->attachShape(*shape);
	PxRigidBodyExt::updateMassAndInertia(*body, density);
	gScene->addActor(*body);

	shape->release();
	return body;
}

static PxRigidDynamic* createRigidSphere(const PxTransform& t, PxReal rad, double density, PxMaterial* material)
{
	PxShape* shape = gPhysics->createShape(PxSphereGeometry(rad), *material);

	PxTransform localTm(PxVec3(0.0f, 0.0f, 0.0f));
	PxRigidDynamic* body = gPhysics->createRigidDynamic(t.transform(localTm));
	body->attachShape(*shape);
	PxRigidBodyExt::updateMassAndInertia(*body, density);
	gScene->addActor(*body);

	shape->release();
	return body;
}


// SnippetSoftBodyAttachment.cppから
static void connectCubeToSoftBody(PxRigidDynamic* cube, PxReal cubeHalfExtent, const PxVec3& cubePosition, PxSoftBody* softBody, PxU32 pointGridResolution = 10)
{
	float f = 2.0f * cubeHalfExtent / (pointGridResolution - 1);
	for (PxU32 ix = 0; ix < pointGridResolution; ++ix)
	{
		PxReal x = ix * f - cubeHalfExtent;
		for (PxU32 iy = 0; iy < pointGridResolution; ++iy)
		{
			PxReal y = iy * f - cubeHalfExtent;
			for (PxU32 iz = 0; iz < pointGridResolution; ++iz)
			{
				PxReal z = iz * f - cubeHalfExtent;
				PxVec3 p(x, y, z);
				PxVec4 bary;
				PxI32 tet = PxTetrahedronMeshExt::findTetrahedronContainingPoint(softBody->getCollisionMesh(), p + cubePosition, bary);
				if (tet >= 0)
				{
					PxU32 handle = softBody->addTetRigidAttachment(cube, tet, bary, p);
					/*softBody->removeRigidAttachment(cube, handle);		アタッチメントを削除する方法*/
				}
			}
		}
	}
}

static PxFilterFlags softBodyRigidBodyFilter(PxFilterObjectAttributes attributes0, PxFilterData filterData0,
	PxFilterObjectAttributes attributes1, PxFilterData filterData1,
	PxPairFlags& pairFlags, const void* constantBlock, PxU32 constantBlockSize)
{
	PX_UNUSED(attributes0);
	PX_UNUSED(attributes1);
	PX_UNUSED(constantBlock);
	PX_UNUSED(constantBlockSize);
	if (filterData0.word2 != 0 && filterData0.word2 != filterData1.word2)
		return PxFilterFlag::eKILL;
	pairFlags |= PxPairFlag::eCONTACT_DEFAULT;
	return PxFilterFlag::eDEFAULT;
}

// CSV関連の関数
void initCSV() {
	// 剛体用のCSVのイニシャライズ

	//---------ファイル名の決定 "年-月-日-時-分-秒.csv"----------
	time_t t = time(NULL);
	struct tm local;
	localtime_s(&local, &t);
	char buf[128];
	strftime(buf, sizeof(buf), "%Y-%m-%d-%H-%M-%S", &local);
	string time = buf;
	string dirPath = "./csv_logs/" + time;
	_mkdir(dirPath.c_str());
	
	

	string filePath = "/rigidBody.csv";
	output_csv_file_path_rigidbody = dirPath + filePath;

	//---------ヘッダの初期化----------
	vector<string> constName, varName;

	//定数のヘッダ
	constName.push_back("並進バネ"); constName.push_back("並進ダンパ"); constName.push_back("回転バネ"); constName.push_back("回転ダンパ");
	constName.push_back("floorDF"); constName.push_back("floorSF"); constName.push_back("floorCR");

	//変数のヘッダ オブジェクトのパラメータ等
	varName.push_back("時間"); varName.push_back("timeStep"); varName.push_back("TSPS");
	for (int i = 0; i < rigidCube.size(); i++) {
		string header = "cube" + to_string(i);
		varName.push_back(header + "PosX"); varName.push_back(header + "PosY"); varName.push_back(header + "PosZ"); varName.push_back(header + "RotX"); varName.push_back(header + "RotY"); varName.push_back(header + "RotZ");
		varName.push_back(header + "VelX"); varName.push_back(header + "VelY"); varName.push_back(header + "VelZ"); varName.push_back(header + "AngVelX"); varName.push_back(header + "AngVelY"); varName.push_back(header + "AngVelZ");
		varName.push_back(header + "Density"); varName.push_back(header + "Mass"); varName.push_back(header + "HWD");
		varName.push_back(header + "SF"); varName.push_back(header + "DF"); varName.push_back(header + "CR");
	}

	for (int i = 0; i < rigidCapsule.size(); i++) {
		string header = "cap" + to_string(i);
		varName.push_back(header + "PosX"); varName.push_back(header + "PosY"); varName.push_back(header + "PosZ"); varName.push_back(header + "Rotx"); varName.push_back(header + "RotY"); varName.push_back(header + "RotZ");
		varName.push_back(header + "VelX"); varName.push_back(header + "VelY"); varName.push_back(header + "VelZ"); varName.push_back(header + "AngVelX"); varName.push_back(header + "AngVelY"); varName.push_back(header + "AngVelZ");
		varName.push_back(header + "VCFPX"); varName.push_back(header + "VCFPY"); varName.push_back(header + "VCFPY"); varName.push_back(header + "VCFRX"); varName.push_back(header + "VCFRX"); varName.push_back(header + "VCFRX");
		varName.push_back(header + "Density"); varName.push_back(header + "Mass"); varName.push_back(header + "R"); varName.push_back(header + "H");
		varName.push_back(header + "SF"); varName.push_back(header + "DF"); varName.push_back(header + "CR");
	}

	//---------ヘッダの書き込み---------
	ofstream ofs(output_csv_file_path_rigidbody);

	for (auto x : varName) {
		ofs << x << ",";
	}
	ofs << ",";
	for (auto x : constName) {
		ofs << x << ",";
	}
	ofs << endl;



	// 変形物体用のCSVのイニシャライズ
	softBodyBuf buffer = *softBodyBuffer;

	if (isMakeSoftBody)
	{
		//---------ファイル名の決定 "年-月-日-時-分-秒.csv"----------
		filePath = "/softBody.csv";
		output_csv_file_path_softbody = dirPath + filePath;
		ofstream ofs_(output_csv_file_path_softbody);

		//---------ヘッダの書き込み----------


		ofs_ << "時間" << "," << "timeStep" << "," << "TSPS" << ",";

		for (int i = 0; i < buffer.sbGeom[0].sbVertexPos.size(); i++)
		{
			ofs_ << "x" + to_string(i) << "," << "y" + to_string(i) << "," << "z" + to_string(i) << ",";
		}

		for(int i=0; i<nbDistanceFromPointToSurface; i++)
			ofs_ << "distance_" + to_string(p_index[i]) << ",";	
		ofs_ << endl;
	}

}

void writeCSV() {
	vector<float> varCSV;
	varCSV.push_back(csvTime); varCSV.push_back(dt); varCSV.push_back(TSPS);

	// ---------剛体---------
	for (auto& cube : rigidCube) {//varCSV.push_back();
		PxVec3 rot = q2ea(cube.quat);
		varCSV.push_back(cube.pos.x); varCSV.push_back(cube.pos.y); varCSV.push_back(cube.pos.z); varCSV.push_back(rot.x); varCSV.push_back(rot.y); varCSV.push_back(rot.z);
		varCSV.push_back(cube.vel.x); varCSV.push_back(cube.vel.y); varCSV.push_back(cube.vel.z); varCSV.push_back(cube.angVel.x); varCSV.push_back(cube.angVel.y); varCSV.push_back(cube.angVel.z);
		varCSV.push_back(cube.density); varCSV.push_back(pow(cube.wdh, 3) * cube.density); varCSV.push_back(cube.wdh);
		varCSV.push_back(cube.sf); varCSV.push_back(cube.df); varCSV.push_back(cube.cr);
	}

	for (auto& capsule : rigidCapsule) {
		PxVec3 rot = q2ea(capsule.quat);

		varCSV.push_back(capsule.pos.x); varCSV.push_back(capsule.pos.y); varCSV.push_back(capsule.pos.z); varCSV.push_back(rot.x); varCSV.push_back(rot.y); varCSV.push_back(rot.z);
		varCSV.push_back(capsule.vel.x); varCSV.push_back(capsule.vel.y); varCSV.push_back(capsule.vel.z); varCSV.push_back(capsule.angVel.x); varCSV.push_back(capsule.angVel.y); varCSV.push_back(capsule.angVel.z);
		varCSV.push_back(capsule.VC.x); varCSV.push_back(capsule.VC.y); varCSV.push_back(capsule.VC.z); varCSV.push_back(capsule.VCR.x); varCSV.push_back(capsule.VCR.y); varCSV.push_back(capsule.VCR.z);
		varCSV.push_back(capsule.density); varCSV.push_back((4 * PxPi * pow(capsule.r, 3) / 3 + pow(capsule.r, 2) * PxPi * capsule.h) * capsule.density); varCSV.push_back(capsule.r); varCSV.push_back(capsule.h);
		varCSV.push_back(capsule.sf); varCSV.push_back(capsule.df); varCSV.push_back(capsule.cr);
	}

	ofstream ofs(output_csv_file_path_rigidbody, ios::app);
	for (auto x : varCSV) {
		ofs << x << ",";
	}

	//定数の格納
	vector<float> constCSV;
	constCSV.push_back(kp); constCSV.push_back(bp); constCSV.push_back(kr); constCSV.push_back(br);
	constCSV.push_back(floorStaticFriction); constCSV.push_back(floorDynamicFriction);	constCSV.push_back(floorCoefficient);
	ofs << ",";
	for (auto x : constCSV) {
		ofs << x << ",";
	}
	ofs << endl;


	// ---------変形物体---------
	softBodyBuf buffer = *softBodyBuffer;
	ofstream ofs_(output_csv_file_path_softbody, ios::app);
	ofs_ << varCSV[0] << "," << varCSV[1] << "," << varCSV[2] << ",";

	if (!buffer.sbGeom.empty())
	{
		for (int i = 0; i < buffer.sbGeom[0].sbVertexPos.size(); i++)
		{
			ofs_ << buffer.sbGeom[0].sbVertexPos[i][0] << "," << buffer.sbGeom[0].sbVertexPos[i][1] << "," << buffer.sbGeom[0].sbVertexPos[i][2] << ",";
		}
		
		for (int i = 0; i < nbDistanceFromPointToSurface; i++)
			ofs_ << distanceFromPointToSurface[i] << ",";

		ofs_ << endl;
	}
}

// シミュレーションの初期化時に行いたい処理
void my_initPhysics()
{

	// 剛体箱の生成
	CubeParam tmpcube;
	tmpcube.pos = PxVec3(-2.0, 0.3, 0);
	tmpcube.rigidMaterial->setFrictionCombineMode(PxCombineMode::eMAX);
	tmpcube.wdh = 0.5; tmpcube.density = 500;
	tmpcube.rigidMaterial = gPhysics->createMaterial(0, 0, 0);
	tmpcube.filterData = PxFilterData(0, 0, 2, 0);

	// 左側の指を生成
	rigidCube.push_back(tmpcube);
	rigidCube[0].make();	
	rigidCube[0].body->setActorFlag(PxActorFlag::eDISABLE_GRAVITY, true);	// 無重力
	rigidCube[0].body->setAngularDamping(0.00);
	//rigidCube[0].body->setRigidDynamicLockFlag(PxRigidDynamicLockFlag::eLOCK_ANGULAR_Z, true);

	// 右側の指を生成
	tmpcube.pos = PxVec3(2.0, 0.3, 0);	
	tmpcube.filterData = PxFilterData(0, 0, 2, 0);
	rigidCube.push_back(tmpcube);
	rigidCube[1].make();
	rigidCube[1].body->setActorFlag(PxActorFlag::eDISABLE_GRAVITY, true);	// 無重力
	rigidCube[1].body->setAngularDamping(0.00);
	//rigidCube[1].body->setRigidDynamicLockFlag(PxRigidDynamicLockFlag::eLOCK_ANGULAR_Z, true);

	// 把持する箱を生成
	tmpcube.pos = PxVec3(0, 0.5, 0);
	tmpcube.wdh = 1;
	tmpcube.density = targetRigidBodyDensity;
	tmpcube.rigidMaterial = gPhysics->createMaterial(softBodyDynamicFriction, softBodyDynamicFriction, 0);
	tmpcube.filterData = PxFilterData(0, 0, 1, 0);
	rigidCube.push_back(tmpcube);
	rigidCube[2].make();
	rigidCube[2].body->setAngularDamping(0.00);


	for (int i = 0; i < 2; i++) 
	{
		rPos[i] = rigidCube[i].pos;
		rPrevPos[i] = rigidCube[i].prevPos;
		rQuat[i] = rigidCube[i].quat;
		rPrevQuat[i] = rigidCube[i].prevQuat;
	}

	// 球の生成テンプレート
	//SphereParam tmpsphere;
	//tmpsphere.pos = PxVec3(0, 0, 0.5);
	//tmpsphere.density = 1000.0;
	//tmpsphere.r = 0.1;
	//rigidSphere.push_back(tmpsphere);
	//rigidSphere[0].make();
	//rigidSphere[0].body->setAngularDamping(0.00);
	//rigidSphere[0].body->setAngularVelocity(PxVec3(0, 3.14, 0));

	// カプセルの生成テンプレート
	//CapsuleParam tmpcapsule;
	//tmpcapsule.rigidMaterial->setFrictionCombineMode(PxCombineMode::eMIN);
	//tmpcapsule.density = 1000.0;
	//tmpcapsule.h = 0.1; tmpcapsule.r = 0.05;
	//tmpcapsule.pos = PxVec3(0.0, 0.0, 1.0);
	//rigidCapsule.push_back(tmpcapsule);
	//rigidCapsule[0].make();
	//rigidCapsule[0].body->setAngularDamping(0.00);
	//rigidCapsule[0].body->setAngularVelocity(PxVec3(0, 3.14, 0));

	// 直方体の生成テンプレート
	//CuboidParam tmpcuboid;
	//tmpcuboid.rigidMaterial->setFrictionCombineMode(PxCombineMode::eMIN);
	//tmpcuboid.pos = PxVec3(0, 0, 1.5);
	//tmpcuboid.w = 0.2;
	//tmpcuboid.h = 0.3;
	//tmpcuboid.d = 0.1;
	//rigidCuboid.push_back(tmpcuboid);
	//rigidCuboid[0].make();
	//rigidCuboid[0].body->setAngularDamping(0.00);
	//rigidCuboid[0].body->setAngularVelocity(PxVec3(0, 3.14, 0));


	// 変形物体の生成
	if (isMakeSoftBody)
	{
		PxTolerancesScale scale;
		PxCookingParams params(scale);
		params.meshWeldTolerance = 0.001f;
		params.meshPreprocessParams = PxMeshPreprocessingFlags(PxMeshPreprocessingFlag::eWELD_VERTICES);
		params.buildTriangleAdjacencies = false;
		params.buildGPUData = true;
		createSoftbodies(params);
	}

	if (isAttatchment and isMakeSoftBody) 
	{
		// 剛体箱と変形物体の接続
		connectCubeToSoftBody(rigidCube[0].body, rigidCube[0].wdh / 2, rigidCube[0].pos, softBody[0], 10);
		connectCubeToSoftBody(rigidCube[1].body, rigidCube[1].wdh / 2, rigidCube[1].pos, softBody[1], 10);
	}

	initLastTime();

	 //変形物体レンダリングのためのバッファへの書き込み
	 //原因不明だがシミュレーションを２回してからでないと変形物体のデータが正しく取れない
	 //elapsedTimeを極小にしておけばシミュレーションへの影響はないと思う
	for (int _ = 0; _ < 2; _++)
	{
		gScene->simulate(0.000001);
		gScene->fetchResults(true);
		for (PxU32 i = 0; i < gSoftBodies.size(); i++)
		{
			SoftBody* sb = &gSoftBodies[i];
			sb->copyDeformedVerticesFromGPU();
		}
	}

	if (softBufFlag == 0) {
		writeSoftBodyBuffer(softBodyBuf0);
		softBodyBuffer = &softBodyBuf0;
		softBufFlag = 1;
	}
	else {
		writeSoftBodyBuffer(softBodyBuf1);
		softBodyBuffer = &softBodyBuf1;
		softBufFlag = 0;
	}


	if (isCSV)
		initCSV();
}

int step = 0;

PxVec3 cubePos[2] = { PxVec3(-1.1, 0.1, 0), PxVec3(1.1, 0.1, 0) };
PxQuat cuveQuat[2] = { PxQuat(PxIdentity), PxQuat(PxIdentity) };
// シミュレーションの１フレームごとに行いたい処理
void my_stepPhysics()
{
	// 計測値dtの取得
	dt = get_dt();
	csvTime += dt;

	// オブジェクトの座標・姿勢を取得
	for (auto& cube : rigidCube) {
		cube.reParam();
	}
	for (auto& capsule : rigidCapsule) {
		capsule.reParam();
	}

	// TSPSの表示
	TSPS = get_physxTSPS();
	cout << TSPS << endl;

	// 変形物体のある頂点と剛体箱のある面との距離を計算する
	if (isCalDistance)
	{
		// 剛体箱の対象面の頂点の座標と計算
		vector<PxVec3> cubeVeretexPos;
		cubeVeretexPos = cal_cube_vertex_pos(rigidCube[2].wdh, rigidCube[2].pos, rigidCube[2].quat);

		vector<PxVec3> p_pos;
		for(int i=0; i<nbDistanceFromPointToSurface; i++)
			p_pos.push_back(softBodyBuffer->sbGeom[0].sbVertexPos[p_index[i]]);

		for (int i=0; i<nbDistanceFromPointToSurface; i++)
		{
			distanceFromPointToSurface[i] = cal_distance_point_to_surface(p_pos[i], cubeVeretexPos[4], cubeVeretexPos[5], cubeVeretexPos[6]);
		}

	}

	// 箱指を動かす
	if (isMoveFinger)
	{
		if (step > 100)
		{
			if (rPos[0].x < /*-0.85*/ -0.725)
			{
				rPos[0].x += 0.01;
				rPos[1].x -= 0.01;
			}
			else if (rPos[0].y < 1.0)
			{
				rPos[0].y += 0.01;
				rPos[1].y += 0.01;
			}
		}
	}

	// VC力を加える
	for (int i = 0; i < 2; i++)
	{	
		if (isVC)
			vc(rigidCube[i].body, rPos[i], rPrevPos[i], rigidCube[i].pos, rigidCube[i].prevPos, kp, bp, dt);
		if (isVCRot)
			vcRot(rigidCube[i].body, rQuat[i], rPrevQuat[i], rigidCube[i].quat, rigidCube[i].prevQuat, kr, br, dt);
	}

	// 前フレーム実指座標の更新
	for (int i = 0; i < 2; i++)
	{
		rPrevPos[i] = rPos[i];
		rPrevQuat[i] = rQuat[i];
	}

	// オブジェクトの前フレームの座標・姿勢更新
	for (auto& cube : rigidCube) {
		cube.rePrevParam();
	}
	for (auto& capsule : rigidCapsule) {
		capsule.rePrevParam();
	}

	if (isCSV)
		writeCSV();
	
	step++;
}


void writeRigidBodyBuffer(rigidBodyBuf& buf) {

	PxU32 numActors;
	vector<PxRigidActor*> actors;
	vector<PxU32> nbShapes;
	vector<bool> sleeping;
	vector<vector<PxMat44>> shapePose;
	vector<vector<geomParam>> geom;
	vector<vector<bool>> isTrigger;


	PxScene* scene;
	PxGetPhysics().getScenes(&scene, 1);
	PxU32 nbActors = scene->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC | PxActorTypeFlag::eRIGID_STATIC);
	if (nbActors)
	{
		std::vector<PxRigidActor*> tmp(nbActors);
		scene->getActors(PxActorTypeFlag::eRIGID_DYNAMIC | PxActorTypeFlag::eRIGID_STATIC, reinterpret_cast<PxActor**>(&tmp[0]), nbActors);
		actors = tmp;

		//renderActorsの中身

		numActors = static_cast<PxU32>(actors.size());

		PxShape* shapes[MAX_NUM_ACTOR_SHAPES];
		for (PxU32 i = 0; i < numActors; i++)
		{
			nbShapes.push_back(actors[i]->getNbShapes());
			PX_ASSERT(nbShapes[i] <= MAX_NUM_ACTOR_SHAPES);
			actors[i]->getShapes(shapes, nbShapes[i]);

			bool tmpSleeping;
			if (changeColorForSleepingActors)
				tmpSleeping = actors[i]->is<PxRigidDynamic>() ? actors[i]->is<PxRigidDynamic>()->isSleeping() : false;
			else
				tmpSleeping = false;
			sleeping.push_back(tmpSleeping);

			vector<PxMat44> tmpVectorShapePose;
			vector<geomParam> vTmpGeom;
			vector<bool> tmpVectorIsTrigger;
			for (PxU32 j = 0; j < nbShapes[i]; j++)
			{
				PxMat44 tmpShapePose(PxShapeExt::getGlobalPose(*shapes[j], *actors[i]));
				tmpVectorShapePose.push_back(tmpShapePose);


				geomParam tmpGeomParam;
				const PxGeometry& tmpGeom = shapes[j]->getGeometry();
				tmpGeomParam.type = tmpGeom.getType();

				// ジオメトリの記録
				switch (tmpGeomParam.type)
				{
					// 立方体
				case PxGeometryType::eBOX:
				{
					const PxBoxGeometry& boxGeom = static_cast<const PxBoxGeometry&>(tmpGeom);
					tmpGeomParam.halfExtents = boxGeom.halfExtents;
				}
				break;

				// 球
				case PxGeometryType::eSPHERE:
				{
					const PxSphereGeometry& sphereGeom = static_cast<const PxSphereGeometry&>(tmpGeom);
					tmpGeomParam.radius = GLdouble(sphereGeom.radius);
				}
				break;

				// カプセル
				case PxGeometryType::eCAPSULE:
				{
					const PxCapsuleGeometry& capsuleGeom = static_cast<const PxCapsuleGeometry&>(tmpGeom);
					tmpGeomParam.capRadius = capsuleGeom.radius;
					tmpGeomParam.capHalfHeight = capsuleGeom.halfHeight;
				}
				break;

				// Convex Mesh
				case PxGeometryType::eCONVEXMESH:
				{
					const PxConvexMeshGeometry& convexGeom = static_cast<const PxConvexMeshGeometry&>(tmpGeom);

					//Compute triangles for each polygon.
					tmpGeomParam.convScale = convexGeom.scale.scale;
					PxConvexMesh* mesh = convexGeom.convexMesh;
					const PxU32 nbPolys = mesh->getNbPolygons();
					const PxU8* polygons = mesh->getIndexBuffer();
					const PxVec3* verts = mesh->getVertices();
					PxU32 nbVerts = mesh->getNbVertices();
					PX_UNUSED(nbVerts);

					prepareVertexBuffer();

					tmpGeomParam.convNumTotalTriangles = 0;
					for (PxU32 i = 0; i < nbPolys; i++)
					{
						PxHullPolygon data;
						mesh->getPolygonData(i, data);

						const PxU32 nbTris = PxU32(data.mNbVerts - 2);
						const PxU8 vref0 = polygons[data.mIndexBase + 0];
						PX_ASSERT(vref0 < nbVerts);
						for (PxU32 j = 0; j < nbTris; j++)
						{
							const PxU32 vref1 = polygons[data.mIndexBase + 0 + j + 1];
							const PxU32 vref2 = polygons[data.mIndexBase + 0 + j + 2];

							//generate face normal:
							PxVec3 e0 = verts[vref1] - verts[vref0];
							PxVec3 e1 = verts[vref2] - verts[vref0];

							PX_ASSERT(vref1 < nbVerts);
							PX_ASSERT(vref2 < nbVerts);

							PxVec3 fnormal = e0.cross(e1);
							fnormal.normalize();

							pushVertex(verts[vref0], verts[vref1], verts[vref2], fnormal);
							tmpGeomParam.convNumTotalTriangles++;
						}
					}
					tmpGeomParam.convVertexBuffer = getVertexBuffer();
				}
				break;

				// Triangle Mesh
				case PxGeometryType::eTRIANGLEMESH:
				{
					const PxTriangleMeshGeometry& triGeom = static_cast<const PxTriangleMeshGeometry&>(tmpGeom);

					const PxTriangleMesh& mesh = *triGeom.triangleMesh;
					tmpGeomParam.triScale = triGeom.scale.scale;

					const PxU32 triangleCount = mesh.getNbTriangles();
					const PxU32 has16BitIndices = mesh.getTriangleMeshFlags() & PxTriangleMeshFlag::e16_BIT_INDICES;
					const void* indexBuffer = mesh.getTriangles();

					const PxVec3* vertices = mesh.getVertices();

					prepareVertexBuffer();

					const PxU32* intIndices = reinterpret_cast<const PxU32*>(indexBuffer);
					const PxU16* shortIndices = reinterpret_cast<const PxU16*>(indexBuffer);
					tmpGeomParam.triNumTotalTriangles = 0;
					for (PxU32 i = 0; i < triangleCount; ++i)
					{
						PxU32 vref0, vref1, vref2;
						if (has16BitIndices)
						{
							vref0 = *shortIndices++;
							vref1 = *shortIndices++;
							vref2 = *shortIndices++;
						}
						else
						{
							vref0 = *intIndices++;
							vref1 = *intIndices++;
							vref2 = *intIndices++;
						}

						const PxVec3& v0 = vertices[vref0];
						const PxVec3& v1 = vertices[vref1];
						const PxVec3& v2 = vertices[vref2];

						PxVec3 fnormal = (v1 - v0).cross(v2 - v0);
						fnormal.normalize();

						pushVertex(v0, v1, v2, fnormal);
						tmpGeomParam.triNumTotalTriangles++;
					}
					tmpGeomParam.triVertexBuffer = getVertexBuffer();
				}
				break;

				// Tetrahedron Mesh
				case PxGeometryType::eTETRAHEDRONMESH:
				{
					const int tetFaces[4][3] = { {0,2,1}, {0,1,3}, {0,3,2}, {1,2,3} };

					const PxTetrahedronMeshGeometry& tetGeom = static_cast<const PxTetrahedronMeshGeometry&>(tmpGeom);

					const PxTetrahedronMesh& mesh = *tetGeom.tetrahedronMesh;

					//Get the deformed vertices			
					const PxVec3* vertices = mesh.getVertices();
					const PxU32 tetCount = mesh.getNbTetrahedrons();
					const PxU32 has16BitIndices = mesh.getTetrahedronMeshFlags() & PxTetrahedronMeshFlag::e16_BIT_INDICES;
					const void* indexBuffer = mesh.getTetrahedrons();

					prepareVertexBuffer();

					const PxU32* intIndices = reinterpret_cast<const PxU32*>(indexBuffer);
					const PxU16* shortIndices = reinterpret_cast<const PxU16*>(indexBuffer);
					tmpGeomParam.tetNumTotalTriangles = 0;
					for (PxU32 i = 0; i < tetCount; ++i)
					{
						PxU32 vref[4];
						if (has16BitIndices)
						{
							vref[0] = *shortIndices++;
							vref[1] = *shortIndices++;
							vref[2] = *shortIndices++;
							vref[3] = *shortIndices++;
						}
						else
						{
							vref[0] = *intIndices++;
							vref[1] = *intIndices++;
							vref[2] = *intIndices++;
							vref[3] = *intIndices++;
						}

						for (PxU32 j = 0; j < 4; ++j)
						{
							const PxVec3& v0 = vertices[vref[tetFaces[j][0]]];
							const PxVec3& v1 = vertices[vref[tetFaces[j][1]]];
							const PxVec3& v2 = vertices[vref[tetFaces[j][2]]];

							PxVec3 fnormal = (v1 - v0).cross(v2 - v0);
							fnormal.normalize();

							pushVertex(v0, v1, v2, fnormal);
							tmpGeomParam.tetNumTotalTriangles++;
						}
					}
					tmpGeomParam.tetVertexBuffer = getVertexBuffer();
				}
				break;

				default:
					break;
				}
				vTmpGeom.push_back(tmpGeomParam);
				tmpVectorIsTrigger.push_back(cb ? cb->isTrigger(shapes[j]) : shapes[j]->getFlags() & PxShapeFlag::eTRIGGER_SHAPE);
			}
			shapePose.push_back(tmpVectorShapePose);
			geom.push_back(vTmpGeom);
			isTrigger.push_back(tmpVectorIsTrigger);
		}



		buf.renew(numActors, actors, nbShapes, sleeping, shapePose, geom, isTrigger);
	}
}

void renderRigidBody(rigidBodyBuf* buffer, PxVec3 dynColor) 
{
	rigidBodyBuf buf = *buffer;

	bool shadows = true;
	PxVec3 shadowDir(0.0f, -0.7071067f, -0.7071067f);
	PxReal shadowMat[] = { 1,0,0,0, -shadowDir.x / shadowDir.y,0,-shadowDir.z / shadowDir.y,0, 0,0,1,0, 0,0,0,1 };

	for (PxU32 i = 0; i < buf.numActors; i++)
	{
		for (PxU32 j = 0; j < buf.nbShapes[i]; j++)
		{
			if (buf.isTrigger[i][j])
				glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

			// render object
			glPushMatrix();
			glMultMatrixf(&buf.shapePose[i][j].column0.x);
			if (buf.sleeping[i])
			{
				const PxVec3 darkColor = dynColor * 0.25f;
				glColor4f(darkColor.x, darkColor.y, darkColor.z, 1.0f);
			}
			else
				glColor4f(dynColor.x, dynColor.y, dynColor.z, 1.0f);

			Snippets::exeRenderGeometry2(buf.geom[i][j]);
			glPopMatrix();

			glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

			if (shadows)
			{
				glPushMatrix();
				glMultMatrixf(shadowMat);
				glMultMatrixf(&buf.shapePose[i][j].column0.x);
				glDisable(GL_LIGHTING);
				//glColor4f(0.1f, 0.2f, 0.3f, 1.0f);
				glColor4f(0.1f, 0.1f, 0.1f, 1.0f);
				Snippets::exeRenderGeometry2(buf.geom[i][j]);
				glEnable(GL_LIGHTING);
				glPopMatrix();
			}
		}
	}
}

void writeSoftBodyBuffer(softBodyBuf& buf) 
{
	vector<SoftBody> softBodies;
	PxU32 softBodiesSize;
	vector<SoftBody*> sb;
	vector<PxArray<PxVec4>> deformedPositionsInvMass;
	vector<PxShape*> sbShape;
	vector<PxMat44> sbShapePose;
	vector<geomParam> sbGeom;

	softBodies = gSoftBodies;
	softBodiesSize = softBodies.size();

	if (softBodiesSize > 0) 
	{
		for (PxU32 i = 0; i < softBodiesSize; i++)
		{
			sb.push_back(&softBodies[i]);
			deformedPositionsInvMass.push_back(sb[i]->mPositionsInvMass);

			// renderSoftBody()の中身
			sbShape.push_back(sb[i]->mSoftBody->getShape());

			const PxMat44 tmpShapePose(PxIdentity);
			sbShapePose.push_back(tmpShapePose);

			const PxGeometry& tmpGeom = sbShape[i]->getGeometry();
			const PxTetrahedronMeshGeometry& tetGeom = static_cast<const PxTetrahedronMeshGeometry&>(tmpGeom);
			geomParam tmpGeomParam;
			const PxTetrahedronMesh& mesh = *tetGeom.tetrahedronMesh;	
			sbGeom.push_back(tmpGeomParam);	

			// renderSoftBodyGeometry()の中身
			const int tetFaces[4][3] = { {0,2,1}, {0,1,3}, {0,3,2}, {1,2,3} };

			const PxU32 tetCount = mesh.getNbTetrahedrons();
			//printf("tetCount %d\n", tetCount);
			const PxU32 has16BitIndices = mesh.getTetrahedronMeshFlags() & PxTetrahedronMeshFlag::e16_BIT_INDICES;
			const void* indexBuffer = mesh.getTetrahedrons();

			//prepareVertexBuffer();

			const PxU32* intIndices = reinterpret_cast<const PxU32*>(indexBuffer);
			const PxU16* shortIndices = reinterpret_cast<const PxU16*>(indexBuffer);
			sbGeom[i].sbNumTotalTriangles = 0;

			for (PxU32 j = 0; j < tetCount; j++) 
			{
				PxU32 vref[4];
				if (has16BitIndices)
				{
					vref[0] = *shortIndices++;
					vref[1] = *shortIndices++;
					vref[2] = *shortIndices++;
					vref[3] = *shortIndices++;
				}
				else
				{
					vref[0] = *intIndices++;
					vref[1] = *intIndices++;
					vref[2] = *intIndices++;
					vref[3] = *intIndices++;
				}

				for (PxU32 k = 0; k < 4; k++)
				{
					const PxVec4& v0 = deformedPositionsInvMass[i][vref[tetFaces[k][0]]];
					const PxVec4& v1 = deformedPositionsInvMass[i][vref[tetFaces[k][1]]];
					const PxVec4& v2 = deformedPositionsInvMass[i][vref[tetFaces[k][2]]];

					PxVec3 fnormal = (v1.getXYZ() - v0.getXYZ()).cross(v2.getXYZ() - v0.getXYZ());
					fnormal.normalize();

					sbGeom[i].sbVertexBuffer.push_back(fnormal); sbGeom[i].sbVertexBuffer.push_back(v0.getXYZ());
					sbGeom[i].sbVertexBuffer.push_back(fnormal); sbGeom[i].sbVertexBuffer.push_back(v1.getXYZ());
					sbGeom[i].sbVertexBuffer.push_back(fnormal); sbGeom[i].sbVertexBuffer.push_back(v2.getXYZ());
					sbGeom[i].sbNumTotalTriangles++;

					// 座標の取得
					vector<PxVec3> tmp_pos;
					tmp_pos.push_back(v0.getXYZ()); tmp_pos.push_back(v1.getXYZ()); tmp_pos.push_back(v2.getXYZ());

					for (auto pos : tmp_pos)
					{
						if (!count(begin(sbGeom[i].sbVertexPos), end(sbGeom[i].sbVertexPos), pos))
						{
							sbGeom[i].sbVertexPos.push_back(pos);
						}
					}
				}
			}

			//sbGeom[i].sbVertexBuffer = getVertexBuffer();
			
		}

		buf.renew(softBodies, softBodiesSize, sb, deformedPositionsInvMass, sbShape, sbShapePose, sbGeom);
	}
}

void renderSoftBody(softBodyBuf* buffer, PxVec3 rcaColor) {

	softBodyBuf buf = *buffer;

	bool shadows = true;
	const PxVec3 shadowDir(0.0f, -0.7071067f, -0.7071067f);
	const PxReal shadowMat[] = { 1,0,0,0, -shadowDir.x / shadowDir.y,0,-shadowDir.z / shadowDir.y,0, 0,0,1,0, 0,0,0,1 };
	PxU32 alpha = 1.0f;

	for (PxU32 i = 0; i < buf.softBodiesSize; i++) 
	{
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);	// 変形物体の描画方法(骨格だけとか，面を表示するとか)

		glPushMatrix();
		glMultMatrixf(&buf.sbShapePose[i].column0.x);
		glColor4f(rcaColor.x, rcaColor.y, rcaColor.z, alpha);

		Snippets::exeRenderSoftBodyGeometry2(buf.sbGeom[i]);
		glPopMatrix();
		

		if (shadows)
		{
			glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);	// 変形物体の影の描画方法
			glPushMatrix();
			glMultMatrixf(shadowMat);
			glMultMatrixf(&buf.sbShapePose[i].column0.x);
			glDisable(GL_LIGHTING);
			glColor4f(0.1f, 0.2f, 0.3f, 1.0f);
			glColor4f(0.1f, 0.1f, 0.1f, 1.0f);
			Snippets::exeRenderSoftBodyGeometry2(buf.sbGeom[i]);
			glEnable(GL_LIGHTING);
			glPopMatrix();
		}
	}
}

void initPhysics(bool /*interactive*/)
{
	gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);
	gPvd = PxCreatePvd(*gFoundation);
	PxPvdTransport* transport = PxDefaultPvdSocketTransportCreate(PVD_HOST, 5425, 10);
	gPvd->connect(*transport, PxPvdInstrumentationFlag::eALL);

	// initialize cuda
	PxCudaContextManagerDesc cudaContextManagerDesc;
	gCudaContextManager = PxCreateCudaContextManager(*gFoundation, cudaContextManagerDesc, PxGetProfilerCallback());
	if (gCudaContextManager && !gCudaContextManager->contextIsValid())
	{
		gCudaContextManager->release();
		gCudaContextManager = NULL;
		printf("Failed to initialize cuda context.\n");
	}

	PxTolerancesScale scale;												// これで単位系変わってる？
	gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, scale, true, gPvd);
	PxInitExtensions(*gPhysics, gPvd);

	//PxCookingParams params(scale);
	//params.meshWeldTolerance = 0.001f;
	//params.meshPreprocessParams = PxMeshPreprocessingFlags(PxMeshPreprocessingFlag::eWELD_VERTICES);
	//params.buildTriangleAdjacencies = false;
	//params.buildGPUData = true;

	PxSceneDesc sceneDesc(gPhysics->getTolerancesScale());
	sceneDesc.gravity = gravity;															//重力

	if (!sceneDesc.cudaContextManager)
		sceneDesc.cudaContextManager = gCudaContextManager;

	sceneDesc.flags |= PxSceneFlag::eENABLE_GPU_DYNAMICS;
	sceneDesc.flags |= PxSceneFlag::eENABLE_PCM;

	PxU32 numCores = SnippetUtils::getNbPhysicalCores();
	gDispatcher = PxDefaultCpuDispatcherCreate(numCores == 0 ? 0 : numCores - 1);
	sceneDesc.cpuDispatcher = gDispatcher;
	sceneDesc.filterShader = PxDefaultSimulationFilterShader;
	sceneDesc.flags |= PxSceneFlag::eENABLE_ACTIVE_ACTORS;

	sceneDesc.sceneQueryUpdateMode = PxSceneQueryUpdateMode::eBUILD_ENABLED_COMMIT_DISABLED;
	sceneDesc.broadPhaseType = PxBroadPhaseType::eGPU;
	sceneDesc.gpuMaxNumPartitions = 8;
\
	sceneDesc.filterShader = softBodyRigidBodyFilter;
	sceneDesc.solverType = PxSolverType::eTGS;

	gScene = gPhysics->createScene(sceneDesc);
	PxPvdSceneClient* pvdClient = gScene->getScenePvdClient();
	if (pvdClient)
	{
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
	}

	gMaterial = gPhysics->createMaterial(floorStaticFriction, floorDynamicFriction, floorCoefficient);		//床のマテリアル

	PxRigidStatic* groundPlane = PxCreatePlane(*gPhysics, PxPlane(0, 1, 0, 0), *gMaterial);					//床の生成
	gScene->addActor(*groundPlane);

	my_initPhysics();

}


void stepPhysics(bool /*interactive*/)
{
	const PxReal dt_ = 1.000000 / timeStepPerSecond;

	my_stepPhysics();

	gScene->simulate(dt_);
	gScene->fetchResults(true);


	for (PxU32 i = 0; i < gSoftBodies.size(); i++)
	{
		SoftBody* sb = &gSoftBodies[i];
		sb->copyDeformedVerticesFromGPU();
	}

	// 剛体レンダリングのためのダブルバッファへの書き込み
	if (rigidBufFlag == 0) {
		writeRigidBodyBuffer(rigidBodyBuf0);
		rigidBodyBuffer = &rigidBodyBuf0;
		rigidBufFlag = 1;
	}
	else {
		writeRigidBodyBuffer(rigidBodyBuf1);
		rigidBodyBuffer = &rigidBodyBuf1;
		rigidBufFlag = 0;
	}

	// 変形物体レンダリングのためのバッファへの書き込み
	if (softBufFlag == 0) {
		writeSoftBodyBuffer(softBodyBuf0);
		softBodyBuffer = &softBodyBuf0;
		softBufFlag = 1;
	}
	else {
		writeSoftBodyBuffer(softBodyBuf1);
		softBodyBuffer = &softBodyBuf1;
		softBufFlag = 0;
	}
}

void cleanupPhysics(bool /*interactive*/)
{
	for (PxU32 i = 0; i < gSoftBodies.size(); i++)
		gSoftBodies[i].release();
	gSoftBodies.clear();
	PX_RELEASE(gScene);
	PX_RELEASE(gDispatcher);
	PX_RELEASE(gPhysics);
	PxPvdTransport* transport = gPvd->getTransport();
	gPvd->release();
	transport->release();
	PxCloseExtensions();
	gCudaContextManager->release();
	PX_RELEASE(gFoundation);

	printf("Snippet Softbody done.\n");
}

int main(int, const char* const*)
{
#ifdef RENDER_SNIPPET



	/*extern void simuLoop();
	simuLoop();*/

	extern void loopMain();
	loopMain();


	/*extern void renderLoop();
	renderLoop();*/



#else
	static const PxU32 frameCount = 100;
	initPhysics(false);
	for (PxU32 i = 0; i < frameCount; i++)
		stepPhysics(false);
	cleanupPhysics(false);
#endif

	return 0;
}