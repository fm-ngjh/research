#pragma once

#define timeStepPerSecond 80


#include "PxPhysicsAPI.h"
#include <string>
#include <vector>
using namespace std;
using namespace physx;

namespace myFunc
{
	float get_physxTSPS();

	void render_string(float x, float y, float z, string str);

	void print_vec3(PxVec3 vec3);

	void print_vec4(PxVec4 vec4);

	void movePos(PxRigidDynamic* body, PxVec3 delta);

	void initLastTime();

	double get_dt();

	PxVec3 vc(PxRigidDynamic* body, PxVec3 pos, PxVec3 prevPos, PxVec3 objPos, PxVec3 objPrevPos, float k, float b, float dt);

	PxVec3 vcRot(PxRigidDynamic* body, PxQuat quat, PxQuat prevQuat, PxQuat objQuat, PxQuat objPrevQuat, float kr, float br, float dt);

	PxVec3 q2ea(PxQuat q);

	struct geomParam
	{
		PxGeometryType::Enum type;

		// 立方体
		PxVec3 halfExtents;

		// 球
		PxF32 radius;
		
		// カプセル
		PxF32 capRadius;
		PxF32 capHalfHeight;

		// Convex Mesh
		PxVec3 convScale;
		PxU32 convNumTotalTriangles;
		const PxVec3* convVertexBuffer;

		// Triangle Mesh
		PxVec3 triScale;
		PxU32 triNumTotalTriangles;
		const PxVec3* triVertexBuffer;

		// Tetrahedron Mesh
		PxU32 tetNumTotalTriangles;
		const PxVec3* tetVertexBuffer;

		// SoftBody
		PxU32 sbNumTotalTriangles;
		vector<PxVec3> sbVertexBuffer;	// 描画に使う
		vector<PxVec3> sbVertexPos;	// メッシュ頂点の座標
	};
}