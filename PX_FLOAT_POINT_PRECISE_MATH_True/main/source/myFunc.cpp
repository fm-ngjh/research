#include "PxPhysicsAPI.h"
#include "foundation/PxTime.h"
#include "GL/freeglut.h"
#include "GL/glut.h"
#include <string>
#include <time.h>
#include <chrono>
#include <vector>
#include "myFunc.h"

using namespace physx;
using namespace	std;

float physxTSPS = 0.0f;
float lastTime0 = 0.0f;
chrono::system_clock::time_point lastTime1;

namespace myFunc
{	
	// 1秒間のシミュレーション回数(計測したdtの逆数)
	float get_physxTSPS() {
		return physxTSPS;
	}

	// 文字列の描画
	void render_string(float x, float y, float z, const string str) {
		glRasterPos3f(x, y, z);

		for(int i=0; i<str.size(); i++)
			glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, str[i]);
	}

	void print_vec3(PxVec3 vec3)
	{
		printf("(%f, %f, %f)\n", vec3.x, vec3.y, vec3.z);
	}

	void print_vec4(PxVec4 vec4)
	{
		printf("(%f, %f, %f, %f)\n", vec4.w, vec4.x, vec4.y, vec4.z);
	}

	//オブジェクトの座標書き換え(移動)
	void movePos(PxRigidDynamic* body, PxVec3 delta)	
	{								
		body->setGlobalPose(PxTransform(body->getGlobalPose().p + delta), true);
	}

	
	void initLastTime()
	{
		lastTime1 = chrono::system_clock::now();
	}

	// フレーム間の時間を取得
	double get_dt() {
		chrono::system_clock::time_point currentTime = chrono::system_clock::now();
		double dt = std::chrono::duration_cast<std::chrono::microseconds>(currentTime - lastTime1).count() / static_cast<double>(1000000);
		lastTime1 = currentTime;

		physxTSPS = 1 / dt;
		return dt;
	}

	// 並進方向のVC力を計算し加える
	PxVec3 vc(PxRigidDynamic* body, PxVec3 pos, PxVec3 prevPos, PxVec3 objPos, PxVec3 objPrevPos, float k, float b, float dt)
	{
		PxVec3 f = PxVec3(PxIdentity);
		if (dt != 0) {
			PxVec3 diffPos = pos - objPos;
			PxVec3 diffVel = ((pos - prevPos) - (objPos - objPrevPos)) / dt;

			f = k * diffPos + b * diffVel;
			body->addForce(f, PxForceMode::eFORCE);
			
		}
		return f;
	}

	// 回転方向のVC力を計算し加える
	PxVec3 vcRot(PxRigidDynamic* body, PxQuat quat, PxQuat prevQuat, PxQuat objQuat, PxQuat objPrevQuat, float kr, float br, float dt) 
	{
		PxVec3 f = PxVec3(PxIdentity);
		if (dt != 0) {
			PxQuat diffQuat = quat * (objQuat.getConjugate());
			PxQuat diffPrevQuat = prevQuat * (objPrevQuat.getConjugate());
			f = diffQuat.getImaginaryPart() * kr + ((diffQuat * (diffPrevQuat.getConjugate())).getImaginaryPart() / dt) * br;
			body->addTorque(f, PxForceMode::eFORCE);
		}
		return f;		
	}

	// クォータニオンによる姿勢をオイラー角に変換する
	PxVec3 q2ea(PxQuat q) {
		double q0 = q.w, q1 = q.x, q2 = q.y, q3 = q.z;
		double q0q0 = q0 * q0;
		double q0q1 = q0 * q1;
		double q0q2 = q0 * q2;
		double q0q3 = q0 * q3;
		double q1q1 = q1 * q1;
		double q1q2 = q1 * q2;
		double q1q3 = q1 * q3;
		double q2q2 = q2 * q2;
		double q2q3 = q2 * q3;
		double q3q3 = q3 * q3;
		double roll = PxAtan2(2.0 * (q2q3 + q0q1), q0q0 - q1q1 - q2q2 + q3q3);
		double pitch = PxAsin(2.0 * (q0q2 - q1q3));
		double yaw = PxAtan2(2.0 * (q1q2 + q0q3), q0q0 + q1q1 - q2q2 - q3q3);
		return PxVec3(roll * 180 / PxPi, pitch * 180 / PxPi, yaw * 180 / PxPi);
	}

	// 点と平面の距離を計算する
	float cal_distance_point_to_surface(PxVec3 p, PxVec3 s0, PxVec3 s1, PxVec3 s2)
	{
		// s0s1ベクトルとs0s2ベクトルの法線ベクトルを求める
		PxVec3 normal_vector = (s1 - s0).cross(s2 - s0);

		// 平面の方程式 ax+by+cz+d=0 におけるa, b, c, dを求める
		float a = normal_vector.x;
		float b = normal_vector.y;
		float c = normal_vector.z;
		float d = -a * s0.x + -b * s0.y - c * s0.z;

		// pと平面の距離を計算
		float distance = abs(a * p.x + b * p.y + c * p.z + d) / sqrt(a * a + b * b + c * c);

		// 点が面の表と裏どちら側にいるか(表なら正)
		if ((p - s0).dot(normal_vector) < 0)
			return distance;
		else
			return -distance;
	}

	vector<PxVec3> cal_cube_vertex_pos(float wdh, PxVec3 pos, PxQuat q)
	{
		vector<PxVec3> vertexPos;

		// 回転していない状態での立方体の中心から各頂点へのベクトル
		float l = wdh / 2;
		vertexPos.push_back(PxVec3(l, l, l)); vertexPos.push_back(PxVec3(l, -l, l)); vertexPos.push_back(PxVec3(l, l, -l)); vertexPos.push_back(PxVec3(l, -l, -l));
		vertexPos.push_back(PxVec3(-l, l, l)); vertexPos.push_back(PxVec3(-l, -l, l)); vertexPos.push_back(PxVec3(-l, l, -l)); vertexPos.push_back(PxVec3(-l, -l, -l));

		for (int i = 0; i < 8; i++)
		{
			//PxQuat p = PxIdentity;
			//// 実部0 虚部が頂点へのベクトルのクォータニオン
			//p.w = 0; p.x = vertexPos[i].x; vertexPos[i].y; vertexPos[i].z;

			//// 以下の式を計算し虚部を取り出すと，あるベクトルをqで回転させたベクトルが計算できる
			//p = q * p * q.getConjugate();
			////vertexPos[i] = p.getImaginaryPart() + pos + PxVec3(0, l, 0);

			vertexPos[i] = q.rotate(vertexPos[i]) + pos;
		}

		return vertexPos;
	}
}