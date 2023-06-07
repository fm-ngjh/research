#include "PxPhysicsAPI.h"
#include "foundation/PxTime.h"
#include "GL/freeglut.h"
#include "GL/glut.h"
#include <string>
#include <time.h>
#include <chrono>


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

	void movePos(PxRigidDynamic* body, PxVec3 delta)	//オブジェクトの座標書き換え(移動)
	{								
		body->setGlobalPose(PxTransform(body->getGlobalPose().p + delta), true);
	}

	
	void initLastTime()
	{
		lastTime1 = chrono::system_clock::now();
	}

	double get_dt() {
		chrono::system_clock::time_point currentTime = chrono::system_clock::now();
		double dt = std::chrono::duration_cast<std::chrono::microseconds>(currentTime - lastTime1).count() / static_cast<double>(1000000);
		lastTime1 = currentTime;


		physxTSPS = 1 / dt;
		return dt;
	}

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


}