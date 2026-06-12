#pragma once

#include "inputSignal.h"

#include <UnigineComponentSystem.h>
#include <UnigineGame.h>
#include <UnigineInput.h>
#include <UnigineVisualizer.h>
#include <UnigineConsole.h>
#include <UniginePhysics.h>
#include <UnigineMathlib.h>
#include"CableWinch.h"

class SubmarineThrusterController : public Unigine::ComponentBase
{
public:
	COMPONENT_DEFINE(SubmarineThrusterController, Unigine::ComponentBase);
	COMPONENT_INIT(Init);
	COMPONENT_UPDATE(Update);
	COMPONENT_UPDATE_PHYSICS(UpdatePhysics);

	// === Thruster nodes (spheres with BodyRigid + fixed joints) ===
	PROP_PARAM(Node, thr_left_front_up);
	PROP_PARAM(Node, thr_left_back_up);
	PROP_PARAM(Node, thr_left_front_down);
	PROP_PARAM(Node, thr_left_back_down);
	PROP_PARAM(Node, thr_right_front_up);
	PROP_PARAM(Node, thr_right_back_up);
	PROP_PARAM(Node, thr_right_front_down);
	PROP_PARAM(Node, thr_right_back_down);
	PROP_PARAM(Node, cabel_node);
	PROP_PARAM(Node, CV);

	// === Limits ===
	PROP_PARAM(Float, max_thrust, 18.64f);      // N per thruster (vector magnitude limit)


	//----------------------------------------
	//коэффициенты ПИ-регулятор линейных контуров
	//----------------------------------------
	PROP_PARAM(Float, kp_x, 267.38f);//348.93f);   
	PROP_PARAM(Float, ki_x, 468.7f);   

	PROP_PARAM(Float, kp_z, 292.4f);//654.91f);   
	PROP_PARAM(Float, ki_z, 673.9f);

	PROP_PARAM(Float, kp_y, 686.73f);
	PROP_PARAM(Float, ki_y, 698.06f);
	//----------------------------------------
	PROP_PARAM(Float, i_max, 2.0f);    // clamp for integrator
	PROP_PARAM(Float, v_dead, 0.02f);  // deadband near zero

	//----------------------------------------
	//коэффициенты ПИ-регулятора угловых контуров
	//----------------------------------------
	PROP_PARAM(Float, k1_yaw, 1.52f);// 10.05f);
	PROP_PARAM(Float, kp_yaw, 3.67f);
	PROP_PARAM(Float, ki_yaw, 0.031f);//0.05f);
	PROP_PARAM(Float, k2_yaw, 0.41f);//33.26f);

	PROP_PARAM(Float, k1_pitch, 1.76f);// 35.91f);
	PROP_PARAM(Float, kp_pitch, 8.12f);
	PROP_PARAM(Float, ki_pitch, 0.4f);// 0.22f);
	PROP_PARAM(Float, k2_pitch, 0.45f);// 221.02f);

	PROP_PARAM(Float, k1_roll, 1.36f);// 35.91f);
	PROP_PARAM(Float, kp_roll, 8.21f);
	PROP_PARAM(Float, ki_roll, 0.67f);// 0.26f);
	PROP_PARAM(Float, k2_roll, 0.3f);// 221.02f);
	//----------------------------------------
	PROP_PARAM(Float, i_ang_max, 1.0f);   // ограничение интеграла (рад)

protected:
	void Init();
	void Update();
	void UpdatePhysics();

private:

	const float k_dv = 18.64 / 500;
	const float T_dv = 0.15;
	Unigine::Math::quat q_ref;

	Unigine::Math::vec3 i_vel = Unigine::Math::vec3(0.0f);
	// интеграл по углам: x=yaw, y=pitch, z=roll (просто договоримся так)
	Unigine::Math::vec3 i_ang = Unigine::Math::vec3(0.0f);

	bool thrusters_ok() const;


	float yaw_ref = 0.0f;
	float pitch_ref = 0.0f;
	float roll_ref = 0.0f;

	// --- измерение vmax при насыщении ---
	float vmax_sat_f = 0.0f;
	float vmax_sat_r = 0.0f;
	float vmax_sat_u = 0.0f;
	float vmax_sat_mag = 0.0f;
	float angle_ny = 0;

	float v_old_f = 0;
	float v_old_l = 0;
	float v_old_d = 0;

	float w_old_yaw = 0;
	float w_old_roll = 0;
	float w_old_pitch = 0;

	float dbg_timer = 0.0f;   // для редкого вывода
	enum position { back, front, right, left };

	//Unigine::InputGamePadPtr gamepad;

	inputSignal vpu;


};


