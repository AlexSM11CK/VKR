#include "SubmarineThrusterController.h"

using namespace Unigine;
using namespace Unigine::Math;

REGISTER_COMPONENT(SubmarineThrusterController)


void SubmarineThrusterController::Init()
{
    Visualizer::setEnabled(true);
    Console::setOnscreen(true);

    // инициализируем angle-hold текущим положением аппарата
    quat q = node->getWorldTransform().getRotate();
    mat3 R(q);
    vec3 ang_deg = decomposeRotationXYZ(R); // (тангаж, крен, рыскание) в градусах

    pitch_ref = ang_deg.x * Consts::DEG2RAD;
    yaw_ref = ang_deg.z * Consts::DEG2RAD;
    roll_ref = ang_deg.y * Consts::DEG2RAD;
	q_ref = node->getWorldTransform().getRotate();

}

void SubmarineThrusterController::Update()
{
    const float dt = Game::getIFps();
    if (dt <= 0.0f) return;

    dbg_timer += dt;
    if (dbg_timer < 0.2f) return; // 5 Гц
    dbg_timer = 0.0f;

    vpu.init();
}

void SubmarineThrusterController::UpdatePhysics()
{
    BodyRigidPtr capsule_body = node->getObjectBodyRigid();
    if (!capsule_body) return;
    //if (!thrusters_ok()) return;

    const float dt = Physics::getIFps();
    if (dt <= 0.0f) return;

    // Оси корпуса в мире
    vec3 forward = node->getWorldDirection(AXIS_Y);
    vec3 right = node->getWorldDirection(AXIS_X);
    vec3 up = node->getWorldDirection(AXIS_Z);

    quat curr = node->getWorldTransform().getRotate();   // или как ты получаешь mat3 вращения
	
    vec3 a = decomposeRotationXYZ((mat3)curr);

    // ------------------------------------------------------------
    // 1) Ввод (удержание): cmd ∈ {-1,0,+1}
    // ------------------------------------------------------------

    vpu.update();

	float yaw_old = yaw_ref;
	float pitch_old = pitch_ref;
	float roll_old = roll_ref;

	float cmd_yaw = vpu.course;
	float cmd_pitch = vpu.pitch;
	float cmd_roll = vpu.roll;
	// текущий курс, вокруг которого уже повернулись локальные оси аппарата
	float cy = cosf(yaw_ref);
	float sy = sinf(yaw_ref);
	// перевод локальных команд pitch/roll в эйлеровы приращения pitch_ref/roll_ref
	float d_pitch_ref = cy * cmd_pitch - sy * cmd_roll;
	float d_roll_ref = sy * cmd_pitch + cy * cmd_roll;

	yaw_ref += cmd_yaw;
	pitch_ref += d_pitch_ref;
	roll_ref += d_roll_ref;

	yaw_ref = clamp(yaw_ref, -Consts::PI, Consts::PI);
	pitch_ref = clamp(pitch_ref, -Consts::PI / 2, Consts::PI / 2);
	roll_ref = clamp(roll_ref, -Consts::PI / 2, Consts::PI / 2);
	// реальные приращения после ограничений
	vec3 ref_deg(
		pitch_ref * Consts::RAD2DEG, // X
		roll_ref * Consts::RAD2DEG, // Y
		yaw_ref * Consts::RAD2DEG  // Z
	);

	mat4 R_ref = composeRotationXYZ(ref_deg);
	quat q_ref = (quat)R_ref;

    // ------------------------------------------------------------
    // 2) Текущие скорости по осям корпуса
    // ------------------------------------------------------------
    const vec3 vel_w = capsule_body->getLinearVelocity();
    float v_f = dot(vel_w, forward);
    float v_r = dot(vel_w, right);
    float v_u = dot(vel_w, up);

    const vec3 w_w = capsule_body->getAngularVelocity();  // рад/с, в мире
    float w_yaw = dot(w_w, up);                            // вокруг оси up аппарата
    float w_roll = dot(w_w, forward);
    float w_pitch = dot(w_w, right);

    // ------------------------------------------------------------
    // 3) Цели скорости (из твоих измерений в идеальных условиях)
    // ------------------------------------------------------------

    //float e_f = vpu.marsh - v_f;
    //float e_r = vpu.lag - v_r;
    float e_u = vpu.depth - v_u;

    // Ошибки по углам
	//-------------------------------------------------------------
	quat q_err = inverse(curr) * q_ref;
	vec3 err_deg = decomposeRotationXYZ(mat3(q_err));

	float e_pitch = err_deg.x;// *Consts::DEG2RAD;
	float e_roll = err_deg.y;// *Consts::DEG2RAD;
	float e_yaw = err_deg.z;// *Consts::DEG2RAD;
	//-------------------------------------------------------

	const float V_HOLD = 0.08f;

	if (fabs(vpu.marsh) < 1e-3f && fabs(v_f) < V_HOLD)
		i_vel.x *= 0.90f;

	if (fabs(vpu.lag) < 1e-3f && fabs(v_r) < V_HOLD)
		i_vel.y *= 0.90f;

	if (fabs(vpu.depth) < 1e-3f && fabs(v_u) < V_HOLD)
		i_vel.z *= 0.90f;

    // ------------------------------------------------------------
    // 4) PI -> суммарные силы по осям корпуса
    // ------------------------------------------------------------
    const float T = max_thrust.get();     // Н на движитель

    //vec3 i_prev = i_vel;

    const float EDEAD = v_dead.get();
    //if (fabs(e_f) < EDEAD) e_f = 0.0f;
    //if (fabs(e_r) < EDEAD) e_r = 0.0f;
    if (fabs(e_u) < EDEAD) e_u = 0.0f;

    // интегратор
    //i_vel.x += e_f * dt;
    //i_vel.y += e_r * dt;
    i_vel.z += e_u * dt;

    const float IMAX = i_max.get();
    i_vel.x = clamp(i_vel.x, -IMAX, IMAX);
    i_vel.y = clamp(i_vel.y, -IMAX, IMAX);
    i_vel.z = clamp(i_vel.z, -IMAX, IMAX);

    // PI (одинаковый для всех осей)
	float F_surge = kp_x.get() * vpu.marsh;// e_f + ki_x.get() * i_vel.x;
	float F_sway = kp_z.get() * vpu.lag;// e_r + ki_z.get() * i_vel.y;
    float F_heave = kp_y.get() * e_u + ki_y.get() * i_vel.z;

    // предварительный осевой лимит
    F_surge = clamp(F_surge, float(-500), float(500));
    F_sway = clamp(F_sway, float(-500), float(500));
    F_heave = clamp(F_heave, float(-500), float(500));

	// ------------------------------------------------------------
	// 6) P + I по углу и демпфирование по угловой скорости
	// ------------------------------------------------------------
	i_ang.x += e_yaw * dt;
	i_ang.y += e_pitch * dt;
	i_ang.z += e_roll * dt; 

	const float IANG_MAX = i_ang_max.get();
	i_ang.x = clamp(i_ang.x, -IANG_MAX, IANG_MAX);
	i_ang.y = clamp(i_ang.y, -IANG_MAX, IANG_MAX);
	i_ang.z = clamp(i_ang.z, -IANG_MAX, IANG_MAX);

	float yaw_u = k1_yaw * (i_ang.x * ki_yaw + e_yaw /** kp_yaw*/) - w_yaw * k2_yaw * Consts::RAD2DEG;
	float pitch_u = k1_pitch * (i_ang.y * ki_pitch + e_pitch/* * kp_pitch*/) - w_pitch * k2_pitch * Consts::RAD2DEG;
	float roll_u = k1_roll * (i_ang.z * ki_roll + e_roll/* * kp_roll*/) - w_roll * k2_roll * Consts::RAD2DEG;
	// Ограничение углового канала
	yaw_u = clamp(yaw_u, float(-500), float(500));
	pitch_u = clamp(pitch_u, float(-500), float(500));
	roll_u = clamp(roll_u, float(-500), float(500));



	double W_dv = k_dv / (T_dv * dt + 1);

	const float yaw_th = yaw_u * W_dv;
	const float pitch_th = pitch_u * W_dv;
	const float roll_th = roll_u * W_dv;

	// ------------------------------------------------------------
	// 7) Базовые линейные каналы на 8 движителей
	// ------------------------------------------------------------
	const float surge_per = F_surge * W_dv;
	const float sway_per = F_sway * W_dv;
	const float heave_per = F_heave * W_dv;
	//vec3 lin_total[8];
	//int i = 0;

	auto apply_thr = [&](const NodePtr& thr, float th, int pos)
	{
		if (!thr) return;
		BodyRigidPtr tb = thr->getObjectBodyRigid();
		if (!tb) return;

		vec3 F;
		if (pos == 1)
		{
			F = thr->getWorldDirection(AXIS_NX) * th;// lin_total / 8
		}
		else if (pos == 0)
		{
			F = thr->getWorldDirection(AXIS_X) * th;
		}
		//lin_total[i++] = F;
		tb->addForce(F);
	};

	CableOneLinkBall* cable =
		Unigine::ComponentSystem::get()->getComponent<CableOneLinkBall>(cabel_node.get());

	if (cable->rad_vec.length() - cable->len_cabel <= 0)
	{
		//======MARSH=======
		apply_thr(thr_left_front_up.get(), surge_per, front);
		apply_thr(thr_left_front_down.get(), surge_per, front);
		apply_thr(thr_right_front_up.get(), surge_per, front);
		apply_thr(thr_right_front_down.get(), surge_per, front);
		// Задние
		apply_thr(thr_left_back_up.get(), surge_per, back);
		apply_thr(thr_left_back_down.get(), surge_per, back);
		apply_thr(thr_right_back_up.get(), surge_per, back);
		apply_thr(thr_right_back_down.get(), surge_per, back);

		//======LAG========
		apply_thr(thr_left_front_up.get(), sway_per, front);
		apply_thr(thr_left_front_down.get(), sway_per, front);
		apply_thr(thr_right_front_up.get(), sway_per, back);
		apply_thr(thr_right_front_down.get(), sway_per, back);
		// Задние
		apply_thr(thr_left_back_up.get(), sway_per, front);
		apply_thr(thr_left_back_down.get(), sway_per, front);
		apply_thr(thr_right_back_up.get(), sway_per, back);
		apply_thr(thr_right_back_down.get(), sway_per, back);

		//======DEPTH======
		apply_thr(thr_left_front_up.get(), heave_per, front);
		apply_thr(thr_left_front_down.get(), heave_per, back);
		apply_thr(thr_right_front_up.get(), heave_per, front);
		apply_thr(thr_right_front_down.get(), heave_per, back);
		// Задние
		apply_thr(thr_left_back_up.get(), heave_per, front);
		apply_thr(thr_left_back_down.get(), heave_per, back);
		apply_thr(thr_right_back_up.get(), heave_per, front);
		apply_thr(thr_right_back_down.get(), heave_per, back);

		//======COURSE======
		apply_thr(thr_left_front_up.get(), yaw_th, back);
		apply_thr(thr_left_front_down.get(), yaw_th, back);
		apply_thr(thr_right_front_up.get(), yaw_th, front);
		apply_thr(thr_right_front_down.get(), yaw_th, front);
		// Задние
		apply_thr(thr_left_back_up.get(), yaw_th, front);
		apply_thr(thr_left_back_down.get(), yaw_th, front);
		apply_thr(thr_right_back_up.get(), yaw_th, back);
		apply_thr(thr_right_back_down.get(), yaw_th, back);

		//======PITCH======
		apply_thr(thr_left_front_up.get(), pitch_th, back);
		apply_thr(thr_left_front_down.get(), pitch_th, front);
		apply_thr(thr_right_front_up.get(), pitch_th, back);
		apply_thr(thr_right_front_down.get(), pitch_th, front);
		// Задние
		apply_thr(thr_left_back_up.get(), pitch_th, front);
		apply_thr(thr_left_back_down.get(), pitch_th, back);
		apply_thr(thr_right_back_up.get(), pitch_th, front);
		apply_thr(thr_right_back_down.get(), pitch_th, back);

		//======ROLL======
		apply_thr(thr_left_front_up.get(), roll_th, front);
		apply_thr(thr_left_front_down.get(), roll_th, back);
		apply_thr(thr_right_front_up.get(), roll_th, back);
		apply_thr(thr_right_front_down.get(), roll_th, front);
		// Задние
		apply_thr(thr_left_back_up.get(), roll_th, front);
		apply_thr(thr_left_back_down.get(), roll_th, back);
		apply_thr(thr_right_back_up.get(), roll_th, back);
		apply_thr(thr_right_back_down.get(), roll_th, front);
	}
	

	//Внешнее воздействие
	const vec3 Fa = vec3(0,0, 103.45);
	capsule_body->addWorldForce(CV->getWorldPosition(),Fa);// (cb_world, Fa);
	//capsule_body->addForce(Fa);

	float water_resistion_marsh = -(24.71 * v_f * fabs(v_f) + 2.471 * v_f + 3.51 * (v_f - v_old_f)/dt);
	float water_resistion_lag = -(29.61 * v_r *fabs(v_r) + 2.961 * v_r + 4.13 * (v_r - v_old_l)/dt);
	float water_resisting_depth = -(31.96 * v_u * fabs(v_u) + 3.196 * v_u + 4.13 * (v_u - v_old_d)/dt);
	const vec3 resist = forward * water_resistion_marsh + right * water_resistion_lag + up * water_resisting_depth;
	capsule_body->addForce(resist);

	

	float resist_yaw = -(4.73 * w_yaw * fabs(w_yaw) + 0.473 * w_yaw + 0.056 * (w_yaw - w_old_yaw)/dt);
	float resist_pitch = -(6.11 * w_pitch * fabs(w_pitch) + 0.611 * w_pitch + 0.037 * (w_pitch - w_old_pitch)/dt);
	float resist_roll = -(3.92 * w_roll * fabs(w_roll) + 0.392 * w_roll + 0.017 * (w_roll - w_old_roll)/dt);
	const vec3 resist_moment = up * resist_yaw  + right * resist_pitch  + forward * resist_roll;
	capsule_body->addTorque(resist_moment);

	v_old_f = v_f;	v_old_l = v_r;	v_old_d = v_u;
	w_old_yaw = w_yaw;	w_old_pitch = w_pitch;	w_old_roll = w_roll;

	// ------------------------------------------------------------
	// 11) Отладочный вывод
	// ------------------------------------------------------------
	static float tlog = 0.0f;
	tlog += dt;

	if (tlog >= 0.25f)
	{
		tlog = 0.0f;

		Log::message(
			"[LIN] ref=(%.2f %.2f %.2f) vel=(%.2f %.2f %.2f)\n"// F=(%.2f %.2f %.2f)\n",
			"[ANG] ref=(%.2f %.2f %.2f)  VPU=(%.5f %.5f %.5f) E=(%.2f %.2f %.2f)\n"
			/*"LFU =(%.5f, %.5f, %.5f)    LFD =(%.5f, %.5f, %.5f)    RFU =(%.5f, %.5f, %.5f)    RFD =(%.5f, %.5f, %.5f)\n"
			"LBU =(%.5f, %.5f, %.5f)    LBD =(%.5f, %.5f, %.5f)    RBU =(%.5f, %.5f, %.5f)    RBD =(%.5f, %.5f, %.5f)\n"*/,
			//"[INPUT] marsh=%.2f lug=%.2f depth=%.2f kurs=%.2f\n",
			vpu.marsh, vpu.lag, vpu.depth,
			v_f, v_r, v_u,
			//F_surge, F_sway, F_heave

			yaw_ref * Consts::RAD2DEG,
			pitch_ref * Consts::RAD2DEG,
			roll_ref * Consts::RAD2DEG,

			/*yaw * Consts::RAD2DEG,
			pitch * Consts::RAD2DEG,
			roll * Consts::RAD2DEG,
			ang=(%.2f %.2f %.2f)*/

			vpu.course, vpu.pitch, vpu.roll,
			e_yaw, e_pitch, e_roll
			/*lin_total[0].x, lin_total[0].y, lin_total[0].z,
			lin_total[1].x, lin_total[1].y, lin_total[1].z,
			lin_total[2].x, lin_total[2].y, lin_total[2].z,
			lin_total[3].x, lin_total[3].y, lin_total[3].z,
			lin_total[4].x, lin_total[4].y, lin_total[4].z, 
			lin_total[5].x, lin_total[5].y, lin_total[5].z, 
			lin_total[6].x, lin_total[6].y, lin_total[6].z,
			lin_total[7].x, lin_total[7].y, lin_total[7].z*/
		);
	}

}

