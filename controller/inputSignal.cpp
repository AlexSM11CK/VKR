#include "inputSignal.h"

using namespace Unigine;
using namespace Unigine::Math;

namespace
{
	float apply_deadzone(float v, float dz = 0.01f)
	{
		if (fabsf(v) < dz)
			return 0.0f;

		// мягкая нормализация после deadzone
		const float sign = (v >= 0.0f) ? 1.0f : -1.0f;
		const float a = (fabsf(v) - dz) / (1.0f - dz);
		return sign * a;
	}
}

void inputSignal::init()
{
	if (Input::getNumGamePads() <= 0)
	{
		//Log::error("inputSignal::init(): no available gamepads\n");
		gamepad = nullptr;
		return;
	}

	gamepad = Input::getGamePad(0);

	//if (!gamepad)
		//Log::error("inputSignal::init(): Input::getGamePad(0) returned null\n");
}

void inputSignal::update()
{
	if (!gamepad)
	{
		marsh = 0.0f;
		lag = 0.0f;
		depth = 0.0f;
		return;
	}

	vec2 left_stick = gamepad->getAxesLeft();

	float lx = apply_deadzone(left_stick.x);
	float ly = apply_deadzone(left_stick.y);

	// При необходимости просто инвертируй ly знаком минус
	marsh = ly  * max_marsh_vel;
	lag = lx  * max_lag_vel;

	float tl = apply_deadzone(gamepad->getTriggerLeft(), 0.05f);
	float tr = apply_deadzone(gamepad->getTriggerRight(), 0.05f);

	depth = (tr - tl) * max_depth_vel;


	vec2 right_stic = -gamepad->getAxesRight();
	course = d_kurs * apply_deadzone(right_stic.x,0.01f);

	pitch = d_pitch * apply_deadzone(right_stic.y, 0.01f);

	plus_roll = minus_roll = 0;
	if(gamepad->isButtonPressed(Input::GAMEPAD_BUTTON_DPAD_RIGHT)) plus_roll = d_roll;
	if(gamepad->isButtonPressed(Input::GAMEPAD_BUTTON_DPAD_LEFT))minus_roll = d_roll;
	roll = plus_roll - minus_roll;
}
