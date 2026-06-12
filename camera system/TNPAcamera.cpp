#include "TNPAcamera.h"

using namespace Unigine;

REGISTER_COMPONENT(TNPAcamera)

void TNPAcamera::Init()
{
	cameras.clear();

	if (cam_1) cameras.append(cam_1.get());
	if (cam_2) cameras.append(cam_2.get());
	if (cam_3) cameras.append(cam_3.get());

	current_camera = 0;
	switchToCurrentCamera();
}

void TNPAcamera::Update()
{
	if (Input::getNumGamePads() <= 0)
	{
		gamepad = nullptr;
	}
	gamepad = Input::getGamePad(0);


	// клавиша C
	//bool button_now = Input::isKeyPressed(Input::KEY_C);

	// переключение только по фронту нажатия
	if (/*(button_now && !button_prev) */Input::isKeyDown(Input::KEY_C) || (gamepad->isButtonDown(Input::GAMEPAD_BUTTON_A)))
	{
		if (cameras.size() > 0)
		{
			current_camera = (current_camera + 1) % cameras.size();
			switchToCurrentCamera();
		}
	}

	//button_prev = button_now;
}

void TNPAcamera::switchToCurrentCamera()
{
	if (cameras.size() == 0)
		return;

	PlayerPtr player = checked_ptr_cast<Player>(cameras[current_camera]);
	if (player)
	{
		Game::setPlayer(player);
		return;
	}

	Log::error("CameraSwitcher: selected node is not a Player\n");
}
