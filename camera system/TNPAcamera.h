#pragma once
#include <UnigineComponentSystem.h>
#include <UnigineGame.h>
#include <UnigineInput.h>
#include <UnigineVisualizer.h>

class TNPAcamera : public Unigine::ComponentBase
{
public:
	COMPONENT_DEFINE(TNPAcamera, Unigine::ComponentBase);
	COMPONENT_INIT(Init);
	COMPONENT_UPDATE(Update);
	PROP_PARAM(Node, cam_1);
	PROP_PARAM(Node, cam_2);
	PROP_PARAM(Node, cam_3);
protected:
	void Init();
	void Update();
private:
	Unigine::Vector<Unigine::NodePtr> cameras;
	Unigine::InputGamePadPtr gamepad;
	int current_camera = 0;
	bool button_prev = false;

	void switchToCurrentCamera();
};

