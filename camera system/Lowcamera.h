#pragma once

#include <UnigineComponentSystem.h>
#include <UnigineGame.h>
#include <UnigineVisualizer.h>
#include <UniginePlayers.h>
#include <UnigineMathLib.h>
#include <UnigineGui.h>

class Lowcamera : public Unigine::ComponentBase
{
public:
	COMPONENT_DEFINE(Lowcamera, Unigine::ComponentBase);
	COMPONENT_INIT(Init);
	COMPONENT_UPDATE(Update);
	COMPONENT_SHUTDOWN(Shutdown);

protected:
	void Init();
	void Update();
	void Shutdown();

private:
	Unigine::GuiPtr gui;
	Unigine::WidgetLabelPtr crosshair;
};

