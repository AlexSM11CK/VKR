#include "Lowcamera.h"

using namespace Unigine;
using namespace Unigine::Math;

REGISTER_COMPONENT(Lowcamera)

using namespace Unigine;

void Lowcamera::Init()
{
	gui = Gui::getCurrent();
	if (!gui)
	{
		Log::error("Lowcamera: Gui::getCurrent() returned null\n");
		return;
	}

	crosshair = WidgetLabel::create(gui, "+");
	if (!crosshair)
	{
		Log::error("Lowcamera: failed to create WidgetLabel\n");
		return;
	}

	crosshair->setFontSize(64);
	crosshair->setHidden(true);

	// Добавляем в GUI и центрируем
	gui->addChild(crosshair, Gui::ALIGN_CENTER | Gui::ALIGN_TOP);
}

void Lowcamera::Update()
{
	if (!crosshair)
		return;

	PlayerPtr current_player = Game::getPlayer();
	PlayerPtr self_player = checked_ptr_cast<Player>(node);

	if (!current_player || !self_player)
	{
		crosshair->setHidden(true);
		return;
	}

	crosshair->setHidden(current_player != self_player);
}

void Lowcamera::Shutdown()
{
	if (gui && crosshair)
		gui->removeChild(crosshair);

	crosshair = nullptr;
	gui = nullptr;
}