#pragma once

#include <UnigineComponentSystem.h>
#include <UnigineNodes.h>
#include <UnigineGame.h>
#include <UnigineMathLib.h>

class CameraController : public Unigine::ComponentBase
{
public:
    COMPONENT_DEFINE(CameraController, Unigine::ComponentBase);
    COMPONENT_INIT(Init);
    COMPONENT_UPDATE(Update);

    // Камера (Player) которую двигаем. В редакторе можно не задавать — возьмём Game::getPlayer()
    // В твоём PROP_PARAM PlayerPtr нет — поэтому просто храним в переменной.
    // Цель (за кем следим): можно не задавать, если компонент висит на капсуле -> target = node
    PROP_PARAM(Node, LookAtObj);  // оставим как у тебя (если хочешь выбирать цель)

protected:
    void Init();
    void Update();

private:
    Unigine::PlayerPtr MainCamera;
    Unigine::NodePtr target;

    Unigine::Math::Vec3 offset_world;   // фиксированное смещение в мире
    bool offset_inited = false;
};




