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

    // Êàìåðà (Player) êîòîðóþ äâèãàåì. Â ðåäàêòîðå ìîæíî íå çàäàâàòü — âîçüì¸ì Game::getPlayer()
    // Â òâî¸ì PROP_PARAM PlayerPtr íåò — ïîýòîìó ïðîñòî õðàíèì â ïåðåìåííîé.
    // Öåëü (çà êåì ñëåäèì): ìîæíî íå çàäàâàòü, åñëè êîìïîíåíò âèñèò íà êàïñóëå -> target = node
    PROP_PARAM(Node, LookAtObj);  // îñòàâèì êàê ó òåáÿ (åñëè õî÷åøü âûáèðàòü öåëü)

protected:
    void Init();
    void Update();

private:
    Unigine::PlayerPtr MainCamera;
    Unigine::NodePtr target;

    Unigine::Math::Vec3 offset_world;   // ôèêñèðîâàííîå ñìåùåíèå â ìèðå
    bool offset_inited = false;
};




