#include "CameraController.h"

using namespace Unigine;
using namespace Math;

REGISTER_COMPONENT(CameraController)

void CameraController::Init()
{
    MainCamera = Game::getPlayer();          // òåêóùàÿ àêòèâíàÿ êàìåðà/ïëååð
    target = LookAtObj.get();               // åñëè çàäàíî â èíñïåêòîðå
    if (!target) target = node;             // èíà÷å ñëåäèì çà òåì óçëîì, ãäå êîìïîíåíò

    if (!MainCamera || !target) return;

    // ôèêñèðóåì ìèðîâóþ äåëüòó "êàìåðà - öåëü"
    offset_world = MainCamera->getWorldPosition() - target->getWorldPosition();
    offset_inited = true;
}

void CameraController::Update()
{
    if (!offset_inited) return;
    if (!MainCamera || !target) return;

    // òîëüêî ïîçèöèÿ. Rotation ÍÅ òðîãàåì.
    Vec3 desired = target->getWorldPosition() + offset_world;
    MainCamera->setWorldPosition(desired);

    // ÂÀÆÍÎ: ÍÅ âûçûâàòü lookAt, worldLookAt, setWorldRotation è ò.ï.
}



