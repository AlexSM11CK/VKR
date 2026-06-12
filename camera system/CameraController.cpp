#include "CameraController.h"

using namespace Unigine;
using namespace Math;

REGISTER_COMPONENT(CameraController)

void CameraController::Init()
{
    MainCamera = Game::getPlayer();          // текущая активная камера/плеер
    target = LookAtObj.get();               // если задано в инспекторе
    if (!target) target = node;             // иначе следим за тем узлом, где компонент

    if (!MainCamera || !target) return;

    // фиксируем мировую дельту "камера - цель"
    offset_world = MainCamera->getWorldPosition() - target->getWorldPosition();
    offset_inited = true;
}

void CameraController::Update()
{
    if (!offset_inited) return;
    if (!MainCamera || !target) return;

    // только позиция. Rotation НЕ трогаем.
    Vec3 desired = target->getWorldPosition() + offset_world;
    MainCamera->setWorldPosition(desired);

    // ВАЖНО: НЕ вызывать lookAt, worldLookAt, setWorldRotation и т.п.
}



