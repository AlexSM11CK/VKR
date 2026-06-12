#include "CableWinch.h"
#include "CableLinkConstraint.h"

REGISTER_COMPONENT(CableOneLinkBall)

using namespace Unigine;
using namespace Unigine::Math;


float CableOneLinkBall::addLink()
{
	source_body = ferst_caps->getObjectBodyRigid();//getRigidBody(node);
	if (!source_body)
		return 0;

	// На всякий случай включаем body исходной капсулы
	source_body->setEnabled(true);

	created_link_node = cloneTemplateLink();
	if (!created_link_node)
		return 0;

	created_link_body = created_link_node->getObjectBodyRigid(); //getRigidBody(created_link_node);
	if (!created_link_body)
		return 0;

	// ВАЖНО: включаем физическое тело клона
	created_link_body->setEnabled(true);

	const Vec3 source_pos = ferst_caps->getWorldPosition();
	const quat source_rot = ferst_caps->getWorldRotation();

	// Направление роста новой капсулы в мировых координатах
	vec3 dir = (vec3)(garage->getWorldPosition() - source_pos);
	if (dir.length() < 1e-6f)
		return 0;
	dir.normalize();
	/*vec3 dir = source_rot * local_link_axis;
	if (dir.length() < 1e-6f)
		dir = vec3(0.0f, 0.0f, -1.0f);
	else
		dir.normalize();*/

	// Центр новой капсулы на расстоянии link_length от старой
	const Vec3 new_pos = source_pos + (Vec3)dir * link_length;

	// World anchor — середина между центрами
	const Vec3 anchor_world = source_pos + (Vec3)dir * (0.5f * link_length);

	created_link_node->setWorldPosition(new_pos);
	created_link_node->setWorldRotation(source_rot);

	// Чтобы не было "выстрела" в первый тик
	created_link_body->setLinearVelocity(source_body->getLinearVelocity()*0.1);
	created_link_body->setAngularVelocity(source_body->getAngularVelocity()*0.1);

	link_joint = createBallJoint(source_body, created_link_body, anchor_world);
	if (!link_joint)
		return 0;

	notes* ptr = new notes;
	ptr->front = link;
	ptr->current = created_link_node;
	ptr->joint = link_joint;
	link = ptr;

	ferst_caps = created_link_node;
	lenght += 0.1;

	//ferst_caps->addProperty(tension.get());

	//CableLinkConstraint* comp =
	//	Unigine::ComponentSystem::get()->getComponent<CableLinkConstraint>(ferst_caps);
	//if (!comp)
	//	return 0;

	//// передаём ссылку на соседнюю ноду
	//comp->back_node = created_link_node;

	//ferst_caps->addProperty(tension.get());
	//Unigine::PropertyParameterPtr param = tension.get()->getParameterPtr("caps");
	//if (!param) 
	//	return 0; // передаём ссылку на соседнюю ноду
	//param->setValueNode(node);
	//param = tension.get()->getParameterPtr("houme");
	//if (!param)
	//	return 0; // передаём ссылку на соседнюю ноду
	//param->setValueNode(garage);
	return lenght;
}


int CableOneLinkBall::Init()
{
	if (!validateSetup())
		return 0;

	body = node->getObjectBodyRigid();
	rad_vec = node->getWorldPosition() - garage->getWorldPosition();

	link = new notes;
	link->front = nullptr;
	link->current = node;
	link->joint = nullptr;

	while((lenght - rad_vec.length())<0.5)
	{

		len_cabel = addLink();

		if (len_cabel <= 0)
			return 0;
	}
	BodyRigidPtr last_body = ferst_caps->getObjectBodyRigid();
	BodyRigidPtr garage_body = garage->getObjectBodyRigid();

	if (!last_body || !garage_body)
		return 0;

	const Vec3 last_pos = ferst_caps->getWorldPosition();
	const Vec3 garage_pos = garage->getWorldPosition();

	// Anchor ровно посередине между последней капсулой и гаражом
	const Vec3 anchor_world = (last_pos + garage_pos) * 0.5f;

	garage_joint = createBallJoint(last_body, garage_body, anchor_world);

	if (!garage_joint)
		return 0;

	/*garage_joint->setAngularLimitAngle(140);
	garage_joint->setAngularLimitFrom(0);
	garage_joint->setAngularLimitTo(0);
	garage_joint->setCollision(0);*/

	return 1;
}

bool CableOneLinkBall::validateSetup() const
{
	if (!node)
		return false;

	if (!link_template_node.get())
		return false;

	// Шаблон не должен быть этой же самой нодой
	if (link_template_node.get() == node.get())
		return false;

	if (link_length <= 0.0f)
		return false;

	if (joint_half_offset <= 0.0f)
		return false;

	return true;
}

NodePtr CableOneLinkBall::cloneTemplateLink() const
{
	NodePtr template_node = link_template_node.get();
	if (!template_node)
		return nullptr;

	NodePtr cloned = template_node->clone();
	if (!cloned)
		return nullptr;

	// В твоей версии addNode() не нужен
	cloned->setEnabled(true);
	return cloned;
}

JointBallPtr CableOneLinkBall::createBallJoint(
	const BodyPtr& body0,
	const BodyPtr& body1,
	const Vec3& anchor_world) const
{
	if (!body0 || !body1)
		return nullptr;

	JointBallPtr joint = JointBall::create(body0, body1, anchor_world);
	if (!joint)
		return nullptr;

	applyBallParams(joint);
	return joint;
}

void CableOneLinkBall::applyBallParams(const JointBallPtr& joint) const
{
	if (!joint)
		return;

	// Общие параметры
	joint->setNumIterations(15);// (joint_iterations);
	joint->setMaxForce(INFINITY);
	joint->setMaxTorque(INFINITY);

	joint->setLinearRestitution(1);// (joint_linear_restitution);
	joint->setAngularRestitution(0.5);// (joint_angular_restitution);

	joint->setLinearSoftness(1);// (joint_linear_softness);
	joint->setAngularSoftness(1);// (joint_angular_softness);

	// ЛОКАЛЬНЫЕ anchor'ы РОВНО как на твоём скрине:
	// Anchor 0 Position = (0,0,+0.05)
	// Anchor 1 Position = (0,0,-0.05)
	joint->setAnchor1(Vec3(0.0f, 0.0f, joint_half_offset));
	joint->setAnchor0(Vec3(0.0f, 0.0f, -joint_half_offset));

	// ЛОКАЛЬНЫЕ оси РОВНО как на скрине:
	// Anchor 0 Axis = (0,0,-1)
	// Anchor 1 Axis = (0,0,-1)
	joint->setAxis0(vec3(0.0f, 0.0f, -1.0f));
	joint->setAxis1(vec3(0.0f, 0.0f, -1.0f));

	// Остальные параметры как в инспекторе
	joint->setAngularDamping(1);// (joint_damping);
	joint->setAngularLimitAngle(150);
	joint->setAngularLimitFrom(-5);
	joint->setAngularLimitTo(5);
	joint->setCollision(0);
}

int CableOneLinkBall::updatePhysics()
{
	BodyRigidPtr last_body = ferst_caps->getObjectBodyRigid();
	BodyRigidPtr garage_body = garage->getObjectBodyRigid();

	rad_vec = node->getWorldPosition() - garage->getWorldPosition();
	last_body->setAngularVelocity(vec3(0,0,0));
	last_body->setLinearVelocity(vec3(0, 0, 0));
	//len_cabel = lenght;
	/*vec3 vel_ferst_caps = body->getLinearVelocity();
	float vel = vel_ferst_caps.length();
	vec3 axis = node->getWorldDirection(AXIS_NZ);*/
	
	if (( lenght - rad_vec.length()) < 0.5 && lenght < 25)
	{
		garage_joint->setEnabled(0);
		if (last_body)
			last_body->removeJoint(garage_joint);
		if (garage_body)
			garage_body->removeJoint(garage_joint);
		garage_joint = nullptr;
		len_cabel = addLink();

		last_body = ferst_caps->getObjectBodyRigid();

		const Vec3 last_pos = ferst_caps->getWorldPosition();
		const Vec3 garage_pos = garage->getWorldPosition();

		// Anchor ровно посередине между последней капсулой и гаражом
		const Vec3 anchor_world = (last_pos + garage_pos) * 0.5f;

		garage_joint = createBallJoint(last_body, garage_body, anchor_world);

		if (!garage_joint)
			return 0;

		/*garage_joint->setAngularLimitAngle(140);
		garage_joint->setAngularLimitFrom(-10);
		garage_joint->setAngularLimitTo(10);
		garage_joint->setCollision(0);*/
	}
	else if ((lenght - rad_vec.length()) >= 0.7 && lenght > 1)
	{
		if (!link || !link->front)
			return 1;

		BodyRigidPtr garage_body = garage->getObjectBodyRigid();
		if (!garage_body)
			return 0;

		// Текущий хвост
		notes* tail = link;

		// Новым хвостом станет предыдущее звено
		notes* new_tail = link->front;
		if (!new_tail || !new_tail->current)
			return 1;

		BodyRigidPtr tail_body = tail->current ? tail->current->getObjectBodyRigid() : nullptr;
		BodyRigidPtr new_tail_body = new_tail->current ? new_tail->current->getObjectBodyRigid() : nullptr;

		if (!tail_body || !new_tail_body)
			return 0;

		// 1. Снимаем хвостовой joint хвост <-> garage
		if (garage_joint)
		{
			garage_joint->setEnabled(0);
			tail_body->removeJoint(garage_joint);
			garage_body->removeJoint(garage_joint);
			garage_joint = nullptr;
		}

		// 2. Снимаем joint между новым хвостом и старым хвостом
		if (tail->joint)
		{
			tail->joint->setEnabled(0);

			// tail->joint соединяет new_tail_body <-> tail_body
			new_tail_body->removeJoint(tail->joint);
			tail_body->removeJoint(tail->joint);
			tail->joint = nullptr;
		}

		// 3. Переключаем хвост списка
		link = new_tail;
		ferst_caps = new_tail->current;
		lenght -= link_length;

		// 4. Пересобираем новый хвостовой joint: новый хвост <-> garage
		const Vec3 last_pos = ferst_caps->getWorldPosition();
		const Vec3 garage_pos = garage->getWorldPosition();
		const Vec3 anchor_world = (last_pos + garage_pos) * 0.5f;

		garage_joint = createBallJoint(new_tail_body, garage_body, anchor_world);
		if (!garage_joint)
			return 0;

		// 5. Старую хвостовую ноду выключаем и удаляем отложенно
		if (tail->current)
		{
			tail->current->setEnabled(0);
			tail->current.deleteLater();
			tail->current = nullptr;
		}

		delete tail;
	}
	return 1;
}