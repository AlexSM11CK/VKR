#pragma once
#include <UnigineComponentSystem.h>
#include <UnigineNodes.h>
#include <UnigineObjects.h>
#include <UnigineWorld.h>
#include <UniginePhysics.h>

class CableOneLinkBall : public Unigine::ComponentBase
{
public:
	COMPONENT_DEFINE(CableOneLinkBall, Unigine::ComponentBase);
	COMPONENT_INIT(Init);
	COMPONENT_UPDATE_PHYSICS(updatePhysics);

	// Чистая шаблонная капсула.
	// На ней НЕ должно быть joints.
	// Этот же компонент на ней либо отсутствует, либо выключен.
	PROP_PARAM(Node, link_template_node);
	PROP_PARAM(Node, garage);
	PROP_PARAM(Property, tension, nullptr);

	// Геометрия звена
	PROP_PARAM(Float, link_length, 0.1f);       // расстояние между центрами двух капсул
	PROP_PARAM(Float, joint_half_offset, 0.05f); // локальный anchor на торце

	// Направление роста цепочки в локальной оси исходной капсулы.
	// Для твоего случая по скрину это -Z.
	PROP_PARAM(Vec3, local_link_axis, Unigine::Math::vec3(0.0f, 0.0f, -1.0f));

	// Параметры joint как на скрине
	PROP_PARAM(Int, joint_iterations, 10);
	PROP_PARAM(Float, joint_max_force, INFINITY);
	PROP_PARAM(Float, joint_max_torque, INFINITY);

	PROP_PARAM(Float, joint_linear_restitution, 1.0f);
	PROP_PARAM(Float, joint_angular_restitution, 0.5f);

	PROP_PARAM(Float, joint_linear_softness, 0.5f);
	PROP_PARAM(Float, joint_angular_softness, 0.5f);

	PROP_PARAM(Float, joint_damping, 0.5f);
	PROP_PARAM(Float, joint_angle, 90.0f);
	PROP_PARAM(Float, joint_from, -10.0f);
	PROP_PARAM(Float, joint_to, 10.0f);


private:
	Unigine::NodePtr created_link_node;
	Unigine::BodyRigidPtr source_body;
	Unigine::BodyRigidPtr created_link_body;
	Unigine::JointBallPtr link_joint;


public:
	int Init();
	int updatePhysics();
	float addLink();

	Unigine::Math::Vec3 rad_vec;
	float len_cabel;

private:
	bool validateSetup() const;
	Unigine::NodePtr cloneTemplateLink() const;
	Unigine::BodyRigidPtr getRigidBody(const Unigine::NodePtr& node_ptr) const;

	Unigine::JointBallPtr createBallJoint(
		const Unigine::BodyPtr& body0,
		const Unigine::BodyPtr& body1,
		const Unigine::Math::Vec3& anchor_world) const;

	void applyBallParams(const Unigine::JointBallPtr& joint) const;

	Unigine::NodePtr ferst_caps = node;
	float lenght = 0.1;
	Unigine::BodyRigidPtr body = nullptr;
	Unigine::JointBallPtr garage_joint = nullptr;

	struct notes {
		notes* front;
		Unigine::NodePtr current;
		Unigine::JointBallPtr joint = nullptr;
	};

	notes* link = nullptr;
	notes* front_caps = nullptr;
};

