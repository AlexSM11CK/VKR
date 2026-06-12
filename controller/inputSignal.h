
#include <UnigineComponentSystem.h>
#include <UnigineGame.h>
#include <UnigineInput.h>
#include <UnigineMathlib.h>



class inputSignal /*: public Unigine::ComponentBase*/
{
public:

	float marsh = 0;
	float lag = 0;
	float depth = 0;
	float course = 0;
	float pitch = 0;
	float roll = 0;

	void init();
	void update();
protected:
	
private:
	const float max_marsh_vel = 1.8;
	const float max_lag_vel = 1.7;
	const float max_depth_vel = 1.4;
	const float d_kurs = 0.2 * Unigine::Math::Consts::DEG2RAD;
	const float d_pitch = 0.2 * Unigine::Math::Consts::DEG2RAD;
	const float d_roll = 0.2 * Unigine::Math::Consts::DEG2RAD;
	float plus_roll = 0;
	float minus_roll = 0;
	Unigine::InputGamePadPtr gamepad;
};

