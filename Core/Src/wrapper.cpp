#include <main.h>
#include <CRSLib/Can/RM0390/include/can_manager.hpp>
#include <CRSLib/Can/RM0390/include/filter_manager.hpp>
#include <CRSLib/Can/CommonAmongMpu/include/pack.hpp>
#include <wrapper.h>
#include <brushless_motor.hpp>

using namespace CRSLib;
using namespace Can;
using namespace RM0390;

using namespace Chibarobo2022;

void stew_wrapper(CAN_HandleTypeDef *const hcan)
{
	CanManager can_manager{hcan};

	FilterManager::dynamic_initialize();

	FilterManager::ConfigFilterArg<FilterWidth::bit32, FilterMode::mask> filter_arg
	{
		.filter =
		{
			{
				.id = /*TODO*/0,
				.mask = /*TODO*/0
			}
		},
		.fifo = /*TODO*/FifoIndex::fifo0
	};

	FilterManager::config_filter_bank(filter_arg);

	HAL_CAN_Start(hcan);

	BrushlessMotor bldc1{/*TODO*/nullptr, nullptr, 0, 0, 0, 0, PidController<float>{0, 0, 0}, 0, 0};

	u32 previous_tick = HAL_GetTick();
	while(true)
	{
		if(!can_manager.lettebox0.empty())
		{
			RxFrame frame;
			can_manager.letterbox0.receive(frame);
			const float target_vel = unpack<float>(frame.data);
			bldc1.change_pwm(target_vel);
		}

		u32 current_tick = HAL_GetTick();
		bldc1.update_from_encoder((current_tick - previous_tick) / 1000.0f);
		previous_tick = current_tick;
	}
}