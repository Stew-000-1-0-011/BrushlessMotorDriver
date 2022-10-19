#include <main.h>
#include <CRSLib/Can/RM0390/include/can_manager.hpp>
#include <CRSLib/Can/RM0390/include/filter_manager.hpp>
#include <wrapper.h>
#include <brushless_motor.hpp>

using namespace CRSLib;
using namespace Can;
using namespace RM0390;

using namespace Chibarobo2022;

void rx_dispatcher(const RxFrame& rx_frame)
{
	switch(rx_frame.header.id)
	{
		case /*TODO*/0:
			;
		
	}
}

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

	BrushlessMotor bldc1{/*TODO*/};

	while(true)
	{
		bldc1.
	}
}