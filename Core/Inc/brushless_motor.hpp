#pragma once

#include <stm32f4xx.h>
#include <stm32f4xx_hal_tim.h>

#include <CRSLib/include/std_int.hpp>
#include <CRSLib/include/pid_controller.hpp>

namespace Chibarobo2022
{
	using namespace CRSLib::IntegerTypes;
	class BrushlessMotor
	{
		TIM_TypeDef *const encoder_timer;
		volatile u32 *const pwm_timer_ccr;
		
		const u32 pwm_timer_arr;
		const float rad_per_count;
		const float max_velocity;
		const float min_velocity;

		// floatはatomicらしいので何かと扱いやすい。
		CRSLib::PidController<float> pid_velocity;
		// CRSLib::PidController<float> pid_position;

		u32 current_count{}; // [rad] * constant
		u32 previous_count{}; // [rad] * constant
		float delta_time{}; // [s] update間の間隔。
		
		bool step_out{false};

	public:
		BrushlessMotor
		(
			TIM_TypeDef *const encoder_timer,
			volatile u32 *const pwm_timer_ccr,
			const u32 pwm_timer_arr,
			const float rad_per_count,
			const float max_velocity,
			const float min_velocity,
			const CRSLib::PidController<float>& pid_velocity
		) noexcept:
			encoder_timer{encoder_timer},
			pwm_timer_ccr{pwm_timer_ccr},
			pwm_timer_arr{pwm_timer_arr},
			rad_per_count{rad_per_count},
			max_velocity{max_velocity},
			min_velocity{min_velocity},
			pid_velocity{pid_velocity}
		{}

		void update_from_encoder(const float delta_time) noexcept
		{
			previous_count = current_count;
			current_count = encoder_timer->CNT;
			this->delta_time = delta_time;
		}

		void change_pwm(const float target_velocity) noexcept
		{
			const float error = target_velocity - get_current_velocity();
			pid_velocity.update(error);
			set_ccr(pid_velocity.calculate(error));
		}

		float get_current_velocity() const noexcept
		{
			return (current_count - previous_count) * rad_per_count / delta_time;
		}

		bool is_step_out()
		{
			// そもそもBLDCって脱調するの？
		}

	private:
		void set_ccr(const float pid_velocity_out) const noexcept
		{
			const float rate = (pid_velocity_out + get_current_velocity() - min_velocity) / max_velocity - min_velocity;
			
			if(rate < 0) rate = 0;
			else if(1 < rate) rate = 1;

			*pwm_timer_ccr = pwm_timer_arr * rate;
		}
	};
}