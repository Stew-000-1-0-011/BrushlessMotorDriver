#pragma once

#include <stm32f4xx.h>
#include <stm32f4xx_hal_tim.h>

#include <cmath>

#include <CRSLib/include/std_int.hpp>
#include <CRSLib/include/pid_controller.hpp>

namespace Chibarobo2022
{
	using namespace CRSLib::IntegerTypes;
	class BrushlessMotor
	{
		TIM_TypeDef *const encoder_timer;
		volatile u32 *const pwm_timer_ccr;
		
		const u32 pwm_timer_arr; // not array. ARR.
		const float rad_per_count;
		const float max_velocity;
		const float min_velocity;
		const float critical_threshold;
		const u32 neutral_ccr;

		// floatはatomicらしいので何かと扱いやすい。
		CRSLib::PidController<float> pid_velocity;

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
			const CRSLib::PidController<float>& pid_velocity,
			const float critical_threshold,
			const u32 neutral_ccr
		) noexcept:
			encoder_timer{encoder_timer},
			pwm_timer_ccr{pwm_timer_ccr},
			pwm_timer_arr{pwm_timer_arr},
			rad_per_count{rad_per_count},
			max_velocity{max_velocity},
			min_velocity{min_velocity},
			pid_velocity{pid_velocity},
			critical_threshold{critical_threshold},
			neutral_ccr{neutral_ccr}
		{}

		void update_from_encoder(const float delta_time) noexcept
		{
			previous_count = current_count;
			current_count = encoder_timer->CNT;
			this->delta_time = delta_time;
		}

		void change_pwm(const float target_velocity) noexcept
		{
			const float current_vel = get_current_velocity();
			const float error = target_velocity - current_vel;
			pid_velocity.update(error);
			const float pid_vel_output = pid_velocity.calculate(error);
			if(std::fabs(pid_vel_output - current_vel) < critical_threshold)
			{
				set_ccr(pid_vel_output);
			}
			else
			{
				step_out = true;
				set_ccr(neutral_ccr);
			}
		}

		float get_current_velocity() const noexcept
		{
			return (current_count - previous_count) * rad_per_count / delta_time;
		}

		// 考える。
		// // BLDCは脱調する。が、今回は脱調を考えない。
		bool is_step_out() const noexcept
		{
			// そもそもBLDCって脱調するの？ -> する。
			return step_out;
		}

	private:
		void set_ccr(const float pid_velocity_out) const noexcept
		{
			const float rate = (pid_velocity_out + get_current_velocity() - min_velocity) / (max_velocity - min_velocity);
			
			const u32 next_pwm_timer_ccr = pwm_timer_arr * rate + neutral_ccr;
			if(pwm_timer_arr < next_pwm_timer_ccr) next_pwm_timer_ccr = pwm_timer_arr;

			*pwm_timer_ccr = next_pwm_timer_ccr;
		}
	};
}