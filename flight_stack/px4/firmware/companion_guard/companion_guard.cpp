#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/workqueue.h>
#include <lib/perf/perf_counter.h>
#include <drivers/drv_hrt.h>
#include <math.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>

using namespace time_literals;

class CompanionGuard final : public ModuleBase<CompanionGuard>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	CompanionGuard() :
		ModuleParams(nullptr),
		ScheduledWorkItem("companion_guard", px4::wq_configurations::nav_and_controllers)
	{}

	~CompanionGuard() override
	{
		perf_free(_loop_perf);
	}

	static int task_spawn(int argc, char *argv[])
	{
		CompanionGuard *instance = new CompanionGuard();

		if (instance == nullptr) {
			PX4_ERR("alloc failed");
			return -1;
		}

		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

		delete instance;
		_object.store(nullptr);
		_task_id = -1;
		return PX4_ERROR;
	}

	static CompanionGuard *instantiate(int argc, char *argv[])
	{
		return nullptr;
	}

	static int custom_command(int argc, char *argv[])
	{
		return print_usage("unknown command");
	}

	static int print_usage(const char *reason = nullptr)
	{
		if (reason) {
			PX4_WARN("%s", reason);
		}

		PRINT_MODULE_DESCRIPTION(
			R"DESCR_STR(
Companion-side safety gate for PX4 Offboard operations.

It monitors Offboard setpoint freshness, battery threshold, and geofence radius and forces
LOITER/RTL on policy violations.
)DESCR_STR");

		PRINT_MODULE_USAGE_NAME("companion_guard", "template");
		PRINT_MODULE_USAGE_COMMAND("start");
		PRINT_MODULE_USAGE_COMMAND("stop");
		PRINT_MODULE_USAGE_COMMAND("status");
		return 0;
	}

	bool init()
	{
		ScheduleOnInterval(100_ms);
		return true;
	}

	int print_status() override
	{
		PX4_INFO("running");
		PX4_INFO("last action: %u", static_cast<unsigned>(_last_action));
		PX4_INFO("last setpoint age ms: %.1f", (hrt_elapsed_time(&_last_setpoint_timestamp) / 1e3f));
		return 0;
	}

private:
	enum class GuardAction : uint8_t {
		NONE = 0,
		LOITER = 1,
		RTL = 2,
	};

	void Run() override
	{
		if (should_exit()) {
			ScheduleClear();
			exit_and_cleanup();
			return;
		}

		perf_begin(_loop_perf);

		if (_parameter_update_sub.updated()) {
			parameter_update_s param_update{};
			_parameter_update_sub.copy(&param_update);
			updateParams();
		}

		battery_status_s battery{};
		if (_battery_sub.update(&battery)) {
			_battery = battery;
		}

		vehicle_local_position_s lpos{};
		if (_local_pos_sub.update(&lpos)) {
			_local_pos = lpos;
		}

		trajectory_setpoint_s traj{};
		if (_trajectory_setpoint_sub.update(&traj)) {
			_last_setpoint_timestamp = traj.timestamp;
		}

		vehicle_status_s status{};
		if (!_vehicle_status_sub.update(&status)) {
			perf_end(_loop_perf);
			return;
		}

		if (status.nav_state != vehicle_status_s::NAVIGATION_STATE_OFFBOARD) {
			_last_action = GuardAction::NONE;
			perf_end(_loop_perf);
			return;
		}

		const bool stale_setpoint = is_setpoint_stale();
		const bool low_battery = is_battery_low();
		const bool geofence_breach = is_geofence_breached();

		if (low_battery) {
			trigger_rtl("low_battery");
		} else if (stale_setpoint || geofence_breach) {
			trigger_loiter(stale_setpoint ? "stale_setpoint" : "geofence_breach");
		}

		perf_end(_loop_perf);
	}

	bool is_setpoint_stale() const
	{
		const hrt_abstime timeout_us = static_cast<hrt_abstime>(_p_com_of_loss_t.get() * 1e6f);
		if (_last_setpoint_timestamp == 0) {
			return true;
		}
		return hrt_elapsed_time(&_last_setpoint_timestamp) > timeout_us;
	}

	bool is_battery_low() const
	{
		if (_battery.connected == 0) {
			return false;
		}
		if (_battery.remaining < 0.f) {
			return false;
		}
		return _battery.remaining <= _p_bat_low_thr.get();
	}

	bool is_geofence_breached() const
	{
		if (!_local_pos.xy_valid) {
			return false;
		}

		const float radius_m = sqrtf(_local_pos.x * _local_pos.x + _local_pos.y * _local_pos.y);
		const float limit = _p_gf_max_hor_dist.get();

		if (limit <= 0.f) {
			return false;
		}

		return radius_m > limit;
	}

	void trigger_loiter(const char *reason)
	{
		if (_last_action == GuardAction::LOITER && hrt_elapsed_time(&_last_action_timestamp) < 500_ms) {
			return;
		}

		publish_vehicle_command(vehicle_command_s::VEHICLE_CMD_NAV_LOITER_UNLIM);
		_last_action = GuardAction::LOITER;
		_last_action_timestamp = hrt_absolute_time();
		PX4_WARN("guard action LOITER: %s", reason);
	}

	void trigger_rtl(const char *reason)
	{
		if (_last_action == GuardAction::RTL && hrt_elapsed_time(&_last_action_timestamp) < 500_ms) {
			return;
		}

		publish_vehicle_command(vehicle_command_s::VEHICLE_CMD_NAV_RETURN_TO_LAUNCH);
		_last_action = GuardAction::RTL;
		_last_action_timestamp = hrt_absolute_time();
		PX4_WARN("guard action RTL: %s", reason);
	}

	void publish_vehicle_command(uint16_t cmd)
	{
		vehicle_command_s vcmd{};
		vcmd.command = cmd;
		vcmd.param1 = 1.0f;
		vcmd.target_system = 1;
		vcmd.target_component = 1;
		vcmd.source_system = 1;
		vcmd.source_component = 1;
		vcmd.confirmation = 0;
		vcmd.from_external = false;
		vcmd.timestamp = hrt_absolute_time();
		_vehicle_cmd_pub.publish(vcmd);
	}

	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _battery_sub{ORB_ID(battery_status)};
	uORB::Subscription _local_pos_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _trajectory_setpoint_sub{ORB_ID(trajectory_setpoint)};
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
	uORB::Publication<vehicle_command_s> _vehicle_cmd_pub{ORB_ID(vehicle_command)};

	battery_status_s _battery{};
	vehicle_local_position_s _local_pos{};
	hrt_abstime _last_setpoint_timestamp{0};
	hrt_abstime _last_action_timestamp{0};
	GuardAction _last_action{GuardAction::NONE};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::COM_OF_LOSS_T>) _p_com_of_loss_t,
		(ParamFloat<px4::params::BAT_LOW_THR>) _p_bat_low_thr,
		(ParamFloat<px4::params::GF_MAX_HOR_DIST>) _p_gf_max_hor_dist
	)

	perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, "companion_guard: loop")};
};

extern "C" __EXPORT int companion_guard_main(int argc, char *argv[])
{
	return CompanionGuard::main(argc, argv);
}
