const int VW_MAX_STEER = 300;               // 3.0 nm
const int VW_MAX_RT_DELTA = 128;            // max delta torque allowed for real time checks
const uint32_t VW_RT_INTERVAL = 250000;     // 250ms between real time checks
const int VW_MAX_RATE_UP = 16;
const int VW_MAX_RATE_DOWN = 32;
const int VW_DRIVER_TORQUE_ALLOWANCE = 100;
const int VW_DRIVER_TORQUE_FACTOR = 4;

int vw_ignition_started = 0;
struct sample_t vw_torque_driver;           // last few driver torques measured
int vw_rt_torque_last = 0;
int vw_desired_torque_last = 0;
uint32_t vw_ts_last = 0;

static void vw_init(int16_t param) {
  UNUSED(param); // May use param in the future to indicate MQB vs PQ35/PQ46/NMS vs MLB, or wiring configuration.
  controls_allowed = 0;
  vw_ignition_started = 0;
}

static int vw_ign_hook(void) {
  // While we do monitor VW Terminal 15 (ignition-on) state, we are not currently acting on it. We may do so in the
  // future for harness integrations at the camera (where we only have T30 unswitched power) instead of the gateway
  // (where we have both T30 and T15 ignition-switched power). For now, use the default GPIO pin behavior.

  // return vw_ignition_started;
  return -1;
}

static void vw_rx_hook(CAN_FIFOMailBox_TypeDef *to_push) {
  int bus = GET_BUS(to_push);
  int addr = GET_ADDR(to_push);

  // Monitor Klemmen_Status_01.ZAS_Kl_15 for Terminal 15 (ignition-on) status, but we make no use of it at the moment.
  if (bus == 0 && addr == 0x3c0) {
    uint32_t ign = (to_push->RDLR) & 0x200;
    vw_ignition_started = ign > 0;
  }

  // Update driver input torque samples from EPS_01.Driver_Strain for absolute torque, and EPS_01.Driver_Strain_VZ
  // for the direction.
  if (bus == 0 && addr == 0x9f) {
    int torque_driver_new = (to_push->RDLR & 0x1f00) | ((to_push->RDLR >> 16) & 0xFF);
    uint8_t sign = (to_push->RDLR & 0x8000) > 0;
    if (sign == 1) torque_driver_new *= -1;
    update_sample(&vw_torque_driver, torque_driver_new);
  }

  // Monitor ACC_06.ACC_Status_ACC for stock ACC status. Because the current MQB port is lateral-only, OP's control
  // allowed state is directly driven by stock ACC engagement.
  if (addr == 0x122) {
    uint8_t acc_status = (GET_BYTE(to_push,7) & 0x70) >> 4;
    controls_allowed = (acc_status == 3) ? true : false;
  }
}

static int vw_tx_hook(CAN_FIFOMailBox_TypeDef *to_send) {
  int addr = GET_ADDR(to_send);
  int violation = 0;

  // Safety check for HCA_01 Heading Control Assist torque.
  if (addr == 0x126) {
    int desired_torque = ((to_send->RDHR & 0x3f) << 8) | ((to_send->RDHR >> 8) & 0xFF);
    uint8_t sign = (to_send->RDHR & 0x80) > 0;
    if (sign == 1) desired_torque *= -1;

    uint32_t ts = TIM2->CNT;

    if (controls_allowed) {
      // *** global torque limit check ***
      violation |= max_limit_check(desired_torque, VW_MAX_STEER, -VW_MAX_STEER);

      // *** torque rate limit check ***
      violation |= driver_limit_check(desired_torque, vw_desired_torque_last, &vw_torque_driver,
        VW_MAX_STEER, VW_MAX_RATE_UP, VW_MAX_RATE_DOWN,
        VW_DRIVER_TORQUE_ALLOWANCE, VW_DRIVER_TORQUE_FACTOR);
      vw_desired_torque_last = desired_torque;

      // *** torque real time rate limit check ***
      violation |= rt_rate_limit_check(desired_torque, vw_rt_torque_last, VW_MAX_RT_DELTA);

      // every RT_INTERVAL set the new limits
      uint32_t ts_elapsed = get_ts_elapsed(ts, vw_ts_last);
      if (ts_elapsed > VW_RT_INTERVAL) {
        vw_rt_torque_last = desired_torque;
        vw_ts_last = ts;
      }
    }

    // no torque if controls is not allowed
    if (!controls_allowed && (desired_torque != 0)) {
      violation = 1;
    }

    // reset to 0 if either controls is not allowed or there's a violation
    if (violation || !controls_allowed) {
      vw_desired_torque_last = 0;
      vw_rt_torque_last = 0;
      vw_ts_last = ts;
    }

  }

  // TODO: Implement force-cancel via GRA_ACC_01 message spamming, which Panda will need to allow specially

  if (violation) {
    // Temporarily disable
    return true;
  } else {
    return true;
  }
}

static int vw_fwd_hook(int bus_num, CAN_FIFOMailBox_TypeDef *to_fwd) {
  int addr = GET_ADDR(to_fwd);
  int bus_fwd = -1;

  // TODO: Will need refactoring for other bus layouts, for example, camera-side split or J533 running-gear xmit only
  switch(bus_num) {
    case 0:
      // Forward all traffic from the J533 gateway to downstream Extended CAN bus devices
      bus_fwd = 1;
      break;
    case 1:
      if(addr == 0x126 || addr == 0x397) {
        // Discard the car's 0x126 HCA_01 and 0x397 LDW_02 in favor of OpenPilot's version
        bus_fwd = -1;
      } else {
        // Forward all remaining traffic from Extended CAN devices to J533 gateway
        bus_fwd = 0;
      }
      break;
    default:
      // No other buses should be in use; fallback to do not forward.
      bus_fwd = -1;
      break;
  }

  return bus_fwd;
}

const safety_hooks vw_hooks = {
  .init = vw_init,
  .rx = vw_rx_hook,
  .tx = vw_tx_hook,
  .tx_lin = nooutput_tx_lin_hook,
  .ignition = vw_ign_hook,
  .fwd = vw_fwd_hook,
};
