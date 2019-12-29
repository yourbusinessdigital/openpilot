const int VOLKSWAGEN_MAX_STEER = 300;               // 3.0 Nm (EPS side max of 3.0Nm with fault if violated)
const int VOLKSWAGEN_MAX_RT_DELTA = 188;            // 10 max rate * 50Hz send rate * 250000 RT interval / 1000000 = 125 ; 125 * 1.5 for safety pad = 187.5
const uint32_t VOLKSWAGEN_RT_INTERVAL = 250000;     // 250ms between real time checks
const int VOLKSWAGEN_MAX_RATE_UP = 10;              // 5.0 Nm/s available rate of change from the steering rack (EPS side delta-limit of 5.0 Nm/s)
const int VOLKSWAGEN_MAX_RATE_DOWN = 300;           // Arbitrary rate of change available on reduction
const int VOLKSWAGEN_DRIVER_TORQUE_ALLOWANCE = 80;
const int VOLKSWAGEN_DRIVER_TORQUE_FACTOR = 1;

struct sample_t volkswagen_torque_driver;           // last few driver torques measured
int volkswagen_rt_torque_last = 0;
int volkswagen_desired_torque_last = 0;
uint32_t volkswagen_ts_last = 0;
int volkswagen_gas_prev = 0;

// Safety-relevant CAN messages for the Volkswagen MQB platform.
#define MSG_EPS_01              0x09F
#define MSG_MOTOR_20            0x121
#define MSG_ACC_06              0x122
#define MSG_HCA_01              0x126
#define MSG_GRA_ACC_01          0x12B
#define MSG_LDW_02              0x397
#define MSG_KLEMMEN_STATUS_01   0x3C0
#define MSG_PQ_HCA              0xD2
#define MSG_LDW_1               0x5BE

static void volkswagen_init(int16_t param) {
  UNUSED(param); // May use param in the future to indicate MQB vs PQ35/PQ46/NMS vs MLB, or wiring configuration.
  controls_allowed = 1;
}

static void volkswagen_rx_hook(CAN_FIFOMailBox_TypeDef *to_push) {
  // FIXME: blowing the brains out of this check until torque-on-road is tested AND we have working GRA cancellation
  UNUSED(to_push);
  return;
}

static int volkswagen_tx_hook(CAN_FIFOMailBox_TypeDef *to_send) {
  // FIXME: blowing the brains out of this check until torque-on-road is tested AND we have working GRA cancellation
  UNUSED(to_send);
  return 1;
}

static int volkswagen_fwd_hook(int bus_num, CAN_FIFOMailBox_TypeDef *to_fwd) {
  int addr = GET_ADDR(to_fwd);
  int bus_fwd = -1;

  // NOTE: Will need refactoring for other bus layouts, such as no-forwarding at camera or J533 running-gear CAN

  switch (bus_num) {
    case 0:
      // Forward all traffic from J533 gateway to Extended CAN devices.
      bus_fwd = 2;
      break;
    case 2:
      if ((addr == MSG_PQ_HCA) | (addr == MSG_LDW_1)) {
        // OP takes control of the Heading Control Assist and Lane Departure Warning messages from the camera.
        bus_fwd = -1;
      } else {
        // Forward all remaining traffic from Extended CAN devices to J533 gateway.
        bus_fwd = 0;
      }
      break;
    default:
      // No other buses should be in use; fallback to do-not-forward.
      bus_fwd = -1;
      break;
  }

  return bus_fwd;
}

const safety_hooks volkswagen_hooks = {
  .init = volkswagen_init,
  .ignition = default_ign_hook,
  .rx = volkswagen_rx_hook,
  .tx = volkswagen_tx_hook,
  .tx_lin = nooutput_tx_lin_hook,
  .fwd = volkswagen_fwd_hook,
};
