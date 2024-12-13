const CONFIG = {
  pid: {
      Kp_angle: { min: 0, max: 10, step: 0.1, default: 5 },
      Ki_angle: { min: 0, max: 5, step: 0.01, default: 0 },
      Kd_angle: { min: 0, max: 2, step: 0.1, default: 0.3 },
      Kp_pos: { min: 0, max: 10, step: 0.1, default: 2 },
      Ki_pos: { min: 0, max: 5, step: 0.01, default: 0.5 },
      Kd_pos: { min: 0, max: 2, step: 0.1, default: 0.1 },
      Kp_speed: { min: 0, max: 10, step: 0.1, default: 3 },
      Ki_speed: { min: 0, max: 5, step: 0.01, default: 0.8 },
      Kd_speed: { min: 0, max: 2, step: 0.1, default: 0.2 },
  },
  modes: {
      available: ['manual', 'auto', 'debug'],
      default: 'manual',
  },
};
