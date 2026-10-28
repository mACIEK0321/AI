#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <webots/robot.h>
#include <webots/camera.h>
#include <webots/compass.h>
#include <webots/gps.h>
#include <webots/gyro.h>
#include <webots/inertial_unit.h>
#include <webots/supervisor.h>
#include <webots/motor.h>

#define CLAMP(v, lo, hi) ((v) < (lo) ? (lo) : ((v) > (hi) ? (hi) : (v)))

#define NUM_EPISODES 100

#define INIT_ROLL 0.0
#define INIT_PITCH 0.0
#define INIT_YAW 0.0
#define WIND_SPEED 0.0
#define PAYLOAD_WEIGHT 0.0

#define HOVER_ALT 1.5
#define LAND_ALT 0.1
#define MAX_DURATION 12.0
#define TOUCHDOWN_ALT 0.25
#define FORWARD_SPEED 0.35    // Smoother forward speed to prevent tipping
#define FORWARD_START 3.0
#define LAND_START 6.0        // altitude metrics are measured only while the target is HOVER_ALT (t < LAND_START)
#define SETTLE_BAND 0.05      // settling band around HOVER_ALT [m]
#define SSE_WINDOW 1.0        // steady-state error = mean |error| over the last SSE_WINDOW s before landing
#define NOT_REACHED_TIME MAX_DURATION  // rise/settling time when the drone never reaches/settles at HOVER_ALT

#define HOVER_FF 0.65        
#define VEL_KD 1.5            

static int reset_drone(WbNodeRef self, WbFieldRef trans, WbFieldRef rot,
                       const double *pos0, const double *rot0,
                       WbDeviceTag motors[4], int timestep) {
  wb_supervisor_field_set_sf_vec3f(trans, pos0);
  wb_supervisor_field_set_sf_rotation(rot, rot0);
  wb_supervisor_node_reset_physics(self);
  for (int m = 0; m < 4; ++m) wb_motor_set_velocity(motors[m], 0.0);
  int steps = (int)(600.0 / timestep);
  if (steps < 1) steps = 1;
  for (int i = 0; i < steps; i++)
    if (wb_robot_step(timestep) == -1) return 0;
  return 1;
}

// GA evaluation mode: candidates read from GA_CANDIDATES (candidate_id,P_gain,I_gain,D_gain)
typedef struct {
  char id[64];
  double P, I, D;
} Candidate;

static int load_candidates(const char *path, Candidate **out) {
  FILE *f = fopen(path, "r");
  if (!f) return -1;
  int cap = 64, n = 0;
  Candidate *c = malloc(cap * sizeof(Candidate));
  char line[512];
  if (!fgets(line, sizeof(line), f)) { fclose(f); free(c); return -1; }  // header
  while (fgets(line, sizeof(line), f)) {
    if (n == cap) { cap *= 2; c = realloc(c, cap * sizeof(Candidate)); }
    if (sscanf(line, " %63[^,],%lf,%lf,%lf", c[n].id, &c[n].P, &c[n].I, &c[n].D) == 4) n++;
  }
  fclose(f);
  *out = c;
  return n;
}

int main(int argc, char **argv) {
  wb_robot_init();
  int timestep = (int)wb_robot_get_basic_time_step();
  const double dt = timestep / 1000.0;
  srand(time(NULL));

  WbDeviceTag camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, timestep);
  WbDeviceTag imu = wb_robot_get_device("inertial unit");
  wb_inertial_unit_enable(imu, timestep);
  WbDeviceTag gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, timestep);
  WbDeviceTag compass = wb_robot_get_device("compass");
  wb_compass_enable(compass, timestep);
  WbDeviceTag gyro = wb_robot_get_device("gyro");
  wb_gyro_enable(gyro, timestep);

  WbDeviceTag camera_roll_motor = wb_robot_get_device("camera roll");
  WbDeviceTag camera_pitch_motor = wb_robot_get_device("camera pitch");

  WbDeviceTag fl_m = wb_robot_get_device("front left propeller");
  WbDeviceTag fr_m = wb_robot_get_device("front right propeller");
  WbDeviceTag rl_m = wb_robot_get_device("rear left propeller");
  WbDeviceTag rr_m = wb_robot_get_device("rear right propeller");
  WbDeviceTag motors[4] = {fl_m, fr_m, rl_m, rr_m};
  for (int m = 0; m < 4; ++m) {
    wb_motor_set_position(motors[m], INFINITY);
    wb_motor_set_velocity(motors[m], 0.0);
  }

  WbNodeRef self_node = wb_supervisor_node_get_self();
  if (!self_node) {
    printf("ERROR: not a supervisor! Set Mavic2Pro 'supervisor' to TRUE and reload the world.\n");
    wb_robot_cleanup();
    return EXIT_FAILURE;
  }
  WbFieldRef trans_field = wb_supervisor_node_get_field(self_node, "translation");
  WbFieldRef rot_field = wb_supervisor_node_get_field(self_node, "rotation");

  double pos0[3], rot0[4];
  const double *tp = wb_supervisor_field_get_sf_vec3f(trans_field);
  const double *rp = wb_supervisor_field_get_sf_rotation(rot_field);
  for (int k = 0; k < 3; k++) pos0[k] = tp[k];
  for (int k = 0; k < 4; k++) rot0[k] = rp[k];

  const char *ga_candidates = getenv("GA_CANDIDATES");
  const char *ga_results = getenv("GA_RESULTS");
  int eval_mode = ga_candidates && ga_candidates[0];
  // GA_FULL_RESET=0 reproduces the sequential conditions of the original dataset run
  const char *ga_full_reset = getenv("GA_FULL_RESET");
  int full_reset = eval_mode && !(ga_full_reset && strcmp(ga_full_reset, "0") == 0);
  Candidate *candidates = NULL;
  int num_episodes = NUM_EPISODES;
  if (eval_mode) {
    num_episodes = load_candidates(ga_candidates, &candidates);
    if (num_episodes < 0 || !ga_results || !ga_results[0]) {
      printf("ERROR: cannot read GA_CANDIDATES '%s' or GA_RESULTS not set.\n", ga_candidates);
      wb_supervisor_simulation_quit(EXIT_FAILURE);
      wb_robot_cleanup();
      return EXIT_FAILURE;
    }
  }

  FILE *csv = fopen(eval_mode ? ga_results : "drone_pid_dataset.csv", "w");
  if (csv) {
    if (eval_mode) fprintf(csv, "candidate_id,");
    fprintf(csv, "episode,scenario,flight_duration,p_gain,I_gain,D_gain,inital_pitch,initail_roll,initail_yaw,wind_speed,payload_weight,rise_time,settling_time,overshoot_percent,steady_state_error,Total_energy_consumed,stability_score,flipped,avg_position_x,avg_position_y,avg_position_z\n");
    fflush(csv);
  }

  if (eval_mode)
    printf("GA evaluation mode: %d candidates from %s -> %s (full reset: %s)\n", num_episodes, ga_candidates,
           ga_results, full_reset ? "on" : "off");
  else
    printf("Starting automated dataset logging for %d stable random episodes (>90%% stability target)...\n", NUM_EPISODES);

  int running = 1;
  for (int ep = 0; ep < num_episodes && running; ep++) {
    double P, I, D;
    if (eval_mode) {
      P = candidates[ep].P;
      I = candidates[ep].I;
      D = candidates[ep].D;
    } else {
      // Tighter, highly stable random PID ranges
      P = 1.2 + ((double)rand() / RAND_MAX) * 1.6;  // Range: 1.2 to 2.8
      I = 0.001 + ((double)rand() / RAND_MAX) * 0.009; // Range: 0.001 to 0.01
      D = 0.5 + ((double)rand() / RAND_MAX) * 0.7;   // Range: 0.5 to 1.2
    }

    if (full_reset) {
      // Full world reset so every candidate starts from the identical state of the .wbt file
      // (reset_drone alone keeps gimbal/motor state from the previous episode).
      wb_supervisor_simulation_reset();
      if (wb_robot_step(timestep) == -1) break;
      for (int m = 0; m < 4; ++m) {
        wb_motor_set_position(motors[m], INFINITY);
        wb_motor_set_velocity(motors[m], 0.0);
      }
    }

    if (!reset_drone(self_node, trans_field, rot_field, pos0, rot0, motors, timestep)) break;

    const double *p0 = wb_gps_get_values(gps);
    double prev_x = p0[0], prev_y = p0[1], prev_z = p0[2];
    double vx = 0, vy = 0, vz = 0;
    double integral = 0.0, total_energy = 0.0, max_alt = 0.0;
    double rise_time = 0.0, settling_time = 0.0, sum_err_sq = 0.0;
    int reached_rise = 0, err_steps = 0;
    double settle_start = -1.0;  // time of the last entry into the settling band, -1 while outside
    double sum_abs_err_end = 0.0;
    int sse_steps = 0;
    double t = 0.0;
    int landed = 0, flipped = 0;
    double landed_time = 0.0, max_tilt = 0.0;

    double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
    int total_steps = 0;

    while (t < MAX_DURATION) {
      if (wb_robot_step(timestep) == -1) { running = 0; break; }
      t += dt;
      total_steps++;

      const double *rpy = wb_inertial_unit_get_roll_pitch_yaw(imu);
      double roll = rpy[0], pitch = rpy[1], yaw = rpy[2];
      const double *pos = wb_gps_get_values(gps);
      double x = pos[0], y = pos[1], z = pos[2];
      const double *g = wb_gyro_get_values(gyro);
      double roll_v = g[0], pitch_v = g[1];

      sum_x += x; sum_y += y; sum_z += z;

      double tilt = fmax(fabs(roll), fabs(pitch));
      if (tilt > max_tilt) max_tilt = tilt;
      if (tilt > 1.2) { flipped = 1; break; }

      vx = 0.7 * vx + 0.3 * ((x - prev_x) / dt);
      vy = 0.7 * vy + 0.3 * ((y - prev_y) / dt);
      vz = 0.7 * vz + 0.3 * ((z - prev_z) / dt);
      prev_x = x; prev_y = y; prev_z = z;

      // GPS velocity is in the world frame; the velocity loop works in the body frame (x forward, y left)
      double v_fwd = cos(yaw) * vx + sin(yaw) * vy;
      double v_left = -sin(yaw) * vx + cos(yaw) * vy;

      double target = fmin(HOVER_ALT, t * 0.6);
      double vx_des = 0.0;

      if (t >= FORWARD_START && t < LAND_START) {
        target = HOVER_ALT;
        vx_des = FORWARD_SPEED;
      } else if (t >= LAND_START) {
        target = LAND_ALT;
        if (!landed && z < TOUCHDOWN_ALT) { landed = 1; landed_time = t; }
      }

      if (t < LAND_START) {
        double alt_err = HOVER_ALT - z;
        if (z > max_alt) max_alt = z;
        if (!reached_rise && z >= 0.9 * HOVER_ALT) { rise_time = t; reached_rise = 1; }
        // settled = inside the band from settle_start until the end of the hover phase
        if (fabs(alt_err) > SETTLE_BAND) settle_start = -1.0;
        else if (settle_start < 0.0) settle_start = t;
        if (t >= LAND_START - SSE_WINDOW) { sum_abs_err_end += fabs(alt_err); sse_steps++; }
      }

      wb_motor_set_position(camera_roll_motor, -0.115 * roll_v);
      wb_motor_set_position(camera_pitch_motor, -0.1 * pitch_v);

      double c_fl = 0, c_fr = 0, c_rl = 0, c_rr = 0;

      if (!landed) {
        double error = target - z;
        integral = CLAMP(integral + error * dt, -2.5, 2.5);
        sum_err_sq += error * error;
        err_steps++;
       
        double tilt_comp = 68.5 * (1.0 / sqrt(CLAMP(cos(roll) * cos(pitch), 0.5, 1.0)) - 1.0);
        double vertical_input = P * error + I * integral - D * vz + HOVER_FF + tilt_comp;
        vertical_input = CLAMP(vertical_input, -5.0, 5.0);

        double pitch_dist = CLAMP(-VEL_KD * (vx_des - v_fwd), -1.0, 1.0);
        double roll_dist = CLAMP(VEL_KD * (0.0 - v_left), -1.0, 1.0);

        const double roll_input = 35.0 * CLAMP(roll, -0.6, 0.6) + roll_v + roll_dist;
        const double pitch_input = 20.0 * CLAMP(pitch, -0.6, 0.6) + pitch_v + pitch_dist;

        c_fl = 68.5 + vertical_input + pitch_input - roll_input;
        c_fr = 68.5 + vertical_input + pitch_input + roll_input;
        c_rl = 68.5 + vertical_input - pitch_input - roll_input;
        c_rr = 68.5 + vertical_input - pitch_input + roll_input;
        c_fl = CLAMP(c_fl, 0.0, 130.0);
        c_fr = CLAMP(c_fr, 0.0, 130.0);
        c_rl = CLAMP(c_rl, 0.0, 130.0);
        c_rr = CLAMP(c_rr, 0.0, 130.0);
      }

      total_energy += (c_fl + c_fr + c_rl + c_rr) * dt;
      wb_motor_set_velocity(fl_m, c_fl);
      wb_motor_set_velocity(fr_m, -c_fr);
      wb_motor_set_velocity(rl_m, -c_rl);
      wb_motor_set_velocity(rr_m, c_rr);

      if (landed && t - landed_time > 0.5) break;
    }

    for (int m = 0; m < 4; ++m) wb_motor_set_velocity(motors[m], 0.0);

    // Not reaching / not settling at HOVER_ALT is the worst case, not 0 s
    if (!reached_rise) rise_time = NOT_REACHED_TIME;
    settling_time = (settle_start >= 0.0 && t >= LAND_START) ? settle_start : NOT_REACHED_TIME;
    double overshoot = ((max_alt - HOVER_ALT) / HOVER_ALT) * 100.0;
    if (overshoot < 0) overshoot = 0;
    double sse = sse_steps ? sum_abs_err_end / sse_steps : HOVER_ALT;
    double stability = 1.0 / (1.0 + (err_steps ? sum_err_sq / err_steps : 0.0));
    double avg_x = sum_x / total_steps;
    double avg_y = sum_y / total_steps;
    double avg_z = sum_z / total_steps;

    if (csv) {
      if (eval_mode) fprintf(csv, "%s,", candidates[ep].id);
      fprintf(csv, "%d,%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.2f,%.3f,%.4f,%.4f,%.2f,%.4f,%.4f,%.4f,%d,%.4f,%.4f,%.4f\n",
            ep + 1, 1, t, P, I, D, INIT_PITCH, INIT_ROLL, INIT_YAW, WIND_SPEED, PAYLOAD_WEIGHT,
            rise_time, settling_time, overshoot, sse, total_energy, stability, flipped, avg_x, avg_y, avg_z);
      fflush(csv);
    }

    printf("Episode %d/%d (P=%.2f I=%.3f D=%.2f): %s\n",
           ep + 1, num_episodes, P, I, D, flipped ? "FLIPPED" : "SUCCESS");
    fflush(stdout);
  }

  if (csv) fclose(csv);
  if (eval_mode) {
    printf("GA evaluation finished: %d candidates.\n", num_episodes);
    fflush(stdout);
    free(candidates);
    wb_supervisor_simulation_quit(EXIT_SUCCESS);
    wb_robot_step(timestep);  // let Webots process the quit request
  } else {
    printf("All %d episodes completed! Dataset saved successfully with 'flipped' status.\n", NUM_EPISODES);
    wb_supervisor_simulation_set_mode(WB_SUPERVISOR_SIMULATION_MODE_PAUSE);
  }
  wb_robot_cleanup();
  return EXIT_SUCCESS;
}