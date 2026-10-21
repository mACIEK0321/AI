#include <math.h>
#include <stdio.h>
#include <stdlib.h>
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

  FILE *csv = fopen("drone_pid_dataset.csv", "w");
  if (csv) {
    fprintf(csv, "episode,scenario,flight_duration,p_gain,I_gain,D_gain,inital_pitch,initail_roll,initail_yaw,wind_speed,payload_weight,rise_time,settling_time,overshoot_percent,steady_state_error,Total_energy_consumed,stability_score,flipped,avg_position_x,avg_position_y,avg_position_z\n");
  }

  printf("Starting automated dataset logging for %d stable random episodes (>90%% stability target)...\n", NUM_EPISODES);

  int running = 1;
  for (int ep = 0; ep < NUM_EPISODES && running; ep++) {
    // Tighter, highly stable random PID ranges
    double P = 1.2 + ((double)rand() / RAND_MAX) * 1.6;  // Range: 1.2 to 2.8
    double I = 0.001 + ((double)rand() / RAND_MAX) * 0.009; // Range: 0.001 to 0.01
    double D = 0.5 + ((double)rand() / RAND_MAX) * 0.7;   // Range: 0.5 to 1.2

    if (!reset_drone(self_node, trans_field, rot_field, pos0, rot0, motors, timestep)) break;

    const double *p0 = wb_gps_get_values(gps);
    double prev_x = p0[0], prev_y = p0[1], prev_z = p0[2];
    double vx = 0, vy = 0, vz = 0;
    double integral = 0.0, total_energy = 0.0, max_alt = 0.0;
    double rise_time = 0.0, settling_time = 0.0, sum_err_sq = 0.0;
    int reached_rise = 0, reached_settle = 0;
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
      double roll = rpy[0], pitch = rpy[1];
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

      double target = fmin(HOVER_ALT, t * 0.6);
      double vx_des = 0.0;
     
      if (t >= 3.0 && t < 6.0) {
        target = HOVER_ALT;
        vx_des = FORWARD_SPEED;
      } else if (t >= 6.0) {
        target = LAND_ALT;
        if (!landed && z < TOUCHDOWN_ALT) { landed = 1; landed_time = t; }
      }

      if (z > max_alt) max_alt = z;
      if (!reached_rise && z >= 0.9 * HOVER_ALT) { rise_time = t; reached_rise = 1; }
      if (!reached_settle && fabs(HOVER_ALT - z) <= 0.05 && t > 1.0 && t < 5.0) { settling_time = t; reached_settle = 1; }

      wb_motor_set_position(camera_roll_motor, -0.115 * roll_v);
      wb_motor_set_position(camera_pitch_motor, -0.1 * pitch_v);

      double c_fl = 0, c_fr = 0, c_rl = 0, c_rr = 0;

      if (!landed) {
        double error = target - z;
        integral = CLAMP(integral + error * dt, -2.5, 2.5);
        sum_err_sq += error * error;
       
        double tilt_comp = 68.5 * (1.0 / sqrt(CLAMP(cos(roll) * cos(pitch), 0.5, 1.0)) - 1.0);
        double vertical_input = P * error + I * integral - D * vz + HOVER_FF + tilt_comp;
        vertical_input = CLAMP(vertical_input, -5.0, 5.0);

        double pitch_dist = CLAMP(-VEL_KD * (vx_des - vx), -1.0, 1.0);
        double roll_dist = CLAMP(VEL_KD * (0.0 - vy), -1.0, 1.0);

        const double roll_input = 35.0 * CLAMP(roll, -0.6, 0.6) + roll_v + roll_dist;
        const double pitch_input = 20.0 * CLAMP(pitch, -0.6, 0.6) + pitch_v + pitch_dist;

        c_fl = 68.5 + vertical_input + pitch_input - roll_input;
        c_fr = 68.5 + vertical_input + pitch_input + roll_input;
        c_rl = 68.5 + vertical_input - pitch_input - roll_input;
        c_rr = 68.5 + vertical_input + pitch_input + roll_input;
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

    double overshoot = ((max_alt - HOVER_ALT) / HOVER_ALT) * 100.0;
    if (overshoot < 0) overshoot = 0;
    double sse = fabs(HOVER_ALT - (sum_z / total_steps));
    double stability = 1.0 / (1.0 + (sum_err_sq / total_steps));
    double avg_x = sum_x / total_steps;
    double avg_y = sum_y / total_steps;
    double avg_z = sum_z / total_steps;

    if (csv) {
      fprintf(csv, "%d,%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.2f,%.3f,%.4f,%.4f,%.2f,%.4f,%.4f,%.4f,%d,%.4f,%.4f,%.4f\n",
            ep + 1, 1, t, P, I, D, INIT_PITCH, INIT_ROLL, INIT_YAW, WIND_SPEED, PAYLOAD_WEIGHT,
            rise_time, settling_time, overshoot, sse, total_energy, stability, flipped, avg_x, avg_y, avg_z);
      fflush(csv);
    }

    printf("Episode %d/%d (P=%.2f I=%.3f D=%.2f): %s\n",
           ep + 1, NUM_EPISODES, P, I, D, flipped ? "FLIPPED" : "SUCCESS");
  }

  printf("All %d episodes completed! Dataset saved successfully with 'flipped' status.\n", NUM_EPISODES);
  if (csv) fclose(csv);
  wb_supervisor_simulation_set_mode(WB_SUPERVISOR_SIMULATION_MODE_PAUSE);
  wb_robot_cleanup();
  return EXIT_SUCCESS;
}