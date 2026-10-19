// Native MuJoCo model inspection: no ROS graph or motor driver.
// Usage: model_probe SCENE KEYFRAME SECONDS [hold|release|open|close|roll|slide|bowl_roll] [OUTPUT.ppm]
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

static void array(const mjtNum* values, int n) {
  std::cout << '[';
  for (int i=0; i<n; ++i) { if (i) std::cout << ','; std::cout << values[i]; }
  std::cout << ']';
}

static bool render(const mjModel* m, mjData* d, const char* path) {
  if (!glfwInit()) return false;
  glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
  GLFWwindow* window = glfwCreateWindow(960, 720, "DM model preview", nullptr, nullptr);
  if (!window) { glfwTerminate(); return false; }
  glfwMakeContextCurrent(window);
  mjvScene scene; mjv_defaultScene(&scene); mjv_makeScene(m, &scene, 2000);
  mjrContext context; mjr_defaultContext(&context); mjr_makeContext(m, &context, mjFONTSCALE_100);
  mjvCamera camera; mjv_defaultFreeCamera(m, &camera);
  mjvOption option; mjv_defaultOption(&option);
  option.geomgroup[3] = 0;  // collision geoms remain active in physics, hidden in preview
  mjv_updateScene(m, d, &option, nullptr, &camera, mjCAT_ALL, &scene);
  mjrRect viewport = {0,0,960,720};
  mjr_render(viewport, &scene, &context);
  std::vector<unsigned char> pixels(960*720*3);
  mjr_readPixels(pixels.data(), nullptr, viewport, &context);
  std::ofstream out(path, std::ios::binary);
  out << "P6\n960 720\n255\n";
  for (int row=719; row>=0; --row) out.write(reinterpret_cast<char*>(pixels.data()+row*960*3), 960*3);
  mjr_freeContext(&context); mjv_freeScene(&scene);
  glfwDestroyWindow(window); glfwTerminate();
  return bool(out);
}

int main(int argc, char** argv) {
  if (argc < 4) { std::cerr << "Usage: model_probe SCENE KEYFRAME SECONDS [hold|release|open|close|roll|slide|bowl_roll] [OUTPUT.ppm]\n"; return 2; }
  char error[2048] = {};
  mjModel* m = mj_loadXML(argv[1], nullptr, error, sizeof(error));
  if (!m) { std::cerr << error << '\n'; return 1; }
  int key = mj_name2id(m, mjOBJ_KEY, argv[2]);
  if (key < 0) { std::cerr << "Unknown keyframe\n"; mj_deleteModel(m); return 1; }
  mjData* d = mj_makeData(m);
  mj_resetDataKeyframe(m, d, key);
  const double seconds = std::stod(argv[3]);
  const std::string mode = argc > 4 ? argv[4] : "hold";
  const bool ball_test = mode == "roll" || mode == "slide" || mode == "bowl_roll";
  if (mode != "hold" && mode != "release" && mode != "open" && mode != "close" && !ball_test) return 2;
  if (mode == "release") m->opt.disableflags |= mjDSBL_ACTUATION;
  const int gripper = mj_name2id(m, mjOBJ_ACTUATOR, "finger_left");
  const double initial_gripper = d->ctrl[gripper];
  const int left = m->jnt_qposadr[mj_name2id(m, mjOBJ_JOINT, "finger_left")];
  const int right = m->jnt_qposadr[mj_name2id(m, mjOBJ_JOINT, "finger_right")];
  double max_velocity = 0, max_mimic_error = 0, max_limit_error = 0;
  double max_force_excess = 0, max_penetration = 0;
  int ball_q = -1, ball_v = -1;
  double ball_start_x = 0, ball_radius = 0, low_speed_since = -1, stop_time = -1;
  if (ball_test) {
    const int joint = mj_name2id(m, mjOBJ_JOINT, "loose_ball_1_joint");
    const int geom = mj_name2id(m, mjOBJ_GEOM, "loose_ball_1_geom");
    if (joint < 0 || geom < 0) { std::cerr << "Ball test requires loose_ball_1\n"; return 2; }
    ball_q = m->jnt_qposadr[joint]; ball_v = m->jnt_dofadr[joint];
    ball_radius = m->geom_size[3*geom];
    // Table tests settle first; bowl test starts at the measured release height.
    if (mode != "bowl_roll")
      for (int i=0; i<std::lround(2/m->opt.timestep); ++i) mj_step(m,d);
    d->time = 0;
    for (int i=0;i<6;++i) d->qvel[ball_v+i] = 0;
    if (mode == "bowl_roll") {
      d->qpos[ball_q] = 0.30;
      d->qpos[ball_q+1] = -0.14;
      d->qpos[ball_q+2] = 0.1057;
      d->qvel[ball_v+1] = 0.15;
      d->qvel[ball_v+3] = -0.15/ball_radius;
    }
    d->qpos[ball_q+3] = 1;
    for (int i=4;i<7;++i) d->qpos[ball_q+i] = 0;
    if (mode != "bowl_roll") d->qvel[ball_v] = 0.1;
    if (mode == "roll") d->qvel[ball_v+4] = 0.1/ball_radius;
    ball_start_x = d->qpos[ball_q];
    mj_forward(m,d);
  }
  for (int step=0; step<std::lround(seconds/m->opt.timestep); ++step) {
    if (mode == "open" || mode == "close") {
      const double fraction = std::min(d->time / 1.0, 1.0);
      d->ctrl[gripper] = initial_gripper + fraction*((mode == "open" ? 0.04 : 0.0)-initial_gripper);
    }
    mj_step(m, d);
    if (ball_test) {
      const double linear = mju_norm3(d->qvel+ball_v);
      const double angular = mju_norm3(d->qvel+ball_v+3);
      if (linear < 0.001 && angular*ball_radius < 0.001) {
        if (low_speed_since < 0) low_speed_since = d->time;
        if (d->time-low_speed_since >= 0.25) stop_time = low_speed_since;
      } else { low_speed_since = -1; stop_time = -1; }
    }
    for (int i=0;i<m->nv;++i) {
      if (!std::isfinite(d->qvel[i])) { std::cerr << "Nonfinite state\n"; return 1; }
      max_velocity=std::max(max_velocity, std::abs(d->qvel[i]));
    }
    max_mimic_error=std::max(max_mimic_error,std::abs(d->qpos[left]+d->qpos[right]));
    for (int i=0;i<m->nu;++i)
      max_force_excess=std::max({max_force_excess, m->actuator_forcerange[2*i]-d->actuator_force[i],
                               d->actuator_force[i]-m->actuator_forcerange[2*i+1]});
    for (int i=0;i<d->ncon;++i) max_penetration=std::max(max_penetration,-d->contact[i].dist);
    for (int i=0;i<m->njnt;++i) {
      if (!m->jnt_limited[i]) continue;
      const double q=d->qpos[m->jnt_qposadr[i]];
      max_limit_error=std::max({max_limit_error,m->jnt_range[2*i]-q,q-m->jnt_range[2*i+1]});
    }
  }
  mj_forward(m,d);
  if (argc > 5 && !render(m,d,argv[5])) { std::cerr << "Rendering failed\n"; return 1; }
  std::cout << std::setprecision(16) << "{\"version\":\"" << mj_versionString() << "\",\"time\":" << d->time;
  std::cout << ",\"nq\":" << m->nq << ",\"nv\":" << m->nv << ",\"nu\":" << m->nu;
  if (ball_test) {
    std::cout << ",\"ball_test\":{\"mode\":\"" << mode << "\",\"initial_speed\":" << (mode == "bowl_roll" ? 0.15 : 0.1) << ",\"x_travel\":" << d->qpos[ball_q]-ball_start_x;
    if (mode == "bowl_roll")
      std::cout << ",\"bowl_radial_distance\":" << std::hypot(d->qpos[ball_q]-0.30, d->qpos[ball_q+1]+0.14);
    std::cout << ",\"final_linear_speed\":" << mju_norm3(d->qvel+ball_v);
    std::cout << ",\"final_angular_speed\":" << mju_norm3(d->qvel+ball_v+3);
    std::cout << ",\"stop_time\":";
    if (stop_time < 0) std::cout << "null"; else std::cout << stop_time;
    std::cout << '}';
  }
  std::cout << ",\"qpos\":"; array(d->qpos,m->nq);
  std::cout << ",\"qvel\":"; array(d->qvel,m->nv);
  std::cout << ",\"ctrl\":"; array(d->ctrl,m->nu);
  std::cout << ",\"actuator_force\":"; array(d->actuator_force,m->nu);
  std::cout << ",\"max_velocity\":" << max_velocity << ",\"max_mimic_error\":" << max_mimic_error << ",\"max_limit_error\":" << max_limit_error;
  std::cout << ",\"max_force_excess\":" << max_force_excess << ",\"max_penetration\":" << max_penetration;
  std::cout << ",\"joints\":[";
  for (int i=0;i<m->njnt;++i) {
    if (i) std::cout << ',';
    std::cout << "{\"name\":\"" << mj_id2name(m,mjOBJ_JOINT,i) << "\",\"type\":" << m->jnt_type[i]
              << ",\"qposadr\":" << m->jnt_qposadr[i] << ",\"axis\":";
    array(m->jnt_axis+3*i,3); std::cout << ",\"range\":"; array(m->jnt_range+2*i,2);
    std::cout << '}';
  }
  std::cout << "],\"actuators\":[";
  for (int i=0;i<m->nu;++i) {
    if (i) std::cout << ',';
    std::cout << "{\"name\":\"" << mj_id2name(m,mjOBJ_ACTUATOR,i) << "\",\"joint\":\""
              << mj_id2name(m,mjOBJ_JOINT,m->actuator_trnid[2*i]) << "\",\"ctrlrange\":";
    array(m->actuator_ctrlrange+2*i,2); std::cout << ",\"forcerange\":"; array(m->actuator_forcerange+2*i,2);
    std::cout << '}';
  }
  std::cout << ']';
  std::cout << ",\"warnings\":";
  int warnings=0; for (int i=0;i<mjNWARNING;++i) warnings+=d->warning[i].number;
  std::cout << warnings << ",\"bodies\":{";
  for (int i=1;i<m->nbody;++i) {
    if (i>1) std::cout << ',';
    std::cout << '"' << mj_id2name(m,mjOBJ_BODY,i) << "\":{\"position\":";
    array(d->xpos+3*i,3); std::cout << ",\"rotation\":"; array(d->xmat+9*i,9);
    std::cout << ",\"mass\":" << m->body_mass[i] << ",\"inertial_position\":";
    array(m->body_ipos+3*i,3); std::cout << ",\"inertial_quaternion\":"; array(m->body_iquat+4*i,4);
    std::cout << ",\"diaginertia\":"; array(m->body_inertia+3*i,3); std::cout << '}';
  }
  int tcp=mj_name2id(m,mjOBJ_SITE,"gripper_tcp");
  std::cout << "},\"tcp_position\":"; array(d->site_xpos+3*tcp,3);
  std::cout << ",\"tcp_rotation\":"; array(d->site_xmat+9*tcp,9);
  std::cout << ",\"contacts\":[";
  for (int i=0;i<d->ncon;++i) {
    if (i) std::cout << ',';
    std::cout << "{\"geom1\":\"" << mj_id2name(m,mjOBJ_GEOM,d->contact[i].geom[0])
              << "\",\"geom2\":\"" << mj_id2name(m,mjOBJ_GEOM,d->contact[i].geom[1])
              << "\",\"distance\":" << d->contact[i].dist
              << ",\"dim\":" << d->contact[i].dim << ",\"friction\":";
    array(d->contact[i].friction,5); std::cout << '}';
  }
  std::cout << "]}\n";
  mj_deleteData(d); mj_deleteModel(m);
  return 0;
}
