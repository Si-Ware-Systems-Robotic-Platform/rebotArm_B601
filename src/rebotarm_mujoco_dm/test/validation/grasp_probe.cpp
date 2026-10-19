// Native-engine counterpart of validate_grasp_physics.py. No ROS or viewer.
#include <mujoco/mujoco.h>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>

int main(int argc, char** argv) {
  if (argc != 2) return 2;
  char error[2048] = {};
  mjModel* m = mj_loadXML(argv[1], nullptr, error, sizeof(error));
  if (!m) { std::cerr << error << '\n'; return 2; }
  mjData* d = mj_makeData(m);
  auto joint = [&](const char* n) { return m->jnt_qposadr[mj_name2id(m,mjOBJ_JOINT,n)]; };
  int left=joint("finger_left"), right=joint("finger_right");
  int ball=mj_name2id(m,mjOBJ_BODY,"loose_ball_1");
  int lg=mj_name2id(m,mjOBJ_GEOM,"finger_left_link_collision_0_3");
  int rg=mj_name2id(m,mjOBJ_GEOM,"finger_right_link_collision_0_3");
  int bg=mj_name2id(m,mjOBJ_GEOM,"loose_ball_1_geom");
  d->qpos[left]=.04; d->qpos[right]=-.04;
  double force=0, depth=0, minheight=1, first[3]={}, last[3]={};
  int samples=0, both=0;
  for (int i=0;i<std::lround(16/m->opt.timestep);++i) {
    double t=i*m->opt.timestep;
    double a=std::clamp((t-6)/3,0.,1.);
    d->ctrl[1]=.10*(3*a*a-2*a*a*a);
    if (t<1) d->ctrl[0]=.04;
    else if (t<4) { a=(t-1)/3; d->ctrl[0]=.04+(.018-.04)*(3*a*a-2*a*a*a); }
    else if (t<12) d->ctrl[0]=.018;
    else { a=std::clamp((t-12)/2,0.,1.); d->ctrl[0]=.018+(.04-.018)*(3*a*a-2*a*a*a); }
    mj_step(m,d);
    bool lc=false, rc=false;
    for (int k=0;k<d->ncon;++k) {
      const auto& c=d->contact[k];
      if (c.geom1!=bg && c.geom2!=bg) continue;
      int g=c.geom1==bg?c.geom2:c.geom1;
      if (g==lg || g==rg) depth=std::max(depth,-c.dist);
      lc |= g==lg; rc |= g==rg;
    }
    force=std::max(force,std::abs(d->actuator_force[0]));
    if (t>=10 && t<12) {
      if (!samples) std::copy(d->xpos+3*ball,d->xpos+3*ball+3,first);
      std::copy(d->xpos+3*ball,d->xpos+3*ball+3,last);
      minheight=std::min(minheight,last[2]); ++samples; both+=lc&&rc;
    }
  }
  double drift=0; for(int i=0;i<3;++i) drift+=(last[i]-first[i])*(last[i]-first[i]);
  drift=std::sqrt(drift);
  int warnings=0; for(int i=0;i<mjNWARNING;++i) warnings+=d->warning[i].number;
  double released=d->xpos[3*ball+2];
  bool pass=minheight>.09 && drift<.003 && double(both)/samples>.95 && warnings==0 && depth<.002 && std::abs(released-.02135)<.002;
  std::cout << "{\"mujoco\":\"" << mj_versionString() << "\",\"grasp_pass\":" << (pass?"true":"false")
            << ",\"peak_actuator_force_N\":" << force << ",\"peak_penetration_m\":" << depth
            << ",\"held_height_m\":" << minheight << ",\"hold_drift_m\":" << drift
            << ",\"bilateral_contact_fraction\":" << double(both)/samples
            << ",\"released_height_m\":" << released << ",\"warnings\":" << warnings << "}\n";
  mj_deleteData(d); mj_deleteModel(m); return pass?0:1;
}
