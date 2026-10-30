#include <h1_2_policy_bridge/core.hpp>
#include <mujoco/mujoco.h>
#include <openssl/evp.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>

using namespace h1_policy;
constexpr double period = .02;
const std::array<const char*, 27> names = {
    "left_hip_yaw_joint", "left_hip_pitch_joint", "left_hip_roll_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_yaw_joint", "right_hip_pitch_joint", "right_hip_roll_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "torso_joint", "left_shoulder_pitch_joint", "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint", "left_elbow_joint", "left_wrist_pitch_joint",
    "left_wrist_roll_joint", "left_wrist_yaw_joint", "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint", "right_shoulder_yaw_joint", "right_elbow_joint",
    "right_wrist_pitch_joint", "right_wrist_roll_joint", "right_wrist_yaw_joint"};
const std::array<float, 27> limits = {
    200,200,200,300,40,40,200,200,200,300,40,40,200,
    40,40,18,18,19,19,19,40,40,18,18,19,19,19};

struct Options {
    std::string prep = "ideal_fixed_base", output;
    std::string model = "/home/llx/policy.onnx";
    std::string xml = "/home/llx/unitree_rl_mjlab/src/assets/robots/unitree_h1_2/xmls/scene_h1_2.xml";
    double duration = 30, vx = 0, wz = 0;
};
Options parse(int argc, char** argv) {
    Options o;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (i + 1 >= argc) throw std::runtime_error("missing argument value");
        std::string v = argv[++i];
        if (a == "--output") o.output = v;
        else if (a == "--duration") o.duration = std::stod(v);
        else if (a == "--vx") o.vx = std::stod(v);
        else if (a == "--wz") o.wz = std::stod(v);
        else if (a == "--prep") o.prep = v;
        else if (a == "--model") o.model = v;
        else if (a == "--xml") o.xml = v;
        else throw std::runtime_error("unknown argument: " + a);
    }
    if ((o.prep != "ideal_fixed_base" && o.prep != "upward_force") || o.output.empty() ||
        !std::isfinite(o.duration) || !std::isfinite(o.vx) || !std::isfinite(o.wz) ||
        o.duration <= 0 || o.duration > 120 || std::abs(o.vx) > .2 || std::abs(o.wz) > .5)
        throw std::runtime_error("invalid output, preparation, duration or velocity");
    return o;
}
std::string hash(const std::string& p) {
    std::ifstream f(p, std::ios::binary);
    if (!f) throw std::runtime_error("cannot open policy");
    EVP_MD_CTX* c = EVP_MD_CTX_new();
    if (!c || !EVP_DigestInit_ex(c, EVP_sha256(), nullptr)) throw std::runtime_error("hash init failed");
    char b[8192];
    while (f) { f.read(b, sizeof b); EVP_DigestUpdate(c, b, f.gcount()); }
    unsigned char d[EVP_MAX_MD_SIZE]; unsigned int n = 0;
    EVP_DigestFinal_ex(c, d, &n); EVP_MD_CTX_free(c);
    std::ostringstream s;
    for (unsigned i = 0; i < n; ++i) s << std::hex << std::setw(2) << std::setfill('0') << int(d[i]);
    return s.str();
}
int id(const mjModel* m, int type, const char* name) {
    int value = mj_name2id(m, type, name);
    if (value < 0) throw std::runtime_error(std::string("missing model object: ") + name);
    return value;
}
struct Stats {
    int samples = 0, saturation = 0;
    double min_height = std::numeric_limits<double>::infinity(), max_height = -min_height;
    double max_roll = 0, max_pitch = 0, sum_vx = 0, sum_vy = 0, sum_wz = 0;
    double left_x = 0, left_y = 0, right_x = 0, right_y = 0, left_drift = 0, right_drift = 0;
    void add(double h, double roll, double pitch, double vx, double vy, double wz,
             int sat, double lx, double ly, double rx, double ry) {
        if (!samples) { left_x=lx; left_y=ly; right_x=rx; right_y=ry; }
        ++samples; saturation += sat; min_height=std::min(min_height,h); max_height=std::max(max_height,h);
        max_roll=std::max(max_roll,std::abs(roll)); max_pitch=std::max(max_pitch,std::abs(pitch));
        sum_vx+=vx; sum_vy+=vy; sum_wz+=wz;
        left_drift=std::max(left_drift,std::hypot(lx-left_x,ly-left_y));
        right_drift=std::max(right_drift,std::hypot(rx-right_x,ry-right_y));
    }
};
int main(int argc, char** argv) {
 try {
    Options o = parse(argc, argv);
    char error[1024]{};
    mjModel* m = mj_loadXML(o.xml.c_str(), nullptr, error, sizeof error);
    if (!m) throw std::runtime_error(error);
    mjData* d = mj_makeData(m);
    int steps = std::lround(period / m->opt.timestep);
    if (steps < 1 || std::abs(steps*m->opt.timestep-period)>1e-9)
        throw std::runtime_error("physics timestep must divide 0.02 seconds");
    std::array<int,27> joints{}, actuators{};
    for (int i=0; i<27; ++i) {
        joints[i]=id(m,mjOBJ_JOINT,names[i]); actuators[i]=id(m,mjOBJ_ACTUATOR,names[i]);
        m->jnt_actfrclimited[joints[i]]=1;
        m->jnt_actfrcrange[2*joints[i]]=-limits[i]; m->jnt_actfrcrange[2*joints[i]+1]=limits[i];
        m->actuator_ctrllimited[actuators[i]]=1;
        m->actuator_ctrlrange[2*actuators[i]]=-limits[i]; m->actuator_ctrlrange[2*actuators[i]+1]=limits[i];
    }
    int pelvis=id(m,mjOBJ_BODY,"pelvis"), imu=id(m,mjOBJ_SITE,"imu"), gyro=id(m,mjOBJ_SENSOR,"imu_gyro");
    int left=id(m,mjOBJ_BODY,"left_ankle_roll_link"), right=id(m,mjOBJ_BODY,"right_ankle_roll_link");
    auto initial=fix_stand_target();
    for (int i=0; i<27; ++i) d->qpos[m->jnt_qposadr[joints[i]]]=initial[i];
    d->qpos[2]=1.03; mj_forward(m,d);
    double mass=0;
    for (int i=1; i<m->nbody; ++i) mass+=m->body_mass[i];
    double support=.6*mass*std::abs(m->opt.gravity[2]);
    Controller controller(o.model); State state;
    auto read = [&]() {
        for (int i=0; i<27; ++i) { state.q[i]=d->qpos[m->jnt_qposadr[joints[i]]]; state.dq[i]=d->qvel[m->jnt_dofadr[joints[i]]]; }
        for (int i=0; i<3; ++i) { state.gyro[i]=d->sensordata[m->sensor_adr[gyro]+i]; state.gravity[i]=-d->site_xmat[9*imu+6+i]; }
    };
    read(); controller.transition(Mode::FixStand,state,0);
    std::filesystem::create_directories(o.output);
    std::ofstream csv(o.output+"/trace.csv");
    if (!csv) throw std::runtime_error("cannot create trace");
    csv << "stage,time,height,roll,pitch,vx,vy,wz,body_vx,body_vy,body_wz,saturated,fall,base_constrained,support_newtons,contact_count,left_knee,right_knee,left_foot_x,left_foot_y,right_foot_x,right_foot_y\n";
    bool fell=false, accepted=false; double takeover=-1, final_height=0, last_time=0;
    int total_saturation=0; Stats stats;
    for (int k=0; k<=std::ceil((15+o.duration)/period); ++k) {
        double t=k*period; last_time=t; read();
        if (k==150) controller.transition(Mode::ArmHoldStand,state,t);
        if (k>=300 && !accepted) { accepted=controller.transition(Mode::ArmHold,state,t); if (accepted) takeover=t; }
        Mode mode=controller.mode();
        const char* stage=mode==Mode::FixStand?"FixStand":mode==Mode::ArmHoldStand?"ArmHoldStand":mode==Mode::ArmHold?"ArmHold":"other";
        Command cmd{}; if (accepted) { cmd[0]=o.vx; cmd[2]=o.wz; }
        auto torque=controller.step(state,cmd,t); int sat=0;
        for (int i=0; i<27; ++i) { d->ctrl[actuators[i]]=torque[i]; if (std::abs(torque[i])>=limits[i]-.001) ++sat; }
        total_saturation+=sat;
        const double* q=d->qpos+3;
        double roll=std::atan2(2*(q[0]*q[1]+q[2]*q[3]),1-2*(q[1]*q[1]+q[2]*q[2]));
        double pitch=std::asin(std::clamp(2*(q[0]*q[2]-q[3]*q[1]),-1.,1.));
        final_height=d->qpos[2]; fell=final_height<.6||std::abs(roll)>1||std::abs(pitch)>1;
        double body_vel[3];
        double inv[4]; mju_negQuat(inv,d->qpos+3); mju_rotVecQuat(body_vel,d->qvel,inv);
        double body_wz=state.gyro[2];
        double lx=d->xpos[3*left],ly=d->xpos[3*left+1],rx=d->xpos[3*right],ry=d->xpos[3*right+1];
        if (accepted) stats.add(final_height,roll,pitch,body_vel[0],body_vel[1],body_wz,sat,lx,ly,rx,ry);
        double applied=accepted||o.prep=="ideal_fixed_base"?0:support;
        csv<<stage<<','<<t<<','<<final_height<<','<<roll<<','<<pitch<<','<<d->qvel[0]<<','<<d->qvel[1]<<','<<d->qvel[5]<<','<<body_vel[0]<<','<<body_vel[1]<<','<<body_wz<<','<<sat<<','<<fell<<','<<(!accepted&&o.prep=="ideal_fixed_base")<<','<<applied<<','<<d->ncon<<','<<state.q[3]<<','<<state.q[9]<<','<<lx<<','<<ly<<','<<rx<<','<<ry<<'\n';
        if (fell || (accepted && t>=takeover+o.duration) || (!accepted && t>=15)) break;
        d->xfrc_applied[6*pelvis+2]=applied;
        for (int j=0; j<steps; ++j) {
            mj_step(m,d);
            if (!accepted && o.prep=="ideal_fixed_base") {
                double f=std::clamp((d->time-3.0)/3.0,0.0,1.0);
                d->qpos[0]=d->qpos[1]=0; d->qpos[2]=1.03-.08*f;
                d->qpos[3]=1; d->qpos[4]=d->qpos[5]=d->qpos[6]=0;
                for (int z=0; z<6; ++z) d->qvel[z]=0;
                mj_forward(m,d);
            }
        }
    }
    double active_duration=accepted?last_time-takeover:0;
    bool completed=accepted&&!fell&&active_duration+1e-9>=o.duration;
    bool height_ok=stats.samples && stats.min_height>=.92 && stats.max_height<=.98;
    std::ofstream summary(o.output+"/summary.json");
    if (!summary) throw std::runtime_error("cannot create summary");
    summary<<std::boolalpha<<std::setprecision(10)
      <<"{\n  \"evaluation_status\": \""<<(completed?"completed_without_fall":accepted?"active_incomplete":"preparation_failed")<<"\",\n"
      <<"  \"completed_without_fall\": "<<completed<<",\n  \"physics_dt\": "<<m->opt.timestep<<",\n  \"control_period\": 0.02,\n"
      <<"  \"dds_enabled\": false,\n  \"base_fixed_during_preparation\": "<<(o.prep=="ideal_fixed_base")<<",\n  \"base_fixed_after_takeover\": false,\n"
      <<"  \"model_sha256\": \""<<hash(o.model)<<"\",\n  \"preparation\": \""<<o.prep<<"\",\n"
      <<"  \"nominal_upward_force_newtons\": "<<support<<",\n  \"preparation_applied_upward_force_newtons\": "<<(o.prep=="upward_force"?support:0)<<",\n  \"active_applied_upward_force_newtons\": 0,\n"
      <<"  \"takeover_accepted\": "<<accepted<<",\n  \"takeover_time\": "<<takeover<<",\n  \"active_duration\": "<<active_duration<<",\n  \"fall\": "<<fell<<",\n"
      <<"  \"final_time\": "<<last_time<<",\n  \"command_vx\": "<<o.vx<<",\n  \"command_wz\": "<<o.wz<<",\n  \"final_height\": "<<final_height<<",\n"
      <<"  \"active_min_height\": "<<(stats.samples?stats.min_height:0)<<",\n  \"active_max_height\": "<<(stats.samples?stats.max_height:0)<<",\n"
      <<"  \"active_height_within_095_plus_minus_003\": "<<height_ok<<",\n  \"active_max_abs_roll\": "<<stats.max_roll<<",\n  \"active_max_abs_pitch\": "<<stats.max_pitch<<",\n"
      <<"  \"active_mean_body_vx\": "<<(stats.samples?stats.sum_vx/stats.samples:0)<<",\n  \"active_mean_body_vy\": "<<(stats.samples?stats.sum_vy/stats.samples:0)<<",\n  \"active_mean_body_wz\": "<<(stats.samples?stats.sum_wz/stats.samples:0)<<",\n"
      <<"  \"active_left_ankle_max_horizontal_drift\": "<<stats.left_drift<<",\n  \"active_right_ankle_max_horizontal_drift\": "<<stats.right_drift<<",\n"
      <<"  \"active_saturation_joint_samples\": "<<stats.saturation<<",\n  \"total_saturation_joint_samples\": "<<total_saturation<<"\n}\n";
    mj_deleteData(d); mj_deleteModel(m); return 0;
 } catch (const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
}
