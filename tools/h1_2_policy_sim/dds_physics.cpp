#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/dds_wrapper/common/crc.h>
#include <mujoco/mujoco.h>
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

using LS=unitree_hg::msg::dds_::LowState_;
using LC=unitree_hg::msg::dds_::LowCmd_;
using Clock=std::chrono::steady_clock;
using namespace std::chrono_literals;
const std::array<const char*,27> names={"left_hip_yaw_joint","left_hip_pitch_joint","left_hip_roll_joint","left_knee_joint","left_ankle_pitch_joint","left_ankle_roll_joint","right_hip_yaw_joint","right_hip_pitch_joint","right_hip_roll_joint","right_knee_joint","right_ankle_pitch_joint","right_ankle_roll_joint","torso_joint","left_shoulder_pitch_joint","left_shoulder_roll_joint","left_shoulder_yaw_joint","left_elbow_joint","left_wrist_roll_joint","left_wrist_pitch_joint","left_wrist_yaw_joint","right_shoulder_pitch_joint","right_shoulder_roll_joint","right_shoulder_yaw_joint","right_elbow_joint","right_wrist_roll_joint","right_wrist_pitch_joint","right_wrist_yaw_joint"};
const std::array<float,27> limits={200,200,200,300,40,40,200,200,200,300,40,40,200,40,40,18,18,19,19,19,40,40,18,18,19,19,19};
int find(const mjModel* m,int type,const char* name){int x=mj_name2id(m,type,name);if(x<0)throw std::runtime_error(std::string("missing ")+name);return x;}
int main(int argc,char**argv){
 try{
  if(argc!=3)throw std::runtime_error("usage: h1_2_dds_physics STAGE_FILE TRACE_CSV");
  char error[1024]{};
  auto m=mj_loadXML("/home/llx/unitree_rl_mjlab/src/assets/robots/unitree_h1_2/xmls/scene_h1_2.xml",nullptr,error,sizeof error);
  if(!m)throw std::runtime_error(error);
  if(std::abs(m->opt.timestep-.002)>1e-9)throw std::runtime_error("expected 2ms physical timestep");
  auto d=mj_makeData(m);std::array<int,27> j{},a{};
  for(int i=0;i<27;++i){
   j[i]=find(m,mjOBJ_JOINT,names[i]);a[i]=find(m,mjOBJ_ACTUATOR,names[i]);
   m->jnt_actfrclimited[j[i]]=1;m->jnt_actfrcrange[2*j[i]]=-limits[i];m->jnt_actfrcrange[2*j[i]+1]=limits[i];
   m->actuator_ctrllimited[a[i]]=1;m->actuator_ctrlrange[2*a[i]]=-limits[i];m->actuator_ctrlrange[2*a[i]+1]=limits[i];
  }
  const std::array<double,27> initial={0,-.2,0,.5,-.3,0,0,-.2,0,.5,-.3,0,0,.28,0,0,.52,0,0,0,.28,0,0,.52,0,0,0};
  for(int i=0;i<27;++i)d->qpos[m->jnt_qposadr[j[i]]]=initial[i];d->qpos[2]=1.03;mj_forward(m,d);
  int imu=find(m,mjOBJ_SITE,"imu"),gyro=find(m,mjOBJ_SENSOR,"imu_gyro");
  int feet[2]={find(m,mjOBJ_BODY,"left_ankle_roll_link"),find(m,mjOBJ_BODY,"right_ankle_roll_link")};
  int floor=find(m,mjOBJ_GEOM,"floor");
  unitree::robot::ChannelFactory::Instance()->Init(42,"lo");
  unitree::robot::ChannelPublisher<LS> pub("rt/lowstate");pub.InitChannel();
  std::mutex mutex; LC latest{}; Clock::time_point cmd_time{};
  std::atomic<int> count{0}, bad_crc{0}, bad_command{0};
  unitree::robot::ChannelSubscriber<LC> sub("rt/lowcmd");
  sub.InitChannel([&](const void* p){
   auto cmd=*static_cast<const LC*>(p);
   if(cmd.crc()!=crc32_core(reinterpret_cast<uint32_t*>(&cmd),(sizeof(cmd)>>2)-1)){++bad_crc;return;}
   for(int i=0;i<27;++i){auto x=cmd.motor_cmd()[i];if(!std::isfinite(x.tau())||!std::isfinite(x.q())||!std::isfinite(x.dq())||!std::isfinite(x.kp())||!std::isfinite(x.kd())||x.kp()!=0||x.kd()!=0||std::abs(x.tau())>limits[i]+.001){++bad_command;return;}}
   {std::lock_guard<std::mutex> lock(mutex);latest=cmd;cmd_time=Clock::now();}++count;
  });
  std::ofstream csv(argv[2]);if(!csv)throw std::runtime_error("cannot create trace");
  csv<<"wall_time,sim_time,stage,height,roll,pitch,body_vx,body_vy,body_wz,commands,bad_crc,bad_command,command_age,states,base_constrained,support_newtons,contacts,fall,invalid,left_knee,right_knee,imu_gx,imu_gy,imu_gz,gravity_x,gravity_y,gravity_z,left_contact,right_contact,left_max_gap,right_max_gap,left_max_lift,right_max_lift,left_max_drift,right_max_drift\n";
  auto start=Clock::now(),next=start,stand_start=start;std::string stage="passive",old_stage;bool active=false,fall=false;int step=0,states=0;
  bool foot_contact[2]={false,false}, foot_initialized=false;
  double foot_ref[2][3]{}, gap[2]{}, max_gap[2]{}, max_lift[2]{}, max_drift[2]{};
  while(Clock::now()-start<55s){
   bool finite = std::isfinite(d->time);
   for (int i=0; i<m->nq; ++i) finite &= std::isfinite(d->qpos[i]);
   for (int i=0; i<m->nv; ++i) finite &= std::isfinite(d->qvel[i]);
   for (int i=0; i<m->nsensordata; ++i) finite &= std::isfinite(d->sensordata[i]);
   for (int i=0; i<9; ++i) finite &= std::isfinite(d->site_xmat[9*imu+i]);
   for (int side=0; side<2; ++side)
    for (int axis=0; axis<3; ++axis) finite &= std::isfinite(d->xpos[3*feet[side]+axis]);
   if (!finite) {
    csv << "0,0,active,0,0,0,0,0,0,0,0,0,-1,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0\n";
    csv.flush();
    break;
   }
   std::ifstream f(argv[1]);std::string requested;if(f>>requested)stage=requested;
   if(stage!=old_stage){if(stage=="stand")stand_start=Clock::now();if(stage=="active")active=true;old_stage=stage;}
   if (active) {
    for (int side=0; side<2; ++side) {
     const double* pos=d->xpos+3*feet[side];
     if (!foot_initialized) for (int axis=0; axis<3; ++axis) foot_ref[side][axis]=pos[axis];
     foot_contact[side]=false;
     for (int c=0; c<d->ncon; ++c) {
      int g1=d->contact[c].geom1, g2=d->contact[c].geom2;
      if ((g1==floor && m->geom_bodyid[g2]==feet[side]) ||
          (g2==floor && m->geom_bodyid[g1]==feet[side])) foot_contact[side]=true;
     }
     gap[side]=foot_contact[side]?0:gap[side]+m->opt.timestep;
     max_gap[side]=std::max(max_gap[side],gap[side]);
     max_lift[side]=std::max(max_lift[side],pos[2]-foot_ref[side][2]);
     max_drift[side]=std::max(max_drift[side],std::hypot(pos[0]-foot_ref[side][0],pos[1]-foot_ref[side][1]));
    }
    foot_initialized=true;
   }
   LC cmd;Clock::time_point last;{std::lock_guard<std::mutex> lock(mutex);cmd=latest;last=cmd_time;}
   double age=last.time_since_epoch().count()?std::chrono::duration<double>(Clock::now()-last).count():-1;
   if(age>=0&&age<.1)for(int i=0;i<27;++i){auto x=cmd.motor_cmd()[i];double q=d->qpos[m->jnt_qposadr[j[i]]],dq=d->qvel[m->jnt_dofadr[j[i]]];d->ctrl[a[i]]=std::clamp(double(x.tau()+x.kp()*(x.q()-q)+x.kd()*(x.dq()-dq)),-double(limits[i]),double(limits[i]));}
   else for(int i=0;i<27;++i)d->ctrl[a[i]]=0;
   {
    LS msg{};double quat[4];mju_mat2Quat(quat,d->site_xmat+9*imu);
    for(int i=0;i<4;++i)msg.imu_state().quaternion()[i]=quat[i];
    for(int i=0;i<3;++i)msg.imu_state().gyroscope()[i]=d->sensordata[m->sensor_adr[gyro]+i];
    for(int i=0;i<27;++i){msg.motor_state()[i].q()=d->qpos[m->jnt_qposadr[j[i]]];msg.motor_state()[i].dq()=d->qvel[m->jnt_dofadr[j[i]]];}
    msg.crc()=crc32_core(reinterpret_cast<uint32_t*>(&msg),(sizeof(msg)>>2)-1);if (!pub.Write(msg,0)) throw std::runtime_error("LowState publish failed");
    ++states;
   }
   if(step%10==0){
    auto q=d->qpos+3;double roll=std::atan2(2*(q[0]*q[1]+q[2]*q[3]),1-2*(q[1]*q[1]+q[2]*q[2]));
    double pitch=std::asin(std::clamp(2*(q[0]*q[2]-q[3]*q[1]),-1.,1.));
    double inv[4],vel[3];mju_negQuat(inv,q);mju_rotVecQuat(vel,d->qvel,inv);
    fall=d->qpos[2]<.6||std::abs(roll)>1||std::abs(pitch)>1;
    csv<<std::chrono::duration<double>(Clock::now()-start).count()<<','<<d->time<<','<<stage<<','<<d->qpos[2]<<','<<roll<<','<<pitch<<','<<vel[0]<<','<<vel[1]<<','<<d->sensordata[m->sensor_adr[gyro]+2]<<','<<count.load()<<','<<bad_crc.load()<<','<<bad_command.load()<<','<<age<<','<<states<<','<<(!active)<<",0,"<<d->ncon<<','<<fall<<",0,"<<d->qpos[m->jnt_qposadr[j[3]]]<<','<<d->qpos[m->jnt_qposadr[j[9]]]<<','<<d->sensordata[m->sensor_adr[gyro]]<<','<<d->sensordata[m->sensor_adr[gyro]+1]<<','<<d->sensordata[m->sensor_adr[gyro]+2]<<','<<-d->site_xmat[9*imu+6]<<','<<-d->site_xmat[9*imu+7]<<','<<-d->site_xmat[9*imu+8]<<','<<foot_contact[0]<<','<<foot_contact[1]<<','<<max_gap[0]<<','<<max_gap[1]<<','<<max_lift[0]<<','<<max_lift[1]<<','<<max_drift[0]<<','<<max_drift[1]<<'\n';csv.flush();
    if(fall)break;
   }
   mj_step(m,d);
   if(!active){
    double f=stage=="stand"?std::clamp(std::chrono::duration<double>(Clock::now()-stand_start).count()/3,0.,1.):0;
    d->qpos[0]=d->qpos[1]=0;d->qpos[2]=1.03-.08*f;d->qpos[3]=1;d->qpos[4]=d->qpos[5]=d->qpos[6]=0;
    for(int z=0;z<6;++z)d->qvel[z]=0;mj_forward(m,d);
   }
   ++step;next+=2ms;std::this_thread::sleep_until(next);
  }
  std::cout<<"commands="<<count.load()<<" bad_crc="<<bad_crc.load()<<" bad_command="<<bad_command.load()<<" fall="<<fall<<std::endl;
  sub.CloseChannel();
  mj_deleteData(d);mj_deleteModel(m);return 0;
 }catch(const std::exception&e){std::cerr<<e.what()<<std::endl;return 1;}
}
