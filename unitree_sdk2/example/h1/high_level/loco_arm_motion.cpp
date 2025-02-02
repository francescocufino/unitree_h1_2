#include <chrono>
#include <iostream>
#include <thread>
#include <cmath>
#include <memory>

//high level (for locomotion)
#include <unitree/robot/h1/loco/h1_loco_api.hpp>
#include <unitree/robot/h1/loco/h1_loco_client.hpp>

//low level (for arms)
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include <unitree/robot/h1/loco/h1_arm_motion.hpp>





class Loco_arm_motion{
  private:
    //parameters
    const std::string kTopicArmSDK = "rt/arm_sdk";
    const std::string kTopicState = "rt/lowstate";

    enum JointIndex {
    // Left leg
    kLeftHipYaw = 0,
    kLeftHipPitch = 1,
    kLeftHipRoll = 2,
    kLeftKnee = 3,
    kLeftAnkle = 4,
    kLeftAnkleRoll = 5,
    // Right leg
    kRightHipYaw = 6,
    kRightHipPitch = 7,
    kRightHipRoll = 8,
    kRightKnee = 9,
    kRightAnkle = 10,
    kRightAnkleRoll = 11,

    kWaistYaw = 12,

    // Left arm
    kLeftShoulderPitch = 13,
    kLeftShoulderRoll = 14,
    kLeftShoulderYaw = 15,
    kLeftElbow = 16,
    kLeftWistRoll = 17,
    kLeftWistPitch = 18,
    kLeftWistYaw = 19,
    // Right arm
    kRightShoulderPitch = 20,
    kRightShoulderRoll = 21,
    kRightShoulderYaw = 22,
    kRightElbow = 23,
    kRightWistRoll = 24,
    kRightWistPitch = 25,
    kRightWistYaw = 26,

    kNotUsedJoint = 27,
    kNotUsedJoint1 = 28,
    kNotUsedJoint2 = 29,
    kNotUsedJoint3 = 30,
    kNotUsedJoint4 = 31,
    kNotUsedJoint5 = 32,
    kNotUsedJoint6 = 33,
    kNotUsedJoint7 = 34
    };

    
    const std::array<float, 15> kp_array = { 120, 120, 80, 50, 50, 50, 50, 
                                    120, 120, 80, 50, 50, 50, 50, 
                                    200 };
    const std::array<float, 15> kd_array = { 2.0, 2.0, 1.5, 1.0, 1.0, 1.0, 1.0, 
                                    2.0, 2.0, 1.5, 1.0, 1.0, 1.0, 1.0, 
                                    2.0 };

    const std::array<JointIndex, 15> arm_joints = {
        JointIndex::kLeftShoulderPitch,  JointIndex::kLeftShoulderRoll,
        JointIndex::kLeftShoulderYaw,    JointIndex::kLeftElbow,
        JointIndex::kLeftWistRoll,       JointIndex::kLeftWistPitch,     JointIndex::kLeftWistYaw,       
        JointIndex::kRightShoulderPitch, JointIndex::kRightShoulderRoll,
        JointIndex::kRightShoulderYaw,   JointIndex::kRightElbow, 
        JointIndex::kRightWistRoll,      JointIndex::kRightWistPitch,    JointIndex::kRightWistYaw,       
        JointIndex::kWaistYaw};
    float weight = 0.f;
    float weight_rate = 0.2f;
    float dq = 0.f;
    float tau_ff = 0.f;

    float control_dt = 0.02f;
    float max_joint_velocity = 0.5f;

    float delta_weight = weight_rate * control_dt;
    float max_joint_delta = max_joint_velocity * control_dt;
    std::chrono::duration<int64_t, std::milli> sleep_time = std::chrono::milliseconds(static_cast<int>(control_dt / 0.001f));

    //client for high level (loco)
    

    std::shared_ptr<unitree::robot::h1::LocoClient> client;
    //Publisher, subscriber and msg for low level (arms)
    unitree::robot::ChannelPublisherPtr<unitree_hg::msg::dds_::LowCmd_> arm_sdk_publisher;
    std::shared_ptr<unitree_hg::msg::dds_::LowCmd_> msg;
    unitree::robot::ChannelSubscriberPtr<unitree_hg::msg::dds_::LowState_> low_state_subscriber;
    std::shared_ptr<unitree_hg::msg::dds_::LowState_> state_msg;
  public:
    Loco_arm_motion();
    void walk(float, float, float, float);
    void initialize_arms();
    void move_arms_integral(std::array<float, 15>);
    void move_arms_polynomial(std::array<float, 15>, float);
    void stop_arms();



  };

Loco_arm_motion::Loco_arm_motion(){
  client = std::make_shared<unitree::robot::h1::LocoClient>();  // Shared allocation  //Start loco client
  //client->Init();
  //client->SetTimeout(10.f);
  //client->Start();

  //Start low cmd publisher and subscriber

  msg = std::make_shared<unitree_hg::msg::dds_::LowCmd_>();
  state_msg = std::make_shared<unitree_hg::msg::dds_::LowState_>();
  arm_sdk_publisher.reset(new unitree::robot::ChannelPublisher<unitree_hg::msg::dds_::LowCmd_>(kTopicArmSDK));
  arm_sdk_publisher->InitChannel();
  low_state_subscriber.reset(new unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowState_>(kTopicState));
  low_state_subscriber->InitChannel(
      [&](const void *msg_ptr) {
          auto s = static_cast<const unitree_hg::msg::dds_::LowState_*>(msg_ptr);
          *state_msg = *s;  // Dereferencing shared_ptr to copy data
      }, 
      1
  );
}




void Loco_arm_motion::initialize_arms(){
   std::array<float, 15> init_pos{0.f, 0.3,  0.f, 0, 0, 0, 0,
                                     0.f, -0.3, 0.f, 0, 0, 0, 0,
                                     0.f};

  // wait for init
  std::cout << "Press ENTER to init arms ...";
  std::cin.get();


  // get current joint position
  std::array<float, 15> current_jpos{};
  std::cout<<"Current joint position: ";
  for (int i = 0; i < arm_joints.size(); ++i) {
	current_jpos.at(i) = 0;//state_msg->motor_state().at(arm_joints.at(i)).q();
	std::cout << current_jpos.at(i) << " ";
  }
  std::cout << std::endl;

  // set init pos
  std::cout << "Initailizing arms ...";
  float init_time = 2.0f;
  int init_time_steps = static_cast<int>(init_time / control_dt);

  for (int i = 0; i < init_time_steps; ++i) {
    // set weight
    weight = 1.0;
    msg->motor_cmd().at(JointIndex::kNotUsedJoint).q(weight);
    float phase = 1.0 * i / init_time_steps;
    //std::cout << "Phase: " << phase << std::endl;

    // set control joints
    for (int j = 0; j < init_pos.size(); ++j) {
      //std::cout << "q" << j << ": " << init_pos.at(j) * phase + current_jpos.at(j) * (1 - phase) << ' ';
      msg->motor_cmd().at(arm_joints.at(j)).q(init_pos.at(j) * phase + current_jpos.at(j) * (1 - phase));
      msg->motor_cmd().at(arm_joints.at(j)).dq(dq);
      msg->motor_cmd().at(arm_joints.at(j)).kp(kp_array.at(j));
      msg->motor_cmd().at(arm_joints.at(j)).kd(kd_array.at(j));
      msg->motor_cmd().at(arm_joints.at(j)).tau(tau_ff);
    }
    //std::cout << std::endl;

    // send dds msg
    arm_sdk_publisher->Write(*msg);

    // sleep
    std::this_thread::sleep_for(sleep_time);
  }

  std::cout << "Done!" << std::endl;
}

void Loco_arm_motion::move_arms_integral(std::array<float, 15> target_pos){

   std::cout << "Press ENTER to start arm ctrl ..." << std::endl;
  std::cin.get();

  // start control
  std::cout << "Start arm ctrl!" << std::endl;
  float period = 5.f;
  int num_time_steps = static_cast<int>(period / control_dt);

  std::array<float, 15> current_jpos_des{0.f, 0.3,  0.f, 0, 0, 0, 0,
                                        0.f, -0.3, 0.f, 0, 0, 0, 0,
                                        0.f};

  // lift arms up
  for (int i = 0; i < num_time_steps; ++i) {
    // update jpos des
    for (int j = 0; j < target_pos.size(); ++j) {
      current_jpos_des.at(j) +=
          std::clamp(target_pos.at(j) - current_jpos_des.at(j),
                     -max_joint_delta, max_joint_delta);
    }

    // set control joints
    for (int j = 0; j < target_pos.size(); ++j) {
      //std::cout << "q" << j << ": " << current_jpos_des.at(j) << ' ';
      msg->motor_cmd().at(arm_joints.at(j)).q(current_jpos_des.at(j));
      msg->motor_cmd().at(arm_joints.at(j)).dq(dq);
      msg->motor_cmd().at(arm_joints.at(j)).kp(kp_array.at(j));
      msg->motor_cmd().at(arm_joints.at(j)).kd(kd_array.at(j));
      msg->motor_cmd().at(arm_joints.at(j)).tau(tau_ff);
    }
    //std::cout << std::endl;

    // send dds msg
    arm_sdk_publisher->Write(*msg);

    // sleep
    std::this_thread::sleep_for(sleep_time);
  }
}

void Loco_arm_motion::move_arms_polynomial(std::array<float, 15> q_f, float t_f){
  //Initial time
  float t = 0;

  //Initial configuration q_i
  std::array<float, 15> q_i{};
  std::cout<<"Current joint position: ";
  for (int i = 0; i < arm_joints.size(); ++i) {
	  q_i.at(i) = 0;//state_msg->motor_state().at(arm_joints.at(i)).q();
  }

  //Configuration to command which will be obtained through 5-th order polynomial
  std::array<float, 15> q_cmd{};

  //Planning parameters
  std::array<float, 15> a0{}, a1{}, a2{}, a3{}, a4{}, a5{};
  for (int j = 0; j < q_f.size(); ++j) {
    a0.at(j) = q_i.at(j);
    a1.at(j) = 0;
    a2.at(j) = 0;
    a3.at(j) = (10*q_f.at(j) - 10*q_i.at(j))/pow(t_f, 3);
    a4.at(j) = (-15*q_f.at(j) + 15*q_i.at(j))/pow(t_f, 4);
    a5.at(j) = (6*q_f.at(j) - 6*q_i.at(j))/pow(t_f, 5);
  }

  std::cout << "Press ENTER to start arm ctrl poly..." << std::endl;
  std::cin.get();

  // start control
  std::cout << "Start arm ctrl poly!" << std::endl;

  //Planning over the time interval [0, t_f]
  while(t<=t_f){

    //Commanding joints
    for (int j = 0; j < q_cmd.size(); ++j) {
      q_cmd.at(j) =a5.at(j)*pow(t,5) + a4.at(j)*pow(t,4) + a3.at(j)*pow(t,3) + a2.at(j)*pow(t,2)+ a1.at(j)*t + a0.at(j);
      std::cout << "q" << j << ": " << q_cmd.at(j) << ' ';
      msg->motor_cmd().at(arm_joints.at(j)).q(q_cmd.at(j));
      msg->motor_cmd().at(arm_joints.at(j)).dq(dq);
      msg->motor_cmd().at(arm_joints.at(j)).kp(kp_array.at(j));
      msg->motor_cmd().at(arm_joints.at(j)).kd(kd_array.at(j));
      msg->motor_cmd().at(arm_joints.at(j)).tau(tau_ff);
    }
    std::cout << std::endl;

    // send dds msg
    arm_sdk_publisher->Write(*msg);

    // sleep
    std::this_thread::sleep_for(sleep_time);

    t = t + control_dt;
  }
  

}


void Loco_arm_motion::stop_arms(){
  std::cout << "Stoping arm ctrl ...";
  float stop_time = 2.0f;
  int stop_time_steps = static_cast<int>(stop_time / control_dt);

  for (int i = 0; i < stop_time_steps; ++i) {
    // increase weight
    weight -= delta_weight;
    weight = std::clamp(weight, 0.f, 1.f);

    // set weight
    msg->motor_cmd().at(JointIndex::kNotUsedJoint).q(weight);

    // send dds msg
    arm_sdk_publisher->Write(*msg);

    // sleep
    std::this_thread::sleep_for(sleep_time);
  }
}


void Loco_arm_motion::walk(float vx, float vy, float vyaw, float duration){
  client->SetVelocity(vx, vy, vyaw, duration);
}





int main(int argc, char const *argv[]) {
  unitree::robot::ChannelFactory::Instance()->Init(0);
  Loco_arm_motion loco;
  //loco.walk(0.3, 0, 0, 1);
  //loco.initialize_arms();
  std::array<float, 15> target_pos = {0.f, M_PI_2,  0.f, M_PI_2, 0, 0, 0,
                                     0.f, -M_PI_2, 0.f, M_PI_2, 0, 0, 0,
                                     0.f};
  //loco.move_arms_integral(target_pos);
  loco.move_arms_polynomial(target_pos, 10);
  loco.stop_arms();

    
}











