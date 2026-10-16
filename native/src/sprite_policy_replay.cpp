// No direct CAN/I2C. Powered IPC requires a separate build and explicit admission.
#if defined(SPRITE_POWERED_LOOP) && (!defined(SPRITE_LOOP_TEST) || defined(SPRITE_SHADOW_LOOP))
#error Invalid powered build flags
#endif
#ifdef SPRITE_LOOP_TEST
#include "policy_state_ipc.hpp"
#include "policy_imu_client.hpp"
#include "policy_command_schedule.hpp"
#include "policy_evidence_buffer.hpp"
#endif
#include "policy_core.hpp"
#include "policy_feedback.hpp"
#include "policy_protection.hpp"
#include <memory>
#include <onnxruntime_cxx_api.h>
#include <fstream>
#include <iostream>
#include <chrono>
#include <cstring>
#include <cstdint>

template<class T> void read(std::istream& in,T& value) {
  if(!in.read(reinterpret_cast<char*>(&value),sizeof(value))) throw std::runtime_error("truncated input");
}
template<class T> void write(std::ostream& out,const T& value) {
  if(!out.write(reinterpret_cast<const char*>(&value),sizeof(value))) throw std::runtime_error("output failure");
}
int main(int argc,char** argv) {
  try {
#if defined(SPRITE_POWERED_LOOP)
    if(argc!=9 || std::string(argv[8])!="ENABLE_NATIVE_ACTOR_WALK40" ||
       std::string(argv[7])!="stand-walk010-stop-40s")
      throw std::runtime_error("powered actor requires explicit walk40 admission");
#elif defined(SPRITE_SHADOW_LOOP)
    if(argc!=7 && argc!=8)throw std::runtime_error("Usage: shadow MODEL INPUT OUTPUT STATE_SOCKET IMU_SOCKET NAMES [PROFILE]");
#elif defined(SPRITE_LOOP_TEST)
    if(argc!=6 && argc!=7) throw std::runtime_error("Usage: loop_test MODEL INPUT OUTPUT FAKE_STATE_SOCKET FAKE_IMU_SOCKET [PROFILE]");
#else
    if(argc!=4) throw std::runtime_error("Usage: sprite_policy_replay MODEL INPUT OUTPUT");
#endif
    const std::uint32_t endian=1;
    if(*reinterpret_cast<const char*>(&endian)!=1) throw std::runtime_error("little endian required");
    std::ifstream in(argv[2],std::ios::binary);
    std::ofstream output_file(argv[3],std::ios::binary);
    if(!in || !output_file) throw std::runtime_error("file open failed");
    std::array<char,8> magic{}; read(in,magic);
    const bool chain=std::memcmp(magic.data(),"SPCHN001",8)==0;
#ifdef SPRITE_LOOP_TEST
    if(!chain)throw std::runtime_error("loop test requires chain fixture");
#endif
    if(!chain && std::memcmp(magic.data(),"SPPOL001",8)) throw std::runtime_error("bad input magic");
    std::uint32_t count=0; read(in,count);
    if(!count || count>1000000) throw std::runtime_error("invalid frame count");
#ifdef SPRITE_POWERED_LOOP
    if(count!=2001)throw std::runtime_error("powered walk40 requires preparation state plus 2000 active states");
#endif
#ifdef SPRITE_LOOP_TEST
    if(count>6001)throw std::runtime_error("live evidence capacity exceeds 120 seconds");
    sprite_policy::EvidenceBuffer output_buffer(static_cast<std::size_t>(count)*6000);
    sprite_policy::EvidenceBuffer input_buffer(static_cast<std::size_t>(count)*72*8);
    sprite_policy::EvidenceBuffer timing_buffer(static_cast<std::size_t>(count)*128+128);
    std::ostream out(&output_buffer);
#else
    auto& out=output_file;
#endif
    sprite_policy::Core core; read(in,core.steps);
    if(core.steps>100000) throw std::runtime_error("invalid handoff steps");
    read(in,core.defaults);read(in,core.offset);read(in,core.scale);
    for(int j=0;j<31;++j) if(!std::isfinite(core.defaults[j]) || !std::isfinite(core.offset[j]) ||
      !std::isfinite(core.scale[j]) || std::abs(core.scale[j])<=1e-12)
        throw std::runtime_error("invalid affine configuration");
    sprite_policy::FeedbackMap feedback;
    std::unique_ptr<sprite_policy::TargetProjector> projector;
    std::unique_ptr<sprite_policy::StartupRamp> startup;
    sprite_policy::ProtectionVector kp{},kd{};
    if(chain) {
      read(in,feedback.motor_zero);read(in,feedback.motor_to_joint);
      sprite_policy::ProtectionVector lower{},upper{},gain{},damping{};
      read(in,lower);read(in,upper);read(in,gain);read(in,damping);
      std::array<double,5> settings{};read(in,settings);
      projector=std::make_unique<sprite_policy::TargetProjector>(lower,upper,gain,damping,settings[0],settings[1]);
      startup=std::make_unique<sprite_policy::StartupRamp>(settings[2],settings[3],settings[4]);
      read(in,kp);read(in,kd);
    }
    Ort::Env env(ORT_LOGGING_LEVEL_WARNING,"sprite-offline-policy");
    Ort::SessionOptions options; options.SetIntraOpNumThreads(1);options.SetInterOpNumThreads(1);
    options.SetExecutionMode(ExecutionMode::ORT_SEQUENTIAL);
    Ort::Session session(env,argv[1],options);
    if(session.GetInputCount()!=1 || session.GetOutputCount()!=1) throw std::runtime_error("model IO count");
    auto it=session.GetInputTypeInfo(0);auto ot=session.GetOutputTypeInfo(0);
    auto info=it.GetTensorTypeAndShapeInfo();auto output_info=ot.GetTensorTypeAndShapeInfo();
    if(info.GetElementType()!=ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT ||
       output_info.GetElementType()!=ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT ||
       info.GetShape()!=std::vector<int64_t>({1,795}) ||
       output_info.GetShape()!=std::vector<int64_t>({1,31})) throw std::runtime_error("model shape/type mismatch");
    Ort::AllocatorWithDefaultOptions allocator;
    auto input_name=session.GetInputNameAllocated(0,allocator);
    auto output_name=session.GetOutputNameAllocated(0,allocator);
    const char* ins[]={input_name.get()};const char* outs[]={output_name.get()};
    const std::array<int64_t,2> shape{1,795};
    auto memory=Ort::MemoryInfo::CreateCpu(OrtArenaAllocator,OrtMemTypeDefault);
#ifdef SPRITE_LOOP_TEST
    std::unique_ptr<sprite_policy::CommandSchedule> schedule;
#if defined(SPRITE_POWERED_LOOP)
    schedule=std::make_unique<sprite_policy::CommandSchedule>(argv[7]);
#elif defined(SPRITE_SHADOW_LOOP)
    if(argc==8) schedule=std::make_unique<sprite_policy::CommandSchedule>(argv[7]);
#else
    if(argc==7) schedule=std::make_unique<sprite_policy::CommandSchedule>(argv[6]);
#endif
    std::string last_stage;
    int last_countdown=0;
    std::array<std::string,31> motors,joints;
#if defined(SPRITE_SHADOW_LOOP) || defined(SPRITE_POWERED_LOOP)
    std::ifstream names(argv[6]);std::string tag;
    if(!(names>>tag))throw std::runtime_error("missing manifest tag");
#ifdef SPRITE_POWERED_LOOP
    if(tag!="SPRITE_POWERED_WALK40_V1")throw std::runtime_error("powered manifest invalid");
#else
    if(tag!="SPRITE_ZERO_GAIN_SHADOW_V1")throw std::runtime_error("shadow manifest invalid");
#endif
    for(auto& n:motors)if(!(names>>n))throw std::runtime_error("missing motor name");
    for(auto& n:joints)if(!(names>>n))throw std::runtime_error("missing joint name");
    names>>std::ws;if(!names.eof())throw std::runtime_error("trailing names");
#else
    for(int j=0;j<31;++j){motors[j]="motor_"+std::to_string(j);joints[j]="joint_"+std::to_string(j);}
#endif
#ifdef SPRITE_ICM_IMU_INPUT
    sprite_policy::NativeImuClient imu(argv[5],false,true);
#elif defined(SPRITE_SOFTWARE_IMU_INPUT)
    sprite_policy::NativeImuClient imu(argv[5],true);
#else
    sprite_policy::NativeImuClient imu(argv[5]);
#endif
    sprite_policy::ImuSnapshot sample;
    const auto ready_deadline=sprite_policy::monotonic_ns()+100000000;
    while(!imu.snapshot(sample)) {
      if(sprite_policy::monotonic_ns()>ready_deadline)throw std::runtime_error("IMU startup timeout");
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    // Test-only hashes cannot match the actual robot controller's ordered names.
    sprite_policy::PolicyStateChannel channel(argv[4],sprite_policy::name_hash(motors),sprite_policy::name_hash(joints),100000000,
#if defined(SPRITE_SHADOW_LOOP) || defined(SPRITE_POWERED_LOOP)
      1000
#else
      100
#endif
    );
    std::ofstream used_file(std::string(argv[3])+".inputs",std::ios::binary);
    std::ofstream timing_file(std::string(argv[3])+".timing");
    if(!used_file || !timing_file)throw std::runtime_error("test evidence file open failed");
    std::ostream used(&input_buffer),timings(&timing_buffer);
    timings<<"compute_ms,source_age_ms,imu_age_ms\n";
    auto save_evidence=[&]() {
      output_buffer.save(output_file);input_buffer.save(used_file);timing_buffer.save(timing_file);
      output_file.flush();used_file.flush();timing_file.flush();
      if(!output_file || !used_file || !timing_file)throw std::runtime_error("evidence flush failed");
    };
#endif
    double max_ms=0,total_ms=0;
#ifdef SPRITE_LOOP_TEST
    try {
#endif
    for(std::uint32_t n=0;n<count;++n) {
#ifdef SPRITE_LOOP_TEST
      auto state=channel.receive(sprite_policy::monotonic_ns);
      if(!imu.snapshot(sample))throw std::runtime_error("IMU missing");
#endif
      const auto start=std::chrono::steady_clock::now();
      std::array<double,71> input{};
      if(chain) {
        std::array<double,69> sensors{};std::array<double,3> command{};
        read(in,sensors);read(in,command);
#ifdef SPRITE_LOOP_TEST
        std::copy(state.position.begin(),state.position.end(),sensors.begin());
        std::copy(state.velocity.begin(),state.velocity.end(),sensors.begin()+31);
        std::copy_n(sample.values.begin()+3,3,sensors.begin()+62);
        std::copy_n(sample.values.begin()+6,4,sensors.begin()+65);
        if(schedule) {
          const auto scheduled=schedule->sample(state.stamp);
          command={scheduled.vx,0,0};
          if(last_stage!=scheduled.stage) {
            last_stage=scheduled.stage;
            std::cout<<"COMMAND stage="<<last_stage<<" vx="<<scheduled.vx<<std::endl;
          }
          const int remaining=schedule->countdown();
          if(remaining && remaining!=last_countdown) {
            last_countdown=remaining;
            std::cout<<"DISABLE_COUNTDOWN approximately "<<remaining<<"s; zero velocity command"<<std::endl;
          }
        }
        write(used,sensors);write(used,command);
#endif
        auto joint_imu=feedback.convert(sensors);
        std::copy(joint_imu.begin(),joint_imu.end(),input.begin());
        std::copy(command.begin(),command.end(),input.begin()+68);
      } else read(in,input);
      auto observation=core.observe(input);
      auto tensor=Ort::Value::CreateTensor<float>(memory,observation.data(),observation.size(),shape.data(),shape.size());
      auto values=session.Run(Ort::RunOptions{nullptr},ins,&tensor,1,outs,1);
      std::array<float,31> raw{};
      std::copy_n(values[0].GetTensorData<float>(),31,raw.begin());
      const auto target=core.latch(raw,input);
      sprite_policy::ImpedanceTarget protected_target;
      if(chain) {
        protected_target.position=target;protected_target.kp=kp;protected_target.kd=kd;
        protected_target=projector->project(protected_target);
        sprite_policy::ProtectionVector measured{};std::copy_n(input.begin(),31,measured.begin());
        if(!startup->captured())measured=projector->require_position(measured);
        protected_target=startup->apply(protected_target,measured);
      }
#ifdef SPRITE_LOOP_TEST
      sprite_policy::ImuSnapshot latest;
      if(!imu.snapshot(latest))throw std::runtime_error("IMU missing before send");
      const auto now=sprite_policy::monotonic_ns();
      if(now<sample.start || now-sample.start>30000000)throw std::runtime_error("consumed IMU stale before send");
#ifdef SPRITE_SHADOW_LOOP
      // Defence in depth: this shadow-only build never emits impedance or torque over IPC.
      auto zero_target=protected_target;
      zero_target.velocity.fill(0);zero_target.kp.fill(0);zero_target.kd.fill(0);zero_target.feedforward.fill(0);
      channel.send(zero_target,state,now);
#else
      channel.send(protected_target,state,now);
#endif
#endif
      const double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
      max_ms=std::max(max_ms,ms);total_ms+=ms;
#ifdef SPRITE_LOOP_TEST
      timings<<ms<<','<<(now-state.stamp)/1e6<<','<<(now-sample.start)/1e6<<'\n';
      if(!timings)throw std::runtime_error("timing evidence overflow");
#endif
      write(out,observation);write(out,raw);write(out,core.previous);write(out,target);
      if(chain) {
        write(out,protected_target.position);write(out,protected_target.velocity);
        write(out,protected_target.kp);write(out,protected_target.kd);write(out,protected_target.feedforward);
      }
    }
#if defined(SPRITE_POWERED_LOOP)
    channel.await_controller_close(1000,true);
#elif defined(SPRITE_SHADOW_LOOP)
    channel.finish_shadow();
#endif
#ifdef SPRITE_LOOP_TEST
    } catch(...) {
      // Disconnect first so slow storage cannot delay controller fault handling.
      channel.close_after_fault();
      try {save_evidence();} catch(const std::exception& e) {std::cerr<<e.what()<<'\n';}
      throw;
    }
    save_evidence();
#endif
    if(in.peek()!=std::char_traits<char>::eof()) throw std::runtime_error("trailing input bytes");
#ifdef SPRITE_POWERED_LOOP
    std::cout<<"POWERED_IPC transport_closed=1 controller_report_required=1 frames=";
#elif defined(SPRITE_SHADOW_LOOP)
    std::cout<<"ZERO_GAIN_IPC frames=";
#else
    std::cout<<"OFFLINE_OR_FAKE frames=";
#endif
    std::cout<<count<<" mean_ms="<<total_ms/count<<" max_ms="<<max_ms<<'\n';
    return 0;
  } catch(const std::exception& error) {std::cerr<<error.what()<<'\n';return 1;}
}
