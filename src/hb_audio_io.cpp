// Copyright (c) 2024，D-Robotics.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "hb_audio_io.h"

#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include "speech_engine.h"
#include <json/json.h>
#include <chrono>
#include <thread>

namespace hobot {
namespace audio {
HBAudioIo::HBAudioIo(const std::string &node_name,
                               const NodeOptions &options)
    : rclcpp::Node(node_name, options), strip(5), sherpa_tts_(){
  
  std::string tros_distro
      = std::string(std::getenv("TROS_DISTRO")? std::getenv("TROS_DISTRO") : "");
  // asr_model_path_ = "/opt/tros/" + tros_distro + "/lib/sensevoice_ros2/model/";
  //asr_model_path_ = "./install/lib/audio_io/model/";
  


  asr_model_path_ = "/root/";
  cmd_word_path_ = "/opt/tros/" + tros_distro + "/lib/sensevoice_ros2/config/cmd_word.json";
  //cmd_word_path_ = "./install/lib/audio_io/config/cmd_word.json";

  this->declare_parameter<std::string>("micphone_name",
                                       micphone_name_);
  this->declare_parameter<std::string>("audio_pub_topic_name",
                                       audio_pub_topic_name_);
  this->declare_parameter<std::string>("asr_pub_topic_name",
                                       asr_pub_topic_name_);
  this->declare_parameter<std::string>("asr_model",
                                       asr_model_);
  this->declare_parameter<int>("push_wakeup",
                                       push_wakeup_);
  this->declare_parameter<std::string>("wakeup_name",
                                       wakeup_name_);
  this->declare_parameter<std::string>("tts_sub_topic_name",
                                       tts_sub_topic_name_);
  this->declare_parameter<std::string>("tts_config_path",
                                       tts_config_path_);
  this->declare_parameter<std::string>("asr_model_path",
                                       asr_model_path_);

  this->get_parameter<std::string>("micphone_name",
                                   micphone_name_);
  this->get_parameter<std::string>("audio_pub_topic_name",
                                   audio_pub_topic_name_);
  this->get_parameter<std::string>("asr_pub_topic_name",
                                   asr_pub_topic_name_);
  this->get_parameter<std::string>("asr_model",
                                   asr_model_);
  this->get_parameter<int>("push_wakeup",
                                   push_wakeup_);
  this->get_parameter<std::string>("wakeup_name",
                                   wakeup_name_);
  this->get_parameter<std::string>("tts_sub_topic_name",
                                   tts_sub_topic_name_);
  this->get_parameter<std::string>("tts_config_path",
                                   tts_config_path_);
  this->get_parameter<std::string>("asr_model_path",
                                   asr_model_path_);
  
  // std::thread loader([this]() {
  //       this->LoadTtsModelAsync();
  //   });
  // loader.detach();
  
  if (wakeup_name_.length() > 0) {
    wakeup_name_1_ = wakeup_name_ + ",";
  }
  int err_code = 0;
  
  asr_model_path_ += asr_model_;
  std::stringstream ss;
  ss << "Parameter:"
     << "\n micphone_name: " << micphone_name_
     << "\n audio_pub_topic_name: " << audio_pub_topic_name_
     << "\n asr_pub_topic_name: " << asr_pub_topic_name_
     << "\n asr_model_path_: " << asr_model_path_
     << "\n tts_sub_topic_name: " << tts_sub_topic_name_
     << "\n push_wakeup: " << push_wakeup_;
  RCLCPP_WARN(rclcpp::get_logger("audio_io"), "%s", ss.str().c_str());
}

HBAudioIo::~HBAudioIo() { DeInit(); }

// void HBAudioIo::LoadTtsModelAsync() {
//     sherpa_tts_.Init(micphone_name_, tts_config_path_);
//     std::cout<<"----------------------init--------------------------"<<std::endl; 
//     std::unique_lock<std::mutex> model_init_lock(model_init_mtx_);
//     model_init_ = true;
//     model_init_cv_.notify_one();
// }


int HBAudioIo::Init() {
  v_cmd_word_ = std::make_shared<std::vector<std::string>>();

  std::ifstream cmd_word(cmd_word_path_);
  if (cmd_word.is_open()) {
    Json::Value root;
    cmd_word >> root;
    if (root.isMember("cmd_word") && root["cmd_word"].isArray()) {
      const Json::Value& cmdWords = root["cmd_word"];
      for (const auto& word : cmdWords) {
        // std::cout << "命令词: " << word.asString() << std::endl;
        v_cmd_word_->push_back(word.asString());
      }
    }
    cmd_word.close();
  }  

  RCLCPP_INFO(rclcpp::get_logger("audio_io"), "init to capture audio");
  micphone_device_ = alsa_device_allocate();
  if (!micphone_device_) {
    RCLCPP_INFO(rclcpp::get_logger("audio_io"), "open mic device fail");
    return -1;
  }

  /* init micphone device*/
  micphone_device_->name = const_cast<char *>(micphone_name_.c_str());
  micphone_device_->format = SND_PCM_FORMAT_S16;
  micphone_device_->direct = SND_PCM_STREAM_CAPTURE;
  micphone_device_->rate = micphone_rate_;
  micphone_device_->channels = micphone_chn_;
  micphone_device_->buffer_time = micphone_buffer_time_;
  micphone_device_->nperiods = micphone_nperiods_;
  micphone_device_->period_size = micphone_period_size_;
  int ret = alsa_device_init(micphone_device_);
  if (ret < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("audio_io"),
                 "alsa device micphone init fail, ret=%d", ret);
    return -1;
  }

  //加载TTS
  sherpa_tts_.Init(micphone_name_, tts_config_path_);

  RCLCPP_WARN_STREAM(rclcpp::get_logger("audio_io"),
    "asr_model_path_ is [" << asr_model_path_ << "]");
   speech_engine::Instance()->Init(asr_model_path_, wakeup_name_, v_cmd_word_,
       std::bind(&HBAudioIo::AudioASRFunc, this, std::placeholders::_1),
       std::bind(&HBAudioIo::AudioCmdDataFunc, this, std::placeholders::_1));

  RCLCPP_WARN(rclcpp::get_logger("audio_io"), "init success");
  // system("rm ./*.pcm -rf");
  if (save_audio_) {
    audio_infile_.open("./audio_in.pcm",
                       std::ios::app | std::ios::out | std::ios::binary);
  }
  msg_publisher_ = this->create_publisher<audio_msg::msg::SmartAudioData>(
      audio_pub_topic_name_, 10);
  asr_msg_publisher_ = this->create_publisher<std_msgs::msg::String>(asr_pub_topic_name_, 10);
  tts_msg_subscriber_ = this->create_subscription<std_msgs::msg::String>(tts_sub_topic_name_, 10, std::bind(&HBAudioIo::TTSMsgCallback, this, std::placeholders::_1));
  timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0), std::bind(&HBAudioIo::CheckLLMNodeExistence, this));
  is_init_ = true;

  for(int i = 0; i < 3; i++){
    strip.clear();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    strip.set_all_same_color(0, 0, 255);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
  return 0;
}

int HBAudioIo::DeInit() {
  RCLCPP_INFO(rclcpp::get_logger("audio_io"), "deinit");
  if (!is_init_) return 0;
  if (!micphone_device_) return -1;
  if (micphone_device_) {
    alsa_device_deinit(micphone_device_);
    alsa_device_free(micphone_device_);
    micphone_device_ = nullptr;
  }
  speech_engine::Instance()->Stop();
  speech_engine::Instance()->DeInit();
  if (audio_infile_.is_open()) {
    audio_infile_.close();
  }
  if (audio_sdk_.is_open()) {
    audio_sdk_.close();
  }
  return 0;
}


int HBAudioIo::Run() {
  if (!is_init_) {
    RCLCPP_ERROR(rclcpp::get_logger("audio_io"), "HBAudioIo not init.");
    return -1;
  }
  speech_engine::Instance()->Start();

  // 启动麦克风采集线程
  auto capture_task = std::make_shared<std::thread>(
      std::bind(&HBAudioIo::MicphoneGetThread, this));
  auto pcm_task = std::make_shared<std::thread>(
      std::bind(&HBAudioIo::PcmThread, this));
  auto speaker_task = std::make_shared<std::thread>(
      std::bind(&HBAudioIo::SpeakerThread, this));

  // 创建并运行执行器
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(this->get_node_base_interface());
  exec.spin();
  // 退出时关闭线程
  if (capture_task && capture_task->joinable()) {
    std::unique_lock<std::mutex> micphone_lock(micphone_mtx_);
    speaker_working_ = false;
    exiting_ = true;
    micphone_lock.unlock();
    micphone_cv_.notify_one();

    std::unique_lock<std::mutex> llm_node_init_lock(llm_node_init_mtx_);
    llm_node_init_ = true;
    llm_node_init_lock.unlock();
    llm_node_init_cv_.notify_all();
    
    capture_task->join();
    capture_task.reset();
  }
  if (speaker_task && speaker_task->joinable()) {
    std::unique_lock<std::mutex> playback_queue_lock(playback_queue_mtx_);
    while (!playback_queue_.empty()) {
        playback_queue_.pop();
    }
    exiting_ = true;
    playback_queue_lock.unlock();
    playback_queue_cv_.notify_one();
    speaker_task->join();
    speaker_task.reset();
  }
  if (pcm_task && pcm_task->joinable()) {
    std::unique_lock<std::mutex> tts_queue_lock(tts_queue_mtx_);
    while (!tts_data_queue_.empty()) {
        tts_data_queue_.pop();
    }
    exiting_ = true;
    tts_queue_lock.unlock();
    tts_queue_cv_.notify_one();

    std::unique_lock<std::mutex> llm_node_init_lock(llm_node_init_mtx_);
    llm_node_init_ = true;
    llm_node_init_lock.unlock();
    llm_node_init_cv_.notify_all();
    
    pcm_task->join();
    pcm_task.reset();
  }
  return 0;
}

int HBAudioIo::MicphoneGetThread() {
  RCLCPP_WARN(rclcpp::get_logger("audio_io"), "start to capture audio");
  if (!micphone_device_) {
    RCLCPP_ERROR(rclcpp::get_logger("audio_io"), "micphone device is null");
    return -1;
  }
  int ret = -1;
  snd_pcm_sframes_t frames;
  frames = micphone_device_->period_size;
  int buffer_size = snd_pcm_frames_to_bytes(micphone_device_->handle, frames);
  char *buffer = new char[buffer_size];
  char *buffer_1 = new char[buffer_size];
  auto vec_ptr = std::make_shared<std::vector<double>>();
  int num = 0;
  while (rclcpp::ok()) {
    {
      //等待大模型启动
      std::unique_lock<std::mutex> llm_node_init_lock(llm_node_init_mtx_);
      llm_node_init_cv_.wait(llm_node_init_lock, [this] { return llm_node_init_ || exiting_;});
      llm_node_init_lock.unlock();

      std::unique_lock<std::mutex> micphone_lock(micphone_mtx_);
      micphone_cv_.wait(micphone_lock, [this] { return !speaker_working_ || exiting_;});
      micphone_lock.unlock();
      if(exiting_ == true) break;
      // digitalWrite(0, HIGH);
      
      ret = alsa_device_read(micphone_device_, buffer, frames);
      if (ret <= 0) continue;
      RCLCPP_DEBUG(rclcpp::get_logger("audio_io"), "capture audio buffer_size:%d",
                  buffer_size);
      audio_num_++;
      int data_audio_size = buffer_size / 2 / 2;
      int16_t *src_ptr = (int16_t *)buffer;
      for (int i = 0; i < data_audio_size; i++) {
        vec_ptr->push_back((double)(src_ptr[i * 2])*2.5);
      }
      if(num % 4 == 0){
        speech_engine::Instance()->send_data(vec_ptr);
        vec_ptr->clear();
        num = 0;
      }
      if (save_audio_ && audio_infile_.is_open()) {
        audio_infile_.write(buffer_1, buffer_size / 2);
      }
      num++;
    }
  }
  RCLCPP_WARN(rclcpp::get_logger("audio_io"), "stop capture audio");
  delete[] buffer;
  delete[] buffer_1;
  return 0;
}

void HBAudioIo::AudioCmdDataFunc(std::string cmd_word) {
  RCLCPP_WARN(rclcpp::get_logger("audio_io"), "recv cmd word:%s", cmd_word.c_str());
  audio_msg::msg::SmartAudioData::UniquePtr frame(new audio_msg::msg::SmartAudioData());
  frame->frame_type.value = frame->frame_type.SMART_AUDIO_TYPE_CMD_WORD;
  frame->cmd_word = cmd_word;
  msg_publisher_->publish(std::move(frame));
  auto message = std::make_unique<std_msgs::msg::String>();
  message->data = cmd_word;
  asr_msg_publisher_->publish(std::move(message));
  strip.clear();
}

void HBAudioIo::AudioASRFunc(std::string asr) {
  if (asr.length() > 0) {
    RCLCPP_WARN(rclcpp::get_logger("audio_io"), "asr msg:%s", asr.c_str());
    if ((push_wakeup_) && (asr == wakeup_name_)) {
      auto message = std::make_unique<std_msgs::msg::String>();
      message->data = asr;
      RCLCPP_WARN(rclcpp::get_logger("audio_io"), "asr publish:%s", asr.c_str());
      asr_msg_publisher_->publish(std::move(message));
      strip.clear();
    }
    size_t pos = asr.find(wakeup_name_, 0);  
    size_t pos1 = asr.find(wakeup_name_1_, 0);      
    if (pos1 != std::string::npos) {
      if (pos1 < (asr.length() - wakeup_name_1_.length())) {
        std::string asr_msg;
        asr_msg.append(asr, pos1 + wakeup_name_1_.length(), asr.length() - pos1 - wakeup_name_1_.length());
        auto message = std::make_unique<std_msgs::msg::String>();
        message->data = asr_msg;
        RCLCPP_WARN(rclcpp::get_logger("audio_io"), "asr publish:%s", asr_msg.c_str());
        asr_msg_publisher_->publish(std::move(message));
        strip.clear();
      }
    } else if (pos != std::string::npos) {
      if (pos < (asr.length() - wakeup_name_.length())) {
        std::string asr_msg;
        asr_msg.append(asr, pos + wakeup_name_.length(), asr.length() - pos - wakeup_name_.length());
        auto message = std::make_unique<std_msgs::msg::String>();
        message->data = asr_msg;
        RCLCPP_WARN(rclcpp::get_logger("audio_io"), "asr publish:%s", asr_msg.c_str());
        asr_msg_publisher_->publish(std::move(message));
        strip.clear();
      }
    }
  }
}

//判断是否包含中文
static bool containsChinese(const std::string& str) {
    for (size_t i = 0; i < str.size(); ) {
        unsigned char c = static_cast<unsigned char>(str[i]);
        if (c >= '0' && c <= '9') return true;
        if (c >= 0xE4 && c <= 0xE9) {
            if (i + 2 < str.size()) {
                unsigned char c1 = static_cast<unsigned char>(str[i + 1]);
                unsigned char c2 = static_cast<unsigned char>(str[i + 2]);
                if ((c1 & 0xC0) == 0x80 && (c2 & 0xC0) == 0x80) {
                    return true;
                }
            }
            i += 3;
        } else if (c >= 0xC0) {
            i += 2;
        } else {
            i += 1;
        }
    }
    return false;
}

int HBAudioIo::PcmThread() {

  std::unique_ptr<float[]> pcm_data;
  int pcm_size;
  while (rclcpp::ok()) {
    // static bool inited = false;
    // if(inited == false){
    //   tts_data_queue_.push("你好");
    //   tts_data_queue_.push("end");
    //   std::cout<<"----------start----------"<<std::endl;
    //   std::unique_lock<std::mutex> micphone_lock(micphone_mtx_);
    //   speaker_working_ = true;
    //   micphone_lock.unlock();
    //   inited = true;
    // }
    // 等待大模型节点启动
    std::unique_lock<std::mutex> llm_node_init_lock(llm_node_init_mtx_);
    llm_node_init_cv_.wait(llm_node_init_lock, [this] { return llm_node_init_ || exiting_;});
    llm_node_init_lock.unlock();

    std::unique_lock<std::mutex> tts_queue_lock(tts_queue_mtx_);
    tts_queue_cv_.wait(tts_queue_lock, [this] { return !tts_data_queue_.empty() || exiting_;});
    if(exiting_ == true) break;
    tts_msg_ = tts_data_queue_.front();
    tts_data_queue_.pop();
    tts_queue_lock.unlock();
    if(rclcpp::ok()){
      std::cout<<"tts_msg_: "<<tts_msg_<<std::endl;
      if (containsChinese(tts_msg_)){
        auto audio = sherpa_tts_.tts_ptr_->Generate(tts_msg_, 0, 1.0f, nullptr);
        PlaybackItem p;
        p.samples = std::move(audio.samples);
        p.sample_rate = audio.sample_rate;
        p.playback = true;

        std::unique_lock<std::mutex> playback_queue_lock(playback_queue_mtx_);
        playback_queue_.push(p);
        playback_queue_lock.unlock();
        playback_queue_cv_.notify_one();
      }
      if(tts_msg_ == "end"){
        std::unique_lock<std::mutex> playback_queue_lock(playback_queue_mtx_);

        PlaybackItem p;
        p.playback = false;

        playback_queue_.push(p);
        playback_queue_lock.unlock();
        playback_queue_cv_.notify_one();
      }
    }

  }
  RCLCPP_WARN(rclcpp::get_logger("audio_io"), "stop speaker audio");
  return 0;
}

int HBAudioIo::SpeakerThread() {
  
  // sherpa_onnx::AlsaPlay alsa(micphone_name_.c_str(), 22050);
  while (rclcpp::ok()) {
    PlaybackItem item;
    std::unique_lock<std::mutex> playback_queue_lock(playback_queue_mtx_);
    playback_queue_cv_.wait(playback_queue_lock, [this] { return !playback_queue_.empty() || exiting_; });
    if(exiting_ == true) break;

    item = std::move(playback_queue_.front());

    playback_queue_.pop();
    playback_queue_lock.unlock();

    if(item.playback == false){
      std::cout<<"------------mic-------------"<<std::endl;
      std::unique_lock<std::mutex> micphone_lock(micphone_mtx_);
      speaker_working_ = false;
      micphone_lock.unlock();
      strip.set_all_same_color(0, 0, 255);
      micphone_cv_.notify_one();
      continue;
    }
    
    sherpa_onnx::AlsaPlay alsa(micphone_name_.c_str(), item.sample_rate > 0 ? item.sample_rate : sherpa_tts_.tts_ptr_->SampleRate());
    if (!item.samples.empty()) {
        alsa.Play(item.samples);
    }
    alsa.Drain();
  }
  return 0;
}


void HBAudioIo::CheckLLMNodeExistence()
{
  auto node_names_and_namespaces = this->get_node_names();
  std::set<std::string> current_nodes(node_names_and_namespaces.begin(), node_names_and_namespaces.end());
  static std::set<std::string> observed_nodes;

  std::string target_node_name = "/llamacpp_node";
  bool exists_now = current_nodes.find(target_node_name) != current_nodes.end();
  bool existed_before = observed_nodes.find(target_node_name) != observed_nodes.end();

  // 节点上线
  if (exists_now && !existed_before) {
      RCLCPP_WARN(this->get_logger(), "Node '%s' has come online!", target_node_name.c_str());
      std::unique_lock<std::mutex> llm_node_init_lock(llm_node_init_mtx_);
      llm_node_init_ = true;
      llm_node_init_lock.unlock();
      llm_node_init_cv_.notify_all();
  }

  // 节点下线
  if (!exists_now && existed_before) {
      RCLCPP_WARN(this->get_logger(), "Node '%s' has gone offline!", target_node_name.c_str());
      std::unique_lock<std::mutex> llm_node_init_lock(llm_node_init_mtx_);
      llm_node_init_ = false;
      llm_node_init_lock.unlock();
      llm_node_init_cv_.notify_all();
  }

  // 更新观察状态
  if (exists_now)
      observed_nodes.insert(target_node_name);
  else
      observed_nodes.erase(target_node_name);
}

void HBAudioIo::TTSMsgCallback(const std_msgs::msg::String::SharedPtr msg){
  {
    std::lock_guard<std::mutex> tts_queue_lock(tts_queue_mtx_);
    tts_data_queue_.push(msg->data);
  }
  std::unique_lock<std::mutex> micphone_lock(micphone_mtx_);
  speaker_working_ = true;
  micphone_lock.unlock();
  tts_queue_cv_.notify_one();
}

}  // namespace audio
}  // namespace hobot
