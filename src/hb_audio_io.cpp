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

namespace hobot {
namespace audio {
HBAudioIo::HBAudioIo(const std::string &node_name,
                               const NodeOptions &options)
    : rclcpp::Node(node_name, options) {
  
  std::string tros_distro
      = std::string(std::getenv("TROS_DISTRO")? std::getenv("TROS_DISTRO") : "");
  asr_model_path_ = "/opt/tros/" + tros_distro + "/lib/sensevoice_ros2/model/";
  //asr_model_path_ = "./install/lib/audio_io/model/";
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
  
  if (wakeup_name_.length() > 0) {
    wakeup_name_1_ = wakeup_name_ + ",";
  }

  int err_code = 0;
  tts_ =
      wetts_init(std::string("/opt/tros/" + tros_distro + "/lib/hobot_tts/tts_model").c_str(),
      "tts.flags", &err_code);
  struct audio_info info = wetts_audio_info(tts_);
  pcm_data_ = new char[info.max_len];

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

int HBAudioIo::Init() {
  v_cmd_word_ = std::make_shared<std::vector<std::string>>();
  std::ifstream cmd_word(cmd_word_path_);
  if (cmd_word.is_open()) {
    Json::Value root;
    cmd_word >> root;
    if (root.isMember("cmd_word") && root["cmd_word"].isArray()) {
      const Json::Value& cmdWords = root["cmd_word"];
      for (const auto& word : cmdWords) {
        std::cout << "命令词: " << word.asString() << std::endl;
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


  /* init speaker device*/
  speaker_device_ = alsa_device_allocate();
  if (!speaker_device_) {
    RCLCPP_ERROR(rclcpp::get_logger("audio_io"),
                 "open speaker device fail, ret=%d", ret);
  }
  speaker_device_->name = const_cast<char*>(micphone_name_.c_str());
  speaker_device_->format = SND_PCM_FORMAT_S16;
  speaker_device_->direct = SND_PCM_STREAM_PLAYBACK;
  speaker_device_->rate = 16000;
  speaker_device_->channels = 2;
  speaker_device_->buffer_time = 0;  // use default buffer time
  speaker_device_->nperiods = 4;
  speaker_device_->period_size = 512;  // 1 period including 1024 frames

  ret = alsa_device_init(speaker_device_);
  if (ret < 0) {
    if (speaker_device_) free(speaker_device_);
    RCLCPP_ERROR(rclcpp::get_logger("audio_io"),
                 "alsa device speaker init fail, ret=%d", ret);
  }

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
  is_init_ = true;
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

int HBAudioIo::ConvertToPCM(const std::string& msg,
                               std::unique_ptr<float[]>& pcm_data,
                               int& pcm_size) {
  auto err_code = wetts_synthesis(tts_, msg.c_str(), 1, pcm_data_, &pcm_size);
  if (err_code != ERRCODE_TTS_SUCC) {
    RCLCPP_ERROR(rclcpp::get_logger("audio_io"), "ConvertToPCM not init.");
    return -1;
  }

  pcm_data.reset(new float[pcm_size]);
  memcpy(pcm_data.get(), pcm_data_, pcm_size * sizeof(float));

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
  auto speaker_task = std::make_shared<std::thread>(
      std::bind(&HBAudioIo::SpeakerThread, this));

  // 创建并运行执行器
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(this->get_node_base_interface());
  exec.spin();
  // 退出时关闭线程
  if (capture_task && capture_task->joinable()) {
    capture_task->join();
    capture_task.reset();
  }
  if (speaker_task && speaker_task->joinable()) {
    get_tts_msg_ = true;
    tts_cv_.notify_one();
    speaker_task->join();
    speaker_task.reset();
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
  std::cout << "MicphoneGetThread------buffer_size:" << buffer_size << std::endl;
  char *buffer = new char[buffer_size];
  char *buffer_1 = new char[buffer_size];
  auto vec_ptr = std::make_shared<std::vector<double>>();
  int num = 0;
  while (rclcpp::ok()) {
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
}

void HBAudioIo::AudioASRFunc(std::string asr) {
  if (asr.length() > 0) {
    RCLCPP_WARN(rclcpp::get_logger("audio_io"), "asr msg:%s", asr.c_str());
    if ((push_wakeup_) && (asr == wakeup_name_)) {
      auto message = std::make_unique<std_msgs::msg::String>();
      message->data = asr;
      RCLCPP_WARN(rclcpp::get_logger("audio_io"), "asr publish:%s", asr.c_str());
      asr_msg_publisher_->publish(std::move(message));
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
      }
    } else if (pos != std::string::npos) {
      if (pos < (asr.length() - wakeup_name_.length())) {
        std::string asr_msg;
        asr_msg.append(asr, pos + wakeup_name_.length(), asr.length() - pos - wakeup_name_.length());
        auto message = std::make_unique<std_msgs::msg::String>();
        message->data = asr_msg;
        RCLCPP_WARN(rclcpp::get_logger("audio_io"), "asr publish:%s", asr_msg.c_str());
        asr_msg_publisher_->publish(std::move(message));
      }
    }
  }
}

int HBAudioIo::SpeakerThread() {
  RCLCPP_WARN(rclcpp::get_logger("audio_io"), "start to speaker audio");
  if (!speaker_device_) {
    RCLCPP_ERROR(rclcpp::get_logger("audio_io"), "speaker device is null");
    return -1;
  }
  while (rclcpp::ok()) {
    std::unique_ptr<float[]> pcm_data;
    int pcm_size;
    {
      std::unique_lock<std::mutex> lock(tts_mtx_);
      tts_cv_.wait(lock, [this] { return get_tts_msg_; });
      if(rclcpp::ok()){
        auto ret = ConvertToPCM(tts_msg_, pcm_data, pcm_size);
        playback_queue_.push(std::make_pair(std::move(pcm_data), pcm_size));
        while (!playback_queue_.empty()) {
          auto pcm_data = std::move(playback_queue_.front().first);
          auto pcm_size = playback_queue_.front().second;
          playback_queue_.pop();
          std::vector<int16_t> pcm_int16;
          auto pcm_float = pcm_data.get();
          for (int i = 0; i < pcm_size; i++) {
            pcm_int16.push_back(*pcm_float);
            pcm_int16.push_back(*pcm_float);
            pcm_float++;
          }
          if (speaker_device_) {
            snd_pcm_sframes_t frames = snd_pcm_bytes_to_frames(
                speaker_device_->handle, pcm_int16.size() * sizeof(int16_t));
            snd_pcm_prepare(speaker_device_->handle);  // 耗时0.1ms
            alsa_device_write(speaker_device_, pcm_int16.data(), frames);
            snd_pcm_drop(speaker_device_->handle);  // 耗时1ms
          }
        }
        get_tts_msg_ = false;
      }

    }
  }
  RCLCPP_WARN(rclcpp::get_logger("audio_io"), "stop speaker audio");
  return 0;
}

void HBAudioIo::TTSMsgCallback(const std_msgs::msg::String::SharedPtr msg){
  {
    std::lock_guard<std::mutex> lock(tts_mtx_);
    tts_msg_ = msg->data;
    get_tts_msg_ = true;
  }
  tts_cv_.notify_one();
}

}  // namespace audio
}  // namespace hobot
