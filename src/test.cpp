#include "sherpa_tts.h"

// ---------------- main ----------------
int main(int argc, char* argv[]) {
    SherpaTTS sherpa_tts;
    std::string device_name_ = "plughw:0,0";
    std::string config_path_ = "/root/TTS/matcha-icefall-zh-baker";
    sherpa_tts.Init(device_name_, config_path_);
    for(int i = 0; i < 3; i++){
        std::cout<<"----------"<<std::endl;
        std::string tts_msg_ = "截至2021年，广东省常住人口约为1.06亿人";
        auto audio = sherpa_tts.tts_ptr_->Generate(tts_msg_, 0, 1.0f, nullptr);
        PlaybackItem p;
        p.samples = std::move(audio.samples);
        p.sample_rate = audio.sample_rate;
        p.playback = true;
        sherpa_onnx::AlsaPlay alsa(device_name_.c_str(), p.sample_rate > 0 ? p.sample_rate : sherpa_tts.tts_ptr_->SampleRate());
        if (!p.samples.empty()) {
            printf("TTS sample count = %ld\n", p.samples.size());
            alsa.Play(p.samples);
        }
        alsa.Drain();
    }

    return 0;
}
