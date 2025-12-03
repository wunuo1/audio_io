#include "sherpa_tts.h"
#include <gpiod.hpp>
#include <iostream>

// ---------------- main ----------------
int main(int argc, char* argv[]) {
    // SherpaTTS sherpa_tts;
    // std::string device_name_ = "plughw:0,0";
    // std::string config_path_ = "/root/TTS/matcha-icefall-zh-baker";
    // sherpa_tts.Init(device_name_, config_path_);
    // for(int i = 0; i < 3; i++){
    //     std::cout<<"----------"<<std::endl;
    //     std::string tts_msg_ = "截至2021年，广东省常住人口约为1.06亿人";
    //     auto audio = sherpa_tts.tts_ptr_->Generate(tts_msg_, 0, 1.0f, nullptr);
    //     PlaybackItem p;
    //     p.samples = std::move(audio.samples);
    //     p.sample_rate = audio.sample_rate;
    //     p.playback = true;
    //     sherpa_onnx::AlsaPlay alsa(device_name_.c_str(), p.sample_rate > 0 ? p.sample_rate : sherpa_tts.tts_ptr_->SampleRate());
    //     if (!p.samples.empty()) {
    //         printf("TTS sample count = %ld\n", p.samples.size());
    //         alsa.Play(p.samples);
    //     }
    //     alsa.Drain();
    // }

    std::string gpio_path = "/sys/class/gpio/gpio401/value";
    int fd = open(gpio_path.c_str(), O_RDONLY);
    if (fd < 0) {
        perror("open");
        return false;
    }

    struct pollfd pfd;
    pfd.fd = fd;
    pfd.events = POLLPRI | POLLERR;

    // 先读一次清除状态
    char buf;
    read(fd, &buf, 1);

    int ret = poll(&pfd, 1, -1);
    if (ret > 0) {
        // 上升沿触发
        lseek(fd, 0, SEEK_SET);
        read(fd, &buf, 1);
        close(fd);
        std::cout<<"触发"<<std::endl;
    } else {
        // 超时或出错
        close(fd);
        std::cout<<"超时"<<std::endl;
    }

    return 0;
}
