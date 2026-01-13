#include <iostream>
#include <vector>
#include <unistd.h>     // usleep
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <cstring>
#include <array>
#include <thread>
#include <chrono>

enum class LightMode : uint8_t {
    Normal    = 0,  // 没有任何特效，只有亮暗
    Blinking  = 1,  // 闪烁
    Breathing = 2,  // 呼吸
    Thinking  = 3   // 思考
};

class WS2812B {
public:
    WS2812B(int num_leds, const char* spi_device="/dev/spidev1.0", int spi_speed=2400000)
        : num_leds(num_leds), spi_speed(spi_speed), lamp_effects_control(&WS2812B::LampEffectsFunc, this)
    {
        leds.resize(num_leds, {0,0,0});

        // 打开 SPI 设备
        spi_fd = open(spi_device, O_RDWR);
        if (spi_fd < 0) {
            perror("SPI open");
            exit(1);
        }

        uint8_t mode = 0;
        uint8_t bits = 8;
        if (ioctl(spi_fd, SPI_IOC_WR_MODE, &mode) < 0 ||
            ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0 ||
            ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &spi_speed) < 0) 
        {
            perror("SPI config");
            exit(1);
        }
    }

    ~WS2812B() {
        std::unique_lock<std::mutex> mode_control_lock(mode_control_mtx_);
        exiting_ = true;
        mode_control_cv_.notify_one();
        close(spi_fd);
    }

    void set_pixel(int index, uint8_t r, uint8_t g, uint8_t b, bool internal_use = false) {
        if(internal_use == false){
            std::unique_lock<std::mutex> mode_control_lock(mode_control_mtx_);
            light_state_ = LightMode::Normal;
            mode_control_cv_.notify_one();  
        }
        if (index >=0 && index < num_leds)
            leds[index] = {r,g,b};
        show();
    }

    void set_all_same_color(uint8_t r, uint8_t g, uint8_t b, bool internal_use = false) {
        if(internal_use == false){
            std::unique_lock<std::mutex> mode_control_lock(mode_control_mtx_);
            light_state_ = LightMode::Normal;
            mode_control_cv_.notify_one();
        }
        for (auto &led : leds)
            led = {r,g,b};
        show();
    }

    void set_lamp_effects(LightMode light_mode){
        std::unique_lock<std::mutex> mode_control_lock(mode_control_mtx_);
        light_state_ = light_mode;
        mode_control_cv_.notify_one();
    }

    void blink_blue(const int& time){
        {
            std::unique_lock<std::mutex> mode_control_lock(mode_control_mtx_);
            light_state_ = LightMode::Blinking;
            mode_control_cv_.notify_one();
        }
        for(int i = 0; i < time; i++){
            clear();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            set_all_same_color(0, 0, 255);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

    }

    void LampEffectsFunc(){
        int breathing_num = 0;
        bool decrease_flag = false;
        int thinking_num = 0;
        while(true){
            std::unique_lock<std::mutex> mode_control_lock(mode_control_mtx_);
            mode_control_cv_.wait(mode_control_lock, [this] { return light_state_ == LightMode::Breathing || light_state_ == LightMode::Thinking || exiting_;});
            auto light_state = light_state_;
            bool exiting = exiting_;
            mode_control_lock.unlock();
            if (exiting_ == true) break;
            switch (light_state){
                case LightMode::Breathing:
                    if (breathing_num < 0) decrease_flag = false;
                    if (breathing_num > 255) decrease_flag = true;
                    if(breathing_num >= 0 && breathing_num <= 255) set_all_same_color(0, 0, breathing_num, true);
                    if (decrease_flag == false) breathing_num+=10 ;
                    if (decrease_flag == true) breathing_num-=10;
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                    break;
                case LightMode::Thinking:
                    set_all_same_color(0, 0, 20, true);
                    set_pixel(thinking_num, 0, 0, 255, true);
                    std::this_thread::sleep_for(std::chrono::milliseconds(400));
                    thinking_num++;
                    if(thinking_num > 3) thinking_num = 0;
                    break;
                default:
                    break;
            }

        }
    }

    void clear() {
        set_all_same_color(0, 0, 0);
    }

    void show() {
        std::vector<uint8_t> spi_bytes;
        for (auto &led : leds) {
            // WS2812B 使用 GRB 顺序
            encode_byte(led[1], spi_bytes); // G
            encode_byte(led[0], spi_bytes); // R
            encode_byte(led[2], spi_bytes); // B
        }

        if (!spi_bytes.empty()) {
            if (write(spi_fd, spi_bytes.data(), spi_bytes.size()) < 0) {
                perror("SPI write");
            }
        }

        usleep(1000); // reset > 50us
    }

private:
    int num_leds;
    int spi_fd;
    int spi_speed;
    std::vector<std::array<uint8_t,3>> leds;
    std::thread lamp_effects_control; 
    const std::vector<uint8_t> BIT_PAT_0 = {0b100, 0b000, 0b000};
    const std::vector<uint8_t> BIT_PAT_1 = {0b110, 0b000, 0b000};

    bool exiting_ = false;
    std::mutex mode_control_mtx_;
    std::condition_variable mode_control_cv_;
    LightMode light_state_ = LightMode::Normal;

    void encode_byte(uint8_t byte, std::vector<uint8_t> &out) {
        for (int i = 7; i >= 0; --i) {
            int bit = (byte >> i) & 1;
            const std::vector<uint8_t> &pattern = bit ? BIT_PAT_1 : BIT_PAT_0;
            for (auto p : pattern) {
                out.push_back(p);
            }
        }
    }
};