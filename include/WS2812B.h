#include <iostream>
#include <vector>
#include <unistd.h>     // usleep
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <cstring>
#include <array>

class WS2812B {
public:
    WS2812B(int num_leds, const char* spi_device="/dev/spidev1.0", int spi_speed=2400000)
        : num_leds(num_leds), spi_speed(spi_speed)
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
        close(spi_fd);
    }

    void set_pixel(int index, uint8_t r, uint8_t g, uint8_t b) {
        if (index >=0 && index < num_leds)
            leds[index] = {r,g,b};
    }

    void set_all_same_color(uint8_t r, uint8_t g, uint8_t b) {
        for (auto &led : leds)
            led = {r,g,b};
        show();
    }

    void clear() {
        set_all_same_color(0,0,0);
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

    const std::vector<uint8_t> BIT_PAT_0 = {0b100, 0b000, 0b000};
    const std::vector<uint8_t> BIT_PAT_1 = {0b110, 0b000, 0b000};

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