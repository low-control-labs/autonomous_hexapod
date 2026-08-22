#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cmath>
#include <algorithm>

class PCA9685Driver : public rclcpp::Node {
public:
    PCA9685Driver() : Node("pca9685_driver") {
        i2c_fd_ = open("/dev/i2c-1", O_RDWR);
        if (i2c_fd_ < 0) {
            RCLCPP_WARN(this->get_logger(), "Impossible to open /dev/i2c-1");
            return;
        }

        // Inizialize PCA9685
        initBoard(0x41); // Channels 0 - 15
        initBoard(0x40); // Channels 16 - 31

        sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/servo_cmd_deg", 10,
            std::bind(&PCA9685Driver::servoCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Driver Hardware PCA9685 (I2C 0x41 [0-15] & 0x40 [16-31]) ready.");
    }

    ~PCA9685Driver() {
        if (i2c_fd_ >= 0) {
            relaxAll();
            close(i2c_fd_);
            RCLCPP_INFO(this->get_logger(), "Servomotors released & bus I2C closed.");
        }
    }

private:
    int i2c_fd_ = -1;

    void writeReg(uint8_t addr, uint8_t reg, uint8_t val) {
        if (i2c_fd_ < 0) return;
        ioctl(i2c_fd_, I2C_SLAVE, addr);
        uint8_t buf[2] = {reg, val};
        write(i2c_fd_, buf, 2);
    }

    uint8_t readReg(uint8_t addr, uint8_t reg) {
        if (i2c_fd_ < 0) return 0;
        ioctl(i2c_fd_, I2C_SLAVE, addr);
        write(i2c_fd_, &reg, 1);
        uint8_t val = 0;
        read(i2c_fd_, &val, 1);
        return val;
    }

    void initBoard(uint8_t addr) {
        writeReg(addr, 0x00, 0x00); // Reset MODE1
        
        // PWM 50Hz (period 20000us)
        float prescaleval = (25000000.0f / (4096.0f * 50.0f)) - 1.0f;
        uint8_t prescale = static_cast<uint8_t>(std::floor(prescaleval + 0.5f));

        uint8_t oldmode = readReg(addr, 0x00);
        uint8_t newmode = (oldmode & 0x7F) | 0x10; // sleep mode for configuration
        writeReg(addr, 0x00, newmode);
        writeReg(addr, 0xFE, prescale);
        writeReg(addr, 0x00, oldmode);
        usleep(5000);
        writeReg(addr, 0x00, oldmode | 0x80); // Auto-increment on
    }

    void setPWM(uint8_t addr, uint8_t channel, uint16_t on, uint16_t off) {
        if (i2c_fd_ < 0) return;
        ioctl(i2c_fd_, I2C_SLAVE, addr);
        uint8_t reg = 0x06 + 4 * channel;
        uint8_t buf[5] = {
            reg,
            static_cast<uint8_t>(on & 0xFF),
            static_cast<uint8_t>(on >> 8),
            static_cast<uint8_t>(off & 0xFF),
            static_cast<uint8_t>(off >> 8)
        };
        write(i2c_fd_, buf, 5);
    }

    // remove power by setting bit FULL OFF (4096)
    void relaxAll() {
        for (uint8_t ch = 0; ch < 16; ++ch) {
            setPWM(0x41, ch, 4096, 4096);
            setPWM(0x40, ch, 4096, 4096);
        }
    }

    void servoCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (i2c_fd_ < 0) return;

        // scan up to 32 channels
        size_t total_channels = std::min(msg->data.size(), static_size_t(32));

        for (size_t ch = 0; ch < total_channels; ++ch) {
            double deg = std::clamp(msg->data[ch], 0.0, 180.0);
            
            // original conversion math Freenove: degrees -> Microseconds(500-2500us) -> Ticks PCA9685
            double pulse_us = 500.0 + (deg / 180.0) * 2000.0;
            uint16_t ticks = static_cast<uint16_t>((pulse_us / 20000.0) * 4095.0);

            // Mappping boards: 0-15 on 0x41, 16-31 on 0x40
            if (ch < 16) {
                setPWM(0x41, static_cast<uint8_t>(ch), 0, ticks);
            } else {
                setPWM(0x40, static_cast<uint8_t>(ch - 16), 0, ticks);
            }
        }
    }

    // Helper for secure cast dimensions
    static constexpr size_t static_size_t(size_t val) { return val; }

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PCA9685Driver>());
    rclcpp::shutdown();
    return 0;
}