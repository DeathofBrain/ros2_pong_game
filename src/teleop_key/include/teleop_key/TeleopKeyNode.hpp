#include <iostream>
#include <string>

#include <unistd.h>
#include <termios.h>
#include <signal.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

// Ascii codes for small keyboard characters
// https://en.wikipedia.org/wiki/ASCII
#define KEYCODE_RIGHT 0x43
#define KEYCODE_LEFT 0x44
#define KEYCODE_UP 0x41
#define KEYCODE_DOWN 0x42
#define KEYCODE_W 0x77
#define KEYCODE_S 0x73
#define KEYCODE_B 0x62
#define KEYCODE_C 0x63
#define KEYCODE_D 0x64
#define KEYCODE_E 0x65
#define KEYCODE_F 0x66
#define KEYCODE_G 0x67
#define KEYCODE_Q 0x71
#define KEYCODE_R 0x72
#define KEYCODE_T 0x74
#define KEYCODE_V 0x76

/**
 * @brief 基于Linux系统下的键盘监听
 *
 */
class KeyBoardReader
{
private:
    termios new_settings{};
    termios stored_settings{};
    int kfd{0}; // 标准输入文件句柄

public:
    KeyBoardReader()
    {
        // 获取标准输入的终端属性，并存储在 stored_settings 结构体中
        tcgetattr(0, &stored_settings);
        // 复制 stored_settings 的内容到 new_settings 结构体中
        memcpy(&new_settings, &stored_settings, sizeof(termios));
        // 屏蔽整行缓存和本地回显功能，设置新行符、文件结束符
        new_settings.c_lflag &= ~(ICANON | ECHO);
        new_settings.c_cc[VEOL] = 1;
        new_settings.c_cc[VEOF] = 2;
        // 将标准输入的终端属性设置为 new_settings 的值，实现键盘监听
        tcsetattr(kfd, TCSANOW, &new_settings);
    }

    ~KeyBoardReader()
    {
        // 恢复标准输入终端属性
        shutdown();
    }

    void reader(char *c)
    {
        int rc = read(kfd, c, 1);

        // 读取失败
        if (rc < 0)
        {
            // 抛出异常
            throw std::runtime_error("读取键盘失败");
        }
    }

    void shutdown()
    {
        tcsetattr(kfd, TCSANOW, &stored_settings);
    }

    termios getStoredSettings()
    {
        return stored_settings;
    }
};

/**
 * @brief 键盘监听节点，仅供参考
 *
 */
class TeleopKeyNode : public rclcpp::Node
{
private:
    KeyBoardReader input;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_left;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_right;

public:
    TeleopKeyNode() : Node("TeleopKey")
    {
        pub_left = this->create_publisher<geometry_msgs::msg::Twist>("/left/cmd_vel", 10);
        pub_right = this->create_publisher<geometry_msgs::msg::Twist>("/right/cmd_vel", 10);

        auto msg = std::make_shared<geometry_msgs::msg::Twist>();

        // 无限循环，用来监听，但会阻塞该节点的主线程
        // 可以创建线程，用线程来监听，这样可以不阻塞主线程
        // 具体的线程创建方法请自行查阅资料
        std::thread{[&]
        {while (true)
        {
            char c;
            getKeyPress(&c);
            // c代表者按下键盘时的字符，通过c的值就可以确认按下的是哪个键
            // std::cout << c << '\n';
            // 一定要设置退出键，否则会无法退出，这里是q键
            if (c == KEYCODE_Q)
            {
                // 退出时一定要恢复控制台的标准输入，否则退出程序后控制台将无法输入任何字符
                this->quit();
            }
            // 在下面可以使用switch或if来对应每个键的作用
            switch (c)
            {
            case KEYCODE_UP:
                msg->linear.x = 3.0;
                pub_right->publish(*msg);
                break;

            case KEYCODE_DOWN:
                msg->linear.x = -3.0;
                pub_right->publish(*msg);
                break;

            case KEYCODE_W:
                msg->linear.x = 3.0;
                pub_left->publish(*msg);
                break;

            case KEYCODE_S:
                msg->linear.x = -3.0;
                pub_left->publish(*msg);
                break;
            default:
                break;
            }
        } }}
            .detach();
    }

    ~TeleopKeyNode()
    {
        input.shutdown();
    }

    void getKeyPress(char *c)
    {
        try
        {
            input.reader(c);
        }
        catch (const std::runtime_error &e)
        {
            std::cerr << e.what() << '\n';
        }
    }

    void quit()
    {
        input.shutdown();
        rclcpp::shutdown();
        exit(0);
    }
};
