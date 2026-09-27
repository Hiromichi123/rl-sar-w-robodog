#pragma once
#include <cstdint>

//  Xbox手柄状态结构体（包含所有需要的按键）
struct GamepadState {
    // 肩键和特殊键
    bool R1 = false;    // 右肩键
    bool L1 = false;    // 左肩键
    bool start = false; // Start键
    bool back = false;  // Back键
    bool R2 = false;    // 右扳机（作为按键）
    bool L2 = false;    // 左扳机（作为按键）
    bool L3 = false;    // 左摇杆按压
    bool R3 = false;    // 右摇杆按压

    // 动作键
    bool A = false;     // A键
    bool B = false;     // B键
    bool X = false;     // X键
    bool Y = false;     // Y键

    // 方向键
    bool up = false;    // 上
    bool down = false;  // 下
    bool left = false;  // 左
    bool right = false; // 右

    // 摇杆值（-1.0~1.0，可选，根据需要添加）
    float left_x = 0.0f;
    float left_y = 0.0f;
    float right_x = 0.0f;
};

// Xbox手柄操作接口
class XboxGamepad {
private:
    void* joystick;      // SDL手柄句柄（隐藏实现细节）
    GamepadState state;  // 缓存当前状态
    bool connected;      // 连接状态

    // 死区过滤
    float deadzone(float value, float threshold = 0.1f) {
        return (value > threshold || value < -threshold) ? value : 0.0f;
    }

public:
    XboxGamepad(int index = 0);  // 构造函数（默认索引0）
    ~XboxGamepad();              // 析构函数

    void update();               // 更新手柄状态
    const GamepadState& getState() const { return state; }  // 获取状态
    bool isConnected() const { return connected; }          // 检查连接
};