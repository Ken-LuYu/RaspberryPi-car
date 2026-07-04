#include "pca9685.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <netdb.h>
#include <sys/types.h>
#include <time.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/tcp.h>
#include <wiringPi.h>
#include <softPwm.h>
#include <math.h>

#define PIN_BASE 300
#define MAX_PWM 4096
#define HERTZ 50
#define BUFSIZE 512
#define max(x,y) ((x)>(y)? (x):(y))
#define min(x,y) ((x)<(y)? (x):(y))

// 小车电机引脚
int PWMA = 1;
int AIN2 = 2;
int AIN1 = 3;
int PWMB = 4;
int BIN2 = 5;
int BIN1 = 6;

// 循迹传感器引脚定义
#define LEFT    23
#define RIGHT   25
#define B2      26
#define B3      27
#define B1      15
#define B4      16

// 自动模式定义
#define AUTO_MODE_STOP      0
#define AUTO_MODE_GOTO_END  1
#define AUTO_MODE_RETURN    2

// 节点处理状态机
#define NODE_STATE_IDLE     0
#define NODE_STATE_WAITING  1
#define NODE_STATE_PROCESS  2
#define NODE_STATE_LEAVING  3
// 新增：云台与截图等待状态
#define NODE_STATE_WAITING_SERVO 4  // 等待云台转动+视频流稳定
#define NODE_STATE_WAITING_CAPTURE 5 // 等待截图完成
// 控制模式：0=手动TCP控制，1=自动循迹模式
int control_mode = 0;

// 自动循迹相关变量
int auto_mode = AUTO_MODE_STOP;
int auto_point_num = 0;
int auto_node_cnt = 0;
int auto_node_lock = 0;
int auto_trace_running = 0;

// 云台自动扫描相关变量
float servo_h_init = 90.0f;
float servo_v_init = 0.0f;
int servo_v_step = 10;
int scan_total_rounds = 0;
int scan_current_round = 0;
float current_servo_v = 0.0f;

// 云台角度变量（输入值，与上位机一致）
float lr_detection = 90;   // 水平：0=左 90=中 180=右
float qh_detection = 90;    // 垂直：0=垂直向下（贴地面） 90=水平向前

// 非阻塞延时相关变量
unsigned long delay_start_time = 0;
unsigned long delay_duration = 0;
int delay_in_progress = 0;

// 节点处理状态变量
int node_state = NODE_STATE_IDLE;
int current_node_type = 0; // 0=去程, 1=回程
int current_sockfd = -1;
// 全局客户端socket
int global_client_fd = -1;

int calcTicks(float impulseMs, int hertz)
{
    float cycleMs = 1000.0f / hertz;
    return (int)(MAX_PWM * impulseMs / cycleMs + 0.5f);
}

/**
 * 已修复云台角度映射
 * 水平：输入0-180 → 实际舵机180-0（反转方向）
 * 垂直：输入0-90 → 实际舵机90-0（0=向下，90=向前）
 */
void PWM_write(int servonum, float x)
{
    float y;
    int tick;
    if (servonum == 1) { // 水平云台（1号舵机）
        x = 180.0f - x;
    }
    else if (servonum == 2) { // 垂直云台（2号舵机）
        x = 90.0f - x;
    }
    y = x / 90.0 + 0.5;
    y = max(y, 0.5);
    y = min(y, 2.5);
    tick = calcTicks(y, HERTZ);
    pwmWrite(PIN_BASE + servonum, tick);
}

void t_up(unsigned int speed, unsigned int t_time)
{
    digitalWrite(AIN2, 0);
    digitalWrite(AIN1, 1);
    softPwmWrite(PWMA, speed);
    digitalWrite(BIN2, 0);
    digitalWrite(BIN1, 1);
    softPwmWrite(PWMB, speed);
    if (t_time > 0) {
        delay_start_time = millis();
        delay_duration = t_time;
        delay_in_progress = 1;
    }
}

void t_stop(unsigned int t_time)
{
    digitalWrite(AIN2, 0);
    digitalWrite(AIN1, 0);
    softPwmWrite(PWMA, 0);
    digitalWrite(BIN2, 0);
    digitalWrite(BIN1, 0);
    softPwmWrite(PWMB, 0);
    if (t_time > 0) {
        delay_start_time = millis();
        delay_duration = t_time;
        delay_in_progress = 1;
    }
}

void t_down(unsigned int speed, unsigned int t_time)
{
    digitalWrite(AIN2, 1);
    digitalWrite(AIN1, 0);
    softPwmWrite(PWMA, speed);
    digitalWrite(BIN2, 1);
    digitalWrite(BIN1, 0);
    softPwmWrite(PWMB, speed);
    if (t_time > 0) {
        delay_start_time = millis();
        delay_duration = t_time;
        delay_in_progress = 1;
    }
}

void t_left(unsigned int speed, unsigned int t_time)
{
    digitalWrite(AIN2, 1);
    digitalWrite(AIN1, 0);
    softPwmWrite(PWMA, speed);
    digitalWrite(BIN2, 0);
    digitalWrite(BIN1, 1);
    softPwmWrite(PWMB, speed);
    if (t_time > 0) {
        delay_start_time = millis();
        delay_duration = t_time;
        delay_in_progress = 1;
    }
}

void t_right(unsigned int speed, unsigned int t_time)
{
    digitalWrite(AIN2, 0);
    digitalWrite(AIN1, 1);
    softPwmWrite(PWMA, speed);
    digitalWrite(BIN2, 1);
    digitalWrite(BIN1, 0);
    softPwmWrite(PWMB, speed);
    if (t_time > 0) {
        delay_start_time = millis();
        delay_duration = t_time;
        delay_in_progress = 1;
    }
}

// 非阻塞延时处理函数
int process_delay()
{
    if (delay_in_progress && (millis() - delay_start_time >= delay_duration)) {
        delay_in_progress = 0;
        return 1; // 延时完成
    }
    return 0; // 延时中
}

// 设置TCP_NODELAY彻底解决粘包问题
void set_tcp_nodelay(int sockfd)
{
    int flag = 1;
    if (setsockopt(sockfd, IPPROTO_TCP, TCP_NODELAY, (char*)&flag, sizeof(int)) < 0) {
        printf("[TCP] 设置TCP_NODELAY失败\n");
    }
    else {
        printf("[TCP] TCP_NODELAY已启用，彻底解决粘包问题\n");
    }
}

// 安全发送函数
void safe_send(int sockfd, const char* data)
{
    if (sockfd < 0) return;
    write(sockfd, data, strlen(data));
    usleep(100000); // 100ms延时确保数据包单独发送
    printf("[TCP] 发送：%s\n", data);
}

void send_node_data(int sockfd, int node_code)
{
    char buf[32];
    snprintf(buf, sizeof(buf), "NODE:0x%02X", node_code);
    safe_send(sockfd, buf);
}

void send_scan_status(int sockfd, const char* status)
{
    char buf[64];
    snprintf(buf, sizeof(buf), "SCAN:%s", status);
    safe_send(sockfd, buf);
}

// 发送自动截图指令
void send_capture_command(int sockfd, const char* node_name, float v_angle, float h_angle)
{
    char buf[128];
    snprintf(buf, sizeof(buf), "CAPTURE:AUTO:%s:%.1f:%.1f", node_name, v_angle, h_angle);
    safe_send(sockfd, buf);
}

void set_servo_angle(float h_angle, float v_angle)
{
    lr_detection = h_angle;
    qh_detection = v_angle;
    PWM_write(1, lr_detection);
    PWM_write(2, qh_detection);
    printf("[SERVO] 设置云台角度：水平%.1f°，垂直%.1f°\n", h_angle, v_angle);
}

// 最终修复版：仅第一轮初始化角度，后续轮次使用已计算好的值
void start_next_trace(int sockfd)
{
    if (scan_current_round >= scan_total_rounds) {
        send_scan_status(sockfd, "COMPLETE");
        printf("[SCAN] 所有扫描任务完成，小车停止\n");
        auto_trace_running = 0;
        auto_mode = AUTO_MODE_STOP;
        control_mode = 0;
        t_stop(0);
        return;
    }
    scan_current_round++;
    printf("[SCAN] 开始第%d次扫描，总次数：%d\n", scan_current_round, scan_total_rounds);

    // 仅在第一轮初始化角度，后续轮次使用process_*_node中已计算好的current_servo_v
    if (scan_current_round == 1) {
        current_servo_v = servo_v_init;
        set_servo_angle(servo_h_init, current_servo_v);
    }

    // 奇数次：去程；偶数次：回程
    if (scan_current_round % 2 == 1) {
        auto_mode = AUTO_MODE_GOTO_END;
        send_scan_status(sockfd, "GOING");
        printf("[SCAN] 第%d次：去程开始\n", scan_current_round);
    }
    else {
        auto_mode = AUTO_MODE_RETURN;
        send_scan_status(sockfd, "RETURNING");
        printf("[SCAN] 第%d次：回程开始\n", scan_current_round);
    }
    // 启动循迹
    auto_trace_running = 1;
    auto_node_cnt = 0;
    auto_node_lock = 0;
    node_state = NODE_STATE_IDLE;
    control_mode = 1;
}

// 处理去程节点（最终修复版）
void process_forward_node(int sockfd)
{
    if (auto_node_cnt == 1) {
        // 起点
        send_node_data(sockfd, 0x19);
        send_capture_command(sockfd, "起点", current_servo_v, servo_h_init);
        printf("[TRACK] 到达起点，计数：%d\n", auto_node_cnt);
    }
    else if (auto_node_cnt >= 2 && auto_node_cnt <= (auto_point_num + 1)) {
        // 途径点
        int point_num = auto_node_cnt - 1;
        send_node_data(sockfd, 0x20 + point_num);
        char node_name[32];
        snprintf(node_name, sizeof(node_name), "途径点%d", point_num);
        send_capture_command(sockfd, node_name, current_servo_v, servo_h_init);
        printf("[TRACK] 到达途径点%d，计数：%d\n", point_num, auto_node_cnt);
    }
    else if (auto_node_cnt == (auto_point_num + 2)) {
        // 终点
        send_node_data(sockfd, 0x20);
        send_capture_command(sockfd, "终点", current_servo_v, servo_h_init);
        printf("[TRACK] 到达终点，计数：%d\n", auto_node_cnt);

        // 检查是否需要进行下一轮扫描
        if (scan_current_round < scan_total_rounds) {
            // 计算下一轮的垂直角度
            float next_v = current_servo_v + servo_v_step;
            float new_v;
            if (fabs(next_v - 90.0f) < 0.001f) {
                new_v = 90.0f;
            }
            else if (next_v > 90.0f) {
                new_v = 85.0f;
            }
            else {
                new_v = next_v;
            }

            // 设置新的云台角度并同步更新全局变量
            set_servo_angle(servo_h_init, new_v);
            current_servo_v = new_v; // 同步更新全局角度
            printf("[SERVO] 准备下一轮扫描，设置云台至垂直%.1f°\n", new_v);

            // 进入云台+视频流稳定等待状态（3秒足够完成所有稳定过程）
            t_stop(3000);
            node_state = NODE_STATE_WAITING_SERVO;
            current_sockfd = sockfd;
            return; // 不执行离开节点操作
        }
        else {
            // 所有扫描完成
            auto_trace_running = 0;
            auto_node_cnt = 0;
            auto_node_lock = 0;
            send_scan_status(sockfd, "COMPLETE");
            printf("[SCAN] 所有扫描任务完成，小车停止\n");
            control_mode = 0;
            t_stop(0);
            node_state = NODE_STATE_IDLE;
            return;
        }
    }

    // 普通节点：离开节点，前进250ms
    t_up(35, 250);
    node_state = NODE_STATE_LEAVING;
}

// 处理回程节点（最终修复版）
void process_backward_node(int sockfd)
{
    if (auto_node_cnt == 1) {
        // 最后一个途径点
        send_node_data(sockfd, 0x20 + auto_point_num);
        char node_name[32];
        snprintf(node_name, sizeof(node_name), "途径点%d", auto_point_num);
        send_capture_command(sockfd, node_name, current_servo_v, servo_h_init);
        printf("[TRACK_BACK] 到达途径点%d，计数：%d\n", auto_point_num, auto_node_cnt);
    }
    else if (auto_node_cnt >= 2 && auto_node_cnt <= auto_point_num) {
        // 中间途径点
        int point_num = auto_point_num - (auto_node_cnt - 1);
        send_node_data(sockfd, 0x20 + point_num);
        char node_name[32];
        snprintf(node_name, sizeof(node_name), "途径点%d", point_num);
        send_capture_command(sockfd, node_name, current_servo_v, servo_h_init);
        printf("[TRACK_BACK] 到达途径点%d，计数：%d\n", point_num, auto_node_cnt);
    }
    else if (auto_node_cnt == (auto_point_num + 1)) {
        // 起点
        send_node_data(sockfd, 0x19);
        send_capture_command(sockfd, "起点", current_servo_v, servo_h_init);
        printf("[TRACK_BACK] 到达起点，计数：%d\n", auto_node_cnt);

        // 检查是否需要进行下一轮扫描
        if (scan_current_round < scan_total_rounds) {
            // 计算下一轮的垂直角度
            float next_v = current_servo_v + servo_v_step;
            float new_v;
            if (fabs(next_v - 90.0f) < 0.001f) {
                new_v = 90.0f;
            }
            else if (next_v > 90.0f) {
                new_v = 85.0f;
            }
            else {
                new_v = next_v;
            }

            // 设置新的云台角度并同步更新全局变量
            set_servo_angle(servo_h_init, new_v);
            current_servo_v = new_v; // 同步更新全局角度
            printf("[SERVO] 准备下一轮扫描，设置云台至垂直%.1f°\n", new_v);

            // 进入云台+视频流稳定等待状态（3秒足够完成所有稳定过程）
            t_stop(3000);
            node_state = NODE_STATE_WAITING_SERVO;
            current_sockfd = sockfd;
            return; // 不执行离开节点操作
        }
        else {
            // 所有扫描完成
            auto_trace_running = 0;
            auto_node_cnt = 0;
            auto_node_lock = 0;
            send_scan_status(sockfd, "COMPLETE");
            printf("[SCAN] 所有扫描任务完成，小车停止\n");
            control_mode = 0;
            t_stop(0);
            node_state = NODE_STATE_IDLE;
            return;
        }
    }

    // 普通节点：离开节点，后退250ms
    t_down(35, 250);
    node_state = NODE_STATE_LEAVING;
}

void trace_run_back(int sockfd)
{
    // 节点状态机处理（最终修复版）
    if (node_state != NODE_STATE_IDLE) {
        if (process_delay()) {
            if (node_state == NODE_STATE_WAITING) {
                // 3秒等待完成，开始处理节点
                node_state = NODE_STATE_PROCESS;
                process_backward_node(sockfd);
            }
            else if (node_state == NODE_STATE_LEAVING) {
                // 离开动作完成，回到空闲状态
                node_state = NODE_STATE_IDLE;
            }
            else if (node_state == NODE_STATE_WAITING_SERVO) {
                // 3秒稳定完成，发送第二张截图
                char node_name[32];
                if (auto_mode == AUTO_MODE_GOTO_END) {
                    snprintf(node_name, sizeof(node_name), "终点");
                }
                else {
                    snprintf(node_name, sizeof(node_name), "起点");
                }
                send_capture_command(current_sockfd, node_name, qh_detection, lr_detection);
                printf("[CAPTURE] 发送第二张截图指令：%s（垂直%.1f°）\n", node_name, qh_detection);

                // 进入截图等待状态（500ms确保截图写入完成）
                t_stop(500);
                node_state = NODE_STATE_WAITING_CAPTURE;
            }
            else if (node_state == NODE_STATE_WAITING_CAPTURE) {
                // 截图完成，开始下一轮扫描
                auto_node_cnt = 0;
                auto_node_lock = 0;
                start_next_trace(current_sockfd);
                node_state = NODE_STATE_IDLE;
            }
        }
        return;
    }
    int B2_val, B3_val, B1_val, B4_val;
    int is_node = 0;
    B2_val = digitalRead(B2);
    B3_val = digitalRead(B3);
    B1_val = digitalRead(B1);
    B4_val = digitalRead(B4);
    if (B2_val == HIGH && B3_val == HIGH && B1_val == HIGH && B4_val == HIGH) {
        is_node = 1;
    }
    if (is_node && auto_node_lock == 0 && auto_trace_running) {
        auto_node_lock = 1;
        auto_node_cnt++;
        // 先停止，再等待3秒
        t_stop(3000);
        node_state = NODE_STATE_WAITING;
        current_sockfd = sockfd;
        current_node_type = 1; // 回程
        printf("[TRACK_BACK] 到达节点，开始3秒稳定等待\n");
    }
    else if (!is_node) {
        auto_node_lock = 0;
    }
    if (auto_trace_running && node_state == NODE_STATE_IDLE) {
        if (B2_val == HIGH && B3_val == HIGH) {
            printf("[TRACK_BACK] BACK\n");
            t_down(25, 0);
        }
        else if (B2_val == LOW && B3_val == HIGH) {
            printf("[TRACK_BACK] RIGHT_CORRECT\n");
            t_right(35, 0);
        }
        else if (B2_val == HIGH && B3_val == LOW) {
            printf("[TRACK_BACK] LEFT_CORRECT\n");
            t_left(35, 0);
        }
        else {
            printf("[TRACK_BACK] STOP\n");
            t_stop(0);
        }
    }
}

void track_run(int sockfd)
{
    // 节点状态机处理（最终修复版）
    if (node_state != NODE_STATE_IDLE) {
        if (process_delay()) {
            if (node_state == NODE_STATE_WAITING) {
                // 3秒等待完成，开始处理节点
                node_state = NODE_STATE_PROCESS;
                process_forward_node(sockfd);
            }
            else if (node_state == NODE_STATE_LEAVING) {
                // 离开动作完成，回到空闲状态
                node_state = NODE_STATE_IDLE;
            }
            else if (node_state == NODE_STATE_WAITING_SERVO) {
                // 3秒稳定完成，发送第二张截图
                char node_name[32];
                if (auto_mode == AUTO_MODE_GOTO_END) {
                    snprintf(node_name, sizeof(node_name), "终点");
                }
                else {
                    snprintf(node_name, sizeof(node_name), "起点");
                }
                send_capture_command(current_sockfd, node_name, qh_detection, lr_detection);
                printf("[CAPTURE] 发送第二张截图指令：%s（垂直%.1f°）\n", node_name, qh_detection);

                // 进入截图等待状态（500ms确保截图写入完成）
                t_stop(500);
                node_state = NODE_STATE_WAITING_CAPTURE;
            }
            else if (node_state == NODE_STATE_WAITING_CAPTURE) {
                // 截图完成，开始下一轮扫描
                auto_node_cnt = 0;
                auto_node_lock = 0;
                start_next_trace(current_sockfd);
                node_state = NODE_STATE_IDLE;
            }
        }
        return;
    }
    int SR, SL;
    int is_node = 0;
    SR = digitalRead(RIGHT);
    SL = digitalRead(LEFT);
    if (SL == HIGH && SR == HIGH) {
        is_node = 1;
    }
    if (is_node && auto_node_lock == 0 && auto_trace_running) {
        auto_node_lock = 1;
        auto_node_cnt++;
        // 非首次去程计数修正
        if (scan_current_round > 1 && auto_node_cnt == 1) {
            auto_node_cnt = 2;
            printf("[TRACK] 非首次去程，自动修正计数：跳过起点，当前计数：%d\n", auto_node_cnt);
        }
        // 完全遵循原始代码逻辑：先停止，再等待3秒
        t_stop(3000);
        node_state = NODE_STATE_WAITING;
        current_sockfd = sockfd;
        current_node_type = 0; // 去程
        printf("[TRACK] 到达节点，开始3秒稳定等待\n");
    }
    else if (!is_node) {
        auto_node_lock = 0;
    }
    if (auto_trace_running && node_state == NODE_STATE_IDLE) {
        if (SL == LOW && SR == LOW) {
            printf("[TRACK] GO\n");
            t_up(25, 0);
        }
        else if (SL == HIGH && SR == LOW) {
            printf("[TRACK] LEFT\n");
            t_left(35, 0);
        }
        else if (SR == HIGH && SL == LOW) {
            printf("[TRACK] RIGHT\n");
            t_right(35, 0);
        }
        else {
            printf("[TRACK] STOP\n");
            t_stop(0);
        }
    }
}

typedef struct CLIENT {
    int fd;
    struct sockaddr_in addr;
}CLIENT;

int main(int argc, char* argv[])
{
    int sockfd;
    int listenfd;
    int connectfd;
    int ret;
    int maxfd = -1;
    struct timeval tv;
    struct sockaddr_in server_addr;
    struct sockaddr_in client_addr;
    socklen_t len;
    int portnumber;
    char buf[BUFSIZE];
    int z, i, maxi = -1;
    fd_set rset, allset;
    CLIENT client[FD_SETSIZE];

    wiringPiSetup();
    pinMode(1, OUTPUT);
    pinMode(2, OUTPUT);
    pinMode(3, OUTPUT);
    pinMode(4, OUTPUT);
    pinMode(5, OUTPUT);
    pinMode(6, OUTPUT);
    pinMode(LEFT, INPUT);
    pinMode(RIGHT, INPUT);
    pinMode(B2, INPUT);
    pinMode(B3, INPUT);
    pinMode(B1, INPUT);
    pinMode(B4, INPUT);
    softPwmCreate(PWMA, 0, 100);
    softPwmCreate(PWMB, 0, 100);

    int fd = pca9685Setup(PIN_BASE, 0x40, HERTZ);
    if (fd < 0)
    {
        printf("Error in setup\n");
        return fd;
    }
    pca9685PWMReset(fd);
    PWM_write(1, lr_detection);
    PWM_write(2, qh_detection);

    if (argc != 2)
    {
        printf("Please add portnumber!");
        exit(1);
    }
    if ((portnumber = atoi(argv[1])) < 0)
    {
        printf("Enter Error!");
        exit(1);
    }

    if ((listenfd = socket(PF_INET, SOCK_STREAM, 0)) == -1)
    {
        printf("Socket Error!");
        exit(1);
    }
    // 设置端口复用
    int opt = 1;
    if (setsockopt(listenfd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) == -1)
    {
        printf("setsockopt Error!\n");
        exit(1);
    }

    memset(&server_addr, 0, sizeof server_addr);
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    server_addr.sin_port = htons(portnumber);

    if ((bind(listenfd, (struct sockaddr*)(&server_addr), sizeof server_addr)) == -1)
    {
        printf("Bind Error!\n");
        exit(1);
    }

    if (listen(listenfd, 128) == -1)
    {
        printf("Listen Error!\n");
        exit(1);
    }

    for (i = 0; i < FD_SETSIZE; i++)
    {
        client[i].fd = -1;
    }
    FD_ZERO(&allset);
    FD_SET(listenfd, &allset);
    maxfd = listenfd;

    printf("waiting for the client's request...\n");

    while (1)
    {
        rset = allset;
        tv.tv_sec = 0;
        tv.tv_usec = 1000;
        ret = select(maxfd + 1, &rset, NULL, NULL, &tv);
        if (ret == 0)
        {
            if (control_mode == 1)
            {
                for (i = 0; i <= maxi; i++) {
                    if (client[i].fd > 0) {
                        if (auto_mode == AUTO_MODE_GOTO_END) {
                            track_run(client[i].fd);
                        }
                        else if (auto_mode == AUTO_MODE_RETURN) {
                            trace_run_back(client[i].fd);
                        }
                        break;
                    }
                }
            }
            continue;
        }
        else if (ret < 0)
        {
            printf("select failed!\n");
            break;
        }
        else
        {
            if (FD_ISSET(listenfd, &rset))
            {
                len = sizeof(struct sockaddr_in);
                if ((connectfd = accept(listenfd, (struct sockaddr*)(&client_addr), &len)) == -1)
                {
                    printf("accept() error\n");
                    continue;
                }
                for (i = 0; i < FD_SETSIZE; i++)
                {
                    if (client[i].fd < 0)
                    {
                        client[i].fd = connectfd;
                        client[i].addr = client_addr;
                        printf("Yout got a connection from %s\n", inet_ntoa(client[i].addr.sin_addr));
                        set_tcp_nodelay(connectfd);
                        global_client_fd = connectfd;
                        break;
                    }
                }
                if (i == FD_SETSIZE)
                    printf("Overfly connections\n");
                FD_SET(connectfd, &allset);
                if (connectfd > maxfd)
                    maxfd = connectfd;
                if (i > maxi)
                    maxi = i;
            }
            else
            {
                for (i = 0; i <= maxi; i++)
                {
                    if ((sockfd = client[i].fd) < 0)
                        continue;
                    if (FD_ISSET(sockfd, &rset))
                    {
                        bzero(buf, BUFSIZE + 1);
                        if ((z = read(sockfd, buf, sizeof buf)) > 0)
                        {
                            buf[z] = '\0';
                            printf("num = %d received data:%s\n", z, buf);

                            if (strncmp(buf, "SET", 3) == 0 && z == 4) {
                                auto_point_num = buf[3] - '0';
                                if (auto_point_num < 1 || auto_point_num > 9) {
                                    auto_point_num = 1;
                                }
                                printf("Set auto point num: %d\n", auto_point_num);
                                safe_send(sockfd, "SET_OK");
                                continue;
                            }

                            if (strncmp(buf, "SCAN:", 5) == 0) {
                                float h_init, v_init;
                                int step, total;
                                if (sscanf(buf, "SCAN:%f:%f:%d:%d", &h_init, &v_init, &step, &total) == 4) {
                                    servo_h_init = max(0.0f, min(180.0f, h_init));
                                    servo_v_init = max(0.0f, min(90.0f, v_init));
                                    servo_v_step = max(1, min(90, step));
                                    scan_total_rounds = max(1, total);
                                    scan_current_round = 0;
                                    printf("[SCAN] 收到扫描参数：水平%.1f°，垂直%.1f°，步长%d，总次数%d\n",
                                        servo_h_init, servo_v_init, servo_v_step, scan_total_rounds);
                                    safe_send(sockfd, "SCAN_OK");
                                    start_next_trace(sockfd);
                                }
                                else {
                                    safe_send(sockfd, "SCAN_ERR");
                                    printf("[SCAN] 指令格式错误：%s\n", buf);
                                }
                                continue;
                            }

                            if (strcmp(buf, "SCAN_STOP") == 0) {
                                // 立即停止所有动作和状态
                                auto_trace_running = 0;
                                auto_mode = AUTO_MODE_STOP;
                                control_mode = 0;
                                scan_current_round = 0;
                                scan_total_rounds = 0;
                                auto_node_cnt = 0;
                                auto_node_lock = 0;
                                node_state = NODE_STATE_IDLE;
                                delay_in_progress = 0;
                                t_stop(0);
                                send_scan_status(sockfd, "STOPPED");
                                printf("[SCAN] 自动扫描已停止，所有状态已重置\n");
                                continue;
                            }

                            if (strcmp(buf, "TR") == 0) {
                                if (auto_trace_running == 1) {
                                    printf("Auto trace already running, ignore\n");
                                    continue;
                                }
                                control_mode = 1;
                                auto_mode = AUTO_MODE_GOTO_END;
                                auto_trace_running = 1;
                                auto_node_cnt = 0;
                                auto_node_lock = 0;
                                node_state = NODE_STATE_IDLE;
                                printf("Start auto trace (GOTO END), point num: %d\n", auto_point_num);
                                safe_send(sockfd, "TR_START");
                                continue;
                            }

                            if (strcmp(buf, "ONF") == 0) {
                                control_mode = 1;
                                auto_mode = AUTO_MODE_GOTO_END;
                                auto_trace_running = 0;
                                printf("Switch to AUTO TRACK mode (GOTO END)\n");
                                continue;
                            }

                            if (strcmp(buf, "ONE") == 0) {
                                control_mode = 0;
                                auto_mode = AUTO_MODE_STOP;
                                auto_trace_running = 0;
                                scan_current_round = 0;
                                node_state = NODE_STATE_IDLE;
                                t_stop(0);
                                printf("Switch to MANUAL TCP mode\n");
                                continue;
                            }

                            if (strcmp(buf, "SERVO:RESET") == 0) {
                                lr_detection = 90.0f;
                                qh_detection = 90.0f;
                                PWM_write(1, lr_detection);
                                PWM_write(2, qh_detection);
                                printf("[SERVO] 云台复位成功！\n");
                                safe_send(sockfd, "SERVO_OK");
                                continue;
                            }
                            else if (strncmp(buf, "SERVO:", 6) == 0) {
                                char type[4], dir[10];
                                int angle;
                                if (sscanf(buf, "SERVO:%3[^:]:%d:%9[^:]", type, &angle, &dir) == 3) {
                                    angle = max(1, min(90, angle));
                                    printf("[SERVO] 类型：%s，角度：%d，方向：%s\n", type, angle, dir);
                                    if (strcmp(type, "H") == 0) {
                                        if (strcmp(dir, "LEFT") == 0) lr_detection -= angle;
                                        else if (strcmp(dir, "RIGHT") == 0) lr_detection += angle;
                                        lr_detection = max(0.0f, min(180.0f, lr_detection));
                                        PWM_write(1, lr_detection);
                                        printf("[SERVO] 水平角度：%.1f°\n", lr_detection);
                                    }
                                    else if (strcmp(type, "V") == 0) {
                                        if (strcmp(dir, "UP") == 0) qh_detection += angle;
                                        else if (strcmp(dir, "DOWN") == 0) qh_detection -= angle;
                                        qh_detection = max(0.0f, min(90.0f, qh_detection));
                                        PWM_write(2, qh_detection);
                                        printf("[SERVO] 垂直角度：%.1f°\n", qh_detection);
                                    }
                                    safe_send(sockfd, "SERVO_OK");
                                }
                                else {
                                    safe_send(sockfd, "SERVO_ERR");
                                    printf("[SERVO] 指令格式错误：%s\n", buf);
                                }
                                continue;
                            }

                            if (control_mode == 0)
                            {
                                if (z == 3 || z == 6)
                                {
                                    if (buf[0] == 'O' && buf[1] == 'N')
                                    {
                                        switch (buf[2])
                                        {
                                        case 'A':t_up(50, 0);      printf("forward\n"); break;
                                        case 'B':t_down(50, 0);    printf("back\n"); break;
                                        case 'C':t_left(50, 0);    printf("left\n"); break;
                                        case 'D':t_right(50, 0);   printf("right\n"); break;
                                        case 'E':t_stop(0);       printf("stop\n"); break;
                                        case 'L':lr_detection -= 10;
                                            if (lr_detection <= 0) lr_detection = 0;
                                            if (lr_detection >= 180) lr_detection = 180;
                                            PWM_write(1, lr_detection);  break;
                                        case 'I':lr_detection += 10;
                                            if (lr_detection <= 0) lr_detection = 0;
                                            if (lr_detection >= 180) lr_detection = 180;
                                            PWM_write(1, lr_detection);  break;
                                        case 'K':qh_detection += 10;
                                            if (qh_detection <= 0) qh_detection = 0;
                                            if (qh_detection >= 90) qh_detection = 90;
                                            PWM_write(2, qh_detection);  break;
                                        case 'J':qh_detection -= 10;
                                            if (qh_detection <= 0) qh_detection = 0;
                                            if (qh_detection >= 90) qh_detection = 90;
                                            PWM_write(2, qh_detection);  break;
                                        default: t_stop(0);       printf("stop\n"); break;
                                        }
                                    }
                                    else if (z == 6)
                                    {
                                        if (buf[2] == 0x00)
                                        {
                                            switch (buf[3])
                                            {
                                            case 0x01:t_up(50, 0); printf("forward\n"); break;
                                            case 0x02:t_down(50, 0);    printf("back\n"); break;
                                            case 0x03:t_left(50, 0);    printf("left\n"); break;
                                            case 0x04:t_right(50, 0);   printf("right\n"); break;
                                            case 0x00:t_stop(0);    printf("stop\n"); break;
                                            default: break;
                                            }
                                        }
                                        else
                                        {
                                            t_stop(0);
                                        }
                                    }
                                    else
                                    {
                                        t_stop(0);
                                    }
                                }
                                else
                                {
                                    t_stop(0);
                                }
                            }
                        }
                        else
                        {
                            printf("disconnected by client!\n");
                            close(sockfd);
                            FD_CLR(sockfd, &allset);
                            client[i].fd = -1;
                            global_client_fd = -1;
                            control_mode = 0;
                            auto_mode = AUTO_MODE_STOP;
                            auto_trace_running = 0;
                            auto_node_cnt = 0;
                            auto_node_lock = 0;
                            scan_current_round = 0;
                            scan_total_rounds = 0;
                            node_state = NODE_STATE_IDLE;
                            delay_in_progress = 0;
                            // 提示重启mjpg-streamer以清理缓冲区
                            printf("[INFO] 客户端已断开，下次连接前建议重启mjpg-streamer\n");
                        }
                    }
                }
            }
        }
    }
    close(listenfd);
    return 0;
}