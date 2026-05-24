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
#include <wiringPi.h>
#include <softPwm.h>

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

// 循迹传感器引脚
#define LEFT    23
#define RIGHT   25

// 控制模式：0=手动TCP控制，1=自动循迹模式（带途径点）
int control_mode = 0;
// 自动循迹相关变量
int auto_point_num = 0;    // 途径点数量
int auto_node_cnt = 0;     // 节点计数
int auto_node_lock = 0;    // 节点锁（防止重复计数）
int auto_trace_running = 0;// 自动循迹运行标志

// 云台角度变量（初始值）
float lr_detection = 90;   // 水平云台初始角度（中间位置）
float qh_detection = 0;    // 垂直云台初始角度（水平位置）

/**
 * Calculate the number of ticks the signal should be high for the required amount of time
 */
int calcTicks(float impulseMs, int hertz)
{
    float cycleMs = 1000.0f / hertz;
    return (int)(MAX_PWM * impulseMs / cycleMs + 0.5f);
}

void PWM_write(int servonum, float x)
{
    float y;
    int tick;
    y = x / 90.0 + 0.5;
    y = max(y, 0.5);
    y = min(y, 2.5);
    tick = calcTicks(y, HERTZ);
    pwmWrite(PIN_BASE + servonum, tick);
}

// 小车运动控制函数
void  t_up(unsigned int speed, unsigned int t_time)
{
    digitalWrite(AIN2, 0);
    digitalWrite(AIN1, 1);
    softPwmWrite(PWMA, speed);
    digitalWrite(BIN2, 0);
    digitalWrite(BIN1, 1);
    softPwmWrite(PWMB, speed);
    delay(t_time);
}

void t_stop(unsigned int t_time)
{
    digitalWrite(AIN2, 0);
    digitalWrite(AIN1, 0);
    softPwmWrite(PWMA, 0);
    digitalWrite(BIN2, 0);
    digitalWrite(BIN1, 0);
    softPwmWrite(PWMB, 0);
    delay(t_time);
}

void t_down(unsigned int speed, unsigned int t_time)
{
    digitalWrite(AIN2, 1);
    digitalWrite(AIN1, 0);
    softPwmWrite(PWMA, speed);
    digitalWrite(BIN2, 1);
    digitalWrite(BIN1, 0);
    softPwmWrite(PWMB, speed);
    delay(t_time);
}

void t_left(unsigned int speed, unsigned int t_time)
{
    digitalWrite(AIN2, 1);
    digitalWrite(AIN1, 0);
    softPwmWrite(PWMA, speed);
    digitalWrite(BIN2, 0);
    digitalWrite(BIN1, 1);
    softPwmWrite(PWMB, speed);
    delay(t_time);
}

void t_right(unsigned int speed, unsigned int t_time)
{
    digitalWrite(AIN2, 0);
    digitalWrite(AIN1, 1);
    softPwmWrite(PWMA, speed);
    digitalWrite(BIN2, 1);
    digitalWrite(BIN1, 0);
    softPwmWrite(PWMB, speed);
    delay(t_time);
}

// 发送节点数据到上位机
void send_node_data(int sockfd, int node_code)
{
    char buf[32];
    snprintf(buf, sizeof(buf), "NODE:0x%02X", node_code);
    write(sockfd, buf, strlen(buf));
    printf("[TCP] 发送节点数据：%s\n", buf);
}

// 自动循迹逻辑函数（已优化）
void track_run(int sockfd)
{
    int SR, SL;
    int is_node = 0;

    // 读取左右循迹传感器电平
    SR = digitalRead(RIGHT);
    SL = digitalRead(LEFT);

    // 节点检测：两个传感器都检测到黑线（HIGH=检测到）
    if (SL == HIGH && SR == HIGH) {
        is_node = 1;
    }

    // 节点处理逻辑
    if (is_node && auto_node_lock == 0 && auto_trace_running) {
        auto_node_lock = 1;
        auto_node_cnt++;

        // 节点处短暂停车
        t_stop(500);

        // 节点数据上报
        if (auto_node_cnt == 1) {
            // 起点：0x19
            send_node_data(sockfd, 0x19);
            printf("[TRACK] 到达起点，计数：%d\n", auto_node_cnt);
        }
        else if (auto_node_cnt >= 2 && auto_node_cnt <= (auto_point_num + 1)) {
            // 途径点：0x21-0x29（对应1-9个途径点）
            send_node_data(sockfd, 0x20 + (auto_node_cnt - 1));
            printf("[TRACK] 到达途径点%d，计数：%d\n", auto_node_cnt - 1, auto_node_cnt);
        }
        else if (auto_node_cnt == (auto_point_num + 2)) {
            // 终点：0x20
            send_node_data(sockfd, 0x20);
            printf("[TRACK] 到达终点，计数：%d\n", auto_node_cnt);
            // 到达终点后自动停止循迹，恢复手动模式
            auto_trace_running = 0;
            control_mode = 0;
            auto_node_cnt = 0;
            auto_node_lock = 0;
            t_stop(0);
            return;
        }
        t_up(35, 0); // 前进200ms，强制离开节点
        delay(250); // 节点停留时间
    }
    else if (!is_node) {
        auto_node_lock = 0; // 离开节点，解锁计数
    }

    // 普通循迹逻辑
    if (auto_trace_running) {
        if (SL == LOW && SR == LOW) {
            // 两个传感器都在黑线上 → 直行
            printf("[TRACK] GO\n");
            t_up(50, 0);
        }
        else if (SL == HIGH && SR == LOW) {
            // 左传感器离线，右传感器在线 → 左转修正
            printf("[TRACK] LEFT\n");
            t_left(50, 0);
        }
        else if (SR == HIGH && SL == LOW) {
            // 右传感器离线，左传感器在线 → 右转修正
            printf("[TRACK] RIGHT\n");
            t_right(50, 0);
        }
        else {
            // 两个传感器都离线 → 停止
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

    /*RPI初始化*/
    wiringPiSetup();

    /* 1. 小车电机引脚初始化 */
    pinMode(1, OUTPUT);	//PWMA
    pinMode(2, OUTPUT);	//AIN2
    pinMode(3, OUTPUT);	//AIN1
    pinMode(4, OUTPUT);	//PWMB
    pinMode(5, OUTPUT);	//BIN2
    pinMode(6, OUTPUT);    //BIN1

    /* 2. 循迹传感器引脚初始化 */
    pinMode(LEFT, INPUT);
    pinMode(RIGHT, INPUT);

    /*PWM输出初始化*/
    softPwmCreate(PWMA, 0, 100);
    softPwmCreate(PWMB, 0, 100);

    // 云台初始化
    int fd = pca9685Setup(PIN_BASE, 0x40, HERTZ);
    if (fd < 0)
    {
        printf("Error in setup\n");
        return fd;
    }
    pca9685PWMReset(fd);
    PWM_write(1, lr_detection);
    PWM_write(2, qh_detection);

    /* TCP服务端初始化 */
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
    memset(&server_addr, 0, sizeof server_addr);
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    server_addr.sin_port = htons(portnumber);
    if ((bind(listenfd, (struct sockaddr*)(&server_addr), sizeof server_addr)) == -1)
    {
        printf("Bind Error!");
        exit(1);
    }
    if (listen(listenfd, 128) == -1)
    {
        printf("Listen Error!");
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

    /* 主循环：处理TCP连接 + 模式切换 + 循迹逻辑 */
    while (1)
    {
        rset = allset;
        tv.tv_sec = 0;
        tv.tv_usec = 1000;

        // 1. 处理TCP网络事件
        ret = select(maxfd + 1, &rset, NULL, NULL, &tv);
        if (ret == 0)
        {
            // 无网络事件时，若处于循迹模式则执行循迹逻辑
            if (control_mode == 1)
            {
                // 遍历所有客户端，执行循迹（仅第一个客户端有效）
                for (i = 0; i <= maxi; i++) {
                    if (client[i].fd > 0) {
                        track_run(client[i].fd);
                        break;
                    }
                }
            }
            continue;
        }
        else if (ret < 0)
        {
            printf("select failed!");
            break;
        }
        else
        {
            // 处理新TCP连接
            if (FD_ISSET(listenfd, &rset))
            {
                len = sizeof(struct sockaddr_in);
                if ((connectfd = accept(listenfd, (struct sockaddr*)(&client_addr), &len)) == -1)
                {
                    printf("accept() error");
                    continue;
                }
                for (i = 0; i < FD_SETSIZE; i++)
                {
                    if (client[i].fd < 0)
                    {
                        client[i].fd = connectfd;
                        client[i].addr = client_addr;
                        printf("Yout got a connection from %s\n", inet_ntoa(client[i].addr.sin_addr));
                        break;
                    }
                }
                if (i == FD_SETSIZE)
                    printf("Overfly connections");
                FD_SET(connectfd, &allset);
                if (connectfd > maxfd)
                    maxfd = connectfd;
                if (i > maxi)
                    maxi = i;
            }
            else
            {
                // 处理已连接客户端的指令
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

                            // ========== 指令解析 ==========
                            // 设置途径点数量：SET1-SET9
                            if (strncmp(buf, "SET", 3) == 0 && z == 4) {
                                auto_point_num = buf[3] - '0';
                                if (auto_point_num < 1 || auto_point_num > 9) {
                                    auto_point_num = 1; // 默认1个途径点
                                }
                                printf("Set auto point num: %d\n", auto_point_num);
                                write(sockfd, "SET_OK", 6);
                                continue;
                            }
                            // 启动带途径点的自动循迹：TR（防止重复启动）
                            if (strcmp(buf, "TR") == 0) {
                                if (auto_trace_running == 1) {
                                    printf("Auto trace already running, ignore\n");
                                    continue;
                                }

                                control_mode = 1;
                                auto_trace_running = 1;
                                auto_node_cnt = 0;
                                auto_node_lock = 0;
                                printf("Start auto trace, point num: %d\n", auto_point_num);
                                write(sockfd, "TR_START", 8);
                                continue;
                            }
                            // 普通循迹模式：ONF
                            if (strcmp(buf, "ONF") == 0) {
                                control_mode = 1;
                                auto_trace_running = 0; // 无途径点计数
                                printf("Switch to AUTO TRACK mode\n");
                                continue;
                            }
                            // 手动模式：ONE
                            if (strcmp(buf, "ONE") == 0) {
                                control_mode = 0;
                                auto_trace_running = 0;
                                t_stop(0);
                                printf("Switch to MANUAL TCP mode\n");
                                continue;
                            }

                            // ========== 云台指令解析 ==========
                            // 1. 先处理复位指令
                            if (strcmp(buf, "SERVO:RESET") == 0) {
                                lr_detection = 90.0f;  // 水平复位到正中间
                                qh_detection = 0.0f;   // 垂直复位到水平位置
                                PWM_write(1, lr_detection);
                                PWM_write(2, qh_detection);
                                printf("[SERVO] 云台复位成功！水平：%.1f°，垂直：%.1f°\n", lr_detection, qh_detection);
                                write(sockfd, "SERVO_OK", 8);
                                continue;
                            }
                            // 2. 再处理普通角度指令
                            else if (strncmp(buf, "SERVO:", 6) == 0) {
                                char type[4], dir[10];
                                int angle;
                                if (sscanf(buf, "SERVO:%3[^:]:%d:%9[^:]", type, &angle, dir) == 3) {
                                    angle = max(1, min(90, angle));
                                    printf("[SERVO] 类型：%s，角度：%d，方向：%s\n", type, angle, dir);

                                    if (strcmp(type, "H") == 0) {
                                        if (strcmp(dir, "LEFT") == 0) lr_detection += angle;
                                        else if (strcmp(dir, "RIGHT") == 0) lr_detection -= angle;
                                        lr_detection = max(0.0f, min(180.0f, lr_detection));
                                        PWM_write(1, lr_detection);
                                        printf("[SERVO] 水平角度：%.1f°\n", lr_detection);
                                    }
                                    else if (strcmp(type, "V") == 0) {
                                        if (strcmp(dir, "UP") == 0) qh_detection += angle;
                                        else if (strcmp(dir, "DOWN") == 0) qh_detection -= angle;
                                        qh_detection = max(0.0f, min(180.0f, qh_detection));
                                        PWM_write(2, qh_detection);
                                        printf("[SERVO] 垂直角度：%.1f°\n", qh_detection);
                                    }
                                    write(sockfd, "SERVO_OK", 8);
                                }
                                else {
                                    write(sockfd, "SERVO_ERR", 8);
                                    printf("[SERVO] 指令格式错误：%s\n", buf);
                                }
                                continue;
                            }

                            // 原TCP控制逻辑（仅手动模式生效）
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
                                        case 'L':lr_detection += 10;
                                            if (lr_detection <= 0) lr_detection = 0;
                                            if (lr_detection >= 180) lr_detection = 180;
                                            PWM_write(1, lr_detection);  break;//左
                                        case 'I':lr_detection -= 10;
                                            if (lr_detection <= 0) lr_detection = 0;
                                            if (lr_detection >= 180) lr_detection = 180;
                                            PWM_write(1, lr_detection);  break;//右
                                        case 'K':qh_detection += 10;
                                            if (qh_detection <= 0) qh_detection = 0;
                                            if (qh_detection >= 180) qh_detection = 180;
                                            PWM_write(2, qh_detection);  break;//上
                                        case 'J':qh_detection -= 10;
                                            if (qh_detection <= 0) qh_detection = 0;
                                            if (qh_detection >= 180) qh_detection = 180;
                                            PWM_write(2, qh_detection);  break;//下
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
                            printf("disconnected by client!");
                            close(sockfd);
                            FD_CLR(sockfd, &allset);
                            client[i].fd = -1;
                            // 客户端断开后重置循迹状态
                            control_mode = 0;
                            auto_trace_running = 0;
                            auto_node_cnt = 0;
                            auto_node_lock = 0;
                        }
                    }
                }
            }
        }
    }
    close(listenfd);
    return 0;
}