#ifndef _CLIENT_H_
#define _CLIENT_H_
#include "main.h"

#define UI_RB     0   //红蓝主色
#define UI_YELLOW 1
#define UI_GREEN  2
#define UI_ORANGE 3
#define UI_PURPLE 4
#define UI_PINK   5
#define UI_CYAN   6   //青色
#define UI_BLACK  7
#define UI_WHITE  8

extern u8  draw_cnt;
extern float pitch_remain;
extern float Yaw_remain;
extern int robot_level_flag;

void Client_send_handle(void);
void delete_Coverage(u8 coverage);
#endif
