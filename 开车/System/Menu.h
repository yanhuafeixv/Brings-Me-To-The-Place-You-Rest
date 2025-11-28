#ifndef __MENU_H
#define __MENU_H

extern uint8_t menu_state;
extern uint8_t menu_index;
extern uint8_t start_flag;
void Menu_Init(void);
void Menu_Process(void);
//const char* main_items[] = {"Press", "PID", "SPEED"};


#endif