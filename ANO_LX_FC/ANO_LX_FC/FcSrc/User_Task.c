#include "User_Task.h"
#include "Drv_RcIn.h"
#include "LX_FC_Fun.h"
#include "Drv_Uart.h" 
#include "ANO_LX.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#define RX_BUF_SIZE 64

extern int X;
extern int Y;
int flag = 0;



static char rx_buf[RX_BUF_SIZE];
static uint8_t rx_index = 0;
static uint8_t in_frame = 0;  


void U3GetOneByte(uint8_t data) {
 
    if (!in_frame && data != '<') {
        rx_index = 0;
        return;
    }

  
    if (!in_frame && data == '<') {
        in_frame = 1;
        rx_index  = 0;
        memset(rx_buf, 0, RX_BUF_SIZE);
        return;
    }


    if (in_frame) {
       
        if (data == '>') {
            rx_buf[rx_index] = '\0';
            int x_val = 0, y_val = 0;
            if (sscanf(rx_buf, "%d %d", &x_val, &y_val) == 2) {
                X = x_val;
                Y = y_val;

         
                switch (X) {
                  case 0x40:     //64
                    FC_Unlock();
                    break;

                  case 0x41:     //65 
                    if (Y=0x42)   //66
											{
											OneKey_Takeoff(85);
									X=0;
						      Y=0;		
											
                    }
                    break;
									case 0x43:      // 67
									{
									OneKey_Land();
									X=0;
						      Y=0;	
									}
                  break;
                
                  default:
                    break;
                }
            }
       
            in_frame = 0;
            rx_index = 0;
            memset(rx_buf, 0, RX_BUF_SIZE);
            return;
        }


        if (data != '\r' && data != '\n' && rx_index < RX_BUF_SIZE - 1) {
            rx_buf[rx_index++] = (char)data;
        } 
				else {
      
            in_frame = 0;
            rx_index = 0;
            memset(rx_buf, 0, RX_BUF_SIZE);
        }
    }
}


