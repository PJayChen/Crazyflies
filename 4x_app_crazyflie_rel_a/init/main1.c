#include "stm32f10x.h"

/*FreeRtos includes*/
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

#include "uart.h"

#include "nvicconf.h"
#include "config.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#pragma anon_unions  //允许加入匿名结构体
typedef union test{
	unsigned char header;
  struct{
  unsigned char channel     : 2;
  unsigned char reserved    : 2;
  unsigned char port        : 4;
	};
}Test;

int main(void)
{
	Test a;
	a.header = 0x00;
	a.port = 0xf;
	printf("header is %x",a.header);
	uartInit();
	while(1)
	{}
	//return 0;
}
