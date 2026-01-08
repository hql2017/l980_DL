#include "canfestival.h"

unsigned int TimeCNT=0;//时间计数
unsigned int NextTime=0;//下一次触发时间计数
unsigned int TIMER_MAX_COUNT=90000;//最大时间计数
static TIMEVAL last_time_set = TIMEVAL_MAX;//上一次的时间计数
//Set the next alarm //
void setTimer(TIMEVAL value)
{
	NextTime=(TimeCNT+value)%TIMER_MAX_COUNT;
	printf("%d\r\n",NextTime);
}

// Get the elapsed time since the last occured alarm //
TIMEVAL getElapsedTime(void)
{
        int ret=0;
        ret = TimeCNT> last_time_set ? TimeCNT - last_time_set : TimeCNT + TIMER_MAX_COUNT - last_time_set;
        
        return ret;
}

void timerForCan(void)
{
        TimeCNT++;
        if (TimeCNT>=TIMER_MAX_COUNT)
        {
                TimeCNT=0;
        }
        if (TimeCNT==NextTime)
        {
                TimeDispatch();
        }
}

void TIM7_IRQHandler(void)
{
	if(TIM7->SR&0X0001)//中断
	{
		
	}
	TIM7->SR&=~(1<<0);//清除中断标志位	
	last_time_set = TimeCNT;
	timerForCan();
}



void TIM3_IRQHandler(void)
{ 		    		  			    
	if(TIM3->SR&0X0002)//溢出中断
	{		
		printf("%X\r\n",TIM3->SR);
	}				   
	TIM3->SR=0;//清除中断标志位 	   
}


