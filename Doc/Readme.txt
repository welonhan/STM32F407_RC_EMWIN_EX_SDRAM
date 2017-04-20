


2016.12.24: 油门舵机工作正常
2016.12.26: 开始emWin
2016.12.27: 参考emwin v1.2.0 STM324xG-EVAL
2016.12.29: 	
	DEMO显示正常，修改了LCD驱动0x3600寄存器，触屏驱动坐标
	中断文件中，声明extern volatile GUI_TIMER_TIME OS_TimeMS;  并在TICK服务程序中使OS_TimeMS++
2017.1.9
	GUI可正常显示油门舵机指针，2通道显示错乱是因为_DrawCh2坐标错误
2017.1.10
	数字显示由于在数字更新是未清除原来的字符，导致显示重叠，在写数字前GUI_ClearRect()显示为背景颜色,会带来闪烁
    使用GUI_MEMDEV_CreateAuto（）创建存储设备解决闪烁问题，不需要GUI_ClearRect()
2017.2.10
    设置GUI_NUMBYTES 100KB，设置RAM为IRAM1
	WINDOW3 只能部分显示，显示花屏，ICONVIEW打开菜单只能显示一次
2017.2.15
	void BSP_LCD_FillRec(uint16_t xStart,uint16_t yStart,uint16_t xEnd,uint16_t yEnd,uint16_t Color)
	{
	uint32_t temp,dx,dy;
	if((xEnd>=xStart)&&(yEnd>=yStart))
	{
		dx=xEnd-xStart+1;				//增加此部分
		dy=yEnd-yStart+1;				//增加此部分
	
		BlockWrite(xStart,xEnd,yStart,yEnd);
		for (temp=0; temp<dx*dy; temp++)
		{
			*(__IO uint16_t *) (Bank1_LCD_D) = Color;
		}
	}
	}
2017.2.24
	window3 闪烁，WM_EnableMemdev(_RC_hWindow3);设置为存储化设备后不再闪烁
	
	
2017.2.27
	控件不释放，在触屏中断响应中将触摸释放的函数加上
	Touch_State.Pressed=TOUCH_Dat.pressed;	//未读到有效数据，触摸释放
	GUI_PID_StoreState(&Touch_State);
	
2017.3.2
	1、通过判断是否有效的窗口句柄再刷新指针，避免打开其他窗口时候部分区域被指针刷掉
			if( !((WM_IsWindow(_RC_hWindowConfig))|| \
					(WM_IsWindow(_RC_hWindowSystem))||\
						(WM_IsWindow(_RC_hWindowSave))))	
		_Dashboard(&_angle[0]);	
	2、删除含有multipage的framework时，需要先删除page，然后删除multipage，再删除framework，否则再重新重新创建该framework时候会死机
	
2017.4.20
	1SD卡读写错误，降低SD卡频率