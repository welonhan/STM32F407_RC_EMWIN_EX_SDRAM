


2016.12.24: ���Ŷ����������
2016.12.26: ��ʼemWin
2016.12.27: �ο�emwin v1.2.0 STM324xG-EVAL
2016.12.29: 	
	DEMO��ʾ�������޸���LCD����0x3600�Ĵ�����������������
	�ж��ļ��У�����extern volatile GUI_TIMER_TIME OS_TimeMS;  ����TICK���������ʹOS_TimeMS++
2017.1.9
	GUI��������ʾ���Ŷ��ָ�룬2ͨ����ʾ��������Ϊ_DrawCh2�������
2017.1.10
	������ʾ���������ָ�����δ���ԭ�����ַ���������ʾ�ص�����д����ǰGUI_ClearRect()��ʾΪ������ɫ,�������˸
    ʹ��GUI_MEMDEV_CreateAuto���������洢�豸�����˸���⣬����ҪGUI_ClearRect()
2017.2.10
    ����GUI_NUMBYTES 100KB������RAMΪIRAM1
	WINDOW3 ֻ�ܲ�����ʾ����ʾ������ICONVIEW�򿪲˵�ֻ����ʾһ��
2017.2.15
	void BSP_LCD_FillRec(uint16_t xStart,uint16_t yStart,uint16_t xEnd,uint16_t yEnd,uint16_t Color)
	{
	uint32_t temp,dx,dy;
	if((xEnd>=xStart)&&(yEnd>=yStart))
	{
		dx=xEnd-xStart+1;				//���Ӵ˲���
		dy=yEnd-yStart+1;				//���Ӵ˲���
	
		BlockWrite(xStart,xEnd,yStart,yEnd);
		for (temp=0; temp<dx*dy; temp++)
		{
			*(__IO uint16_t *) (Bank1_LCD_D) = Color;
		}
	}
	}
2017.2.24
	window3 ��˸��WM_EnableMemdev(_RC_hWindow3);����Ϊ�洢���豸������˸
	
	
2017.2.27
	�ؼ����ͷţ��ڴ����ж���Ӧ�н������ͷŵĺ�������
	Touch_State.Pressed=TOUCH_Dat.pressed;	//δ������Ч���ݣ������ͷ�
	GUI_PID_StoreState(&Touch_State);
	
2017.3.2
	1��ͨ���ж��Ƿ���Ч�Ĵ��ھ����ˢ��ָ�룬�������������ʱ�򲿷�����ָ��ˢ��
			if( !((WM_IsWindow(_RC_hWindowConfig))|| \
					(WM_IsWindow(_RC_hWindowSystem))||\
						(WM_IsWindow(_RC_hWindowSave))))	
		_Dashboard(&_angle[0]);	
	2��ɾ������multipage��frameworkʱ����Ҫ��ɾ��page��Ȼ��ɾ��multipage����ɾ��framework���������������´�����frameworkʱ�������
	
2017.4.20
	1�SSD����д���󣬽���SD��Ƶ��