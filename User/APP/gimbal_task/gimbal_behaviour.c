#include "gimbal_task.h"
#include "gimbal_behaviour.h"
#include "detect_task.h"
#include "user_lib.h"
#include "usart6.h"
#include "FreeRTOS.h"
#include "kalman.h"

//debug behaviour
int autoRampPara[14][2] ={{1,30},{3,60},{5,80},{7,130},{9,180},{12,240},{14,310},
																//pit��
												  {2,20},{4,60},{7,140},{10,200},{14,240},{16,320},{18,400}};
																//yaw��
extern int beijing_fla;													
/*******************************************DEBUGģʽ*******************************************/											
bool_t gimbalDebugBehaviourOutCondition()
{
	#ifdef DEBUG_GIMBAL
		return FALSE;
	#endif
	
	#ifndef DEBUG_GIMBAL
		return TRUE;
	#endif
}

bool_t gimbalDebugBehaviourEnterCondition()
{
	#ifdef DEBUG_GIMBAL
		return TRUE;
	#endif
	
	#ifndef DEBUG_GIMBAL
		return FALSE;
	#endif	
}

void gimbalDebugBehaviourHandleFun(float *yawExp, float *pitExp)
{
	static float yawInc=0,pitInc=0;
	if(toe_is_error(DBUSTOE))
	{
		*yawExp = 0;
		*pitExp = 0;   
		return;		
	}
	
	*yawExp = 0;
	*pitExp = 0;
	
	if(IF_RC_SW2_MID && IF_RC_SW1_MID)
	{
		*yawExp = (float)RC_CH2_LLR_OFFSET/660.0 * 60;
		*pitExp = (float)RC_CH3_LUD_OFFSET/660.0 * 40;
	}

}
/*******************************************DEBUGģʽ*******************************************/		



/*******************************************Midģʽ*******************************************/		
bool_t gimbalMidBehaviourOutCondition()
{
	#ifdef MID_GIMBAL
		return FALSE;
	#endif
	
	#ifndef MID_GIMBAL
		return TRUE;
	#endif
}
bool_t gimbalMidBehaviourEnterCondition()
{
	#ifdef MID_GIMBAL
		return TRUE;
	#endif
	
	#ifndef MID_GIMBAL
		return FALSE;
	#endif	
}

void gimbalMidBehaviourHandleFun(float *yawExp, float *pitExp)
{
	if(toe_is_error(DBUSTOE))
	{
		*yawExp = 0;
		*pitExp = 0;
		return;		
	}
	*yawExp	 = 0;
	*pitExp += RC_CH3_LUD_OFFSET/10000.0;
}
/*******************************************Midģʽ*******************************************/		



/*******************************************ZeroForceģʽ*******************************************/	

bool_t gimbalZeroForceBehaviourEnterCondition()
{

	if(toe_is_error(DBUSTOE) || toe_is_error(PITCH_MOTOR) || toe_is_error(YAW_MOTOR) || gimbalTaskStructure.robotMode == ROBOT_ZERO_FORCE || (*(gimbalTaskStructure.robotMode) == ROBOT_INIT && *(gimbalTaskStructure.robotModeStep) < ROBOT_INIT_GIMBAL))
	{
		return TRUE;
	}
	if( toe_is_error(DETECT_CHASSIS_CM1_MOTOR) || toe_is_error(DETECT_CHASSIS_CM2_MOTOR) || toe_is_error(DETECT_CHASSIS_CM3_MOTOR) || toe_is_error(DETECT_CHASSIS_CM4_MOTOR))
	{
		return TRUE;
	}
	return FALSE;
}

bool_t gimbalZeroForceBehaviourOutCondition()
{
	if(!gimbalZeroForceBehaviourEnterCondition())
	{
		gimbalBehaviourChange(gimbalTaskStructure.behaviorList + GIMBAL_INIT);
	}
	return FALSE;
}

void gimbalZeroForceBehaviourHandleFun(float *yawExp, float *pitExp)
{
	*yawExp = 0;
	*pitExp =RC_CH3_OFFSET/660.0*12;
}

/*******************************************ZeroForceģʽ*******************************************/



/*******************************************ZeroForceģʽ*******************************************/	
bool_t gimbalInitBehaviourEnterCondition()
{
	if(*(gimbalTaskStructure.robotMode) == ROBOT_INIT && *(gimbalTaskStructure.robotModeStep) == ROBOT_INIT_GIMBAL)
	{
		return TRUE;
	}
	return FALSE;
}

bool_t gimbalInitBehaviourOutCondition()
{
	if(gimbalTaskStructure.behaviourStep == 4)
	{
		gimbalTaskStructure.behaviourStep = 0;
		return TRUE;
	}
	return FALSE;
}


void gimbalInitBehaviourHandleFun(float *yawExp, float *pitExp)
{
	static s16 stepCorrectMs = 0; //��ĳһ�����������ʱ��
	static s16 stepErrMs = 0;
	if(gimbalTaskStructure.behaviourStep == 0)
	{
		*yawExp = gimbalTaskStructure.yawMotor.angle[ENCONDE];
		*pitExp = gimbalTaskStructure.pitchMotor.angle[ENCONDE];
		gimbalTaskStructure.behaviourStep++;
	}
	else if(gimbalTaskStructure.behaviourStep == 1)
	{ //����yaw����
		*pitExp = gimbalTaskStructure.pitchMotor.angle[ENCONDE];
		*yawExp = loop_ramp_float(0, *yawExp, 0.09*GIMBAL_TASK_MS, RELATIVE_YAW_MIN_RANGE, RELATIVE_YAW_MAX_RANGE);
		if(f_abs(gimbalTaskStructure.yawMotor.angle[ENCONDE]) < 3)
		{
			stepErrMs = 0;
			stepCorrectMs += GIMBAL_TASK_MS;
		}
		else
		{
			stepErrMs += GIMBAL_TASK_MS;
			stepCorrectMs = 0;
		}
		
		if(stepErrMs > 5000)
		{
			gimbalTaskStructure.behaviourStep = 0;
			stepErrMs = 0;
			stepCorrectMs = 0;
		}
		if(stepCorrectMs > 500)
		{
			gimbalTaskStructure.behaviourStep++;
			stepErrMs = 0;
			stepCorrectMs = 0;
		}
	}
	else if(gimbalTaskStructure.behaviourStep == 2)
	{
		*yawExp = 0;
		*pitExp = loop_ramp_float(0, *pitExp, 0.1*GIMBAL_TASK_MS, RELATIVE_PITCH_MIN_RANGE, RELATIVE_PITCH_MAX_RANGE);

		if(f_abs(gimbalTaskStructure.pitchMotor.angle[ENCONDE]) < 3)
		{
			stepErrMs = 0;
			stepCorrectMs += GIMBAL_TASK_MS;
		}
		else
		{
			stepErrMs += GIMBAL_TASK_MS;
			stepCorrectMs = 0;
		}		
		
		if(stepErrMs > 5000)
		{
			gimbalTaskStructure.behaviourStep = 0;
			stepErrMs = 0;
			stepCorrectMs = 0;
		}
		if(stepCorrectMs > 500)
		{
			gimbalTaskStructure.behaviourStep++;
			stepErrMs = 0;
			stepCorrectMs = 0;
		}
	}
	else if(gimbalTaskStructure.behaviourStep == 3)
	{
		*yawExp = 0;
		*pitExp = 0;
		if(*(gimbalTaskStructure.robotModeStep) == ROBOT_INIT_GIMBAL)
			(*(gimbalTaskStructure.robotModeStep)) = ROBOT_INIT_GIMBAL + 1;
		gimbalTaskStructure.behaviourStep = 4;
	}
	else if(gimbalTaskStructure.behaviourStep == 4)
	{
		*yawExp = 0;
		*pitExp = 0;
	}
}
/*******************************************ZeroForceģʽ*******************************************/	




/*******************************************Dropshotģʽ*******************************************/	
//bool_t gimbalDropshotBehaviourEnterCondition(void)
//{
//	if(IF_KEY_PRESSED_CTRL&&IF_KEY_PRESSED_E)
//	{
//		return TRUE;
//	}
//	else
//		return FALSE;
//}


//bool_t gimbalADropshotBehaviourOutCondition(void)
//{ 
//	if(IF_KEY_PRESSED_SHIFT&&IF_KEY_PRESSED_E)
//	{
//		return TRUE;
//	}
//	else
//		return FALSE;
//}


//void gimbalDropshotBehaviourHandleFun(float *yawExp, float *pitExp)
//{
//	float yawInc, pitInc;
//	yawInc = gimbalTaskStructure.yawMotor.offsetEcd;
//	pitInc = gimbalTaskStructure.pitchMotor.offsetEcd;
//	
//	if(IF_KEY_PRESSED_SHIFT&&IF_KEY_PRESSED_W)
//	{
//		pitInc += 100;
//	}
//	else if(IF_KEY_PRESSED_SHIFT&&IF_KEY_PRESSED_S)
//	{
//		pitInc -= 100;
//	}
//	
//  if(GIMBAL_FIRST_PRESS_SHIFT&&GIMBAL_FIRST_PRESS_A)
//	{
//		yawInc += 100;
//	}
//	else if(GIMBAL_FIRST_PRESS_CTRL&&GIMBAL_FIRST_PRESS_D)
//	{
//		yawInc -= 100;
//	}

//	gimbalTaskStructure.yawMotor.offsetEcd = yawInc;
//	gimbalTaskStructure.pitchMotor.offsetEcd = pitInc;
//}
/*******************************************Dropshotģʽ*******************************************/	



/*******************************************Normalģʽ*******************************************/	
bool_t gimbalNormalBehaviourEnterCondition()
{
	return TRUE;
}

bool_t gimbalNormalBehaviourOutCondition()
{
	
	return FALSE;
}


void gimbalNormalBehaviourHandleFun(float *yawExp, float *pitExp)
{
	//����ģʽ
	float yawInc, pitInc;
	
	if(beijing_fla==0)//�ޱ���
	{
	yawInc = 0.00005*4*MOUSE_X_MOVE_SPEED*10;
	pitInc = 0.00005*4*MOUSE_Y_MOVE_SPEED*10;
		
	}
 if(beijing_fla==1)//�б���
 {
	yawInc = 0.00005*MOUSE_X_MOVE_SPEED*10;
	pitInc = 0.00005*MOUSE_Y_MOVE_SPEED*10;
 }
 
	
	if(s16_abs(RC_CH2_LLR_OFFSET) > 20)
		yawInc += 0.001*RC_CH2_LLR_OFFSET;
	if(s16_abs(RC_CH3_LUD_OFFSET) > 20)
		pitInc = 0.0005*RC_CH3_LUD_OFFSET;
	
	if(GIMBAL_FIRST_CH2_MID && GIMBAL_FIRST_MOUSE_X_STOP)
		*yawExp = gimbalTaskStructure.yawMotor.angle[GYRO];
	else
		*yawExp += yawInc;
	
	if(GIMBAL_FIRST_CH3_MID && GIMBAL_FIRST_MOUSE_Y_STOP)
		*pitExp = gimbalTaskStructure.pitchMotor.angle[ENCONDE];
	else
     *pitExp -= pitInc;
	
//	//����ң��������
//	*pitExp=0;
//	*yawExp=0;

}
/*******************************************Normalģʽ*******************************************/	


///*******************************************Normalģʽ*******************************************/
///*

//bool_t gimbalAutoBehaviourEnterCondition()
//{  
//	if((IF_MOUSE_PRESSED_RIGH)||(IF_RC_SW1_MID&&GIMBAL_FIRST_S2_MID))
//		return TRUE;
//	else
//		return FALSE;
//}


//bool_t gimbalAutoBehaviourOutCondition()
//{
//	if((!IF_MOUSE_PRESSED_RIGH)||(GIMBAL_FIRST_PRESS_R)||(IF_RC_SW1_MID&&GIMBAL_FIRST_S2_MID))
//		return TRUE;
//	else
//		return FALSE;
//}


//extKalman_t usr_look1,usr_look2;


//void gimbalAutoBehaviourHandleFun(float *yawExp, float *pitExp)
//{
//	static float  yawInc = 0, pitInc = 0;
//	
//	//��¼��ǰϵͳʱ��
//	portTickType currentTime;
//	//��¼��һ��ϵͳʱ��
//	static portTickType currentTimeLast;
//	//��¼�Ӿ�����δ����ʱ��
//	static uint16_t notFindTargetTimeCount = 0;
//	//yaw��pitch�Ƕ��ݴ����
//  static float yawTemp ,pitTemp =0;
//	//�ҵ�Ŀ���־λ
//	static uint8_t findTargetFlag = 0;
//	//��ǰϵͳʱ���¼
//	currentTime = xTaskGetTickCount();
//	//��һ��ʱ���¼
//	currentTimeLast = currentTime;

//  /**************************************************************************************/
//	gimbalTaskStructure.Auto_Mode.yaw.err   = VisionRevData_Struct.pcResponseMsg.yawerr;
//	gimbalTaskStructure.Auto_Mode.pitch.err = VisionRevData_Struct.pcResponseMsg.piterr;
//	//�Ӿ������ĽǶȴ洢������ṹ����
//	gimbalTaskStructure.Auto_Mode.yaw.raw_exp   = gimbalTaskStructure.Auto_Mode.yaw.err + gimbalTaskStructure.imu->yaw;
//	gimbalTaskStructure.Auto_Mode.pitch.raw_exp = gimbalTaskStructure.Auto_Mode.pitch.err + gimbalTaskStructure.pitchMotor.angle[ENCONDE];
//  /**************************************************************************************/
//  //���Ӿ�����Ƶ�ʺʹ���Ƶ�ʲ�ͬ����ֹ������׷�ٹ���
//	if(VisionRevData_Struct.flag.amor_if_updata == TRUE)
//	{		
//		VisionRevData_Struct.flag.amor_if_updata = FALSE;
//		//���λ����
//		//�����ҵ���0������Ϊ�Ǹ����ͣ�0���Ϊ0.00��
//		if((fabs(gimbalTaskStructure.Auto_Mode.yaw.err) <= 0.3)		&& 
//			 (fabs(gimbalTaskStructure.Auto_Mode.pitch.err) <= 0.3) && 
//			 (VisionRevData_Struct.pcResponseMsg.fire_flag == 0))
//		{
//		  findTargetFlag = 0;    	
//		}
//		//û��ʶ��Ŀ��
//		else
//		{
//			findTargetFlag = 1;
//			//δʶ���ʱ������
//		}
//	}
//	
//		if((fabs(gimbalTaskStructure.Auto_Mode.yaw.err) <= 0.3)		&& 
//			 (fabs(gimbalTaskStructure.Auto_Mode.pitch.err) <= 0.3) && 
//			 (VisionRevData_Struct.pcResponseMsg.fire_flag == 0))
//			notFindTargetTimeCount++;	//δʶ���ʱ���ۼ�
//		else
//			notFindTargetTimeCount = 0;
///**************************************************************************************/
////													�м���ϽǶ��˲�����
////

//		
//	
//	if(findTargetFlag)
//	{
//		if((fabs(VisionRevData_Struct.pcResponseMsg.piterr) >= autoRampPara[0][0])&&(fabs(VisionRevData_Struct.pcResponseMsg.piterr) < autoRampPara[1][0]))
//			pitTemp = RAMP_float(	gimbalTaskStructure.Auto_Mode.pitch.raw_exp, *pitExp, autoRampPara[0][1]/500.0);
//		else if((fabs(VisionRevData_Struct.pcResponseMsg.piterr) >= autoRampPara[1][0])&&(fabs(VisionRevData_Struct.pcResponseMsg.piterr) < autoRampPara[2][0]))
//			pitTemp = RAMP_float(	gimbalTaskStructure.Auto_Mode.pitch.raw_exp, *pitExp, autoRampPara[1][1]/500.0);
//		else if((fabs(VisionRevData_Struct.pcResponseMsg.piterr) >= autoRampPara[2][0])&&(fabs(VisionRevData_Struct.pcResponseMsg.piterr) < autoRampPara[3][0]))
//			pitTemp = RAMP_float(	gimbalTaskStructure.Auto_Mode.pitch.raw_exp, *pitExp, autoRampPara[2][1]/500.0);
//		else if((fabs(VisionRevData_Struct.pcResponseMsg.piterr) >= autoRampPara[3][0])&&(fabs(VisionRevData_Struct.pcResponseMsg.piterr) < autoRampPara[4][0]))
//			pitTemp = RAMP_float(	gimbalTaskStructure.Auto_Mode.pitch.raw_exp, *pitExp, autoRampPara[3][1]/500.0);
//		else if((fabs(VisionRevData_Struct.pcResponseMsg.piterr) >= autoRampPara[4][0])&&(fabs(VisionRevData_Struct.pcResponseMsg.piterr) < autoRampPara[5][0]))
//			pitTemp = RAMP_float(	gimbalTaskStructure.Auto_Mode.pitch.raw_exp, *pitExp, autoRampPara[4][1]/500.0);
//		else if((fabs(VisionRevData_Struct.pcResponseMsg.piterr) >= autoRampPara[5][0])&&(fabs(VisionRevData_Struct.pcResponseMsg.piterr) < autoRampPara[6][0]))
//			pitTemp = RAMP_float(	gimbalTaskStructure.Auto_Mode.pitch.raw_exp, *pitExp, autoRampPara[5][1]/500.0);
//		else
//			pitTemp = RAMP_float(	gimbalTaskStructure.Auto_Mode.pitch.raw_exp, *pitExp, autoRampPara[6][1]/500.0);

//		if((fabs( VisionRevData_Struct.pcResponseMsg.yawerr) >= autoRampPara[7][0])&&(fabs( VisionRevData_Struct.pcResponseMsg.yawerr) < autoRampPara[8][0]))
//			yawTemp = RAMP_float(gimbalTaskStructure.Auto_Mode.yaw.raw_exp, *yawExp, autoRampPara[7][1]/500.0);
//		else if((fabs( VisionRevData_Struct.pcResponseMsg.yawerr) >= autoRampPara[8][0])&&(fabs( VisionRevData_Struct.pcResponseMsg.yawerr) < autoRampPara[9][0]))
//			yawTemp = RAMP_float(gimbalTaskStructure.Auto_Mode.yaw.raw_exp, *yawExp, autoRampPara[8][1]/500.0);
//		else if((fabs( VisionRevData_Struct.pcResponseMsg.yawerr) >= autoRampPara[9][0])&&(fabs( VisionRevData_Struct.pcResponseMsg.yawerr) < autoRampPara[10][0]))
//			yawTemp = RAMP_float(gimbalTaskStructure.Auto_Mode.yaw.raw_exp, *yawExp, autoRampPara[9][1]/500.0);
//		else if((fabs( VisionRevData_Struct.pcResponseMsg.yawerr) >= autoRampPara[10][0])&&(fabs( VisionRevData_Struct.pcResponseMsg.yawerr)< autoRampPara[11][0]))
//			yawTemp = RAMP_float(gimbalTaskStructure.Auto_Mode.yaw.raw_exp, *yawExp, autoRampPara[10][1]/500.0);
//		else if((fabs( VisionRevData_Struct.pcResponseMsg.yawerr) >= autoRampPara[11][0])&&(fabs( VisionRevData_Struct.pcResponseMsg.yawerr) < autoRampPara[12][0]))
//		  yawTemp = RAMP_float(gimbalTaskStructure.Auto_Mode.yaw.raw_exp, *yawExp, autoRampPara[11][1]/500.0);
//		else if((fabs( VisionRevData_Struct.pcResponseMsg.yawerr) >= autoRampPara[12][0])&&(fabs( VisionRevData_Struct.pcResponseMsg.yawerr) < autoRampPara[13][0]))
//			yawTemp = RAMP_float(gimbalTaskStructure.Auto_Mode.yaw.raw_exp, *yawExp, autoRampPara[12][1]/500.0);
//		else
//			yawTemp = RAMP_float(gimbalTaskStructure.Auto_Mode.yaw.raw_exp, *yawExp, autoRampPara[13][1]/500.0);
//	}
///**************************************************************************************/

//	
//	if(notFindTargetTimeCount < 150)
//	// ˵�����ڿ����Ѿ��ҵ�Ŀ��, ����Ƕȿ���
//	{
//		*yawExp = yawTemp;
//		*pitExp = pitTemp;
//	}
//	else
//	//˵��û���ҵ�Ŀ��,ң�������߼��̿���
//	{
//		
//		//�ȸ�ֵ,Ϊ���������ռ�
////		yawInc = 0.006*MOUSE_X_MOVE_SPEED;
////		pitInc = -0.006*MOUSE_Y_MOVE_SPEED;
////		
////		if(s16_abs(RC_CH2_LLR_OFFSET) > 20)
////			yawInc += 0.001*RC_CH2_LLR_OFFSET;
////		if(s16_abs(RC_CH3_LUD_OFFSET) > 20)
////			pitInc += 0.0005*RC_CH3_LUD_OFFSET;
//		
//    //		if(GIMBAL_FIRST_CH2_MID && GIMBAL_FIRST_MOUSE_X_STOP)
//		
//			*yawExp = gimbalTaskStructure.yawMotor.angle[GYRO]+0.4*MOUSE_X_MOVE_SPEED;
//		//		else
//    //		*yawExp = yawInc+ 0.38*MOUSE_Y_MOVE_SPEED;	
//		  *pitExp = gimbalTaskStructure.pitchMotor.angle[ENCONDE] + (-0.08)*MOUSE_Y_MOVE_SPEED;
//	}
//	
////		//Ŀ���ٶȽ��� ���������Ƕ�׵�
////	//	if(VisionRevData_Struct.pcResponseMsg.pcResponseCatch.fire_flag==TRUE){//ʶ��Ŀ��
////		if(1){
////			No_catchFlag=0;//���λ����
//////			gimbalTaskStructure.Auto_Mode.yaw.target_speed  =Target_Speed_Calc(&gimbalTaskStructure.Auto_Mode.yaw.speed_struct,xTaskGetTickCount(),gimbalTaskStructure.Auto_Mode.yaw.raw_exp);
//////			gimbalTaskStructure.Auto_Mode.pitch.target_speed=Target_Speed_Calc(&gimbalTaskStructure.Auto_Mode.pitch.speed_struct,xTaskGetTickCount(),gimbalTaskStructure.Auto_Mode.pitch.raw_exp);
//////			//�ԽǶȺ��ٶȽ��ж��׿������˲��ں�,0λ��,1�ٶ�
//////			gimbalTaskStructure.Auto_Mode.yaw.kf_result= kalman_filter_calc(&gimbalTaskStructure.Auto_Mode.yaw.kalman_filter, gimbalTaskStructure.Auto_Mode.yaw.raw_exp, gimbalTaskStructure.Auto_Mode.yaw.target_speed);
//////			gimbalTaskStructure.Auto_Mode.pitch.kf_result= kalman_filter_calc(&gimbalTaskStructure.Auto_Mode.pitch.kalman_filter, gimbalTaskStructure.Auto_Mode.pitch.raw_exp, gimbalTaskStructure.Auto_Mode.pitch.target_speed);
////		}
////		else{//ʶ�𲻵�Ŀ��ʱ���Ե�ǰֵΪ��������ֹʧ��
//////			No_catchFlag++;//��¼δʶ�𵽴���
//////			gimbalTaskStructure.Auto_Mode.yaw.target_speed  =Target_Speed_Calc(&gimbalTaskStructure.Auto_Mode.yaw.speed_struct,xTaskGetTickCount(),gimbalTaskStructure.imu->yaw);
//////			gimbalTaskStructure.Auto_Mode.pitch.target_speed=Target_Speed_Calc(&gimbalTaskStructure.Auto_Mode.pitch.speed_struct,xTaskGetTickCount(),gimbalTaskStructure.imu->pit);
//////			//�ԽǶȺ��ٶȽ��ж��׿������˲��ں�,0λ��,1�ٶ�
//////			gimbalTaskStructure.Auto_Mode.yaw.kf_result= kalman_filter_calc(&gimbalTaskStructure.Auto_Mode.yaw.kalman_filter, gimbalTaskStructure.imu->yaw, 0);
//////			gimbalTaskStructure.Auto_Mode.pitch.kf_result= kalman_filter_calc(&gimbalTaskStructure.Auto_Mode.pitch.kalman_filter, gimbalTaskStructure.imu->pit,0);
////		}
////}
///*******************************************Normalģʽ*******************************************/	
