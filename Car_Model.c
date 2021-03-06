#include "Car_Model.h"
#include "malloc.h"
#include "stdio.h"
#include "string.h"
#include "task.h"
#include "sys.h"
#include "delay.h"
#include "math.h"
#include "APP_MALLOC.h"
#include "VMsgHandler.h"

SemaphoreHandle_t BinarySemaphore;

float Decimals=0.001;

Agv_ModCfg 		agv_modcfg;
Agv_Para 		  agv_para;
Para_of_C 	  para_from_C;
Agv_Odo 		  agv_odo;
struct VT_Calc 		  vt_calc;
struct VT_Real 		  vt_real;

//角度归一化
static void Th_Normal(float *Th){
	if(*Th>PI){
		*Th=*Th-2*PI;
	}else if(*Th<PI){
		*Th=*Th+2*PI;
	}
}

//输出限幅
static void Limit(float *para,float max,float min)
{
	if (*para>max){
		*para=max;
	}else if(*para<min){
		*para=min;
	}
}


/*---------------------------------------------------------------------------
单舵轮VW解算
-----------------------------------------------------------------------------*/
struct VT_Calc MathMod_Proc1(Para_of_C CP , Agv_Para agv, struct VT_Real Driver)      
{
	struct VT_Calc math_out;
	float Vec,Theta;         //本模块的结果，舵轮的速度和角度
	float Vx_temp,Vy_temp;   //运动中心到舵轮的牵连速度	
 	float rH,rL,rD;
	rH=agv.rH;   //单舵轮到运动中心的偏置距离
	rL=agv.rL;	 //单舵轮到运动中心的轴线距离
	rD=sqrt(rH*rH+rL*rL);
	
	if ((CP.VW_Type==1) || (CP.VW_Type==4))
		rH=-1*agv.rH;
	else
		rH=agv.rH;	
	
	if (CP.Cir_Flag!=4) //非原地旋转
	{
		Vx_temp=CP.Vx + rH*CP.W;
		Vy_temp=rL * CP.W;
		if (CP.Vx!=0){
			Vec=CP.Vx/fabs(CP.Vx) * sqrt(Vx_temp*Vx_temp + Vy_temp*Vy_temp);
		}else{
			Vec=0;
		}
		if(Vec!=0){
			Theta=asin(Vy_temp/Vec);
		}
	}else{                       //原地旋转
		if(rD!=0){
			Theta=asin(rL/rD);
		}
		if(fabs(Theta-vt_real.Th1)<0.25){
			Vec=rL*CP.W;
		}else{
			Vec=0;
		}
	}
	Th_Normal(&Theta);
	
	math_out.V1 = Vec;
	math_out.Th1 = Theta;
	
	return math_out;
}


/*---------------------------------------------------------------------------
双舵轮VW解算,【全向运动】
-----------------------------------------------------------------------------*/
struct VT_Calc MathMod_Proc2(Para_of_C CP, Agv_Para agv, struct VT_Real Driver, int Offset_Dir)      
{
	_Bool bWalk, bOnce, bSwing;
	struct VT_Calc math_out;
	float Vec1,Vec2,Theta1,Theta2;         //本模块的结果，两个舵轮的速度和角度
	int Vmx, Vmy, Wm;                      //以舵轮连线为x轴，速度分解合成
	int V1_tmp, V2_tmp;
	
	float Length = agv.rD_FrontBack/2;
	float Offset = agv.rD_TurnRight/2;	
	float Ang_W;
	/*
	以车头方向为Y轴，横向为X轴，建立的XOY坐标系，标定舵轮的位置。
	如果舵轮位置在一三象限，Offset_Dir给任意正值;
	如果舵轮位置在二四象限，Offset_Dir给任意负值;
	如果舵轮位置在中轴线，无偏置，则Offset_Dir=0;
	*/
	float rD = sqrt(Length*Length + Offset*Offset);
	
	if (Offset_Dir>0){
		Ang_W = asin(Length/rD);    //如果舵轮位置在一三象限，原地旋转舵轮角度在(0,1.57)之间	
	}else if(Offset_Dir<0){
		Ang_W = -1*asin(Length/rD); //如果舵轮位置在二四象限，原地旋转舵轮角度在(-1.57,0) 之间
	}else{
		Ang_W = 1.57;
	}
		
	if (rD!=0){
		Vmx = CP.Vx*(Length/rD) + CP.Vy*(Offset/rD);	
		Vmy = -1*CP.Vx*(Offset/rD) + CP.Vy*(Offset/rD);
	}
	Wm = CP.W;
	V1_tmp = Vmy + Wm*Length;
	V2_tmp = Vmy - Wm*Length;
	Theta1 = atan(V1_tmp/Vmx);
	Theta2 = atan(V2_tmp/Vmx);
	
	Th_Normal(&Theta1);
	Th_Normal(&Theta2);	
	Limit(&Theta1,2.268,-2.268);  //舵轮限位130度，确认电机型号
	
	if (0==Theta1){
		Vec1 = Vmx;
	}else{
		Vec1 = V1_tmp/sin(Theta1);		
	}
	if (0==Theta2){
		Vec2 = Vmx;
	}else{
		Vec2 = V2_tmp/sin(Theta2);		
	}
	
	if (4==CP.Cir_Flag){
		//Theta1=1.57;
		//Theta2=1.57;
		Theta1 = Ang_W;     //原地旋转的设定舵轮角度
		Theta2 = Ang_W;
		if (fabs(Driver.Th1-Theta1)>0.1 || fabs(Driver.Th2-Theta2)>0.1 ){  //原地旋转最小允许角度误差
			Vec1 = 0;
			Vec2 = 0;
		}
	}else{
		if (fabs(Driver.Th1-Theta1)>0.2 || fabs(Driver.Th2-Theta2)>0.2 ){  //非原地旋转最小允许角度误差
			Vec1 = 0;
			Vec2 = 0;
		}
	}
	
	/*自动控制走圆弧，舵轮角度确认，防止角度逼近极限位*/
	if (CP.Cir_Flag!=0 && !bWalk ){
		if ((fabs(Driver.Th1)>0.4 || fabs(Driver.Th2)>0.4) && !bOnce){
			bOnce = 1;
			bSwing = 1;
		}
		if (bSwing){
			if (1==CP.VW_Type || 3==CP.VW_Type){
				Theta1 = -1.57;
				Theta2 = -1.57;
				Vec1 = 0;
				Vec2 = 0;
			}else if (2==CP.VW_Type || 4==CP.VW_Type){
				Theta1 = 1.57;
				Theta2 = 1.57;
				Vec1 = 0;
				Vec2 = 0;			
			}
			if (fabs(Driver.Th1-Theta1)<0.1 && fabs(Driver.Th2-Theta2)<0.1){
				bSwing = 0;
			}
		}else{
			bWalk = 1;
		}	
	}
	if (0==CP.Cir_Flag){
		bWalk = 0;
	}
	
	math_out.V1 = Vec1;
	math_out.V2 = Vec2;
	math_out.Th1 = Theta1;
	math_out.Th2 = Theta2;
	
	return math_out;
}


/*---------------------------------------------------------------------------
四舵轮VW解算(斜对角两两一对，单独计算),【全向运动】
-----------------------------------------------------------------------------*/
struct VT_Calc MathMod_Proc3(Para_of_C CP, Agv_Para agv, struct VT_Real Driver)       
{
	struct VT_Calc math_out;
	struct VT_Calc calc_tmp1;
	struct VT_Calc calc_tmp2;
	/*以车头方向为Y轴，横向为X轴，建立的XOY坐标系，标定舵轮的安装位置。*/
	calc_tmp1 = MathMod_Proc2(CP, agv, Driver, 1);     //Offset>0, 位于二四象限的一组舵轮之解算
	calc_tmp2 = MathMod_Proc2(CP, agv, Driver, -1);    //Offset<0, 位于一三象限的一组舵轮之解算
	math_out.V1 = calc_tmp1.V1;   //第一象限舵轮之速度
	math_out.V2 = calc_tmp1.V2;   //第三象限舵轮之速度
	math_out.V3 = calc_tmp2.V1;   //第二象限舵轮之速度
	math_out.V4 = calc_tmp2.V2;   //第四象限舵轮之速度
	math_out.Th1 = calc_tmp1.Th1;  //第一象限舵轮之角度
	math_out.Th2 = calc_tmp1.Th2;  //第三象限舵轮之角度
	math_out.Th3 = calc_tmp2.Th1;  //第二象限舵轮之角度
	math_out.Th4 = calc_tmp2.Th2;  //第四象限舵轮之角度
	
	return math_out;
}


/*---------------------------------------------------------------------------
单差速驱动VW解算
-----------------------------------------------------------------------------*/
struct VT_Calc MathMod_Proc4(Para_of_C CP, Agv_Para agv, struct VT_Real Driver)    
{
	struct VT_Calc math_out;
	float Vec1,Vec2;         //本模块的结果，两个驱动轮的轮速	
	float Length = agv.rL/2;   //驱动轮到运动中心的距离
	Vec1 = CP.Vx - CP.W*Length;
	Vec2 = CP.Vx + CP.W*Length;
	
	math_out.V1 = Vec1;
	math_out.V2 = Vec2;	
	
	return math_out;
}


/*---------------------------------------------------------------------------
双差速驱动VW解算,【全向运动】
-----------------------------------------------------------------------------*/
struct VT_Calc MathMod_Proc5(Para_of_C CP, Agv_Para agv, struct VT_Real Driver)     
{
	struct VT_Calc math_out;
	struct VT_Calc calc_tmp;
	float V1_tmp, V2_tmp, Ang1_tmp, Ang2_tmp, W1_tmp, W2_tmp, Kw;
	float Vec1, Vec2, Vec3, Vec4;
	float Length = agv.rL/2;   //驱动轮到驱动模块运动中心的距离
	
	/*1、运动中心到驱动模块的解算*/
	calc_tmp = MathMod_Proc2(CP, agv, Driver, 0);     //Offset>0, 两组驱动模块之解算
	V1_tmp = calc_tmp.V1;     //驱动模块期望之速度
	V2_tmp = calc_tmp.V2;
	Ang1_tmp = calc_tmp.Th1;  //驱动模块期望之转角
	Ang2_tmp = calc_tmp.Th2;
	
	Kw = 0.4;
	W1_tmp = Kw*(Ang1_tmp-Driver.Th1);
	W2_tmp = Kw*(Ang2_tmp-Driver.Th2);

	/*2、驱动模块到驱动轮的解算*/
	Vec1 = V1_tmp - W1_tmp*Length;   //前驱动模块之左轮速度
	Vec2 = V1_tmp + W1_tmp*Length;   //前驱动模块之右轮速度
	Vec3 = V2_tmp - W2_tmp*Length;   //后驱动模块之左轮速度
	Vec4 = V2_tmp + W2_tmp*Length;   //后驱动模块之右轮速度
	
	math_out.V1 = Vec1;
	math_out.V2 = Vec2;
	math_out.V3 = Vec3;
	math_out.V4 = Vec4;	
	
	return math_out;
}


/*---------------------------------------------------------------------------
四差速驱动VW解算,【全向运动】
-----------------------------------------------------------------------------*/
struct VT_Calc MathMod_Proc6(Para_of_C CP, Agv_Para agv, struct VT_Real Driver)      
{
	struct VT_Calc math_out;
	struct VT_Calc calc_tmp1;
	struct VT_Calc calc_tmp2;
	float Length = agv.rL/2;   //驱动轮到驱动模块运动中心的距离
	float Kw = 0.4;
	float W1_tmp, W2_tmp, W3_tmp, W4_tmp;
	/*以车头方向为Y轴，横向为X轴，建立的XOY坐标系，标定驱动模块的安装位置。*/
	calc_tmp1 = MathMod_Proc2(CP, agv, Driver, 1);     //Offset>0, 位于一三象限的一组驱动模块之解算
	calc_tmp2 = MathMod_Proc2(CP, agv, Driver, -1);    //Offset<0, 位于二四象限的一组驱动模块之解算	
	
	W1_tmp = Kw*(calc_tmp1.Th1-Driver.Th1);   
	W2_tmp = Kw*(calc_tmp1.Th2-Driver.Th2); 
	W3_tmp = Kw*(calc_tmp2.Th1-Driver.Th3);   
	W4_tmp = Kw*(calc_tmp2.Th2-Driver.Th4); 	
	
	math_out.V1 = calc_tmp1.V1 - W1_tmp*Length;  //第一象限驱动模块之左轮速度
	math_out.V2 = calc_tmp1.V1 + W1_tmp*Length;	 //第一象限驱动模块之右轮速度
	math_out.V3 = calc_tmp1.V2 - W2_tmp*Length;   //第三象限驱动模块之左轮速度
	math_out.V4 = calc_tmp1.V2 + W2_tmp*Length;   //第三象限驱动模块之右轮速度
	math_out.V5 = calc_tmp2.V1 - W3_tmp*Length;   //第二象限驱动模块之左轮速度
	math_out.V6 = calc_tmp2.V1 + W3_tmp*Length;	  //第二象限驱动模块之右轮速度
	math_out.V7 = calc_tmp2.V2 - W4_tmp*Length;	  //第四象限驱动模块之左轮速度
	math_out.V8 = calc_tmp2.V2 + W4_tmp*Length;	  //第四象限驱动模块之右轮速度	
	
	return math_out;
}


/*---------------------------------------------------------------------------
car_like运动学模型VW解算，参考https://blog.csdn.net/weixin_39549161/article/details/88712443
-----------------------------------------------------------------------------*/
struct VT_Calc MathMod_Proc7(Para_of_C CP, Agv_Para agv, struct VT_Real Driver)      
{
	struct VT_Calc math_out;
	float Length = agv.rL; //前轮与后轮之间轴线距离
	float R_cir;
	math_out.V1 = CP.Vx;   //运动中心速度就是后轮速度
	if (CP.W!=0){
		R_cir = CP.Vx/CP.W;  //运动半径	
		math_out.Th1 = atan(Length/R_cir);
	}else{
		math_out.Th1 = 0;
	}

	return math_out;
}


/*---------------------------------------------------------------------------
四麦轮VW解算（停车agv，单动）【全向运动】
-----------------------------------------------------------------------------*/
struct VT_Calc MathMod_Proc8(Para_of_C CP, Agv_Para agv, struct VT_Real Driver)        
{
	struct VT_Calc math_out;
	float Length = agv.rL/2;
	float Offset = agv.rH/2;
	
	math_out.V1 = CP.Vy - CP.Vx + CP.W*(Length + Offset);  //第一象限麦轮速度
	math_out.V2 = CP.Vy - CP.Vx - CP.W*(Length + Offset);	 //第三象限麦轮速度
	math_out.V3 = CP.Vy + CP.Vx - CP.W*(Length + Offset);	 //第二象限麦轮速度
	math_out.V4 = CP.Vy + CP.Vx + CP.W*(Length + Offset);	 //第四象限麦轮速度
	
	return math_out;
}


/*---------------------------------------------------------------------------
两车联动（四麦轮）VW解算（停车agv，两车合体后，联动）【全向运动】
-----------------------------------------------------------------------------*/
struct VT_Calc MathMod_Proc9(Para_of_C CP, Agv_Para agv, struct VT_Real Driver)     
{
	struct VT_Calc math_out;  
	struct VT_Calc calc_tmp1;  //前车
	struct VT_Calc calc_tmp2;  //后车
	float Length = agv.rD_FrontBack/2;
	float Offset = agv.rD_TurnRight/2;
	
	/*车子运动中心到前后agv运动中心的解算*/
	Para_of_C CP1;   
	Para_of_C CP2;  
	CP1.Vx = CP.Vx;
	CP2.Vx = CP.Vx;
	CP1.W = 0;
	CP2.W = 0;	
	CP1.Vy = CP.Vy + CP.W*Length;   
	CP2.Vy = CP.Vy - CP.W*Length;	
	
	calc_tmp1 = MathMod_Proc8(CP1, agv, Driver);
	calc_tmp2 = MathMod_Proc8(CP2, agv, Driver);	
	
	math_out.V1 = calc_tmp1.V1;
	math_out.V2 = calc_tmp1.V2;
	math_out.V3 = calc_tmp1.V3;
	math_out.V4 = calc_tmp1.V4;
	math_out.V5 = calc_tmp2.V1;
	math_out.V6 = calc_tmp2.V2;
	math_out.V7 = calc_tmp2.V3;
	math_out.V8 = calc_tmp2.V4;
	
	return math_out;
}


/*---------------------------------------------------------------------------
单舵轮里程计
-----------------------------------------------------------------------------*/
Agv_Odo Speedometer_Calc1(Agv_Para agv, struct VT_Real Driver)
{
	Agv_Odo Odo_out;
	float Vx1, Vy1;
	float Vx2, Vy2;
	float Length = agv.rL;
	float Offset = agv.rH;
	float rD = sqrt(Length*Length + Offset*Offset);	
	float AngVec, LineVelocity;  //牵连速度、角速度
	float Vec, W;  //运动中心速度、角速度
	float cur_X, cur_Y, cur_Phi, last_X, last_Y, last_Phi;
	float Sample_Time = 30;
	float Vx_g, Vy_g, W_g;	
	Vx1 = Driver.V1*cos(Driver.Th1);
	Vy1 = Driver.V1*sin(Driver.Th1);
	LineVelocity = Vy1*rD/Length;
	
	AngVec = LineVelocity/rD;          //运动中心角速度
	Vec = Vx1-LineVelocity*Offset/rD;  //运动中心速度
	Vx_g = Vec*cos(cur_Phi);           //世界坐标系下Vx
	Vy_g = Vec*sin(cur_Phi);	         //世界坐标系下Vy
	W = AngVec;                        //世界坐标系下W
	
	cur_X = last_X+Vx_g*Sample_Time*Decimals;
	cur_Y = last_Y+Vy_g*Sample_Time*Decimals;
	cur_Phi = last_Phi+W*Sample_Time*Decimals;	
	
	last_X = cur_X;
	last_Y = cur_Y;
	last_Phi = cur_Phi;
	
	Odo_out.Vx = Vec;
	Odo_out.W = AngVec;
	Odo_out.x = cur_X;
	Odo_out.y = cur_Y;
	Odo_out.phi = cur_Phi;
	
	return Odo_out;
}


/*---------------------------------------------------------------------------
双舵轮里程计，沿舵轮连线方向进行速度分解。
-----------------------------------------------------------------------------*/
Agv_Odo Speedometer_Calc2(Agv_Para agv, struct VT_Real Driver, int Offset_Dir)
{
	Agv_Odo Odo_out;
	float Length = agv.rD_FrontBack;
	float Offset = agv.rD_TurnRight;	
	float Ang_W;
	float Vmx, Vmy, Wm;
	float Vx, Vy, W;
	float Vx_g, Vy_g, W_g;
	float cur_X, cur_Y, cur_Phi, last_X, last_Y, last_Phi;
	float Sample_Time = 30;
	/*
	以车头方向为Y轴，横向为X轴，建立的XOY坐标系，标定舵轮的位置。
	如果舵轮位置在一三象限，Offset_Dir给任意正值;
	如果舵轮位置在二四象限，Offset_Dir给任意负值;
	如果舵轮位置在中轴线，无偏置，则Offset_Dir=0;
	*/
	float rD = sqrt(Length*Length + Offset*Offset);
	
	Ang_W = asin(Length/rD);    
 
	//沿轴线方向进行速度分解;
	if (Offset_Dir>=0){
		//舵轮安装位于一三象限
		Vmx = (Driver.V1*cos(Driver.Th1)+Driver.V2*cos(Driver.Th2))/2.0;   //沿求舵轮连线方向运动中心的速度
		Vmy = (Driver.V1*sin(Driver.Th1)+Driver.V2*sin(Driver.Th2))/2.0;   //垂直于舵轮连线方向的速度
		Wm  = (Driver.V1*sin(Driver.Th1)-Driver.V2*sin(Driver.Th2))/Length; 			
		Vx = Vmx*cos(Ang_W)+Vmy*sin(Ang_W);    //求相对坐标系下运动中心的速度
		Vy = Vmy*cos(Ang_W)-Vmx*sin(Ang_W);
		W =  Wm;	
	}else{
		//舵轮安装位于二四象限
		Vmx = ( Driver.V1*cos(Driver.Th1)+Driver.V2*cos(Driver.Th2) )/2.0;   //沿求舵轮连线方向运动中心的速度
		Vmy = ( -Driver.V1*sin(Driver.Th1)-Driver.V2*sin(Driver.Th2) )/2.0;   //垂直于舵轮连线方向的速度
		Wm  = ( -Driver.V1*sin(Driver.Th1)+Driver.V2*sin(Driver.Th2) )/Length; 			
		Vx = Vmx*cos(Ang_W)-Vmy*sin(Ang_W);    //求相对坐标系下运动中心的速度
		Vy = Vmy*cos(Ang_W)+Vmx*sin(Ang_W);
		W =  Wm;			
	}

	cur_Phi = last_Phi+W*Sample_Time*Decimals; /*求世界坐标系下运动中心坐标系的旋转角度(即车体相对世界坐标系的实时姿态角)*/	
	/*求世界坐标系下，运动中心的速度*/
	Vx_g = Vx*cos(cur_Phi)-Vy*sin(cur_Phi);
	Vy_g = Vx*sin(cur_Phi)+Vy*cos(cur_Phi);
	cur_X = last_X+Vx_g*Sample_Time*Decimals;
	cur_Y = last_Y+Vy_g*Sample_Time*Decimals;	
	
	Th_Normal(&cur_Phi);
	last_X = cur_X;
	last_Y = cur_Y;
	last_Phi = cur_Phi;
	
	Odo_out.Vx = Vx;
	Odo_out.Vy = Vy;
	Odo_out.W = W;	
	Odo_out.x = cur_X;
	Odo_out.y = cur_Y;
	Odo_out.phi = cur_Phi;
	
	return Odo_out;
}


/*---------------------------------------------------------------------------
四舵轮里程计，沿对角一组舵轮的连线方向进行速度分解。
-----------------------------------------------------------------------------*/
Agv_Odo Speedometer_Calc3(Agv_Para agv, struct VT_Real Driver)
{
	Agv_Odo Odo_out;
	float Length = agv.rD_FrontBack;
	float Offset = agv.rD_TurnRight;
	float Ang_W;
	float Vmx1, Vmy1, Wm1, Vmx2, Vmy2, Wm2;
	float Vx1, Vy1, W1, Vx2, Vy2, W2;
	float Vx_sum, Vy_sum, W_sum;
	float Vx_g, Vy_g, W_g;   //世界坐标系下的速度、角速度
	float cur_X, cur_Y, cur_Phi, last_X, last_Y, last_Phi;
	float Sample_Time = 30;	
	
	//沿轴线方向进行速度分解;
	
	//位于一三象限的一组舵轮
	Vmx1 = (Driver.V1*cos(Driver.Th1)+Driver.V2*cos(Driver.Th2))/2.0;   //沿求舵轮连线方向运动中心的速度
	Vmy1 = (Driver.V1*sin(Driver.Th1)+Driver.V2*sin(Driver.Th2))/2.0;   //垂直于舵轮连线方向的速度
	Wm1  = (Driver.V1*sin(Driver.Th1)-Driver.V2*sin(Driver.Th2))/Length; 			
	Vx1 = Vmx1*cos(Ang_W)+Vmy1*sin(Ang_W);    //求车体坐标系下运动中心的速度
	Vy1 = Vmy1*cos(Ang_W)-Vmx1*sin(Ang_W);
	W1 =  Wm1;	

	//位于二四象限的一组舵轮
	Vmx2 = ( Driver.V3*cos(Driver.Th3)+Driver.V4*cos(Driver.Th4) )/2.0;   //沿求舵轮连线方向运动中心的速度
	Vmy2 = ( -Driver.V3*sin(Driver.Th3)-Driver.V4*sin(Driver.Th4) )/2.0;   //垂直于舵轮连线方向的速度
	Wm2  = ( -Driver.V3*sin(Driver.Th3)+Driver.V4*sin(Driver.Th4) )/Length; 			
	Vx2 = Vmx2*cos(Ang_W)-Vmy2*sin(Ang_W);    //求车体坐标系下运动中心的速度
	Vy2 = Vmy2*cos(Ang_W)+Vmx2*sin(Ang_W);
	W2 =  Wm2;			

	Vx_sum = (Vx1+Vx2)/2;
	Vy_sum = (Vy1+Vy2)/2;
	W_sum = (W1+W2)/2;
	
	W_g = W_sum;
	cur_Phi = last_Phi+W_g*Sample_Time*Decimals; /*求世界坐标系下运动中心坐标系的旋转角度(即车体相对世界坐标系的实时姿态角)*/	
	/*求世界坐标系下，运动中心的速度*/
	Vx_g = Vx_sum*cos(cur_Phi)-Vy_sum*sin(cur_Phi);
	Vy_g = Vx_sum*sin(cur_Phi)+Vy_sum*cos(cur_Phi);
	cur_X = last_X+Vx_g*Sample_Time*Decimals;
	cur_Y = last_Y+Vy_g*Sample_Time*Decimals;	

	last_Phi = cur_Phi;
	last_X = cur_X;
	last_Y = cur_Y;
	
	Odo_out.Vx = Vx_sum;
	Odo_out.Vy = Vy_sum;
	Odo_out.W = W_sum;
	Odo_out.x = cur_X;
	Odo_out.y = cur_Y;
	Odo_out.phi = cur_Phi;
	
	return Odo_out;
}


/*---------------------------------------------------------------------------
单差速驱动里程计
-----------------------------------------------------------------------------*/
Agv_Odo Speedometer_Calc4(Agv_Para agv, struct VT_Real Driver)
{
	Agv_Odo Odo_out;
	float Vec,W;         //本模块的结果，两个驱动轮的轮速	
	float cur_X, cur_Y, cur_Phi, last_X, last_Y, last_Phi;
	float Vx_g, Vy_g, W_g;
	float Sample_Time = 30;	
	
	Vec = (Driver.V1+Driver.V2)/2;
	W = (Driver.V2-Driver.V1)/agv.rL;  //agv.rL ,两车轮之间的距离
	
	cur_Phi = last_Phi+W*Sample_Time*Decimals; /*求世界坐标系下运动中心坐标系的旋转角度(即车体相对世界坐标系的实时姿态角)*/	
	
	W_g = W;
	cur_Phi = last_Phi+W_g*Sample_Time*Decimals; /*求世界坐标系下运动中心坐标系的旋转角度(即车体相对世界坐标系的实时姿态角)*/	
	/*求世界坐标系下，运动中心的速度*/
	Vx_g = Vec*cos(cur_Phi);  //世界坐标系下的速度、角速度
	Vy_g = Vec*sin(cur_Phi);
	cur_X = last_X+Vx_g*Sample_Time*Decimals;
	cur_Y = last_Y+Vy_g*Sample_Time*Decimals;	

	last_Phi = cur_Phi;
	last_X = cur_X;
	last_Y = cur_Y;	
	
	Odo_out.Vx = Vec;
	Odo_out.W = W;
	Odo_out.x = cur_X;
	Odo_out.y = cur_Y;
	Odo_out.phi = cur_Phi;
	
	return Odo_out;
	
}


/*---------------------------------------------------------------------------
双差速驱动里程计,（与双舵轮里程计相同）
-----------------------------------------------------------------------------*/
Agv_Odo Speedometer_Calc5(Agv_Para agv, struct VT_Real Driver)
{
	Agv_Odo Odo_out;
	//Driver输入为四个驱动轮速度，以及两个驱动模块的角度
	struct VT_Real Driver_tmp;
	Driver_tmp.V1 = (Driver.V1+Driver.V2)/2;
	Driver_tmp.V2 = (Driver.V3+Driver.V4)/2;
	Driver_tmp.Th1 = Driver.Th1;
	Driver_tmp.Th2 = Driver.Th2;
	
	Odo_out = Speedometer_Calc2(agv, Driver_tmp, 0);  //一般双差速驱动模块位于中心，所以取0
	
	return Odo_out;
	
}


/*---------------------------------------------------------------------------
四差速驱动里程计,直接调用四舵轮里程计算过程。
-----------------------------------------------------------------------------*/
Agv_Odo Speedometer_Calc6(Agv_Para agv, struct VT_Real Driver)
{
	Agv_Odo Odo_out;
	//Driver输入为四个驱动轮速度，以及两个驱动模块的角度
	struct VT_Real Driver_tmp;		
	
	Driver_tmp.V1 = (Driver.V1+Driver.V2)/2;
	Driver_tmp.V2 = (Driver.V3+Driver.V4)/2;
	Driver_tmp.V3 = (Driver.V5+Driver.V6)/2;
	Driver_tmp.V4 = (Driver.V7+Driver.V8)/2;	
	Driver_tmp.Th1 = Driver.Th1;
	Driver_tmp.Th2 = Driver.Th2;
	Driver_tmp.Th1 = Driver.Th3;
	Driver_tmp.Th2 = Driver.Th4;	
	
	Odo_out = Speedometer_Calc3(agv, Driver_tmp);
	return Odo_out;
}


/*---------------------------------------------------------------------------
Car-like模型里程计
-----------------------------------------------------------------------------*/
Agv_Odo Speedometer_Calc7(Agv_Para agv, struct VT_Real Driver)
{
	Agv_Odo Odo_out;
	float Vec, W;
	float R_cir;	
	float cur_X, cur_Y, cur_Phi, last_X, last_Y, last_Phi;
	float Vx_g, Vy_g, W_g;
	float Sample_Time = 30;	
	Vec = Driver.V1;
	if(0==Driver.Th1){
		W = 0;
	}else{
		R_cir = agv.rL/tan(Driver.Th1);		
		W = Vec/R_cir;
	}

	W_g = W;
	cur_Phi = last_Phi+W_g*Sample_Time*Decimals; /*求世界坐标系下运动中心坐标系的旋转角度(即车体相对世界坐标系的实时姿态角)*/	
	/*求世界坐标系下，运动中心的速度*/
	Vx_g = Vec*cos(cur_Phi);  //世界坐标系下的速度、角速度
	Vy_g = Vec*sin(cur_Phi);
	cur_X = last_X+Vx_g*Sample_Time*Decimals;
	cur_Y = last_Y+Vy_g*Sample_Time*Decimals;	

	last_Phi = cur_Phi;
	last_X = cur_X;
	last_Y = cur_Y;	
	
	Odo_out.Vx = Vec;
	Odo_out.W = W;
	Odo_out.x = cur_X;
	Odo_out.y = cur_Y;
	Odo_out.phi = cur_Phi;
	
	return Odo_out;	
}


/*---------------------------------------------------------------------------
四轮Mecanum模型里程计
-----------------------------------------------------------------------------*/
Agv_Odo Speedometer_Calc8(Agv_Para agv, struct VT_Real Driver)
{
	Agv_Odo Odo_out;	
	float cur_X, cur_Y, cur_Phi, last_X, last_Y, last_Phi;
	float Sample_Time = 30;		
	float Length = agv.rL/2;
	float Offset = agv.rH/2;
	float VecX, VecY, W; //车体坐标系下的速度角速度
	float Vx_g, Vy_g, W_g;  //世界坐标系下的速度角速度
	
	/*由逆运动学，已知各麦轮的实际速度，求得沿车体方向的速度、垂直于车体方向速度、角速度*/
	VecX = (-Driver.V1-Driver.V2+Driver.V3+Driver.V4)/4;
	VecY = (Driver.V1+Driver.V2+Driver.V3+Driver.V4)/4;
	W = (Driver.V1-Driver.V2-Driver.V3+Driver.V4)/(4*(Length+Offset));

	W_g = W;
	cur_Phi = last_Phi+W_g*Sample_Time*Decimals; /*求世界坐标系下运动中心坐标系的旋转角度(即车体相对世界坐标系的实时姿态角)*/	
	/*求世界坐标系下，运动中心的速度*/
	Vx_g = VecX*cos(cur_Phi)-VecY*sin(cur_Phi);
	Vy_g = VecX*sin(cur_Phi)+VecY*cos(cur_Phi);
	cur_X = last_X+Vx_g*Sample_Time*Decimals;
	cur_Y = last_Y+Vy_g*Sample_Time*Decimals;	

	last_Phi = cur_Phi;
	last_X = cur_X;
	last_Y = cur_Y;	
	
	Odo_out.Vx = VecX;
	Odo_out.Vy = VecY;
	Odo_out.W = W;
	Odo_out.x = cur_X;
	Odo_out.y = cur_Y;
	Odo_out.phi = cur_Phi;
	
	return Odo_out;		
}


/*---------------------------------------------------------------------------
双车联动，四轮Mecanum模型里程计（假设知道车体中心的实时位姿）
-----------------------------------------------------------------------------*/
Agv_Odo Speedometer_Calc9(Agv_Para agv, struct VT_Real Driver)
{
	Agv_Odo Odo_out;	
	float cur_X, cur_Y, cur_Phi, last_X, last_Y, last_Phi;
	float Sample_Time = 30;		
	float Length = agv.rL/2;
	float Offset = agv.rH/2;
	float VecX1, VecY1, W1, VecX2, VecY2, W2; //单体AGV坐标系下的速度、角速度
	float VecX, VecY, W; //两AGV中心坐标系下的速度、角速度
	float Vx_g, Vy_g, W_g;  //世界坐标系下的速度角速度	

	/*由逆运动学，已知各麦轮的实际速度，求得沿车体方向的速度、垂直于车体方向速度、角速度*/
	//1号AGV
	VecX1 = (-Driver.V1-Driver.V2+Driver.V3+Driver.V4)/4;
	VecY1 = (Driver.V1+Driver.V2+Driver.V3+Driver.V4)/4;
	W1 = (Driver.V1-Driver.V2-Driver.V3+Driver.V4)/(4*(Length+Offset));	
	//2号AGV
	VecX1 = (-Driver.V1-Driver.V2+Driver.V3+Driver.V4)/4;
	VecY1 = (Driver.V1+Driver.V2+Driver.V3+Driver.V4)/4;
	W1 = (Driver.V1-Driver.V2-Driver.V3+Driver.V4)/(4*(Length+Offset));	
	
	VecX = (VecX1+VecX2)/2;
	VecY = (VecY1+VecY2)/2;
	W = (VecY1-VecY2)/agv.rD_FrontBack;
	
	W_g = W;
	cur_Phi = last_Phi+W_g*Sample_Time*Decimals; /*求世界坐标系下运动中心坐标系的旋转角度(即车体相对世界坐标系的实时姿态角)*/	
	/*求世界坐标系下，运动中心的速度*/
	Vx_g = VecX*cos(cur_Phi)-VecY*sin(cur_Phi);
	Vy_g = VecX*sin(cur_Phi)+VecY*cos(cur_Phi);
	
	cur_X = last_X+Vx_g*Sample_Time*Decimals;
	cur_Y = last_Y+Vy_g*Sample_Time*Decimals;	

	last_Phi = cur_Phi;
	last_X = cur_X;
	last_Y = cur_Y;	
	
	Odo_out.Vx = VecX;
	Odo_out.Vy = VecY;
	Odo_out.W = W;
	Odo_out.x = cur_X;
	Odo_out.y = cur_Y;
	Odo_out.phi = cur_Phi;
	
	return Odo_out;		
}


void Car_Model_Config(void *Para)
{
	switch (agv_modcfg)
	{
		case 1:
			vt_calc = MathMod_Proc1(para_from_C, agv_para, vt_real) ;       //One_Helm         
			agv_odo = Speedometer_Calc1(agv_para, vt_real);                 
			break;
		case 2:
			vt_calc = MathMod_Proc2(para_from_C, agv_para, vt_real, 1) ;    //Two_Helm          
			agv_odo = Speedometer_Calc2(agv_para, vt_real, 1);                 
			break;
		case 3:
			vt_calc = MathMod_Proc3(para_from_C, agv_para, vt_real) ;       //Four_Helm        
			agv_odo = Speedometer_Calc3(agv_para, vt_real); 
			break;
		case 4:
			vt_calc = MathMod_Proc4(para_from_C, agv_para, vt_real) ;       //One_Differential_Driver          
			agv_odo = Speedometer_Calc4(agv_para, vt_real); 
			break;
		case 5:
			vt_calc = MathMod_Proc5(para_from_C, agv_para, vt_real) ;       //Two_Differential_Driver       
			agv_odo = Speedometer_Calc5(agv_para, vt_real); 
			break;
		case 6:
			vt_calc = MathMod_Proc6(para_from_C, agv_para, vt_real) ;       //Four_Differential_Driver          
			agv_odo = Speedometer_Calc6(agv_para, vt_real); 
			break;
		case 7:
			vt_calc = MathMod_Proc7(para_from_C, agv_para, vt_real) ;       //Car_like Module    
			agv_odo = Speedometer_Calc7(agv_para, vt_real); 
			break;
		case 8:
			vt_calc = MathMod_Proc8(para_from_C, agv_para, vt_real) ;       //Mecanum Module of Sigle AGV           
			agv_odo = Speedometer_Calc8(agv_para, vt_real); 
			break;
		case 9:
			vt_calc = MathMod_Proc9(para_from_C, agv_para, vt_real) ;       //Mecanum Module of Double AGV       
			agv_odo = Speedometer_Calc9(agv_para, vt_real); 
			break;	
	}	
 

}

void Car_Model_MathMod(void *para)	//速度解算
{
	int mod;
	BaseType_t xHigherPriorityTaskWoken_Msg;
	for(; ;)
	{
		mod = 1;
		switch (mod)
		{
			case 1:																										//One_Helm 
				vt_calc = (MathMod_Proc1(para_from_C, agv_para, vt_real)) ;                          
				break;
			case 2:																										//Two_Helm   
				vt_calc = MathMod_Proc2(para_from_C, agv_para, vt_real, 1) ;                        
				break;
			case 3: 																									//Four_Helm  
				vt_calc = MathMod_Proc3(para_from_C, agv_para, vt_real) ;             
				break;
			case 4:																										//One_Differential_Driver  
				vt_calc = MathMod_Proc4(para_from_C, agv_para, vt_real) ;               
				break;
			case 5:																										//Two_Differential_Driver
				vt_calc = MathMod_Proc5(para_from_C, agv_para, vt_real) ;              
				break;
			case 6:																										//Four_Differential_Driver 
				vt_calc = MathMod_Proc6(para_from_C, agv_para, vt_real) ;                
				break;
			case 7:																										//Car_like Module
				vt_calc = MathMod_Proc7(para_from_C, agv_para, vt_real) ;           
				break;
			case 8:																										//Mecanum Module of Sigle AGV
				vt_calc = MathMod_Proc8(para_from_C, agv_para, vt_real) ;                  
				break;
			case 9:
				break;
			default:
				break;
		}
		
	  //二值信号量，速度解算后，发送数据出去			
		if (mod ==1)
		{
				//释放二值信号量
        if(BinarySemaphore_M4toMcu!=NULL)//接收到数据，并且二值信号量有效
        {
            xSemaphoreGiveFromISR(BinarySemaphore_M4toMcu,&xHigherPriorityTaskWoken_Msg);	//释放二值信号量
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken_Msg);//如果需要的话进行一次任务切换
        }
		}
		vTaskDelay(100);      //延时100ms		

	}


}

void Car_Model_Odo(void *para)									//里程计
{
	u32 NotifyValue;
   BaseType_t err=pdFALSE;

 for(;;)
//    while(1)
 {
      if(BinarySemaphore!=NULL)
        {
            err=xSemaphoreTake(BinarySemaphore,portMAX_DELAY); //获取二值信号量
            if(err==pdTRUE)
                {
										switch (agv_modcfg)
										{
											case 1:																					//One_Helm         
												agv_odo = Speedometer_Calc1(agv_para, vt_real);                 
												break;
											case 2:																					//Two_Helm          
												agv_odo = Speedometer_Calc2(agv_para, vt_real, 1);  
												break;
											case 3:																					//Four_Helm        
												agv_odo = Speedometer_Calc3(agv_para, vt_real); 
												break;
											case 4:																					//One_Differential_Driver          
												agv_odo = Speedometer_Calc4(agv_para, vt_real); 
												break;
											case 5:																					//Two_Differential_Driver       
												agv_odo = Speedometer_Calc5(agv_para, vt_real); 
												break;
											case 6:																					//Four_Differential_Driver          
												agv_odo = Speedometer_Calc6(agv_para, vt_real); 
												break;
											case 7:																					//Car_like Module    
												agv_odo = Speedometer_Calc7(agv_para, vt_real); 
												break;
											case 8:																					//Mecanum Module of Sigle AGV           
												agv_odo = Speedometer_Calc8(agv_para, vt_real); 
												break;
											case 9:																  				//Mecanum Module of Double AGV       
												agv_odo = Speedometer_Calc9(agv_para, vt_real); 
												break;	
										}	
										
							}
					}								
				}
}
								