#ifndef __CAR_MODEL_H
#define __CAR_MODEL_H	 

//////////////////////////////////////////////////////////////////////////////////	  
#define PI acos(-1)

class CarMdl
{
public:
	CarMdl(Agv_ModCfg m, Agv_Para p) :agv_type(m), agv_parm(p) {};
	~CarMdl() {};	
	bool Get_C_Data();
	bool Get_Agv_Odo();
	void Car_Model_MathMod();				//速度解算
	void Car_Model_Odo();						//里程计

	struct VT_Calc MathMod_Proc1(Para_of_C, Agv_Para, struct VT_Real);
	struct VT_Calc MathMod_Proc2(Para_of_C, Agv_Para, struct VT_Real, int);
	struct VT_Calc MathMod_Proc3(Para_of_C, Agv_Para, struct VT_Real);
	struct VT_Calc MathMod_Proc4(Para_of_C, Agv_Para, struct VT_Real);
	struct VT_Calc MathMod_Proc5(Para_of_C, Agv_Para, struct VT_Real);
	struct VT_Calc MathMod_Proc6(Para_of_C, Agv_Para, struct VT_Real);
	struct VT_Calc MathMod_Proc7(Para_of_C, Agv_Para, struct VT_Real);
	struct VT_Calc MathMod_Proc8(Para_of_C, Agv_Para, struct VT_Real);
	struct VT_Calc MathMod_Proc9(Para_of_C, Agv_Para, struct VT_Real);

	//里程计
	Agv_Odo Speedometer_Calc1(Agv_Para, struct VT_Real);
	Agv_Odo Speedometer_Calc2(Agv_Para, struct VT_Real, int);
	Agv_Odo Speedometer_Calc3(Agv_Para, struct VT_Real);
	Agv_Odo Speedometer_Calc4(Agv_Para, struct VT_Real);
	Agv_Odo Speedometer_Calc5(Agv_Para, struct VT_Real);
	Agv_Odo Speedometer_Calc6(Agv_Para, struct VT_Real);
	Agv_Odo Speedometer_Calc7(Agv_Para, struct VT_Real);
	Agv_Odo Speedometer_Calc8(Agv_Para, struct VT_Real);
	Agv_Odo Speedometer_Calc9(Agv_Para, struct VT_Real);

private:
	Agv_ModCfg agv_type;
	Agv_Para agv_parm; 
	Para_of_C c_parm;
	Agv_Odo agv_odo;
	struct VT_Calc vt_calc;
	struct VT_Real vt_real;
};


/*车体模型配置*/
typedef enum Car_ModCfg {
	One_Helm = 1,   //单舵轮
	Two_Helm = 2,   //双舵轮，全向运动
	Four_Helm = 3,   //四舵轮，全向运动
	One_Diff = 4,	 //单差速
	Two_Diff = 5,	 //双差速，全向运动
	Four_Diff = 6,   //四差速，全向运动
	Car_like = 7,   //Car-like，阿克曼自行车模型  
	Mecanum = 8,   //四麦轮，全向运动
	Mecanum_2 = 9    //四麦轮车型双车联动，全向运动
}Agv_ModCfg;


/*车体几何参数配置*/
typedef struct Geom_Param
{
	float rL;            				//单舵轮车型，运动中心到舵轮的距离
	float rH;           				//单舵轮车型，舵轮与运动中心的偏置距离

/*单车不同驱动模块之间的距离*/
	float rD_TurnRight;  				//左右驱动距离
	float rD_FrontBack;  				//前后驱动距离

/*两车合体后的联动。*/
	float rD_Merge;   //前车到后车的轴线距离
}Agv_Para;


/*C程序输出*/
typedef struct Para_From_C
{
	float Vx;
	float Vy;
	float W;
	char VW_Type;   //VW类型
	char Cir_Flag;  //圆弧标志
  /*
	VW_Type=1,车头向前左转
	VW_Type=2,车头向前右转
	VW_Type=3,车头向前左转
	VW_Type=4,车头向前右转
	Cir_Flag=0,直线
	Cir_Flag=1,直线后的第一个圆弧
	Cir_Flag=2,圆弧圆弧的第二个圆弧
	Cir_Flag=3,圆弧结束
	Cir_Flag=4,原地旋转
	*/
}*pPara_of_C, Para_of_C;


/*里程计输出*/
typedef struct Car_Odo
{
	float x;    //位置里程计x
	float y;    //位置里程计y
	float phi;  //位置里程计phi
	float Vx;   //速度里程计Vx
	float Vy;   //速度里程计Vy
	float W;    //速度里程计W
}*p_Agv_Odo, Agv_Odo;


/*速度解算输出*/
struct VT_Calc
{
	float V1;   //行走驱动1-速度
	float V2;   //行走驱动2-速度
	float V3;   //行走驱动3-速度
	float V4;   //行走驱动4-速度 
	float V5;   //行走驱动4-速度 
	float V6;   //行走驱动4-速度 
	float V7;   //行走驱动4-速度 
	float V8;   //行走驱动4-速度 	
	float Th1;  //转向驱动1-角度
	float Th2;  //转向驱动2-角度
	float Th3;  //转向驱动3-角度
	float Th4;  //转向驱动4-角度
};

/*电机实际上传*/
struct VT_Real
{
	float V1;   //行走驱动1-速度
	float V2;   //行走驱动2-速度
	float V3;   //行走驱动3-速度
	float V4;   //行走驱动4-速度 
	float V5;   //行走驱动4-速度 
	float V6;   //行走驱动4-速度 
	float V7;   //行走驱动4-速度 
	float V8;   //行走驱动4-速度 
	float Th1;  //转向驱动1-角度
	float Th2;  //转向驱动2-角度
	float Th3;  //转向驱动3-角度
	float Th4;  //转向驱动4-角度	
};

extern Agv_ModCfg 		agv_modcfg;
extern Agv_Para 		  agv_para;
extern Para_of_C 	  para_from_C;
extern Agv_Odo 		  agv_odo;
extern struct VT_Calc 		  vt_calc;
extern struct VT_Real 		  vt_real;
extern struct VT_Calc vt_calc;
//速度解算
struct VT_Calc MathMod_Proc1(Para_of_C, Agv_Para, struct VT_Real);
struct VT_Calc MathMod_Proc2(Para_of_C, Agv_Para, struct VT_Real, int);
struct VT_Calc MathMod_Proc3(Para_of_C, Agv_Para, struct VT_Real);
struct VT_Calc MathMod_Proc4(Para_of_C, Agv_Para, struct VT_Real);
struct VT_Calc MathMod_Proc5(Para_of_C, Agv_Para, struct VT_Real);
struct VT_Calc MathMod_Proc6(Para_of_C, Agv_Para, struct VT_Real);
struct VT_Calc MathMod_Proc7(Para_of_C, Agv_Para, struct VT_Real);
struct VT_Calc MathMod_Proc8(Para_of_C, Agv_Para, struct VT_Real);
struct VT_Calc MathMod_Proc9(Para_of_C, Agv_Para, struct VT_Real);

//里程计
Agv_Odo Speedometer_Calc1(Agv_Para, struct VT_Real);
Agv_Odo Speedometer_Calc2(Agv_Para, struct VT_Real, int);
Agv_Odo Speedometer_Calc3(Agv_Para, struct VT_Real);
Agv_Odo Speedometer_Calc4(Agv_Para, struct VT_Real);
Agv_Odo Speedometer_Calc5(Agv_Para, struct VT_Real);
Agv_Odo Speedometer_Calc6(Agv_Para, struct VT_Real);
Agv_Odo Speedometer_Calc7(Agv_Para, struct VT_Real);
Agv_Odo Speedometer_Calc8(Agv_Para, struct VT_Real);
Agv_Odo Speedometer_Calc9(Agv_Para, struct VT_Real);


void Car_Model_Config(void *Para);
void Car_Model_MathMod(void *para);				//速度解算
void Car_Model_Odo(void *para);						//里程计

#endif
