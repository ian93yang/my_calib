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
	void Car_Model_MathMod();				//�ٶȽ���
	void Car_Model_Odo();						//��̼�

	struct VT_Calc MathMod_Proc1(Para_of_C, Agv_Para, struct VT_Real);
	struct VT_Calc MathMod_Proc2(Para_of_C, Agv_Para, struct VT_Real, int);
	struct VT_Calc MathMod_Proc3(Para_of_C, Agv_Para, struct VT_Real);
	struct VT_Calc MathMod_Proc4(Para_of_C, Agv_Para, struct VT_Real);
	struct VT_Calc MathMod_Proc5(Para_of_C, Agv_Para, struct VT_Real);
	struct VT_Calc MathMod_Proc6(Para_of_C, Agv_Para, struct VT_Real);
	struct VT_Calc MathMod_Proc7(Para_of_C, Agv_Para, struct VT_Real);
	struct VT_Calc MathMod_Proc8(Para_of_C, Agv_Para, struct VT_Real);
	struct VT_Calc MathMod_Proc9(Para_of_C, Agv_Para, struct VT_Real);

	//��̼�
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


/*����ģ������*/
typedef enum Car_ModCfg {
	One_Helm = 1,   //������
	Two_Helm = 2,   //˫���֣�ȫ���˶�
	Four_Helm = 3,   //�Ķ��֣�ȫ���˶�
	One_Diff = 4,	 //������
	Two_Diff = 5,	 //˫���٣�ȫ���˶�
	Four_Diff = 6,   //�Ĳ��٣�ȫ���˶�
	Car_like = 7,   //Car-like�����������г�ģ��  
	Mecanum = 8,   //�����֣�ȫ���˶�
	Mecanum_2 = 9    //�����ֳ���˫��������ȫ���˶�
}Agv_ModCfg;


/*���弸�β�������*/
typedef struct Geom_Param
{
	float rL;            				//�����ֳ��ͣ��˶����ĵ����ֵľ���
	float rH;           				//�����ֳ��ͣ��������˶����ĵ�ƫ�þ���

/*������ͬ����ģ��֮��ľ���*/
	float rD_TurnRight;  				//������������
	float rD_FrontBack;  				//ǰ����������

/*����������������*/
	float rD_Merge;   //ǰ�����󳵵����߾���
}Agv_Para;


/*C�������*/
typedef struct Para_From_C
{
	float Vx;
	float Vy;
	float W;
	char VW_Type;   //VW����
	char Cir_Flag;  //Բ����־
  /*
	VW_Type=1,��ͷ��ǰ��ת
	VW_Type=2,��ͷ��ǰ��ת
	VW_Type=3,��ͷ��ǰ��ת
	VW_Type=4,��ͷ��ǰ��ת
	Cir_Flag=0,ֱ��
	Cir_Flag=1,ֱ�ߺ�ĵ�һ��Բ��
	Cir_Flag=2,Բ��Բ���ĵڶ���Բ��
	Cir_Flag=3,Բ������
	Cir_Flag=4,ԭ����ת
	*/
}*pPara_of_C, Para_of_C;


/*��̼����*/
typedef struct Car_Odo
{
	float x;    //λ����̼�x
	float y;    //λ����̼�y
	float phi;  //λ����̼�phi
	float Vx;   //�ٶ���̼�Vx
	float Vy;   //�ٶ���̼�Vy
	float W;    //�ٶ���̼�W
}*p_Agv_Odo, Agv_Odo;


/*�ٶȽ������*/
struct VT_Calc
{
	float V1;   //��������1-�ٶ�
	float V2;   //��������2-�ٶ�
	float V3;   //��������3-�ٶ�
	float V4;   //��������4-�ٶ� 
	float V5;   //��������4-�ٶ� 
	float V6;   //��������4-�ٶ� 
	float V7;   //��������4-�ٶ� 
	float V8;   //��������4-�ٶ� 	
	float Th1;  //ת������1-�Ƕ�
	float Th2;  //ת������2-�Ƕ�
	float Th3;  //ת������3-�Ƕ�
	float Th4;  //ת������4-�Ƕ�
};

/*���ʵ���ϴ�*/
struct VT_Real
{
	float V1;   //��������1-�ٶ�
	float V2;   //��������2-�ٶ�
	float V3;   //��������3-�ٶ�
	float V4;   //��������4-�ٶ� 
	float V5;   //��������4-�ٶ� 
	float V6;   //��������4-�ٶ� 
	float V7;   //��������4-�ٶ� 
	float V8;   //��������4-�ٶ� 
	float Th1;  //ת������1-�Ƕ�
	float Th2;  //ת������2-�Ƕ�
	float Th3;  //ת������3-�Ƕ�
	float Th4;  //ת������4-�Ƕ�	
};

extern Agv_ModCfg 		agv_modcfg;
extern Agv_Para 		  agv_para;
extern Para_of_C 	  para_from_C;
extern Agv_Odo 		  agv_odo;
extern struct VT_Calc 		  vt_calc;
extern struct VT_Real 		  vt_real;
extern struct VT_Calc vt_calc;
//�ٶȽ���
struct VT_Calc MathMod_Proc1(Para_of_C, Agv_Para, struct VT_Real);
struct VT_Calc MathMod_Proc2(Para_of_C, Agv_Para, struct VT_Real, int);
struct VT_Calc MathMod_Proc3(Para_of_C, Agv_Para, struct VT_Real);
struct VT_Calc MathMod_Proc4(Para_of_C, Agv_Para, struct VT_Real);
struct VT_Calc MathMod_Proc5(Para_of_C, Agv_Para, struct VT_Real);
struct VT_Calc MathMod_Proc6(Para_of_C, Agv_Para, struct VT_Real);
struct VT_Calc MathMod_Proc7(Para_of_C, Agv_Para, struct VT_Real);
struct VT_Calc MathMod_Proc8(Para_of_C, Agv_Para, struct VT_Real);
struct VT_Calc MathMod_Proc9(Para_of_C, Agv_Para, struct VT_Real);

//��̼�
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
void Car_Model_MathMod(void *para);				//�ٶȽ���
void Car_Model_Odo(void *para);						//��̼�

#endif
