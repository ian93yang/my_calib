#include "Car_Model.h"

void manage()
{
	Agv_ModCfg m= One_Helm;  //µ¥¶æÂÖÄ£ÐÍ
	Agv_Para p;
	p.rH = 262.93;
	p.rL = 1470;
	CarMdl myMdl(m,p);
	if (myMdl.Get_C_Data() && myMdl.Get_Agv_Odo())
	{
		myMdl.Car_Model_MathMod();
		myMdl.Car_Model_Odo();
	}
}



