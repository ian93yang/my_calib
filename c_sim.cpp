#include "ros/ros.h"            //������Ros�д󲿷ֵ�ͷ�ļ�
#include "std_msgs/String.h"  //�������� std_msgs/String ��Ϣ, ������� std_msgs package ����� String.msg �ļ��Զ����ɵ�ͷ�ļ���

#include <sstream>


int main(int argc, char **argv)
{
	/*��ʼ��ROS��ָ���ڵ������*/
	ros::init(argc, argv, "C_Prg"); 

	/*Ϊ������̵Ľڵ㴴��һ�������
	��һ��������NodeHandle��Ϊ�ڵ���г�ʼ����
	���һ�����ٵ�NodeHandle������ڵ�ʹ�õ�������Դ��*/
	ros::NodeHandle n;

	//���� master ���ǽ�Ҫ�� chatter���������� �Ϸ��� std_msgs / String ��Ϣ���͵���Ϣ��
	//���� master �ͻ�������ж����� chatter ����Ľڵ㣬��Ҫ�����ݷ�����*/
	//�ڶ��������Ƿ������еĴ�С��������Ƿ�������Ϣ��Ƶ��̫�ߣ��������е���Ϣ�ڴ��� 1000 ����ʱ��ͻῪʼ������ǰ��������Ϣ��
	ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

	//ros::Rate �������������ָ����ѭ����Ƶ�ʡ�����׷�ټ�¼����һ�ε��� Rate::sleep() ��ʱ������ţ�������ֱ��һ��Ƶ�����ڵ�ʱ�䡣	
	ros::Rate loop_rate(10);


	//roscpp ��Ĭ������һ�� SIGINT ������������� Ctrl - C ���̲�������ʹ�� ros::ok() ���� false��
	/*
	�����������֮һ������ros::ok() ����false��

		SIGINT ������(Ctrl - C)
		����һͬ���ڵ��߳� ROS ����
		ros::shutdown() ���������һ���ֵ���

		�ڵ��е����� ros::NodeHandles ���Ѿ�������

		һ�� ros::ok() ���� false, ���е� ROS ���ö���ʧЧ��
		*/	
	int count = 0;
	while (ros::ok())
	{
		/*����ʹ��һ���� msg file �ļ������ġ���Ϣ����Ӧ������ ROS �����й㲥��Ϣ��
		��������ʹ�ñ�׼��String��Ϣ��
		��ֻ��һ�����ݳ�Ա "data"��
		��Ȼ����Ҳ���Է��������ӵ���Ϣ���͡�*/
		chatter_pub.publish(msg);
		ros::spinOnce();

		/*��spinOnce����������ʱ��spinOnce�ͻ���ûص����������е�һ��callback��������ʱcallback�����ű�ִ�У�
			Ȼ��ȵ��´�spinOnce�����ֱ�����ʱ���ص����������еڶ���callback�����ͻᱻ���ã��Դ����ơ�
			���ԣ������һ�����⡣��Ϊ�ص��������еĳ��������޵ģ�����������������ݵ��ٶ�̫�죬spinOnce�������õ�Ƶ��̫�٣��ͻᵼ�¶��������
			һЩcallback�����ͻᱻ����������û��ִ�е���
			������spin������һ������spin���������Ͳ��᷵���ˣ��൱�������Լ��ĺ���������ѭ���ˡ�
			ֻҪ�ص���������������callback�����ڣ����ͻ�����ȥִ��callback���������û�еĻ������ͻ�����������ռ��CPU��*/ 
		loop_rate.sleep();
		++count;
	}
	return 0;
}