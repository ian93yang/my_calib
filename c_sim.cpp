#include "ros/ros.h"            //引用了Ros中大部分的头文件
#include "std_msgs/String.h"  //这引用了 std_msgs/String 消息, 它存放在 std_msgs package 里，是由 String.msg 文件自动生成的头文件。

#include <sstream>


int main(int argc, char **argv)
{
	/*初始化ROS，指定节点的名称*/
	ros::init(argc, argv, "C_Prg"); 

	/*为这个进程的节点创建一个句柄。
	第一个创建的NodeHandle会为节点进行初始化，
	最后一个销毁的NodeHandle会清理节点使用的所有资源。*/
	ros::NodeHandle n;

	//告诉 master 我们将要在 chatter（话题名） 上发布 std_msgs / String 消息类型的消息。
	//这样 master 就会告诉所有订阅了 chatter 话题的节点，将要有数据发布。*/
	//第二个参数是发布序列的大小。如果我们发布的消息的频率太高，缓冲区中的消息在大于 1000 个的时候就会开始丢弃先前发布的消息。
	ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

	//ros::Rate 对象可以允许你指定自循环的频率。它会追踪记录自上一次调用 Rate::sleep() 后时间的流逝，并休眠直到一个频率周期的时间。	
	ros::Rate loop_rate(10);


	//roscpp 会默认生成一个 SIGINT 句柄，它负责处理 Ctrl - C 键盘操作——使得 ros::ok() 返回 false。
	/*
	如果下列条件之一发生，ros::ok() 返回false：

		SIGINT 被触发(Ctrl - C)
		被另一同名节点踢出 ROS 网络
		ros::shutdown() 被程序的另一部分调用

		节点中的所有 ros::NodeHandles 都已经被销毁

		一旦 ros::ok() 返回 false, 所有的 ROS 调用都会失效。
		*/	
	int count = 0;
	while (ros::ok())
	{
		/*我们使用一个由 msg file 文件产生的『消息自适应』类在 ROS 网络中广播消息。
		现在我们使用标准的String消息，
		它只有一个数据成员 "data"。
		当然，你也可以发布更复杂的消息类型。*/
		chatter_pub.publish(msg);
		ros::spinOnce();

		/*当spinOnce函数被调用时，spinOnce就会调用回调函数队列中第一个callback函数，此时callback函数才被执行，
			然后等到下次spinOnce函数又被调用时，回调函数队列中第二个callback函数就会被调用，以此类推。
			所以，这会有一个问题。因为回调函数队列的长度是有限的，如果发布器发送数据的速度太快，spinOnce函数调用的频率太少，就会导致队列溢出，
			一些callback函数就会被挤掉，导致没被执行到。
			而对于spin函数，一旦进入spin函数，它就不会返回了，相当于它在自己的函数里面死循环了。
			只要回调函数队列里面有callback函数在，它就会马上去执行callback函数。如果没有的话，它就会阻塞，不会占用CPU。*/ 
		loop_rate.sleep();
		++count;
	}
	return 0;
}