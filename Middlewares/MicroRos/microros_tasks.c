#include "microros_tasks.h"
#include "microros_allocators.h"
#include "STM_custom_transport.h"
#include <time.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include <geometry_msgs/msg/twist.h>
#include <stars_interfaces/msg/battery.h>
#include <stars_interfaces/msg/imu.h>	
#include <stars_interfaces/msg/sonar.h>	
#include <stars_interfaces/msg/velocities.h>	
#include "FreeRTOS.h"
#include "task.h"
#include "moveBase_Task.h"

enum states
{
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
} state; // 定义了一个枚举类型 states，用于表示机器人当前的状态
static uint8_t resetNodeNum = 0;
/*==================MicroROS相关执行器&节点===================*/
rcl_node_t node;								// 代表一个 ROS 2 系统中的节点，用于与其他节点通信
rcl_timer_t timer_pub;							// 用于在指定的时间间隔内执行回调函数
rclc_support_t support;							// 用于在 ROS 2 上下文中初始化和配置执行器、节点等资源
rcl_allocator_t allocator;						// 用于在 ROS 2 节点中分配和释放内存
rclc_executor_t executor;						// 用于在单个线程中处理多个 ROS 2 资源的回调函数。

/*==================MicroROS消息============================*/
stars_interfaces__msg__Battery bat_msg; 		//底盘电池数据上报msgs
stars_interfaces__msg__Imu imu_msg; 			//底盘IMU数据上报msgs
stars_interfaces__msg__Sonar sonar_msg;			//底盘超声波数据上报msgs
stars_interfaces__msg__Velocities vel_msg;		//底盘电机速度数据上报msgs

geometry_msgs__msg__Twist twist_msg;			// 机器人的速度控制指令

/*==================MicroROS订阅发布者服务========================*/
rcl_publisher_t     bat_publisher;				// 用于发布底盘电池数据
rcl_publisher_t     imu_publisher;				// 用于发布底盘IMU数据
rcl_publisher_t     sonar_publisher;			// 用于发布底盘超声波数据
rcl_publisher_t     vel_publisher;				// 用于发布底盘电机速度数据
rcl_subscription_t 	twist_subscriber;			// 用于订阅机器人的速度控制指令（Twist）

static uint8_t err_val = 0;
static stcATBuff testBuff_t;

static void devPort_Init(void);
static uint8_t rclcNode_Init(void);
static uint8_t rosNode_Init(void);
static void rosNode_DesInit(void);
static void loop_starRobot_transport(void);

//注册读写函数和分配与释放内存
static void devPort_Init(void){
    rmw_uros_set_custom_transport(true,NULL,\
                            dev_transport_open,dev_transport_close,\
							dev_transport_write,dev_transport_read);
    rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
    freeRTOS_allocator.allocate = microros_allocate;
    freeRTOS_allocator.deallocate = microros_deallocate;
    freeRTOS_allocator.reallocate = microros_reallocate;
    freeRTOS_allocator.zero_allocate =  microros_zero_allocate;					
    rcutils_set_default_allocator(&freeRTOS_allocator);								// 用于在 ROS 2 节点中分配和释放内存
}
static uint8_t rclcNode_Init(void)
{
    // 调用 rclc_support_init 函数初始化 ROS 2 运行时的支持库，传入 allocator
    allocator = rcl_get_default_allocator();
    rcl_ret_t rc = rclc_support_init(&support, 0, NULL, &allocator);
    if (rc != RCL_RET_OK){
		rclc_support_fini(&support);
		return 0;
    }
    // 调用 rclc_node_init_default 函数初始化 ROS 2 节点，传入节点名称、命名空间和支持库
    rclc_node_init_default(&node, "starsRobot_node", "", &support);
	return 1;
}
void setRos2ImuValue(const stcTwist *srcAcc,const stcTwist *srcAngVel,const stcTwist *srcMag){
	imu_msg.lin_acceleration.x	= srcAcc->linearVelX;
	imu_msg.lin_acceleration.y 	= srcAcc->linearVelY;
	imu_msg.lin_acceleration.z 	= srcAcc->angularVelZ;
	imu_msg.ang_velocity.x 		= srcAngVel->linearVelX;
	imu_msg.ang_velocity.y 		= srcAngVel->linearVelY;
	imu_msg.ang_velocity.z 		= srcAngVel->angularVelZ;
	imu_msg.mag_field.x 		= srcMag->linearVelX;
	imu_msg.mag_field.y 		= srcMag->linearVelY;
	imu_msg.mag_field.z 		= srcMag->angularVelZ;
//	rcl_publish(&imu_publisher, 	&imu_msg, 	NULL);
}
void setRos2BatteryValue(float srcVol,float srcCur,float srcTemp){
	bat_msg.vol 	= srcVol;
	bat_msg.cur 	= srcCur;
	bat_msg.temp 	= srcTemp;
//	rcl_publish(&bat_publisher, 	&bat_msg, 	NULL);
}
void setRos2SonarValue(const SonarDate *srcSonar_p){
	sonar_msg.sonar1 = srcSonar_p->Sonar1;
	sonar_msg.sonar2 = srcSonar_p->Sonar2;
	sonar_msg.sonar3 = srcSonar_p->Sonar3;
	sonar_msg.sonar4 = srcSonar_p->Sonar4;
//	rcl_publish(&sonar_publisher, 	&sonar_msg, NULL);
}
void setRos2VelocityValue(const stcMotorVel *srcVel_p){
	vel_msg.linear_x = srcVel_p->Vel_x;
	vel_msg.linear_y = srcVel_p->Vel_y;
	vel_msg.angular_z = srcVel_p->Vel_z;
	vel_msg.encoder_m1 = srcVel_p->EncM1;
	vel_msg.encoder_m2 = srcVel_p->EncM2;
	vel_msg.encoder_m3 = srcVel_p->EncM3;
	vel_msg.encoder_m4 = srcVel_p->EncM4;
//	rcl_publish(&vel_publisher, 	&vel_msg, 	NULL);
}
// 用于在定时器触发时发布机器人的位置和速度信息
void publisher_callback(rcl_timer_t * timer, int64_t last_call_time){
    (void) last_call_time;
    (void) timer;
    rcl_publish(&bat_publisher, 	&bat_msg, 	NULL);
	rcl_publish(&imu_publisher, 	&imu_msg, 	NULL);
	rcl_publish(&sonar_publisher, 	&sonar_msg, NULL);
	rcl_publish(&vel_publisher, 	&vel_msg, 	NULL);
}

// 用于处理接收到的Twist类型的ROS消息
void callback_twist_subscription(const void *msgin){
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
	stcTwist str_p;
	str_p.linearVelX = msg->linear.x;
	str_p.linearVelY = msg->linear.y;
	str_p.angularVelZ = msg->angular.z;
	twistlinkSendPacket( &str_p);
	
}
static uint8_t rosNode_Init(void){
	uint8_t status_tt = 0;
     // initial configurations 
    if(rclcNode_Init()){
        status_tt = 1;
    }
	// 调用 rclc_publisher_init_default 函数初始化 ROS 2 发布者，传入节点、消息类型和主题名称。
	rclc_publisher_init_default(&bat_publisher,&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(stars_interfaces, msg, Battery),"battery");
	rclc_publisher_init_default(&imu_publisher,&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(stars_interfaces, msg, Imu),"raw_imu");
	rclc_publisher_init_default(&sonar_publisher,&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(stars_interfaces, msg, Sonar),"sonar");
	rclc_publisher_init_default(&vel_publisher,&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(stars_interfaces, msg, Velocities),"raw_vel");
	
	// 调用 rclc_subscription_init_default 函数初始化 ROS 2 订阅者，传入节点、消息类型和主题名称。
    rclc_subscription_init_default(&twist_subscriber,&node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),"cmd_vel");
	
    // 调用 rclc_timer_init_default 函数初始化 ROS 2 定时器，传入支持库、定时器周期和回调函数
    rclc_timer_init_default(&timer_pub,&support,RCL_MS_TO_NS(5),publisher_callback);
	
    // create executor
    executor = rclc_executor_get_zero_initialized_executor();
	// 调用 rclc_executor_init 函数初始化 ROS 2 执行器，传入支持库、执行器线程数和内存分配器
    rclc_executor_init(&executor, &support.context, 2, &allocator);
	
	// 调用 rclc_executor_add_timer 函数将定时器添加到执行器中，传入执行器和定时器。
    rclc_executor_add_timer(&executor, &timer_pub);
	
	// 调用 rclc_executor_add_subscription 函数将订阅者添加到执行器中，传入执行器、订阅者、消息和回调函数。
    rclc_executor_add_subscription(&executor, &twist_subscriber, &twist_msg, &callback_twist_subscription, ON_NEW_DATA);
	return status_tt;
}
static void rosNode_DesInit(void)
{
    // 获取 ROS 2 上下文中的 RMW 上下文，并将其赋值给 rmw_context 变量。
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    // 设置 ROS 2 上下文中的 RMW 上下文的实体销毁会话超时时间为 0，这意味着实体销毁操作将立即返回，而不是等待超时
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
    // RCSOFTCHECK 是一个宏定义，用于检查执行函数的返回值是否出错，如果出错，则会打印错误信息并退出程序。
    // 用于销毁一个 ROS 2 发布者（Publisher）
    rcl_publisher_fini(&bat_publisher, 	&node);
	rcl_publisher_fini(&imu_publisher, 	&node);
	rcl_publisher_fini(&sonar_publisher,&node);
	rcl_publisher_fini(&vel_publisher, 	&node);
	
    // 用于销毁一个 ROS 2 订阅者（Subscriber）
    rcl_subscription_fini(&twist_subscriber, &node);
    // // 用于销毁一个 ROS 2 服务（Service）
    // rcl_service_fini(&config_service, &node);
    // 用于销毁一个 ROS 2 定时器（Timer）
    rcl_timer_fini(&timer_pub);
    // 用于停止执行器（Executor）并释放相关资源
    rclc_executor_fini(&executor);
    // 用于销毁一个 ROS 2 节点（Node）
    rcl_node_fini(&node);
    // 用于释放支持库中分配的资源
    rclc_support_fini(&support);
}
static void loop_starRobot_transport(void)
{
    // 函数根据当前的状态执行不同的操作
    switch (state){
        // 对于WAITING_AGENT状态，函数会每500毫秒执行一次RMW_RET_OK == rmw_uros_ping_agent(100, 1)语句
        // 该语句用于向MicroROS代理发送ping消息，并检查是否能够收到pong消息。
        // 如果收到pong消息，则将状态设置为AGENT_AVAILABLE；否则保持等待状态。
        case WAITING_AGENT:{
			configParam.IsRosNodePub = 0;			
            if(RMW_RET_OK == rmw_uros_ping_agent(100, 1)){				
                state = AGENT_AVAILABLE;
            } else {
                 state = WAITING_AGENT;
				
            }
        }break;

        // 对于AGENT_AVAILABLE状态，函数将尝试创建starRobot传输，并将状态设置为AGENT_CONNECTED。
        // 如果创建成功，则继续保持AGENT_CONNECTED状态；否则将状态设置为WAITING_AGENT，并销毁starRobot传输。
        case AGENT_AVAILABLE:{
            if( rosNode_Init()){
                state = AGENT_CONNECTED;
            }
            state = AGENT_CONNECTED;
        }break;
        // 对于AGENT_CONNECTED状态，函数会每200毫秒执行一次RMW_RET_OK == rmw_uros_ping_agent(100, 1)语句
        // 该语句用于向MicroROS代理发送ping消息，并检查是否能够收到pong消息。
        // 如果收到pong消息，则保持AGENT_CONNECTED状态，并尝试同步时间。
        case AGENT_CONNECTED:{
            if(RMW_RET_OK == rmw_uros_ping_agent(100, 1)){
				configParam.IsRosNodePub = 1;
                state = AGENT_CONNECTED;
				resetNodeNum = 0;
                // 函数调用rclc_executor_spin_some函数，在100毫秒内执行一些待处理的 ROS2 消息。
                rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
            } else {
				resetNodeNum++;
            }
			if(resetNodeNum>20){
				resetNodeNum = 0;
				state = AGENT_DISCONNECTED;
			}
        }break;
        // 当状态为AGENT_DISCONNECTED时，函数将销毁starRobot传输并将状态设置为WAITING_AGENT，表示等待MicroROS代理连接。
        // AGENT_DISCONNECTED状态下无法与MicroROS代理通信，需要重新连接
        case AGENT_DISCONNECTED:{
            rosNode_DesInit();
            state = WAITING_AGENT;
        }break;
        default:{
            state = WAITING_AGENT;
        }break;
    }
}
static void ros2_node_init(void *pvParameters){	
	devPort_Init();
//	rosNode_Init();
	uint8_t InitDow = 1;
	for(;;){
		if(InitDow){
			vTaskDelay(500);
			InitDow = 0;
		}
//		loop_starRobot_transport();
		vTaskDelay(10);
	}
}
void ros2_task_create(void)
{	
	if(configParam.ROSType){
		xTaskCreate(ros2_node_init,   (const char *)"ros2NodeTask",    	1024, NULL, RosSub_Pri, NULL);
	}
}






























