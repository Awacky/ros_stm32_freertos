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
} state; // ������һ��ö������ states�����ڱ�ʾ�����˵�ǰ��״̬
static uint8_t resetNodeNum = 0;
/*==================MicroROS���ִ����&�ڵ�===================*/
rcl_node_t node;								// ����һ�� ROS 2 ϵͳ�еĽڵ㣬�����������ڵ�ͨ��
rcl_timer_t timer_pub;							// ������ָ����ʱ������ִ�лص�����
rclc_support_t support;							// ������ ROS 2 �������г�ʼ��������ִ�������ڵ����Դ
rcl_allocator_t allocator;						// ������ ROS 2 �ڵ��з�����ͷ��ڴ�
rclc_executor_t executor;						// �����ڵ����߳��д����� ROS 2 ��Դ�Ļص�������

/*==================MicroROS��Ϣ============================*/
stars_interfaces__msg__Battery bat_msg; 		//���̵�������ϱ�msgs
stars_interfaces__msg__Imu imu_msg; 			//����IMU�����ϱ�msgs
stars_interfaces__msg__Sonar sonar_msg;			//���̳����������ϱ�msgs
stars_interfaces__msg__Velocities vel_msg;		//���̵���ٶ������ϱ�msgs

geometry_msgs__msg__Twist twist_msg;			// �����˵��ٶȿ���ָ��

/*==================MicroROS���ķ����߷���========================*/
rcl_publisher_t     bat_publisher;				// ���ڷ������̵������
rcl_publisher_t     imu_publisher;				// ���ڷ�������IMU����
rcl_publisher_t     sonar_publisher;			// ���ڷ������̳���������
rcl_publisher_t     vel_publisher;				// ���ڷ������̵���ٶ�����
rcl_subscription_t 	twist_subscriber;			// ���ڶ��Ļ����˵��ٶȿ���ָ�Twist��

static uint8_t err_val = 0;
static stcATBuff testBuff_t;

static void devPort_Init(void);
static uint8_t rclcNode_Init(void);
static uint8_t rosNode_Init(void);
static void rosNode_DesInit(void);
static void loop_starRobot_transport(void);

//ע���д�����ͷ������ͷ��ڴ�
static void devPort_Init(void){
    rmw_uros_set_custom_transport(true,NULL,\
                            dev_transport_open,dev_transport_close,\
							dev_transport_write,dev_transport_read);
    rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
    freeRTOS_allocator.allocate = microros_allocate;
    freeRTOS_allocator.deallocate = microros_deallocate;
    freeRTOS_allocator.reallocate = microros_reallocate;
    freeRTOS_allocator.zero_allocate =  microros_zero_allocate;					
    rcutils_set_default_allocator(&freeRTOS_allocator);								// ������ ROS 2 �ڵ��з�����ͷ��ڴ�
}
static uint8_t rclcNode_Init(void)
{
    // ���� rclc_support_init ������ʼ�� ROS 2 ����ʱ��֧�ֿ⣬���� allocator
    allocator = rcl_get_default_allocator();
    rcl_ret_t rc = rclc_support_init(&support, 0, NULL, &allocator);
    if (rc != RCL_RET_OK){
		rclc_support_fini(&support);
		return 0;
    }
    // ���� rclc_node_init_default ������ʼ�� ROS 2 �ڵ㣬����ڵ����ơ������ռ��֧�ֿ�
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
// �����ڶ�ʱ������ʱ���������˵�λ�ú��ٶ���Ϣ
void publisher_callback(rcl_timer_t * timer, int64_t last_call_time){
    (void) last_call_time;
    (void) timer;
    rcl_publish(&bat_publisher, 	&bat_msg, 	NULL);
	rcl_publish(&imu_publisher, 	&imu_msg, 	NULL);
	rcl_publish(&sonar_publisher, 	&sonar_msg, NULL);
	rcl_publish(&vel_publisher, 	&vel_msg, 	NULL);
}

// ���ڴ�����յ���Twist���͵�ROS��Ϣ
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
	// ���� rclc_publisher_init_default ������ʼ�� ROS 2 �����ߣ�����ڵ㡢��Ϣ���ͺ��������ơ�
	rclc_publisher_init_default(&bat_publisher,&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(stars_interfaces, msg, Battery),"battery");
	rclc_publisher_init_default(&imu_publisher,&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(stars_interfaces, msg, Imu),"raw_imu");
	rclc_publisher_init_default(&sonar_publisher,&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(stars_interfaces, msg, Sonar),"sonar");
	rclc_publisher_init_default(&vel_publisher,&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(stars_interfaces, msg, Velocities),"raw_vel");
	
	// ���� rclc_subscription_init_default ������ʼ�� ROS 2 �����ߣ�����ڵ㡢��Ϣ���ͺ��������ơ�
    rclc_subscription_init_default(&twist_subscriber,&node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),"cmd_vel");
	
    // ���� rclc_timer_init_default ������ʼ�� ROS 2 ��ʱ��������֧�ֿ⡢��ʱ�����ںͻص�����
    rclc_timer_init_default(&timer_pub,&support,RCL_MS_TO_NS(5),publisher_callback);
	
    // create executor
    executor = rclc_executor_get_zero_initialized_executor();
	// ���� rclc_executor_init ������ʼ�� ROS 2 ִ����������֧�ֿ⡢ִ�����߳������ڴ������
    rclc_executor_init(&executor, &support.context, 2, &allocator);
	
	// ���� rclc_executor_add_timer ��������ʱ����ӵ�ִ�����У�����ִ�����Ͷ�ʱ����
    rclc_executor_add_timer(&executor, &timer_pub);
	
	// ���� rclc_executor_add_subscription ��������������ӵ�ִ�����У�����ִ�����������ߡ���Ϣ�ͻص�������
    rclc_executor_add_subscription(&executor, &twist_subscriber, &twist_msg, &callback_twist_subscription, ON_NEW_DATA);
	return status_tt;
}
static void rosNode_DesInit(void)
{
    // ��ȡ ROS 2 �������е� RMW �����ģ������丳ֵ�� rmw_context ������
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    // ���� ROS 2 �������е� RMW �����ĵ�ʵ�����ٻỰ��ʱʱ��Ϊ 0������ζ��ʵ�����ٲ������������أ������ǵȴ���ʱ
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
    // RCSOFTCHECK ��һ���궨�壬���ڼ��ִ�к����ķ���ֵ�Ƿ���������������ӡ������Ϣ���˳�����
    // ��������һ�� ROS 2 �����ߣ�Publisher��
    rcl_publisher_fini(&bat_publisher, 	&node);
	rcl_publisher_fini(&imu_publisher, 	&node);
	rcl_publisher_fini(&sonar_publisher,&node);
	rcl_publisher_fini(&vel_publisher, 	&node);
	
    // ��������һ�� ROS 2 �����ߣ�Subscriber��
    rcl_subscription_fini(&twist_subscriber, &node);
    // // ��������һ�� ROS 2 ����Service��
    // rcl_service_fini(&config_service, &node);
    // ��������һ�� ROS 2 ��ʱ����Timer��
    rcl_timer_fini(&timer_pub);
    // ����ִֹͣ������Executor�����ͷ������Դ
    rclc_executor_fini(&executor);
    // ��������һ�� ROS 2 �ڵ㣨Node��
    rcl_node_fini(&node);
    // �����ͷ�֧�ֿ��з������Դ
    rclc_support_fini(&support);
}
static void loop_starRobot_transport(void)
{
    // �������ݵ�ǰ��״ִ̬�в�ͬ�Ĳ���
    switch (state){
        // ����WAITING_AGENT״̬��������ÿ500����ִ��һ��RMW_RET_OK == rmw_uros_ping_agent(100, 1)���
        // �����������MicroROS������ping��Ϣ��������Ƿ��ܹ��յ�pong��Ϣ��
        // ����յ�pong��Ϣ����״̬����ΪAGENT_AVAILABLE�����򱣳ֵȴ�״̬��
        case WAITING_AGENT:{
			configParam.IsRosNodePub = 0;			
            if(RMW_RET_OK == rmw_uros_ping_agent(100, 1)){				
                state = AGENT_AVAILABLE;
            } else {
                 state = WAITING_AGENT;
				
            }
        }break;

        // ����AGENT_AVAILABLE״̬�����������Դ���starRobot���䣬����״̬����ΪAGENT_CONNECTED��
        // ��������ɹ������������AGENT_CONNECTED״̬������״̬����ΪWAITING_AGENT��������starRobot���䡣
        case AGENT_AVAILABLE:{
            if( rosNode_Init()){
                state = AGENT_CONNECTED;
            }
            state = AGENT_CONNECTED;
        }break;
        // ����AGENT_CONNECTED״̬��������ÿ200����ִ��һ��RMW_RET_OK == rmw_uros_ping_agent(100, 1)���
        // �����������MicroROS������ping��Ϣ��������Ƿ��ܹ��յ�pong��Ϣ��
        // ����յ�pong��Ϣ���򱣳�AGENT_CONNECTED״̬��������ͬ��ʱ�䡣
        case AGENT_CONNECTED:{
            if(RMW_RET_OK == rmw_uros_ping_agent(100, 1)){
				configParam.IsRosNodePub = 1;
                state = AGENT_CONNECTED;
				resetNodeNum = 0;
                // ��������rclc_executor_spin_some��������100������ִ��һЩ������� ROS2 ��Ϣ��
                rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
            } else {
				resetNodeNum++;
            }
			if(resetNodeNum>20){
				resetNodeNum = 0;
				state = AGENT_DISCONNECTED;
			}
        }break;
        // ��״̬ΪAGENT_DISCONNECTEDʱ������������starRobot���䲢��״̬����ΪWAITING_AGENT����ʾ�ȴ�MicroROS�������ӡ�
        // AGENT_DISCONNECTED״̬���޷���MicroROS����ͨ�ţ���Ҫ��������
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






























