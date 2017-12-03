#ifndef __PLC_H__
#define __PLC_H__

#include <native/task.h>
#include <native/heap.h>
#include <native/mutex.h>
#include <native/cond.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#define ROBOT_AXIS_COUNT 6          /* 机器人轴个数 */
#define CIRCULAR_INTERP_QUEUE_SIZE 10   /* 环形插补队列大小 */

extern RT_HEAP rc_heap_desc;
extern RT_COND rc_cond_desc;
extern RT_MUTEX rc_mutex_desc;



/*-----------------------------------------------------------------------------
 * Robot Configuration
 *---------------------------------------------------------------------------*/
#define MAX_AXIS_COUNT 10
enum RobotModel{
    ESTUN_ER4,
    EFFUTE
};

typedef struct {
    double Theta;
    double d;
    double a;
    double Alpha;
    double offset;
} DH_param;

typedef struct {
    double vellim;
    double acclim;
    double pos_min;
    double pos_max;
} Roblim_param;

typedef struct {
    DH_param DH_p;
    Roblim_param Lim_p;
} Robot_param;

typedef struct {
    int axis_count;
    RobotModel robot_type ;
    Robot_param Axis[MAX_AXIS_COUNT];
    double transTool[4][4];
} RobotConfig;

 /*-----------------------------------------------------------------------------
  * RC PLC Shared Memory Data Structure
  *---------------------------------------------------------------------------*/
struct SingleInterpData{        /* 单轴插补值,来自RC */
    double command_pos;         /* 目标位置 */
    double command_vel;         /* 目标速度 */
    double command_acc;         /* 目标加速度 */
} ;

struct SingleAxisInfo{          /* 单轴实际位置值,来自PLC  */
	double actual_pos;            /* 实际位置 */
    double actual_vel;          /* 实际速度 */
    double actual_acc;          /* 实际加速度 */
} ;

struct RobotAxisActualInfo{     /* 机器人各个轴的信息 */
	int size ;                    /* 机器人轴个数 */
	SingleAxisInfo axis_info[MAX_AXIS_COUNT];    /* 对应各个轴的位置，速度，加速度信息 */
} ;

struct RobotInterpData{         /* 机器人各个轴的插补值 */
	int size;			          /* 机器人轴个数 */
	SingleInterpData interp_value[MAX_AXIS_COUNT];   /* 对应各个轴的插补值 */
} ;

struct CircularInterpQueue{/* 环形插补队列 */
	int queue_size;          /* 环形插补队列大小 */
	int head;                /* 环形队列头指针（实际是data[]数组的索引，即0～queue_size-1之间），RC由此写入插补值 */
	int tail;                /* 环形队列尾指针（实际是data[]数组的索引，即0～queue_size-1之间），PLC由此读出插补值 */
	RobotInterpData data[CIRCULAR_INTERP_QUEUE_SIZE];  /* 环形队列中存放插补值的实际数组 */
} ;

struct RCMem{                              /* RC与PLC共享内存数据结构 */
  uint8_t permit_flag;                        
  uint8_t rc_mode;
  uint8_t interp_startup_flag;
  uint8_t servo_poweron_flag;
  uint32_t sys_task_statusword;

	RobotAxisActualInfo actual_info;         /* 机器人各个轴实际位置，速度，加速度值 */
	CircularInterpQueue interp_queue;        /* 机器人插补值队列 */
  RobotInterpData interp_data;
} ;

extern RCMem *rc_shm;						/* RC与PLC共享内存区指针 */

/*-----------------------------------------------------------------------------
 * RC Shared Memory　Create Operation Funcions
 *---------------------------------------------------------------------------*/

 #define RC_MEM_NAME "rc_mem"
 inline void rc_mem_create(RCMem *&rcmem, RobotConfig *config) {
 	int size = sizeof(RCMem);
 	int ret = 0;
 	if ((ret = rt_heap_create(&rc_heap_desc, RC_MEM_NAME, sizeof(RCMem), H_SHARED)) < 0) {

    }
    /* MUST called from realtime context (REF: Xenomai API) */
    if ((ret = rt_heap_alloc(&rc_heap_desc, sizeof(RCMem), TM_INFINITE, (void **)&rcmem)) < 0) {

    }
 }

 inline void rc_mem_bind(RCMem *&rcmem, RobotConfig *config) {
 	int size = sizeof(RCMem);
 	int ret = 0;
 	if (rt_heap_bind(&rc_heap_desc, RC_MEM_NAME, TM_INFINITE) < 0) {

    }
    if (rt_heap_alloc(&rc_heap_desc, 0, TM_NONBLOCK, (void **)&rcmem) < 0) {

    }
 }

inline void rc_mem_unbind(RCMem *rcmem, RobotConfig *config) {
    int size = sizeof(RCMem);
   	int ret = 0;
    if (rt_heap_unbind(&rc_heap_desc) < 0) {

    }
}

/*-----------------------------------------------------------------------------
 * RC Shared Memory Sync Operation Funcions
 *---------------------------------------------------------------------------*/
 #define RC_MUTEX_NAME  "rc_mutex"
 #define RC_COND_NAME   "rc_cond"

 inline void rc_syncobj_bind(RT_MUTEX *mutex_desc, const char* mutex_name, RT_COND *cond_desc, const char* cond_name){
     if(rt_mutex_bind(mutex_desc, mutex_name, TM_INFINITE) < 0){
         printf("mutex bind error !\n");
     }
     if(rt_cond_bind(cond_desc, cond_name, TM_INFINITE) < 0){
         printf("cond bind error !\n");
     }
 }

void rc_shm_servo_write(RCMem *rc_shm, RobotInterpData *axis_command_info);
void rc_shm_servo_read(RCMem *rc_shm, RobotAxisActualInfo *axis_actual_info);

#endif
