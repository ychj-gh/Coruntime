#ifndef __ROBOTSYS_H__
#define __ROBOTSYS_H__


#include <native/task.h>
#include <native/heap.h>
#include <native/cond.h>
#include <native/mutex.h>
#include <native/timer.h>
#include <native/queue.h>

#include <string>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <netdb.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <arpa/inet.h>

#include "io.h"
#include "servo.h"
#include "rc.h"
#include "plcmodel.h"
#include "loader.h"
#include "vm.h"



extern RT_HEAP ioconf_heap;                /* I/O配置信息共享区描述符 */
extern RT_HEAP svconf_heap;                /* 伺服配置信息共享区描述符 */
extern RT_HEAP rc_heap_desc;               /* RC/PLC共享数据区描述符 */
extern RT_HEAP sv_heap_desc;               /* 伺服映像数据共享区描述符 */

extern RT_COND rc_cond_desc;               /* RC/PLC同步对象－－条件变量描述符 */
extern RT_MUTEX rc_mutex_desc;             /* RC/PLC同步对象－－互斥量描述符 */
extern RT_MUTEX sv_mutex_desc;             /* PLC/Servo同步对象－－互斥量描述符 */

extern IOConfig *io_conf;                  /* I/O配置信息共享区指针 */
extern ServoConfig *sv_conf;               /* 伺服配置信息共享区指针 */
extern RobotConfig *rc_conf;               /* 机器人配置信息共享区指针 */

extern IOMem io_shm;                       /* I/O映像区指针 */
extern SVMem *sv_shm;                      /* 伺服共享区指针 */
extern RCMem *rc_shm;                      /* RC/PLC共享区指针 */

extern pid_t io_pid; 						/* I/O子任务进程号 */
extern pid_t sv_pid; 						/* 伺服子任务进程号 */
extern pid_t rc_pid;       					/* RC子任务进程号 */


#define SYS_SUPERVISOR_NAME "sys_supervisor_task"   /* 系统监控器任务名 */
#define SYS_SUPERVISOR_PRIORITY 99					/* 系统监控器任务优先级 */
#define SYS_SUPERVISOR_PERIODE  1000000000

#define SYS_MANAGER_NAME "sys_manager_task"   		/* 系统管理任务名 */
#define SYS_MANAGER_PRIORITY 98						/* 系统管理任务优先级 */

#define MAX_PLC_TASK_COUNT 16						/* 最大PLC虚拟机任务 */
#define RC_TASK_COUNT 3								/* RC任务数 */

struct TaskInfo{
	std::string task_name;			/* 任务名 */
	RT_TASK task_desc;				/* 任务描述符 */
	RT_TASK_INFO task_info;			/* 任务状态信息 */
	int prio;						/* 任务优先级 */
};

struct SysTaskTree{
	uint8_t	mode;					/* 系统模式：AUTORUN、DEBUG */
	uint64_t timer;					/* 系统定时器计数值 */
	uint8_t task_count;           	/* 当前系统任务数 */
	TaskInfo sys_supervisor_task;   /* 系统监控器任务 */
	TaskInfo sys_manager_task;		/* 运行管理任务 */
	TaskInfo servo_task;			/* 伺服驱动任务 */
	TaskInfo io_task;				/* I/O驱动任务 */
	TaskInfo rc_task[RC_TASK_COUNT];/* RC任务 */
	TaskList *plcmodel;				/* PLC运行镜像 */
};


extern SysTaskTree tasktree;
extern TaskList plc_task;                  /* PLC任务列表 */

int systasktree_init(int mode);				// 系统任务树初始化


#define PORTNUMBER  8888  // 与PC连接端口号

int insert_io_driver(pid_t io_pid);			// 加载io子进程
int insert_sv_driver(pid_t sv_pid);			// 加载伺服子进程
int insert_rc_task(pid_t rc_pid);			// 加载rc子进程

extern int sockfd;     						// 控制器监听套接字描述符
extern int connfd;     						// 连接套接字描述符
int pc_conn_init();							// pc连接初始化

int sys_proj_reload();						// 系统工程重新加载
int sys_task_start();						// 任务启动


#endif
