#include "plc.h"
#include <exception>

#include <native/task.h>
#include <native/timer.h>

#include <signal.h>
#include <unistd.h>
#include <setjmp.h>  				// 用于设置长跳转，实现解释器运行控制
#include "opmanager.hh"

#include "RCInterpreter.h"
#include "RSIExecutor.h"


RT_TASK rc_supervisor_desc;							/* RC监控器任务描述符 */
RT_TASK_INFO  rc_supervisor_info;					/* RC监控器任务状态 */

RT_TASK rc_manager_desc;							/* RC管理器任务描述符 */
RT_TASK_INFO  rc_manager_info;						/* RC管理器任务状态 */

RT_TASK rc_interp_desc;								/* RC interp任务描述符 */
RT_TASK_INFO  rc_interp_info;						/* RC interp任务状态 */

RT_TASK rc_rsi_desc;								/* RSI任务描述符 */
RT_TASK_INFO  rc_rsi_info;							/* RSI任务状态 */

RT_TASK rc_executor_desc;							/* RC执行器任务描述符 */
RT_TASK_INFO  rc_executor_info;						/* RC执行器任务状态 */



RCMem *rc_shm;							/* RC与PLC共享内存区指针 */
RT_HEAP rc_heap_desc;					/* 共享内存区描述符 */

RobotConfig *rc_conf;					/* 机器人配置信息变量指针 */
RobotConfig  rc_runtime_param;			/* 机器人runtime parameters */	

RT_COND rc_cond_desc;               	/* RC/PLC同步对象－－条件变量描述符 */
RT_MUTEX rc_mutex_desc;             	/* RC/PLC同步对象－－互斥量描述符 */

RCOperationCtrl rc_core;				/* RC运行状态机 */

jmp_buf exec_startpoint;				/* 解释执行器起点 */
jmp_buf interp_startpoint;				/* RC插补器起点 */
jmp_buf rsi_startpoint;					/* RSI实时任务起点 */

RT_QUEUE mq_rc_exec_desc;        	    /* 解释器 消息队列描述符 */
RT_QUEUE mq_rc_manager_desc;        	/* 运行管理器 消息队列描述符 */
RT_QUEUE mq_rc_interp_desc;				/* RC插补器 消息队列描述符 */
RT_QUEUE mq_plc_desc;					/* PLC 消息队列描述符 */	
RT_QUEUE mq_rc_rsi_desc;				/* RSI 消息队列描述符 */

ROBOT_INST robot_inst_buffer_code;		/* inst buffer */
bool robot_inst_buffer_flag = false;	/* inst buffer flag */
RT_COND inst_cond_desc;               	/* 同步对象－－条件变量描述符 */
RT_MUTEX inst_mutex_desc;             	/* 同步对象－－互斥量描述符 */


/**
 * 函数名：rsi_routine
 * 函数功能：RC RSI实时任务
 * 参数：cookie  用户给定参数
 * 返回值：无
 */
void rsi_routine(void *cookie) {
	std::cout << "rsi start" << std::endl;
	
	rt_queue_flush(&mq_rc_rsi_desc);

	std::string *rsiFileName = (std::string*)cookie;
	/* create a RSI Executor according to file name */
	RSIExecutor rsiExec(*rsiFileName);

 	/* compile the specified rsi file to obtain address space and code shadow */
 	try {
		rsiExec.compile();
	} catch(rc_exception &e) {
		e.what();
	} catch(std::exception &e) {
		std::cout << "C++ runtime exception" << std::endl;
	}
	/* set RSIStopFlag as false to start to running RSI */
	RSIStopFlag = false;
	/* set RSI executor running period */
	// rt_task_set_periodic(NULL, TM_NOW, RSI_RUN_PERIOD);
	int jstatus = setjmp(rsi_startpoint);
	/* start running RSI periodic */
	while(!RSIStopFlag){
		// rt_task_wait_period(NULL);
		try {
			RTIME start = rt_timer_read();
			rsiExec.execute();
			RTIME end = rt_timer_read();
			std::cout << "RSI cost " << end - start << " ns" << std::endl;
			if(!RSIStopFlag) {
				rsi_step();
			}
		} catch(rc_exception &e) {
			e.what();
			RSIStopFlag = true;
		} catch(std::exception &e) {
			std::cout << "C++ runtime exception" << std::endl;
			RSIStopFlag = true;
		}
	}
}



/**
 * 函数名：interp_routine
 * 函数功能：RC插补器实时任务
 * 参数：cookie  用户给定参数
 * 返回值：无
 */
static void interp_routine(void *cookie){

	char** argv = (char**)cookie;

	int jstatus = setjmp(interp_startpoint);
	while(1){
		fprintf(stderr,"\n|<< ------------------------------ interp start ---------------------- >>|\n");
		rc_core.interp_status = 0;
		// waiting for order input
		// read order
		rc_core.jog_mode = 0;
		ROBOT_INST temp_inst;

		inst_buffer_read(temp_inst);
		
		rc_core.interp_status = 1;
		rc_shm->interp_startup_flag = 1;
		// waiting for start interp msg

		interp_compute(temp_inst);

		if(rc_core.jog_mode == 1) {
			rc_core.jog_mode = 0; // clear jog mode
		} else {
			send_cmd_to_exec(CMD_NEXT);
		}
		
		rc_shm->interp_startup_flag = 0;

		// TODO: send NEXT msg to exec
		rt_task_sleep(1000000);
	}
}

/**
 * 函数名：executor_routine
 * 函数功能：RC解释执行器实时任务
 * 参数：cookie  用户给定参数
 * 返回值：无
 */
static void executor_routine(void *cookie){

	char** argv = (char**)cookie;

	while(1){
		int jstatus = setjmp(exec_startpoint);
		rc_core.exec_run_mode = 0;
		if(rc_core.startup) { 			/* 判断是否启动运行 */
			rc_core.exec_run_mode = 1;

			RCInterpreter executor(rc_core.cur_project, rc_core.cur_program);

			try{
				executor.compile();
				STEPCHECK(1);
				executor.execute();
			} catch(rc_exception &e) {
				e.what();
				rc_core.startup = 0;
				longjmp(exec_startpoint, 0);        /* 跳转至解释器起点 */
			} catch(std::exception &e) {
				std::cout << "C++ runtime exception" << std::endl;
				rc_core.startup = 0;
				longjmp(exec_startpoint, 0);        /* 跳转至解释器起点 */
			}
		}
		rt_task_sleep(1000000);
	}
}

/**
 * 函数名：supervisor_routine
 * 函数功能：RC监控器实时任务
 * 参数：cookie  用户给定参数
 * 返回值：无
 */
static void supervisor_routine(void *cookie){
	rt_task_set_periodic(NULL, TM_NOW, RC_SUPERVISOR_PERIODE*10);
	while(1) {
		rt_task_wait_period(NULL);
		// std::cout << "supervisor_routine" << std::endl;
		double count = 0;
		RTIME start = rt_timer_read();
		// for(int i = 0; i < 10000; i ++) {
		// 	count *= 1.5 * i;
		// }
		//std::cout << "rc_core.startup = " << rc_core.startup << std::endl;
		RTIME end = rt_timer_read();
		// std::cout << "it cost " << end - start << " ns" << std::endl;
		// std::cout << std::to_string(1.23) << std::endl;
		// std::cout << std::stod("1.235") << std::endl;
	}
}

/**
 * 函数名：supervisor_routine
 * 函数功能：RC运行控制器实时任务，
 * 参数：cookie  用户给定参数
 * 返回值：无
 */
static void manager_routine(void *cookie){
	int err = 0;
	
	init_runtime_param();

	rc_mem_bind(rc_shm, rc_conf);			/* rc_shm绑定共享内存区地址 */
	rc_syncobj_bind(&rc_mutex_desc, RC_MUTEX_NAME, &rc_cond_desc, RC_COND_NAME);    /* 绑定RC/PLC同步对象 */

	rt_queue_bind(&mq_rc_interp_desc, MQ_RC_INTERP_NAME, TM_INFINITE);
	rt_queue_bind(&mq_plc_desc, MQ_PLC_NAME, TM_INFINITE);

	rt_queue_create(&mq_rc_exec_desc, MQ_RC_EXEC_NAME, 50, Q_UNLIMITED, Q_FIFO);
	rt_queue_create(&mq_rc_manager_desc, MQ_RC_MANAGER_NAME, 50, Q_UNLIMITED, Q_FIFO);
	rt_queue_create(&mq_rc_rsi_desc, MQ_RC_RSI_NAME, 50, 1, Q_FIFO);

	rt_mutex_create(&inst_mutex_desc, INST_MUTEX_NAME);
	rt_cond_create(&inst_cond_desc, INST_COND_NAME);

	err = rt_task_create(&rc_executor_desc, RC_EXECUTOR_NAME, 0, RC_EXECUTOR_PRIORITY, T_JOINABLE|T_FPU|T_CPU(1));
	err = rt_task_create(&rc_supervisor_desc, RC_SUPERVISOR_NAME, 0, RC_SUPERVISOR_PRIORITY, T_JOINABLE|T_CPU(1));
	err = rt_task_create(&rc_interp_desc, RC_INTERP_NAME, 0, RC_INTERP_PRIORITY, T_JOINABLE|T_FPU|T_CPU(1));
	// err = rt_task_create(&rc_rsi_desc, RC_RSI_NAME, 0, RC_RSI_PRIORITY, T_JOINABLE|T_FPU);
	if(!err){
		rt_task_start(&rc_executor_desc, &executor_routine, NULL);
		rt_task_start(&rc_interp_desc, &interp_routine, NULL);
		rt_task_start(&rc_supervisor_desc, &supervisor_routine, NULL);
		// rt_task_start(&rc_rsi_desc, &rsi_routine, NULL);
	}

	rc_core_init();					// rc状态信息初始化

	start_client_service(0, 0);   	// listen to teachbox
}

void cleanup(void){
	rt_task_delete(&rc_interp_desc);
	rt_task_delete(&rc_executor_desc);
	rt_task_delete(&rc_manager_desc);
	rt_task_delete(&rc_supervisor_desc);
	rt_task_delete(&rc_rsi_desc);

	rt_queue_delete(&mq_rc_exec_desc);
	rt_queue_delete(&mq_rc_manager_desc);
	rt_queue_delete(&mq_rc_rsi_desc);
}

void sig_handler(int signo){
	if(signo == SIGUSR1){
	    printf("Receive Signal No: %d \n", signo);
	    cleanup();
	    exit(0);
  	} else {
  		exit(0);
	}
}


int main(int argc, char **argv) {

	signal(SIGINT, sig_handler);
	signal(SIGUSR1, sig_handler);
	printf("RC Start ...\n");

	int err = rt_task_create(&rc_manager_desc, RC_MANAGER_NAME, 0, RC_MANAGER_PRIORITY, T_JOINABLE|T_CPU(1));

	if(!err){
		rt_task_start(&rc_manager_desc, &manager_routine, argv);
	}

	// /* create a RSI Executor according to file name */
	// RSIExecutor rsiExec("rsidemo");
 // 	/* compile the specified rsi file to obtain address space and code shadow */
 // 	try {
	// 	rsiExec.compile();
	// } catch(rc_exception &e) {
	// 	e.what();
	// } catch(std::exception &e) {
	// 	std::cout << "C++ runtime exception" << std::endl;
	// }
	// /* set RSIStopFlag as false to start to running RSI */
	// RSIStopFlag = false;
	// /* set RSI executor running period */
	// rt_task_set_periodic(NULL, TM_NOW, RSI_RUN_PERIOD);
	// /* start running RSI periodic */
	// while(!RSIStopFlag){
	// 	rt_task_wait_period(NULL);
	// 	try {
	// 		rsiExec.execute();
	// 	} catch(rc_exception &e) {
	// 		e.what();
	// 		RSIStopFlag = true;
	// 	} catch(std::exception &e) {
	// 		std::cout << "C++ runtime exception" << std::endl;
	// 		RSIStopFlag = true;
	// 	}
	// }

	pause();

	return 0;
}
