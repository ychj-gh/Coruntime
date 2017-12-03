#include <sys/mman.h> /* required for mlockall() */
#include <linux/input.h> /* required for input event */
#include <stdio.h>
#include <unistd.h> /* required for fork/exec */
#include <fcntl.h>  /* required for open */
#include <signal.h>
#include <native/queue.h>
#include <native/heap.h>

#include <iostream>

#include "loader.h"
#include "mq.h"
#include "vm.h"
#include "io.h"
#include "servo.h"
#include "logger.h"
#include "rc.h"
#include  "robotsys.h"




TaskList plc_task;                  /* PLC任务列表 */

void sig_handler(int signo) {
    LOGGER_DBG(DFLAG_SHORT, "PLC Kernel Received Signal: %d", (int)signo);
    if (signo == SIGINT) {
        printf("cp 1\n");
        /* ORDER SENSITIVE */
        // if (kill(io_pid, SIGUSR1) == -1)
        // {
        //     printf("send signal to io-task error !\n");
        // }

        // printf("cp 2\n");
        // if (kill(sv_pid, SIGUSR1) == -1)
        // {
        //     printf("send signal to sv-task error !\n");
        // }

        // if (kill(rc_pid, SIGUSR1) == -1)
        // {
        //     printf("send signal to sv-task error !\n");
        // }
        printf("cp 3\n");
        plc_task_delete(&plc_task);
        rt_task_delete(&tasktree.sys_supervisor_task.task_desc);
        rt_task_delete(&tasktree.sys_manager_task.task_desc);
        io_mem_delete(&io_shm, io_conf);
        rt_task_unbind(&tasktree.io_task.task_desc);
        rt_task_unbind(&tasktree.servo_task.task_desc);
        for(int i = 0; i < RC_TASK_COUNT; i ++) {
            rt_task_unbind(&tasktree.rc_task[i].task_desc);
        }

        io_conf_delete(&ioconf_heap);
        sv_conf_delete(&svconf_heap);
        // rc_syncobj_delete(&rc_mutex_desc, &rc_cond_desc);
        printf("exit sys\n");
        exit(0);
    } else {

    }
}



RT_TASK test_desc;
RobotInterpData axis_command_info;
void test_routine(void *cookie){
    rt_task_set_periodic(NULL, TM_NOW, 8000000);
    int cnt = 1;

    axis_command_info.size = 6;

    while(1){
        rt_task_wait_period(NULL);
        rc_shm_servo_read(rc_shm, &axis_command_info);
        printf("cnt: %d\n", cnt++);
        for(int i = 0; i < 6; i ++){
            printf("plc axis %d, pos: %f, vel: %f, acc: %f\n", i,axis_command_info.interp_value[i].command_pos, axis_command_info.interp_value[i].command_vel, axis_command_info.interp_value[i].command_acc);
            // fflush(stdin);
        }
    }
}

static void sys_supervisor_routine(void *cookie) {
    rt_task_set_periodic(NULL, TM_NOW, SYS_SUPERVISOR_PERIODE);
	while(1) {

		rt_task_wait_period(NULL);

        RTIME start = rt_timer_read();
        tasktree.timer ++;
        rt_task_inquire(&tasktree.sys_supervisor_task.task_desc, &tasktree.sys_supervisor_task.task_info);
        rt_task_inquire(&tasktree.sys_manager_task.task_desc, &tasktree.sys_manager_task.task_info);
        rt_task_inquire(&tasktree.servo_task.task_desc, &tasktree.servo_task.task_info);
        rt_task_inquire(&tasktree.io_task.task_desc, &tasktree.io_task.task_info);
        for(int i = 0; i < RC_TASK_COUNT; i ++) {
            rt_task_inquire(&tasktree.rc_task[i].task_desc, &tasktree.rc_task[i].task_info);
        }
        for(int i = 0; i < tasktree.plcmodel->task_count; i ++) {
            rt_task_inquire(&tasktree.plcmodel->rt_task[i], &tasktree.plcmodel->rt_info[i]);
        }
        RTIME end = rt_timer_read();
        // fprintf(stderr, "sys_supervisor_routine: %d ns\n", end - start);
    }
}

static int initflag = 0;        /* 初始化标志 */

static void sys_manager_routine(void *cookie) {
    int err = 0;
    err = rt_task_create(&tasktree.sys_supervisor_task.task_desc, SYS_SUPERVISOR_NAME, 0, SYS_SUPERVISOR_PRIORITY, T_JOINABLE);
    err = sys_proj_reload();        // 系统工程重新加载,建立所有共享内存
    if(!err){
		rt_task_start(&tasktree.sys_supervisor_task.task_desc, &sys_supervisor_routine, NULL);
	}
    err = sys_task_start();         // 启动所有子进程任务

    err = pc_conn_init();           // PC连接初始化，开始监听端口
    pause();
    rt_task_set_periodic(NULL, TM_NOW, SYS_SUPERVISOR_PERIODE);
    while(1) {
        rt_task_wait_period(NULL);
        fprintf(stderr, "sys_manager_routine\n");
        std::string cmd;
        std::cin >> cmd;
        if(cmd == "1"){
            raise(SIGINT);
        }
    }

    if(!err){
        int sin_size = sizeof(struct sockaddr_in);
        struct sockaddr_in client_addr;
    	while((connfd = accept(sockfd, (struct sockaddr*)(&client_addr),(socklen_t*)&sin_size)) == -1){

    	}
    }
}

int main(int argc, char *argv[]) {
	//Avoids memory swapping for this program
    mlockall(MCL_CURRENT|MCL_FUTURE);

    if (signal(SIGINT, &sig_handler) == SIG_ERR) {
        LOGGER_ERR(E_SIG_PLCKILL, "");
    }

    int err = systasktree_init(0);

    err = rt_task_create(&tasktree.sys_manager_task.task_desc, SYS_MANAGER_NAME, 0, SYS_MANAGER_PRIORITY, T_JOINABLE|T_CPU(3));
	if(!err){
		rt_task_start(&tasktree.sys_manager_task.task_desc, &sys_manager_routine, NULL);
	}
    pause();

    return 0;
}
