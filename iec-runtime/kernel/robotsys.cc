#include "robotsys.h"


#define MAX_OBJNAME 50              /* 目标文件名最大字符数 */


SysTaskTree tasktree;               /* 系统任务树 */

RT_HEAP ioconf_heap;                /* I/O配置信息共享区描述符 */
RT_HEAP svconf_heap;                /* 伺服配置信息共享区描述符 */
RT_HEAP rc_heap_desc;               /* RC/PLC共享数据区描述符 */
RT_HEAP sv_heap_desc;               /* 伺服映像数据共享区描述符 */

RT_COND rc_cond_desc;               /* RC/PLC同步对象－－条件变量描述符 */
RT_MUTEX rc_mutex_desc;             /* RC/PLC同步对象－－互斥量描述符 */
RT_MUTEX sv_mutex_desc;             /* PLC/Servo同步对象－－互斥量描述符 */

RT_QUEUE mq_rc_interp_desc;
RT_QUEUE mq_plc_desc;

IOConfig *io_conf;                  /* I/O配置信息共享区指针 */
ServoConfig *sv_conf;               /* 伺服配置信息共享区指针 */
RobotConfig *rc_conf;               /* 机器人配置信息共享区指针 */

IOMem io_shm;                       /* I/O映像区指针 */
SVMem *sv_shm;                      /* 伺服共享区指针 */
RCMem *rc_shm;                      /* RC/PLC共享区指针 */

pid_t io_pid, sv_pid, rc_pid;       /* I/O子任务，伺服子任务，RC子任务进程号 */

char objname[MAX_OBJNAME] = "exec.obj";     /* 目标文件名（PLC程序文件） */

int insert_io_driver(pid_t io_pid) {
    if ((io_pid = fork()) < 0) {
        LOGGER_ERR(E_IOTASK_FORK, "");
    } else if (io_pid == 0) {
        if (execlp("./io-task", "io-task",NULL) < 0) {
            LOGGER_ERR(E_IOTASK_EXEC, "");
        }
    } else {
        return 0;
    }
}

int insert_sv_driver(pid_t sv_pid) {
    if ((sv_pid = fork()) < 0) {
        LOGGER_ERR(E_SVTASK_FORK, "");
    } else if (sv_pid == 0) {
        if (execlp("./sv-task", "sv-task", NULL) < 0) {
            LOGGER_ERR(E_SVTASK_EXEC, "");
        }
    } else {
        return 0;
    }
}

int insert_rc_task(pid_t rc_pid){
    if((rc_pid = fork()) < 0){
        LOGGER_ERR(E_RCTASK_FORK, "");
    } else if(rc_pid == 0) {
        if(execl("../rc-runtime/rc-runtime.exe", "rc-runtime.exe",  NULL) < 0){
            LOGGER_ERR(E_RCTASK_EXEC, "");
        } else {
            printf("OK\n");
        }
    } else {
        return 0;
    }
}

int sockfd;     // 控制器监听套接字描述符
int connfd;     //　连接套接字描述符

int pc_conn_init(){
	struct sockaddr_in server_addr;
	struct sockaddr_in client_addr;

	char buffer[2048];

	if((sockfd = socket(AF_INET, SOCK_STREAM, 0)) == -1){
		fprintf(stderr, "Socker error: %s\n", strerror(errno));
		return -1;
	}

	bzero(&server_addr, sizeof(struct sockaddr_in));
	server_addr.sin_family = AF_INET;
	server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
	server_addr.sin_port = htons(PORTNUMBER);

	if(bind(sockfd, (struct sockaddr*)(&server_addr), sizeof(struct sockaddr)) == -1){
		fprintf(stderr, "Bind error: %s\n\a", strerror(errno));
		return -2;
	}

	if(listen(sockfd, 5) == -1){
		fprintf(stderr, "Listen error: %s\n\a", strerror(errno));
		return -3;
	}

    return 0;
}


int systasktree_init(int mode){
    tasktree.mode = mode;
    tasktree.timer = 0;
    tasktree.task_count = 1;

    tasktree.sys_supervisor_task.task_name = SYS_SUPERVISOR_NAME;
    tasktree.sys_supervisor_task.prio = SYS_SUPERVISOR_PRIORITY;
    tasktree.sys_manager_task.task_name = SYS_MANAGER_NAME;
    tasktree.sys_manager_task.prio = SYS_MANAGER_PRIORITY;

    tasktree.servo_task.task_name = SV_TASK_NAME;
    tasktree.servo_task.prio = SV_TASK_PRIORITY;
    tasktree.io_task.task_name = IO_TASK_NAME;
    tasktree.io_task.prio = IO_TASK_PRIORITY;

    tasktree.rc_task[0].task_name = RC_SUPERVISOR_NAME;
    tasktree.rc_task[0].prio = RC_SUPERVISOR_PRIORITY;
    tasktree.rc_task[1].task_name = RC_MANAGER_NAME;
    tasktree.rc_task[1].prio = RC_MANAGER_PRIORITY;
    tasktree.rc_task[2].task_name = RC_EXECUTOR_NAME;
    tasktree.rc_task[2].prio = RC_EXECUTOR_PRIORITY;

    tasktree.plcmodel = &plc_task;
    return 0;
}


int sys_proj_reload(){

    io_conf_create(&ioconf_heap, &io_conf);     /* 创建I/O配置共享内存区，用于向I/O驱动子任务共享配置信息 */
    sv_conf_create(&svconf_heap, &sv_conf);     /* 创建伺服配置共享内存区，用于向伺服驱动子任务共享配置信息 */

    LOGGER_DBG(DFLAG_LONG, "plc file name = %s", objname);
    load_obj(objname, io_conf, sv_conf, &plc_task);     /* 加载目标文件 */

    io_mem_create(&io_shm, io_conf, M_SHARED);          /* 创建I/O映像区 */
    io_mem_zero(&io_shm, io_conf);                      /* I/O映像区初始化 */

    sv_mem_create(sv_shm, sv_conf);                    /* 创建伺服映像区 */
    sv_mem_init(sv_shm, sv_conf);                      /* 伺服映像区初始化（注意：暂时未实现，为空函数）*/
    sv_syncobj_create(&sv_mutex_desc, SV_MUTEX_NAME);  /* 创建PLC/Servo同步对象 */

    rc_mem_create(rc_shm, rc_conf);                     /* 创建RC与PLC共享内存区 */
    rc_shm_servo_init(rc_shm);                          /* 对RC/PLC内存共享区进行初始化 */
    rc_syncobj_create(&rc_mutex_desc, RC_MUTEX_NAME, &rc_cond_desc, RC_COND_NAME);    /* 创建RC/PLC同步对象 */

    rt_queue_create(&mq_rc_interp_desc, MQ_RC_INTERP_NAME, 50, Q_UNLIMITED, Q_FIFO);
    rt_queue_create(&mq_plc_desc, MQ_PLC_NAME, 50, Q_UNLIMITED, Q_FIFO);

    return 0;
}

int sys_task_start(){

    // insert_io_driver(io_pid);                            /* 加载I/O驱动子任务 */
    insert_sv_driver(sv_pid);                            /* 加载伺服驱动子任务 */
    insert_rc_task(rc_pid);                              /* 加载RC子任务 */
    //
    // /* block until binding successfully */
    // rt_task_bind(&tasktree.io_task.task_desc, IO_TASK_NAME, TM_INFINITE);  /* 任务描述符io_task绑定I/O驱动子任务 */
    rt_task_bind(&tasktree.servo_task.task_desc, SV_TASK_NAME, TM_INFINITE);  /* 任务描述符sv_task绑定伺服驱动子任务 */
    // rt_task_bind(&tasktree.rc_task[0].task_desc, RC_SUPERVISOR_NAME, TM_INFINITE);  /* 任务描述符rc_supervisor_desc绑定RC子任务 */
    // rt_task_bind(&tasktree.rc_task[1].task_desc, RC_MANAGER_NAME, TM_INFINITE);  /* 任务描述符rc_manager_desc绑定RC子任务 */
    // rt_task_bind(&tasktree.rc_task[2].task_desc, RC_EXECUTOR_NAME, TM_INFINITE);  /* 任务描述符rc_executor_desc绑定RC子任务 */

    plc_task_init(&plc_task);                           /* 初始化plc任务 */
    plc_task_start(&plc_task);                          /* 启动plc任务 */

    return 0;
}
