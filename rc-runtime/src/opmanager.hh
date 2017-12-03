#ifndef _OPMANAGER_HH_
#define _OPMANAGER_HH_

#include <native/task.h>
#include <native/event.h>
#include <native/queue.h>
#include <native/mutex.h>
#include <native/cond.h>
#include <native/timer.h>

#include <iostream>
#include <string>
#include <setjmp.h>
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

#include "inst_type.h"
#include "Interpolation.hh"

#include "rclinterface.h"



extern RT_TASK rc_supervisor_desc;                         /* RC监控器任务描述符 */
extern RT_TASK_INFO  rc_supervisor_info;                   /* RC监控器任务状态 */
#define RC_SUPERVISOR_NAME "rc_supervisor_task"     /* RC监控器任务名 */
#define RC_SUPERVISOR_PRIORITY 84                   /* RC监控器任务优先级 */

extern RT_TASK rc_manager_desc;                            /* RC管理器任务描述符 */
extern RT_TASK_INFO  rc_manager_info;                      /* RC管理器任务状态 */
#define RC_MANAGER_NAME "rc_manager_task"           /* RC管理器任务名 */
#define RC_MANAGER_PRIORITY 83                      /* RC管理器任务优先级 */

extern RT_TASK rc_interp_desc;                             /* RC interp任务描述符 */
extern RT_TASK_INFO  rc_interp_info;                       /* RC interp任务状态 */
#define RC_INTERP_NAME "rc_interp_task"             /* RC interp任务名 */
#define RC_INTERP_PRIORITY 82                       /* RC interp任务优先级 */

extern RT_TASK rc_rsi_desc;                                /* RSI任务描述符 */
extern RT_TASK_INFO  rc_rsi_info;                          /* RSI任务状态 */
#define RC_RSI_NAME "rc_rsi_task"                   /* RSI任务名 */
#define RC_RSI_PRIORITY 81                          /* RSI任务优先级 */

extern RT_TASK rc_executor_desc;                           /* RC执行器任务描述符 */
extern RT_TASK_INFO  rc_executor_info;                     /* RC执行器任务状态 */
#define RC_EXECUTOR_NAME "rc_executor_task"         /* RC执行器任务名 */
#define RC_EXECUTOR_PRIORITY 80                     /* RC执行器任务优先级 */


#define RC_SUPERVISOR_PERIODE  100000000       // RC监控器循环周期


// RC运行模式
enum RcMode{
    OP_TEACH,           // 示教模式
    OP_RUN              // 再现运行模式
};

// RC运行管理器控制解释器命令
enum RcCommand{
    CMD_START,          // 开始运行
    CMD_NEXT,           // 单步运行
    CMD_RUN,            // 自动运行
    CMD_RESET,          // 返回程序起点
    CMD_STOP            // 停止运行
};

// RC运行状态机
enum  RcStatusMachine{
    STATUS_STOP,        // 待机态
    STATUS_READY,       // READY态，此时示教程序已完成词法、语法语义分析生成程序运行数据结构
    STATUS_AUTORUN,     // 自动运行态
    STATUS_STEP,        // 单步运行态
    STATUS_PAUSE        // 暂停态
};


extern RT_QUEUE mq_rc_exec_desc;                    // RC解释器 消息队列描述符
#define MQ_RC_EXEC_NAME    "mq_rc_exec"             // RC解释器 消息队列名

extern RT_QUEUE mq_rc_manager_desc;                 // RC运行管理器 消息队列描述符
#define MQ_RC_MANAGER_NAME    "mq_rc_manager"       // RC运行管理器 消息队列名

extern RT_QUEUE mq_rc_interp_desc;                  // RC插补器 消息队列描述符
#define MQ_RC_INTERP_NAME    "mq_rc_interp"         // RC插补器 消息队列名

extern RT_QUEUE mq_plc_desc;                        // PLC 消息队列描述符
#define MQ_PLC_NAME    "mq_plc"                     // PLC 消息队列名

extern RT_QUEUE mq_rc_rsi_desc;                     // RSI 消息队列描述符 
#define MQ_RC_RSI_NAME "mq_rc_rsi"                  // RSI 消息队列名


extern RT_COND inst_cond_desc;               /* 同步对象－－条件变量描述符 */
extern RT_MUTEX inst_mutex_desc;             /* 同步对象－－互斥量描述符 */
#define INST_COND_NAME   "inst_cond"
#define INST_MUTEX_NAME  "inst_mutex"
extern ROBOT_INST robot_inst_buffer_code;              // inst buffer
extern bool robot_inst_buffer_flag ;                   // inst buffer flag


extern bool RSIStopFlag; /* THIS IS VERY IMPORTANT, WHICH CONTROL THE WHOLE LIFECYCLE OF RSI */
#define RSI_RUN_PERIOD   160000000   /* 160 ms */
inline void RSI_SET_STOP() { RSIStopFlag = true; }



// 运行管理器核心数据结构——当前RC运行状态信息数据
struct RCOperationCtrl{
    RcMode  mode;                   // RC运行模式
    bool startup;                   // 是否启动运行，0：停止    1：启动
    int stepflag;                   // 是否进行单步运行，0：非单步   1：单步step in  2:单步step over

    std::string cur_project;        // 当前工程
    std::string cur_program;        // 当前程序
    // robot_program_file_process::statement_node *exec_program_head; // 当前程序头指针
    // robot_program_file_process::statement_node *pc;     // 程序指针PC
    int cur_linenum;                    // 当前指令对应程序文件行号
    int exec_run_mode;              // 当前程序运行状态，0:待机态，1:执行态
    RcStatusMachine  status;        // RC状态机
    int interp_status;              // 0:standy, 1:run
    int prog_ready;

    int jog_mode;
    JogProc procedure;  
    int jog_startup;

    double vper;
    double aper;
    double Ts;

    int coordinate;                 // 0: joint 1: base 2: tool
public:
    RCOperationCtrl():mode(OP_TEACH), startup(0), stepflag(1),
                    cur_linenum(0),exec_run_mode(0),vper(10), aper(40),Ts(0.008),
                    jog_mode(0), coordinate(0), procedure(JOG_STOP)
    {}

};

extern RobotConfig  rc_runtime_param;
extern RCOperationCtrl rc_core;         /* RC核心管理器 */

extern jmp_buf exec_startpoint;         /* 解释执行器起点 */
extern jmp_buf interp_startpoint;       /* RC插补器起点 */
extern jmp_buf rsi_startpoint;          /* RSI实时任务起点 */


int rc_core_init();                     // RC运行状态信息初始化


inline void STEPCHECK(int linenum)  {
    int flag = 0;    
    void *line ;                                      
    do{ 
        void* rc_cmd;                    /* RC控制命令 */                   
        int len = rt_queue_receive(&mq_rc_exec_desc, &rc_cmd, TM_INFINITE);   /* 等待管理器命令 */
        printf("[ received message: len=%d bytes, cmd=%d ]\n", len, *((const char *)rc_cmd));       
        char cmd = *((const char *)rc_cmd);                                               
        switch(cmd) {                                                                       
            case CMD_START:   
                flag = 0;    
                std::cout << "PC --------->  line : " << linenum  << std::endl;   
                send_filename_and_ptr(linenum, rc_core.cur_program);
                     
                break;   
            case CMD_NEXT:
                std::cout << "PC --------->  line : " << linenum  << std::endl;  
                send_filename_and_ptr(linenum, rc_core.cur_program);
                       
                if(rc_core.stepflag) {
                    flag = 1;
                } else {

                }
                break;                                                                       
            case CMD_RESET:                                                                 
                rc_core.cur_linenum = 1;                                                
                longjmp(exec_startpoint, 0);        /* 跳转至解释器起点 */                     
                break;                                                                      
            default:                                                                    
                break;                                                                      
            }                                                                               
            rt_queue_free(&mq_rc_exec_desc, rc_cmd);            /* 释放消息内存 */       
    } while(flag || !rc_core.startup);                                                          
}                                                                                       

inline void inst_buffer_write(ROBOT_INST &temp_inst) {
    rt_mutex_acquire(&inst_mutex_desc, TM_INFINITE); 
    robot_inst_buffer_code = temp_inst;
    robot_inst_buffer_flag = true;
    rt_cond_signal(&inst_cond_desc);
    rt_mutex_release(&inst_mutex_desc);       /* 释放同步互斥量 */
}

inline void inst_buffer_read(ROBOT_INST &temp_inst) {
    rt_mutex_acquire(&inst_mutex_desc, TM_INFINITE); 
    while(robot_inst_buffer_flag == false) {
        rt_cond_wait(&inst_cond_desc, &inst_mutex_desc, TM_INFINITE);
    }
    temp_inst = robot_inst_buffer_code;
    robot_inst_buffer_flag = false;
    rt_mutex_release(&inst_mutex_desc);       /* 释放同步互斥量 */
}

inline int send_cmd_to_rsi(int command) {
    void *cmd ;
    cmd = rt_queue_alloc(&mq_rc_rsi_desc, 1);
    (*(char*)cmd) = command;
    rt_queue_send(&mq_rc_rsi_desc, cmd, 1, Q_NORMAL);
}


inline void inst_buffer_read_nonblock(ROBOT_INST &temp_inst, bool &flag) {
    int ret;
    rc_shm->interp_startup_flag = 0;
    send_cmd_to_rsi(1);

    rt_mutex_acquire(&inst_mutex_desc, TM_INFINITE); 
    if(robot_inst_buffer_flag == false) {
        ret = rt_cond_wait(&inst_cond_desc, &inst_mutex_desc, 4000000);
    }
    if(ret == -ETIMEDOUT) 
    {
        RSI_SET_STOP();
        flag = false;
        robot_inst_buffer_flag = false;
        send_cmd_to_rsi(2);
        std::cout << "rsi buffer time out " << std::endl;
    }
    else
    {
        temp_inst = robot_inst_buffer_code;
        robot_inst_buffer_flag = false;
        flag = true;
    }
    rt_mutex_release(&inst_mutex_desc);       /* 释放同步互斥量 */
    rc_shm->interp_startup_flag = 1;
}

inline void init_runtime_param(){
    rc_runtime_param.axis_count = 6;
    rc_runtime_param.robot_type = ESTUN_ER4;
    // tmatrix transtool = dmatrix::Identity(4,4);
    // transtool(2,3) = 115;
    // rc_runtime_param.transTool = transtool;

    // for(int i = 0; i < 4; ++i)
    //      for(int j = 0; j < 4; ++j)
    //      {
    //         if(i == j)
    //              rc_runtime_param.transTool[i][j] = 1;
    //          else 
    //              rc_runtime_param.transTool[i][j] = 0;
    //      }
    // rc_runtime_param.transTool[2][3] = 115;
 
    tmatrix toolmat;
    XyzPose toolpos;
    toolpos << 0,0,0,0,0,0;
    toolpos[2] = 342;//192+165;
    toolpos[5] = 27.7872;
    toolmat = TermPos2TransMatrix(toolpos);
    for(int i = 0; i < 4; ++i)
        for(int j = 0; j < 4; ++j)
        {
                rc_runtime_param.transTool[i][j] = toolmat(i,j);
        }
 
    rc_runtime_param.Axis[0].DH_p.Theta = 0;
    rc_runtime_param.Axis[0].DH_p.d = 505;
    rc_runtime_param.Axis[0].DH_p.a = 150;
    rc_runtime_param.Axis[0].DH_p.Alpha = -90;
    rc_runtime_param.Axis[0].DH_p.offset = 0;

    rc_runtime_param.Axis[0].Lim_p.vellim = 155;
    rc_runtime_param.Axis[0].Lim_p.acclim = 200;
    rc_runtime_param.Axis[0].Lim_p.pos_min = -180;
    rc_runtime_param.Axis[0].Lim_p.pos_max = 180;

    rc_runtime_param.Axis[1].DH_p.Theta = 0;
    rc_runtime_param.Axis[1].DH_p.d = 0;
    rc_runtime_param.Axis[1].DH_p.a = 610;
    rc_runtime_param.Axis[1].DH_p.Alpha = 0;
    rc_runtime_param.Axis[1].DH_p.offset = -90;

    rc_runtime_param.Axis[1].Lim_p.vellim = 155;
    rc_runtime_param.Axis[1].Lim_p.acclim = 200;
    rc_runtime_param.Axis[1].Lim_p.pos_min = -70;
    rc_runtime_param.Axis[1].Lim_p.pos_max = 160;

    rc_runtime_param.Axis[2].DH_p.Theta = 0;
    rc_runtime_param.Axis[2].DH_p.d = 0;
    rc_runtime_param.Axis[2].DH_p.a = 150;
    rc_runtime_param.Axis[2].DH_p.Alpha = 90;
    rc_runtime_param.Axis[2].DH_p.offset = 0;

    rc_runtime_param.Axis[2].Lim_p.vellim = 230;
    rc_runtime_param.Axis[2].Lim_p.acclim = 200;
    rc_runtime_param.Axis[2].Lim_p.pos_min = -200;
    rc_runtime_param.Axis[2].Lim_p.pos_max = 80;

    rc_runtime_param.Axis[3].DH_p.Theta = 0;
    rc_runtime_param.Axis[3].DH_p.d = -675.5;
    rc_runtime_param.Axis[3].DH_p.a = 0;
    rc_runtime_param.Axis[3].DH_p.Alpha = -90;
    rc_runtime_param.Axis[3].DH_p.offset = 0;

    rc_runtime_param.Axis[3].Lim_p.vellim = 360;
    rc_runtime_param.Axis[3].Lim_p.acclim = 200;
    rc_runtime_param.Axis[3].Lim_p.pos_min = -170;
    rc_runtime_param.Axis[3].Lim_p.pos_max = 170;

    rc_runtime_param.Axis[4].DH_p.Theta = 0;
    rc_runtime_param.Axis[4].DH_p.d = 0;
    rc_runtime_param.Axis[4].DH_p.a = 0;
    rc_runtime_param.Axis[4].DH_p.Alpha = 90; 
    rc_runtime_param.Axis[4].DH_p.offset = 0;

    rc_runtime_param.Axis[4].Lim_p.vellim = 200;
    rc_runtime_param.Axis[4].Lim_p.acclim = 200;
    rc_runtime_param.Axis[4].Lim_p.pos_min = -135;
    rc_runtime_param.Axis[4].Lim_p.pos_max = 135;

    rc_runtime_param.Axis[5].DH_p.Theta = 0;
    rc_runtime_param.Axis[5].DH_p.d = -136;
    rc_runtime_param.Axis[5].DH_p.a = 0;
    rc_runtime_param.Axis[5].DH_p.Alpha = 180; 
    rc_runtime_param.Axis[5].DH_p.offset = 0;

    rc_runtime_param.Axis[5].Lim_p.vellim = 300;
    rc_runtime_param.Axis[5].Lim_p.acclim = 200;
    rc_runtime_param.Axis[5].Lim_p.pos_min = -360;
    rc_runtime_param.Axis[5].Lim_p.pos_max = 360;


}


inline int interp_step(AxisPos_Deg &onepoint){
    void *request;
    rt_queue_receive(&mq_rc_interp_desc, &request, TM_INFINITE);
    char cmd = *((const char *)request); 
    switch(cmd) {
        case 1:
            for(int i = 0; i < onepoint.size(); i ++) {
                rc_shm->interp_data.interp_value[i].command_pos = -onepoint[i];
            }
            rc_shm->interp_data.interp_value[1].command_pos = onepoint[1];
            rc_shm->interp_data.interp_value[5].command_pos = onepoint[5];
            rt_queue_free(&mq_rc_interp_desc, request);
            break;
        case 2:
            rt_queue_free(&mq_rc_interp_desc, request);
            longjmp(interp_startpoint, 0);        /* 跳转至interp起点 */    
            break;
        default:
            break;
    }

    void *msg ;
    msg = rt_queue_alloc(&mq_plc_desc, 1);
    (*(char*)msg) = 2;
    rt_queue_send(&mq_plc_desc, msg, 1, Q_NORMAL);   
}

inline int rsi_step() {
    void *request;
    rt_queue_receive(&mq_rc_rsi_desc, &request, TM_INFINITE);
    char cmd = *((const char *)request); 
    switch(cmd) {
        case 1:
            rt_queue_free(&mq_rc_rsi_desc, request);
            break;
        case 2:
            printf("RSI time out\n");
            rt_queue_free(&mq_rc_rsi_desc, request);
            RSI_SET_STOP();
            longjmp(rsi_startpoint, 0);        /* 跳转至rsi起点 */    
            break;
        default:
            break;
    }
}




inline int send_cmd_to_exec(RcCommand command) {
    void *cmd ;
    cmd = rt_queue_alloc(&mq_rc_exec_desc, 1);
    (*(char*)cmd) = command;
    rt_queue_send(&mq_rc_exec_desc, cmd, 1, Q_NORMAL);
}


inline int send_cmd_to_interp(int command) {
    void *cmd ;
    cmd = rt_queue_alloc(&mq_rc_interp_desc, 1);
    (*(char*)cmd) = command;
    rt_queue_send(&mq_rc_interp_desc, cmd, 1, Q_NORMAL);
}

inline int rsi_waitfor_run() {
    void *request;
    rt_queue_receive(&mq_rc_rsi_desc, &request, TM_INFINITE);
    char cmd = *((const char *)request); 
    return cmd;
}

inline void cur_cartpos_get(std::vector<double> &pos) {
    AxisPos_Deg p1(6);
    rt_mutex_acquire(&rc_mutex_desc, TM_INFINITE);
    for(int i = 0; i < 6; i ++) {
        p1[i] = -rc_shm->actual_info.axis_info[i].actual_pos;
    }
    p1[1] = rc_shm->actual_info.axis_info[1].actual_pos;
    p1[5] = rc_shm->actual_info.axis_info[5].actual_pos;
    rt_mutex_release(&rc_mutex_desc);
    XyzPose Outpos;
    calForwardKin(p1, rc_runtime_param, Outpos);
    for(int i = 0; i < 6; i ++) {
        pos[i] = Outpos[i];
    }

}

inline void cur_axispos_get(std::vector<double> &pos) {
    rt_mutex_acquire(&rc_mutex_desc, TM_INFINITE);
    for(int i = 0; i < 6; i ++) {
        pos[i] = -rc_shm->actual_info.axis_info[i].actual_pos;
    }
    pos[1] = rc_shm->actual_info.axis_info[1].actual_pos;
    pos[5] = rc_shm->actual_info.axis_info[5].actual_pos;
    rt_mutex_release(&rc_mutex_desc);
} 



#endif
