#include "plc.h"
#include <stdlib.h>


/**
 * 函数名：rc_shm_io_write
 * 功能：PLC向共享内存数据区写入IO数据；
 *      待实现
 * 参数：rc_shm：内存共享区指针
 *
 * 返回值：无
 */
void rc_shm_io_write(RCMem *rc_shm, void *cookie){
    rt_mutex_acquire(&rc_mutex_desc, TM_INFINITE);
    /* TODO */
    rt_mutex_release(&rc_mutex_desc);
}

/**
 * 函数名：rc_shm_io_read
 * 功能：PLC从共享内存数据区读出IO数据；
 *      待实现
 * 参数：rc_shm：内存共享区指针
 *
 * 返回值：无
 */
void rc_shm_io_read(RCMem *rc_shm, void *cookie){
    rt_mutex_acquire(&rc_mutex_desc, TM_INFINITE);
    /* TODO */
    rt_mutex_release(&rc_mutex_desc);
}

/**
 * 函数名：rc_shm_servo_read
 * 功能：RC从共享内存数据区读出伺服电机实际位置、速度、加速度信息；
 *      首先获得同步互斥量，然后读出数据，最后释放同步互斥量。
 * 参数：rc_shm：内存共享区指针
 *      axis_actual_info：伺服电机实际位置、速度、加速度信息变量数据结构（实际为读出的返回值）
 * 返回值：无
 */
void rc_shm_servo_read(RCMem *rc_shm, RobotAxisActualInfo *axis_actual_info){
    rt_mutex_acquire(&rc_mutex_desc, TM_INFINITE);
    for(int i =0; i < axis_actual_info->size; i ++){
        axis_actual_info->axis_info[i] = rc_shm->actual_info.axis_info[i];
    }
    rt_mutex_release(&rc_mutex_desc);
}

/**
 * 函数名：rc_shm_servo_write
 * 功能：RC向共享内存数据区写入伺服电机目标位置、速度、加速度信息(即插补值);
 *      首先获得同步互斥量，然后:(1)判断伺服插补队列是否为空，若为空则在队列头部写入插补值，最后更新队列头指针，
 *      即head值,最后向PLC发送信号告知其等待的条件变量已满足。(2)判断伺服插补队列是否已满，若已满则等待条件变量
 *      （即等待PLC从队列中取走插补值），否则，直接在队列头部写入插补值，最后更新队列头指针，即head值;最后释放同
 *      步互斥量。
 * 参数：rc_shm：内存共享区指针
 *      axis_command_info：伺服电机目标位置、速度、加速度信息变量(即插补值)
 * 返回值：无
 */
void rc_shm_servo_write(RCMem *rc_shm, RobotInterpData *axis_command_info){
    rt_mutex_acquire(&rc_mutex_desc, TM_INFINITE);      /* 获得同步互斥量 */
    if(rc_shm->interp_queue.head == rc_shm->interp_queue.tail){             /* 判断插补队列是否为空 */
        int p_head = rc_shm->interp_queue.head;
        // printf("p_head: %d\n", p_head);
        for(int i = 0; i < ROBOT_AXIS_COUNT; i ++){                  /* 插补值写入队列 */
            rc_shm->interp_queue.data[p_head].interp_value[i].command_pos = axis_command_info->interp_value[i].command_pos;
            rc_shm->interp_queue.data[p_head].interp_value[i].command_vel = axis_command_info->interp_value[i].command_vel;
            rc_shm->interp_queue.data[p_head].interp_value[i].command_acc = axis_command_info->interp_value[i].command_acc;
        }
        /* 更新环形队列头指针 */
        if(rc_shm->interp_queue.head < CIRCULAR_INTERP_QUEUE_SIZE - 1){
            rc_shm->interp_queue.head ++;
        } else if(rc_shm->interp_queue.head == CIRCULAR_INTERP_QUEUE_SIZE - 1){
            rc_shm->interp_queue.head = 0;
        } else {
            printf("E_INTERP_QUEUE_POINTER: (name=head, size=%d)\n", rc_shm->interp_queue.head);
            exit(-1);
        }
        rt_cond_signal(&rc_cond_desc);     /* 向PLC发送信号告知其等待的条件变量已满足 */
    } else {
        /* 判断插补队列是否已满 */
        while((rc_shm->interp_queue.tail == rc_shm->interp_queue.head + 1)
            || (rc_shm->interp_queue.tail == 0 && rc_shm->interp_queue.head == CIRCULAR_INTERP_QUEUE_SIZE - 1) ){
            rt_cond_wait(&rc_cond_desc, &rc_mutex_desc, TM_INFINITE);
        }
        int p_head = rc_shm->interp_queue.head;
        // printf("p_head: %d\n", p_head);
        for(int i = 0; i < ROBOT_AXIS_COUNT; i ++){                  /* 插补值写入队列 */
            rc_shm->interp_queue.data[p_head].interp_value[i].command_pos = axis_command_info->interp_value[i].command_pos;
            rc_shm->interp_queue.data[p_head].interp_value[i].command_vel = axis_command_info->interp_value[i].command_vel;
            rc_shm->interp_queue.data[p_head].interp_value[i].command_acc = axis_command_info->interp_value[i].command_acc;
        }
        /* 更新环形队列头指针 */
        if(rc_shm->interp_queue.head < CIRCULAR_INTERP_QUEUE_SIZE - 1){
            rc_shm->interp_queue.head ++;
        } else if(rc_shm->interp_queue.head == CIRCULAR_INTERP_QUEUE_SIZE - 1){
            rc_shm->interp_queue.head = 0;
        } else {
            printf("E_INTERP_QUEUE_POINTER: (name=head, size=%d)\n", rc_shm->interp_queue.head);
            exit(-1);
        }
    }
    rt_mutex_release(&rc_mutex_desc);       /* 释放同步互斥量 */
}
