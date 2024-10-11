/**
 * @ Author: zauberflote1
 * @ Create Time: 2024-10-06 23:21:06
 * @ Modified by: zauberflote1
 * @ Modified time: 2024-10-11 03:44:18
 * @ Description:
 * PIPELINE TO VOXL-MAPPER VIA COMMANDS (HEADER)
 */

#ifndef MAPPER_CMD_HPP
#define MAPPER_CMD_HPP
#pragma once
//MODAL_PIPE_DEFAULT_BASE_DIR is defined in modal_pipe_common.h
#include <modal_pipe_common.h>
#include "rc_transform_ringbuf.h"
#include <modal_pipe.h>
#include <iostream>
#include "misc.h"
#include <Eigen/Dense>
#include <string>
#include <stdio.h>
#include <unistd.h>
#include <thread>
#include <mutex>
#include <atomic>
#include <csignal> 
#include <memory>   


#define PLAN_NAME "plan_msgs"
#define PLAN_LOCATION (MODAL_PIPE_DEFAULT_BASE_DIR PLAN_NAME "/")
#define BODY_WRT_FIXED_POSE_PATH MODAL_PIPE_DEFAULT_BASE_DIR "vvhub_body_wrt_fixed/"
#define QVIO_SIMPLE_LOCATION MODAL_PIPE_DEFAULT_BASE_DIR "qvio/"

#define PLAN_TO "plan_to"
#define FOLLOW_PATH "follow_path"
#define STOP_FOLLOWING "stop_following"

class MapperCmd {
public:
    MapperCmd();
    ~MapperCmd();
    void initMPA();

private:
    int sendPlanCommand(Eigen::Vector3d goal_vtf);
    void _vio_connect_cb(__attribute__((unused)) int ch, __attribute__((unused)) void *context);
    void _vio_helper_cb(__attribute__((unused)) int ch, char *data, int bytes, __attribute__((unused)) void *context);
    void _vio_disconnect_cb(__attribute__((unused)) int ch, __attribute__((unused)) void *context);
    int getRobotPose(rc_tf_t &tf_body_wrt_fixed, int64_t ts);
    void cmdHandler();
    void _cmdHandler_cb(); 
    void _cmdHandler_terminate_cb();  



    rc_tfv_ringbuf_t pose_buffer;
    int ch_mapper;
    int ch_vio;
    int ret_mapper;
    int ret_vio;
    bool goal_plan = false;
    std::thread cmd_thread;           
    std::atomic<bool> stop_thread = false;   
    std::mutex cmd_mutex;             

    //SQUARE MISSION ONLY
    Eigen::Matrix<double, 4, 3> square_mission;
    int pts_togo;

};





#endif // MAPPER_CMD_HPP