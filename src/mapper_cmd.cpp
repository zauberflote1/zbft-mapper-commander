/**
 * @ Author: zauberflote1
 * @ Create Time: 2024-10-06 23:20:17
 * @ Modified by: zauberflote1
 * @ Modified time: 2024-10-11 19:31:20
 * @ Description:
 * PIPELINE TO VOXL-MAPPER VIA COMMANDS
 */


#include "mapper_cmd.hpp"

MapperCmd::MapperCmd() {
    

    ch_mapper = pipe_client_get_next_available_channel();
    ch_vio = pipe_client_get_next_available_channel();

    pose_buffer = RC_TF_RINGBUF_INITIALIZER;
    rc_tf_ringbuf_alloc(&pose_buffer, 2500);

    pipe_client_set_simple_helper_cb(ch_vio, [](int ch, char* data, int bytes, void* context) {
        static_cast<MapperCmd*>(context)->_vio_helper_cb(ch, data, bytes, context);
    }, this);

    pipe_client_set_connect_cb(ch_vio, [](int ch, void* context) {
        static_cast<MapperCmd*>(context)->_vio_connect_cb(ch, context);
    }, this);

    pipe_client_set_disconnect_cb(ch_vio, [](int ch, void* context) {
        static_cast<MapperCmd*>(context)->_vio_disconnect_cb(ch, context);
    }, this);


//SQUARE MISSION ONLY
//1M SQUARE
    square_mission << 1.0, 0.0, 0.0,
                      0.0, 1.0, 0.0,
                      -1.0, 0.0, 0.0,
                      0.0, -1.0, 0.0;

    pts_togo = square_mission.rows();
}

MapperCmd::~MapperCmd() {
    //CLOSE PIPES --  AND FREE BUFFERS?
    _cmdHandler_terminate_cb();
    pipe_client_close_all();
}



int MapperCmd::getRobotPose(rc_tf_t &tf_body_wrt_fixed, int64_t ts){
    static int error_ctr = 0;

    int ret = rc_tf_ringbuf_get_tf_at_time(&pose_buffer, ts, &tf_body_wrt_fixed);
    if (ret < 0)
    {
        error_ctr++;
        if(error_ctr>5){
            //fprintf(stderr, "ERROR fetching tf from tf ringbuffer\n");
            if (ret == -2){
                printf("INFO waiting for VIO to start\n");
                error_ctr = 0;
            }
            if (ret == -3){
                printf("WARNING the requested timestamp was too new, VIO may have stopped\n");
                error_ctr = 0;
            }
            if (ret == -4){
                printf("WARNING the requested timestamp was too old\n");
            }
        }
        return -1;
    }

    return 0;
}

int MapperCmd::sendPlanCommand(Eigen::Vector3d goal_vtf) {


    char command[100];
    //DO NOT STOP FOLLOWING IF THIS IS THE FIRST ITERATION -- MAY CHANGE THIS IN THE FUTURE...
    if (goal_plan){ //CHECK POTENTIAL RACE CONDITION
        snprintf(command, sizeof(command), STOP_FOLLOWING); // STOP_FOLLOWING = "stop_following"
        if (pipe_client_send_control_cmd(ch_mapper, command) == 0) {
            std::cout << "Sent STOP_FOLLOWING command: " << command << std::endl;
        } else {
            std::cerr << "Failed to send STOP_FOLLOWING command" << std::endl;
            return -1;
        }
    }
    usleep(500000); // SLEEP FOR 0.5 SECOND TO GIVE THE SERVER TIME TO PROCESS STOP_FOLLOWING COMMAND
    snprintf(command, sizeof(command), "plan_to:%.2f,%.2f,%.2f", goal_vtf.x(), goal_vtf.y(), goal_vtf.z());

    // Send the command to the server using pipe_client_send_control_cmd
    if (pipe_client_send_control_cmd(ch_mapper, command) == 0) {
        std::cout << "Sent PLAN_TO command: " << command << std::endl;
    } else {
        std::cerr << "Failed to send PLAN_TO command" << std::endl;
        return -1;
    }
    usleep(500000); // SLEEP FOR 0.5 SECOND TO GIVE THE SERVER TIME TO PROCESS PLAN_TO COMMAND
    snprintf(command, sizeof(command), FOLLOW_PATH); // FOLLOW_PATH = "follow_path"
    if (pipe_client_send_control_cmd(ch_mapper, command) == 0) { 
        std::cout << "Sent FOLLOW_TO command: " << command << std::endl;
    } else {
        std::cerr << "Failed to send FOLLOW_TO command" << std::endl;
        return -1;
    }
    return 0;

}
//////////////////////////////////////////////////////////////////////////////////////////////////////////
//VIO HELPER FUNCTIONS (FROM VOXL-MAPPER)
void MapperCmd::_vio_helper_cb(__attribute__((unused)) int ch, char *data, int bytes, __attribute__((unused)) void *context){
    // validate data
    int n_packets;
    pose_vel_6dof_t *d = pipe_validate_pose_vel_6dof_t(data, bytes, &n_packets);

    // if there was an error OR no packets received, just return;
    if (d == NULL){
        std::cerr << "[VIO] ERROR: Failed to validate pose_vel_6dof_t data" << std::endl;
        return;
    }
    if (n_packets <= 0){
        std::cerr << "[VIO] ERROR: No packets received" << std::endl;
        return;
    }

    for (int i = 0; i < n_packets; i++)
    {
        rc_tf_ringbuf_insert_pose(&pose_buffer, d[i]);
    }
    // std::cout << "[VIO] GOT NEW" << std::endl;
    return;
}

void MapperCmd::_vio_connect_cb(__attribute__((unused)) int ch, __attribute__((unused)) void *context){
    printf("Connected to VIO server\n");
}

void MapperCmd::_vio_disconnect_cb(__attribute__((unused)) int ch, __attribute__((unused)) void *context){
    printf("Disconnected from VIO server\n");
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////

//MAPPERCMD HELPER FUNCTIONS
void MapperCmd::cmdHandler(){
    //GET CURRENT POSITION
    rc_tf_t tf_body_wrt_fixed = RC_TF_INITIALIZER;
    int ret_cmd = -2;
    if (getRobotPose(tf_body_wrt_fixed, my_time_monotonic_ns()) != 0) {
        return; //EARLY EXIT IF WE FAIL TO GET THE CURRENT POSITION
    }
    Eigen::Vector3d current_pos = Eigen::Vector3d(tf_body_wrt_fixed.d[0][3], tf_body_wrt_fixed.d[1][3], tf_body_wrt_fixed.d[2][3]);
    // std::cout << tf_body_wrt_fixed.d[0][3] << std::endl;
    // std::cout << tf_body_wrt_fixed.d[1][3] << std::endl;
    // std::cout << tf_body_wrt_fixed.d[2][3] << std::endl;
    if (pts_togo == 0){

        std::cout << "MISSION COMPLETE" << std::endl;

        return;
    }
    Eigen::Vector3d goal_pos = square_mission.row(pts_togo - 1).transpose() + current_pos;
    //START GOAL PLAN, IF WE HAVEN'T ALREADY
    if (!goal_plan){
        //SEND INITIAL GOAL
        ret_cmd = sendPlanCommand(goal_pos);
        if (ret_cmd != 0){
            std::cerr << "[MAPPER-CMD] Failed to send initial goal" << std::endl;
            return;
        }
        goal_plan = true; //CHECK POTENTIAL RACE CONDITION IF USING MULTIPLE DRONES
    }
    else{
        if((current_pos - (square_mission.row(pts_togo)).transpose()).norm() < 0.1){
            //SEND NEW GOAL
            ret_cmd = sendPlanCommand(goal_pos);
            if (ret_cmd != 0){
                std::cerr << "[MAPPER-CMD] Failed to send new goal" << std::endl;
                return;
            }   
        } else{
            // std::cout << "WAITING FOR GOAL" << std::endl;
            return;
        }
        
    }
    if (pts_togo > 0) { //EXTRA SAFETY CHECK
        pts_togo--;
    }


}

void MapperCmd::_cmdHandler_cb() {
    int64_t next_time = 0;
    while (!stop_thread) {
        {
            std::lock_guard<std::mutex> lock(cmd_mutex);
            // std::cout << "[CMD HANDLER] Iteration start" << std::endl;
            cmdHandler();
        }
        // std::cout << "[CMD HANDLER] Sleeping for 20Hz cycle." << std::endl; 
        my_loop_sleep(20.0, &next_time); //20HZ, MIGHT BE TOO FAST OR TOO SLOW...
    }
        // std::cout << "[CMD HANDLER] Thread stopping." << std::endl; 
}
void MapperCmd::_cmdHandler_terminate_cb() {
    stop_thread = true;
    if (cmd_thread.joinable()) {
        cmd_thread.join();
    }
}
void MapperCmd::initMPA(){
//MAPPER
    ret_mapper = pipe_client_open(ch_mapper, PLAN_LOCATION, "zbft-mapper-cmd", EN_PIPE_CLIENT_AUTO_RECONNECT, 1024 * 1024 * 64); //NOT SURE ABOUT THE BUFFER SIZE HERE...JUST COPYING FROM VOXL-MAPPER
    if (ret_mapper != 0) {
        std::cerr << "[MAPPER] Failed to open the pipe client." << std::endl;
        return;
    }
    std::cout << "[MAPPER] Pipe client opened successfully." << std::endl;
//VIO
   ret_vio = pipe_client_open(ch_vio, BODY_WRT_FIXED_POSE_PATH, "zbft-mapper-cmd",
                         EN_PIPE_CLIENT_SIMPLE_HELPER | EN_PIPE_CLIENT_AUTO_RECONNECT,
                         POSE_6DOF_RECOMMENDED_READ_BUF_SIZE);
    if (ret_vio != 0){
        std::cerr << "[VIO] Failed to open the pipe client." << std::endl;
        return;
    }
    std::cout << "[VIO] Pipe client opened successfully." << std::endl;
//CMD THREAD
    cmd_thread = std::thread(&MapperCmd::_cmdHandler_cb, this);
}


