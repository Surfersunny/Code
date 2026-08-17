#include "all.h"
#include "bfs_planning.h"

// ========== 宏定义 ==========
#define PWM_FREQ (17000)
#define OPENART_UART UART_4
#define OPENART_BAUD 115200
#define OPENART_TX_PIN UART4_TX_C16
#define OPENART_RX_PIN UART4_RX_C17
#define MAP_SIZE (ROWS * COLS)

#define STAGE_1 1
#define STAGE_2 2
#define STAGE_3 3
#define TOTAL_STAGES 3

// ========== 外部变量声明 ==========
extern uint8_t map_buffer[MAP_SIZE];
extern uint16_t map_recv_cnt;
extern uint8_t map_complete;

extern uint8_t recog_ready;
extern uint8_t recog_target_idx;
extern uint8_t recog_box_idx;
extern uint8_t recog_phase;
extern uint8_t recog_total;
extern uint8_t data;

// ========== 本地全局变量 ==========
uint8_t current_stage = STAGE_1;
uint8_t all_done = 0;
uint8_t success = 0;

uint8_t canqueryangle = 1;
uint8_t is_initialized = 0;

volatile uint32_t sys_cnt = 0;
int16 encoder_data[4] = {0};
float omega[4] = {0};

MotionControl_t motion = {0};
IMU_TypeDef imu = {0};

BFS_MotionQueue_t motion_queue;
uint16_t cmd_index = 0;
uint8_t sequence_running = 0;
uint8_t planning = 0;
uint8_t vision_waiting = 0;
uint8_t recognize_done = 0;     // 识别阶段是否完成
uint8_t map_loaded = 0;         // 地图是否已载入
uint8_t awaiting_push_plan = 0; // 识别完成等待推送规划

// ========== 串口发送 ==========
void send_cmd(uint8_t cmd) {
    uart_write_byte(OPENART_UART, cmd);
}

// ========== 请求地图 ==========
void request_map(void) {
    map_recv_cnt = 0;
    map_complete = 0;
    canqueryangle = 0;
    recognize_done = 0;
    map_loaded = 0;
    awaiting_push_plan = 0;
    send_cmd('S');
}

// ========== 执行运动命令 ==========
void execute_motion_cmd(BFS_MotionCmd_t *cmd) {
    if (cmd->type == 0) {
        motion_translate(&motion, cmd->value, cmd->dir);
    } else if (cmd->type == 1) {
        motion_rotate(&motion, cmd->value);
    } else if (cmd->type == 2) {
        uint8_t phase = (uint8_t)cmd->value;
        
        recog_ready = 0;
        recog_phase = phase;
        canqueryangle = 0;
        
        if (phase == 1) {
            recog_target_idx = cmd->idx;
            recog_total = bfs_get_target_count();
						system_delay_ms(2000);
            send_cmd('t');
        } else {
            recog_box_idx = cmd->idx;
            recog_total = bfs_get_box_count();
						system_delay_ms(2000);
            send_cmd('b');
        }
        
        while (!recog_ready) {
            system_delay_ms(1);
        }
        recog_ready = 0;
        canqueryangle = 1;
    }
}

// ========== 规划并启动 ==========
void plan_and_start(void) {
    if (!map_complete || planning || sequence_running || all_done) return;
    
    planning = 1;
    canqueryangle = 1;
    
    // 第一次规划才载入地图，避免二次载入清掉识别IDs
    if (!map_loaded) {
        uint8_t map_2d[ROWS][COLS];
        for (int i = 0; i < ROWS; i++) {
            for (int j = 0; j < COLS; j++) {
                map_2d[i][j] = map_buffer[i * COLS + j];
            }
        }
        bfs_load_map(map_2d);
        map_loaded = 1;
    }
    
    memset(&motion_queue, 0, sizeof(BFS_MotionQueue_t));
    
    if (current_stage == STAGE_1) {
        success = bfs_plan_stage1(&motion_queue);
    } else if (!recognize_done) {
        // 第2/3关：先识别
        bfs_plan_recognize(&motion_queue);
        recognize_done = 1;
        awaiting_push_plan = 1;
        success = 1;
    } else {
        // 第2/3关：推送规划
        awaiting_push_plan = 0;
        uint8_t has_bomb = (bfs_get_bomb_count() > 0);
        success = bfs_plan_stage23(&motion_queue, has_bomb);
    }
    
    if (success || motion_queue.count > 0) {
        cmd_index = 0;
        sequence_running = 1;
        vision_waiting = 0;
    }
    
    planning = 0;
}

// ========== 硬件初始化 ==========
void hardware_init(void) {
    gpio_init(MOTOR1_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    pwm_init(MOTOR1_PWM, PWM_FREQ, 0);
    gpio_init(MOTOR2_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    pwm_init(MOTOR2_PWM, PWM_FREQ, 0);
    gpio_init(MOTOR3_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    pwm_init(MOTOR3_PWM, PWM_FREQ, 0);
    gpio_init(MOTOR4_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    pwm_init(MOTOR4_PWM, PWM_FREQ, 0);
    
    encoder_dir_init(ENCODER_1, ENCODER_1_LSB, ENCODER_1_DIR);
    encoder_dir_init(ENCODER_2, ENCODER_2_LSB, ENCODER_2_DIR);
    encoder_dir_init(ENCODER_3, ENCODER_3_LSB, ENCODER_3_DIR);
    encoder_dir_init(ENCODER_4, ENCODER_4_LSB, ENCODER_4_DIR);
    
    imu_init(&imu);
    
    uart_init(OPENART_UART, OPENART_BAUD, OPENART_TX_PIN, OPENART_RX_PIN);
    uart_rx_interrupt(OPENART_UART, 1);
    
    pit_ms_init(PIT_CH0, 1);
    interrupt_global_enable(0);
}

// ========== PIT中断处理 ==========
void pit_handler(void) {
    static uint64_t imu_cnt = 0;
    imu_cnt++;
    
    if (imu_cnt <= 1000) {
        return;
    }
    else if (imu_cnt > 1000 && imu_cnt <= 2000) {
        if (imu_cnt % 5 == 0) imu_calibrate(&imu);
    }
    else {
        encoder_read_all(encoder_data);
        encoder_to_omega_all(encoder_data, omega);
        motion_update(&motion);
        sys_cnt++;
        
        if (sys_cnt >= 5) {
            imu_update(&imu);
            sys_cnt = 0;
        }
    }
}

// ========== 离开发车区 ==========
void leave_start_zone(void) {
    memset(&motion_queue, 0, sizeof(BFS_MotionQueue_t));
    
    BFS_MotionCmd_t cmd1;
    cmd1.type = 0;
    cmd1.value = 300.0f;
    cmd1.dir = UP;
    motion_queue.cmds[motion_queue.count++] = cmd1;
    
    BFS_MotionCmd_t cmd2;
    cmd2.type = 0;
    cmd2.value = 200.0f;
    cmd2.dir = RIGHT;
    motion_queue.cmds[motion_queue.count++] = cmd2;
    
    cmd_index = 0;
    sequence_running = 1;
    is_initialized = 1;
    
    while (sequence_running) {
        if (motion.state == MOTION_STATE_IDLE) {
            if (cmd_index < motion_queue.count) {
                execute_motion_cmd(&motion_queue.cmds[cmd_index]);
                cmd_index++;
            } else {
                sequence_running = 0;
            }
        }
        system_delay_ms(5);
    }
}

// ========== 回到发车区 ==========
void return_to_start_zone(void) {
    memset(&motion_queue, 0, sizeof(BFS_MotionQueue_t));
    
    BFS_MotionCmd_t cmd1;
    cmd1.type = 0;
    cmd1.value = 200.0f;
    cmd1.dir = LEFT;
    motion_queue.cmds[motion_queue.count++] = cmd1;
    
    BFS_MotionCmd_t cmd2;
    cmd2.type = 0;
    cmd2.value = 300.0f;
    cmd2.dir = DOWN;
    motion_queue.cmds[motion_queue.count++] = cmd2;
    
    cmd_index = 0;
    sequence_running = 1;
    
    while (sequence_running) {
        if (motion.state == MOTION_STATE_IDLE) {
            if (cmd_index < motion_queue.count) {
                execute_motion_cmd(&motion_queue.cmds[cmd_index]);
                cmd_index++;
            } else {
                sequence_running = 0;
            }
        }
        system_delay_ms(5);
    }
}

// ========== 主函数 ==========
int main(void) {
    clock_init(SYSTEM_CLOCK_600M);
    debug_init();
    system_delay_ms(2000);
    
    hardware_init();
    motion_init(&motion);
    bfs_init();
    
    recog_ready = 0;
    recog_target_idx = 0;
    recog_box_idx = 0;
    recog_phase = 0;
    recog_total = 0;
    all_done = 0;
    is_initialized = 0;
    
    leave_start_zone();
    request_map();
    
    while (1) {
        if (all_done) {
            system_delay_ms(100);
            continue;
        }
        
        if (map_complete && !planning && !sequence_running) {
            plan_and_start();
        }
        
        if (sequence_running) {
            if (motion.state == MOTION_STATE_IDLE) {
                if (cmd_index < motion_queue.count) {
                    BFS_MotionCmd_t *cmd = &motion_queue.cmds[cmd_index];
                    execute_motion_cmd(cmd);
                    cmd_index++;
                } else {
                    sequence_running = 0;
                    
                    if (awaiting_push_plan) {
                        // 识别阶段结束，等主循环重新规划推送
                    } else if (current_stage < TOTAL_STAGES) {
                        return_to_start_zone();
                        system_delay_ms(3000);
                        leave_start_zone();
                        current_stage++;
                        request_map();
                    } else {
                        return_to_start_zone();
                        all_done = 1;
                        stop_all_motors();
                    }
                }
            }
        }
        
        system_delay_ms(5);
    }
}