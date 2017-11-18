#pragma once

#ifdef __cplusplus

#include "user_os.h"

#define MPU6050_STACK_SIZE              400

struct mpu6050_cfg {
    uint32_t                            period;
    uint8_t                             prio;
    USER_OS_STATIC_BIN_SEMAPHORE*       ext_semaphore;
};

class mpu6050_i2c {
public:
    mpu6050_i2c( const mpu6050_cfg* const cfg ) : cfg( cfg ) {}

    void    init            ( void ) const;
    int     get_data        ( float accelerometer[3], float magnetometer[3] );
    int     get_data_sync   ( float accelerometer[3], float magnetometer[3] );

    void    data_ready_IRQ  ( void );

private:
    static void thread ( void* p_cfg );

    const mpu6050_cfg*                      const cfg;

    mutable float accelerometer[3];
    mutable float magnetometer[3];

    mutable USER_OS_STATIC_STACK_TYPE               tb[ MPU6050_STACK_SIZE ] = { 0 };
    mutable USER_OS_STATIC_TASK_STRUCT_TYPE         ts;

    mutable USER_OS_STATIC_MUTEX                    m             = nullptr;
    mutable USER_OS_STATIC_MUTEX_BUFFER             mb;

    mutable USER_OS_STATIC_BIN_SEMAPHORE            s             = nullptr;
    mutable USER_OS_STATIC_BIN_SEMAPHORE_BUFFER     sb;
};

#endif
