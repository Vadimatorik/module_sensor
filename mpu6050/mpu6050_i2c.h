#pragma once

#ifdef __cplusplus

#include "user_os.h"

#define MPU6050_STACK_SIZE              300

struct mpu6050_cfg {

};

class mpu6050_i2c {
public:
    mpu6050_i2c( const mpu6050_cfg* const cfg ) : cfg( cfg ) {}

    void    init            ( void ) const;
    int     get_data        ( float accelerometer[3], float magnetometer[3] );
    int     get_data_sync   ( float accelerometer[3], float magnetometer[3] );

    void    data_ready_IRQ  ( void );

private:
    const mpu6050_cfg*              const cfg;
    USER_OS_STATIC_STACK_TYPE       task_stack[ MPU6050_STACK_SIZE ] = { 0 };
    USER_OS_STATIC_TASK_STRUCT_TYPE task_struct;

    USER_OS_STATIC_MUTEX                    m             = nullptr;
    USER_OS_STATIC_MUTEX_BUFFER             mb;

    USER_OS_STATIC_BIN_SEMAPHORE            s             = nullptr;
    USER_OS_STATIC_BIN_SEMAPHORE_BUFFER     sb;
};

#endif
