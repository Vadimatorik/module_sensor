#include "mpu6050_i2c.h"

void mpu6050_i2c::init ( void ) const {
    this->m                = USER_OS_STATIC_MUTEX_CREATE( &mb );
    this->s                = USER_OS_STATIC_BIN_SEMAPHORE_CREATE( &sb );
    USER_OS_STATIC_TASK_CREATE( mpu6050_i2c::thread, "mpu6050", MPU6050_STACK_SIZE, ( void* )this, this->cfg->prio, this->tb, &this->ts );
}

void mpu6050_i2c::thread ( void* p_cfg ) {
    (void)p_cfg;
    while ( true ) {

    }
}
