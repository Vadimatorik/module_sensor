#pragma once

#ifdef __cplusplus

#include "user_os.h"
#include "mc_hardware_interfaces_i2c.h"
#include "mpu6050_defs.h"

#define MPU6050_STACK_SIZE              400

struct mpu6050_cfg {
    uint32_t                          		period;
    uint8_t                         	    prio;
    McHardwareInterfaces::I2cMaster*        i2c;
    USER_OS_STATIC_BIN_SEMAPHORE*     		ext_semaphore;// синхонизация внешних потоков. Выставляется при готовности решения.
};

class mpu6050_i2c {
public:
    mpu6050_i2c( const mpu6050_cfg* const cfg ) : cfg( cfg ) {}

    void    init            ( void ) const;
    int     get_data        ( float accelerometer[3], float magnetometer[3] );
    int     get_data_sync   ( float accelerometer[3], float magnetometer[3] );

    void    data_ready_irq  ( void );

private:
    static void thread ( void* p_cfg );

    int     get_raw_accel_gyro ( int16_t* accel_gyro );
    void    add_bias ( int16_t *d );
    void    hardware_init ( void );
    void    write_bits ( uint8_t slave_addr, uint8_t reg_addr, uint8_t bit_start, uint8_t length, uint8_t data );

    const mpu6050_cfg*                      const cfg;

    mutable float accelerometer[3];                     // данные акселерометра xyz g
    mutable float magnetometer[3];                      // даныне гироскопа xyz rad/sec

    mutable USER_OS_STATIC_STACK_TYPE               tb[ MPU6050_STACK_SIZE ] = { 0 };
    mutable USER_OS_STATIC_TASK_STRUCT_TYPE         ts;

    mutable USER_OS_STATIC_MUTEX                    m             = nullptr;
    mutable USER_OS_STATIC_MUTEX_BUFFER             mb;

    //синхронизация по готовности решения(внтуренняя, по прерыванию на ножке IMU)
    mutable USER_OS_STATIC_BIN_SEMAPHORE            s             = nullptr;
    mutable USER_OS_STATIC_BIN_SEMAPHORE_BUFFER     sb;
};

#endif
