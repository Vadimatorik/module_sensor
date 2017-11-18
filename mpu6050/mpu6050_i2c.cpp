#include "mpu6050_i2c.h"
#include <string.h>

#define DEGTORAD(x) ((x)*(3.14159265358979323846/180.0F))

void mpu6050_i2c::init ( void ) const {
    this->m                = USER_OS_STATIC_MUTEX_CREATE( &mb );
    this->s                = USER_OS_STATIC_BIN_SEMAPHORE_CREATE( &sb );
    USER_OS_STATIC_TASK_CREATE( mpu6050_i2c::thread, "mpu6050", MPU6050_STACK_SIZE, ( void* )this, this->cfg->prio, this->tb, &this->ts );
}

//**********************************************************************
// Поток.
//**********************************************************************
void mpu6050_i2c::thread ( void* p_cfg ) {
    mpu6050_i2c* obj = ( mpu6050_i2c* )p_cfg;
    int16_t acel_gyro_d[6]= {0};
    float a_data[3] = {0};
    float g_dataDeg[3] = {0};
    float g_dataRad[3] = {0};
    int axes_count;
    USER_OS_TICK_TYPE xLastWakeTime;
    const USER_OS_TICK_TYPE period = obj->cfg->period;

    obj->hardware_init();

    xLastWakeTime = USER_OS_TASK_GET_TICK_COUNT();
    while ( true ) {
        if ( obj->s == NULL ) {
            USER_OS_TASK_DELAY_UNTIL( &xLastWakeTime, period );
        }else{
            USER_OS_TAKE_BIN_SEMAPHORE(obj->s, portMAX_DELAY); // ждём прерывания по готовности данных
        }
        if( !obj->get_raw_accel_gyro( acel_gyro_d ) ) {
            //добавим калибровочное смещение (хардкод)
            obj->add_bias( acel_gyro_d );
            //приведём в физическим величинам
            for( axes_count = 0; axes_count < 3; axes_count++ ){
                a_data[ axes_count ]      = ( float )acel_gyro_d[ axes_count ] / 16384.0;   // g
                g_dataDeg[ axes_count ]   = ( float )acel_gyro_d[ axes_count+3 ] / 131.0;   // deg/sec
                g_dataRad[ axes_count ]   = DEGTORAD(g_dataDeg[axes_count]);                // преобразуем угловую скорость в рад/сек
            }

            if ( USER_OS_TAKE_BIN_SEMAPHORE( obj->m, portMAX_DELAY ) == pdTRUE){
                memcpy( obj->accelerometer, a_data, sizeof( a_data ) );
                memcpy( obj->magnetometer, g_dataRad, sizeof( g_dataRad ) );
                USER_OS_GIVE_BIN_SEMAPHORE( obj->m );

                if ( obj->cfg->ext_semaphore ) {
                    USER_OS_GIVE_BIN_SEMAPHORE( obj->cfg->ext_semaphore ); //синхронизация внешних потоков (если требуется).
                }
            }
        }
    }
}

int mpu6050_i2c::get_data ( float accelerometer[3], float magnetometer[3] ) {
    int rv = -1;
    if ( USER_OS_TAKE_BIN_SEMAPHORE( this->m, portMAX_DELAY ) == pdTRUE ){
        memcpy( accelerometer, this->accelerometer, sizeof( this->accelerometer) );
        memcpy( magnetometer, this->magnetometer,  sizeof( this->magnetometer ) );
        USER_OS_GIVE_BIN_SEMAPHORE( this->m );
        rv = 0;
    }
    return rv;
}

int mpu6050_i2c::get_data_sync ( float accelerometer[3], float magnetometer[3] ) {
    if ( this->cfg->ext_semaphore != nullptr ) {
        USER_OS_TAKE_BIN_SEMAPHORE( this->cfg->ext_semaphore, portMAX_DELAY); //ждём семафор внешней синхронизации
    }
    return get_data( accelerometer, magnetometer );
}

//**********************************************************************
// Служебные методы.
//**********************************************************************

//возвращает 0 в случае успеха
int mpu6050_i2c::get_raw_accel_gyro ( int16_t* accel_gyro ) {
    int i, rv = 1;
    uint8_t tmpBuffer[14];
  //  rv = MPU6050_I2C_BufferRead_DMA( MPU6050_DEFAULT_ADDRESS, tmpBuffer, MPU6050_RA_ACCEL_XOUT_H, 14);

    if ( rv != 0 ) return rv;

    /* Get acceleration */
    for (i = 0; i < 3; i++)
        accel_gyro[i] = ((uint16_t) ((uint16_t) tmpBuffer[2 * i] << 8) + tmpBuffer[2 * i + 1]);
    /* Get Angular rate */
    for (i = 4; i < 7; i++)
        accel_gyro[i - 1] = ((uint16_t) ((uint16_t) tmpBuffer[2 * i] << 8) + tmpBuffer[2 * i + 1]);
    return rv;
}

void mpu6050_i2c::add_bias ( int16_t *d ) {
    d[2] += 684;
    d[3] += 270;
    d[4] -= 50;
    d[5] -= 30;
}

void mpu6050_i2c::hardware_init ( void ) {
    /*
    //внутренним тактовым генератором назначим, генератор на оси X гироскопа
    MPU6050_WriteBits(d, MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, MPU6050_CLOCK_PLL_XGYRO);
    //установим размерность осей гироскопа 250 град/сек
    MPU6050_WriteBits(d, MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, MPU6050_GYRO_FS_250);
    //установим размерность осей акселерометра 2g
    MPU6050_WriteBits(d, MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, MPU6050_ACCEL_FS_2);
    //Включим ногу прерываний по готовности решения
    MPU6050_WriteBits(d, MPU6050_DEFAULT_ADDRESS, MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_DATA_RDY_BIT, 1, ENABLE);
    //Включим рабочий режим
    MPU6050_WriteBits(d, MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT,1, DISABLE);
    //Понизим частоту выдачи решения до 250Гц
    MPU6050_WriteBits(d, MPU6050_DEFAULT_ADDRESS, MPU6050_RA_SMPLRT_DIV, 4, 5, 0x1C);
*/
}

