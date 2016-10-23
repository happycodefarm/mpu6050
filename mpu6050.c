/*
 * mpu6050.c 
 * A basic MPU6050 I2C Accelerometer/Gyrosope Pd External for RaspberryPi
 * (cc) 2016, HappyCodeFarm, Guillaume Stagnaro. 
 * http://www.happycodefarm.net
 *
 * Dependency : wiringPi : http://wiringpi.com
 *
 * Provided under the terms of the WTFPL 2.0 license (Do What the Fuck You Want to Public License)
 *
 *
 *          DO WHAT THE FUCK YOU WANT TO PUBLIC LICENSE
 *                  Version 2, December 2004
 *
 * Copyright (C) 2004 Sam Hocevar <sam@hocevar.net>
 *
 * Everyone is permitted to copy and distribute verbatim or modified
 * copies of this license document, and changing it is allowed as long
 * as the name is changed.

 *          DO WHAT THE FUCK YOU WANT TO PUBLIC LICENSE
 * TERMS AND CONDITIONS FOR COPYING, DISTRIBUTION AND MODIFICATION
 *
 * 0. You just DO WHAT THE FUCK YOU WANT TO.
 */

#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>


// I2C registers defines.
#define MPU6050_ACCEL_XOUT_H       0x3B
#define MPU6050_ACCEL_YOUT_H       0x3D
#define MPU6050_ACCEL_ZOUT_H       0x3F

#define MPU6050_ACCEL_XOUT_L       0x3C
#define MPU6050_ACCEL_YOUT_L       0x3E
#define MPU6050_ACCEL_ZOUT_L       0x40

#define MPU6050_GYRO_XOUT_H        0x43
#define MPU6050_GYRO_YOUT_H        0x45
#define MPU6050_GYRO_ZOUT_H        0x47

#define MPU6050_GYRO_XOUT_L        0x44
#define MPU6050_GYRO_YOUT_L        0x46
#define MPU6050_GYRO_ZOUT_L        0x48

#define MPU6050_PWR_MGMT_1         0x6B
#define MPU6050_I2C_DEFAULT_ADDRESS        0x68

#include "m_pd.h"

// Structure definition of a single Pd 'mpu6050' object.
typedef struct mpu6050 {
  	t_object x_ob;

	t_outlet *xa_outlet;
	t_outlet *ya_outlet;
	t_outlet *za_outlet;

	t_outlet *xg_outlet;
    t_outlet *yg_outlet;
    t_outlet *zg_outlet;

	int fd;
} t_mpu6050;

// MPU6050 set device I2C address method.
void mpu6050_set_addr(t_mpu6050 *mpu, t_floatarg f) {
	unsigned char i2cAddr = (unsigned char)f;
	int fd = wiringPiI2CSetup(i2cAddr);
    if (fd == -1) {
            error("mpu6050: setup error, couldn't open device at I2C address %u (0x%x), %s.",i2cAddr, i2cAddr, strerror(errno));
            return ;
    } else {
		post("mpu6050: opening device at address: %u (0x%x).", i2cAddr, i2cAddr);
	}

	wiringPiI2CReadReg8 (fd, MPU6050_PWR_MGMT_1);
    wiringPiI2CWriteReg16(fd, MPU6050_PWR_MGMT_1, 0);

    mpu->fd = fd;
}

// MPU6050 update method. Output accelerometer and gyroscope values to corresponding outlets.
// All values are int16_t range : -32768 <-> 32767
void mpu6050_update(t_mpu6050 *mpu) {

	int16_t xa = ((int16_t)wiringPiI2CReadReg8(mpu->fd, MPU6050_ACCEL_XOUT_H)<<8) |
                        wiringPiI2CReadReg8(mpu->fd, MPU6050_ACCEL_XOUT_L);


	int16_t ya = ((int16_t)wiringPiI2CReadReg8(mpu->fd, MPU6050_ACCEL_YOUT_H)<<8) |
                        wiringPiI2CReadReg8(mpu->fd, MPU6050_ACCEL_YOUT_L);


	int16_t za = ((int16_t)wiringPiI2CReadReg8(mpu->fd, MPU6050_ACCEL_ZOUT_H)<<8) |
                        wiringPiI2CReadReg8(mpu->fd, MPU6050_ACCEL_ZOUT_L);


	int16_t xg = ((int16_t)wiringPiI2CReadReg8(mpu->fd, MPU6050_GYRO_XOUT_H)<<8) |
                        wiringPiI2CReadReg8(mpu->fd, MPU6050_GYRO_XOUT_L);


    int16_t yg = ((int16_t)wiringPiI2CReadReg8(mpu->fd, MPU6050_GYRO_YOUT_H)<<8) |
                        wiringPiI2CReadReg8(mpu->fd, MPU6050_GYRO_YOUT_L);


    int16_t zg = ((int16_t)wiringPiI2CReadReg8(mpu->fd, MPU6050_GYRO_ZOUT_H)<<8) |
                        wiringPiI2CReadReg8(mpu->fd, MPU6050_GYRO_ZOUT_L);


	outlet_float(mpu->xa_outlet,xa);
	outlet_float(mpu->ya_outlet,ya);
	outlet_float(mpu->za_outlet,za);

	outlet_float(mpu->xg_outlet,xg);
    outlet_float(mpu->yg_outlet,yg);
    outlet_float(mpu->zg_outlet,zg);
}

t_class *mpu6050_class;

// Create an instance of a Pd 'mpu6050' object.
void *mpu6050_new(t_symbol *selector, int argcount, t_atom *argvec) {
    post("mpu6050: new.");

    t_mpu6050 *mpu6050 = (t_mpu6050 *)pd_new(mpu6050_class);

 	mpu6050->fd = -1;

	// Check optional initial creation arguments. Argument is the I2C address of the MPU6050 chip.
	unsigned char i2cAddr = argcount>0 ? atom_getint(&argvec[0]) : MPU6050_I2C_DEFAULT_ADDRESS;

	// Open I2C device
	int fd = wiringPiI2CSetup(i2cAddr);
    if (fd == -1) {
        error("mpu6050: setup error, unable to open device at %u (0x%x), %s.", i2cAddr, i2cAddr,strerror(errno));
        return NULL ;
    } else {
        post("mpu6050: opening device at address %u, (0x%x).", i2cAddr, i2cAddr);
    }

	// Power on the mpu6050 device.
	wiringPiI2CReadReg8(fd, MPU6050_PWR_MGMT_1);
    wiringPiI2CWriteReg16(fd, MPU6050_PWR_MGMT_1, 0);

	// Set object I2C file descriptor.
	mpu6050->fd = fd;

	// Create object outlets : Accelerometer X,Y,Z and gyroscope X,Y,Z.
	mpu6050->xa_outlet = outlet_new(&mpu6050->x_ob,NULL);
	mpu6050->ya_outlet = outlet_new(&mpu6050->x_ob,NULL);
	mpu6050->za_outlet = outlet_new(&mpu6050->x_ob,NULL);

	mpu6050->xg_outlet = outlet_new(&mpu6050->x_ob,NULL);
    mpu6050->yg_outlet = outlet_new(&mpu6050->x_ob,NULL);
    mpu6050->zg_outlet = outlet_new(&mpu6050->x_ob,NULL);

	return (void *)mpu6050;
}

// Realease an instance of a Pd 'mpu6050' object.
void mpu6050_free(t_mpu6050 *mpu) {
	post("mpu6050: free.");
	if (mpu) {
		outlet_free(mpu->xa_outlet);
		outlet_free(mpu->ya_outlet);
		outlet_free(mpu->za_outlet);

		outlet_free(mpu->xg_outlet);
		outlet_free(mpu->yg_outlet);
		outlet_free(mpu->zg_outlet);

		mpu->xa_outlet = NULL;
   		mpu->ya_outlet = NULL;
   		mpu->za_outlet = NULL;

		mpu->xg_outlet = NULL;
   		mpu->yg_outlet = NULL;
   		mpu->zg_outlet = NULL;
	}
}

// Initialization of the Pd 'mpu6050' external.
void mpu6050_setup(void) {
    post("mpu6050: setup.");

    mpu6050_class = class_new(gensym("mpu6050"),		// t_symbol *name
				(t_newmethod)mpu6050_new,	// new method
				(t_method)mpu6050_free,		// free method
                sizeof(t_mpu6050),		// size
				0,				// int flag
				A_GIMME,0);			// t_atomtype, arg1

    	// Add device address and refresh methods
	class_addfloat(mpu6050_class, mpu6050_set_addr);
	class_addbang(mpu6050_class, mpu6050_update);
}
