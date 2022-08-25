#ifndef __TMC5130_REG_DEF_H
#define __TMC5130_REG_DEF_H

/*  TMC5130 Register Address Defines */

#define GCONF 0x00 /**< Please check datasheet for register description */

#define DIRECTION(n) (((n)&0x1) << 4)	  /**< Please check datasheet for register description */
#define EN_PWM_MODE(n) (((n)&0x1) << 2)	/**< Please check datasheet for register description */
#define I_SCALE_ANALOG(n) (((n)&0x1) << 0) /**< Please check datasheet for register description */

#define GSTAT 0x01		/**< Please check datasheet for register description */
#define X_COMPARE 0x05  /**< Please check datasheet for register description */
#define IHOLD_IRUN 0x10 /**< Please check datasheet for register description */
#define TPOWERDOWN 0x11 /**< Please check datasheet for register description */
#define TSTEP 0x12
#define TPWMTHRS 0x13  /**< Please check datasheet for register description */
#define TCOOLTHRS 0x14 /**< Please check datasheet for register description */
#define THIGH 0x15	 /**< Please check datasheet for register description */
#define RAMPMODE 0x20  /**< Please check datasheet for register description */
#define XACTUAL 0x21   /**< Please check datasheet for register description */
#define VACTUAL 0x22   /**< Please check datasheet for register description */

/** Ramp curves */
#define VSTART_REG 0x23				 /**< Please check datasheet for register description */
#define A1_REG 0x24					 /**< Please check datasheet for register description */
#define V1_REG 0x25					 /**< Please check datasheet for register description */
#define AMAX_REG 0x26				 /**< Please check datasheet for register description */
#define VMAX_REG 0x27				 /**< Please check datasheet for register description */
#define DMAX_REG 0x28				 /**< Please check datasheet for register description */
#define D1_REG 0x2A					 /**< Please check datasheet for register description */
#define VSTOP_REG 0x2B				 /**< Please check datasheet for register description */
#define TZEROWAIT 0x2C				 /**< Please check datasheet for register description */
#define XTARGET 0x2D				 /**< Please check datasheet for register description */
#define VDCMIN 0x33					 /**< Please check datasheet for register description */
#define SW_MODE 0x34				 /**< Please check datasheet for register description */
#define SG_STOP(n) (((n)&0x1) << 10) /**< Please check datasheet for register description */
#define RAMP_STAT 0x35				 /**< Please check datasheet for register description */
#define XLATCH 0x36					 /**< Please check datasheet for register description */

#define PWMCONF 0x70 /**< Please check datasheet for register description */

#define FREEWHEEL(n) (((n)&0x3UL) << 20)	 /**< Please check datasheet for register description */
#define PWM_AUTOSCALE(n) (((n)&0x1UL) << 18) /**< Please check datasheet for register description */
#define PWM_FREQ(n) (((n)&0x3UL) << 16)		 /**< Please check datasheet for register description */
#define PWM_GRAD(n) (((n)&0xFF) << 8)		 /**< Please check datasheet for register description */
#define PWM_AMPL(n) (((n)&0xFF) << 0)		 /**< Please check datasheet for register description */

/** Chopper and driver configuration */

#define CHOPCONF 0x6C /**< Please check datasheet for register description */

#define DISS2G(n) (((n)&0x1UL) << 30)   /**< Please check datasheet for register description */
#define DEDGE(n) (((n)&0x1UL) << 29)	/**< Please check datasheet for register description */
#define INTPOL(n) (((n)&0x1UL) << 28)   /**< Please check datasheet for register description */
#define MRES(n) (((n)&0xFUL) << 24)		/**< Please check datasheet for register description */
#define SYNC(n) (((n)&0xFUL) << 20)		/**< Please check datasheet for register description */
#define VHIGHCHM(n) (((n)&0x1UL) << 19) /**< Please check datasheet for register description */
#define VHIGHFS(n) (((n)&0x1UL) << 18)  /**< Please check datasheet for register description */
#define VSENSE(n) (((n)&0x1UL) << 17)   /**< Please check datasheet for register description */
#define TBL(n) (((n)&0x3UL) << 15)		/**< Please check datasheet for register description */
#define CHM(n) (((n)&0x1UL) << 14)		/**< Please check datasheet for register description */
#define RNDTF(n) (((n)&0x1) << 13)		/**< Please check datasheet for register description */
#define DISFDCC(n) (((n)&0x1) << 12)	/**< Please check datasheet for register description */
#define TFD3(n) (((n)&0x1) << 11)		/**< Please check datasheet for register description */
#define HEND(n) (((n)&0xF) << 7)		/**< Please check datasheet for register description */
#define HSTRT_TFD(n) (((n)&0x7) << 4)   /**< Please check datasheet for register description */
#define TOFF(n) (((n)&0xF) << 0)		/**< Please check datasheet for register description */

/* CoolStep smart current control register and stallGuard2 configuration **/

#define COOLCONF 0x6D				  /**< Please check datasheet for register description */
#define SFILT(n) (((n)&0x1UL) << 24)  /**< Please check datasheet for register description */
#define SGT(n) (((n)&0x7FUL) << 16)   /**< Please check datasheet for register description */
#define SEIMIN(n) (((n)&0x1UL) << 15) /**< Please check datasheet for register description */
#define SEDN(n) (((n)&0x3) << 13)	 /**< Please check datasheet for register description */
#define SEMAX(n) (((n)&0xF) << 8)	 /**< Please check datasheet for register description */
#define SEUP(n) (((n)&0x3) << 5)	  /**< Please check datasheet for register description */
#define SEMIN(n) (((n)&0xF) << 0)	 /**< Please check datasheet for register description */

#define DCCTRL 0x6E					  /**< Please check datasheet for register description */
#define DC_SG(n) (((n)&0xFFUL) << 16) /**< Please check datasheet for register description */
#define DC_TIME(n) (((n)&0x3FF) << 0) /**< Please check datasheet for register description */

#define DRV_STATUS 0x6F /**< stallGuard2 value and driver error flags*/

#define IHOLDDELAY(n) (((n)&0xFUL) << 16) /**< Please check datasheet for register description */
#define IRUN(n) (((n)&0x1F) << 8)		  /**< Please check datasheet for register description */
#define IHOLD(n) (((n)&0x1F) << 0)		  /**< Please check datasheet for register description */

#define WRITE_ACCESS 0x80 /**< Write access for spi communication*/

/** Modes for RAMPMODE register */

#define POSITIONING_MODE 0x00  /**< using all A, D and V parameters)*/
#define VELOCITY_MODE_POS 0x01 /**< positiv VMAX, using AMAX acceleration*/
#define VELOCITY_MODE_NEG 0x02 /**< negativ VMAX, using AMAX acceleration*/
#define HOLD_MODE 0x03		   /**< velocity remains unchanged, unless stop event occurs*/

#define DRIVER_STOP 0	 /**< Define label for indicating driver is in standstill mode */
#define DRIVER_VELOCITY 1 /**< Define label for indicating driver is in velocity mode */
#define DRIVER_POSITION 2 /**< Define label for indicating driver is in position mode */

#endif