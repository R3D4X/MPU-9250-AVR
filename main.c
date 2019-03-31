//#include <Wire.h>
#include <avr/io.h>

#include "i2cmaster.h"
#include "mpu9250.h"
#include "uart.h"
#include "millis.h"

int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
int16_t magCount[3];    // Stores the 16-bit signed magnetometer sensor output

float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values 
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
float eInt[3] = {0.0f, 0.0f, 0.0f};       // vector to hold integral error for Mahony method

uint32_t lastUpdate = 0; // used to calculate integration interval
unsigned long Now = 0;        // used to calculate integration interval
float deltat = 0.0f, sum = 0.0f;        // integration interval for both filter schemes
float pitch, yaw, roll;
uint32_t delt_t = 0; // used to control display output rate
uint32_t count = 0, sumCount = 0; // used to control display output rate

//void setup()
int main(void)
{
	//Wire.begin();
    	i2c_init();                                // init I2C interface
  	//Serial.begin(115200);
	UART_Init(115200);

  	mpu9250_setup();

	//void loop()
	while(1)
	{  
  		// If intPin goes high, all data registers have new data
  		if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01) 
		{  // On interrupt, check if data ready interrupt
    			readAccelData(accelCount);  // Read the x/y/z adc values
    			getAres();
    
    			// Now we'll calculate the accleration value into actual g's
    			ax = (float)accelCount[0]*aRes; // - accelBias[0];  // get actual g value, this depends on scale being set
    			ay = (float)accelCount[1]*aRes; // - accelBias[1];   
    			az = (float)accelCount[2]*aRes; // - accelBias[2];  
   
    			readGyroData(gyroCount);  // Read the x/y/z adc values
    			getGres();
 
    			// Calculate the gyro value into actual degrees per second
    			gx = (float)gyroCount[0]*gRes;  // get actual gyro value, this depends on scale being set
    			gy = (float)gyroCount[1]*gRes;  
    			gz = (float)gyroCount[2]*gRes;   
  
    			readMagData(magCount);  // Read the x/y/z adc values
    			getMres();
//    			magbias[0] = +470.;  // User environmental x-axis correction in milliGauss, should be automatically calculated
//    			magbias[1] = +120.;  // User environmental x-axis correction in milliGauss
//    			magbias[2] = +125.;  // User environmental x-axis correction in milliGauss
    
    			// Calculate the magnetometer values in milliGauss
    			// Include factory calibration per data sheet and user environmental corrections
    			mx = (float)magCount[0]*mRes*magCalibration[0] - magBias[0];  // get actual magnetometer value, this depends on scale being set
    			my = (float)magCount[1]*mRes*magCalibration[1] - magBias[1];  
    			mz = (float)magCount[2]*mRes*magCalibration[2] - magBias[2];   
  		}
  
  		Now = micros_get();
  		deltat = ((Now - lastUpdate)/1000000.0f); // set integration time by time elapsed since last filter update
  		lastUpdate = Now;

  		sum += deltat; // sum for averaging filter update rate
  		sumCount++;
  
  		// Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of the magnetometer;
  		// the magnetometer z-axis (+ down) is opposite to z-axis (+ up) of accelerometer and gyro!
  		// We have to make some allowance for this orientationmismatch in feeding the output to the quaternion filter.
  		// For the MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward along the x-axis just like
  		// in the LSM9DS0 sensor. This rotation can be modified to allow any convenient orientation convention.
  		// This is ok by aircraft orientation standards!  
  		// Pass gyro rate as rad/s
  		MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f,  my,  mx, mz , q);
//  		MahonyQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, my, mx, mz);


    		if (!AHRS) 
		{
    			delt_t = millis() - count;
    			if(delt_t > 500) 
			{
    				if(SerialDebug) 
				{
    				// Print acceleration values in milligs!
//    					UART_Printf("X-acceleration: "); UART_Printf(1000*ax); UART_Printf(" mg ");
//    					UART_Printf("Y-acceleration: "); UART_Printf(1000*ay); UART_Printf(" mg ");
//    					UART_Printf("Z-acceleration: "); UART_Printf(1000*az); UART_Printf(" mg "); UART_Printf("\n");
// 
//    				// Print gyro values in degree/sec
//    					UART_Printf("X-gyro rate: "); UART_Printf(gx, 3); UART_Printf(" degrees/sec "); 
//    					UART_Printf("Y-gyro rate: "); UART_Printf(gy, 3); UART_Printf(" degrees/sec "); 
//    					UART_Printf("Z-gyro rate: "); UART_Printf(gz, 3); UART_Printf(" degrees/sec"); UART_Printf("\n");
//    
//    				// Print mag values in degree/sec
//    					UART_Printf("X-mag field: "); UART_Printf(mx); UART_Printf(" mG "); 
//    					UART_Printf("Y-mag field: "); UART_Printf(my); UART_Printf(" mG "); 
//    					UART_Printf("Z-mag field: "); UART_Printf(mz); UART_Printf(" mG"); UART_Printf("\n");
 
    					tempCount = readTempData();  // Read the adc values
    					temperature = ((float) tempCount) / 333.87 + 21.0; // Temperature in degrees Centigrade
   				// Print temperature in degrees Centigrade      
    					UART_Printf("Temperature is %d in °/C\n",temperature, 1);
    				}
    
    				count = millis();
    				digitalWrite(myLed, !digitalRead(myLed));  // toggle led
    			}
    		}
    		else 
    		{
      
    			// UART_Printf and/or display at 0.5 s rate independent of data rates
    			delt_t = millis() - count;
    			if (delt_t > 50) 
			{ // update LCD once per half-second independent of read rate

    				if(SerialDebug) 
				{
//    					UART_Printf("ax = "); UART_Printf((int)1000*ax);  
//    					UART_Printf(" ay = "); UART_Printf((int)1000*ay); 
//    					UART_Printf(" az = "); UART_Printf((int)1000*az); UART_Printfln(" mg");
//    					UART_Printf("gx = "); UART_Printf( gx, 2); 
//    					UART_Printf(" gy = "); UART_Printf( gy, 2); 
//    					UART_Printf(" gz = "); UART_Printf( gz, 2); UART_Printfln(" deg/s");
//    					UART_Printf("mx = "); UART_Printf( (int)mx ); 
//    					UART_Printf(" my = "); UART_Printf( (int)my ); 
//    					UART_Printf(" mz = "); UART_Printf( (int)mz ); UART_Printfln(" mG");
//    	
//    					UART_Printf("q0 = "); UART_Printf(q[0]);
//   	 				UART_Printf(" qx = "); UART_Printf(q[1]); 
//   	 				UART_Printf(" qy = "); UART_Printf(q[2]); 
//   	 				UART_Printf(" qz = "); UART_Printfln(q[3]); 
    				}               
    
  				// Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
  				// In this coordinate system, the positive z-axis is down toward Earth. 
  				// Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
  				// Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
  				// Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
  				// These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
  				// Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
  				// applied in the correct order which for this configuration is yaw, pitch, and then roll.
  				// For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
  	  			yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);   
  	  			pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
  	  			roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
  	  			pitch *= 180.0f / PI;
  	  			yaw   *= 180.0f / PI; 
    				yaw   -= 2.45; /* 2019-03-30	2° 45' E  changing by  0.083° E per year (+ve for west )*/
    				roll  *= 180.0f / PI;
     

    				UART_Printf("Yaw, Pitch, Roll: %d , %d , %d\n" , yaw+180, pitch, roll);
    
  	  			// With these settings the filter is updating at a ~145 Hz rate using the Madgwick scheme and 
  	  			// >200 Hz using the Mahony scheme even though the display refreshes at only 2 Hz.
  	  			// The filter update rate is determined mostly by the mathematical steps in the respective algorithms, 
  	  			// the processor speed (8 MHz for the 3.3V Pro Mini), and the magnetometer ODR:
  	  			// an ODR of 10 Hz for the magnetometer produce the above rates, maximum magnetometer ODR of 100 Hz produces
  	  			// filter update rates of 36 - 145 and ~38 Hz for the Madgwick and Mahony schemes, respectively. 
  	  			// This is presumably because the magnetometer read takes longer than the gyro or accelerometer reads.
  	  			// This filter update rate should be fast enough to maintain accurate platform orientation for 
  	  			// stabilization control of a fast-moving robot or quadcopter. Compare to the update rate of 200 Hz
  	  			// produced by the on-board Digital Motion Processor of Invensense's MPU6050 6 DoF and MPU9150 9DoF sensors.
  	  			// The 3.3 V 8 MHz Pro Mini is doing pretty well!
	
	    			count = millis(); 
	    			sumCount = 0;
	    			sum = 0;    
	
			}
		}
	}
	return -1;
}
