# 1 "main.c"
# 1 "/home/vonbana/Programme/MicroController/AVR/Gyrosensor_Seed/Gyro_LIB_AvB//"
# 1 "<built-in>"
# 1 "<command-line>"
# 1 "main.c"

# 1 "/usr/lib/avr/include/avr/io.h" 1 3
# 99 "/usr/lib/avr/include/avr/io.h" 3
# 1 "/usr/lib/avr/include/avr/sfr_defs.h" 1 3
# 126 "/usr/lib/avr/include/avr/sfr_defs.h" 3
# 1 "/usr/lib/avr/include/inttypes.h" 1 3
# 37 "/usr/lib/avr/include/inttypes.h" 3
# 1 "/usr/lib/gcc/avr/4.9.2/include/stdint.h" 1 3 4
# 9 "/usr/lib/gcc/avr/4.9.2/include/stdint.h" 3 4
# 1 "/usr/lib/avr/include/stdint.h" 1 3 4
# 121 "/usr/lib/avr/include/stdint.h" 3 4
typedef signed int int8_t __attribute__((__mode__(__QI__)));
typedef unsigned int uint8_t __attribute__((__mode__(__QI__)));
typedef signed int int16_t __attribute__ ((__mode__ (__HI__)));
typedef unsigned int uint16_t __attribute__ ((__mode__ (__HI__)));
typedef signed int int32_t __attribute__ ((__mode__ (__SI__)));
typedef unsigned int uint32_t __attribute__ ((__mode__ (__SI__)));

typedef signed int int64_t __attribute__((__mode__(__DI__)));
typedef unsigned int uint64_t __attribute__((__mode__(__DI__)));
# 142 "/usr/lib/avr/include/stdint.h" 3 4
typedef int16_t intptr_t;




typedef uint16_t uintptr_t;
# 159 "/usr/lib/avr/include/stdint.h" 3 4
typedef int8_t int_least8_t;




typedef uint8_t uint_least8_t;




typedef int16_t int_least16_t;




typedef uint16_t uint_least16_t;




typedef int32_t int_least32_t;




typedef uint32_t uint_least32_t;







typedef int64_t int_least64_t;






typedef uint64_t uint_least64_t;
# 213 "/usr/lib/avr/include/stdint.h" 3 4
typedef int8_t int_fast8_t;




typedef uint8_t uint_fast8_t;




typedef int16_t int_fast16_t;




typedef uint16_t uint_fast16_t;




typedef int32_t int_fast32_t;




typedef uint32_t uint_fast32_t;







typedef int64_t int_fast64_t;






typedef uint64_t uint_fast64_t;
# 273 "/usr/lib/avr/include/stdint.h" 3 4
typedef int64_t intmax_t;




typedef uint64_t uintmax_t;
# 10 "/usr/lib/gcc/avr/4.9.2/include/stdint.h" 2 3 4
# 38 "/usr/lib/avr/include/inttypes.h" 2 3
# 77 "/usr/lib/avr/include/inttypes.h" 3
typedef int32_t int_farptr_t;



typedef uint32_t uint_farptr_t;
# 127 "/usr/lib/avr/include/avr/sfr_defs.h" 2 3
# 100 "/usr/lib/avr/include/avr/io.h" 2 3
# 210 "/usr/lib/avr/include/avr/io.h" 3
# 1 "/usr/lib/avr/include/avr/iom644p.h" 1 3
# 38 "/usr/lib/avr/include/avr/iom644p.h" 3
# 1 "/usr/lib/avr/include/avr/iomxx4.h" 1 3
# 918 "/usr/lib/avr/include/avr/iomxx4.h" 3
       
# 919 "/usr/lib/avr/include/avr/iomxx4.h" 3

       
       
       
       
       
       
       
       
       
       
       
       
       
       
       
       
       
       
       
       
       
       
       
       
       
       
       
       
       
       
# 39 "/usr/lib/avr/include/avr/iom644p.h" 2 3
# 211 "/usr/lib/avr/include/avr/io.h" 2 3
# 627 "/usr/lib/avr/include/avr/io.h" 3
# 1 "/usr/lib/avr/include/avr/portpins.h" 1 3
# 628 "/usr/lib/avr/include/avr/io.h" 2 3

# 1 "/usr/lib/avr/include/avr/common.h" 1 3
# 630 "/usr/lib/avr/include/avr/io.h" 2 3

# 1 "/usr/lib/avr/include/avr/version.h" 1 3
# 632 "/usr/lib/avr/include/avr/io.h" 2 3






# 1 "/usr/lib/avr/include/avr/fuse.h" 1 3
# 239 "/usr/lib/avr/include/avr/fuse.h" 3
typedef struct
{
    unsigned char low;
    unsigned char high;
    unsigned char extended;
} __fuse_t;
# 639 "/usr/lib/avr/include/avr/io.h" 2 3


# 1 "/usr/lib/avr/include/avr/lock.h" 1 3
# 642 "/usr/lib/avr/include/avr/io.h" 2 3
# 3 "main.c" 2

# 1 "i2cmaster.h" 1
# 102 "i2cmaster.h"
extern void i2c_init(void);






extern void i2c_stop(void);
# 119 "i2cmaster.h"
extern unsigned char i2c_start(unsigned char addr);
# 129 "i2cmaster.h"
extern unsigned char i2c_rep_start(unsigned char addr);
# 139 "i2cmaster.h"
extern void i2c_start_wait(unsigned char addr);
# 148 "i2cmaster.h"
extern unsigned char i2c_write(unsigned char data);






extern unsigned char i2c_readAck(void);





extern unsigned char i2c_readNak(void);
# 172 "i2cmaster.h"
extern unsigned char i2c_read(unsigned char ack);
# 5 "main.c" 2
# 1 "mpu9250.h" 1
# 34 "mpu9250.h"
# 1 "/usr/lib/gcc/avr/4.9.2/include/stdbool.h" 1 3 4
# 35 "mpu9250.h" 2
# 1 "/usr/lib/avr/include/math.h" 1 3
# 127 "/usr/lib/avr/include/math.h" 3
extern double cos(double __x) __attribute__((__const__));





extern double sin(double __x) __attribute__((__const__));





extern double tan(double __x) __attribute__((__const__));






extern double fabs(double __x) __attribute__((__const__));






extern double fmod(double __x, double __y) __attribute__((__const__));
# 168 "/usr/lib/avr/include/math.h" 3
extern double modf(double __x, double *__iptr);



extern float modff (float __x, float *__iptr);




extern double sqrt(double __x) __attribute__((__const__));
extern float sqrtf (float) __attribute__((__const__));




extern double cbrt(double __x) __attribute__((__const__));
# 194 "/usr/lib/avr/include/math.h" 3
extern double hypot (double __x, double __y) __attribute__((__const__));







extern double square(double __x) __attribute__((__const__));






extern double floor(double __x) __attribute__((__const__));






extern double ceil(double __x) __attribute__((__const__));
# 234 "/usr/lib/avr/include/math.h" 3
extern double frexp(double __x, int *__pexp);







extern double ldexp(double __x, int __exp) __attribute__((__const__));





extern double exp(double __x) __attribute__((__const__));





extern double cosh(double __x) __attribute__((__const__));





extern double sinh(double __x) __attribute__((__const__));





extern double tanh(double __x) __attribute__((__const__));







extern double acos(double __x) __attribute__((__const__));







extern double asin(double __x) __attribute__((__const__));






extern double atan(double __x) __attribute__((__const__));
# 298 "/usr/lib/avr/include/math.h" 3
extern double atan2(double __y, double __x) __attribute__((__const__));





extern double log(double __x) __attribute__((__const__));





extern double log10(double __x) __attribute__((__const__));





extern double pow(double __x, double __y) __attribute__((__const__));






extern int isnan(double __x) __attribute__((__const__));
# 333 "/usr/lib/avr/include/math.h" 3
extern int isinf(double __x) __attribute__((__const__));






__attribute__((__const__)) static inline int isfinite (double __x)
{
    unsigned char __exp;
    __asm__ (
 "mov	%0, %C1		\n\t"
 "lsl	%0		\n\t"
 "mov	%0, %D1		\n\t"
 "rol	%0		"
 : "=r" (__exp)
 : "r" (__x) );
    return __exp != 0xff;
}






__attribute__((__const__)) static inline double copysign (double __x, double __y)
{
    __asm__ (
 "bst	%D2, 7	\n\t"
 "bld	%D0, 7	"
 : "=r" (__x)
 : "0" (__x), "r" (__y) );
    return __x;
}
# 376 "/usr/lib/avr/include/math.h" 3
extern int signbit (double __x) __attribute__((__const__));






extern double fdim (double __x, double __y) __attribute__((__const__));
# 392 "/usr/lib/avr/include/math.h" 3
extern double fma (double __x, double __y, double __z) __attribute__((__const__));







extern double fmax (double __x, double __y) __attribute__((__const__));







extern double fmin (double __x, double __y) __attribute__((__const__));






extern double trunc (double __x) __attribute__((__const__));
# 426 "/usr/lib/avr/include/math.h" 3
extern double round (double __x) __attribute__((__const__));
# 439 "/usr/lib/avr/include/math.h" 3
extern long lround (double __x) __attribute__((__const__));
# 453 "/usr/lib/avr/include/math.h" 3
extern long lrint (double __x) __attribute__((__const__));
# 36 "mpu9250.h" 2

# 1 "mpu9250_registers.h" 1
# 38 "mpu9250.h" 2
# 61 "mpu9250.h"
enum Ascale {
  AFS_2G = 0,
  AFS_4G,
  AFS_8G,
  AFS_16G
};

enum Gscale {
  GFS_250DPS = 0,
  GFS_500DPS,
  GFS_1000DPS,
  GFS_2000DPS
};

enum Mscale {
  MFS_14BITS = 0,
  MFS_16BITS
};


uint8_t Gscale = GFS_250DPS;
uint8_t Ascale = AFS_2G;
uint8_t Mscale = MFS_16BITS;
uint8_t Mmode = 0x02;
float aRes, gRes, mRes;

float temperature;
float SelfTest[6];

float magBias[3],magScale[3];

float magCalibration[3] = {0, 0, 0}, magbias[3] = {0, 0, 0};
float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0};
int16_t tempCount;
# 114 "mpu9250.h"
void mpu9250_setup(void);


void writeByte(uint8_t address, uint8_t subAddress, uint8_t data);
uint8_t readByte(uint8_t address, uint8_t subAddress);
void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest);
void magcalMPU9250(float * dest1, float * dest2);
void getMres(void);
void getGres(void);
void getAres(void);
void readAccelData(int16_t * destination);
void getMres(void);
void readMagData(int16_t * destination);
int16_t readTempData(void);
void initAK8963(float * destination);
void readGyroData(int16_t * destination);
void initMPU9250(void);
void MPU9250SelfTest(float * destination);
void calibrateMPU9250(float * dest1, float * dest2);
void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
# 6 "main.c" 2
# 1 "uart.h" 1
# 100 "uart.h"
void UART_Init(uint32_t v_baudRate_u32);
void UART_SetBaudRate(uint32_t v_baudRate_u32);
void UART_TxChar(char v_uartData_u8);
char UART_RxChar(void);
void UART_TxString(char *ptr_string);
uint8_t UART_RxString(char *ptr_string);
void UART_TxNumber(uint8_t v_numericSystem_u8, uint32_t v_number_u32, uint8_t v_numOfDigitsToTransmit_u8);
void UART_TxFloatNumber(float v_floatNumber_f32);
void UART_Printf(const char *argList, ...);
# 7 "main.c" 2
# 1 "millis.h" 1
# 49 "millis.h"
void millis_init(void);






unsigned long millis_get(void);






unsigned long micros_get(void);






void millis_resume(void);






void millis_pause(void);






void millis_reset(void);







void millis_add(unsigned long ms);







void millis_subtract(unsigned long ms);
# 8 "main.c" 2

int16_t accelCount[3];
int16_t gyroCount[3];
int16_t magCount[3];

float ax, ay, az, gx, gy, gz, mx, my, mz;
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
float eInt[3] = {0.0f, 0.0f, 0.0f};

uint32_t lastUpdate = 0;
unsigned long Now = 0;
float deltat = 0.0f, sum = 0.0f;
float pitch, yaw, roll;
uint32_t delt_t = 0;
uint32_t count = 0, sumCount = 0;


int main(void)
{

     i2c_init();

 UART_Init(115200);

   mpu9250_setup();


 while(1)
 {

    if (readByte(0x68, 0x3A) & 0x01)
  {
       readAccelData(accelCount);
       getAres();


       ax = (float)accelCount[0]*aRes;
       ay = (float)accelCount[1]*aRes;
       az = (float)accelCount[2]*aRes;

       readGyroData(gyroCount);
       getGres();


       gx = (float)gyroCount[0]*gRes;
       gy = (float)gyroCount[1]*gRes;
       gz = (float)gyroCount[2]*gRes;

       readMagData(magCount);
       getMres();






       mx = (float)magCount[0]*mRes*magCalibration[0] - magBias[0];
       my = (float)magCount[1]*mRes*magCalibration[1] - magBias[1];
       mz = (float)magCount[2]*mRes*magCalibration[2] - magBias[2];
    }

    Now = micros_get();
    deltat = ((Now - lastUpdate)/1000000.0f);
    lastUpdate = Now;

    sum += deltat;
    sumCount++;
# 83 "main.c"
    MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, my, mx, mz , q);



      if (!1)
  {
       delt_t = millis_get() - count;
       if(delt_t > 500)
   {
        if(1)
    {
# 109 "main.c"
         tempCount = readTempData();
         temperature = ((float) tempCount) / 333.87 + 21.0;

         UART_Printf("Temperature is %d in °/C\n",temperature, 1);
        }

        count = millis_get();
        digitalWrite(myLed, !digitalRead(myLed));
       }
      }
      else
      {


       delt_t = millis_get() - count;
       if (delt_t > 50)
   {

        if(1)
    {
# 143 "main.c"
        }
# 154 "main.c"
        yaw = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
        pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
        roll = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
        pitch *= 180.0f / PI;
        yaw *= 180.0f / PI;
        yaw -= 2.45;
        roll *= 180.0f / PI;


        UART_Printf("Yaw, Pitch, Roll: %d , %d , %d\n" , yaw+180, pitch, roll);
# 177 "main.c"
        count = millis_get();
        sumCount = 0;
        sum = 0;

   }
  }
 }
 return -1;
}
