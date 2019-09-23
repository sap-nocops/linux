#include "tpd_directives.h"

//#define I2C_RST                     GPIO43
//#define I2C_SCL                     GPIO181
//#define I2C_SDA                     GPIO182

//#define RST_GPIO_MODE               0
//#define SCL_GPIO_MODE               0
//#define SDA_GPIO_MODE               0

#define I2C_LOW                       0
#define I2C_HIGHT                     1

#define PASS                        1
#define FAIL                        0

//  flash block is 128 bytes. Note Block-Verify Uses 64-Bytes of RAM
#define TARGET_DATABUFF_LEN         128  			
											
#ifdef CY8CTMA300
    #define NUM_BANKS               1
    #define BLOCKS_PER_BANK         256
    #define SECURITY_BYTES_PER_BANK 64
#endif
#ifdef CY8CTMA140
    #define NUM_BANKS               1
    #define BLOCKS_PER_BANK         256
    #define SECURITY_BYTES_PER_BANK 64
#endif
#ifdef CY8CTMA301D
    #define NUM_BANKS               1
    #define BLOCKS_PER_BANK         128
    #define SECURITY_BYTES_PER_BANK 64
#endif
#ifdef CY8CTMA300E
    #define NUM_BANKS               1
    #define BLOCKS_PER_BANK         256
    #define SECURITY_BYTES_PER_BANK 64
    #define ACTIVE_LOW_XRES
#endif
#ifdef CY8CTMA301E
    #define NUM_BANKS               1
    #define BLOCKS_PER_BANK         128
    #define SECURITY_BYTES_PER_BANK 64
    #define ACTIVE_LOW_XRES
#endif

// TRANSITION_TIMEOUT is a loop counter for a 100msec timeout when waiting for 
// a high-to-low transition. This is used in the polling loop of 
// fDetectHiLoTransition(). Each pass through the loop takes approximately 15
// usec. 100 msec is about 6740 loops. 200ms = 13480
#define TRANSITION_TIMEOUT          13480

#define XRES_CLK_DELAY              300
#define CLK_DELAY              		6

#define WAIT_TIME                   100

#define debug_printk(args...) \
        do { \
                if (debug) { \
                    printk("\n[tpd_upgrader] %s: ", __func__); \
                    printk(args); \
                } \
        } while (0)

