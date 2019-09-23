#include <linux/fs.h>
#include <linux/file.h>
#include <linux/delay.h>

//#include <mach/mt6573_gpio.h>
//#include <mach/mt6573_pll.h>

#include "tpd_defs.h"
#include "tpd_vectors.h"
#include "tpd_directives.h"

#include <mach/sys_config.h>
#include <mach/gpio.h>

unsigned char target_data_in;
unsigned char target_data_out[TARGET_DATABUFF_LEN];

unsigned char target_address;
unsigned char target_data_ptr = 0;
unsigned char target_id[10];
unsigned char target_status[10];

extern int debug;

extern int gpio_i2c_scl_hdle;
extern int gpio_i2c_sda_hdle;
extern int gpio_i2c_reset_hdle;

extern user_gpio_set_t gpio_i2c_scl_info;
extern user_gpio_set_t gpio_i2c_sda_info;
extern user_gpio_set_t gpio_i2c_reset_info;

int tpd_load_program_data(unsigned char bank_num, unsigned char block_num, struct file *fp)
{
    unsigned char buffer[TARGET_DATABUFF_LEN];
    unsigned char temp[2];
    int retval, i = 0, index = 0;

    retval = fp->f_op->read(fp, buffer, 9, &fp->f_pos);

    for (i = 0; i < 64; i++, index++)
    {
        retval = fp->f_op->read(fp, temp, 2, &fp->f_pos);

        if (((temp[0] & 0xF0) >> 4) == 0x03)
            target_data_out[index] = (temp[0] & 0x0F) << 4;
        
        if (((temp[0] & 0xF0) >> 4) == 0x06 || ((temp[0] & 0xF0) >> 4) == 0x04)
            target_data_out[index] = ((temp[0] & 0x0F) + 9) << 4;

        if (((temp[1] & 0xF0) >> 4) == 0x03)
            target_data_out[index] = target_data_out[index] | (temp[1] & 0x0F);
        
        if (((temp[1] & 0xF0) >> 4) == 0x06 || ((temp[1] & 0xF0) >> 4) == 0x04)
            target_data_out[index] = target_data_out[index] | ((temp[1] & 0x0F) + 9);    
    }
    msleep(1);

    retval = fp->f_op->read(fp, buffer, 4, &fp->f_pos);

    retval = fp->f_op->read(fp, buffer, 9, &fp->f_pos);
    
    for (i = 0; i < 64; i++, index++)
    {
        retval = fp->f_op->read(fp, temp, 2, &fp->f_pos);

        if (((temp[0] & 0xF0) >> 4) == 0x03)
            target_data_out[index] = (temp[0] & 0x0F) << 4;
        
        if (((temp[0] & 0xF0) >> 4) == 0x06 || ((temp[0] & 0xF0) >> 4) == 0x04)
            target_data_out[index] = ((temp[0] & 0x0F) + 9) << 4;

        if (((temp[1] & 0xF0) >> 4) == 0x03)
            target_data_out[index] = target_data_out[index] | (temp[1] & 0x0F);
        
        if (((temp[1] & 0xF0) >> 4) == 0x06 || ((temp[1] & 0xF0) >> 4) == 0x04)
            target_data_out[index] = target_data_out[index] | ((temp[1] & 0x0F) + 9);    
    }
    msleep(1);

    retval = fp->f_op->read(fp, buffer, 4, &fp->f_pos);

    if (debug)
    {
        debug_printk("read data from file:\n");
        for (i = 0; i < TARGET_DATABUFF_LEN; i++)
        {
            debug_printk("%02X ", target_data_out[i]);
        }
        debug_printk("\n");
    }
    return PASS;
}

int tpd_load_security_data(unsigned char bank_num, struct file *fp)
{ 
    unsigned char buffer[TARGET_DATABUFF_LEN];
    unsigned char temp[2];
    int retval, i = 0, index = 0;

    for (i = 0; i < BLOCKS_PER_BANK; i++)
    {
        retval = fp->f_op->read(fp, buffer, 71, &fp->f_pos);
        retval = fp->f_op->read(fp, buffer, 70, &fp->f_pos);
        retval = fp->f_op->read(fp, buffer, 71, &fp->f_pos);
        retval = fp->f_op->read(fp, buffer, 70, &fp->f_pos);
        msleep(1);
    }

    retval = fp->f_op->read(fp, buffer, 17, &fp->f_pos);

    retval = fp->f_op->read(fp, buffer, 9, &fp->f_pos);

    for (i = 0; i < 64; i++, index++)
    {
        retval = fp->f_op->read(fp, temp, 2, &fp->f_pos);

        if (((temp[0] & 0xF0) >> 4) == 0x03)
            target_data_out[index] = (temp[0] & 0x0F) << 4;
        
        if (((temp[0] & 0xF0) >> 4) == 0x06 || ((temp[0] & 0xF0) >> 4) == 0x04)
            target_data_out[index] = ((temp[0] & 0x0F) + 9) << 4;
      
        if (((temp[1] & 0xF0) >> 4) == 0x03)
            target_data_out[index] = target_data_out[index] | (temp[1] & 0x0F);
        
        if (((temp[1] & 0xF0) >> 4) == 0x06 || ((temp[1] & 0xF0) >> 4) == 0x04)
            target_data_out[index] = target_data_out[index] | ((temp[1] & 0x0F) + 9);   
    }
    msleep(1);

    retval = fp->f_op->read(fp, buffer, 4, &fp->f_pos);

    retval = fp->f_op->read(fp, buffer, 9, &fp->f_pos);
    
    for (i = 0; i < 64; i++, index++)
    {
        retval = fp->f_op->read(fp, temp, 2, &fp->f_pos);

        if (((temp[0] & 0xF0) >> 4) == 0x03)
            target_data_out[index] = (temp[0] & 0x0F) << 4;
        
        if (((temp[0] & 0xF0) >> 4) == 0x06 || ((temp[0] & 0xF0) >> 4) == 0x04)
            target_data_out[index] = ((temp[0] & 0x0F) + 9) << 4;

        if (((temp[1] & 0xF0) >> 4) == 0x03)
            target_data_out[index] = target_data_out[index] | (temp[1] & 0x0F);
        
        if (((temp[1] & 0xF0) >> 4) == 0x06 || ((temp[1] & 0xF0) >> 4) == 0x04)
            target_data_out[index] = target_data_out[index] | ((temp[1] & 0x0F) + 9);   
    }
    msleep(1);

    retval = fp->f_op->read(fp, buffer, 4, &fp->f_pos);

    if (debug)
    {
        debug_printk("read data from file:\n");
        for (i = 0; i < TARGET_DATABUFF_LEN; i++)
        {
            debug_printk("%02X ", target_data_out[i]);
        }
        debug_printk("\n");
    }

    printk("#");

    return PASS;
}

// ============================================================================
// Description: 
// Run Clock without sending/receiving bits. Use this when transitioning from 
// write to read and read to write "num_cycles" is number of SCLK cycles, not
// number of counter cycles.
//
// SCLK cannot run faster than the specified maximum frequency of 8MHz. Some 
// processors may need to have delays added after setting SCLK low and setting
// SCLK high in order to not exceed this specification. The maximum frequency 
// of SCLK should be measured as part of validation of the final program
//
// ============================================================================
void tpd_run_clock(unsigned int num_cycles)
{
    int i;

    for (i = 0; i < num_cycles; i++) 
    {
//        mt_set_gpio_out(I2C_SCL, GPIO_OUT_ZERO);
//        mt_set_gpio_out(I2C_SCL, GPIO_OUT_ONE);
		gpio_write_one_pin_value(gpio_i2c_scl_hdle, I2C_LOW, "twi1_scl");
		udelay(CLK_DELAY);
		gpio_write_one_pin_value(gpio_i2c_scl_hdle, I2C_HIGHT, "twi1_scl");	
		udelay(CLK_DELAY);
    }
}

// ============================================================================
// Clocks the SCLK pin (high-low-high) and reads the status of the SDATA pin
// after the rising edge.
//
// SCLK cannot run faster than the specified maximum frequency of 8MHz. Some 
// processors may need to have delays added after setting SCLK low and setting
// SCLK high in order to not exceed this specification. The maximum frequency 
// of SCLK should be measured as part of validation of the final program
//
// Returns:
//     0 if SDATA was low
//     1 if SDATA was high
// ============================================================================
unsigned char tpd_receive_bit(void)
{
//    mt_set_gpio_out(I2C_SCL, GPIO_OUT_ZERO);
//    mt_set_gpio_out(I2C_SCL, GPIO_OUT_ONE);

	gpio_write_one_pin_value(gpio_i2c_scl_hdle, I2C_LOW, "twi1_scl");
	udelay(CLK_DELAY);
	gpio_write_one_pin_value(gpio_i2c_scl_hdle, I2C_HIGHT, "twi1_scl");
	udelay(CLK_DELAY);
//    return mt_get_gpio_in(I2C_SDA);
	return gpio_read_one_pin_value(gpio_i2c_sda_hdle, "twi1_sda");
}          

// ============================================================================
// Calls ReceiveBit 8 times to receive one byte.
// Returns:
//     The 8-bit values recieved.
// ============================================================================
unsigned char tpd_receive_byte(void)
{
    unsigned char b;
    unsigned char curr_byte = 0x00;

    for (b = 0; b < 8; b++) 
    {
        curr_byte = (curr_byte << 1) + tpd_receive_bit();
    }
    return curr_byte;
}          

// ============================================================================
// This routine sends up to one byte of a vector, one bit at a time.
//    bCurrByte   the byte that contains the bits to be sent.
//    bSize       the number of bits to be sent. Valid values are 1 to 8.
//
// SCLK cannot run faster than the specified maximum frequency of 8MHz. Some 
// processors may need to have delays added after setting SCLK low and setting
// SCLK high in order to not exceed this specification. The maximum frequency 
// of SCLK should be measured as part of validation of the final program
// ============================================================================
void tpd_send_byte(unsigned char curr_byte, unsigned char size)
{
    unsigned char b = 0;
//	printk("Jim_dream tpd_send_byte %d , size %d \n",curr_byte,size);

//	while(1);

    for (b = 0; b < size; b++) 
    {
        if (curr_byte & 0x80) 
        {
//            mt_set_gpio_out(I2C_SDA, GPIO_OUT_ONE); 
//            mt_set_gpio_out(I2C_SCL, GPIO_OUT_ONE);
//            mt_set_gpio_out(I2C_SCL, GPIO_OUT_ZERO);
//			printk("Jim_dream tpd_send_byte hight\n");

			gpio_write_one_pin_value(gpio_i2c_sda_hdle, I2C_HIGHT, "twi1_sda");
			gpio_write_one_pin_value(gpio_i2c_scl_hdle, I2C_HIGHT, "twi1_scl");
			udelay(CLK_DELAY);
			gpio_write_one_pin_value(gpio_i2c_scl_hdle, I2C_LOW, "twi1_scl");
			udelay(CLK_DELAY);
        }
        else 
        {
//            mt_set_gpio_out(I2C_SDA, GPIO_OUT_ZERO); 
//            mt_set_gpio_out(I2C_SCL, GPIO_OUT_ONE);
//            mt_set_gpio_out(I2C_SCL, GPIO_OUT_ZERO);
//			printk("Jim_dream tpd_send_byte low\n");
			gpio_write_one_pin_value(gpio_i2c_sda_hdle, I2C_LOW, "twi1_sda");
			gpio_write_one_pin_value(gpio_i2c_scl_hdle, I2C_HIGHT, "twi1_scl");
			udelay(CLK_DELAY);
			gpio_write_one_pin_value(gpio_i2c_scl_hdle, I2C_LOW, "twi1_scl");
			udelay(CLK_DELAY);
        }
        curr_byte = curr_byte << 1;
    }

}

// ============================================================================
// SendVector()
// This routine sends the vector specifed. All vectors constant strings found
// in ISSP_Vectors.h.  The data line is returned to HiZ after the vector is
// sent.
//    bVect      a pointer to the vector to be sent.
//    nNumBits   the number of bits to be sent.
//    bCurrByte  scratch var to keep the byte to be sent.
//
// There is no returned value.
// ============================================================================
void tpd_send_vector(const unsigned char* vect, unsigned int num_bits)
{
//    mt_set_gpio_dir(I2C_SDA, GPIO_DIR_OUT);
	gpio_i2c_sda_info.mul_sel = 1;
	gpio_set_one_pin_status(gpio_i2c_sda_hdle,&gpio_i2c_sda_info,"twi1_sda",1);

    while (num_bits > 0)
    {
        if (num_bits >= 8) 
        {
            tpd_send_byte(*(vect), 8);
            num_bits -= 8;
            vect++;
        }
        else 
        {
            tpd_send_byte(*(vect), num_bits);
            num_bits = 0;
        }
    }

	gpio_i2c_sda_info.mul_sel = 0;
	gpio_set_one_pin_status(gpio_i2c_sda_hdle,&gpio_i2c_sda_info,"twi1_sda",1);
	
//    mt_set_gpio_dir(I2C_SDA, GPIO_DIR_IN);
}


// ============================================================================
// Waits for transition from SDATA = 1 to SDATA = 0.  Has a 100 msec timeout.
// TRANSITION_TIMEOUT is a loop counter for a 100msec timeout when waiting for
// a high-to-low transition. This is used in the polling loop of 
// fDetectHiLoTransition(). The timing of the while(1) loops can be calculated
// and the number of loops is counted, using iTimer, to determine when 100 
// msec has passed.
//
// SCLK cannot run faster than the specified maximum frequency of 8MHz. Some 
// processors may need to have delays added after setting SCLK low and setting
// SCLK high in order to not exceed this specification. The maximum frequency 
// of SCLK should be measured as part of validation of the final program
//
// Returns:
//     0 if successful
//    -1 if timed out.
// ============================================================================
signed char tpd_detect_hi_lo_transition(void)
{
    unsigned int n;
    static unsigned int iTimer;

	gpio_write_one_pin_value(gpio_i2c_scl_hdle, I2C_LOW, "twi1_scl");
	udelay(CLK_DELAY);
	gpio_i2c_sda_info.mul_sel = 1;
	gpio_set_one_pin_status(gpio_i2c_sda_hdle,&gpio_i2c_sda_info,"twi1_sda",1);
	gpio_write_one_pin_value(gpio_i2c_sda_hdle, I2C_LOW, "twi1_sda");
	udelay(CLK_DELAY);
	gpio_write_one_pin_value(gpio_i2c_scl_hdle, I2C_HIGHT, "twi1_scl");
	udelay(CLK_DELAY);
	gpio_write_one_pin_value(gpio_i2c_scl_hdle, I2C_LOW, "twi1_scl");

	udelay(CLK_DELAY);
	gpio_i2c_sda_info.mul_sel = 0;
	gpio_set_one_pin_status(gpio_i2c_sda_hdle,&gpio_i2c_sda_info,"twi1_sda",1);
	
//    mt_set_gpio_out(I2C_SCL, GPIO_OUT_ZERO);
    
//    mt_set_gpio_dir(I2C_SDA, GPIO_DIR_OUT);
//    mt_set_gpio_out(I2C_SDA, GPIO_OUT_ZERO);
    
//    mt_set_gpio_out(I2C_SCL, GPIO_OUT_ONE);
//    mt_set_gpio_out(I2C_SCL, GPIO_OUT_ZERO);
    
//    mt_set_gpio_dir(I2C_SDA, GPIO_DIR_IN);

    msleep(50);
    
    return PASS;
}

int tpd_initialize(void)
{
    unsigned char n;
/*
    mt_set_gpio_dir(I2C_SDA, GPIO_DIR_IN);
    mt_set_gpio_dir(I2C_SCL, GPIO_DIR_OUT);
    mt_set_gpio_out(I2C_SCL, GPIO_OUT_ZERO);
        
    mt_set_gpio_dir(I2C_RST, GPIO_DIR_IN);
    msleep(500);
    mt_set_gpio_out(I2C_RST, GPIO_OUT_ONE);
    mt_set_gpio_dir(I2C_RST, GPIO_DIR_OUT);
    mt_set_gpio_out(I2C_RST, GPIO_OUT_ZERO);
    udelay((XRES_CLK_DELAY));
    mt_set_gpio_out(I2C_RST, GPIO_OUT_ONE);
    udelay(10);
*/

//	gpio_i2c_sda_info.mul_sel = 0;
//	gpio_set_one_pin_status(gpio_i2c_sda_hdle,&gpio_i2c_sda_info,"twi1_sda",1);

//		gpio_i2c_scl_info.mul_sel = 0;
//		gpio_set_one_pin_status(gpio_i2c_scl_hdle,&gpio_i2c_scl_info,"twi1_sda",1);

		gpio_write_one_pin_value(gpio_i2c_scl_hdle, I2C_LOW, "twi1_scl");

		gpio_i2c_scl_info.mul_sel = 1;
		gpio_set_one_pin_status(gpio_i2c_reset_hdle,&gpio_i2c_reset_info,"ctp_wakeup",1);
		
		gpio_write_one_pin_value(gpio_i2c_reset_hdle, I2C_HIGHT, "ctp_wakeup");
		msleep(500);
		gpio_write_one_pin_value(gpio_i2c_reset_hdle, I2C_LOW, "ctp_wakeup");
		mdelay(10);
		gpio_write_one_pin_value(gpio_i2c_reset_hdle, I2C_HIGHT, "ctp_wakeup");
	 	udelay(10);

	
    //  The timing spec that requires that the first Init-Vector happen within
    //  1 msec after the reset/power up. For this reason, it is not advisable
    //  to separate the above RESET_MODE or POWER_CYCLE_MODE code from the 
    //  Init-Vector instructions below. Doing so could introduce excess delay
    //  and cause the target device to exit ISSP Mode.
    
    tpd_send_vector(id_setup_1, num_bits_id_setup_1); 
    if (!tpd_detect_hi_lo_transition()) 
    {
        printk("send id_setup_1 fail\n");
        //return FAIL;
    }
    tpd_send_vector(wait_and_poll_end, num_bits_wait_and_poll_end);

    printk("#");
    
    return PASS;
}

int tpd_verify_id(void)
{
    tpd_send_vector(id_setup_2, num_bits_id_setup_2);
    if (!tpd_detect_hi_lo_transition()) 
    {
        printk("send id_setup_2 fail\n");
        //return FAIL;
    }
    tpd_send_vector(wait_and_poll_end, num_bits_wait_and_poll_end); 
    
    tpd_send_vector(tsync_enable, num_bits_tsync_enable);	

    //Send Read ID vector and get Target ID
    tpd_send_vector(read_id_v, 11);      // Read-MSB Vector is the first 11-Bits
    tpd_run_clock(2);                    // Two SCLK cycles between write & read
    target_id[0] = tpd_receive_byte();
    tpd_run_clock(1);
    tpd_send_vector(read_id_v + 2, 12);    // 1+11 bits starting from the 3rd byte

    tpd_run_clock(2);                    // Read-LSB Command
    target_id[1] = tpd_receive_byte();

    tpd_run_clock(1);
    tpd_send_vector(read_id_v + 4, 1);     // 1 bit starting from the 5th byte
    
    //read Revision ID from Accumulator A and Accumulator X
    tpd_send_vector(read_id_v + 5, 11);	//11 bits starting from the 6th byte
    tpd_run_clock(2);
    target_id[2] = tpd_receive_byte();	//Read from Acc.X
    tpd_run_clock(1);
    tpd_send_vector(read_id_v + 7, 12);    //1+11 bits starting from the 8th byte
    
    tpd_run_clock(2);
    target_id[3] = tpd_receive_byte();	//Read from Acc.A
    
    tpd_run_clock(1);
    tpd_send_vector(read_id_v + 4, 1);     //1 bit starting from the 5th byte,
    
    tpd_send_vector(tsync_disable, num_bits_tsync_disable);

    printk("target_id   = %02X %02X\n", target_id[0], target_id[1]);
    printk("target_id_v = %02X %02X\n", target_id_v[0], target_id_v[1]);

    printk("#");

    if (target_id[0] != target_id_v[0] || target_id[1] != target_id_v[1])
        return FAIL;
    else
        return PASS;
}

int tpd_read_status(void)
{
    tpd_send_vector(tsync_enable, num_bits_tsync_enable);

    //Send Read ID vector and get Target ID
    tpd_send_vector(read_id_v, 11);      // Read-MSB Vector is the first 11-Bits
    tpd_run_clock(2);                    // Two SCLK cycles between write & read
    target_status[0] = tpd_receive_byte();
    tpd_run_clock(1);
    tpd_send_vector(read_id_v+2, 12);    // 12 bits starting from the 3rd character

    tpd_run_clock(2);                    // Read-LSB Command
    target_status[1] = tpd_receive_byte();

    tpd_run_clock(1);
    tpd_send_vector(read_id_v+4, 1);     // 1 bit starting from the 5th character

    tpd_send_vector(tsync_disable, num_bits_tsync_disable);			

    if (target_status[0] == target_status00_v)
        return PASS;
    if (target_status[0] == target_status01_v)
        return FAIL;
    if (target_status[0] == target_status03_v)
        return FAIL;
    if (target_status[0] == target_status04_v)
        return FAIL;
    if (target_status[0] == target_status06_v)
        return FAIL;
    else
        return FAIL;
}

int tpd_erase_target(void)
{
    tpd_send_vector(erase, num_bits_erase);
    if (!tpd_detect_hi_lo_transition()) 
    {
        printk("send erase fail\n");
        //return FAIL;
    }
    tpd_send_vector(wait_and_poll_end, num_bits_wait_and_poll_end);

    printk("#");

    return PASS;
}

// ============================================================================
// Transfers data from array in Host to RAM buffer in the target.
// Returns the checksum of the data.
// ============================================================================
unsigned int tpd_load_target(void)
{
    unsigned char temp;
    unsigned int  checksum_data = 0;

    tpd_send_vector(tsync_enable, num_bits_tsync_enable);	

    tpd_send_vector(read_write_setup, num_bits_read_write_setup);

//    mt_set_gpio_dir(I2C_SDA, GPIO_DIR_OUT);
	gpio_i2c_sda_info.mul_sel = 1;
	gpio_set_one_pin_status(gpio_i2c_sda_hdle,&gpio_i2c_sda_info,"twi1_sda",1);

    // Transfer the temporary RAM array into the target.
    // In this section, a 128-Byte array was specified by #define, so the entire
    // 128-Bytes are written in this loop.
    target_address = 0x00;
    target_data_ptr = 0x00;
    
    while (target_data_ptr < TARGET_DATABUFF_LEN) 
    {   
        temp = target_data_out[target_data_ptr]; //PROGRAM_DATA;
        checksum_data += temp;

        tpd_send_byte(write_byte_start,4);    // we need to be able to write 128 bytes from address 0x80 to 0xFF  
        tpd_send_byte(target_address, 7);	 // we need to be able to write 128 bytes from address 0x80 to 0xFF 
        tpd_send_byte(temp, 8);
        tpd_send_byte(write_byte_end, 3);
        
        // SendByte() uses MSbits, so inc by '2' to put the 0..128 address into
        // the seven MSBit locations.
        //
        // This can be confusing, but check the logic:
        //   The address is only 7-Bits long. The SendByte() subroutine will
        // send however-many bits, BUT...always reads them bits from left-to-
        // right. So in order to pass a value of 0..128 as the address using
        // SendByte(), we have to left justify the address by 1-Bit.
        //   This can be done easily by incrementing the address each time by
        // '2' rather than by '1'.

        target_address += 2;			// inc by 2 in order to support a 128 byte address space
        target_data_ptr++; 
    }
    msleep(1);
    
    return checksum_data;
}

// ============================================================================
// Program one block with data that has been loaded into a RAM buffer in the 
// target device.
// ============================================================================
int tpd_program_target_block(unsigned char bBankNumber, unsigned char bBlockNumber)
{
    // TSYNC should still be set when entering this function so this call is not necessary but added for insurance
    tpd_send_vector(tsync_enable, num_bits_tsync_enable);	

    tpd_send_vector(set_block_num, num_bits_set_block_num);
	
    // Set the drive here because SendByte() does not.
//    mt_set_gpio_dir(I2C_SDA, GPIO_DIR_OUT);
	gpio_i2c_sda_info.mul_sel = 1;
	gpio_set_one_pin_status(gpio_i2c_sda_hdle,&gpio_i2c_sda_info,"twi1_sda",1);


    tpd_send_byte(bBlockNumber,8);
    tpd_send_byte(set_block_num_end, 3);
    
    tpd_send_vector(tsync_disable, num_bits_tsync_disable);	

    tpd_send_vector(program_and_verify, num_bits_program_and_verify);		

    if (!tpd_detect_hi_lo_transition()) 
    {
        debug_printk("send program_and_verify fail\n");
        //return FAIL;
    }
    // Send the Wait-For-Poll-End vector
    tpd_send_vector(wait_and_poll_end, num_bits_wait_and_poll_end);
    return PASS;
}

// ============================================================================
// Reads and adds the target bank checksum to the referenced accumulator.
// ============================================================================
int tpd_target_bank_checksum(unsigned int* acc)
{
    unsigned char msb = 0;
    unsigned char lsb = 0;
	
    unsigned int n;
	
    tpd_send_vector(checksum_setup, num_bits_checksum_setup);
    if (!tpd_detect_hi_lo_transition()) 
    {					
        debug_printk("send checksum_setup fail\n");
        //return FAIL;		
    }

	msleep(250);

    tpd_send_vector(wait_and_poll_end, num_bits_wait_and_poll_end); 
   
    tpd_send_vector(tsync_enable, num_bits_tsync_enable);			

    //Send Read Checksum vector and get Target Checksum
    tpd_send_vector(read_checksum_v, 11);     // first 11-bits is ReadCKSum-MSB		
    tpd_run_clock(2);                         // Two SCLKs between write & read
    msb = tpd_receive_byte();		 
    tpd_run_clock(1);                         // See Fig. 6
    tpd_send_vector(read_checksum_v + 2, 12); // 12 bits starting from 3rd character	
    tpd_run_clock(2);                         // Read-LSB Command
    lsb = tpd_receive_byte();		 

    tpd_run_clock(1);
    tpd_send_vector(read_checksum_v + 3, 1);  // Send the final bit of the command	
    
    tpd_send_vector(tsync_disable, num_bits_tsync_disable);			
    
    *acc = (msb << 8) | lsb;  // combine the MSB and the LSB
	
    return PASS;    
}    


// ============================================================================
// After programming, the target PSoC must be reset to take it out of 
// programming mode. This routine performs a reset.
// ============================================================================
void tpd_restart_target(void)
{
	gpio_write_one_pin_value(gpio_i2c_reset_hdle, I2C_LOW, "ctp_wakeup");

//    mt_set_gpio_out(I2C_RST, GPIO_OUT_ZERO);
    udelay(XRES_CLK_DELAY);
//    mt_set_gpio_out(I2C_RST, GPIO_OUT_ONE);
	gpio_write_one_pin_value(gpio_i2c_reset_hdle, I2C_HIGHT, "ctp_wakeup");

}

// ============================================================================
// Verify the block just written to. This can be done byte-by-byte before the
// protection bits are set.
// ============================================================================
int tpd_verify_setup(unsigned char bank_number, unsigned char block_number)
{
    tpd_send_vector(tsync_enable, num_bits_tsync_enable);	

    tpd_send_vector(read_write_setup, num_bits_read_write_setup);
	
    tpd_send_vector(set_block_num, num_bits_set_block_num);					
	
    //Set the drive here because SendByte() does not
//    mt_set_gpio_dir(I2C_SDA, GPIO_DIR_OUT);
	gpio_i2c_sda_info.mul_sel = 1;
	gpio_set_one_pin_status(gpio_i2c_sda_hdle,&gpio_i2c_sda_info,"twi1_sda",1);

    tpd_send_byte(block_number,8);
    tpd_send_byte(set_block_num_end, 3);					
    
    tpd_send_vector(tsync_disable, num_bits_tsync_disable);	
    
    tpd_send_vector(verify_setup, num_bits_my_verify_setup);	

	if (!tpd_detect_hi_lo_transition()) 
	{
	    debug_printk("send verify_setup fail\n");
        //return FAIL;		
    }
    tpd_send_vector(wait_and_poll_end, num_bits_wait_and_poll_end);     

    return PASS;
}

// ============================================================================
// Reads the data back from Target SRAM and compares it to expected data in
// Host SRAM
// ============================================================================
int tpd_read_byte_loop(void)
{
    target_address = 0;
    target_data_ptr = 0;
	
    tpd_send_vector(tsync_enable, num_bits_tsync_enable);	

    tpd_send_vector(read_write_setup, num_bits_read_write_setup);

    debug_printk("tpd_read_byte_loop:\n");
    while(target_data_ptr < TARGET_DATABUFF_LEN) 
    {
        //Send Read Byte vector and then get a byte from Target
        tpd_send_vector(read_byte_v, 4);					

		gpio_i2c_sda_info.mul_sel = 1;
		gpio_set_one_pin_status(gpio_i2c_sda_hdle,&gpio_i2c_sda_info,"twi1_sda",1);
        
//        mt_set_gpio_dir(I2C_SDA, GPIO_DIR_OUT);
        tpd_send_byte(target_address,7);

        tpd_run_clock(2);       // Run two SCLK cycles between writing and reading
//        mt_set_gpio_dir(I2C_SDA, GPIO_DIR_IN);     // Set to HiZ so Target can drive SDATA
		gpio_i2c_sda_info.mul_sel = 0;
		gpio_set_one_pin_status(gpio_i2c_sda_hdle,&gpio_i2c_sda_info,"twi1_sda",1);

        target_data_in = tpd_receive_byte();
        debug_printk("%02x ", target_data_in);

        tpd_run_clock(1);
        tpd_send_vector(read_byte_v + 1, 1);     // Send the ReadByte Vector End
		
        // Test the Byte that was read from the Target against the original
        // value (already in the 128-Byte array "abTargetDataOUT[]"). If it
        // matches, then bump the address & pointer,loop-back and continue.
        // If it does NOT match abort the loop and return and error.
        if (target_data_in != target_data_out[target_data_ptr])
        {
            debug_printk("target_data_in      = %02x\n", target_data_in);
            debug_printk("target_data_out[%d] = %02x\n", target_data_ptr, target_data_out[target_data_ptr]);
            return FAIL;
        }
        
        target_data_ptr++;
        // Increment the address by 2 to accomodate 7-Bit addressing
        // (puts the 7-bit address into MSBit locations for "SendByte()").
        target_address += 2;

        

    }
    msleep(1);
    
    debug_printk("\n");
    tpd_send_vector(tsync_disable, num_bits_tsync_disable);	

    return PASS;
}

// ============================================================================
// Before calling, load the array, abTargetDataOUT, with the desired security
// settings using LoadArrayWithSecurityData(StartAddress,Length,SecurityType).
// The can be called multiple times with different SecurityTypes as needed for
// particular Flash Blocks. Or set them all the same using the call below:
// LoadArrayWithSecurityData(0,SECURITY_BYTES_PER_BANK, 0); 
// ============================================================================
int tpd_secure_target_flash(void)
{
    unsigned char temp;

    // Transfer the temporary RAM array into the target
    target_address = 0x00;
    target_data_ptr = 0x00;
	
    tpd_send_vector(tsync_enable, num_bits_tsync_enable);	

    tpd_send_vector(read_write_setup, num_bits_read_write_setup);
		
//    mt_set_gpio_dir(I2C_SDA, GPIO_DIR_OUT);
	gpio_i2c_sda_info.mul_sel = 1;
	gpio_set_one_pin_status(gpio_i2c_sda_hdle,&gpio_i2c_sda_info,"twi1_sda",1);

    while(target_data_ptr < SECURITY_BYTES_PER_BANK) 
    {     
        temp = target_data_out[target_data_ptr];
        tpd_send_byte(write_byte_start,4);    	
        tpd_send_byte(target_address, 7);
        tpd_send_byte(temp, 8);
        tpd_send_byte(write_byte_end, 3);

        // SendBytes() uses MSBits, so increment the address by '2' to put
        // the 0..n address into the seven MSBit locations
        target_address += 2;				// inc by 2 in order to support a 128 byte address space
        target_data_ptr++; 
    }
	msleep(1);
	
    tpd_send_vector(tsync_disable, num_bits_tsync_disable);	
 
    tpd_send_vector(secure, num_bits_secure);	
    if (!tpd_detect_hi_lo_transition()) 
    {
        debug_printk("send secure fail\n");
        //return FAIL;	
    }
    tpd_send_vector(wait_and_poll_end, num_bits_wait_and_poll_end);
    return PASS;
}

// ============================================================================
// This step is optional. Verifies that the security bits have been written correctly
// ============================================================================
int tpd_verify_security(void)
{
    target_address = 0x00;

    tpd_send_vector(verify_security, num_bits_verify_security);

    if (!tpd_detect_hi_lo_transition()) 
    {
        printk("send verify_security fail\n");
        //return FAIL;	
    }

    tpd_send_vector(wait_and_poll_end, num_bits_wait_and_poll_end);
    
    target_address = 0x00;
    target_data_ptr = 0x00;

    tpd_send_vector(tsync_enable, num_bits_tsync_enable);	

    tpd_send_vector(read_write_setup, num_bits_read_write_setup);
	
    //fReadWriteSetup();
    
    while(target_address <(SECURITY_BYTES_PER_BANK * 2)) 
    {			
        // we do SECURITY_BYTES_PER_BANK * 2 because we bTargetAddress += 2
        
        //Send Read Byte vector and then get a byte from Target
        tpd_send_vector(read_byte_v, 4);									
        // Set the drive here because SendByte() does not
//        mt_set_gpio_dir(I2C_SDA, GPIO_DIR_OUT);
		gpio_i2c_sda_info.mul_sel = 1;
		gpio_set_one_pin_status(gpio_i2c_sda_hdle,&gpio_i2c_sda_info,"twi1_sda",1);

        tpd_send_byte(target_address,7);

//        mt_set_gpio_dir(I2C_SDA, GPIO_DIR_IN);
		gpio_i2c_sda_info.mul_sel = 0;
		gpio_set_one_pin_status(gpio_i2c_sda_hdle,&gpio_i2c_sda_info,"twi1_sda",1);

        tpd_run_clock(2);       // Run two SCLK cycles between writing and reading
        target_data_in = tpd_receive_byte();

        tpd_run_clock(1);
        tpd_send_vector(read_byte_v + 1, 1);     // Send the ReadByte Vector End
        
        // Test the Byte that was read from the Target against the original
        // value (already in the 128-Byte array "abTargetDataOUT[]"). If it
        // matches, then bump the address & pointer,loop-back and continue.
        // If it does NOT match abort the loop and return and error.
		
        if (target_data_in != target_data_out[target_data_ptr])
            return FAIL;
        
        // Increment the address by two to accomodate 7-Bit addressing
        // (puts the 7-bit address into MSBit locations for "SendByte()").
        
        target_data_ptr++;
        target_address += 2;
    }
    msleep(1);

    tpd_send_vector(tsync_disable, num_bits_tsync_disable);	

    return PASS;
}
