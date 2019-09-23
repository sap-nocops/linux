#include "disp_eink.h"
#include <linux/cpufreq.h>
//private
#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/init.h>
#include <linux/skbuff.h>
#include <linux/spinlock.h>
#include <linux/crc32.h>
#include <linux/mii.h>
#include <linux/ethtool.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/clk.h>
#include <linux/ctype.h>
#include <linux/pm.h>
#include <asm/cacheflush.h>
#include <asm/delay.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <mach/dma.h>
#include <mach/sys_config.h>
#include <mach/clock.h>
#include <mach/platform.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/syscalls.h>
#include <linux/kmod.h>
#include <linux/types.h>

extern int tps65185_vcom_set(int vcom_mv);
extern int tps65185_v3p3_set(void);
//end
Disp_eink_t disp_eink;

#define MIN(x, y) (((x) < (y)) ? (x) : (y))
#define MAX(x, y) (((x) > (y)) ? (x) : (y))


__u32 EINK_LCD_W = 0, EINK_LCD_H = 0;
__u32 EINK_PANEL_W = 0, EINK_PANEL_H = 0;
__u32 EINK_LDL = 0, EINK_FDL = 0;
__u32 EINK_WF_WIDTH = 0, EINK_WF_HEIGHT = 0;



__u8 rotate_buffer[776192]; //1024*758

const __u8 dithering_color[256] = {
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
	255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
	255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
	255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
	255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
	255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
	255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
	255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255
};

static void eink_init_fb_para(__disp_fb_create_para_t *fb_create_para, 
                               __u32 width, 
                               __u32 height,
                               __u32 buffer_num)
{
    memset(fb_create_para, 0, sizeof(__disp_fb_create_para_t));
    
    fb_create_para->fb_mode         = FB_MODE_SCREEN0;
    fb_create_para->mode            = DISP_LAYER_WORK_MODE_NORMAL;
    fb_create_para->buffer_num      = buffer_num;
    fb_create_para->width           = width;
    fb_create_para->height          = height;
    fb_create_para->output_width    = width;
    fb_create_para->output_height   = height;
}

static void eink_set_input_argb32b(__disp_fb_t *dst, __disp_rectsz_t *input_size, __u32 argb_addr)
{
    dst->addr[0] = argb_addr;
    dst->addr[1] = 0;
    dst->addr[2] = 0;
    
    dst->size.width  = input_size->width;
    dst->size.height = input_size->height;
    
    dst->br_swap = FALSE;
    dst->cs_mode = DISP_YCC;
	dst->format  = DISP_FORMAT_ARGB8888;
    dst->mode    = DISP_MOD_INTERLEAVED;
    dst->seq     = DISP_SEQ_ARGB;
}

static void eink_set_output_YuvPlanar(__disp_fb_t *dst, __disp_rectsz_t *output_size, __u32 Y_addr, __u32 U_addr, __u32 V_addr)
{
    dst->addr[0] = Y_addr;
    dst->addr[1] = U_addr;
    dst->addr[2] = V_addr;
    
    dst->size.width  = output_size->width;
    dst->size.height = output_size->height;
    
    dst->br_swap = FALSE;
	dst->cs_mode = DISP_YCC;
	dst->format  = DISP_FORMAT_YUV444;
    dst->mode    = DISP_MOD_NON_MB_PLANAR;
    dst->seq     = DISP_SEQ_P3210;            
}

static __s32 eink_scale_image(__disp_fb_t *src, __disp_rect_t *src_rect, __disp_fb_t *dst)
{
    __s32 scale_handle;
    __disp_scaler_para_t param[1];
    __s32 ret;

    memset(param, 0, sizeof(__disp_scaler_para_t));
    
    scale_handle = BSP_disp_scaler_request();
    if (scale_handle < 0)
    {
        __EINK_INF("request scaler fail!!, error number is %d \n", scale_handle);
        
        return DIS_NO_RES;
    }
    
    param->input_fb    = *src;
    param->output_fb   = *dst;
    param->source_regn = *src_rect;
    
    ret = BSP_disp_scaler_start(scale_handle, param);
    if (ret == DIS_FAIL)
    {
        __EINK_INF("scale image err!\n");
    }
    else
    {
        ret = DIS_SUCCESS;
    }
    
    BSP_disp_scaler_release(scale_handle);
    
    return ret;
}

static __u32* eink_load_waveform(char * path)
{
	ES_FILE *fp = NULL;
	__u32 *wav_buf = NULL;
	__s32 file_len = 0, read_len = 0;
	mm_segment_t fs;
	loff_t pos;
	__u32 wf_data_len = 0;           //waveform数据总长度

	fp = filp_open(path, O_RDONLY, 0);
	if (IS_ERR(fp)) {
		__EINK_WRN("open gc16 waveform failed!");
		return NULL;
	} else {
		__EINK_WRN("open gc16 waveform successfully!\n");
	}

	fs = get_fs();
	set_fs(KERNEL_DS);
	pos = 0;

	file_len = fp->f_dentry->d_inode->i_size;

	wf_data_len = file_len+1023;
	wav_buf = vmalloc(wf_data_len);
	__EINK_MSG("eink_load_waveform: wav_buf = %x\n", wav_buf);

	if (!wav_buf) {
		__EINK_WRN("palloc failed for eink gc16 waveform...\n");
		goto error;
	}

	read_len = vfs_read(fp, (char *)wav_buf, file_len, &pos);
	if (read_len != file_len) {
		__EINK_WRN("fread gc16 waveform fail...\n");
		goto error;
	}
	if (fp) {
		filp_close(fp, NULL);
		set_fs(fs);
	}

	return wav_buf;

error:
	if (wav_buf != NULL)
		vfree(wav_buf);

	if (fp) {
		filp_close(fp, NULL);
		set_fs(fs);
	}

	__EINK_MSG("error in load gc16 waveform\n");
	return NULL;
}

#ifndef AWF_WAVEFORM_SUPPORTED
void eink_loadWaveFormData(void) 
{
	if (disp_eink.wav_du_data == NULL)
		disp_eink.wav_du_data = eink_load_waveform(EINK_WAV_DU_PATH);

	if (disp_eink.wav_short_du_data == NULL)
		disp_eink.wav_short_du_data = eink_load_waveform(EINK_WAV_SHORT_DU_PATH);

	if (disp_eink.wav_short_gc16_local_data == NULL)
		disp_eink.wav_short_gc16_local_data = eink_load_waveform(EINK_WAV_SHORT_GC16_LOCAL_PATH);

	if (disp_eink.wav_gc16_data == NULL)
		disp_eink.wav_gc16_data = eink_load_waveform(EINK_WAV_GC16_PATH);

	if (disp_eink.wav_common_data == NULL)
		disp_eink.wav_common_data = eink_load_waveform(EINK_WAV_COMMON_PATH); 

	if (disp_eink.wav_a2_data == NULL)
		disp_eink.wav_a2_data = eink_load_waveform(EINK_WAV_A2_PATH); 

	if (disp_eink.wav_a2_in_data == NULL)
		disp_eink.wav_a2_in_data = eink_load_waveform(EINK_WAV_A2_IN_PATH); 

	if (disp_eink.wav_a2_out_data == NULL)
		disp_eink.wav_a2_out_data = eink_load_waveform(EINK_WAV_A2_OUT_PATH); 
}

static __s32 eink_init_update_waveform(void)
{
    __u32 frame_id = 0, temperature = 26, temp_index = 0;
    __u32* point = NULL;
    const __u8* p_wf_data = NULL;
    __u32 row, col = 0, row_temp = 0, ctrl_data = 0, data = 0, temp = 0;
    __u8 index = 0;
    __u32 pixel1 = 0, pixel2 = 0, pixel3 = 0, pixel4 = 0;
    __s32 wav_compose_index;

	__s32 buf_index;

	buf_index = disp_eink.bufQueue.curBufIndex;

	wav_compose_index = disp_eink.bufQueue.slots[buf_index].wav_compose_index;
/*    __EINK_LOG("eink_init_update_waveform: wav_compose_index = %d\n", wav_compose_index);*/
    if (wav_compose_index == 0)
    {
	    temperature = disp_eink.temperature;
	    if(temperature < 0)
	    {
	        temperature = 0;
	    }
	    if(temperature >= T_MAX)
	    {
	        temperature = T_MAX - 1;
	    }
        
        if (disp_eink.eink_moudule_type == 0)
        {
            disp_eink.eink_init_T_tbl   = eink_init_T_tbl_OED;
            disp_eink.eink_init_mode_wf = eink_init_mode_wf_OED;
        }
        else if (disp_eink.eink_moudule_type == 1)
        {
            disp_eink.eink_init_T_tbl   = eink_init_T_tbl_PVI;
            disp_eink.eink_init_mode_wf = eink_init_mode_wf_PVI;
        }
        
        if (disp_eink.eink_ctrl_data_type == 0)
        {
            disp_eink.eink_ctrl_tbl_GC16 = eink_ctrl_tbl_GC16_COMMON;
        }
        else if (disp_eink.eink_ctrl_data_type == 1)
        {
       //     disp_eink.eink_ctrl_tbl_GC16 = eink_ctrl_tbl_GC16_TIANZHI;
        }
        
	    index = disp_eink.eink_init_T_tbl[temperature];
	    disp_eink.bufQueue.slots[buf_index].total_frame = disp_eink.eink_init_mode_wf[index].total_frame;
	    disp_eink.pframe_data = (__s32*)disp_eink.eink_init_mode_wf[index].wf_data;
	    
	    __EINK_LOG("eink_init_update_waveform: total_frame->[%d]\n", disp_eink.bufQueue.slots[buf_index].total_frame);
    }
	
	
    if (wav_compose_index < disp_eink.bufQueue.slots[buf_index].total_frame)
    {   
	    frame_id = wav_compose_index;
		p_wf_data = (__u8*)disp_eink.pframe_data;
		temp_index = wav_compose_index % EINK_WAV_BUFFER_NUM;

    	if(disp_eink.wav_buffer[temp_index].wav_index == -1)
    	{
	        point = disp_eink.wav_buffer[temp_index].wav_address;
	
	        for(row = 0; row < disp_eink.wav_height; row++)
	        {
	            row_temp = eink_ctrl_line_index_tbl[row];
	            for(col = 0; col < disp_eink.wav_width; col++)
	            {                            	                
	                if((row > (EINK_FSL+EINK_FBL-1))&&(row < (EINK_LCD_H - EINK_FEL)))
	                {
	                    if((col > (EINK_LSL+EINK_LBL-1))&&(col < (EINK_LCD_W - EINK_LEL)))
	                    {                       
	                        pixel1 =  *(p_wf_data + frame_id);
	                        pixel2 =  pixel1;
	                        pixel3 =  pixel1;
	                        pixel4 =  pixel1;
	                        
	                        data = pixel1<<11;           //D12, D11
	                        
	                        data |= (pixel2&0x02)<<9;     //D10
	                        data |= (pixel2&0x01)<<7;     //D7
	                        
	                        data |= pixel3<<5;           //D6,D5
	                        data |= pixel4<<3;           //D4,D3                           
	                        
	                        ctrl_data = disp_eink.eink_ctrl_tbl_GC16[row_temp][col];                      
	                        temp = ctrl_data | data; 
	                        *point = temp;
	                        point++;
	                        continue;
	                    }
	                }            
	                ctrl_data = disp_eink.eink_ctrl_tbl_GC16[row_temp][col];                    
	                temp = ctrl_data | data; 
	
	                *point = temp;
	                point++;
	            }
	        }
			disp_eink.wav_buffer[temp_index].wav_index = wav_compose_index;
			disp_eink.bufQueue.slots[buf_index].wav_compose_index++;
			
	    }
	    else
	    {
	    	return 0;
	    }
    }
    else
    {
    	return -1;
    }

    return 0;
}
#endif

static __s32 eink_gc16_update_waveform(__u32* p_wf_file)
{
    __u32 frame_id = 0, temperature = 26, temp_index = 0;
    register __u32* point = NULL;
    register __u8* p_wf_index = NULL;
              
    register const __u8* p_wf_data = NULL;
    register __u32 row, col = 0, row_temp = 0, data = 0;

    register __u32 pixel1 = 0, pixel2 = 0, pixel3 = 0, pixel4 = 0;
    
    register __u32* p_ctrl_line = NULL; 
    register const __u32* p_ctrl_gc_tbl = NULL; 

    __u32 *input_wf_file = p_wf_file;

	__s32 buf_index, wav_compose_index;

	buf_index = disp_eink.bufQueue.curBufIndex;

	wav_compose_index = disp_eink.bufQueue.slots[buf_index].wav_compose_index;

    if (wav_compose_index == 0)
    {
	    temperature = disp_eink.temperature;
	    if(temperature < 0)
	    {
	        temperature = 0;
	    }
	    if(temperature >= T_MAX)
	    {
	        temperature = T_MAX - 1;
	    }
        
        if (disp_eink.eink_ctrl_data_type == 0)
        {
            disp_eink.eink_ctrl_tbl_GC16 = eink_ctrl_tbl_GC16_COMMON;
        }
        else if (disp_eink.eink_ctrl_data_type == 1)
        {
         //   disp_eink.eink_ctrl_tbl_GC16 = eink_ctrl_tbl_GC16_TIANZHI;
        }
        
        disp_eink.wf_len_divider = *input_wf_file;
        input_wf_file++;
            
	    __u32* ptemp = input_wf_file + temperature;		
		__u16* pFrame = (__u16*)((__u32)input_wf_file + *ptemp);
    	disp_eink.bufQueue.slots[buf_index].total_frame = *pFrame; 
        
    	pFrame++;
    	disp_eink.pframe_data = (__s32*)pFrame;                  
	    __EINK_LOG("eink_gc16_update_waveform: total_frame = [%d]\n", disp_eink.bufQueue.slots[buf_index].total_frame);
    }

/*     __EINK_LOG("wav_compose_index = %d, total_frame = %d\n", wav_compose_index, disp_eink.bufQueue.slots[buf_index].total_frame);*/
    if (wav_compose_index < disp_eink.bufQueue.slots[buf_index].total_frame)
    {   
	    frame_id = disp_eink.bufQueue.slots[buf_index].wav_compose_index;
		p_wf_data = (__u8*)disp_eink.pframe_data;
    	temp_index = disp_eink.bufQueue.slots[buf_index].wav_compose_index % EINK_WAV_BUFFER_NUM;
 


        point = disp_eink.wav_buffer[temp_index].wav_address;       
        p_wf_index = disp_eink.wav_form_index;
        
        p_ctrl_gc_tbl = (__u32*)disp_eink.eink_ctrl_tbl_GC16[4];
        p_ctrl_line = (__u32*)eink_ctrl_line_index_tbl;

        //line1--line8
        for(row = 0; row < (EINK_FSL+EINK_FBL); row++)
        {
            row_temp = *p_ctrl_line++;
            for(col = 0; col < EINK_WF_WIDTH; col++)
            {   
                *point = disp_eink.eink_ctrl_tbl_GC16[row_temp][col]; 
                point++;                
            }                 
        }

        for(; row < (EINK_LCD_H - EINK_FEL); row++)
        {
            row_temp = *p_ctrl_line++; 
            for(col = 0; col<(EINK_LSL+EINK_LBL); col++)
            {
                *point = (*(p_ctrl_gc_tbl + col));
                point++;  
            }

            for(; col<(EINK_LCD_W - EINK_LEL); col++)
            {	
				pixel1 = *(p_wf_data + ((*p_wf_index<< disp_eink.wf_len_divider) + frame_id));
				p_wf_index++;
				pixel2 = *(p_wf_data + ((*p_wf_index<< disp_eink.wf_len_divider) + frame_id));
				p_wf_index++;
				pixel3 = *(p_wf_data + ((*p_wf_index<< disp_eink.wf_len_divider) + frame_id));
				p_wf_index++;
				pixel4 = *(p_wf_data + ((*p_wf_index<< disp_eink.wf_len_divider) + frame_id));
				p_wf_index++;  
                
				data = pixel1<<11;               //D12, D11
				
				data |= (pixel2&0x02)<<9;        //D10
				data |= (pixel2&0x01)<<7;        //D7
				
				data |= pixel3<<5;               //D6,D5
				data |= pixel4<<3;               //D4,D3                                               
				
				*point = (*(p_ctrl_gc_tbl + col) | data);
				point++;         
            }

            for(; col<EINK_WF_WIDTH; col++)
            {
                *point = (*(p_ctrl_gc_tbl + col));
                point++;  
            }
        } 

        for(; row < EINK_WF_HEIGHT; row++)
        {
            row_temp = *p_ctrl_line++;
            for(col = 0; col < EINK_WF_WIDTH; col++)
            {   
                *point = disp_eink.eink_ctrl_tbl_GC16[row_temp][col]; 
                point++;                
            }
        }
		disp_eink.wav_buffer[temp_index].wav_index = wav_compose_index;
		disp_eink.bufQueue.slots[buf_index].wav_compose_index++;
    }
    else
    {
    	return -1;
    }

    return 0;
}

#ifdef AWF_WAVEFORM_SUPPORTED
static __s32 eink_load_awf_waveform(char * path)
{
	ES_FILE *fp = NULL;
	__s32 file_len = 0, read_len = 0;
	mm_segment_t fs;
	loff_t pos;
	__u32 wf_buffer_len = 0;           //waveform数据总长度
	__u32* pAddr = NULL;

	__EINK_DBG("starting to load awf waveform file from [%s]\n", path);

	fp = filp_open(path, O_RDONLY, 0);
	if (IS_ERR(fp)) {
		__EINK_WRN("open gc16 waveform failed!");
		return -1;
	} else {
		__EINK_WRN("open gc16 waveform successfully!\n");
	}

	fs = get_fs();
	set_fs(KERNEL_DS);
	pos = 0;

	file_len = fp->f_dentry->d_inode->i_size;

	wf_buffer_len = file_len+1023;
	disp_eink.p_wf_buffer = vmalloc(wf_buffer_len);
	__EINK_DBG("eink_load_awf_waveform: p_wf_buffer = %x\n", (__u32)disp_eink.p_wf_buffer);

	if (!disp_eink.p_wf_buffer) {
		__EINK_WRN("palloc failed for eink awf waveform...\n");
		goto error;
	}

	read_len = vfs_read(fp, (char *)disp_eink.p_wf_buffer, file_len, &pos);
	if (read_len != file_len) {
		__EINK_WRN("fread awf waveform fail...\n");
		goto error;
	}

	//starting to load data
	__EINK_DBG("starting to load wf temperature data from awf.\n");
	memcpy(disp_eink.wf_temp_area_tbl,(disp_eink.p_wf_buffer+C_TEMP_TBL_OFFSET),C_TEMP_TBL_SIZE);

	pAddr = (__u32*)(disp_eink.p_wf_buffer + C_INIT_MODE_ADDR_OFFSET);
	__EINK_DBG("load from awf INIT MODE addr = %x\n", *pAddr);
	disp_eink.p_init_wf = (__u32*)(disp_eink.p_wf_buffer + *pAddr);

	pAddr = (__u32*)(disp_eink.p_wf_buffer + C_GC16_MODE_ADDR_OFFSET);
	__EINK_DBG("load from awf GC16 MODE addr = %x\n", *pAddr);
	disp_eink.p_gc16_wf = (__u32*)(disp_eink.p_wf_buffer + *pAddr);

	pAddr = (__u32*)(disp_eink.p_wf_buffer + C_GC4_MODE_ADDR_OFFSET);
	__EINK_DBG("load from awf GC4 MODE addr = %x\n", *pAddr);
	disp_eink.p_gc4_wf = (__u32*)(disp_eink.p_wf_buffer + *pAddr);

	pAddr = (__u32*)(disp_eink.p_wf_buffer + C_DU_MODE_ADDR_OFFSET);
	__EINK_DBG("load from awf DU MODE addr = %x\n", *pAddr);
	disp_eink.p_du_wf = (__u32*)(disp_eink.p_wf_buffer + *pAddr);

	pAddr = (__u32*)(disp_eink.p_wf_buffer + C_A2_MODE_ADDR_OFFSET);
	__EINK_DBG("load from awf A2 MODE addr = %x\n", *pAddr);
	disp_eink.p_A2_wf = (__u32*)(disp_eink.p_wf_buffer + *pAddr);

	pAddr = (__u32*)(disp_eink.p_wf_buffer + C_A2_IN_MODE_ADDR_OFFSET);
	__EINK_DBG("load from awf A2 in MODE addr = %x\n", *pAddr);
	disp_eink.p_A2_in_wf = (__u32*)(disp_eink.p_wf_buffer + *pAddr);

	pAddr = (__u32*)(disp_eink.p_wf_buffer + C_A2_OUT_MODE_ADDR_OFFSET);
	__EINK_DBG("load from awf A2 out MODE addr = %x\n", *pAddr);
	disp_eink.p_A2_out_wf = (__u32*)(disp_eink.p_wf_buffer + *pAddr);

	pAddr = (__u32*)(disp_eink.p_wf_buffer + C_GC16_LOCAL_MODE_ADDR_OFFSET);
	__EINK_DBG("load from awf GC16 local MODE addr = %x\n", *pAddr);
	disp_eink.p_gc16_local_wf = (__u32*)(disp_eink.p_wf_buffer + *pAddr);

	pAddr = (__u32*)(disp_eink.p_wf_buffer + C_GC4_LOCAL_MODE_ADDR_OFFSET);
	__EINK_DBG("load from awf GC4 local MODE addr = %x\n", *pAddr);
	disp_eink.p_gc4_local_wf = (__u32*)(disp_eink.p_wf_buffer + *pAddr);

	__EINK_DBG("load from awf load awf data finished.\n");

	if (fp) {
	    filp_close(fp, NULL);
	    set_fs(fs);
	}

	// load the short du and short gc16 bin files
	if (disp_eink.wav_short_du_data == NULL)
		disp_eink.wav_short_du_data = eink_load_waveform(EINK_WAV_SHORT_DU_PATH);
	if (disp_eink.wav_short_gc16_local_data == NULL)
		disp_eink.wav_short_gc16_local_data = eink_load_waveform(EINK_WAV_SHORT_GC16_LOCAL_PATH);

	return 0;

error:
	if (disp_eink.p_wf_buffer!= NULL)
		vfree(disp_eink.p_wf_buffer);

	if (fp) {
		filp_close(fp, NULL);
		set_fs(fs);
	}

	__EINK_MSG("error in load awf waveform\n");
	return -1;
}

__s32 check_temp_area_index(int temperature)
{
    int index = 0;
    if(temperature >= T_MAX)
    {
        temperature = T_MAX - 1;
    }
    for(index = 0; index < C_TEMP_TBL_SIZE; index++)
    {
        if((disp_eink.wf_temp_area_tbl[index] == 0)&&(index>0))
        {
            break;
        }
        if(temperature < disp_eink.wf_temp_area_tbl[index])
        {
            break;
        }
    }
    if(index > 0)
    {
        index -= 1;
    }
    return index;
}

static __s32 eink_init_update_waveform_awf(void)
{
    __u32 frame_id = 0, temperature = 26, temp_index = 0;
    __u32* point = NULL;
    const __u8* p_wf_data = NULL;
    __u32 row, col = 0, row_temp = 0, ctrl_data = 0, data = 0, temp = 0;
    __u8 index = 0;
    __u32 pixel1 = 0, pixel2 = 0, pixel3 = 0, pixel4 = 0;
	register __s32 total_frame = 0;
    __u32 *init_wf_file = disp_eink.p_wf_buffer;

	__s32 buf_index, wav_compose_index;

	buf_index = disp_eink.bufQueue.curBufIndex;

	wav_compose_index = disp_eink.bufQueue.slots[buf_index].wav_compose_index;

    if (wav_compose_index == 0)
    {
	    temperature = disp_eink.temperature;
        __EINK_DBG("when make init waveform, The temperature is %d \n", temperature);
	    if(temperature < 0)
	    {
	        temperature = 0;
	    }
	    if(temperature >= T_MAX)
	    {
	        temperature = T_MAX - 1;
	    }
        
        if (disp_eink.eink_moudule_type == 0)
        {
            disp_eink.eink_init_T_tbl   = eink_init_T_tbl_OED;
            disp_eink.eink_init_mode_wf = eink_init_mode_wf_OED;
        }
        else if (disp_eink.eink_moudule_type == 1)
        {
            disp_eink.eink_init_T_tbl   = eink_init_T_tbl_PVI;
            disp_eink.eink_init_mode_wf = eink_init_mode_wf_PVI;
        }
        
        if (disp_eink.eink_ctrl_data_type == 0)
        {
            disp_eink.eink_ctrl_tbl_GC16 = eink_ctrl_tbl_GC16_COMMON;
        }
        else if (disp_eink.eink_ctrl_data_type == 1)
        {
     //       disp_eink.eink_ctrl_tbl_GC16 = eink_ctrl_tbl_GC16_TIANZHI;
        }

		//get entry of table from temperature
		index = check_temp_area_index(temperature);
		__EINK_DBG("get index->[%d] from temp area with temperature->[%d]\n", index, temperature);
		__u32* ptemp = init_wf_file + index;		
		__u16* pFrame = (__u16*)((__u32)init_wf_file + *ptemp); 

		//get total frame
		disp_eink.bufQueue.slots[buf_index].total_frame = *pFrame;
		total_frame = disp_eink.bufQueue.slots[buf_index].total_frame;
		__EINK_DBG("get total frame->[%d] from p_init_wf of awf file\n", *pFrame);

		//added by yuanlian
		//get divider
		pFrame++;
		disp_eink.wf_len_divider = *pFrame;
		__EINK_DBG("get divider->[%d] from p_init_wf of awf file\n", *pFrame);

		//get the waveform data entry
		pFrame++;
		disp_eink.pframe_data = (__s32*)pFrame;                  //wf data address is the pFrame

	    __EINK_DBG("eink_init_update_waveform_awf: total_frame->[%d]\n", disp_eink.bufQueue.slots[buf_index].total_frame);
    }
	
	//__EINK_MSG("eink_init_update_waveform_awf: wav_compose_index = %d\n", disp_eink.wav_compose_index);
    if(wav_compose_index < disp_eink.bufQueue.slots[buf_index].total_frame)
    {   
	    frame_id = wav_compose_index;
		p_wf_data = (__u8*)disp_eink.pframe_data;
		temp_index =  disp_eink.bufQueue.slots[buf_index].wav_compose_index % EINK_WAV_BUFFER_NUM;
    	__EINK_MSG("eink_init_update_waveform_awf: wav_index = %d\n", disp_eink.wav_buffer[temp_index].wav_index);

    	if(disp_eink.wav_buffer[temp_index].wav_index == -1)
    	{
	        point =  disp_eink.wav_buffer[temp_index].wav_address;
	
	        for(row = 0; row < disp_eink.wav_height; row++)
	        {
	            row_temp = eink_ctrl_line_index_tbl[row];
	            for(col = 0; col < disp_eink.wav_width; col++)
	            {                            
	                //有效数据区, 行(6-605), 列(6-205)
	                if((row > (EINK_FSL+EINK_FBL-1))&&(row < (EINK_LCD_H - EINK_FEL)))
	                {
	                    if((col > (EINK_LSL+EINK_LBL-1))&&(col < (EINK_LCD_W - EINK_LEL)))
	                    {                       
	                        pixel1 =  *(p_wf_data + frame_id);
	                        pixel2 =  pixel1;
	                        pixel3 =  pixel1;
	                        pixel4 =  pixel1;
	                        
	                        data = pixel1<<11;           //D12, D11
	                        
	                        data |= (pixel2&0x02)<<9;     //D10
	                        data |= (pixel2&0x01)<<7;     //D7
	                        
	                        data |= pixel3<<5;           //D6,D5
	                        data |= pixel4<<3;           //D4,D3                           
	                        
	                        ctrl_data = disp_eink.eink_ctrl_tbl_GC16[row_temp][col];                      
	                        temp = ctrl_data | data; 
	                        *point = temp;
	                        point++;
	                        continue;
	                    }
	                }            
	                ctrl_data = disp_eink.eink_ctrl_tbl_GC16[row_temp][col];                    
	                temp = ctrl_data | data; 
	
	                *point = temp;
	                point++;
	            }
	        }
			disp_eink.wav_buffer[temp_index].wav_index =  disp_eink.bufQueue.slots[buf_index].wav_compose_index;
			disp_eink.bufQueue.slots[buf_index].wav_compose_index++;
			
	    }
	    else
	    {
	    	return 0;
	    }
    }
    else
    {
    	return -1;
    }

    return 0;
}

#ifdef CONFIG_FB_SUNXI_CONN_SCRAMBLED 
#define gc16_update_waveform() \
{ \
        point = disp_eink.wav_buffer[temp_index].wav_address;\
        point += (EINK_LCD_W * (EINK_FSL + EINK_FBL));\
\
        for(row = (EINK_FSL + EINK_FBL); row < (EINK_FSL + EINK_FBL + EINK_FDL); row++)\
        {\
        	point += (EINK_LSL + EINK_LBL);\
		for(col = (EINK_LSL + EINK_LBL); col < (EINK_LSL + EINK_LBL + 4); col++)\
		{\
			if ((col % 16) == 0)\
				__builtin_prefetch(p_wf_index + 256);\
			pixel1 = *(p_wf_data + (*p_wf_index++));\
			pixel2 = *(p_wf_data + (*p_wf_index++));\
			pixel3 = *(p_wf_data + (*p_wf_index++));\
			pixel4 = *(p_wf_data + (*p_wf_index++));\
			data = pixel1<<11;\
			data |= (pixel2&0x02)<<9;\
			data |= (pixel2&0x01)<<7;\
			data |= pixel3<<5;\
			data |= pixel4<<3;\
			*point++ = (data | 0xD08000|0xff030303);\
		}\
		for(col = (EINK_LSL + EINK_LBL + 4); col < (EINK_LSL + EINK_LBL + EINK_LDL); col++)\
		{\
			if ((col % 16) == 0)\
				__builtin_prefetch(p_wf_index + 256);\
			pixel1 = *(p_wf_data + (*p_wf_index++));\
			pixel2 = *(p_wf_data + (*p_wf_index++));\
			pixel3 = *(p_wf_data + (*p_wf_index++));\
			pixel4 = *(p_wf_data + (*p_wf_index++));\
			data = pixel1<<11;\
			data |= (pixel2&0x02)<<9;\
			data |= (pixel2&0x01)<<7;\
			data |= pixel3<<5;\
			data |= pixel4<<3;\
			*point++ = (data | 0xF08000|0xff030303);\
		}\
        	point += EINK_LEL;\
        }\
	 disp_eink.wav_buffer[temp_index].wav_index = wav_compose_index;\
	 disp_eink.bufQueue.slots[buf_index].wav_compose_index++;    \
}
#else 
#ifdef CONFIG_FB_SUNXI_CONN_LINEAR

#define gc16_update_waveform() \
{ \
        point = disp_eink.wav_buffer[temp_index].wav_address;\
        point += (EINK_LCD_W * (EINK_FSL + EINK_FBL));\
\
        for(row = (EINK_FSL + EINK_FBL); row < (EINK_FSL + EINK_FBL + EINK_FDL); row++)\
        {\
        	point += (EINK_LSL + EINK_LBL);\
		for(col = (EINK_LSL + EINK_LBL); col < (EINK_LSL + EINK_LBL + 4); col++)\
		{\
			if ((col % 16) == 0)\
				__builtin_prefetch(p_wf_index + 256);\
			pixel1 = *(p_wf_data + (*p_wf_index++));\
			pixel2 = *(p_wf_data + (*p_wf_index++));\
			pixel3 = *(p_wf_data + (*p_wf_index++));\
			pixel4 = *(p_wf_data + (*p_wf_index++));\
			data  = pixel1<<10;\
			data |= pixel2<<6;\
			data |= pixel3<<4;\
			data |= pixel4<<2;\
			*point++ = (data | 0xD08000|0xff030303);\
		}\
		for(col = (EINK_LSL + EINK_LBL + 4); col < (EINK_LSL + EINK_LBL + EINK_LDL); col++)\
		{\
			if ((col % 16) == 0)\
				__builtin_prefetch(p_wf_index + 256);\
			pixel1 = *(p_wf_data + (*p_wf_index++));\
			pixel2 = *(p_wf_data + (*p_wf_index++));\
			pixel3 = *(p_wf_data + (*p_wf_index++));\
			pixel4 = *(p_wf_data + (*p_wf_index++));\
			data  = pixel1<<10;\
			data |= pixel2<<6;\
			data |= pixel3<<4;\
			data |= pixel4<<2;\
			*point++ = (data | 0xF08000|0xff030303);\
		}\
        	point += EINK_LEL;\
        }\
	 disp_eink.wav_buffer[temp_index].wav_index = wav_compose_index;\
	 disp_eink.bufQueue.slots[buf_index].wav_compose_index++;    \
}

#endif
#endif

static __s32 (*eink_gc16_update_waveform_awf)(__u32*);

static __s32 eink_gc16_update_waveform_awf_linear(__u32* p_wf_file)
{
	__u32 temperature = 0, temp_index = 0;
	register __u32* point = NULL;
	register __u8* p_wf_index = NULL;

	register const __u8* p_wf_data = NULL;
	register __u32 row = 0 , col = 0, FRi = 0, TRi = 0, data = 0;

	register __u32 pixel1 = 0, pixel2 = 0, pixel3 = 0, pixel4 = 0;

	register __s32 wav_compose_index;
	register __s32 total_frame = 0;
	int index = 0;

	__u32 *input_wf_file = p_wf_file;
	__s32 buf_index;


	buf_index = disp_eink.bufQueue.curBufIndex;

	wav_compose_index = disp_eink.bufQueue.slots[buf_index].wav_compose_index;

	if (input_wf_file == NULL) {
		__EINK_WRN("waveform data doesn't exist\n");            
		return -1;
	}
	if (wav_compose_index == -1) {
		return -1;
	}
	if(wav_compose_index == 0)
	{
		temperature = disp_eink.temperature;         
		if (temperature < 0) {
			temperature = 0;
		}
		if (temperature >= T_MAX) {
			temperature = T_MAX - 1;
		}

		if (disp_eink.eink_ctrl_data_type == 0) {
			disp_eink.eink_ctrl_tbl_GC16 = eink_ctrl_tbl_GC16_COMMON;
		}
		else if (disp_eink.eink_ctrl_data_type == 1) {
			//disp_eink.eink_ctrl_tbl_GC16 = eink_ctrl_tbl_GC16_TIANZHI;
		}

		//get entry of table from temperature
		index = check_temp_area_index(temperature);
		__u32* ptemp = input_wf_file + index;		
		__u16* pFrame = (__u16*)((__u32)input_wf_file + *ptemp); 

		//get total frame
		disp_eink.bufQueue.slots[buf_index].total_frame = *pFrame;

		//get divider
		pFrame++;
		disp_eink.wf_len_divider = *pFrame;

		//get the waveform data entry
		pFrame++;
		p_wf_data = (__u8*)pFrame;
		//reorder waveform
		memset(disp_eink.pframe_data, 0, 256*256);
		for (TRi = 0; TRi < 256; TRi++) 
			for (FRi = 0; FRi < (1 << disp_eink.wf_len_divider); FRi++) 
				*(disp_eink.pframe_data + (256 * FRi) + TRi)  = *p_wf_data++;

		pr_info("[EINK]%s: total_frame=%d, divider=%d\n", \
		__func__, disp_eink.bufQueue.slots[buf_index].total_frame, disp_eink.wf_len_divider);
	}

	total_frame = disp_eink.bufQueue.slots[buf_index].total_frame;
	if (wav_compose_index < total_frame) {
		p_wf_data = (__u8*)disp_eink.pframe_data + (256 * wav_compose_index);
		temp_index = wav_compose_index % EINK_WAV_BUFFER_NUM;
		if (disp_eink.wav_buffer[temp_index].wav_index == -1) 
		{
			point = disp_eink.wav_buffer[temp_index].wav_address;
			p_wf_index = disp_eink.wav_form_index;
			point = disp_eink.wav_buffer[temp_index].wav_address;
			point += (EINK_LCD_W * (EINK_FSL + EINK_FBL));

			for(row = (EINK_FSL + EINK_FBL); row < (EINK_FSL + EINK_FBL + EINK_FDL); row++)
	        {
	        	point += (EINK_LSL + EINK_LBL);
				for(col = (EINK_LSL + EINK_LBL); col < (EINK_LSL + EINK_LBL + 4); col++)
				{
					if ((col % 16) == 0)
						__builtin_prefetch(p_wf_index + 256);
					pixel1 = *(p_wf_data + (*p_wf_index++));
					pixel2 = *(p_wf_data + (*p_wf_index++));
					pixel3 = *(p_wf_data + (*p_wf_index++));
					pixel4 = *(p_wf_data + (*p_wf_index++));
					data  = pixel1<<10;
					data |= pixel2<<6;
					data |= pixel3<<4;
					data |= pixel4<<2;
					*point++ = (data | 0xD08000|0xff030303);
				}
				for(col = (EINK_LSL + EINK_LBL + 4); col < (EINK_LSL + EINK_LBL + EINK_LDL); col++)
				{
					if ((col % 16) == 0)
						__builtin_prefetch(p_wf_index + 256);
					pixel1 = *(p_wf_data + (*p_wf_index++));
					pixel2 = *(p_wf_data + (*p_wf_index++));
					pixel3 = *(p_wf_data + (*p_wf_index++));
					pixel4 = *(p_wf_data + (*p_wf_index++));
					data  = pixel1<<10;
					data |= pixel2<<6;
					data |= pixel3<<4;
					data |= pixel4<<2;
					*point++ = (data | 0xF08000|0xff030303);
				}
		        	point += EINK_LEL;
	        }
			 disp_eink.wav_buffer[temp_index].wav_index = wav_compose_index;
			 disp_eink.bufQueue.slots[buf_index].wav_compose_index++;    
		}
		else {
			return 0;
		} 
		__EINK_INF("--\n");
	}
	else {
		return -1;
	}

	return 0;

}

static __s32 eink_gc16_update_waveform_awf_scrambled(__u32* p_wf_file)
{
	__u32 temperature = 0, temp_index = 0;
	register __u32* point = NULL;
	register __u8* p_wf_index = NULL;

	register const __u8* p_wf_data = NULL;
	register __u32 row = 0 , col = 0, FRi = 0, TRi = 0, data = 0;

	register __u32 pixel1 = 0, pixel2 = 0, pixel3 = 0, pixel4 = 0;

	register __s32 wav_compose_index;
	register __s32 total_frame = 0;
	int index = 0;

	__u32 *input_wf_file = p_wf_file;
	__s32 buf_index;


	buf_index = disp_eink.bufQueue.curBufIndex;

	wav_compose_index = disp_eink.bufQueue.slots[buf_index].wav_compose_index;

	if (input_wf_file == NULL) {
		__EINK_WRN("waveform data doesn't exist\n");            
		return -1;
	}
	if (wav_compose_index == -1) {
		return -1;
	}
	if(wav_compose_index == 0)
	{
		temperature = disp_eink.temperature;         
		if (temperature < 0) {
			temperature = 0;
		}
		if (temperature >= T_MAX) {
			temperature = T_MAX - 1;
		}

		if (disp_eink.eink_ctrl_data_type == 0) {
			disp_eink.eink_ctrl_tbl_GC16 = eink_ctrl_tbl_GC16_COMMON;
		}
		else if (disp_eink.eink_ctrl_data_type == 1) {
			//disp_eink.eink_ctrl_tbl_GC16 = eink_ctrl_tbl_GC16_TIANZHI;
		}

		//get entry of table from temperature
		index = check_temp_area_index(temperature);
		__u32* ptemp = input_wf_file + index;		
		__u16* pFrame = (__u16*)((__u32)input_wf_file + *ptemp); 

		//get total frame
		disp_eink.bufQueue.slots[buf_index].total_frame = *pFrame;

		//get divider
		pFrame++;
		disp_eink.wf_len_divider = *pFrame;

		//get the waveform data entry
		pFrame++;
		p_wf_data = (__u8*)pFrame;
		//reorder waveform
		memset(disp_eink.pframe_data, 0, 256*256);
		for (TRi = 0; TRi < 256; TRi++) 
			for (FRi = 0; FRi < (1 << disp_eink.wf_len_divider); FRi++) 
				*(disp_eink.pframe_data + (256 * FRi) + TRi)  = *p_wf_data++;

		pr_info("[EINK]%s: total_frame=%d, divider=%d\n", \
		__func__, disp_eink.bufQueue.slots[buf_index].total_frame, disp_eink.wf_len_divider);
	}

	total_frame = disp_eink.bufQueue.slots[buf_index].total_frame;
	if (wav_compose_index < total_frame) 
	{
		p_wf_data = (__u8*)disp_eink.pframe_data + (256 * wav_compose_index);
		temp_index = wav_compose_index % EINK_WAV_BUFFER_NUM;
		if (disp_eink.wav_buffer[temp_index].wav_index == -1) 
		{

			point = disp_eink.wav_buffer[temp_index].wav_address;
			p_wf_index = disp_eink.wav_form_index;

			point = disp_eink.wav_buffer[temp_index].wav_address;
			point += (EINK_LCD_W * (EINK_FSL + EINK_FBL));

	        for(row = (EINK_FSL + EINK_FBL); row < (EINK_FSL + EINK_FBL + EINK_FDL); row++)
			{
	        	point += (EINK_LSL + EINK_LBL);
				for(col = (EINK_LSL + EINK_LBL); col < (EINK_LSL + EINK_LBL + 4); col++)
				{
					if ((col % 16) == 0)
						__builtin_prefetch(p_wf_index + 256);
					pixel1 = *(p_wf_data + (*p_wf_index++));
					pixel2 = *(p_wf_data + (*p_wf_index++));
					pixel3 = *(p_wf_data + (*p_wf_index++));
					pixel4 = *(p_wf_data + (*p_wf_index++));
					data = pixel1<<11;
					data |= (pixel2&0x02)<<9;
					data |= (pixel2&0x01)<<7;
					data |= pixel3<<5;
					data |= pixel4<<3;
					*point++ = (data | 0xD08000|0xff030303);
				}
				for(col = (EINK_LSL + EINK_LBL + 4); col < (EINK_LSL + EINK_LBL + EINK_LDL); col++)
				{
					if ((col % 16) == 0)
						__builtin_prefetch(p_wf_index + 256);
					pixel1 = *(p_wf_data + (*p_wf_index++));
					pixel2 = *(p_wf_data + (*p_wf_index++));
					pixel3 = *(p_wf_data + (*p_wf_index++));
					pixel4 = *(p_wf_data + (*p_wf_index++));
					data = pixel1<<11;
					data |= (pixel2&0x02)<<9;
					data |= (pixel2&0x01)<<7;
					data |= pixel3<<5;
					data |= pixel4<<3;
					*point++ = (data | 0xF08000|0xff030303);
				}
				point += EINK_LEL;
			}
		disp_eink.wav_buffer[temp_index].wav_index = wav_compose_index;
		disp_eink.bufQueue.slots[buf_index].wav_compose_index++;
		}
		else {
			return 0;
		} 
		__EINK_INF("--\n");
	}
	else {
		return -1;
	}

	return 0;

}

#endif


static __s32 eink_unload_waveform(__u32 * wav_buf)
{
    if(wav_buf != NULL)
    {
        vfree(wav_buf);        		
    }  
    return 0;
}

static __s32 eink_init_update_waveindex(void)
{
    int col = 0, row = 0, width = 0, height = 0;
    register __u8 index = 0;
    register __u8* p_old_data = disp_eink.wav_last_frame;
    register __u8* p_wf_index = disp_eink.wav_form_index;    

    width = disp_eink.frame_width;
    height = disp_eink.frame_height;
    __EINK_LOG("width = %d, height = %d\n", width, height);
    
    for(row =0; row < height; row++)
    {
        for(col = 0; col < width; col++)
        {            
            index = 0;
            *p_wf_index++ = index;
            *p_old_data++ = 0xff;
        }
    }
    __EINK_LOG("----------index = %d----------------------------------------\n", index);  

   return EPDK_OK;
}

static __s32 eink_a2_update_waveindex(void)
{
    register int col = 0, row = 0, width = 0, height = 0;
    register __u8 index = 0;
    register __u8* p_new_data;			//新显示数据
    register __u8* p_old_data = disp_eink.wav_last_frame; 					//前一次显示的数据
    register __u8* p_wf_index = disp_eink.wav_form_index;					//波形文件索引号    
    register __u8 new = 0,old = 0;    

	p_new_data = disp_eink.bufQueue.slots[disp_eink.bufQueue.curBufIndex].buffer;
	__eink_update_mode update_mode = disp_eink.bufQueue.slots[disp_eink.bufQueue.curBufIndex].flushMode;

	__s32 x_start = disp_eink.bufQueue.slots[disp_eink.bufQueue.curBufIndex].x_start;
	__s32 x_end = disp_eink.bufQueue.slots[disp_eink.bufQueue.curBufIndex].x_end;
	__s32 y_start = disp_eink.bufQueue.slots[disp_eink.bufQueue.curBufIndex].y_start;
	__s32 y_end = disp_eink.bufQueue.slots[disp_eink.bufQueue.curBufIndex].y_end;
	
    width = disp_eink.frame_width;
    height = disp_eink.frame_height;
    __EINK_LOG("xstart=[%d], xend=[%d], ystart=[%d], yend=[%d]\n", x_start, x_end, y_start, y_end);
    
	__s32 offset = 0;
	for(row=y_start; row<y_end; row++){
		offset = row*EINK_PANEL_W+x_start;
		for(col=x_start; col<x_end; col++){
			new = p_new_data[offset]>>4;
			old = p_old_data[offset]>>4;
			index = ((old << 4) + new);
			p_wf_index[offset]=index;
			p_old_data[offset] = p_new_data[offset];
			offset++;
		}
	}
    __EINK_LOG("----------index a2 finished---------\n");    
    
    return 0;
}

static __s32 eink_gc16_update_waveindex(void)
{
    register int col = 0, row = 0, width = 0, height = 0;
    register __u8 index = 0;
    register __u8* p_new_data;			//新显示数据
    register __u8* p_old_data = disp_eink.wav_last_frame; 					//前一次显示的数据
    register __u8* p_wf_index = disp_eink.wav_form_index;					//波形文件索引号    
    register __u8 new = 0,old = 0;    

	p_new_data = disp_eink.bufQueue.slots[disp_eink.bufQueue.curBufIndex].buffer;
	__eink_update_mode update_mode = disp_eink.bufQueue.slots[disp_eink.bufQueue.curBufIndex].flushMode;

	__s32 x_start = disp_eink.bufQueue.slots[disp_eink.bufQueue.curBufIndex].x_start;
	__s32 x_end = disp_eink.bufQueue.slots[disp_eink.bufQueue.curBufIndex].x_end;
	__s32 y_start = disp_eink.bufQueue.slots[disp_eink.bufQueue.curBufIndex].y_start;
	__s32 y_end = disp_eink.bufQueue.slots[disp_eink.bufQueue.curBufIndex].y_end;
	
    width = disp_eink.frame_width;
    height = disp_eink.frame_height;
    __EINK_LOG("width = %d, height = %d\n", width, height);
    
    for(row =0; row < height; row++)
    {
        for(col = 0; col < width; col++)
        {   
				//20130417, yuanlian. keep origianl pixel value out of rectangle
			//just need to update inside pixels
			if(update_mode & EINK_RECTANGLE_MODE){
				if((col >= x_start) && (col < x_end) && (row >= y_start) && (row < y_end)){

				}
				else{
					*p_new_data = *p_old_data;
				}
			}
			if((update_mode & EINK_SHORT_GC16_LOCAL_MODE) && (*p_old_data==*p_new_data))
			{
				*p_old_data  = 0xff;
			}

            new = *p_new_data>>4;                //8bpp -> 4bpp
            old = *p_old_data>>4;  


			index = ((old << 4) | new);
            *p_wf_index++ = index;                
            *p_old_data++ = *p_new_data++;
        }
    }     
    __EINK_LOG("----------index---------\n");    
    
    return 0;
}

__s32 eink_du_update_local_waveindex(void)
{
    int col = 0, row = 0, width = 0, height = 0;
    __u8 index = 0;
    __u8* p_new_data;			//新显示数据
    __u8* p_old_data = disp_eink.wav_last_frame; 					//前一次显示的数据
    __u8* p_wf_index = disp_eink.wav_form_index;					//波形文件索引号    
    __u8 new = 0,old = 0;   
    __u8 eink_type = OPM060A1;

	p_new_data = disp_eink.bufQueue.slots[disp_eink.bufQueue.curBufIndex].buffer;
	__eink_update_mode update_mode = disp_eink.bufQueue.slots[disp_eink.bufQueue.curBufIndex].flushMode;

	__s32 x_start = disp_eink.bufQueue.slots[disp_eink.bufQueue.curBufIndex].x_start;
	__s32 x_end = disp_eink.bufQueue.slots[disp_eink.bufQueue.curBufIndex].x_end;
	__s32 y_start = disp_eink.bufQueue.slots[disp_eink.bufQueue.curBufIndex].y_start;
	__s32 y_end = disp_eink.bufQueue.slots[disp_eink.bufQueue.curBufIndex].y_end;
	

    __EINK_INF("-----------------------DU local MODE---------------------------\n");

    if (disp_eink.eink_moudule_type == 0)
    {
        eink_type = OPM060A1;
    }
    else if ((disp_eink.eink_moudule_type == 1)
          && (disp_eink.eink_version_type == 0))
    {
        eink_type = OPM060A1;//ED060SC4;
    }
    else if ((disp_eink.eink_moudule_type == 1)
          && (disp_eink.eink_version_type == 1))
    {
        eink_type = ED060SC7;
    }
    
    width  = disp_eink.frame_width;
    height = disp_eink.frame_height;
    __EINK_INF("width = %d, height = %d\n", width, height);
    for(row =0; row < height; row++)
    {
        for(col = 0; col < width; col++)
        {            
			//20130417, yuanlian. keep origianl pixel value out of rectangle
			//just need to update inside pixels
			if(update_mode & EINK_RECTANGLE_MODE){
				if(col >= x_start && col < x_end && row >= y_start && row < y_end){
				}
				else{
					*p_new_data = *p_old_data;
				}
			}

            new = *p_new_data>>4;                //8bpp -> 4bpp
            old = *p_old_data>>4;  

            if (eink_type == OPM060A1) 
            {
/*                if(new == old)                      //局部模式,若相等，则index = 0,waveform波形送0电平*/
                if(*p_new_data == *p_old_data)                      //局部模式,若相等，则index = 0,waveform波形送0电平
                {
                    index = 0;
                }
                else
                {
                    if(new)
                    {
                        index = old + 16;               //G0,G15-->G15
                    }
                    else
                    {
                        index = old;                    // G0, G15 --> G0
                    }
                }
                *p_wf_index++ = index;
                *p_old_data++ = *p_new_data++;    
            }
            else if ((eink_type == ED060SC7)||(eink_type == ED060SC4)) 
            {
/*                if(new == old)                      //局部模式,若相等，则index = 0,waveform波形送0电平*/
                if(*p_new_data == *p_old_data)                      //局部模式,若相等，则index = 0,waveform波形送0电平
                {
                    index = 0;
                }
                else
                {
                    index = old * 16 + new;             //
                }
                *p_wf_index++ = index;                
                *p_old_data++ = *p_new_data++;
            }
        }
    } 
    
    return EPDK_OK;
}

__s32 eink_du_update_waveindex(void)
{
    int col = 0, row = 0, width = 0, height = 0;
    register __u8 index = 0;
    register __u8* p_new_data;			//新显示数据
    register __u8* p_old_data = disp_eink.wav_last_frame; 					//前一次显示的数据
    register __u8* p_wf_index = disp_eink.wav_form_index;					//波形文件索引号    
    register __u8 new = 0,old = 0;   
    __u8 eink_type = OPM060A1;
    
    __EINK_INF("-----------------------DU All area MODE---------------------------\n");
	p_new_data = disp_eink.bufQueue.slots[disp_eink.bufQueue.curBufIndex].buffer;
	__eink_update_mode update_mode = disp_eink.bufQueue.slots[disp_eink.bufQueue.curBufIndex].flushMode;

	__s32 x_start = disp_eink.bufQueue.slots[disp_eink.bufQueue.curBufIndex].x_start;
	__s32 x_end = disp_eink.bufQueue.slots[disp_eink.bufQueue.curBufIndex].x_end;
	__s32 y_start = disp_eink.bufQueue.slots[disp_eink.bufQueue.curBufIndex].y_start;
	__s32 y_end = disp_eink.bufQueue.slots[disp_eink.bufQueue.curBufIndex].y_end;
	
    if (disp_eink.eink_moudule_type == 0)
    {
        eink_type = OPM060A1;
    }
    else if ((disp_eink.eink_moudule_type == 1)
          && (disp_eink.eink_version_type == 0))
    {
        eink_type = OPM060A1;//ED060SC4;
    }
    else if ((disp_eink.eink_moudule_type == 1)
          && (disp_eink.eink_version_type == 1))
    {
        eink_type = ED060SC7;
    }
    
    width  = disp_eink.frame_width; //PIXEL_MONO_8BPP,图层中每个byte存放一个pixel
    height = disp_eink.frame_height;
    __EINK_INF("width = %d, height = %d\n", width, height);
    
    for(row =0; row < height; row++)
    {
        for(col = 0; col < width; col++)
        {   

			//20130417, yuanlian. keep origianl pixel value out of rectangle
			//just need to update inside pixels
			if(update_mode & EINK_RECTANGLE_MODE){
				if(col >= x_start && col < x_end && row >= y_start && row < y_end){
				}
				else{
					*p_new_data = *p_old_data;
				}
			}
			else{
			}

			//1----------------------------------------------
            new = *p_new_data>>4;                //8bpp -> 4bpp
            old = *p_old_data>>4;  
     
            if (eink_type == OPM060A1) 
            {
                if(new)
                {
                    index = old + 16;               //G0,G15-->G15
                }
                else
                {
                    index = old;                    // G0, G15 --> G0
                }
                *p_wf_index++ = index;
                *p_old_data++ = *p_new_data++;   
            }
            else if ((eink_type == ED060SC7)||(eink_type == ED060SC4)) 
            {
                index = old * 16 + new;             //
                *p_wf_index++ = index;                
                *p_old_data++ = *p_new_data++;
            }
        }
    }
    
    return EPDK_OK;
}


static __s32 eink_compose_one_frame(void)
{
	int buf_index = disp_eink.bufQueue.curBufIndex;
	int update_mode = disp_eink.bufQueue.slots[buf_index].flushMode;

//	__EINK_LOG("eink_compose_one_frame update_mode = %x\n", update_mode);

	if (disp_eink.bufQueue.slots[buf_index].wav_compose_index == 0)
	{
		disp_eink.last_update_mode = update_mode;

		if (update_mode & EINK_INIT_MODE) {
			__EINK_DBG("EINK_INIT_MODE\n");
			eink_init_update_waveindex();
		}
		else if ((update_mode & EINK_GC16_MODE) 
					|| (update_mode & (EINK_GC16_MODE|EINK_LOCAL_MODE)) 
					|| (update_mode & EINK_SHORT_GC16_LOCAL_MODE))
		{
			__EINK_DBG("EINK_SHORT_GC16_LOCAL_MODE or EINK_GC16_MODE | EINK_LOCAL_MODE\n");
			eink_gc16_update_waveindex();
		}
		else if (update_mode & EINK_DU_MODE)
		{
			if (update_mode & EINK_LOCAL_MODE) {
				__EINK_DBG("EINK_DU_MODE | EINK_LOCAL_MODE\n");
#ifdef AWF_WAVEFORM_SUPPORTED
				eink_gc16_update_waveindex();
#else
				eink_du_update_local_waveindex();
#endif
			} else {
				__EINK_DBG("EINK_DU_MODE\n");
#ifdef AWF_WAVEFORM_SUPPORTED
				eink_gc16_update_waveindex();
#else
				eink_du_update_waveindex();
#endif			
			}
		}
		else if (update_mode & EINK_SHORT_DU_MODE){
			__EINK_DBG("EINK_SHORT_DU_MODE\n");
			eink_du_update_local_waveindex();
		}
		else if (update_mode & EINK_A2_MODE) {
			__EINK_DBG("EINK_A2_MODE\n");
			eink_a2_update_waveindex();
		}
		else if (update_mode & EINK_A2_IN_MODE) {
			__EINK_DBG("EINK_A2_IN_MODE\n");
			eink_a2_update_waveindex();
		}
		else if (update_mode & EINK_A2_OUT_MODE) {
			__EINK_DBG("EINK_A2_OUT_MODE\n");
			eink_a2_update_waveindex();
		}
	}

	/* get the waveform data */
	if (update_mode == EINK_INIT_MODE) {
#ifdef AWF_WAVEFORM_SUPPORTED
		eink_init_update_waveform_awf();
#else
		eink_init_update_waveform();
#endif
	}
	else if (update_mode & EINK_GC16_MODE) {
		if (update_mode & EINK_LOCAL_MODE){
#ifdef AWF_WAVEFORM_SUPPORTED		
			eink_gc16_update_waveform_awf(disp_eink.p_gc16_local_wf);
#else
			eink_gc16_update_waveform(disp_eink.wav_common_data);
#endif
		} else {
#ifdef AWF_WAVEFORM_SUPPORTED
			eink_gc16_update_waveform_awf(disp_eink.p_gc16_wf);
#else
			eink_gc16_update_waveform(disp_eink.wav_gc16_data);
#endif
		}
	}
	else if (update_mode & EINK_SHORT_GC16_LOCAL_MODE) {
		eink_gc16_update_waveform(disp_eink.wav_short_gc16_local_data);
	}
	else if ((update_mode & EINK_DU_MODE)
			|| (update_mode & (EINK_DU_MODE | EINK_LOCAL_MODE)))
	{
#ifdef AWF_WAVEFORM_SUPPORTED
		eink_gc16_update_waveform_awf(disp_eink.p_du_wf);
#else
		eink_gc16_update_waveform(disp_eink.wav_du_data);
#endif
	}
	else if (update_mode & EINK_SHORT_DU_MODE) {
		eink_gc16_update_waveform(disp_eink.wav_short_du_data);
	}
	else if (update_mode & EINK_A2_MODE) {
#ifdef AWF_WAVEFORM_SUPPORTED
		//eink_a2_update_waveform_awf(disp_eink.p_A2_wf);
		eink_gc16_update_waveform_awf(disp_eink.p_A2_wf);
#else
		eink_gc16_update_waveform(disp_eink.wav_a2_data);
#endif
	}
	else if (update_mode & EINK_A2_OUT_MODE) {
#ifdef AWF_WAVEFORM_SUPPORTED
		//eink_a2_update_waveform_awf(disp_eink.p_A2_out_wf);
		eink_gc16_update_waveform_awf(disp_eink.p_A2_out_wf);
#else
		eink_gc16_update_waveform(disp_eink.wav_a2_out_data);
#endif
	}
	else if (update_mode & EINK_A2_IN_MODE) {
#ifdef AWF_WAVEFORM_SUPPORTED
		//eink_a2_update_waveform_awf(disp_eink.p_A2_in_wf);
		eink_gc16_update_waveform_awf(disp_eink.p_A2_in_wf);
#else
		eink_gc16_update_waveform(disp_eink.wav_a2_in_data);
#endif
	}

	return 0;
}

void eink_TCON_IO_enable(void)
{
	unsigned int* pReg;
	pReg = (unsigned int*)0xf1c0c08c;
	*pReg &= (~0x04FFFFFF);
}

void eink_TCON_IO_disable(void)
{
	unsigned int* pReg;
	pReg = (unsigned int*)0xf1c0c08c;
	*pReg |= (0x04FFFFFF);
}

__s32 eink_FreeBuffer(__s32 index) 
{
	down(disp_eink.buffer_lock_sem);

	disp_eink.bufQueue.slots[index].bufState = FREE;
	disp_eink.bufQueue.slots[index].wav_compose_index = 0;
	disp_eink.bufQueue.slots[index].wav_refresh_index = 0;
	disp_eink.bufQueue.slots[index].total_frame = 0;
	disp_eink.bufQueue.slots[index].current_frame = 0;
	up(disp_eink.buffer_lock_sem);

	return 0;
}

__s32 eink_UpdateBuffer(void) 
{
	__s32 i = 0, found = -1;
	bool isOld;

	down(disp_eink.buffer_lock_sem);

	/*find the oldest DEQUEUED buffer*/
	for (i = 0; i < NUM_BUFFER_SLOTS; i++) 
	{
		if (disp_eink.bufQueue.slots[i].bufState == DEQUEUED) 
		{
			if (found != -1) 
			{
				/*find the oldest buffer*/
				isOld = disp_eink.bufQueue.slots[i].alloc_index < disp_eink.bufQueue.slots[found].alloc_index;
			}
			
			if (found < 0 || isOld) 
			{
				found = i;
			}
			
		}
	}

	
	__EINK_LOG("eink_UpdateBuffer found = %d\n", found);

	if (found == -1) 
	{
		up(disp_eink.buffer_lock_sem);
		return -1;
	}
		
	disp_eink.bufQueue.slots[found].bufState = QUEUED;
	disp_eink.bufQueue.useCount++;
	disp_eink.bufQueue.slots[found].use_index = disp_eink.bufQueue.useCount;
	disp_eink.bufQueue.curBufIndex = found;
	
	up(disp_eink.buffer_lock_sem);

	return 0;
}

static __s32 eink_refresh_pwr_ctrl(__s32 sel)
{
	__u32 i;
	__s32 buf_index;

	down(disp_eink.wav_compose_sem);
	__EINK_LOG("eink_refresh_pwr_ctrl \n");

	if (eink_UpdateBuffer() != 0) 
		return 0;

	disp_eink.update_status = 1;
	eink_compose_one_frame();

	buf_index = disp_eink.bufQueue.curBufIndex;
	__EINK_DBG("Lcd on ... \n");

	disp_eink.bufQueue.slots[buf_index].wav_refresh_index=1;
	Drv_disp_fb_set_layer_addr_eink(sel, WF_FB_ID, 0);

	eink_TCON_IO_enable();
	Drv_disp_eink_panel_on(sel);

	__EINK_DBG("wait for refresh waveform ... \n");
	down(disp_eink.power_off_sem);

	Drv_disp_eink_panel_off(sel);
	__EINK_DBG("Lcd off ... \n");

	for (i = 0; i < EINK_WAV_BUFFER_NUM; i++)
		disp_eink.wav_buffer[i].wav_index = -1;

	disp_eink.update_status = 0;

	eink_FreeBuffer(buf_index);

	return 0;
}

void eink_refresh_wav_form_1(unsigned long sel)
{
	BufSlot* bufSlot = &disp_eink.bufQueue.slots[disp_eink.bufQueue.curBufIndex];

	if (bufSlot->bufState != QUEUED) {
		__EINK_WRN("come to interrupt handle function but Eink not initial!\n");
		return;
	}


	if (bufSlot->wav_refresh_index == bufSlot->total_frame)
	{
		disp_eink.wav_buffer[(bufSlot->current_frame ) % EINK_WAV_BUFFER_NUM].wav_index = -1;
		bufSlot->wav_refresh_index++;
	}
	else if(bufSlot->wav_refresh_index == (bufSlot->total_frame+1))
	{
        disp_eink.wav_buffer[bufSlot->current_frame % EINK_WAV_BUFFER_NUM].wav_index = -1;
        bufSlot->current_frame = -1;
			
		disp_eink.last_update_mode = bufSlot->flushMode;

        eink_TCON_IO_disable();
        up(disp_eink.power_off_sem);

		//20130402, yuanlian
		//check the update mode and trigger the internal update task
		if((bufSlot->flushMode & (EINK_SHORT_DU_MODE | EINK_A2_MODE)) && (disp_eink.internal_update == 0)){
			int i = 0;
			//check if buffer queue is empty or not
			for(i=0; i<NUM_BUFFER_SLOTS; i++){
				if(i==disp_eink.bufQueue.curBufIndex){
					continue;
				}
				else if(disp_eink.bufQueue.slots[i].bufState !=FREE){
					break;
				}
			}

			if(i == NUM_BUFFER_SLOTS){
				disp_eink.internal_update = 1;
				__EINK_LOG("enable internal_update\n");
				up(disp_eink.internal_update_sem);
			}
		}
		else if (disp_eink.internal_update != 0){
			__EINK_LOG("disable internal_update\n");
			disp_eink.internal_update = 0;
		}
		else{}
    }
	else if (bufSlot->wav_refresh_index < bufSlot->total_frame)
    {
        if (disp_eink.wav_buffer[bufSlot->wav_refresh_index % EINK_WAV_BUFFER_NUM].wav_index != -1)
        {
        	bufSlot->current_frame = bufSlot->wav_refresh_index;
			if (bufSlot->current_frame >= 1)
			{
				disp_eink.wav_buffer[(bufSlot->current_frame - 1) % EINK_WAV_BUFFER_NUM].wav_index = -1;
			}

			
            Drv_disp_fb_set_layer_addr_eink(sel, WF_FB_ID, bufSlot->wav_refresh_index % EINK_WAV_BUFFER_NUM);
        	bufSlot->wav_refresh_index++;
        }
        else
        {
            printk(KERN_WARNING "frame[%d] not ready ... \n\n", bufSlot->wav_refresh_index);
        }
    }
    eink_compose_one_frame();
}

static __s32 internal_update_thread(void *p_arg)
{
	while (1) {
		down(disp_eink.internal_update_sem);
		msleep(500);

		if (disp_eink.internal_update == 0)
			continue;

		if (disp_eink.bufQueue.slots[disp_eink.bufQueue.curBufIndex].current_frame==0 
			&& (disp_eink.last_update_mode & (EINK_SHORT_DU_MODE | EINK_A2_MODE)))
		{
			int i = 0;
			//check if buffer queue is empty or not
			for (i=0; i<NUM_BUFFER_SLOTS; i++) {
				if (disp_eink.bufQueue.slots[i].bufState !=FREE)
					break;
			}

			//if all buffer all free, update once with short gc16 local mode
			if (NUM_BUFFER_SLOTS == i) {
				__EINK_LOG("internal update oncen");
				__u32 coordinate[4] = {0};
				if (disp_eink.last_update_mode & EINK_A2_MODE)
					Disp_eink_update(0,0, EINK_GC16_MODE, coordinate);
				else
					Disp_eink_update(0,0, EINK_SHORT_GC16_LOCAL_MODE, coordinate);
			} else {
				__EINK_LOG("buffer queue is not empty,skip internal flush\n");
			}
		} else {
			__EINK_LOG("eink is busy, skip internal_update::current_frame->[%d], mode->[%d]n", disp_eink.bufQueue.slots[disp_eink.bufQueue.curBufIndex].current_frame, disp_eink.last_update_mode);
		}

		disp_eink.internal_update = 0;
	}

	return 0;
}

static __s32 eink_refresh_thread(void *p_arg)
{
	while (1)
		eink_refresh_pwr_ctrl(0);

	return 0;
}

__s32 Disp_eink_get_sys_config(__u32 sel, Disp_eink_t* para)
{
    __s32 value = 0;
    char primary_key[20];
    __s32 ret = 0;
    
    sprintf(primary_key, "lcd%d_para", sel);
    
    ret = OSAL_Script_FetchParser_Data(primary_key, "lcd_eink_module_type", &value, 1);
    if(ret < 0)
    {
        DE_WRN("fetch script data %s.lcd_eink_module_type fail\n", primary_key);
    }
    else
    {
        para->eink_moudule_type = value;
        DE_INF("lcd_eink_module_type = %d\n", value);
    }

    ret = OSAL_Script_FetchParser_Data(primary_key, "lcd_eink_version_type", &value, 1);
    if(ret < 0)
    {
        DE_WRN("fetch script data %s.lcd_eink_version_type fail\n", primary_key);
    }
    else
    {
        para->eink_version_type = value;
        DE_INF("lcd_eink_version_type = %d\n", value);
    }

    ret = OSAL_Script_FetchParser_Data(primary_key, "lcd_eink_version", &value, 1);
    if(ret < 0)
    {
        DE_WRN("fetch script data %s.lcd_eink_version fail\n", primary_key);
    }
    else
    {
        para->eink_conn_version = value;
        DE_INF("lcd_eink_version = %d\n", value);
    }



    ret = OSAL_Script_FetchParser_Data(primary_key, "lcd_eink_ctrl_data_type", &value, 1);
    if(ret < 0)
    {
        DE_WRN("fetch script data %s.lcd_eink_ctrl_data_type fail\n", primary_key);
    }
    else
    {
        para->eink_ctrl_data_type = value;
        DE_INF("lcd_eink_ctrl_data_type = %d\n", value);
    }

    return 0;
}


void eink_initBufferQueue(void)
{
	__disp_fb_create_para_t fb_create_para;
	unsigned long screen_base;
	unsigned long buf_base;
	__u32 xoffset, yoffset;
	__s32 reqBufNum = 0;
	__s32 buf_size;
	__s32 i = 0;

	reqBufNum = (NUM_BUFFER_SLOTS - 3) / 4 + 2;

	__EINK_LOG("reqBufNum = %d\n", reqBufNum);
	eink_init_fb_para(&fb_create_para, disp_eink.frame_width, disp_eink.frame_height, reqBufNum);
	Drv_disp_Fb_Request(SCALER_OUT_FB_ID, &fb_create_para);
	Drv_disp_get_fb_info(SCALER_OUT_FB_ID, &xoffset, &yoffset, &screen_base);
	Drv_disp_fb_layer_set_bottom(0, SCALER_OUT_FB_ID);

	buf_size = disp_eink.frame_width * disp_eink.frame_height;
	buf_base = screen_base;
	disp_eink.bufQueue.bufCount = NUM_BUFFER_SLOTS;
	disp_eink.bufQueue.curBufIndex  = 0;

	for (i = 0; i < NUM_BUFFER_SLOTS; i++) {
		disp_eink.bufQueue.slots[i].use_index = 0;
		disp_eink.bufQueue.slots[i].alloc_index = 0;
		disp_eink.bufQueue.slots[i].bufState = FREE;
		disp_eink.bufQueue.slots[i].flushMode = EINK_INIT_MODE;
		disp_eink.bufQueue.slots[i].wav_compose_index = 0;
		disp_eink.bufQueue.slots[i].wav_refresh_index = 0;
		disp_eink.bufQueue.slots[i].buffer = (__u8 *)(buf_base + i * buf_size);
	}

	disp_eink.u_address = (__u8 *)(buf_base + i * buf_size);
	disp_eink.v_address = (__u8 *)(buf_base + (i+1) * buf_size);
}

static int eink_dequeueBuffer(__u32* indexBuf, __u32 mode, __u32 coordinate[])
{
	__s32 i = 0, found = -1;
	bool isOld;
	bool reuse = 0;
	int ret = 0;

	down(disp_eink.buffer_lock_sem);

	/* Looking for an empty slot */
	for (i = 0; i < NUM_BUFFER_SLOTS; i++)  {
		if (disp_eink.bufQueue.slots[i].bufState == FREE) {
			if (found != -1) {
				/* find the oldest buffer */
				isOld = disp_eink.bufQueue.slots[i].use_index < disp_eink.bufQueue.slots[found].use_index;
			}

			if (found < 0 || isOld)
				found = i;
		}
	}

	if (found >= 0) {
		/* Buffer found */
		disp_eink.bufQueue.slots[found].bufState = DEQUEUED;
		*indexBuf = found;
	} else {
		/* All buffers are full */
		ret = -EAGAIN;
		goto exit;
	}

	// else {
	// 	/* We did not find any empty slot */
	// 	reuse = 1;

	// 	/* cover the last alloc buffer */
	// 	for (i = 0; i < NUM_BUFFER_SLOTS; i++) {
	// 		if (disp_eink.bufQueue.slots[i].bufState == DEQUEUED) {
	// 			if (found != -1) {
	// 				/* find the oldest alloc buffer */
	// 				isOld = disp_eink.bufQueue.slots[i].alloc_index < disp_eink.bufQueue.slots[found].alloc_index;
	// 			}

	// 			if (found < 0 || isOld)
	// 				found = i;
	// 		}
	// 	}

	// 	disp_eink.bufQueue.slots[found].use_index = 0;
	// 	disp_eink.bufQueue.slots[found].bufState = DEQUEUED;

	// 	*indexBuf = found;
	// }

	disp_eink.bufQueue.allocCount++;
	disp_eink.bufQueue.slots[found].alloc_index = disp_eink.bufQueue.allocCount;
	disp_eink.bufQueue.slots[found].flushMode = mode;
	disp_eink.bufQueue.slots[found].wav_compose_index = 0;
	disp_eink.bufQueue.slots[found].wav_refresh_index = 0;
	disp_eink.bufQueue.slots[found].total_frame = 0;

	if (mode & EINK_RECTANGLE_MODE) {
		if (!reuse) {
			disp_eink.bufQueue.slots[found].x_start = coordinate[0];
			disp_eink.bufQueue.slots[found].x_end 	= coordinate[1];
			disp_eink.bufQueue.slots[found].y_start = coordinate[2];
			disp_eink.bufQueue.slots[found].y_end 	= coordinate[3];
		} else {
			disp_eink.bufQueue.slots[found].x_start = MIN(coordinate[0], disp_eink.bufQueue.slots[found].x_start);
			disp_eink.bufQueue.slots[found].x_end 	= MAX(coordinate[1], disp_eink.bufQueue.slots[found].x_end);
			disp_eink.bufQueue.slots[found].y_start = MIN(coordinate[2], disp_eink.bufQueue.slots[found].y_start);
			disp_eink.bufQueue.slots[found].y_end 	= MAX(coordinate[3], disp_eink.bufQueue.slots[found].y_end);
		}
	} else{
		disp_eink.bufQueue.slots[found].x_start = 0;
		disp_eink.bufQueue.slots[found].x_end 	= EINK_PANEL_H;
		disp_eink.bufQueue.slots[found].y_start = 0;
		disp_eink.bufQueue.slots[found].y_end 	= EINK_PANEL_W;
	}

	__EINK_LOG("dequueBufer amend index = %d\n", found);

exit:
	up(disp_eink.buffer_lock_sem);
	return ret;
}


__u8* eink_get32BppBuffer(void)
{
	__u32 xoffset, yoffset;
	unsigned long screen_base;
	__u32 bits_per_pixel;
	__u32 pixel_offset;
	__u8 *src32BppBuf = NULL;

	/*get 32bpp framebuffer*/
	Drv_disp_close_fb(0, 0);
	Drv_disp_get_fb_info(0, &xoffset, &yoffset, &screen_base);

	bits_per_pixel = 32;
	pixel_offset = disp_eink.frame_width * yoffset + xoffset;
	src32BppBuf = (__u8 *)screen_base + pixel_offset * bits_per_pixel / 8;

	return src32BppBuf;
}

static int eink_get8BppBuffer(__u32 mode, __u32 coordinate[], __u8 **buffer)
{
	int ret = 0;
	__u32 indexBuf;

	ret = eink_dequeueBuffer(&indexBuf, mode, coordinate);
	*buffer = disp_eink.bufQueue.slots[indexBuf].buffer;

	return ret;
}

__s32 eink_set1b_color(__u8* input_buf, __s32 xstart, __s32 xend, __s32 ystart,  __s32 yend)
{
	__s32 ret = 0;
	__s32 x = 0;
	__s32 y = 0;
	__u8 original_value = 0;
	__s32 offset = 0;

	int width, height;

	width  = disp_eink.frame_width;
	height = disp_eink.frame_height;

	printk("Converting dithering, xstart->[%d], xend->[%d], ystart->[%d], yend->[%d]\n", xstart, xend, ystart, yend);

	for (y = ystart; y < yend; y++) {
		for (x = xstart; x < xend; x++) {
			offset = y * EINK_PANEL_W + xstart;
			// get original pixel value
			original_value = input_buf[offset];

			// reset original value to new pixel
			input_buf[offset] = dithering_color[original_value];
			offset++;
		}
	}

	return ret;
}

__s32 eink_set_8bit_dithering(__u8* input_buf)
{
	__s32 ret = 0;

	__s32 x = 0;
	__s32 y = 0;

	__u8 new_pixel = 0;
	__u8 err_pixel = 0;
	__u8 temp_pixel = 0;
	__u8 original_value = 0;
	__s32 offset = 0;
	
	int width, height;

	width = disp_eink.frame_width;
	height = disp_eink.frame_height;

	__EINK_DBG("starting set 8bit dithering.");
	
	for (y=0; y<EINK_PANEL_H; y++){	//600
		for (x=0; x<EINK_PANEL_W; x++){	//800
			offset = y * EINK_PANEL_W + x;
			//get original pixel value
			original_value = input_buf[offset];

			
			//get new pixel value
			new_pixel = original_value & 0xf0 ;



			//get err value
			err_pixel = original_value-new_pixel;

			//reset original value to new pixel
			input_buf[offset] = new_pixel;

			if(new_pixel == 0xf0 || new_pixel == 0x10){
				continue;
			}

			//-------------
			/*
			 *belows coordinate testing
			 -------600-x-0--------
			0 				
			||		P1(1/4)	xy	
			y|		P2(1/4)	P3(2/4)	
			||				
		  EINK_PANEL_W|
			 topright(0,0): p1(x+1, y) p2(x+1, y+1) p3(x, y+1) 
			 */
			//1, set dithering
			if(y < (EINK_PANEL_H - 1)) {
				temp_pixel = input_buf[offset+EINK_PANEL_W] + err_pixel/4;
				if(temp_pixel < 0xf0 && temp_pixel > 0x10){
					input_buf[offset+EINK_PANEL_W] = temp_pixel;
				}
			}

			//2, set dithering
			if(y < (EINK_PANEL_H - 1) && (x < (EINK_PANEL_W - 1))) {
				temp_pixel = input_buf[offset+EINK_PANEL_W+1] + err_pixel/4;
				if(temp_pixel < 0xf0 && temp_pixel > 0x10){
					input_buf[offset+EINK_PANEL_W+1] = temp_pixel;
				}
			}

			//3, set dithering
/*            if(y<799) {*/
/*                temp_pixel = input_buf[offset+1] + err_pixel/2;*/
/*                if(temp_pixel < 0xf0 && temp_pixel > 0x10){*/
/*                    input_buf[offset+1] = temp_pixel;*/
/*                }*/

/*            }*/
		}
	}

	return ret;
}

__s32 eink_set_dithering(__u8* input_buf, __s32 xstart, __s32 xend, __s32 ystart,  __s32 yend)
{
	__s32 ret = 0;

	__s32 x = 0;
	__s32 y = 0;

	__u8 new_pixel = 0;
	__u8 err_pixel = 0;
	__u8 original_value = 0;
	__s32 offset = 0;
	
	int width, height;

	width = disp_eink.frame_width;
	height = disp_eink.frame_height;

	__EINK_DBG("converting dithering, xstart->[%d], xend->[%d], ystart->[%d], yend->[%d]\n", xstart, xend, ystart, yend);
/*    memcpy(dithering_buf, input_buf, 480000);*/
	
	for (y=ystart; y<yend; y++){	//600
		for (x=xstart; x<xend; x++){	//800
			offset = y * EINK_PANEL_W + x;
			//get original pixel value
/*            original_value = dithering_buf[x][y];*/
			original_value = input_buf[offset];

			//get new pixel value
			new_pixel = dithering_color[original_value];

			//get err value
			err_pixel = (original_value - new_pixel)/8;

			//reset original value to new pixel
/*            dithering_buf[x][y] = new_pixel;*/
			input_buf[offset] = new_pixel;

			//-------------
			/*
			 *belows coordinate testing
			 -------600-x-0--------
			0 		P1		
			||	P2	P3	xy	
			y|		P4	P5	
			||			p6	
		  800|
			 topright(0,0): p1(x+1, y-1) p2(x+2, y) p3(x+1, y) p4(x+1, y+1) p5(x, y+1) p6(x, y+2) 
			 */
			//1, set dithering
			if(y < (EINK_PANEL_H - 1) && x > 0) {
/*                dithering_buf[x+1][y-1] += err_pixel;*/
				input_buf[offset+(EINK_PANEL_W-1)] += err_pixel;
			}

			//2, set dithering
			if(y < (EINK_PANEL_H - 2)) {
/*                dithering_buf[x+2][y] += err_pixel;*/
				input_buf[offset+(EINK_PANEL_W*2)] += err_pixel;
			}

			//3, set dithering
			if(y < (EINK_PANEL_H - 1)) {
/*                dithering_buf[x+1][y] += err_pixel;*/
				input_buf[offset+EINK_PANEL_W] += err_pixel;
			}

			//4, set dithering
			if(y < (EINK_PANEL_H - 1) && (x < (EINK_PANEL_W - 1))) {
/*                dithering_buf[x+1][y+1] += err_pixel;*/
				input_buf[offset+EINK_PANEL_W+1] += err_pixel;
			}

			//5, set dithering
			if(x < (EINK_PANEL_W - 1)) {
/*                dwdithering_buf[x][y+1] += err_pixel;*/
				input_buf[offset+1] += err_pixel;
			}

			//6, set dithering
			if(x < (EINK_PANEL_W - 2)) {
/*                dithering_buf[x][y+2] += err_pixel;*/
				input_buf[offset+2] += err_pixel;
			}
		}
	}

/*    for (x=xstart; x<xend; x++){*/
/*        offset = x*800+ystart;*/
/*        for (y=ystart; y<yend; y++){*/
/*            input_buf[offset++]=dithering_buf[x][y];*/
/*        }*/
/*    }*/

	return ret;
}



__s32 eink_32To8Bpp(__u8* inupt_buf, __u8* output_buf) 
{
	__u32 xres, yres, addr_y, addr_u, addr_v;
	__disp_fb_t src, dst;
	__disp_rect_t src_rect;
	__disp_rectsz_t size;
	__s32 ret  = 0;    

	memset(&src, 0, sizeof(__disp_fb_t));
	memset(&dst, 0, sizeof(__disp_fb_t));
	memset(&src_rect, 0, sizeof(__disp_rect_t));

	addr_y = (__u32)output_buf;
	addr_u = (__u32)disp_eink.u_address;
	addr_v = (__u32)disp_eink.v_address;

	Drv_disp_get_fb_size(0, &xres, &yres);

	size.width      = xres;
	size.height     = yres;
	src_rect.x      = 0;
	src_rect.y      = 0;
	src_rect.width  = xres;
	src_rect.height = yres;
	eink_set_input_argb32b(&src, &size, (__u32)inupt_buf);
	eink_set_output_YuvPlanar(&dst, &size, addr_y, addr_u, addr_v);

	ret = eink_scale_image(&src, &src_rect, &dst);

	return ret;
}


void eink_setCurBufFlushMode(__u32 mode)
{
	down(disp_eink.buffer_lock_sem);
	disp_eink.bufQueue.slots[disp_eink.bufQueue.curBufIndex].flushMode = mode;
	up(disp_eink.buffer_lock_sem);
}

__u32 eink_getCurBufFlushMode(void)
{
	__u32 mode;

	down(disp_eink.buffer_lock_sem);
	mode = disp_eink.bufQueue.slots[disp_eink.bufQueue.curBufIndex].flushMode;
	up(disp_eink.buffer_lock_sem);

	return mode;
}


void initGlobalCtrlData()
{
        register __u32 frame=0, line = 0;
        register __u32* ptr = NULL;
        int BUFi = 0;

	for (BUFi = 0; BUFi < EINK_WAV_BUFFER_NUM; BUFi++) {
        	ptr = disp_eink.wav_buffer[BUFi].wav_address;     
		//frame: 0 -> (EINK_FSL - 1)  
		for (frame = 0; frame < EINK_FSL; frame++)
			for (line = 0; line < EINK_LCD_W; line++) 
				*ptr++ = 0xA00000|0xFF030303;

		//frame: EINK_FSL
		for (line = 0; line < (EINK_LSL + EINK_LBL); line++)
			*ptr++ = 0xA08000|0xff030303;
		for (line = (EINK_LSL + EINK_LBL); line < 108; line++)
			*ptr++ = 0xE08000|0xff030303;
		for (line = 108; line < (EINK_LSL + EINK_LBL + EINK_LDL + 1); line++)
			*ptr++ = 0x608000|0xff030303;
		for (line = (EINK_LSL + EINK_LBL + EINK_LDL + 1); line < EINK_LCD_W; line++)
			*ptr++ = 0x208000|0xff030303;
		frame++;

		//frame: EINK_FSL + 1
		for (line = 0; line < (EINK_LSL + EINK_LBL); line++)
			*ptr++ = 0x208000|0xff030303;
		for (line = (EINK_LSL + EINK_LBL); line < 108; line++)
			*ptr++ = 0x608000|0xff030303;
		for (line = 108; line < (EINK_LSL + EINK_LBL + EINK_LDL + 1); line++)
			*ptr++ = 0xE08000|0xff030303;
		for (line = (EINK_LSL + EINK_LBL + EINK_LDL + 1); line < EINK_LCD_W; line++)
			*ptr++ = 0xA08000|0xff030303;
		frame++;

		//frame: (EINK_FSL + 2) -> (EINK_FSL + EINK_FBL - 1)
		for (line = 0; line < (EINK_LSL + EINK_LBL); line++)
			*ptr++ = 0xA08000|0xff030303;
		for (line = (EINK_LSL + EINK_LBL); line < (EINK_LSL + EINK_LBL + EINK_LDL + 1); line++)
			*ptr++ = 0xE08000|0xff030303;
		for (line = (EINK_LSL + EINK_LBL + EINK_LDL + 1); line < EINK_LCD_W; line++)
			*ptr++ = 0xA08000|0xff030303;

		//frame: (EINK_FSL + EINK_FBL) -> (EINK_FSL + EINK_FBL + EINK_LDL - 1);
		for (frame = (EINK_FSL + EINK_FBL); frame < (EINK_FSL + EINK_FBL + EINK_FDL); frame++) {
			for (line = 0; line < EINK_LSL; line++)
				*ptr++ = 0xB0A000|0xff030303;
			for (line = EINK_LSL; line < (EINK_LSL + EINK_LBL); line++)
				*ptr++ = 0xB08000|0xff030303;
			for (line = (EINK_LSL + EINK_LBL); line < (EINK_LSL + EINK_LBL + 4); line++)
				*ptr++ = 0xD08000|0xff030303;
			for (line = (EINK_LSL + EINK_LBL + 4); line < (EINK_LSL + EINK_LBL + EINK_LDL + 1); line++)
				*ptr++ = 0xF08000|0xff030303;
			for (line = (EINK_LSL + EINK_LBL + EINK_LDL + 1); line < EINK_LCD_W; line++)
				*ptr++ = 0xB08000|0xff030303;
		}

		//frame: (EINK_FSL + EINK_FBL + EINK_LDL)
		for (line = 0; line < (EINK_LSL + EINK_LBL); line++)
			*ptr++ = 0xA08000|0xff030303;
		for (line = (EINK_LSL + EINK_LBL); line < (EINK_LSL + EINK_LBL + EINK_LDL + 1); line++)
			*ptr++ = 0xE08000|0xff030303;
		for (line = (EINK_LSL + EINK_LBL + EINK_LDL + 1); line < EINK_LCD_W; line++)
			*ptr++ = 0xA08000|0xff030303;
		frame++;

		//frame: (EINK_FSL + EINK_FBL + EINK_LDL +1) -> EINK_LCD_H 
		for (; frame < EINK_LCD_H; frame++) 
			for (line = 0; line < EINK_LCD_W; line++) 
				*ptr++ = 0xA08000|0xff030303;


	}
}


__s32 Disp_eink_init(__u32 sel, __eink_init_para* para)
{

	__disp_fb_create_para_t fb_create_para[1];
	unsigned long screen_base;
	__u32 xoffset, yoffset;
	__s32 i;

	memset((void*)(&disp_eink), 0, sizeof(Disp_eink_t));

	ADC_init();
	
printk("Disp_eink_init, apres l'init : para->width : %d, para->height = %d, para->pixelfmt : %d\n ",para->width,para->height,para->pixelfmt);

	EINK_PANEL_W = para->width;
	EINK_PANEL_H = para->height;

	EINK_LDL = (EINK_PANEL_W/4);
	EINK_FDL = EINK_PANEL_H;
	
	EINK_LCD_W  = (EINK_LDL+EINK_HYNC);
	EINK_LCD_H  = (EINK_FDL+EINK_VYNC);
	
	EINK_WF_WIDTH   = EINK_LCD_W;
	EINK_WF_HEIGHT = EINK_LCD_H;

	disp_eink.internal_update = 0;
	disp_eink.last_update_mode = EINK_INIT_MODE;

	disp_eink.frame_width = para->width;
	disp_eink.frame_height = para->height;
	disp_eink.frame_pixelfmt = para->pixelfmt;
	disp_eink.wav_width = disp_eink.frame_width / 4 + EINK_HYNC;
	disp_eink.wav_height = disp_eink.frame_height + EINK_VYNC;
	disp_eink.temperature = 26;

	sema_init(disp_eink.buffer_lock_sem, 1);
	eink_initBufferQueue();

	disp_eink.wav_last_frame = (__u8 *)vmalloc(disp_eink.frame_width * disp_eink.frame_height);
	if (disp_eink.wav_last_frame == NULL) {
		__EINK_ERR("Balloc memory for last frame buffer failed\n");
		return EPDK_FAIL;
	}
	memset(disp_eink.wav_last_frame, 0xFF, disp_eink.frame_width * disp_eink.frame_height);

	disp_eink.pframe_data  = (__u8 *)vmalloc(256*256);
        if(disp_eink.pframe_data == NULL)    {
                __EINK_ERR("malloc memory for pframe_data buffer failed\n");        
                return EPDK_FAIL;
        }

	disp_eink.wav_form_index = (__u8 *)vmalloc(disp_eink.frame_width * disp_eink.frame_height);
	if (disp_eink.wav_form_index == NULL) {
		__EINK_ERR("Balloc memory for last frame buffer failed\n");
		return EPDK_FAIL;
	}

	eink_init_fb_para(fb_create_para, disp_eink.wav_width, disp_eink.wav_height, EINK_WAV_BUFFER_NUM + 1);
	Drv_disp_Fb_Request(WF_FB_ID, fb_create_para);
	Drv_disp_get_fb_info(WF_FB_ID, &xoffset, &yoffset, &screen_base);
	Drv_disp_fb_layer_set_top(sel, WF_FB_ID);

	for (i = 0; i < EINK_WAV_BUFFER_NUM; i++) {
		disp_eink.wav_buffer[i].wav_address = (__u8 *)(screen_base + 4 * i * disp_eink.wav_width * disp_eink.wav_height);
		disp_eink.wav_buffer[i].wav_index = -1;
	}

	initGlobalCtrlData();           

	sema_init(disp_eink.wav_compose_sem, 0);
	sema_init(disp_eink.wav_refresh_sem, 0);

	disp_eink.wav_refresh_task = kthread_create(eink_refresh_thread, NULL, "eink refresh proc");
	if (IS_ERR(disp_eink.wav_refresh_task)) {
		__EINK_WRN("Unable to start kernel thread %s.\n", "eink update proc");
		return PTR_ERR(disp_eink.wav_refresh_task);
	}

	sema_init(disp_eink.stby_act_lock, 1);
	sema_init(disp_eink.power_off_sem, 0);
	wake_up_process(disp_eink.wav_refresh_task);

	sema_init(disp_eink.internal_update_sem, 0);
	disp_eink.internal_update_task = kthread_create(internal_update_thread, NULL, "internal update task");
	if (IS_ERR(disp_eink.internal_update_task)) {
		__EINK_WRN("Unable to start kernel thread %s.\n", "internal update task");
		return PTR_ERR(disp_eink.internal_update_task);
	}

	wake_up_process(disp_eink.internal_update_task);

	Disp_eink_get_sys_config(sel, &disp_eink);
	if(2 == disp_eink.eink_conn_version)	//2 = linear version
	{
		eink_gc16_update_waveform_awf = eink_gc16_update_waveform_awf_linear;
	}
	else 	//Default to scrambled version
	{
		eink_gc16_update_waveform_awf = eink_gc16_update_waveform_awf_scrambled;
	}


	Drv_disp_eink_panel_off(sel);

	return 0;
}

__s32 Disp_eink_exit(__u32 sel)
{
	if (disp_eink.wav_refresh_task) {
		kthread_stop(disp_eink.wav_refresh_task);
		disp_eink.wav_refresh_task = NULL;
	}

	if (disp_eink.wav_compose_task) {
		kthread_stop(disp_eink.wav_compose_task);
		disp_eink.wav_compose_task = NULL;
	}

	Drv_disp_Fb_Release(WF_FB_ID);
	Drv_disp_Fb_Release(SCALER_OUT_FB_ID);

	if (disp_eink.wav_form_index != NULL) {
		vfree(disp_eink.wav_form_index);
		disp_eink.wav_form_index = NULL;
	}

	if (disp_eink.wav_last_frame != NULL) {
		vfree(disp_eink.wav_last_frame);
		disp_eink.wav_last_frame = NULL;
	}

#ifdef AWF_WAVEFORM_SUPPORTED
	eink_unload_waveform((__u32*)disp_eink.p_wf_buffer);
#else
	eink_unload_waveform(disp_eink.wav_du_data);
	eink_unload_waveform(disp_eink.wav_gc16_data);
	eink_unload_waveform(disp_eink.wav_common_data);
	eink_unload_waveform(disp_eink.wav_a2_data);
	eink_unload_waveform(disp_eink.wav_a2_in_data);
	eink_unload_waveform(disp_eink.wav_a2_out_data);
#endif
	eink_unload_waveform(disp_eink.wav_short_du_data);
	eink_unload_waveform(disp_eink.wav_short_gc16_local_data);

	Drv_disp_eink_panel_off(sel);

	return 0;
}

static int set_vcom()
{
	struct file *fp;
	bool setVcomValue = false;
	int vcom = 0;
	char buffer[5];
	mm_segment_t fs1;
	loff_t pos1;

	fp = filp_open(EINK_VCOM_PATH, O_RDONLY, 0);
	if (IS_ERR(fp)) {
		printk("open file error\n");
		return tps65185_vcom_set(-2000);
	}

	fs1 = get_fs();
	set_fs(KERNEL_DS);
	pos1 = 0;

	vfs_read(fp, buffer, sizeof(buffer), &pos1);
	buffer[4] = '\0';
	printk("read_vcom: %s\n", buffer);
	filp_close(fp, NULL);

	set_fs(fs1);

	if (strlen(buffer) == 0)
		return tps65185_vcom_set(-2000);

	vcom = simple_strtoul(buffer, NULL, 10);
	printk("change_vcom: %d\n", vcom);

	return tps65185_vcom_set(-vcom);
}

__s32 Disp_eink_update(__u32 sel, __u32 fb_id, __u32 mode, __u32 coordinate[])
{
	int ret = 0;
	__u8 *in_buffer = NULL;
	__u8 *out_buffer = NULL;
	__u32 xres, yres, bpp;
	__u32 x_start 	= coordinate[0];
	__u32 x_end 	= coordinate[1];
	__u32 y_start 	= coordinate[2];
	__u32 y_end 	= coordinate[3];

	cpufreq_driver_target(cpufreq_cpu_get(0), 1008000, 0);

	disp_eink.temperature = ADC_Get_temperature();

	/* disable the internal update task */
	disp_eink.internal_update = 0;

	if (mode == EINK_INIT_MODE)
		goto exit;

	Drv_disp_get_fb_size(0, &xres, &yres);
	bpp = Drv_disp_get_fb_bpp(0);

	in_buffer = eink_get32BppBuffer();
	ret = eink_get8BppBuffer(mode, coordinate, &out_buffer);
	if (ret < 0)
		goto exit;

	if (bpp != 8)
		eink_32To8Bpp(in_buffer, out_buffer);
	else
		memcpy(out_buffer, in_buffer, xres * yres);

	if (mode & EINK_DITHERING_MODE) {
		if (mode & EINK_GC16_MODE)
			eink_set_8bit_dithering((__u8 *)out_buffer);
		else
			eink_set_dithering((__u8 *)out_buffer, x_start, x_end, y_start, y_end);
	} else if (mode & (EINK_A2_MODE | EINK_A2_IN_MODE)) {
		eink_set1b_color((__u8 *)out_buffer, x_start, x_end, y_start, y_end);
	}

#ifdef AWF_WAVEFORM_SUPPORTED
	if (disp_eink.p_wf_buffer == NULL) {
		set_vcom();
		eink_load_awf_waveform(EINK_AWF_WAV_PATH);
	}
#else
	eink_loadWaveFormData();
#endif

exit:
	up(disp_eink.wav_compose_sem);
	return ret;
}

__s32 Disp_eink_set_mode(__u32 sel, __eink_update_mode mode)
{
	eink_setCurBufFlushMode(mode);

	return 0;
}

__s32 Disp_eink_set_temperature(__u32 sel, __u32 temperature)
{
	disp_eink.temperature = temperature;

	return 0;
}

void Disp_eink_set_stby_act(Eink_stby_act_e act)
{
	disp_eink.stby_act = act;
}

Eink_stby_act_e Disp_eink_get_stby_act(void)
{
	return disp_eink.stby_act;
}

__s32 Disp_eink_get_update_status(__u32 sel)
{
	return disp_eink.update_status;
}

__s32 Disp_eink_set_3v3(void)
{
	return tps65185_v3p3_set();

}

void Disp_eink_stby_act_lock(void)
{
	down(disp_eink.stby_act_lock);
}

void Disp_eink_stby_act_unlock(void)
{
	up(disp_eink.stby_act_lock);
}
