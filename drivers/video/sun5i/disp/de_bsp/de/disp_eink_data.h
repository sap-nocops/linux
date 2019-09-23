#ifndef __DISP_EINK_DATA_H__
#define __DISP_EINK_DATA_H__

//#include "disp_display_i.h"

#define         ED060SC4                0x01
#define         ED060SC7                0x02
#define         OPM060A1                0x03

#define         EINK_MODULE_TYPE        OPM060A1


/*
#if defined CONFIG_FB_SUNXI_800_600

	#define         EINK_PANEL_W            800
	#define         EINK_PANEL_H            600

#elif defined CONFIG_FB_SUNXI_1024_758

	#define         EINK_PANEL_W            1024
	#define         EINK_PANEL_H            758

#elif defined CONFIG_FB_SUNXI_1024_768

	#define         EINK_PANEL_W            1024
	#define         EINK_PANEL_H            768

#elif defined CONFIG_FB_SUNXI_1448_1072

	#define         EINK_PANEL_W            1448
	#define         EINK_PANEL_H            1072

#else

//Default resolution : 1024 x 758
	#define         EINK_PANEL_W            1024
	#define         EINK_PANEL_H            758
#endif
*/

#define MAX_EINK_PANEL_W	1448
#define MAX_EINK_PANEL_H	1072

#define         EINK_BLANK              0 


#define         EINK_LSL                10
#define         EINK_LBL                4
//#define         EINK_LDL                (EINK_PANEL_W/4)             //200*4 = 800,
#define         EINK_LEL                44
#define         EINK_HYNC               (EINK_LSL+EINK_LBL+EINK_LEL)        //50

#define         EINK_FSL                5
#define         EINK_FBL                3
//#define         EINK_FDL                EINK_PANEL_H             //600
#define         EINK_FEL                12
#define         EINK_VYNC               (EINK_FSL+EINK_FBL+EINK_FEL)        //20

//#define         EINK_LCD_W              (EINK_LDL+EINK_HYNC)
//#define         EINK_LCD_H              (EINK_FDL+EINK_VYNC)

//#define         EINK_WF_WIDTH           EINK_LCD_W
//#define         EINK_WF_HEIGHT          EINK_LCD_H

#define         EINK_PANEL_SCAN_RL        0x00000004
#define         EINK_PANEL_SCAN_UD        0x00004000
#define         EINK_PANEL_VGG_EN         0x00040000
#define         EINK_PANEL_VCOM_EN        0x00080000 

#define         COL_MAX           128
#define         ROW_MAX           256                     //16*16 = 256

#define         DU_ROW_MAX        32                    

#define         T_MAX             50                   


typedef struct __EINK_INIT_WF_S
{
    unsigned short total_frame;
    unsigned char wf_data[COL_MAX];
}eink_init_wf_t; 

typedef struct __EINK_DU_WF_S
{
    unsigned short total_frame; 
    unsigned char wf_data[DU_ROW_MAX][COL_MAX];
}eink_du_wf_t;

typedef struct __EINK_WF_DATA_S
{
    unsigned short total_frame;
    unsigned char wf_data[ROW_MAX][COL_MAX];
}eink_wf_data_t;

extern  eink_wf_data_t eink_gc16_mode_wf[14];

extern const unsigned int eink_ctrl_line_index_tbl[MAX_EINK_PANEL_H+EINK_FSL+EINK_FBL+EINK_FEL+20];
extern const unsigned int eink_ctrl_tbl_GC16_COMMON[8][MAX_EINK_PANEL_W];

extern const unsigned char eink_init_T_tbl_OED[T_MAX];
extern const eink_init_wf_t eink_init_mode_wf_OED[15];
extern const unsigned char eink_A2_mode_wf_OED[3][10];

extern const unsigned char eink_init_T_tbl_PVI[T_MAX];
extern const eink_init_wf_t eink_init_mode_wf_PVI[15];
extern const unsigned char eink_A2_mode_wf_PVI[3][10];


#endif
