#define AC100_CHIP_AUDIO_RST_DEVICE_VERSION_OFF                 0
#define AC100_CHIP_AUDIO_RST_DEVICE_VERSION(v)                  ((v) & 0xffff)

#define AC100_PLL_CTRL1_DPLL_DAC_BIAS_OFF                       14
#define AC100_PLL_CTRL1_DPLL_DAC_BIAS(v)                        (((v) & 0x3) << 14)
#define AC100_PLL_CTRL1_PLL_POSTDIV_M_OFF                       8
#define AC100_PLL_CTRL1_PLL_POSTDIV_M(v)                        (((v) & 0x3f) << 8)
#define AC100_PLL_CTRL1_CLOSE_LOOP_OFF                          6
#define AC100_PLL_CTRL1_CLOSE_LOOP_MASK                         BIT(6)
#define AC100_PLL_CTRL1_CLOSE_LOOP_NO_VCO                       0
#define AC100_PLL_CTRL1_CLOSE_LOOP_YES_PLL                      BIT(6)
#define AC100_PLL_CTRL1_LOOP_BANDWIDTH_OFF                      0
#define AC100_PLL_CTRL1_LOOP_BANDWIDTH(v)                       ((v) & 0x3f)

#define AC100_PLL_CTRL2_EN_OFF                                  15
#define AC100_PLL_CTRL2_EN_MASK                                 BIT(15)
#define AC100_PLL_CTRL2_EN_NO                                   0
#define AC100_PLL_CTRL2_EN_YES                                  BIT(15)
#define AC100_PLL_CTRL2_LOCKED_OFF                              14
#define AC100_PLL_CTRL2_LOCKED_MASK                             BIT(14)
#define AC100_PLL_CTRL2_LOCKED_NO                               0
#define AC100_PLL_CTRL2_LOCKED_YES                              BIT(14)
#define AC100_PLL_CTRL2_PREDIV_NI_OFF                           4
#define AC100_PLL_CTRL2_PREDIV_NI(v)                            (((v) & 0x3ff) << 4)
#define AC100_PLL_CTRL2_POSTDIV_NF_OFF                          0
#define AC100_PLL_CTRL2_POSTDIV_NF(v)                           ((v) & 0x7)

#define AC100_SYSCLK_CTRL_PLLCLK_ENA_OFF                        15
#define AC100_SYSCLK_CTRL_PLLCLK_ENA_MASK                       BIT(15)
#define AC100_SYSCLK_CTRL_PLLCLK_ENA_DISABLED                   0
#define AC100_SYSCLK_CTRL_PLLCLK_ENA_ENABLED                    BIT(15)
#define AC100_SYSCLK_CTRL_PLLCLK_SRC_OFF                        12
#define AC100_SYSCLK_CTRL_PLLCLK_SRC_MASK                       GENMASK(13, 12)
#define AC100_SYSCLK_CTRL_PLLCLK_SRC_MCLK1                      (0x0 << 12)
#define AC100_SYSCLK_CTRL_PLLCLK_SRC_MCLK2                      (0x1 << 12)
#define AC100_SYSCLK_CTRL_PLLCLK_SRC_BCLK1                      (0x2 << 12)
#define AC100_SYSCLK_CTRL_PLLCLK_SRC_BCLK2                      (0x3 << 12)
#define AC100_SYSCLK_CTRL_I2S1CLK_ENA_OFF                       11
#define AC100_SYSCLK_CTRL_I2S1CLK_ENA_MASK                      BIT(11)
#define AC100_SYSCLK_CTRL_I2S1CLK_ENA_DISABLED                  0
#define AC100_SYSCLK_CTRL_I2S1CLK_ENA_ENABLED                   BIT(11)
#define AC100_SYSCLK_CTRL_I2S1CLK_SRC_OFF                       8
#define AC100_SYSCLK_CTRL_I2S1CLK_SRC_MASK                      GENMASK(9, 8)
#define AC100_SYSCLK_CTRL_I2S1CLK_SRC_MCLK1                     (0x0 << 8)
#define AC100_SYSCLK_CTRL_I2S1CLK_SRC_MCLK2                     (0x1 << 8)
#define AC100_SYSCLK_CTRL_I2S1CLK_SRC_PLL                       (0x2 << 8)
#define AC100_SYSCLK_CTRL_I2S2CLK_ENA_OFF                       7
#define AC100_SYSCLK_CTRL_I2S2CLK_ENA_MASK                      BIT(7)
#define AC100_SYSCLK_CTRL_I2S2CLK_ENA_DISABLED                  0
#define AC100_SYSCLK_CTRL_I2S2CLK_ENA_ENABLED                   BIT(7)
#define AC100_SYSCLK_CTRL_I2S2CLK_SRC_OFF                       4
#define AC100_SYSCLK_CTRL_I2S2CLK_SRC_MASK                      GENMASK(5, 4)
#define AC100_SYSCLK_CTRL_I2S2CLK_SRC_MCLK1                     (0x0 << 4)
#define AC100_SYSCLK_CTRL_I2S2CLK_SRC_MCLK2                     (0x1 << 4)
#define AC100_SYSCLK_CTRL_I2S2CLK_SRC_PLL                       (0x2 << 4)
#define AC100_SYSCLK_CTRL_SYSCLK_ENA_OFF                        3
#define AC100_SYSCLK_CTRL_SYSCLK_ENA_MASK                       BIT(3)
#define AC100_SYSCLK_CTRL_SYSCLK_ENA_DISABLED                   0
#define AC100_SYSCLK_CTRL_SYSCLK_ENA_ENABLED                    BIT(3)
#define AC100_SYSCLK_CTRL_SYSCLK_SRC_OFF                        0
#define AC100_SYSCLK_CTRL_SYSCLK_SRC_MASK                       BIT(0)
#define AC100_SYSCLK_CTRL_SYSCLK_SRC_I2S1CLK                    0
#define AC100_SYSCLK_CTRL_SYSCLK_SRC_I2S2CLK                    BIT(0)

#define AC100_MOD_CLK_ENA_I2S1_OFF                              15
#define AC100_MOD_CLK_ENA_I2S1_MASK                             BIT(15)
#define AC100_MOD_CLK_ENA_I2S1_DISABLED                         0
#define AC100_MOD_CLK_ENA_I2S1_ENABLED                          BIT(15)
#define AC100_MOD_CLK_ENA_I2S2_OFF                              14
#define AC100_MOD_CLK_ENA_I2S2_MASK                             BIT(14)
#define AC100_MOD_CLK_ENA_I2S2_DISABLED                         0
#define AC100_MOD_CLK_ENA_I2S2_ENABLED                          BIT(14)
#define AC100_MOD_CLK_ENA_I2S3_OFF                              13
#define AC100_MOD_CLK_ENA_I2S3_MASK                             BIT(13)
#define AC100_MOD_CLK_ENA_I2S3_DISABLED                         0
#define AC100_MOD_CLK_ENA_I2S3_ENABLED                          BIT(13)
#define AC100_MOD_CLK_ENA_SRC1_OFF                              11
#define AC100_MOD_CLK_ENA_SRC1_MASK                             BIT(11)
#define AC100_MOD_CLK_ENA_SRC1_DISABLED                         0
#define AC100_MOD_CLK_ENA_SRC1_ENABLED                          BIT(11)
#define AC100_MOD_CLK_ENA_SRC2_OFF                              10
#define AC100_MOD_CLK_ENA_SRC2_MASK                             BIT(10)
#define AC100_MOD_CLK_ENA_SRC2_DISABLED                         0
#define AC100_MOD_CLK_ENA_SRC2_ENABLED                          BIT(10)
#define AC100_MOD_CLK_ENA_HPF_AGC_OFF                           7
#define AC100_MOD_CLK_ENA_HPF_AGC_MASK                          BIT(7)
#define AC100_MOD_CLK_ENA_HPF_AGC_DISABLED                      0
#define AC100_MOD_CLK_ENA_HPF_AGC_ENABLED                       BIT(7)
#define AC100_MOD_CLK_ENA_HPF_DRC_OFF                           6
#define AC100_MOD_CLK_ENA_HPF_DRC_MASK                          BIT(6)
#define AC100_MOD_CLK_ENA_HPF_DRC_DISABLED                      0
#define AC100_MOD_CLK_ENA_HPF_DRC_ENABLED                       BIT(6)
#define AC100_MOD_CLK_ENA_ADC_DIGITAL_OFF                       3
#define AC100_MOD_CLK_ENA_ADC_DIGITAL_MASK                      BIT(3)
#define AC100_MOD_CLK_ENA_ADC_DIGITAL_DISABLED                  0
#define AC100_MOD_CLK_ENA_ADC_DIGITAL_ENABLED                   BIT(3)
#define AC100_MOD_CLK_ENA_DAC_DIGITAL_OFF                       2
#define AC100_MOD_CLK_ENA_DAC_DIGITAL_MASK                      BIT(2)
#define AC100_MOD_CLK_ENA_DAC_DIGITAL_DISABLED                  0
#define AC100_MOD_CLK_ENA_DAC_DIGITAL_ENABLED                   BIT(2)

#define AC100_MOD_RST_CTRL_I2S1_OFF                             15
#define AC100_MOD_RST_CTRL_I2S1_MASK                            BIT(15)
#define AC100_MOD_RST_CTRL_I2S1_ASSERTED                        0
#define AC100_MOD_RST_CTRL_I2S1_DEASSERTED                      BIT(15)
#define AC100_MOD_RST_CTRL_I2S2_OFF                             14
#define AC100_MOD_RST_CTRL_I2S2_MASK                            BIT(14)
#define AC100_MOD_RST_CTRL_I2S2_ASSERTED                        0
#define AC100_MOD_RST_CTRL_I2S2_DEASSERTED                      BIT(14)
#define AC100_MOD_RST_CTRL_I2S3_OFF                             13
#define AC100_MOD_RST_CTRL_I2S3_MASK                            BIT(13)
#define AC100_MOD_RST_CTRL_I2S3_ASSERTED                        0
#define AC100_MOD_RST_CTRL_I2S3_DEASSERTED                      BIT(13)
#define AC100_MOD_RST_CTRL_SRC1_OFF                             11
#define AC100_MOD_RST_CTRL_SRC1_MASK                            BIT(11)
#define AC100_MOD_RST_CTRL_SRC1_ASSERTED                        0
#define AC100_MOD_RST_CTRL_SRC1_DEASSERTED                      BIT(11)
#define AC100_MOD_RST_CTRL_SRC2_OFF                             10
#define AC100_MOD_RST_CTRL_SRC2_MASK                            BIT(10)
#define AC100_MOD_RST_CTRL_SRC2_ASSERTED                        0
#define AC100_MOD_RST_CTRL_SRC2_DEASSERTED                      BIT(10)
#define AC100_MOD_RST_CTRL_HPF_AGC_OFF                          7
#define AC100_MOD_RST_CTRL_HPF_AGC_MASK                         BIT(7)
#define AC100_MOD_RST_CTRL_HPF_AGC_ASSERTED                     0
#define AC100_MOD_RST_CTRL_HPF_AGC_DEASSERTED                   BIT(7)
#define AC100_MOD_RST_CTRL_HPF_DRC_OFF                          6
#define AC100_MOD_RST_CTRL_HPF_DRC_MASK                         BIT(6)
#define AC100_MOD_RST_CTRL_HPF_DRC_ASSERTED                     0
#define AC100_MOD_RST_CTRL_HPF_DRC_DEASSERTED                   BIT(6)
#define AC100_MOD_RST_CTRL_ADC_DIGITAL_OFF                      3
#define AC100_MOD_RST_CTRL_ADC_DIGITAL_MASK                     BIT(3)
#define AC100_MOD_RST_CTRL_ADC_DIGITAL_ASSERTED                 0
#define AC100_MOD_RST_CTRL_ADC_DIGITAL_DEASSERTED               BIT(3)
#define AC100_MOD_RST_CTRL_DAC_DIGITAL_OFF                      2
#define AC100_MOD_RST_CTRL_DAC_DIGITAL_MASK                     BIT(2)
#define AC100_MOD_RST_CTRL_DAC_DIGITAL_ASSERTED                 0
#define AC100_MOD_RST_CTRL_DAC_DIGITAL_DEASSERTED               BIT(2)

#define AC100_I2S_SR_CTRL_FS_I2S1_OFF                           12
#define AC100_I2S_SR_CTRL_FS_I2S1_MASK                          GENMASK(15, 12)
#define AC100_I2S_SR_CTRL_FS_I2S1_8KHZ                          (0x0 << 12)
#define AC100_I2S_SR_CTRL_FS_I2S1_11_025KHZ                     (0x1 << 12)
#define AC100_I2S_SR_CTRL_FS_I2S1_12KHZ                         (0x2 << 12)
#define AC100_I2S_SR_CTRL_FS_I2S1_16KHZ                         (0x3 << 12)
#define AC100_I2S_SR_CTRL_FS_I2S1_22_05KHZ                      (0x4 << 12)
#define AC100_I2S_SR_CTRL_FS_I2S1_24KHZ                         (0x5 << 12)
#define AC100_I2S_SR_CTRL_FS_I2S1_32KHZ                         (0x6 << 12)
#define AC100_I2S_SR_CTRL_FS_I2S1_44_1KHZ                       (0x7 << 12)
#define AC100_I2S_SR_CTRL_FS_I2S1_48KHZ                         (0x8 << 12)
#define AC100_I2S_SR_CTRL_FS_I2S1_96KHZ                         (0x9 << 12)
#define AC100_I2S_SR_CTRL_FS_I2S1_192KHZ                        (0xa << 12)
#define AC100_I2S_SR_CTRL_FS_I2S2_OFF                           8
#define AC100_I2S_SR_CTRL_FS_I2S2_MASK                          GENMASK(11, 8)
#define AC100_I2S_SR_CTRL_FS_I2S2_8KHZ                          (0x0 << 8)
#define AC100_I2S_SR_CTRL_FS_I2S2_11_025KHZ                     (0x1 << 8)
#define AC100_I2S_SR_CTRL_FS_I2S2_12KHZ                         (0x2 << 8)
#define AC100_I2S_SR_CTRL_FS_I2S2_16KHZ                         (0x3 << 8)
#define AC100_I2S_SR_CTRL_FS_I2S2_22_05KHZ                      (0x4 << 8)
#define AC100_I2S_SR_CTRL_FS_I2S2_24KHZ                         (0x5 << 8)
#define AC100_I2S_SR_CTRL_FS_I2S2_32KHZ                         (0x6 << 8)
#define AC100_I2S_SR_CTRL_FS_I2S2_44_1KHZ                       (0x7 << 8)
#define AC100_I2S_SR_CTRL_FS_I2S2_48KHZ                         (0x8 << 8)
#define AC100_I2S_SR_CTRL_FS_I2S2_96KHZ                         (0x9 << 8)
#define AC100_I2S_SR_CTRL_FS_I2S2_192KHZ                        (0xa << 8)
#define AC100_I2S_SR_CTRL_SRC1_ENA_OFF                          3
#define AC100_I2S_SR_CTRL_SRC1_ENA_MASK                         BIT(3)
#define AC100_I2S_SR_CTRL_SRC1_ENA_DISABLED                     0
#define AC100_I2S_SR_CTRL_SRC1_ENA_ENABLED                      BIT(3)
#define AC100_I2S_SR_CTRL_SRC1_SRC_OFF                          2
#define AC100_I2S_SR_CTRL_SRC1_SRC_MASK                         BIT(2)
#define AC100_I2S_SR_CTRL_SRC1_SRC_I2S1_DAC_TS0                 0
#define AC100_I2S_SR_CTRL_SRC1_SRC_I2S2_DAC                     BIT(2)
#define AC100_I2S_SR_CTRL_SRC2_ENA_OFF                          1
#define AC100_I2S_SR_CTRL_SRC2_ENA_MASK                         BIT(1)
#define AC100_I2S_SR_CTRL_SRC2_ENA_DISABLED                     0
#define AC100_I2S_SR_CTRL_SRC2_ENA_ENABLED                      BIT(1)
#define AC100_I2S_SR_CTRL_SRC2_SRC_OFF                          0
#define AC100_I2S_SR_CTRL_SRC2_SRC_MASK                         BIT(0)
#define AC100_I2S_SR_CTRL_SRC2_SRC_I2S1_ADC_TS0                 0
#define AC100_I2S_SR_CTRL_SRC2_SRC_I2S2_ADC                     BIT(0)

#define AC100_I2S1_CLK_CTRL_MSTR_MOD_OFF                        15
#define AC100_I2S1_CLK_CTRL_MSTR_MOD_MASK                       BIT(15)
#define AC100_I2S1_CLK_CTRL_MSTR_MOD_MASTER                     0
#define AC100_I2S1_CLK_CTRL_MSTR_MOD_SLAVE                      BIT(15)
#define AC100_I2S1_CLK_CTRL_BCLK_INV_OFF                        14
#define AC100_I2S1_CLK_CTRL_BCLK_INV_MASK                       BIT(14)
#define AC100_I2S1_CLK_CTRL_BCLK_INV_NORMAL                     0
#define AC100_I2S1_CLK_CTRL_BCLK_INV_INVERTED                   BIT(14)
#define AC100_I2S1_CLK_CTRL_LRCK_INV_OFF                        13
#define AC100_I2S1_CLK_CTRL_LRCK_INV_MASK                       BIT(13)
#define AC100_I2S1_CLK_CTRL_LRCK_INV_NORMAL                     0
#define AC100_I2S1_CLK_CTRL_LRCK_INV_INVERTED                   BIT(13)
#define AC100_I2S1_CLK_CTRL_BCLK_DIV_OFF                        9
#define AC100_I2S1_CLK_CTRL_BCLK_DIV_MASK                       GENMASK(12, 9)
#define AC100_I2S1_CLK_CTRL_BCLK_DIV_1                          (0x0 << 9)
#define AC100_I2S1_CLK_CTRL_BCLK_DIV_2                          (0x1 << 9)
#define AC100_I2S1_CLK_CTRL_BCLK_DIV_4                          (0x2 << 9)
#define AC100_I2S1_CLK_CTRL_BCLK_DIV_6                          (0x3 << 9)
#define AC100_I2S1_CLK_CTRL_BCLK_DIV_8                          (0x4 << 9)
#define AC100_I2S1_CLK_CTRL_BCLK_DIV_12                         (0x5 << 9)
#define AC100_I2S1_CLK_CTRL_BCLK_DIV_16                         (0x6 << 9)
#define AC100_I2S1_CLK_CTRL_BCLK_DIV_24                         (0x7 << 9)
#define AC100_I2S1_CLK_CTRL_BCLK_DIV_32                         (0x8 << 9)
#define AC100_I2S1_CLK_CTRL_BCLK_DIV_48                         (0x9 << 9)
#define AC100_I2S1_CLK_CTRL_BCLK_DIV_64                         (0xa << 9)
#define AC100_I2S1_CLK_CTRL_BCLK_DIV_96                         (0xb << 9)
#define AC100_I2S1_CLK_CTRL_BCLK_DIV_128                        (0xc << 9)
#define AC100_I2S1_CLK_CTRL_BCLK_DIV_192                        (0xd << 9)
#define AC100_I2S1_CLK_CTRL_LRCK_DIV_OFF                        6
#define AC100_I2S1_CLK_CTRL_LRCK_DIV_MASK                       GENMASK(8, 6)
#define AC100_I2S1_CLK_CTRL_LRCK_DIV_16                         (0x0 << 6)
#define AC100_I2S1_CLK_CTRL_LRCK_DIV_32                         (0x1 << 6)
#define AC100_I2S1_CLK_CTRL_LRCK_DIV_64                         (0x2 << 6)
#define AC100_I2S1_CLK_CTRL_LRCK_DIV_128                        (0x3 << 6)
#define AC100_I2S1_CLK_CTRL_LRCK_DIV_256                        (0x4 << 6)
#define AC100_I2S1_CLK_CTRL_WORD_SIZE_OFF                       4
#define AC100_I2S1_CLK_CTRL_WORD_SIZE_MASK                      GENMASK(5, 4)
#define AC100_I2S1_CLK_CTRL_WORD_SIZE_8BIT                      (0x0 << 4)
#define AC100_I2S1_CLK_CTRL_WORD_SIZE_16BIT                     (0x1 << 4)
#define AC100_I2S1_CLK_CTRL_WORD_SIZE_20BIT                     (0x2 << 4)
#define AC100_I2S1_CLK_CTRL_WORD_SIZE_24BIT                     (0x3 << 4)
#define AC100_I2S1_CLK_CTRL_DATA_FMT_OFF                        2
#define AC100_I2S1_CLK_CTRL_DATA_FMT_MASK                       GENMASK(3, 2)
#define AC100_I2S1_CLK_CTRL_DATA_FMT_I2S_MODE                   (0x0 << 2)
#define AC100_I2S1_CLK_CTRL_DATA_FMT_LEFT_MODE                  (0x1 << 2)
#define AC100_I2S1_CLK_CTRL_DATA_FMT_RIGHT_MODE                 (0x2 << 2)
#define AC100_I2S1_CLK_CTRL_DATA_FMT_DSP_MODE                   (0x3 << 2)
#define AC100_I2S1_CLK_CTRL_DSP_MONO_PCM_OFF                    1
#define AC100_I2S1_CLK_CTRL_DSP_MONO_PCM_MASK                   BIT(1)
#define AC100_I2S1_CLK_CTRL_DSP_MONO_PCM_STEREO                 0
#define AC100_I2S1_CLK_CTRL_DSP_MONO_PCM_MONO                   BIT(1)
#define AC100_I2S1_CLK_CTRL_TDMM_ENA_OFF                        0
#define AC100_I2S1_CLK_CTRL_TDMM_ENA_MASK                       BIT(0)
#define AC100_I2S1_CLK_CTRL_TDMM_ENA_DISABLED                   0
#define AC100_I2S1_CLK_CTRL_TDMM_ENA_ENABLED                    BIT(0)

#define AC100_I2S1_SND_OUT_CTRL_ADCL0_EN_OFF                    15
#define AC100_I2S1_SND_OUT_CTRL_ADCL0_EN_MASK                   BIT(15)
#define AC100_I2S1_SND_OUT_CTRL_ADCL0_EN_DISABLED               0
#define AC100_I2S1_SND_OUT_CTRL_ADCL0_EN_ENABLED                BIT(15)
#define AC100_I2S1_SND_OUT_CTRL_ADCR0_EN_OFF                    14
#define AC100_I2S1_SND_OUT_CTRL_ADCR0_EN_MASK                   BIT(14)
#define AC100_I2S1_SND_OUT_CTRL_ADCR0_EN_DISABLED               0
#define AC100_I2S1_SND_OUT_CTRL_ADCR0_EN_ENABLED                BIT(14)
#define AC100_I2S1_SND_OUT_CTRL_ADCL1_EN_OFF                    13
#define AC100_I2S1_SND_OUT_CTRL_ADCL1_EN_MASK                   BIT(13)
#define AC100_I2S1_SND_OUT_CTRL_ADCL1_EN_DISABLED               0
#define AC100_I2S1_SND_OUT_CTRL_ADCL1_EN_ENABLED                BIT(13)
#define AC100_I2S1_SND_OUT_CTRL_ADCR1_EN_OFF                    12
#define AC100_I2S1_SND_OUT_CTRL_ADCR1_EN_MASK                   BIT(12)
#define AC100_I2S1_SND_OUT_CTRL_ADCR1_EN_DISABLED               0
#define AC100_I2S1_SND_OUT_CTRL_ADCR1_EN_ENABLED                BIT(12)
#define AC100_I2S1_SND_OUT_CTRL_ADCL0_SRC_OFF                   10
#define AC100_I2S1_SND_OUT_CTRL_ADCL0_SRC_MASK                  GENMASK(11, 10)
#define AC100_I2S1_SND_OUT_CTRL_ADCL0_SRC_ADCL0                 (0x0 << 10)
#define AC100_I2S1_SND_OUT_CTRL_ADCL0_SRC_ADCR0                 (0x1 << 10)
#define AC100_I2S1_SND_OUT_CTRL_ADCL0_SRC_ADCL0_PLUS_ADCR0      (0x2 << 10)
#define AC100_I2S1_SND_OUT_CTRL_ADCL0_SRC_ADCL0_PLUS_ADCR0_DIV_2 (0x3 << 10)
#define AC100_I2S1_SND_OUT_CTRL_ADCR0_SRC_OFF                   8
#define AC100_I2S1_SND_OUT_CTRL_ADCR0_SRC_MASK                  GENMASK(9, 8)
#define AC100_I2S1_SND_OUT_CTRL_ADCR0_SRC_ADCR0                 (0x0 << 8)
#define AC100_I2S1_SND_OUT_CTRL_ADCR0_SRC_ADCL0                 (0x1 << 8)
#define AC100_I2S1_SND_OUT_CTRL_ADCR0_SRC_ADCL0_PLUS_ADCR0      (0x2 << 8)
#define AC100_I2S1_SND_OUT_CTRL_ADCR0_SRC_ADCL0_PLUS_ADCR0_DIV_2 (0x3 << 8)
#define AC100_I2S1_SND_OUT_CTRL_ADCL1_SRC_OFF                   6
#define AC100_I2S1_SND_OUT_CTRL_ADCL1_SRC_MASK                  GENMASK(7, 6)
#define AC100_I2S1_SND_OUT_CTRL_ADCL1_SRC_ADCL1                 (0x0 << 6)
#define AC100_I2S1_SND_OUT_CTRL_ADCL1_SRC_ADCR1                 (0x1 << 6)
#define AC100_I2S1_SND_OUT_CTRL_ADCL1_SRC_ADCL1_PLUS_ADCR1      (0x2 << 6)
#define AC100_I2S1_SND_OUT_CTRL_ADCL1_SRC_ADCL1_PLUS_ADCR1_DIV_2 (0x3 << 6)
#define AC100_I2S1_SND_OUT_CTRL_ADCR1_SRC_OFF                   4
#define AC100_I2S1_SND_OUT_CTRL_ADCR1_SRC_MASK                  GENMASK(5, 4)
#define AC100_I2S1_SND_OUT_CTRL_ADCR1_SRC_ADCR1                 (0x0 << 4)
#define AC100_I2S1_SND_OUT_CTRL_ADCR1_SRC_ADCL1                 (0x1 << 4)
#define AC100_I2S1_SND_OUT_CTRL_ADCR1_SRC_ADCL1_PLUS_ADCR1      (0x2 << 4)
#define AC100_I2S1_SND_OUT_CTRL_ADCR1_SRC_ADCL1_PLUS_ADCR1_DIV_2 (0x3 << 4)
#define AC100_I2S1_SND_OUT_CTRL_ADCP_EN_OFF                     3
#define AC100_I2S1_SND_OUT_CTRL_ADCP_EN_MASK                    BIT(3)
#define AC100_I2S1_SND_OUT_CTRL_ADCP_EN_DISABLED                0
#define AC100_I2S1_SND_OUT_CTRL_ADCP_EN_ENABLED                 BIT(3)
#define AC100_I2S1_SND_OUT_CTRL_ADCP_SEL_OFF                    2
#define AC100_I2S1_SND_OUT_CTRL_ADCP_SEL_MASK                   BIT(2)
#define AC100_I2S1_SND_OUT_CTRL_ADCP_SEL_A_LAW                  0
#define AC100_I2S1_SND_OUT_CTRL_ADCP_SEL_U_LAW                  BIT(2)
#define AC100_I2S1_SND_OUT_CTRL_SLOT_SIZE_OFF                   0
#define AC100_I2S1_SND_OUT_CTRL_SLOT_SIZE_MASK                  GENMASK(1, 0)
#define AC100_I2S1_SND_OUT_CTRL_SLOT_SIZE_8                     0x0
#define AC100_I2S1_SND_OUT_CTRL_SLOT_SIZE_16                    0x1
#define AC100_I2S1_SND_OUT_CTRL_SLOT_SIZE_32                    0x2

#define AC100_I2S1_SND_IN_CTRL_DACL0_EN_OFF                     15
#define AC100_I2S1_SND_IN_CTRL_DACL0_EN_MASK                    BIT(15)
#define AC100_I2S1_SND_IN_CTRL_DACL0_EN_DISABLED                0
#define AC100_I2S1_SND_IN_CTRL_DACL0_EN_ENABLED                 BIT(15)
#define AC100_I2S1_SND_IN_CTRL_DACR0_EN_OFF                     14
#define AC100_I2S1_SND_IN_CTRL_DACR0_EN_MASK                    BIT(14)
#define AC100_I2S1_SND_IN_CTRL_DACR0_EN_DISABLED                0
#define AC100_I2S1_SND_IN_CTRL_DACR0_EN_ENABLED                 BIT(14)
#define AC100_I2S1_SND_IN_CTRL_DACL1_EN_OFF                     13
#define AC100_I2S1_SND_IN_CTRL_DACL1_EN_MASK                    BIT(13)
#define AC100_I2S1_SND_IN_CTRL_DACL1_EN_DISABLED                0
#define AC100_I2S1_SND_IN_CTRL_DACL1_EN_ENABLED                 BIT(13)
#define AC100_I2S1_SND_IN_CTRL_DACR1_EN_OFF                     12
#define AC100_I2S1_SND_IN_CTRL_DACR1_EN_MASK                    BIT(12)
#define AC100_I2S1_SND_IN_CTRL_DACR1_EN_DISABLED                0
#define AC100_I2S1_SND_IN_CTRL_DACR1_EN_ENABLED                 BIT(12)
#define AC100_I2S1_SND_IN_CTRL_DACL0_SRC_OFF                    10
#define AC100_I2S1_SND_IN_CTRL_DACL0_SRC_MASK                   GENMASK(11, 10)
#define AC100_I2S1_SND_IN_CTRL_DACL0_SRC_DACL0                  (0x0 << 10)
#define AC100_I2S1_SND_IN_CTRL_DACL0_SRC_DACR0                  (0x1 << 10)
#define AC100_I2S1_SND_IN_CTRL_DACL0_SRC_DACL0_PLUS_DACR0       (0x2 << 10)
#define AC100_I2S1_SND_IN_CTRL_DACL0_SRC_DACL0_PLUS_DACR0_DIV_2 (0x3 << 10)
#define AC100_I2S1_SND_IN_CTRL_DACR0_SRC_OFF                    8
#define AC100_I2S1_SND_IN_CTRL_DACR0_SRC_MASK                   GENMASK(9, 8)
#define AC100_I2S1_SND_IN_CTRL_DACR0_SRC_DACR0                  (0x0 << 8)
#define AC100_I2S1_SND_IN_CTRL_DACR0_SRC_DACL0                  (0x1 << 8)
#define AC100_I2S1_SND_IN_CTRL_DACR0_SRC_DACL0_PLUS_DACR0       (0x2 << 8)
#define AC100_I2S1_SND_IN_CTRL_DACR0_SRC_DACL0_PLUS_DACR0_DIV_2 (0x3 << 8)
#define AC100_I2S1_SND_IN_CTRL_DACL1_SRC_OFF                    6
#define AC100_I2S1_SND_IN_CTRL_DACL1_SRC_MASK                   GENMASK(7, 6)
#define AC100_I2S1_SND_IN_CTRL_DACL1_SRC_DACL1                  (0x0 << 6)
#define AC100_I2S1_SND_IN_CTRL_DACL1_SRC_DACR1                  (0x1 << 6)
#define AC100_I2S1_SND_IN_CTRL_DACL1_SRC_DACL1_PLUS_DACR1       (0x2 << 6)
#define AC100_I2S1_SND_IN_CTRL_DACL1_SRC_DACL1_PLUS_DACR1_DIV_2 (0x3 << 6)
#define AC100_I2S1_SND_IN_CTRL_DACR1_SRC_OFF                    4
#define AC100_I2S1_SND_IN_CTRL_DACR1_SRC_MASK                   GENMASK(5, 4)
#define AC100_I2S1_SND_IN_CTRL_DACR1_SRC_DACR1                  (0x0 << 4)
#define AC100_I2S1_SND_IN_CTRL_DACR1_SRC_DACL1                  (0x1 << 4)
#define AC100_I2S1_SND_IN_CTRL_DACR1_SRC_DACL1_PLUS_DACR1       (0x2 << 4)
#define AC100_I2S1_SND_IN_CTRL_DACR1_SRC_DACL1_PLUS_DACR1_DIV_2 (0x3 << 4)
#define AC100_I2S1_SND_IN_CTRL_DACP_EN_OFF                      3
#define AC100_I2S1_SND_IN_CTRL_DACP_EN_MASK                     BIT(3)
#define AC100_I2S1_SND_IN_CTRL_DACP_EN_DISABLED                 0
#define AC100_I2S1_SND_IN_CTRL_DACP_EN_ENABLED                  BIT(3)
#define AC100_I2S1_SND_IN_CTRL_DACP_SEL_OFF                     2
#define AC100_I2S1_SND_IN_CTRL_DACP_SEL_MASK                    BIT(2)
#define AC100_I2S1_SND_IN_CTRL_DACP_SEL_A_LAW                   0
#define AC100_I2S1_SND_IN_CTRL_DACP_SEL_U_LAW                   BIT(2)
#define AC100_I2S1_SND_IN_CTRL_LOOP_EN_OFF                      0
#define AC100_I2S1_SND_IN_CTRL_LOOP_EN_MASK                     BIT(0)
#define AC100_I2S1_SND_IN_CTRL_LOOP_EN_NO                       0
#define AC100_I2S1_SND_IN_CTRL_LOOP_EN_YES                      BIT(0)

#define AC100_I2S1_MXR_SRC_ADCL0_MXL_SRC_I2S1_DA0L_OFF          15
#define AC100_I2S1_MXR_SRC_ADCL0_MXL_SRC_I2S1_DA0L_MASK         BIT(15)
#define AC100_I2S1_MXR_SRC_ADCL0_MXL_SRC_I2S1_DA0L_DISABLED     0
#define AC100_I2S1_MXR_SRC_ADCL0_MXL_SRC_I2S1_DA0L_ENABLED      BIT(15)
#define AC100_I2S1_MXR_SRC_ADCL0_MXL_SRC_I2S2_DACL_OFF          14
#define AC100_I2S1_MXR_SRC_ADCL0_MXL_SRC_I2S2_DACL_MASK         BIT(14)
#define AC100_I2S1_MXR_SRC_ADCL0_MXL_SRC_I2S2_DACL_DISABLED     0
#define AC100_I2S1_MXR_SRC_ADCL0_MXL_SRC_I2S2_DACL_ENABLED      BIT(14)
#define AC100_I2S1_MXR_SRC_ADCL0_MXL_SRC_ADCL_OFF               13
#define AC100_I2S1_MXR_SRC_ADCL0_MXL_SRC_ADCL_MASK              BIT(13)
#define AC100_I2S1_MXR_SRC_ADCL0_MXL_SRC_ADCL_DISABLED          0
#define AC100_I2S1_MXR_SRC_ADCL0_MXL_SRC_ADCL_ENABLED           BIT(13)
#define AC100_I2S1_MXR_SRC_ADCL0_MXL_SRC_I2S2_DACR_OFF          12
#define AC100_I2S1_MXR_SRC_ADCL0_MXL_SRC_I2S2_DACR_MASK         BIT(12)
#define AC100_I2S1_MXR_SRC_ADCL0_MXL_SRC_I2S2_DACR_DISABLED     0
#define AC100_I2S1_MXR_SRC_ADCL0_MXL_SRC_I2S2_DACR_ENABLED      BIT(12)
#define AC100_I2S1_MXR_SRC_ADCR0_MXR_SRC_I2S1_DA0R_OFF          11
#define AC100_I2S1_MXR_SRC_ADCR0_MXR_SRC_I2S1_DA0R_MASK         BIT(11)
#define AC100_I2S1_MXR_SRC_ADCR0_MXR_SRC_I2S1_DA0R_DISABLED     0
#define AC100_I2S1_MXR_SRC_ADCR0_MXR_SRC_I2S1_DA0R_ENABLED      BIT(11)
#define AC100_I2S1_MXR_SRC_ADCR0_MXR_SRC_I2S2_DACR_OFF          10
#define AC100_I2S1_MXR_SRC_ADCR0_MXR_SRC_I2S2_DACR_MASK         BIT(10)
#define AC100_I2S1_MXR_SRC_ADCR0_MXR_SRC_I2S2_DACR_DISABLED     0
#define AC100_I2S1_MXR_SRC_ADCR0_MXR_SRC_I2S2_DACR_ENABLED      BIT(10)
#define AC100_I2S1_MXR_SRC_ADCR0_MXR_SRC_ADCR_OFF               9
#define AC100_I2S1_MXR_SRC_ADCR0_MXR_SRC_ADCR_MASK              BIT(9)
#define AC100_I2S1_MXR_SRC_ADCR0_MXR_SRC_ADCR_DISABLED          0
#define AC100_I2S1_MXR_SRC_ADCR0_MXR_SRC_ADCR_ENABLED           BIT(9)
#define AC100_I2S1_MXR_SRC_ADCR0_MXR_SRC_I2S2_DACL_OFF          8
#define AC100_I2S1_MXR_SRC_ADCR0_MXR_SRC_I2S2_DACL_MASK         BIT(8)
#define AC100_I2S1_MXR_SRC_ADCR0_MXR_SRC_I2S2_DACL_DISABLED     0
#define AC100_I2S1_MXR_SRC_ADCR0_MXR_SRC_I2S2_DACL_ENABLED      BIT(8)
#define AC100_I2S1_MXR_SRC_ADCL1_MXR_SRC_I2S2_DACL_OFF          7
#define AC100_I2S1_MXR_SRC_ADCL1_MXR_SRC_I2S2_DACL_MASK         BIT(7)
#define AC100_I2S1_MXR_SRC_ADCL1_MXR_SRC_I2S2_DACL_DISABLED     0
#define AC100_I2S1_MXR_SRC_ADCL1_MXR_SRC_I2S2_DACL_ENABLED      BIT(7)
#define AC100_I2S1_MXR_SRC_ADCL1_MXR_SRC_ADCL_OFF               6
#define AC100_I2S1_MXR_SRC_ADCL1_MXR_SRC_ADCL_MASK              BIT(6)
#define AC100_I2S1_MXR_SRC_ADCL1_MXR_SRC_ADCL_DISABLED          0
#define AC100_I2S1_MXR_SRC_ADCL1_MXR_SRC_ADCL_ENABLED           BIT(6)
#define AC100_I2S1_MXR_SRC_ADCR1_MXR_SRC_I2S2_DACR_OFF          3
#define AC100_I2S1_MXR_SRC_ADCR1_MXR_SRC_I2S2_DACR_MASK         BIT(3)
#define AC100_I2S1_MXR_SRC_ADCR1_MXR_SRC_I2S2_DACR_DISABLED     0
#define AC100_I2S1_MXR_SRC_ADCR1_MXR_SRC_I2S2_DACR_ENABLED      BIT(3)
#define AC100_I2S1_MXR_SRC_ADCR1_MXR_SRC_ADCR_OFF               2
#define AC100_I2S1_MXR_SRC_ADCR1_MXR_SRC_ADCR_MASK              BIT(2)
#define AC100_I2S1_MXR_SRC_ADCR1_MXR_SRC_ADCR_DISABLED          0
#define AC100_I2S1_MXR_SRC_ADCR1_MXR_SRC_ADCR_ENABLED           BIT(2)

#define AC100_I2S1_VOL_CTRL1_ADCL0_VOL_OFF                      8
#define AC100_I2S1_VOL_CTRL1_ADCL0_VOL(v)                       (((v) & 0xff) << 8)
#define AC100_I2S1_VOL_CTRL1_ADCR0_VOL_OFF                      0
#define AC100_I2S1_VOL_CTRL1_ADCR0_VOL(v)                       ((v) & 0xff)

#define AC100_I2S1_VOL_CTRL2_ADCL1_VOL_OFF                      8
#define AC100_I2S1_VOL_CTRL2_ADCL1_VOL(v)                       (((v) & 0xff) << 8)
#define AC100_I2S1_VOL_CTRL2_ADCR1_VOL_OFF                      0
#define AC100_I2S1_VOL_CTRL2_ADCR1_VOL(v)                       ((v) & 0xff)

#define AC100_I2S1_VOL_CTRL3_DACL0_VOL_OFF                      8
#define AC100_I2S1_VOL_CTRL3_DACL0_VOL(v)                       (((v) & 0xff) << 8)
#define AC100_I2S1_VOL_CTRL3_DACR0_VOL_OFF                      0
#define AC100_I2S1_VOL_CTRL3_DACR0_VOL(v)                       ((v) & 0xff)

#define AC100_I2S1_VOL_CTRL4_DACL1_VOL_OFF                      8
#define AC100_I2S1_VOL_CTRL4_DACL1_VOL(v)                       (((v) & 0xff) << 8)
#define AC100_I2S1_VOL_CTRL4_DACR1_VOL_OFF                      0
#define AC100_I2S1_VOL_CTRL4_DACR1_VOL(v)                       ((v) & 0xff)

#define AC100_I2S1_MXR_GAIN_ADCL0_MXL_GAIN_I2S1_DA0L_OFF        15
#define AC100_I2S1_MXR_GAIN_ADCL0_MXL_GAIN_I2S1_DA0L_MASK       BIT(15)
#define AC100_I2S1_MXR_GAIN_ADCL0_MXL_GAIN_I2S1_DA0L_0DB        0
#define AC100_I2S1_MXR_GAIN_ADCL0_MXL_GAIN_I2S1_DA0L_MINUS_6DB  BIT(15)
#define AC100_I2S1_MXR_GAIN_ADCL0_MXL_GAIN_I2S2_DACL_OFF        14
#define AC100_I2S1_MXR_GAIN_ADCL0_MXL_GAIN_I2S2_DACL_MASK       BIT(14)
#define AC100_I2S1_MXR_GAIN_ADCL0_MXL_GAIN_I2S2_DACL_0DB        0
#define AC100_I2S1_MXR_GAIN_ADCL0_MXL_GAIN_I2S2_DACL_MINUS_6DB  BIT(14)
#define AC100_I2S1_MXR_GAIN_ADCL0_MXL_GAIN_ADCL_OFF             13
#define AC100_I2S1_MXR_GAIN_ADCL0_MXL_GAIN_ADCL_MASK            BIT(13)
#define AC100_I2S1_MXR_GAIN_ADCL0_MXL_GAIN_ADCL_0DB             0
#define AC100_I2S1_MXR_GAIN_ADCL0_MXL_GAIN_ADCL_MINUS_6DB       BIT(13)
#define AC100_I2S1_MXR_GAIN_ADCL0_MXL_GAIN_I2S2_DACR_OFF        12
#define AC100_I2S1_MXR_GAIN_ADCL0_MXL_GAIN_I2S2_DACR_MASK       BIT(12)
#define AC100_I2S1_MXR_GAIN_ADCL0_MXL_GAIN_I2S2_DACR_0DB        0
#define AC100_I2S1_MXR_GAIN_ADCL0_MXL_GAIN_I2S2_DACR_MINUS_6DB  BIT(12)
#define AC100_I2S1_MXR_GAIN_ADCR0_MXR_GAIN_I2S1_DA0R_OFF        11
#define AC100_I2S1_MXR_GAIN_ADCR0_MXR_GAIN_I2S1_DA0R_MASK       BIT(11)
#define AC100_I2S1_MXR_GAIN_ADCR0_MXR_GAIN_I2S1_DA0R_0DB        0
#define AC100_I2S1_MXR_GAIN_ADCR0_MXR_GAIN_I2S1_DA0R_MINUS_6DB  BIT(11)
#define AC100_I2S1_MXR_GAIN_ADCR0_MXR_GAIN_I2S2_DACR_OFF        10
#define AC100_I2S1_MXR_GAIN_ADCR0_MXR_GAIN_I2S2_DACR_MASK       BIT(10)
#define AC100_I2S1_MXR_GAIN_ADCR0_MXR_GAIN_I2S2_DACR_0DB        0
#define AC100_I2S1_MXR_GAIN_ADCR0_MXR_GAIN_I2S2_DACR_MINUS_6DB  BIT(10)
#define AC100_I2S1_MXR_GAIN_ADCR0_MXR_GAIN_ADCR_OFF             9
#define AC100_I2S1_MXR_GAIN_ADCR0_MXR_GAIN_ADCR_MASK            BIT(9)
#define AC100_I2S1_MXR_GAIN_ADCR0_MXR_GAIN_ADCR_0DB             0
#define AC100_I2S1_MXR_GAIN_ADCR0_MXR_GAIN_ADCR_MINUS_6DB       BIT(9)
#define AC100_I2S1_MXR_GAIN_ADCR0_MXR_GAIN_I2S2_DACL_OFF        8
#define AC100_I2S1_MXR_GAIN_ADCR0_MXR_GAIN_I2S2_DACL_MASK       BIT(8)
#define AC100_I2S1_MXR_GAIN_ADCR0_MXR_GAIN_I2S2_DACL_0DB        0
#define AC100_I2S1_MXR_GAIN_ADCR0_MXR_GAIN_I2S2_DACL_MINUS_6DB  BIT(8)
#define AC100_I2S1_MXR_GAIN_ADCL1_MXR_GAIN_I2S2_DACL_OFF        7
#define AC100_I2S1_MXR_GAIN_ADCL1_MXR_GAIN_I2S2_DACL_MASK       BIT(7)
#define AC100_I2S1_MXR_GAIN_ADCL1_MXR_GAIN_I2S2_DACL_0DB        0
#define AC100_I2S1_MXR_GAIN_ADCL1_MXR_GAIN_I2S2_DACL_MINUS_6DB  BIT(7)
#define AC100_I2S1_MXR_GAIN_ADCL1_MXR_GAIN_ADCL_OFF             6
#define AC100_I2S1_MXR_GAIN_ADCL1_MXR_GAIN_ADCL_MASK            BIT(6)
#define AC100_I2S1_MXR_GAIN_ADCL1_MXR_GAIN_ADCL_0DB             0
#define AC100_I2S1_MXR_GAIN_ADCL1_MXR_GAIN_ADCL_MINUS_6DB       BIT(6)
#define AC100_I2S1_MXR_GAIN_ADCR1_MXR_GAIN_I2S2_DACR_OFF        3
#define AC100_I2S1_MXR_GAIN_ADCR1_MXR_GAIN_I2S2_DACR_MASK       BIT(3)
#define AC100_I2S1_MXR_GAIN_ADCR1_MXR_GAIN_I2S2_DACR_0DB        0
#define AC100_I2S1_MXR_GAIN_ADCR1_MXR_GAIN_I2S2_DACR_MINUS_6DB  BIT(3)
#define AC100_I2S1_MXR_GAIN_ADCR1_MXR_GAIN_ADCR_OFF             2
#define AC100_I2S1_MXR_GAIN_ADCR1_MXR_GAIN_ADCR_MASK            BIT(2)
#define AC100_I2S1_MXR_GAIN_ADCR1_MXR_GAIN_ADCR_0DB             0
#define AC100_I2S1_MXR_GAIN_ADCR1_MXR_GAIN_ADCR_MINUS_6DB       BIT(2)

#define AC100_I2S2_CLK_CTRL_MSTR_MOD_OFF                        15
#define AC100_I2S2_CLK_CTRL_MSTR_MOD_MASK                       BIT(15)
#define AC100_I2S2_CLK_CTRL_MSTR_MOD_MASTER                     0
#define AC100_I2S2_CLK_CTRL_MSTR_MOD_SLAVE                      BIT(15)
#define AC100_I2S2_CLK_CTRL_BCLK_INV_OFF                        14
#define AC100_I2S2_CLK_CTRL_BCLK_INV_MASK                       BIT(14)
#define AC100_I2S2_CLK_CTRL_BCLK_INV_NORMAL                     0
#define AC100_I2S2_CLK_CTRL_BCLK_INV_INVERTED                   BIT(14)
#define AC100_I2S2_CLK_CTRL_LRCK_INV_OFF                        13
#define AC100_I2S2_CLK_CTRL_LRCK_INV_MASK                       BIT(13)
#define AC100_I2S2_CLK_CTRL_LRCK_INV_NORMAL                     0
#define AC100_I2S2_CLK_CTRL_LRCK_INV_INVERTED                   BIT(13)
#define AC100_I2S2_CLK_CTRL_BCLK_DIV_OFF                        9
#define AC100_I2S2_CLK_CTRL_BCLK_DIV_MASK                       GENMASK(12, 9)
#define AC100_I2S2_CLK_CTRL_BCLK_DIV_1                          (0x0 << 9)
#define AC100_I2S2_CLK_CTRL_BCLK_DIV_2                          (0x1 << 9)
#define AC100_I2S2_CLK_CTRL_BCLK_DIV_4                          (0x2 << 9)
#define AC100_I2S2_CLK_CTRL_BCLK_DIV_6                          (0x3 << 9)
#define AC100_I2S2_CLK_CTRL_BCLK_DIV_8                          (0x4 << 9)
#define AC100_I2S2_CLK_CTRL_BCLK_DIV_12                         (0x5 << 9)
#define AC100_I2S2_CLK_CTRL_BCLK_DIV_16                         (0x6 << 9)
#define AC100_I2S2_CLK_CTRL_BCLK_DIV_24                         (0x7 << 9)
#define AC100_I2S2_CLK_CTRL_BCLK_DIV_32                         (0x8 << 9)
#define AC100_I2S2_CLK_CTRL_BCLK_DIV_48                         (0x9 << 9)
#define AC100_I2S2_CLK_CTRL_BCLK_DIV_64                         (0xa << 9)
#define AC100_I2S2_CLK_CTRL_BCLK_DIV_96                         (0xb << 9)
#define AC100_I2S2_CLK_CTRL_BCLK_DIV_128                        (0xc << 9)
#define AC100_I2S2_CLK_CTRL_BCLK_DIV_192                        (0xd << 9)
#define AC100_I2S2_CLK_CTRL_LRCK_DIV_OFF                        6
#define AC100_I2S2_CLK_CTRL_LRCK_DIV_MASK                       GENMASK(8, 6)
#define AC100_I2S2_CLK_CTRL_LRCK_DIV_16                         (0x0 << 6)
#define AC100_I2S2_CLK_CTRL_LRCK_DIV_32                         (0x1 << 6)
#define AC100_I2S2_CLK_CTRL_LRCK_DIV_64                         (0x2 << 6)
#define AC100_I2S2_CLK_CTRL_LRCK_DIV_128                        (0x3 << 6)
#define AC100_I2S2_CLK_CTRL_LRCK_DIV_256                        (0x4 << 6)
#define AC100_I2S2_CLK_CTRL_WORD_SIZE_OFF                       4
#define AC100_I2S2_CLK_CTRL_WORD_SIZE_MASK                      GENMASK(5, 4)
#define AC100_I2S2_CLK_CTRL_WORD_SIZE_8BIT                      (0x0 << 4)
#define AC100_I2S2_CLK_CTRL_WORD_SIZE_16BIT                     (0x1 << 4)
#define AC100_I2S2_CLK_CTRL_WORD_SIZE_20BIT                     (0x2 << 4)
#define AC100_I2S2_CLK_CTRL_WORD_SIZE_24BIT                     (0x3 << 4)
#define AC100_I2S2_CLK_CTRL_DATA_FMT_OFF                        2
#define AC100_I2S2_CLK_CTRL_DATA_FMT_MASK                       GENMASK(3, 2)
#define AC100_I2S2_CLK_CTRL_DATA_FMT_I2S_MODE                   (0x0 << 2)
#define AC100_I2S2_CLK_CTRL_DATA_FMT_LEFT_MODE                  (0x1 << 2)
#define AC100_I2S2_CLK_CTRL_DATA_FMT_RIGHT_MODE                 (0x2 << 2)
#define AC100_I2S2_CLK_CTRL_DATA_FMT_DSP_MODE                   (0x3 << 2)
#define AC100_I2S2_CLK_CTRL_DSP_MONO_PCM_OFF                    1
#define AC100_I2S2_CLK_CTRL_DSP_MONO_PCM_MASK                   BIT(1)
#define AC100_I2S2_CLK_CTRL_DSP_MONO_PCM_STEREO                 0
#define AC100_I2S2_CLK_CTRL_DSP_MONO_PCM_MONO                   BIT(1)

#define AC100_I2S2_SND_OUT_CTRL_ADCL_EN_OFF                     15
#define AC100_I2S2_SND_OUT_CTRL_ADCL_EN_MASK                    BIT(15)
#define AC100_I2S2_SND_OUT_CTRL_ADCL_EN_DISABLED                0
#define AC100_I2S2_SND_OUT_CTRL_ADCL_EN_ENABLED                 BIT(15)
#define AC100_I2S2_SND_OUT_CTRL_ADCR_EN_OFF                     14
#define AC100_I2S2_SND_OUT_CTRL_ADCR_EN_MASK                    BIT(14)
#define AC100_I2S2_SND_OUT_CTRL_ADCR_EN_DISABLED                0
#define AC100_I2S2_SND_OUT_CTRL_ADCR_EN_ENABLED                 BIT(14)
#define AC100_I2S2_SND_OUT_CTRL_ADCL_SRC_OFF                    10
#define AC100_I2S2_SND_OUT_CTRL_ADCL_SRC_MASK                   GENMASK(11, 10)
#define AC100_I2S2_SND_OUT_CTRL_ADCL_SRC_ADCL                   (0x0 << 10)
#define AC100_I2S2_SND_OUT_CTRL_ADCL_SRC_ADCR                   (0x1 << 10)
#define AC100_I2S2_SND_OUT_CTRL_ADCL_SRC_ADCL_PLUS_ADCR         (0x2 << 10)
#define AC100_I2S2_SND_OUT_CTRL_ADCL_SRC_ADCL_PLUS_ADCR_DIV_2   (0x3 << 10)
#define AC100_I2S2_SND_OUT_CTRL_ADCR_SRC_OFF                    8
#define AC100_I2S2_SND_OUT_CTRL_ADCR_SRC_MASK                   GENMASK(9, 8)
#define AC100_I2S2_SND_OUT_CTRL_ADCR_SRC_ADCR                   (0x0 << 8)
#define AC100_I2S2_SND_OUT_CTRL_ADCR_SRC_ADCL                   (0x1 << 8)
#define AC100_I2S2_SND_OUT_CTRL_ADCR_SRC_ADCL_PLUS_ADCR         (0x2 << 8)
#define AC100_I2S2_SND_OUT_CTRL_ADCR_SRC_ADCL_PLUS_ADCR_DIV_2   (0x3 << 8)
#define AC100_I2S2_SND_OUT_CTRL_ADCP_EN_OFF                     3
#define AC100_I2S2_SND_OUT_CTRL_ADCP_EN_MASK                    BIT(3)
#define AC100_I2S2_SND_OUT_CTRL_ADCP_EN_DISABLED                0
#define AC100_I2S2_SND_OUT_CTRL_ADCP_EN_ENABLED                 BIT(3)
#define AC100_I2S2_SND_OUT_CTRL_ADCP_SEL_OFF                    2
#define AC100_I2S2_SND_OUT_CTRL_ADCP_SEL_MASK                   BIT(2)
#define AC100_I2S2_SND_OUT_CTRL_ADCP_SEL_A_LAW                  0
#define AC100_I2S2_SND_OUT_CTRL_ADCP_SEL_U_LAW                  BIT(2)

#define AC100_I2S2_SND_IN_CTRL_DACL_EN_OFF                      15
#define AC100_I2S2_SND_IN_CTRL_DACL_EN_MASK                     BIT(15)
#define AC100_I2S2_SND_IN_CTRL_DACL_EN_DISABLED                 0
#define AC100_I2S2_SND_IN_CTRL_DACL_EN_ENABLED                  BIT(15)
#define AC100_I2S2_SND_IN_CTRL_DACR_EN_OFF                      14
#define AC100_I2S2_SND_IN_CTRL_DACR_EN_MASK                     BIT(14)
#define AC100_I2S2_SND_IN_CTRL_DACR_EN_DISABLED                 0
#define AC100_I2S2_SND_IN_CTRL_DACR_EN_ENABLED                  BIT(14)
#define AC100_I2S2_SND_IN_CTRL_DACL_SRC_OFF                     10
#define AC100_I2S2_SND_IN_CTRL_DACL_SRC_MASK                    GENMASK(11, 10)
#define AC100_I2S2_SND_IN_CTRL_DACL_SRC_DACL                    (0x0 << 10)
#define AC100_I2S2_SND_IN_CTRL_DACL_SRC_DACR                    (0x1 << 10)
#define AC100_I2S2_SND_IN_CTRL_DACL_SRC_DACL_PLUS_DACR          (0x2 << 10)
#define AC100_I2S2_SND_IN_CTRL_DACL_SRC_DACL_PLUS_DACR_DIV_2    (0x3 << 10)
#define AC100_I2S2_SND_IN_CTRL_DACR_SRC_OFF                     8
#define AC100_I2S2_SND_IN_CTRL_DACR_SRC_MASK                    GENMASK(9, 8)
#define AC100_I2S2_SND_IN_CTRL_DACR_SRC_DACR                    (0x0 << 8)
#define AC100_I2S2_SND_IN_CTRL_DACR_SRC_DACL                    (0x1 << 8)
#define AC100_I2S2_SND_IN_CTRL_DACR_SRC_DACL_PLUS_DACR          (0x2 << 8)
#define AC100_I2S2_SND_IN_CTRL_DACR_SRC_DACL_PLUS_DACR_DIV_2    (0x3 << 8)
#define AC100_I2S2_SND_IN_CTRL_DACP_EN_OFF                      3
#define AC100_I2S2_SND_IN_CTRL_DACP_EN_MASK                     BIT(3)
#define AC100_I2S2_SND_IN_CTRL_DACP_EN_DISABLED                 0
#define AC100_I2S2_SND_IN_CTRL_DACP_EN_ENABLED                  BIT(3)
#define AC100_I2S2_SND_IN_CTRL_DACP_SEL_OFF                     2
#define AC100_I2S2_SND_IN_CTRL_DACP_SEL_MASK                    BIT(2)
#define AC100_I2S2_SND_IN_CTRL_DACP_SEL_A_LAW                   0
#define AC100_I2S2_SND_IN_CTRL_DACP_SEL_U_LAW                   BIT(2)
#define AC100_I2S2_SND_IN_CTRL_LOOP_EN_OFF                      0
#define AC100_I2S2_SND_IN_CTRL_LOOP_EN_MASK                     BIT(0)
#define AC100_I2S2_SND_IN_CTRL_LOOP_EN_NO                       0
#define AC100_I2S2_SND_IN_CTRL_LOOP_EN_YES                      BIT(0)

#define AC100_I2S2_MXR_SRC_ADCL_MXR_SRC_I2S1_DA0L_OFF           15
#define AC100_I2S2_MXR_SRC_ADCL_MXR_SRC_I2S1_DA0L_MASK          BIT(15)
#define AC100_I2S2_MXR_SRC_ADCL_MXR_SRC_I2S1_DA0L_DISABLED      0
#define AC100_I2S2_MXR_SRC_ADCL_MXR_SRC_I2S1_DA0L_ENABLED       BIT(15)
#define AC100_I2S2_MXR_SRC_ADCL_MXR_SRC_I2S1_DA1L_OFF           14
#define AC100_I2S2_MXR_SRC_ADCL_MXR_SRC_I2S1_DA1L_MASK          BIT(14)
#define AC100_I2S2_MXR_SRC_ADCL_MXR_SRC_I2S1_DA1L_DISABLED      0
#define AC100_I2S2_MXR_SRC_ADCL_MXR_SRC_I2S1_DA1L_ENABLED       BIT(14)
#define AC100_I2S2_MXR_SRC_ADCL_MXR_SRC_I2S2_DACR_OFF           13
#define AC100_I2S2_MXR_SRC_ADCL_MXR_SRC_I2S2_DACR_MASK          BIT(13)
#define AC100_I2S2_MXR_SRC_ADCL_MXR_SRC_I2S2_DACR_DISABLED      0
#define AC100_I2S2_MXR_SRC_ADCL_MXR_SRC_I2S2_DACR_ENABLED       BIT(13)
#define AC100_I2S2_MXR_SRC_ADCL_MXR_SRC_ADCL_OFF                12
#define AC100_I2S2_MXR_SRC_ADCL_MXR_SRC_ADCL_MASK               BIT(12)
#define AC100_I2S2_MXR_SRC_ADCL_MXR_SRC_ADCL_DISABLED           0
#define AC100_I2S2_MXR_SRC_ADCL_MXR_SRC_ADCL_ENABLED            BIT(12)
#define AC100_I2S2_MXR_SRC_ADCR_MXR_SRC_I2S1_DA0R_OFF           11
#define AC100_I2S2_MXR_SRC_ADCR_MXR_SRC_I2S1_DA0R_MASK          BIT(11)
#define AC100_I2S2_MXR_SRC_ADCR_MXR_SRC_I2S1_DA0R_DISABLED      0
#define AC100_I2S2_MXR_SRC_ADCR_MXR_SRC_I2S1_DA0R_ENABLED       BIT(11)
#define AC100_I2S2_MXR_SRC_ADCR_MXR_SRC_I2S1_DA1R_OFF           10
#define AC100_I2S2_MXR_SRC_ADCR_MXR_SRC_I2S1_DA1R_MASK          BIT(10)
#define AC100_I2S2_MXR_SRC_ADCR_MXR_SRC_I2S1_DA1R_DISABLED      0
#define AC100_I2S2_MXR_SRC_ADCR_MXR_SRC_I2S1_DA1R_ENABLED       BIT(10)
#define AC100_I2S2_MXR_SRC_ADCR_MXR_SRC_I2S2_DACL_OFF           9
#define AC100_I2S2_MXR_SRC_ADCR_MXR_SRC_I2S2_DACL_MASK          BIT(9)
#define AC100_I2S2_MXR_SRC_ADCR_MXR_SRC_I2S2_DACL_DISABLED      0
#define AC100_I2S2_MXR_SRC_ADCR_MXR_SRC_I2S2_DACL_ENABLED       BIT(9)
#define AC100_I2S2_MXR_SRC_ADCR_MXR_SRC_ADCR_OFF                8
#define AC100_I2S2_MXR_SRC_ADCR_MXR_SRC_ADCR_MASK               BIT(8)
#define AC100_I2S2_MXR_SRC_ADCR_MXR_SRC_ADCR_DISABLED           0
#define AC100_I2S2_MXR_SRC_ADCR_MXR_SRC_ADCR_ENABLED            BIT(8)

#define AC100_I2S2_VOL_CTRL1_ADCL_VOL_OFF                       8
#define AC100_I2S2_VOL_CTRL1_ADCL_VOL(v)                        (((v) & 0xff) << 8)
#define AC100_I2S2_VOL_CTRL1_ADCR_VOL_OFF                       0
#define AC100_I2S2_VOL_CTRL1_ADCR_VOL(v)                        ((v) & 0xff)

#define AC100_I2S2_VOL_CTRL2_DACL_VOL_OFF                       8
#define AC100_I2S2_VOL_CTRL2_DACL_VOL(v)                        (((v) & 0xff) << 8)
#define AC100_I2S2_VOL_CTRL2_DACR_VOL_OFF                       0
#define AC100_I2S2_VOL_CTRL2_DACR_VOL(v)                        ((v) & 0xff)

#define AC100_I2S2_MXR_GAIN_ADCL_MXR_GAIN_I2S1_DA0L_OFF         15
#define AC100_I2S2_MXR_GAIN_ADCL_MXR_GAIN_I2S1_DA0L_MASK        BIT(15)
#define AC100_I2S2_MXR_GAIN_ADCL_MXR_GAIN_I2S1_DA0L_0DB         0
#define AC100_I2S2_MXR_GAIN_ADCL_MXR_GAIN_I2S1_DA0L_MINUS_6DB   BIT(15)
#define AC100_I2S2_MXR_GAIN_ADCL_MXR_GAIN_I2S1_DA1L_OFF         14
#define AC100_I2S2_MXR_GAIN_ADCL_MXR_GAIN_I2S1_DA1L_MASK        BIT(14)
#define AC100_I2S2_MXR_GAIN_ADCL_MXR_GAIN_I2S1_DA1L_0DB         0
#define AC100_I2S2_MXR_GAIN_ADCL_MXR_GAIN_I2S1_DA1L_MINUS_6DB   BIT(14)
#define AC100_I2S2_MXR_GAIN_ADCL_MXR_GAIN_I2S2_DACR_OFF         13
#define AC100_I2S2_MXR_GAIN_ADCL_MXR_GAIN_I2S2_DACR_MASK        BIT(13)
#define AC100_I2S2_MXR_GAIN_ADCL_MXR_GAIN_I2S2_DACR_0DB         0
#define AC100_I2S2_MXR_GAIN_ADCL_MXR_GAIN_I2S2_DACR_MINUS_6DB   BIT(13)
#define AC100_I2S2_MXR_GAIN_ADCL_MXR_GAIN_ADCL_OFF              12
#define AC100_I2S2_MXR_GAIN_ADCL_MXR_GAIN_ADCL_MASK             BIT(12)
#define AC100_I2S2_MXR_GAIN_ADCL_MXR_GAIN_ADCL_0DB              0
#define AC100_I2S2_MXR_GAIN_ADCL_MXR_GAIN_ADCL_MINUS_6DB        BIT(12)
#define AC100_I2S2_MXR_GAIN_ADCR_MXR_GAIN_I2S1_DA0R_OFF         11
#define AC100_I2S2_MXR_GAIN_ADCR_MXR_GAIN_I2S1_DA0R_MASK        BIT(11)
#define AC100_I2S2_MXR_GAIN_ADCR_MXR_GAIN_I2S1_DA0R_0DB         0
#define AC100_I2S2_MXR_GAIN_ADCR_MXR_GAIN_I2S1_DA0R_MINUS_6DB   BIT(11)
#define AC100_I2S2_MXR_GAIN_ADCR_MXR_GAIN_I2S1_DA1R_OFF         10
#define AC100_I2S2_MXR_GAIN_ADCR_MXR_GAIN_I2S1_DA1R_MASK        BIT(10)
#define AC100_I2S2_MXR_GAIN_ADCR_MXR_GAIN_I2S1_DA1R_0DB         0
#define AC100_I2S2_MXR_GAIN_ADCR_MXR_GAIN_I2S1_DA1R_MINUS_6DB   BIT(10)
#define AC100_I2S2_MXR_GAIN_ADCR_MXR_GAIN_I2S2_DACL_OFF         9
#define AC100_I2S2_MXR_GAIN_ADCR_MXR_GAIN_I2S2_DACL_MASK        BIT(9)
#define AC100_I2S2_MXR_GAIN_ADCR_MXR_GAIN_I2S2_DACL_0DB         0
#define AC100_I2S2_MXR_GAIN_ADCR_MXR_GAIN_I2S2_DACL_MINUS_6DB   BIT(9)
#define AC100_I2S2_MXR_GAIN_ADCR_MXR_GAIN_ADCR_OFF              8
#define AC100_I2S2_MXR_GAIN_ADCR_MXR_GAIN_ADCR_MASK             BIT(8)
#define AC100_I2S2_MXR_GAIN_ADCR_MXR_GAIN_ADCR_0DB              0
#define AC100_I2S2_MXR_GAIN_ADCR_MXR_GAIN_ADCR_MINUS_6DB        BIT(8)

#define AC100_I2S3_CLK_CTRL_BCLK_INV_OFF                        14
#define AC100_I2S3_CLK_CTRL_BCLK_INV_MASK                       BIT(14)
#define AC100_I2S3_CLK_CTRL_BCLK_INV_NORMAL                     0
#define AC100_I2S3_CLK_CTRL_BCLK_INV_INVERTED                   BIT(14)
#define AC100_I2S3_CLK_CTRL_LRCK_INV_OFF                        13
#define AC100_I2S3_CLK_CTRL_LRCK_INV_MASK                       BIT(13)
#define AC100_I2S3_CLK_CTRL_LRCK_INV_NORMAL                     0
#define AC100_I2S3_CLK_CTRL_LRCK_INV_INVERTED                   BIT(13)
#define AC100_I2S3_CLK_CTRL_WORD_SIZE_OFF                       4
#define AC100_I2S3_CLK_CTRL_WORD_SIZE_MASK                      GENMASK(5, 4)
#define AC100_I2S3_CLK_CTRL_WORD_SIZE_8BIT                      (0x0 << 4)
#define AC100_I2S3_CLK_CTRL_WORD_SIZE_16BIT                     (0x1 << 4)
#define AC100_I2S3_CLK_CTRL_WORD_SIZE_20BIT                     (0x2 << 4)
#define AC100_I2S3_CLK_CTRL_WORD_SIZE_24BIT                     (0x3 << 4)
#define AC100_I2S3_CLK_CTRL_CLOCK_SRC_OFF                       0
#define AC100_I2S3_CLK_CTRL_CLOCK_SRC_MASK                      GENMASK(1, 0)
#define AC100_I2S3_CLK_CTRL_CLOCK_SRC_I2S1                      0x0
#define AC100_I2S3_CLK_CTRL_CLOCK_SRC_I2S2                      0x1
#define AC100_I2S3_CLK_CTRL_CLOCK_SRC_I2S3_I2S1_CLK             0x2

#define AC100_I2S3_SND_OUT_CTRL_ADCP_EN_OFF                     3
#define AC100_I2S3_SND_OUT_CTRL_ADCP_EN_MASK                    BIT(3)
#define AC100_I2S3_SND_OUT_CTRL_ADCP_EN_DISABLED                0
#define AC100_I2S3_SND_OUT_CTRL_ADCP_EN_ENABLED                 BIT(3)
#define AC100_I2S3_SND_OUT_CTRL_ADCP_SEL_OFF                    2
#define AC100_I2S3_SND_OUT_CTRL_ADCP_SEL_MASK                   BIT(2)
#define AC100_I2S3_SND_OUT_CTRL_ADCP_SEL_A_LAW                  0
#define AC100_I2S3_SND_OUT_CTRL_ADCP_SEL_U_LAW                  BIT(2)

#define AC100_I2S3_SND_IN_CTRL_DACP_EN_OFF                      3
#define AC100_I2S3_SND_IN_CTRL_DACP_EN_MASK                     BIT(3)
#define AC100_I2S3_SND_IN_CTRL_DACP_EN_DISABLED                 0
#define AC100_I2S3_SND_IN_CTRL_DACP_EN_ENABLED                  BIT(3)
#define AC100_I2S3_SND_IN_CTRL_DACP_SEL_OFF                     2
#define AC100_I2S3_SND_IN_CTRL_DACP_SEL_MASK                    BIT(2)
#define AC100_I2S3_SND_IN_CTRL_DACP_SEL_A_LAW                   0
#define AC100_I2S3_SND_IN_CTRL_DACP_SEL_U_LAW                   BIT(2)
#define AC100_I2S3_SND_IN_CTRL_LOOP_EN_OFF                      0
#define AC100_I2S3_SND_IN_CTRL_LOOP_EN_MASK                     BIT(0)
#define AC100_I2S3_SND_IN_CTRL_LOOP_EN_NO                       0
#define AC100_I2S3_SND_IN_CTRL_LOOP_EN_YES                      BIT(0)

#define AC100_I2S3_SIG_PATH_CTRL_I2S3_ADC_SRC_OFF               10
#define AC100_I2S3_SIG_PATH_CTRL_I2S3_ADC_SRC_MASK              GENMASK(11, 10)
#define AC100_I2S3_SIG_PATH_CTRL_I2S3_ADC_SRC_NONE              (0x0 << 10)
#define AC100_I2S3_SIG_PATH_CTRL_I2S3_ADC_SRC_I2S2_ADCL         (0x1 << 10)
#define AC100_I2S3_SIG_PATH_CTRL_I2S3_ADC_SRC_I2S2_ADCR         (0x1 << 10)
#define AC100_I2S3_SIG_PATH_CTRL_I2S2_DAC_SRC_OFF               8
#define AC100_I2S3_SIG_PATH_CTRL_I2S2_DAC_SRC_MASK              GENMASK(9, 8)
#define AC100_I2S3_SIG_PATH_CTRL_I2S2_DAC_SRC_I2S2_ADCL_I2S2_ADCR (0x0 << 8)
#define AC100_I2S3_SIG_PATH_CTRL_I2S2_DAC_SRC_I2S3_DAC_I2S2_ADCR (0x1 << 8)
#define AC100_I2S3_SIG_PATH_CTRL_I2S2_DAC_SRC_I2S2_ADCL_I2S3_DAC (0x2 << 8)

#define AC100_ADC_DIG_CTRL_DIG_EN_OFF                           15
#define AC100_ADC_DIG_CTRL_DIG_EN_MASK                          BIT(15)
#define AC100_ADC_DIG_CTRL_DIG_EN_DISABLED                      0
#define AC100_ADC_DIG_CTRL_DIG_EN_ENABLED                       BIT(15)
#define AC100_ADC_DIG_CTRL_MIC_EN_OFF                           14
#define AC100_ADC_DIG_CTRL_MIC_EN_MASK                          BIT(14)
#define AC100_ADC_DIG_CTRL_MIC_EN_ANALOG_ADC_MODE               0
#define AC100_ADC_DIG_CTRL_MIC_EN_DIGITAL_MIC_MODE              BIT(14)
#define AC100_ADC_DIG_CTRL_ADFIR32_OFF                          13
#define AC100_ADC_DIG_CTRL_ADFIR32_MASK                         BIT(13)
#define AC100_ADC_DIG_CTRL_ADFIR32_64TAP                        0
#define AC100_ADC_DIG_CTRL_ADFIR32_32TAP                        BIT(13)
#define AC100_ADC_DIG_CTRL_ADOUT_DTS_OFF                        2
#define AC100_ADC_DIG_CTRL_ADOUT_DTS_MASK                       GENMASK(3, 2)
#define AC100_ADC_DIG_CTRL_ADOUT_DTS_5MS                        (0x0 << 2)
#define AC100_ADC_DIG_CTRL_ADOUT_DTS_10MS                       (0x1 << 2)
#define AC100_ADC_DIG_CTRL_ADOUT_DTS_20MS                       (0x2 << 2)
#define AC100_ADC_DIG_CTRL_ADOUT_DTS_30MS                       (0x3 << 2)
#define AC100_ADC_DIG_CTRL_ADOUT_DLY_OFF                        1
#define AC100_ADC_DIG_CTRL_ADOUT_DLY_MASK                       BIT(1)
#define AC100_ADC_DIG_CTRL_ADOUT_DLY_DISABLED                   0
#define AC100_ADC_DIG_CTRL_ADOUT_DLY_ENABLED                    BIT(1)

#define AC100_ADC_VOL_CTRL_ADCL_VOL_OFF                         8
#define AC100_ADC_VOL_CTRL_ADCL_VOL(v)                          (((v) & 0xff) << 8)
#define AC100_ADC_VOL_CTRL_ADCR_VOL_OFF                         0
#define AC100_ADC_VOL_CTRL_ADCR_VOL(v)                          ((v) & 0xff)

#define AC100_HMIC_CTRL1_M_OFF                                  12
#define AC100_HMIC_CTRL1_M(v)                                   (((v) & 0xf) << 12)
#define AC100_HMIC_CTRL1_N_OFF                                  8
#define AC100_HMIC_CTRL1_N(v)                                   (((v) & 0xf) << 8)
#define AC100_HMIC_CTRL1_DATA_IRQ_MODE_OFF                      7
#define AC100_HMIC_CTRL1_DATA_IRQ_MODE_MASK                     BIT(7)
#define AC100_HMIC_CTRL1_DATA_IRQ_MODE_ONCE                     0
#define AC100_HMIC_CTRL1_DATA_IRQ_MODE_REPEATED                 BIT(7)
#define AC100_HMIC_CTRL1_HYST_TH1_OFF                           5
#define AC100_HMIC_CTRL1_HYST_TH1_MASK                          GENMASK(6, 5)
#define AC100_HMIC_CTRL1_HYST_TH1_NONE                          (0x0 << 5)
#define AC100_HMIC_CTRL1_HYST_TH1_TH2_MINUS_1                   (0x1 << 5)
#define AC100_HMIC_CTRL1_HYST_TH1_TH2_MINUS_2                   (0x2 << 5)
#define AC100_HMIC_CTRL1_HYST_TH1_TH2_MINUS_3                   (0x3 << 5)
#define AC100_HMIC_CTRL1_PULLOUT_IRQ_EN_OFF                     4
#define AC100_HMIC_CTRL1_PULLOUT_IRQ_EN_MASK                    BIT(4)
#define AC100_HMIC_CTRL1_PULLOUT_IRQ_EN_DISABLED                0
#define AC100_HMIC_CTRL1_PULLOUT_IRQ_EN_ENABLED                 BIT(4)
#define AC100_HMIC_CTRL1_PLUGIN_IRQ_EN_OFF                      3
#define AC100_HMIC_CTRL1_PLUGIN_IRQ_EN_MASK                     BIT(3)
#define AC100_HMIC_CTRL1_PLUGIN_IRQ_EN_DISABLED                 0
#define AC100_HMIC_CTRL1_PLUGIN_IRQ_EN_ENABLED                  BIT(3)
#define AC100_HMIC_CTRL1_KEYUP_IRQ_EN_OFF                       2
#define AC100_HMIC_CTRL1_KEYUP_IRQ_EN_MASK                      BIT(2)
#define AC100_HMIC_CTRL1_KEYUP_IRQ_EN_DISABLED                  0
#define AC100_HMIC_CTRL1_KEYUP_IRQ_EN_ENABLED                   BIT(2)
#define AC100_HMIC_CTRL1_KEYDOWN_IRQ_EN_OFF                     1
#define AC100_HMIC_CTRL1_KEYDOWN_IRQ_EN_MASK                    BIT(1)
#define AC100_HMIC_CTRL1_KEYDOWN_IRQ_EN_DISABLED                0
#define AC100_HMIC_CTRL1_KEYDOWN_IRQ_EN_ENABLED                 BIT(1)
#define AC100_HMIC_CTRL1_DATA_IRQ_EN_OFF                        0
#define AC100_HMIC_CTRL1_DATA_IRQ_EN_MASK                       BIT(0)
#define AC100_HMIC_CTRL1_DATA_IRQ_EN_DISABLED                   0
#define AC100_HMIC_CTRL1_DATA_IRQ_EN_ENABLED                    BIT(0)

#define AC100_HMIC_CTRL2_SAMPLE_SELECT_OFF                      14
#define AC100_HMIC_CTRL2_SAMPLE_SELECT_MASK                     GENMASK(15, 14)
#define AC100_HMIC_CTRL2_SAMPLE_SELECT_128HZ                    (0x0 << 14)
#define AC100_HMIC_CTRL2_SAMPLE_SELECT_64HZ                     (0x1 << 14)
#define AC100_HMIC_CTRL2_SAMPLE_SELECT_32HZ                     (0x2 << 14)
#define AC100_HMIC_CTRL2_SAMPLE_SELECT_16HZ                     (0x3 << 14)
#define AC100_HMIC_CTRL2_HYST_TH2_OFF                           13
#define AC100_HMIC_CTRL2_HYST_TH2_MASK                          BIT(13)
#define AC100_HMIC_CTRL2_HYST_TH2_NONE                          0
#define AC100_HMIC_CTRL2_HYST_TH2_TH2_MINUS_1                   BIT(13)
#define AC100_HMIC_CTRL2_TH2_OFF                                8
#define AC100_HMIC_CTRL2_TH2(v)                                 (((v) & 0x1f) << 8)
#define AC100_HMIC_CTRL2_SMOOTH_FILTER_OFF                      6
#define AC100_HMIC_CTRL2_SMOOTH_FILTER_MASK                     GENMASK(7, 6)
#define AC100_HMIC_CTRL2_SMOOTH_FILTER_BYPASS                   (0x0 << 6)
#define AC100_HMIC_CTRL2_SMOOTH_FILTER_AVG2                     (0x1 << 6)
#define AC100_HMIC_CTRL2_SMOOTH_FILTER_AVG4                     (0x2 << 6)
#define AC100_HMIC_CTRL2_SMOOTH_FILTER_AVG8                     (0x3 << 6)
#define AC100_HMIC_CTRL2_KEYUP_CLEAR_OFF                        5
#define AC100_HMIC_CTRL2_KEYUP_CLEAR_MASK                       BIT(5)
#define AC100_HMIC_CTRL2_KEYUP_CLEAR_NO                         0
#define AC100_HMIC_CTRL2_KEYUP_CLEAR_AUTO                       BIT(5)
#define AC100_HMIC_CTRL2_TH1_OFF                                0
#define AC100_HMIC_CTRL2_TH1(v)                                 ((v) & 0x1f)

#define AC100_HMIC_STATUS_DATA_OFF                              8
#define AC100_HMIC_STATUS_DATA(v)                               (((v) & 0x1f) << 8)
#define AC100_HMIC_STATUS_PULLOUT_IRQF_OFF                      4
#define AC100_HMIC_STATUS_PULLOUT_IRQF_MASK                     BIT(4)
#define AC100_HMIC_STATUS_PULLOUT_IRQF_NONE                     0
#define AC100_HMIC_STATUS_PULLOUT_IRQF_PENDING                  BIT(4)
#define AC100_HMIC_STATUS_PLUGIN_IRQF_OFF                       3
#define AC100_HMIC_STATUS_PLUGIN_IRQF_MASK                      BIT(3)
#define AC100_HMIC_STATUS_PLUGIN_IRQF_NONE                      0
#define AC100_HMIC_STATUS_PLUGIN_IRQF_PENDING                   BIT(3)
#define AC100_HMIC_STATUS_KEYUP_IRQF_OFF                        2
#define AC100_HMIC_STATUS_KEYUP_IRQF_MASK                       BIT(2)
#define AC100_HMIC_STATUS_KEYUP_IRQF_NONE                       0
#define AC100_HMIC_STATUS_KEYUP_IRQF_PENDING                    BIT(2)
#define AC100_HMIC_STATUS_KEYDOWN_IRQF_OFF                      1
#define AC100_HMIC_STATUS_KEYDOWN_IRQF_MASK                     BIT(1)
#define AC100_HMIC_STATUS_KEYDOWN_IRQF_NONE                     0
#define AC100_HMIC_STATUS_KEYDOWN_IRQF_PENDING                  BIT(1)
#define AC100_HMIC_STATUS_DATA_IRQF_OFF                         0
#define AC100_HMIC_STATUS_DATA_IRQF_MASK                        BIT(0)
#define AC100_HMIC_STATUS_DATA_IRQF_NONE                        0
#define AC100_HMIC_STATUS_DATA_IRQF_PENDING                     BIT(0)

#define AC100_DAC_DIG_CTRL_DA_EN_OFF                            15
#define AC100_DAC_DIG_CTRL_DA_EN_MASK                           BIT(15)
#define AC100_DAC_DIG_CTRL_DA_EN_DISABLED                       0
#define AC100_DAC_DIG_CTRL_DA_EN_ENABLED                        BIT(15)
#define AC100_DAC_DIG_CTRL_HPF_EN_OFF                           14
#define AC100_DAC_DIG_CTRL_HPF_EN_MASK                          BIT(14)
#define AC100_DAC_DIG_CTRL_HPF_EN_DISABLED                      0
#define AC100_DAC_DIG_CTRL_HPF_EN_ENABLED                       BIT(14)
#define AC100_DAC_DIG_CTRL_DAFIR32_OFF                          13
#define AC100_DAC_DIG_CTRL_DAFIR32_MASK                         BIT(13)
#define AC100_DAC_DIG_CTRL_DAFIR32_64TAP                        0
#define AC100_DAC_DIG_CTRL_DAFIR32_32TAP                        BIT(13)
#define AC100_DAC_DIG_CTRL_MODQU_OFF                            8
#define AC100_DAC_DIG_CTRL_MODQU(v)                             (((v) & 0xf) << 8)

#define AC100_DAC_VOL_CTRL_DACL_VOL_OFF                         8
#define AC100_DAC_VOL_CTRL_DACL_VOL(v)                          (((v) & 0xff) << 8)
#define AC100_DAC_VOL_CTRL_DACR_VOL_OFF                         0
#define AC100_DAC_VOL_CTRL_DACR_VOL(v)                          ((v) & 0xff)

#define AC100_DAC_MXR_SRC_DACL_MXR_SRC_I2S1_DA0L_OFF            15
#define AC100_DAC_MXR_SRC_DACL_MXR_SRC_I2S1_DA0L_MASK           BIT(15)
#define AC100_DAC_MXR_SRC_DACL_MXR_SRC_I2S1_DA0L_DISABLED       0
#define AC100_DAC_MXR_SRC_DACL_MXR_SRC_I2S1_DA0L_ENABLED        BIT(15)
#define AC100_DAC_MXR_SRC_DACL_MXR_SRC_I2S1_DA1L_OFF            14
#define AC100_DAC_MXR_SRC_DACL_MXR_SRC_I2S1_DA1L_MASK           BIT(14)
#define AC100_DAC_MXR_SRC_DACL_MXR_SRC_I2S1_DA1L_DISABLED       0
#define AC100_DAC_MXR_SRC_DACL_MXR_SRC_I2S1_DA1L_ENABLED        BIT(14)
#define AC100_DAC_MXR_SRC_DACL_MXR_SRC_I2S2_DACL_OFF            13
#define AC100_DAC_MXR_SRC_DACL_MXR_SRC_I2S2_DACL_MASK           BIT(13)
#define AC100_DAC_MXR_SRC_DACL_MXR_SRC_I2S2_DACL_DISABLED       0
#define AC100_DAC_MXR_SRC_DACL_MXR_SRC_I2S2_DACL_ENABLED        BIT(13)
#define AC100_DAC_MXR_SRC_DACL_MXR_SRC_ADCL_OFF                 12
#define AC100_DAC_MXR_SRC_DACL_MXR_SRC_ADCL_MASK                BIT(12)
#define AC100_DAC_MXR_SRC_DACL_MXR_SRC_ADCL_DISABLED            0
#define AC100_DAC_MXR_SRC_DACL_MXR_SRC_ADCL_ENABLED             BIT(12)
#define AC100_DAC_MXR_SRC_DACR_MXR_SRC_I2S1_DA0R_OFF            11
#define AC100_DAC_MXR_SRC_DACR_MXR_SRC_I2S1_DA0R_MASK           BIT(11)
#define AC100_DAC_MXR_SRC_DACR_MXR_SRC_I2S1_DA0R_DISABLED       0
#define AC100_DAC_MXR_SRC_DACR_MXR_SRC_I2S1_DA0R_ENABLED        BIT(11)
#define AC100_DAC_MXR_SRC_DACR_MXR_SRC_I2S1_DA1R_OFF            10
#define AC100_DAC_MXR_SRC_DACR_MXR_SRC_I2S1_DA1R_MASK           BIT(10)
#define AC100_DAC_MXR_SRC_DACR_MXR_SRC_I2S1_DA1R_DISABLED       0
#define AC100_DAC_MXR_SRC_DACR_MXR_SRC_I2S1_DA1R_ENABLED        BIT(10)
#define AC100_DAC_MXR_SRC_DACR_MXR_SRC_I2S2_DACR_OFF            9
#define AC100_DAC_MXR_SRC_DACR_MXR_SRC_I2S2_DACR_MASK           BIT(9)
#define AC100_DAC_MXR_SRC_DACR_MXR_SRC_I2S2_DACR_DISABLED       0
#define AC100_DAC_MXR_SRC_DACR_MXR_SRC_I2S2_DACR_ENABLED        BIT(9)
#define AC100_DAC_MXR_SRC_DACR_MXR_SRC_ADCR_OFF                 8
#define AC100_DAC_MXR_SRC_DACR_MXR_SRC_ADCR_MASK                BIT(8)
#define AC100_DAC_MXR_SRC_DACR_MXR_SRC_ADCR_DISABLED            0
#define AC100_DAC_MXR_SRC_DACR_MXR_SRC_ADCR_ENABLED             BIT(8)

#define AC100_DAC_MXR_GAIN_DACL_MXR_SRC_I2S1_DA0L_OFF           15
#define AC100_DAC_MXR_GAIN_DACL_MXR_SRC_I2S1_DA0L_MASK          BIT(15)
#define AC100_DAC_MXR_GAIN_DACL_MXR_SRC_I2S1_DA0L_0DB           0
#define AC100_DAC_MXR_GAIN_DACL_MXR_SRC_I2S1_DA0L_MINUS_6DB     BIT(15)
#define AC100_DAC_MXR_GAIN_DACL_MXR_SRC_I2S1_DA1L_OFF           14
#define AC100_DAC_MXR_GAIN_DACL_MXR_SRC_I2S1_DA1L_MASK          BIT(14)
#define AC100_DAC_MXR_GAIN_DACL_MXR_SRC_I2S1_DA1L_0DB           0
#define AC100_DAC_MXR_GAIN_DACL_MXR_SRC_I2S1_DA1L_MINUS_6DB     BIT(14)
#define AC100_DAC_MXR_GAIN_DACL_MXR_SRC_I2S2_DACL_OFF           13
#define AC100_DAC_MXR_GAIN_DACL_MXR_SRC_I2S2_DACL_MASK          BIT(13)
#define AC100_DAC_MXR_GAIN_DACL_MXR_SRC_I2S2_DACL_0DB           0
#define AC100_DAC_MXR_GAIN_DACL_MXR_SRC_I2S2_DACL_MINUS_6DB     BIT(13)
#define AC100_DAC_MXR_GAIN_DACL_MXR_SRC_ADCL_OFF                12
#define AC100_DAC_MXR_GAIN_DACL_MXR_SRC_ADCL_MASK               BIT(12)
#define AC100_DAC_MXR_GAIN_DACL_MXR_SRC_ADCL_0DB                0
#define AC100_DAC_MXR_GAIN_DACL_MXR_SRC_ADCL_MINUS_6DB          BIT(12)
#define AC100_DAC_MXR_GAIN_DACR_MXR_SRC_I2S1_DA0R_OFF           11
#define AC100_DAC_MXR_GAIN_DACR_MXR_SRC_I2S1_DA0R_MASK          BIT(11)
#define AC100_DAC_MXR_GAIN_DACR_MXR_SRC_I2S1_DA0R_0DB           0
#define AC100_DAC_MXR_GAIN_DACR_MXR_SRC_I2S1_DA0R_MINUS_6DB     BIT(11)
#define AC100_DAC_MXR_GAIN_DACR_MXR_SRC_I2S1_DA1R_OFF           10
#define AC100_DAC_MXR_GAIN_DACR_MXR_SRC_I2S1_DA1R_MASK          BIT(10)
#define AC100_DAC_MXR_GAIN_DACR_MXR_SRC_I2S1_DA1R_0DB           0
#define AC100_DAC_MXR_GAIN_DACR_MXR_SRC_I2S1_DA1R_MINUS_6DB     BIT(10)
#define AC100_DAC_MXR_GAIN_DACR_MXR_SRC_I2S2_DACR_OFF           9
#define AC100_DAC_MXR_GAIN_DACR_MXR_SRC_I2S2_DACR_MASK          BIT(9)
#define AC100_DAC_MXR_GAIN_DACR_MXR_SRC_I2S2_DACR_0DB           0
#define AC100_DAC_MXR_GAIN_DACR_MXR_SRC_I2S2_DACR_MINUS_6DB     BIT(9)
#define AC100_DAC_MXR_GAIN_DACR_MXR_SRC_ADCR_OFF                8
#define AC100_DAC_MXR_GAIN_DACR_MXR_SRC_ADCR_MASK               BIT(8)
#define AC100_DAC_MXR_GAIN_DACR_MXR_SRC_ADCR_0DB                0
#define AC100_DAC_MXR_GAIN_DACR_MXR_SRC_ADCR_MINUS_6DB          BIT(8)

#define AC100_ADC_APC_CTRL_ADCR_EN_OFF                          15
#define AC100_ADC_APC_CTRL_ADCR_EN_MASK                         BIT(15)
#define AC100_ADC_APC_CTRL_ADCR_EN_DISABLED                     0
#define AC100_ADC_APC_CTRL_ADCR_EN_ENABLED                      BIT(15)
#define AC100_ADC_APC_CTRL_ADCR_GAIN_OFF                        12
#define AC100_ADC_APC_CTRL_ADCR_GAIN(v)                         (((v) & 0x7) << 12)
#define AC100_ADC_APC_CTRL_ADCL_EN_OFF                          11
#define AC100_ADC_APC_CTRL_ADCL_EN_MASK                         BIT(11)
#define AC100_ADC_APC_CTRL_ADCL_EN_DISABLED                     0
#define AC100_ADC_APC_CTRL_ADCL_EN_ENABLED                      BIT(11)
#define AC100_ADC_APC_CTRL_ADCL_GAIN_OFF                        8
#define AC100_ADC_APC_CTRL_ADCL_GAIN(v)                         (((v) & 0x7) << 8)
#define AC100_ADC_APC_CTRL_MBIAS_EN_OFF                         7
#define AC100_ADC_APC_CTRL_MBIAS_EN_MASK                        BIT(7)
#define AC100_ADC_APC_CTRL_MBIAS_EN_DISABLED                    0
#define AC100_ADC_APC_CTRL_MBIAS_EN_ENABLED                     BIT(7)
#define AC100_ADC_APC_CTRL_MMIC_BIAS_CHOPPER_EN_OFF             6
#define AC100_ADC_APC_CTRL_MMIC_BIAS_CHOPPER_EN_MASK            BIT(6)
#define AC100_ADC_APC_CTRL_MMIC_BIAS_CHOPPER_EN_DISABLED        0
#define AC100_ADC_APC_CTRL_MMIC_BIAS_CHOPPER_EN_ENABLED         BIT(6)
#define AC100_ADC_APC_CTRL_MMIC_BIAS_CHOPPER_CKS_OFF            4
#define AC100_ADC_APC_CTRL_MMIC_BIAS_CHOPPER_CKS_MASK           GENMASK(5, 4)
#define AC100_ADC_APC_CTRL_MMIC_BIAS_CHOPPER_CKS_250K           (0x0 << 4)
#define AC100_ADC_APC_CTRL_MMIC_BIAS_CHOPPER_CKS_500K           (0x1 << 4)
#define AC100_ADC_APC_CTRL_MMIC_BIAS_CHOPPER_CKS_1M             (0x2 << 4)
#define AC100_ADC_APC_CTRL_MMIC_BIAS_CHOPPER_CKS_2M             (0x3 << 4)
#define AC100_ADC_APC_CTRL_HBIAS_MODE_OFF                       2
#define AC100_ADC_APC_CTRL_HBIAS_MODE_MASK                      BIT(2)
#define AC100_ADC_APC_CTRL_HBIAS_MODE_LOAD                      0
#define AC100_ADC_APC_CTRL_HBIAS_MODE_HBIAS_EN                  BIT(2)
#define AC100_ADC_APC_CTRL_HBIAS_EN_OFF                         1
#define AC100_ADC_APC_CTRL_HBIAS_EN_MASK                        BIT(1)
#define AC100_ADC_APC_CTRL_HBIAS_EN_DISABLED                    0
#define AC100_ADC_APC_CTRL_HBIAS_EN_ENABLED                     BIT(1)
#define AC100_ADC_APC_CTRL_HBIAS_ADC_EN_OFF                     0
#define AC100_ADC_APC_CTRL_HBIAS_ADC_EN_MASK                    BIT(0)
#define AC100_ADC_APC_CTRL_HBIAS_ADC_EN_DISABLED                0
#define AC100_ADC_APC_CTRL_HBIAS_ADC_EN_ENABLED                 BIT(0)

#define AC100_ADC_SRC_ADCR_MIC1_BOOST_OFF                       13
#define AC100_ADC_SRC_ADCR_MIC1_BOOST_MASK                      BIT(13)
#define AC100_ADC_SRC_ADCR_MIC1_BOOST_DISABLED                  0
#define AC100_ADC_SRC_ADCR_MIC1_BOOST_ENABLED                   BIT(13)
#define AC100_ADC_SRC_ADCR_MIC2_BOOST_OFF                       12
#define AC100_ADC_SRC_ADCR_MIC2_BOOST_MASK                      BIT(12)
#define AC100_ADC_SRC_ADCR_MIC2_BOOST_DISABLED                  0
#define AC100_ADC_SRC_ADCR_MIC2_BOOST_ENABLED                   BIT(12)
#define AC100_ADC_SRC_ADCR_LINEINL_LINEINR_OFF                  11
#define AC100_ADC_SRC_ADCR_LINEINL_LINEINR_MASK                 BIT(11)
#define AC100_ADC_SRC_ADCR_LINEINL_LINEINR_DISABLED             0
#define AC100_ADC_SRC_ADCR_LINEINL_LINEINR_ENABLED              BIT(11)
#define AC100_ADC_SRC_ADCR_LINEINR_OFF                          10
#define AC100_ADC_SRC_ADCR_LINEINR_MASK                         BIT(10)
#define AC100_ADC_SRC_ADCR_LINEINR_DISABLED                     0
#define AC100_ADC_SRC_ADCR_LINEINR_ENABLED                      BIT(10)
#define AC100_ADC_SRC_ADCR_AUXINR_OFF                           9
#define AC100_ADC_SRC_ADCR_AUXINR_MASK                          BIT(9)
#define AC100_ADC_SRC_ADCR_AUXINR_DISABLED                      0
#define AC100_ADC_SRC_ADCR_AUXINR_ENABLED                       BIT(9)
#define AC100_ADC_SRC_ADCR_ROUTMIX_OFF                          8
#define AC100_ADC_SRC_ADCR_ROUTMIX_MASK                         BIT(8)
#define AC100_ADC_SRC_ADCR_ROUTMIX_DISABLED                     0
#define AC100_ADC_SRC_ADCR_ROUTMIX_ENABLED                      BIT(8)
#define AC100_ADC_SRC_ADCR_LOUTMIX_OFF                          7
#define AC100_ADC_SRC_ADCR_LOUTMIX_MASK                         BIT(7)
#define AC100_ADC_SRC_ADCR_LOUTMIX_DISABLED                     0
#define AC100_ADC_SRC_ADCR_LOUTMIX_ENABLED                      BIT(7)
#define AC100_ADC_SRC_ADCL_MIC1_BOOST_OFF                       6
#define AC100_ADC_SRC_ADCL_MIC1_BOOST_MASK                      BIT(6)
#define AC100_ADC_SRC_ADCL_MIC1_BOOST_DISABLED                  0
#define AC100_ADC_SRC_ADCL_MIC1_BOOST_ENABLED                   BIT(6)
#define AC100_ADC_SRC_ADCL_MIC2_BOOST_OFF                       5
#define AC100_ADC_SRC_ADCL_MIC2_BOOST_MASK                      BIT(5)
#define AC100_ADC_SRC_ADCL_MIC2_BOOST_DISABLED                  0
#define AC100_ADC_SRC_ADCL_MIC2_BOOST_ENABLED                   BIT(5)
#define AC100_ADC_SRC_ADCL_LINEINL_LINEINR_OFF                  4
#define AC100_ADC_SRC_ADCL_LINEINL_LINEINR_MASK                 BIT(4)
#define AC100_ADC_SRC_ADCL_LINEINL_LINEINR_DISABLED             0
#define AC100_ADC_SRC_ADCL_LINEINL_LINEINR_ENABLED              BIT(4)
#define AC100_ADC_SRC_ADCL_LINEINL_OFF                          3
#define AC100_ADC_SRC_ADCL_LINEINL_MASK                         BIT(3)
#define AC100_ADC_SRC_ADCL_LINEINL_DISABLED                     0
#define AC100_ADC_SRC_ADCL_LINEINL_ENABLED                      BIT(3)
#define AC100_ADC_SRC_ADCL_AUXINL_OFF                           2
#define AC100_ADC_SRC_ADCL_AUXINL_MASK                          BIT(2)
#define AC100_ADC_SRC_ADCL_AUXINL_DISABLED                      0
#define AC100_ADC_SRC_ADCL_AUXINL_ENABLED                       BIT(2)
#define AC100_ADC_SRC_ADCL_LOUTMIX_OFF                          1
#define AC100_ADC_SRC_ADCL_LOUTMIX_MASK                         BIT(1)
#define AC100_ADC_SRC_ADCL_LOUTMIX_DISABLED                     0
#define AC100_ADC_SRC_ADCL_LOUTMIX_ENABLED                      BIT(1)
#define AC100_ADC_SRC_ADCL_ROUTMIX_OFF                          0
#define AC100_ADC_SRC_ADCL_ROUTMIX_MASK                         BIT(0)
#define AC100_ADC_SRC_ADCL_ROUTMIX_DISABLED                     0
#define AC100_ADC_SRC_ADCL_ROUTMIX_ENABLED                      BIT(0)

#define AC100_ADC_SRC_BST_CTRL_MIC1AMPEN_OFF                    15
#define AC100_ADC_SRC_BST_CTRL_MIC1AMPEN_MASK                   BIT(15)
#define AC100_ADC_SRC_BST_CTRL_MIC1AMPEN_DISABLED               0
#define AC100_ADC_SRC_BST_CTRL_MIC1AMPEN_ENABLED                BIT(15)
#define AC100_ADC_SRC_BST_CTRL_MIC1BOOST_OFF                    12
#define AC100_ADC_SRC_BST_CTRL_MIC1BOOST(v)                     (((v) & 0x7) << 12)
#define AC100_ADC_SRC_BST_CTRL_MIC2AMPEN_OFF                    11
#define AC100_ADC_SRC_BST_CTRL_MIC2AMPEN_MASK                   BIT(11)
#define AC100_ADC_SRC_BST_CTRL_MIC2AMPEN_DISABLED               0
#define AC100_ADC_SRC_BST_CTRL_MIC2AMPEN_ENABLED                BIT(11)
#define AC100_ADC_SRC_BST_CTRL_MIC2BOOST_OFF                    8
#define AC100_ADC_SRC_BST_CTRL_MIC2BOOST(v)                     (((v) & 0x7) << 8)
#define AC100_ADC_SRC_BST_CTRL_MIC2SLT_OFF                      7
#define AC100_ADC_SRC_BST_CTRL_MIC2SLT_MASK                     BIT(7)
#define AC100_ADC_SRC_BST_CTRL_MIC2SLT_MIC2                     0
#define AC100_ADC_SRC_BST_CTRL_MIC2SLT_MIC3                     BIT(7)
#define AC100_ADC_SRC_BST_CTRL_LINEIN_DIFF_PREG_OFF             4
#define AC100_ADC_SRC_BST_CTRL_LINEIN_DIFF_PREG(v)              (((v) & 0x7) << 4)
#define AC100_ADC_SRC_BST_CTRL_AXI_PREG_OFF                     0
#define AC100_ADC_SRC_BST_CTRL_AXI_PREG(v)                      ((v) & 0x7)

#define AC100_OUT_MXR_DAC_A_CTRL_DAC_AR_EN_OFF                  15
#define AC100_OUT_MXR_DAC_A_CTRL_DAC_AR_EN_MASK                 BIT(15)
#define AC100_OUT_MXR_DAC_A_CTRL_DAC_AR_EN_DISABLED             0
#define AC100_OUT_MXR_DAC_A_CTRL_DAC_AR_EN_ENABLED              BIT(15)
#define AC100_OUT_MXR_DAC_A_CTRL_DAC_AL_EN_OFF                  14
#define AC100_OUT_MXR_DAC_A_CTRL_DAC_AL_EN_MASK                 BIT(14)
#define AC100_OUT_MXR_DAC_A_CTRL_DAC_AL_EN_DISABLED             0
#define AC100_OUT_MXR_DAC_A_CTRL_DAC_AL_EN_ENABLED              BIT(14)
#define AC100_OUT_MXR_DAC_A_CTRL_AL_MIX_EN_OFF                  13
#define AC100_OUT_MXR_DAC_A_CTRL_AL_MIX_EN_MASK                 BIT(13)
#define AC100_OUT_MXR_DAC_A_CTRL_AL_MIX_EN_DISABLED             0
#define AC100_OUT_MXR_DAC_A_CTRL_AL_MIX_EN_ENABLED              BIT(13)
#define AC100_OUT_MXR_DAC_A_CTRL_AR_MIX_EN_OFF                  12
#define AC100_OUT_MXR_DAC_A_CTRL_AR_MIX_EN_MASK                 BIT(12)
#define AC100_OUT_MXR_DAC_A_CTRL_AR_MIX_EN_DISABLED             0
#define AC100_OUT_MXR_DAC_A_CTRL_AR_MIX_EN_ENABLED              BIT(12)
#define AC100_OUT_MXR_DAC_A_CTRL_HP_DCRM_EN_OFF                 8
#define AC100_OUT_MXR_DAC_A_CTRL_HP_DCRM_EN(v)                  (((v) & 0xf) << 8)

#define AC100_OUT_MXR_SRC_RMIX_MIC1_BOOST_OFF                   13
#define AC100_OUT_MXR_SRC_RMIX_MIC1_BOOST_MASK                  BIT(13)
#define AC100_OUT_MXR_SRC_RMIX_MIC1_BOOST_DISABLED              0
#define AC100_OUT_MXR_SRC_RMIX_MIC1_BOOST_ENABLED               BIT(13)
#define AC100_OUT_MXR_SRC_RMIX_MIC2_BOOST_OFF                   12
#define AC100_OUT_MXR_SRC_RMIX_MIC2_BOOST_MASK                  BIT(12)
#define AC100_OUT_MXR_SRC_RMIX_MIC2_BOOST_DISABLED              0
#define AC100_OUT_MXR_SRC_RMIX_MIC2_BOOST_ENABLED               BIT(12)
#define AC100_OUT_MXR_SRC_RMIX_LINEINL_LINEINR_OFF              11
#define AC100_OUT_MXR_SRC_RMIX_LINEINL_LINEINR_MASK             BIT(11)
#define AC100_OUT_MXR_SRC_RMIX_LINEINL_LINEINR_DISABLED         0
#define AC100_OUT_MXR_SRC_RMIX_LINEINL_LINEINR_ENABLED          BIT(11)
#define AC100_OUT_MXR_SRC_RMIX_LINEINR_OFF                      10
#define AC100_OUT_MXR_SRC_RMIX_LINEINR_MASK                     BIT(10)
#define AC100_OUT_MXR_SRC_RMIX_LINEINR_DISABLED                 0
#define AC100_OUT_MXR_SRC_RMIX_LINEINR_ENABLED                  BIT(10)
#define AC100_OUT_MXR_SRC_RMIX_AUXINR_OFF                       9
#define AC100_OUT_MXR_SRC_RMIX_AUXINR_MASK                      BIT(9)
#define AC100_OUT_MXR_SRC_RMIX_AUXINR_DISABLED                  0
#define AC100_OUT_MXR_SRC_RMIX_AUXINR_ENABLED                   BIT(9)
#define AC100_OUT_MXR_SRC_RMIX_DACR_OFF                         8
#define AC100_OUT_MXR_SRC_RMIX_DACR_MASK                        BIT(8)
#define AC100_OUT_MXR_SRC_RMIX_DACR_DISABLED                    0
#define AC100_OUT_MXR_SRC_RMIX_DACR_ENABLED                     BIT(8)
#define AC100_OUT_MXR_SRC_RMIX_DACL_OFF                         7
#define AC100_OUT_MXR_SRC_RMIX_DACL_MASK                        BIT(7)
#define AC100_OUT_MXR_SRC_RMIX_DACL_DISABLED                    0
#define AC100_OUT_MXR_SRC_RMIX_DACL_ENABLED                     BIT(7)
#define AC100_OUT_MXR_SRC_LMIX_MIC1_BOOST_OFF                   6
#define AC100_OUT_MXR_SRC_LMIX_MIC1_BOOST_MASK                  BIT(6)
#define AC100_OUT_MXR_SRC_LMIX_MIC1_BOOST_DISABLED              0
#define AC100_OUT_MXR_SRC_LMIX_MIC1_BOOST_ENABLED               BIT(6)
#define AC100_OUT_MXR_SRC_LMIX_MIC2_BOOST_OFF                   5
#define AC100_OUT_MXR_SRC_LMIX_MIC2_BOOST_MASK                  BIT(5)
#define AC100_OUT_MXR_SRC_LMIX_MIC2_BOOST_DISABLED              0
#define AC100_OUT_MXR_SRC_LMIX_MIC2_BOOST_ENABLED               BIT(5)
#define AC100_OUT_MXR_SRC_LMIX_LINEINL_LINEINR_OFF              4
#define AC100_OUT_MXR_SRC_LMIX_LINEINL_LINEINR_MASK             BIT(4)
#define AC100_OUT_MXR_SRC_LMIX_LINEINL_LINEINR_DISABLED         0
#define AC100_OUT_MXR_SRC_LMIX_LINEINL_LINEINR_ENABLED          BIT(4)
#define AC100_OUT_MXR_SRC_LMIX_LINEINL_OFF                      3
#define AC100_OUT_MXR_SRC_LMIX_LINEINL_MASK                     BIT(3)
#define AC100_OUT_MXR_SRC_LMIX_LINEINL_DISABLED                 0
#define AC100_OUT_MXR_SRC_LMIX_LINEINL_ENABLED                  BIT(3)
#define AC100_OUT_MXR_SRC_LMIX_AUXINL_OFF                       2
#define AC100_OUT_MXR_SRC_LMIX_AUXINL_MASK                      BIT(2)
#define AC100_OUT_MXR_SRC_LMIX_AUXINL_DISABLED                  0
#define AC100_OUT_MXR_SRC_LMIX_AUXINL_ENABLED                   BIT(2)
#define AC100_OUT_MXR_SRC_LMIX_DACL_OFF                         1
#define AC100_OUT_MXR_SRC_LMIX_DACL_MASK                        BIT(1)
#define AC100_OUT_MXR_SRC_LMIX_DACL_DISABLED                    0
#define AC100_OUT_MXR_SRC_LMIX_DACL_ENABLED                     BIT(1)
#define AC100_OUT_MXR_SRC_LMIX_DACR_OFF                         0
#define AC100_OUT_MXR_SRC_LMIX_DACR_MASK                        BIT(0)
#define AC100_OUT_MXR_SRC_LMIX_DACR_DISABLED                    0
#define AC100_OUT_MXR_SRC_LMIX_DACR_ENABLED                     BIT(0)

#define AC100_OUT_MXR_SRC_BST_HMICBIAS_VOLTAGE_OFF              14
#define AC100_OUT_MXR_SRC_BST_HMICBIAS_VOLTAGE_MASK             GENMASK(15, 14)
#define AC100_OUT_MXR_SRC_BST_HMICBIAS_VOLTAGE_1_88V            (0x0 << 14)
#define AC100_OUT_MXR_SRC_BST_HMICBIAS_VOLTAGE_2_09V            (0x1 << 14)
#define AC100_OUT_MXR_SRC_BST_HMICBIAS_VOLTAGE_2_33V            (0x2 << 14)
#define AC100_OUT_MXR_SRC_BST_HMICBIAS_VOLTAGE_2_50V            (0x3 << 14)
#define AC100_OUT_MXR_SRC_BST_MMICBIAS_VOLTAGE_OFF              12
#define AC100_OUT_MXR_SRC_BST_MMICBIAS_VOLTAGE_MASK             GENMASK(13, 12)
#define AC100_OUT_MXR_SRC_BST_MMICBIAS_VOLTAGE_1_88V            (0x0 << 12)
#define AC100_OUT_MXR_SRC_BST_MMICBIAS_VOLTAGE_2_09V            (0x1 << 12)
#define AC100_OUT_MXR_SRC_BST_MMICBIAS_VOLTAGE_2_33V            (0x2 << 12)
#define AC100_OUT_MXR_SRC_BST_MMICBIAS_VOLTAGE_2_50V            (0x3 << 12)
#define AC100_OUT_MXR_SRC_BST_AX_GAIN_OFF                       9
#define AC100_OUT_MXR_SRC_BST_AX_GAIN(v)                        (((v) & 0x7) << 9)
#define AC100_OUT_MXR_SRC_BST_MIC1_GAIN_OFF                     6
#define AC100_OUT_MXR_SRC_BST_MIC1_GAIN(v)                      (((v) & 0x7) << 6)
#define AC100_OUT_MXR_SRC_BST_MIC2_GAIN_OFF                     3
#define AC100_OUT_MXR_SRC_BST_MIC2_GAIN(v)                      (((v) & 0x7) << 3)
#define AC100_OUT_MXR_SRC_BST_LINEIN_GAIN_OFF                   0
#define AC100_OUT_MXR_SRC_BST_LINEIN_GAIN(v)                    ((v) & 0x7)

#define AC100_HPOUT_CTRL_RIGHT_SRC_OFF                          15
#define AC100_HPOUT_CTRL_RIGHT_SRC_MASK                         BIT(15)
#define AC100_HPOUT_CTRL_RIGHT_SRC_DACR                         0
#define AC100_HPOUT_CTRL_RIGHT_SRC_RAMIX                        BIT(15)
#define AC100_HPOUT_CTRL_LEFT_SRC_OFF                           14
#define AC100_HPOUT_CTRL_LEFT_SRC_MASK                          BIT(14)
#define AC100_HPOUT_CTRL_LEFT_SRC_DACL                          0
#define AC100_HPOUT_CTRL_LEFT_SRC_LAMIX                         BIT(14)
#define AC100_HPOUT_CTRL_RIGHT_PA_MUTE_OFF                      13
#define AC100_HPOUT_CTRL_RIGHT_PA_MUTE_MASK                     BIT(13)
#define AC100_HPOUT_CTRL_RIGHT_PA_MUTE_MUTE                     0
#define AC100_HPOUT_CTRL_RIGHT_PA_MUTE_NOT_MUTE                 BIT(13)
#define AC100_HPOUT_CTRL_LEFT_PA_MUTE_OFF                       12
#define AC100_HPOUT_CTRL_LEFT_PA_MUTE_MASK                      BIT(12)
#define AC100_HPOUT_CTRL_LEFT_PA_MUTE_MUTE                      0
#define AC100_HPOUT_CTRL_LEFT_PA_MUTE_NOT_MUTE                  BIT(12)
#define AC100_HPOUT_CTRL_PA_EN_OFF                              11
#define AC100_HPOUT_CTRL_PA_EN_MASK                             BIT(11)
#define AC100_HPOUT_CTRL_PA_EN_DISABLED                         0
#define AC100_HPOUT_CTRL_PA_EN_ENABLED                          BIT(11)
#define AC100_HPOUT_CTRL_VOLUME_OFF                             4
#define AC100_HPOUT_CTRL_VOLUME(v)                              (((v) & 0x3f) << 4)
#define AC100_HPOUT_CTRL_STARTUP_DELAY_OFF                      2
#define AC100_HPOUT_CTRL_STARTUP_DELAY_MASK                     GENMASK(3, 2)
#define AC100_HPOUT_CTRL_STARTUP_DELAY_4ms                      (0x0 << 2)
#define AC100_HPOUT_CTRL_STARTUP_DELAY_8ms                      (0x1 << 2)
#define AC100_HPOUT_CTRL_STARTUP_DELAY_16ms                     (0x2 << 2)
#define AC100_HPOUT_CTRL_STARTUP_DELAY_32ms                     (0x3 << 2)
#define AC100_HPOUT_CTRL_OUTPUT_CURRENT_OFF                     0
#define AC100_HPOUT_CTRL_OUTPUT_CURRENT(v)                      ((v) & 0x3)

#define AC100_ERPOUT_CTRL_RAMP_TIME_OFF                         11
#define AC100_ERPOUT_CTRL_RAMP_TIME_MASK                        GENMASK(12, 11)
#define AC100_ERPOUT_CTRL_RAMP_TIME_256ms                       (0x0 << 11)
#define AC100_ERPOUT_CTRL_RAMP_TIME_512ms                       (0x1 << 11)
#define AC100_ERPOUT_CTRL_RAMP_TIME_640ms                       (0x2 << 11)
#define AC100_ERPOUT_CTRL_RAMP_TIME_768ms                       (0x3 << 11)
#define AC100_ERPOUT_CTRL_OUT_CURRENT_OFF                       9
#define AC100_ERPOUT_CTRL_OUT_CURRENT(v)                        (((v) & 0x3) << 9)
#define AC100_ERPOUT_CTRL_INPUT_SOURCE_OFF                      7
#define AC100_ERPOUT_CTRL_INPUT_SOURCE_MASK                     GENMASK(8, 7)
#define AC100_ERPOUT_CTRL_INPUT_SOURCE_DACR                     (0x0 << 7)
#define AC100_ERPOUT_CTRL_INPUT_SOURCE_DACL                     (0x1 << 7)
#define AC100_ERPOUT_CTRL_INPUT_SOURCE_RAMIX                    (0x2 << 7)
#define AC100_ERPOUT_CTRL_INPUT_SOURCE_LAMIX                    (0x3 << 7)
#define AC100_ERPOUT_CTRL_MUTE_OFF                              6
#define AC100_ERPOUT_CTRL_MUTE_MASK                             BIT(6)
#define AC100_ERPOUT_CTRL_MUTE_MUTE                             0
#define AC100_ERPOUT_CTRL_MUTE_NOT_MUTE                         BIT(6)
#define AC100_ERPOUT_CTRL_PA_EN_OFF                             5
#define AC100_ERPOUT_CTRL_PA_EN_MASK                            BIT(5)
#define AC100_ERPOUT_CTRL_PA_EN_DISABLED                        0
#define AC100_ERPOUT_CTRL_PA_EN_ENABLED                         BIT(5)
#define AC100_ERPOUT_CTRL_VOLUME_OFF                            0
#define AC100_ERPOUT_CTRL_VOLUME(v)                             ((v) & 0x1f)

#define AC100_SPKOUT_CTRL_RIGHT_SRC_OFF                         12
#define AC100_SPKOUT_CTRL_RIGHT_SRC_MASK                        BIT(12)
#define AC100_SPKOUT_CTRL_RIGHT_SRC_MIXR                        0
#define AC100_SPKOUT_CTRL_RIGHT_SRC_MIXL_MIXR                   BIT(12)
#define AC100_SPKOUT_CTRL_RIGHT_INV_EN_OFF                      11
#define AC100_SPKOUT_CTRL_RIGHT_INV_EN_MASK                     BIT(11)
#define AC100_SPKOUT_CTRL_RIGHT_INV_EN_DISABLED                 0
#define AC100_SPKOUT_CTRL_RIGHT_INV_EN_ENABLED                  BIT(11)
#define AC100_SPKOUT_CTRL_RIGHT_EN_OFF                          9
#define AC100_SPKOUT_CTRL_RIGHT_EN_MASK                         BIT(9)
#define AC100_SPKOUT_CTRL_RIGHT_EN_DISABLED                     0
#define AC100_SPKOUT_CTRL_RIGHT_EN_ENABLED                      BIT(9)
#define AC100_SPKOUT_CTRL_LEFT_SRC_OFF                          8
#define AC100_SPKOUT_CTRL_LEFT_SRC_MASK                         BIT(8)
#define AC100_SPKOUT_CTRL_LEFT_SRC_MIXL                         0
#define AC100_SPKOUT_CTRL_LEFT_SRC_MIXL_MIXR                    BIT(8)
#define AC100_SPKOUT_CTRL_LEFT_INV_EN_OFF                       7
#define AC100_SPKOUT_CTRL_LEFT_INV_EN_MASK                      BIT(7)
#define AC100_SPKOUT_CTRL_LEFT_INV_EN_DISABLED                  0
#define AC100_SPKOUT_CTRL_LEFT_INV_EN_ENABLED                   BIT(7)
#define AC100_SPKOUT_CTRL_LEFT_EN_OFF                           5
#define AC100_SPKOUT_CTRL_LEFT_EN_MASK                          BIT(5)
#define AC100_SPKOUT_CTRL_LEFT_EN_DISABLED                      0
#define AC100_SPKOUT_CTRL_LEFT_EN_ENABLED                       BIT(5)
#define AC100_SPKOUT_CTRL_VOLUME_OFF                            0
#define AC100_SPKOUT_CTRL_VOLUME(v)                             ((v) & 0x1f)

#define AC100_LINEOUT_CTRL_LINEOUT_GAIN_OFF                     5
#define AC100_LINEOUT_CTRL_LINEOUT_GAIN(v)                      (((v) & 0x7) << 5)
#define AC100_LINEOUT_CTRL_LINEOUT_EN_OFF                       4
#define AC100_LINEOUT_CTRL_LINEOUT_EN_MASK                      BIT(4)
#define AC100_LINEOUT_CTRL_LINEOUT_EN_DISABLED                  0
#define AC100_LINEOUT_CTRL_LINEOUT_EN_ENABLED                   BIT(4)
#define AC100_LINEOUT_CTRL_LINEOUT_S0_OFF                       3
#define AC100_LINEOUT_CTRL_LINEOUT_S0_MASK                      BIT(3)
#define AC100_LINEOUT_CTRL_LINEOUT_S0_MUTE                      0
#define AC100_LINEOUT_CTRL_LINEOUT_S0_ON                        BIT(3)
#define AC100_LINEOUT_CTRL_LINEOUT_S1_OFF                       2
#define AC100_LINEOUT_CTRL_LINEOUT_S1_MASK                      BIT(2)
#define AC100_LINEOUT_CTRL_LINEOUT_S1_MUTE                      0
#define AC100_LINEOUT_CTRL_LINEOUT_S1_ON                        BIT(2)
#define AC100_LINEOUT_CTRL_LINEOUT_S2_OFF                       1
#define AC100_LINEOUT_CTRL_LINEOUT_S2_MASK                      BIT(1)
#define AC100_LINEOUT_CTRL_LINEOUT_S2_MUTE                      0
#define AC100_LINEOUT_CTRL_LINEOUT_S2_ON                        BIT(1)
#define AC100_LINEOUT_CTRL_LINEOUT_S3_OFF                       0
#define AC100_LINEOUT_CTRL_LINEOUT_S3_MASK                      BIT(0)
#define AC100_LINEOUT_CTRL_LINEOUT_S3_MUTE                      0
#define AC100_LINEOUT_CTRL_LINEOUT_S3_ON                        BIT(0)

#define AC100_DAC_DAP_H_G_OFF_HIGH13_BITS_OFF                   0
#define AC100_DAC_DAP_H_G_OFF_HIGH13_BITS(v)                    ((v) & 0x1fff)

#define AC100_DAC_DAP_L_G_OFF_LOW16_BITS_OFF                    0
#define AC100_DAC_DAP_L_G_OFF_LOW16_BITS(v)                     ((v) & 0xffff)

#define AC100_DAC_DAP_OPT_DRC_GAIN_OFF                          5
#define AC100_DAC_DAP_OPT_DRC_GAIN_MASK                         BIT(5)
#define AC100_DAC_DAP_OPT_DRC_GAIN_1                            0
#define AC100_DAC_DAP_OPT_DRC_GAIN_0                            BIT(5)
#define AC100_DAC_DAP_OPT_HYSTHERESIS_OFF                       0
#define AC100_DAC_DAP_OPT_HYSTHERESIS(v)                        ((v) & 0x1f)

#define AC100_ADC_DAP_ENA_I2S1_ADCL0_AGC_EN_OFF                 15
#define AC100_ADC_DAP_ENA_I2S1_ADCL0_AGC_EN_MASK                BIT(15)
#define AC100_ADC_DAP_ENA_I2S1_ADCL0_AGC_EN_DISABLED            0
#define AC100_ADC_DAP_ENA_I2S1_ADCL0_AGC_EN_ENABLED             BIT(15)
#define AC100_ADC_DAP_ENA_I2S1_ADCR0_AGC_EN_OFF                 14
#define AC100_ADC_DAP_ENA_I2S1_ADCR0_AGC_EN_MASK                BIT(14)
#define AC100_ADC_DAP_ENA_I2S1_ADCR0_AGC_EN_DISABLED            0
#define AC100_ADC_DAP_ENA_I2S1_ADCR0_AGC_EN_ENABLED             BIT(14)
#define AC100_ADC_DAP_ENA_I2S1_ADCL1_AGC_EN_OFF                 13
#define AC100_ADC_DAP_ENA_I2S1_ADCL1_AGC_EN_MASK                BIT(13)
#define AC100_ADC_DAP_ENA_I2S1_ADCL1_AGC_EN_DISABLED            0
#define AC100_ADC_DAP_ENA_I2S1_ADCL1_AGC_EN_ENABLED             BIT(13)
#define AC100_ADC_DAP_ENA_I2S1_ADCR1_AGC_EN_OFF                 12
#define AC100_ADC_DAP_ENA_I2S1_ADCR1_AGC_EN_MASK                BIT(12)
#define AC100_ADC_DAP_ENA_I2S1_ADCR1_AGC_EN_DISABLED            0
#define AC100_ADC_DAP_ENA_I2S1_ADCR1_AGC_EN_ENABLED             BIT(12)
#define AC100_ADC_DAP_ENA_I2S2_ADCL_AGC_EN_OFF                  11
#define AC100_ADC_DAP_ENA_I2S2_ADCL_AGC_EN_MASK                 BIT(11)
#define AC100_ADC_DAP_ENA_I2S2_ADCL_AGC_EN_DISABLED             0
#define AC100_ADC_DAP_ENA_I2S2_ADCL_AGC_EN_ENABLED              BIT(11)
#define AC100_ADC_DAP_ENA_I2S2_ADCR_AGC_EN_OFF                  10
#define AC100_ADC_DAP_ENA_I2S2_ADCR_AGC_EN_MASK                 BIT(10)
#define AC100_ADC_DAP_ENA_I2S2_ADCR_AGC_EN_DISABLED             0
#define AC100_ADC_DAP_ENA_I2S2_ADCR_AGC_EN_ENABLED              BIT(10)
#define AC100_ADC_DAP_ENA_I2S2_DACL_AGC_EN_OFF                  9
#define AC100_ADC_DAP_ENA_I2S2_DACL_AGC_EN_MASK                 BIT(9)
#define AC100_ADC_DAP_ENA_I2S2_DACL_AGC_EN_DISABLED             0
#define AC100_ADC_DAP_ENA_I2S2_DACL_AGC_EN_ENABLED              BIT(9)
#define AC100_ADC_DAP_ENA_I2S2_DACR_AGC_EN_OFF                  8
#define AC100_ADC_DAP_ENA_I2S2_DACR_AGC_EN_MASK                 BIT(8)
#define AC100_ADC_DAP_ENA_I2S2_DACR_AGC_EN_DISABLED             0
#define AC100_ADC_DAP_ENA_I2S2_DACR_AGC_EN_ENABLED              BIT(8)
#define AC100_ADC_DAP_ENA_ADCL_AGC_EN_OFF                       7
#define AC100_ADC_DAP_ENA_ADCL_AGC_EN_MASK                      BIT(7)
#define AC100_ADC_DAP_ENA_ADCL_AGC_EN_DISABLED                  0
#define AC100_ADC_DAP_ENA_ADCL_AGC_EN_ENABLED                   BIT(7)
#define AC100_ADC_DAP_ENA_ADCR_AGC_EN_OFF                       6
#define AC100_ADC_DAP_ENA_ADCR_AGC_EN_MASK                      BIT(6)
#define AC100_ADC_DAP_ENA_ADCR_AGC_EN_DISABLED                  0
#define AC100_ADC_DAP_ENA_ADCR_AGC_EN_ENABLED                   BIT(6)

#define AC100_DAC_DAP_ENA_I2S1_DAC0_DRC_EN_OFF                  15
#define AC100_DAC_DAP_ENA_I2S1_DAC0_DRC_EN_MASK                 BIT(15)
#define AC100_DAC_DAP_ENA_I2S1_DAC0_DRC_EN_DISABLED             0
#define AC100_DAC_DAP_ENA_I2S1_DAC0_DRC_EN_ENABLED              BIT(15)
#define AC100_DAC_DAP_ENA_I2S1_DAC1_DRC_EN_OFF                  13
#define AC100_DAC_DAP_ENA_I2S1_DAC1_DRC_EN_MASK                 BIT(13)
#define AC100_DAC_DAP_ENA_I2S1_DAC1_DRC_EN_DISABLED             0
#define AC100_DAC_DAP_ENA_I2S1_DAC1_DRC_EN_ENABLED              BIT(13)
#define AC100_DAC_DAP_ENA_I2S2_DAC_DRC_EN_OFF                   11
#define AC100_DAC_DAP_ENA_I2S2_DAC_DRC_EN_MASK                  BIT(11)
#define AC100_DAC_DAP_ENA_I2S2_DAC_DRC_EN_DISABLED              0
#define AC100_DAC_DAP_ENA_I2S2_DAC_DRC_EN_ENABLED               BIT(11)
#define AC100_DAC_DAP_ENA_DAC_DRC_EN_OFF                        7
#define AC100_DAC_DAP_ENA_DAC_DRC_EN_MASK                       BIT(7)
#define AC100_DAC_DAP_ENA_DAC_DRC_EN_DISABLED                   0
#define AC100_DAC_DAP_ENA_DAC_DRC_EN_ENABLED                    BIT(7)

#define AC100_SRC1_CTRL1_ENABLE_OFF                             15
#define AC100_SRC1_CTRL1_ENABLE_MASK                            BIT(15)
#define AC100_SRC1_CTRL1_ENABLE_DISABLED                        0
#define AC100_SRC1_CTRL1_ENABLE_ENABLED                         BIT(15)
#define AC100_SRC1_CTRL1_LOCK_STATUS_OFF                        14
#define AC100_SRC1_CTRL1_LOCK_STATUS_MASK                       BIT(14)
#define AC100_SRC1_CTRL1_LOCK_STATUS_NOT_LOCKED                 0
#define AC100_SRC1_CTRL1_LOCK_STATUS_LOCKED                     BIT(14)
#define AC100_SRC1_CTRL1_FIFO_OVERFLOW_OFF                      13
#define AC100_SRC1_CTRL1_FIFO_OVERFLOW_MASK                     BIT(13)
#define AC100_SRC1_CTRL1_FIFO_OVERFLOW_NORMAL                   0
#define AC100_SRC1_CTRL1_FIFO_OVERFLOW_OVERFLOWED               BIT(13)
#define AC100_SRC1_CTRL1_FIFO_LEVEL_HIGH3_OFF                   10
#define AC100_SRC1_CTRL1_FIFO_LEVEL_HIGH3(v)                    (((v) & 0x7) << 10)
#define AC100_SRC1_CTRL1_RATIO_VALUE_HIGH10_OFF                 0
#define AC100_SRC1_CTRL1_RATIO_VALUE_HIGH10(v)                  ((v) & 0x3ff)

#define AC100_SRC1_CTRL2_RATIO_VALUE_LOW16_OFF                  0
#define AC100_SRC1_CTRL2_RATIO_VALUE_LOW16(v)                   ((v) & 0xffff)

#define AC100_SRC1_CTRL3_FIFO_LEVEL_LOW6_OFF                    10
#define AC100_SRC1_CTRL3_FIFO_LEVEL_LOW6(v)                     (((v) & 0x3f) << 10)
#define AC100_SRC1_CTRL3_RATIO_VALUE_HIGH10_OFF                 0
#define AC100_SRC1_CTRL3_RATIO_VALUE_HIGH10(v)                  ((v) & 0x3ff)

#define AC100_SRC1_CTRL4_RATIO_VALUE_LOW16_OFF                  0
#define AC100_SRC1_CTRL4_RATIO_VALUE_LOW16(v)                   ((v) & 0xffff)

#define AC100_SRC2_CTRL1_ENABLE_OFF                             15
#define AC100_SRC2_CTRL1_ENABLE_MASK                            BIT(15)
#define AC100_SRC2_CTRL1_ENABLE_DISABLED                        0
#define AC100_SRC2_CTRL1_ENABLE_ENABLED                         BIT(15)
#define AC100_SRC2_CTRL1_LOCK_STATUS_OFF                        14
#define AC100_SRC2_CTRL1_LOCK_STATUS_MASK                       BIT(14)
#define AC100_SRC2_CTRL1_LOCK_STATUS_NOT_LOCKED                 0
#define AC100_SRC2_CTRL1_LOCK_STATUS_LOCKED                     BIT(14)
#define AC100_SRC2_CTRL1_FIFO_OVERFLOW_OFF                      13
#define AC100_SRC2_CTRL1_FIFO_OVERFLOW_MASK                     BIT(13)
#define AC100_SRC2_CTRL1_FIFO_OVERFLOW_NORMAL                   0
#define AC100_SRC2_CTRL1_FIFO_OVERFLOW_OVERFLOWED               BIT(13)
#define AC100_SRC2_CTRL1_FIFO_LEVEL_HIGH3_OFF                   10
#define AC100_SRC2_CTRL1_FIFO_LEVEL_HIGH3(v)                    (((v) & 0x7) << 10)
#define AC100_SRC2_CTRL1_RATIO_VALUE_HIGH10_OFF                 0
#define AC100_SRC2_CTRL1_RATIO_VALUE_HIGH10(v)                  ((v) & 0x3ff)

#define AC100_SRC2_CTRL2_RATIO_VALUE_LOW16_OFF                  0
#define AC100_SRC2_CTRL2_RATIO_VALUE_LOW16(v)                   ((v) & 0xffff)

#define AC100_SRC2_CTRL3_FIFO_LEVEL_LOW6_OFF                    10
#define AC100_SRC2_CTRL3_FIFO_LEVEL_LOW6(v)                     (((v) & 0x3f) << 10)
#define AC100_SRC2_CTRL3_RATIO_VALUE_HIGH10_OFF                 0
#define AC100_SRC2_CTRL3_RATIO_VALUE_HIGH10(v)                  ((v) & 0x3ff)

#define AC100_SRC2_CTRL4_RATIO_VALUE_LOW16_OFF                  0
#define AC100_SRC2_CTRL4_RATIO_VALUE_LOW16(v)                   ((v) & 0xffff)

