#ifndef __MLX90520_H
#define __MLX90520_H

#include "MyProject.h"

#define MLX90520_CPR_FROM_VDP(vdp)   ((uint32_t)(vdp) * 65536u)

#define MLX90520_DEV_INFO_EXPECTED   0x016198AAu

#define MLX90520_CMD_READ            ((uint8_t)0xCCu)
#define MLX90520_CMD_WRITE           ((uint8_t)0x78u)
#define MLX90520_CMD_FRAME_START     ((uint8_t)0x34u)

#define MLX90520_EE_START            ((uint16_t)0x0200u)
#define MLX90520_EE_END              ((uint16_t)0x02BCu)
#define MLX90520_EE_SIZE_WORDS       ((uint16_t)((MLX90520_EE_END - MLX90520_EE_START) / 2u))

#define MLX90520_REG_RST_CONTROL     ((uint16_t)0x0004u)
#define MLX90520_REG_EEPROM_CONTROL  ((uint16_t)0x0010u)
#define MLX90520_REG_ER_NVM_SR_CRC_DED ((uint16_t)0x0038u)
#define MLX90520_REG_IN_APPLICATION  ((uint16_t)0x003Eu)
#define MLX90520_REG_CRC             ((uint16_t)0x0040u)
#define MLX90520_REG_CRC_STRT_CALC   ((uint16_t)0x0042u)
#define MLX90520_REG_DSP_AGC_GAIN    ((uint16_t)0x004Au)
#define MLX90520_REG_DSP_DCCALIB     ((uint16_t)0x0050u)
#define MLX90520_REG_LIN_PHASE_A     ((uint16_t)0x005Au)
#define MLX90520_REG_LIN_PHASE_B     ((uint16_t)0x005Cu)
#define MLX90520_REG_SPEED_LO_A      ((uint16_t)0x005Eu)
#define MLX90520_REG_SPEED_LO_B      ((uint16_t)0x0060u)
#define MLX90520_REG_SPEED_HI        ((uint16_t)0x0062u)
#define MLX90520_REG_ACC_A           ((uint16_t)0x0064u)
#define MLX90520_REG_ACC_B           ((uint16_t)0x0066u)
#define MLX90520_REG_FESUM           ((uint16_t)0x006Au)
#define MLX90520_REG_SSI             ((uint16_t)0x006Cu)
#define MLX90520_REG_SC_FC1          ((uint16_t)0x006Eu)
#define MLX90520_REG_SC_FC2          ((uint16_t)0x0070u)
#define MLX90520_REG_VRES            ((uint16_t)0x0072u)
#define MLX90520_REG_DSP_DFT_KEY     ((uint16_t)0x0074u)
#define MLX90520_REG_PCNT            ((uint16_t)0x00A0u)

#define MLX90520_REG_CEE_DC01_CONST  ((uint16_t)0x00FEu)
#define MLX90520_REG_CEE_DC12_CONST  ((uint16_t)0x0100u)
#define MLX90520_REG_CEE_DC20_CONST  ((uint16_t)0x0102u)
#define MLX90520_REG_CEE_DE_XXX      ((uint16_t)0x0188u)
#define MLX90520_REG_DE_NVM_SR       ((uint16_t)0x018Cu)

#define MLX90520_REG_PROTOCOL        ((uint16_t)0x0202u)
#define MLX90520_REG_DC01A           ((uint16_t)0x0204u)
#define MLX90520_REG_DC12A           ((uint16_t)0x0206u)
#define MLX90520_REG_DC20A           ((uint16_t)0x0208u)
#define MLX90520_REG_DC01B           ((uint16_t)0x020Au)
#define MLX90520_REG_DC12B           ((uint16_t)0x020Cu)
#define MLX90520_REG_DC20B           ((uint16_t)0x020Eu)
#define MLX90520_REG_AGC_GAIN_MAX    ((uint16_t)0x0210u)
#define MLX90520_REG_VERNIER         ((uint16_t)0x0240u)
#define MLX90520_REG_VERNIER_VM      ((uint16_t)0x0242u)
#define MLX90520_REG_FC_CFG          ((uint16_t)0x026Eu)
#define MLX90520_REG_SPI_FADDR01     ((uint16_t)0x028Eu)
#define MLX90520_REG_SPI_FADDR23     ((uint16_t)0x0290u)
#define MLX90520_REG_SPI_FRAME       ((uint16_t)0x0292u)
#define MLX90520_REG_EE_CUS_CRC      ((uint16_t)0x02BAu)
#define MLX90520_REG_DEV_INFO        ((uint16_t)0x02ECu)

typedef enum {
    MLX90520_OK = 0,
    MLX90520_SPI_ERROR,
    MLX90520_DEV_ID_ERROR,
    MLX90520_FS_ERROR,
    MLX90520_CRC_ERROR,
    MLX90520_ARG_ERROR,
    MLX90520_VERIFY_ERROR
} MLX90520_Status_t;

typedef struct {
    uint8_t fs;
    uint16_t fc1;
    uint16_t fc2;
} MLX90520_Frame_t;

typedef struct {
    uint16_t protocol;
    uint16_t vernier;
    uint16_t vernier_vm;
    uint16_t fc_cfg;
    uint16_t spi_faddr01;
    uint16_t spi_faddr23;
    uint16_t spi_frame;
    uint16_t sc_fc1;
    uint16_t sc_fc2;
} MLX90520_ConfigSnapshot_t;

void MLX90520_SPI3_Init(void);
MLX90520_Status_t MLX90520_CheckDevice(void);

MLX90520_Status_t MLX90520_ReadWords(uint16_t addr, uint16_t *data, uint16_t n);
MLX90520_Status_t MLX90520_WriteWords(uint16_t addr, const uint16_t *data, uint16_t n, uint8_t read_check);
MLX90520_Status_t MLX90520_ReadReg(uint16_t addr, uint16_t *value);
MLX90520_Status_t MLX90520_WriteReg(uint16_t addr, uint16_t value, uint8_t read_check);
MLX90520_Status_t MLX90520_UpdateReg(uint16_t addr, uint16_t mask, uint16_t value, uint8_t read_check);
MLX90520_Status_t MLX90520_ReadDeviceInfo(uint32_t *dev_info);
MLX90520_Status_t MLX90520_ReadConfigSnapshot(MLX90520_ConfigSnapshot_t *cfg);

MLX90520_Status_t MLX90520_StartFrameRead(void);
void MLX90520_StopFrameRead(void);

MLX90520_Status_t MLX90520_ReadFrame2(MLX90520_Frame_t *frame);
MLX90520_Status_t MLX90520_ReadRaw16(uint16_t *raw16);
MLX90520_Status_t MLX90520_ReadRaw22(uint32_t *raw22);

#endif /* __MLX90520_H */
