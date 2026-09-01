#include "mlx90520.h"

#define MLX90520_READ_FS           1u
#define MLX90520_READ_CRC          1u
#define MLX90520_MAX_WORD_ADDR     0x01FFu

// 官方 example 里把 SPI_FRFS 配成 0xA；如果你的芯片还是 datasheet 默认值，可能要改成 0x50。
#define MLX90520_FS_START          0xA0u

#define MLX90520_CS_L()            GPIO_ResetBits(GPIOB, GPIO_Pin_3)
#define MLX90520_CS_H()            GPIO_SetBits(GPIOB, GPIO_Pin_3)

static uint8_t mlx90520_frame_started = 0u;
static uint8_t mlx90520_next_fs = MLX90520_FS_START;

static void mlx90520_cs_delay(void)
{
    for (volatile uint16_t i = 0; i < 80u; ++i) {
        __NOP();
    }
}

static uint8_t mlx90520_spi_xfer(uint8_t tx)
{
    while (SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_TXE) == RESET);
    SPI_I2S_SendData(SPI3, tx);
    while (SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_RXNE) == RESET);
    return (uint8_t)SPI_I2S_ReceiveData(SPI3);
}

static void mlx90520_spi_wait_idle(void)
{
    while (SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_BSY) == SET);
}

static void mlx90520_abort_frame_read(void)
{
    mlx90520_spi_wait_idle();
    mlx90520_cs_delay();
    MLX90520_CS_H();
    mlx90520_frame_started = 0u;
    mlx90520_next_fs = MLX90520_FS_START;
}

static uint8_t mlx90520_crc8_ccitt(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0xffu;

    for (uint8_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8u; ++b) {
            crc = (crc & 0x80u) ? (uint8_t)((crc << 1) ^ 0x07u) : (uint8_t)(crc << 1);
        }
    }

    return crc;
}

static MLX90520_Status_t mlx90520_check_rw_args(uint16_t addr, const void *data, uint16_t n)
{
    uint16_t word_addr;

    if ((data == 0) || (n == 0u) || (addr & 1u)) {
        return MLX90520_ARG_ERROR;
    }

    word_addr = addr >> 1;
    if ((word_addr + n - 1u) > MLX90520_MAX_WORD_ADDR) {
        return MLX90520_ARG_ERROR;
    }

    return MLX90520_OK;
}

MLX90520_Status_t MLX90520_ReadWords(uint16_t addr, uint16_t *data, uint16_t n)
{
    uint16_t word_addr;
    uint8_t cmd;
    uint8_t addr_lsb;
    uint8_t rx0;
    uint8_t cmd_echo;
    uint8_t addr_echo;

    MLX90520_Status_t st = mlx90520_check_rw_args(addr, data, n);
    if (st != MLX90520_OK) {
        return st;
    }

    if (mlx90520_frame_started) {
        MLX90520_StopFrameRead();
    }

    word_addr = addr >> 1;
    cmd = (uint8_t)(MLX90520_CMD_READ | ((word_addr & 0x100u) >> 8));
    addr_lsb = (uint8_t)word_addr;

    MLX90520_CS_L();
    mlx90520_cs_delay();

    rx0 = mlx90520_spi_xfer(cmd);
    cmd_echo = mlx90520_spi_xfer(addr_lsb);
    addr_echo = mlx90520_spi_xfer(0x00u);
    (void)rx0;

    for (uint16_t i = 0; i < n; ++i) {
        uint8_t msb = mlx90520_spi_xfer(0x00u);
        uint8_t lsb = mlx90520_spi_xfer(0x00u);
        data[i] = ((uint16_t)msb << 8) | lsb;
    }

    mlx90520_spi_wait_idle();
    mlx90520_cs_delay();
    MLX90520_CS_H();

    if ((cmd_echo != cmd) || (addr_echo != addr_lsb)) {
        return MLX90520_SPI_ERROR;
    }

    return MLX90520_OK;
}

MLX90520_Status_t MLX90520_WriteWords(uint16_t addr, const uint16_t *data, uint16_t n, uint8_t read_check)
{
    uint16_t word_addr;
    uint8_t cmd;
    uint8_t addr_lsb;
    uint8_t rx0;
    uint8_t cmd_echo;
    uint8_t addr_echo;

    MLX90520_Status_t st = mlx90520_check_rw_args(addr, data, n);
    if (st != MLX90520_OK) {
        return st;
    }

    if (mlx90520_frame_started) {
        MLX90520_StopFrameRead();
    }

    word_addr = addr >> 1;
    cmd = (uint8_t)(MLX90520_CMD_WRITE | ((word_addr & 0x100u) >> 8));
    addr_lsb = (uint8_t)word_addr;

    MLX90520_CS_L();
    mlx90520_cs_delay();

    rx0 = mlx90520_spi_xfer(cmd);
    cmd_echo = mlx90520_spi_xfer(addr_lsb);
    addr_echo = mlx90520_spi_xfer((uint8_t)(data[0] >> 8));
    (void)rx0;

    (void)mlx90520_spi_xfer((uint8_t)data[0]);
    for (uint16_t i = 1u; i < n; ++i) {
        (void)mlx90520_spi_xfer((uint8_t)(data[i] >> 8));
        (void)mlx90520_spi_xfer((uint8_t)data[i]);
    }

    mlx90520_spi_wait_idle();
    mlx90520_cs_delay();
    MLX90520_CS_H();

    if ((cmd_echo != cmd) || (addr_echo != addr_lsb)) {
        return MLX90520_SPI_ERROR;
    }

    if (read_check != 0u) {
        for (uint16_t i = 0; i < n; ++i) {
            uint16_t check_value;
            st = MLX90520_ReadReg((uint16_t)(addr + 2u * i), &check_value);
            if (st != MLX90520_OK) {
                return st;
            }
            if (check_value != data[i]) {
                return MLX90520_VERIFY_ERROR;
            }
        }
    }

    return MLX90520_OK;
}

MLX90520_Status_t MLX90520_ReadReg(uint16_t addr, uint16_t *value)
{
    return MLX90520_ReadWords(addr, value, 1u);
}

MLX90520_Status_t MLX90520_WriteReg(uint16_t addr, uint16_t value, uint8_t read_check)
{
    return MLX90520_WriteWords(addr, &value, 1u, read_check);
}

MLX90520_Status_t MLX90520_UpdateReg(uint16_t addr, uint16_t mask, uint16_t value, uint8_t read_check)
{
    uint16_t reg_value;

    MLX90520_Status_t st = MLX90520_ReadReg(addr, &reg_value);
    if (st != MLX90520_OK) {
        return st;
    }

    reg_value = (uint16_t)((reg_value & (uint16_t)(~mask)) | (value & mask));
    return MLX90520_WriteReg(addr, reg_value, read_check);
}

MLX90520_Status_t MLX90520_ReadDeviceInfo(uint32_t *dev_info)
{
    uint16_t words[2];

    if (dev_info == 0) {
        return MLX90520_ARG_ERROR;
    }

    MLX90520_Status_t st = MLX90520_ReadWords(MLX90520_REG_DEV_INFO, words, 2u);
    if (st != MLX90520_OK) {
        return st;
    }

    *dev_info = ((uint32_t)words[1] << 16) | words[0];
    return MLX90520_OK;
}

MLX90520_Status_t MLX90520_ReadConfigSnapshot(MLX90520_ConfigSnapshot_t *cfg)
{
    MLX90520_Status_t st;

    if (cfg == 0) {
        return MLX90520_ARG_ERROR;
    }

    st = MLX90520_ReadReg(MLX90520_REG_PROTOCOL, &cfg->protocol);
    if (st != MLX90520_OK) {
        return st;
    }
    st = MLX90520_ReadReg(MLX90520_REG_VERNIER, &cfg->vernier);
    if (st != MLX90520_OK) {
        return st;
    }
    st = MLX90520_ReadReg(MLX90520_REG_VERNIER_VM, &cfg->vernier_vm);
    if (st != MLX90520_OK) {
        return st;
    }
    st = MLX90520_ReadReg(MLX90520_REG_FC_CFG, &cfg->fc_cfg);
    if (st != MLX90520_OK) {
        return st;
    }
    st = MLX90520_ReadReg(MLX90520_REG_SPI_FADDR01, &cfg->spi_faddr01);
    if (st != MLX90520_OK) {
        return st;
    }
    st = MLX90520_ReadReg(MLX90520_REG_SPI_FADDR23, &cfg->spi_faddr23);
    if (st != MLX90520_OK) {
        return st;
    }
    st = MLX90520_ReadReg(MLX90520_REG_SPI_FRAME, &cfg->spi_frame);
    if (st != MLX90520_OK) {
        return st;
    }
    st = MLX90520_ReadReg(MLX90520_REG_SC_FC1, &cfg->sc_fc1);
    if (st != MLX90520_OK) {
        return st;
    }
    st = MLX90520_ReadReg(MLX90520_REG_SC_FC2, &cfg->sc_fc2);
    if (st != MLX90520_OK) {
        return st;
    }

    return MLX90520_OK;
}

void MLX90520_SPI3_Init(void)
{
    GPIO_InitTypeDef gpio;
    SPI_InitTypeDef spi;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);

    gpio.GPIO_Pin = GPIO_Pin_3;
    gpio.GPIO_Mode = GPIO_Mode_OUT;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &gpio);
    MLX90520_CS_H();

    gpio.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOC, &gpio);

    GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_SPI3);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_SPI3);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_SPI3);

    RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI3, ENABLE);
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI3, DISABLE);

    SPI_I2S_DeInit(SPI3);

    spi.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    spi.SPI_Mode = SPI_Mode_Master;
    spi.SPI_DataSize = SPI_DataSize_8b;
    spi.SPI_CPOL = SPI_CPOL_Low;
    spi.SPI_CPHA = SPI_CPHA_1Edge;
    spi.SPI_NSS = SPI_NSS_Soft;
    spi.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;
    spi.SPI_FirstBit = SPI_FirstBit_MSB;
    spi.SPI_CRCPolynomial = 7;

    SPI_Init(SPI3, &spi);
    SPI_Cmd(SPI3, ENABLE);
}

MLX90520_Status_t MLX90520_CheckDevice(void)
{
    uint16_t words[2];
    uint32_t dev_official;
    uint32_t dev_legacy;

    MLX90520_Status_t st = MLX90520_ReadWords(MLX90520_REG_DEV_INFO, words, 2u);
    if (st != MLX90520_OK) {
        return st;
    }

    dev_official = ((uint32_t)words[1] << 16) | words[0];
    dev_legacy = ((uint32_t)words[0] << 16) | words[1];
    return ((dev_official == MLX90520_DEV_INFO_EXPECTED) ||
            (dev_legacy == MLX90520_DEV_INFO_EXPECTED)) ? MLX90520_OK : MLX90520_DEV_ID_ERROR;
}

MLX90520_Status_t MLX90520_StartFrameRead(void)
{
    uint8_t rx0;
    uint8_t rx1;

    MLX90520_Status_t st = MLX90520_CheckDevice();
    if (st != MLX90520_OK) {
        return st;
    }

    MLX90520_CS_L();
    mlx90520_cs_delay();

    rx0 = mlx90520_spi_xfer(MLX90520_CMD_FRAME_START);
    rx1 = mlx90520_spi_xfer(0x00u);
    (void)rx0;

    if (rx1 != MLX90520_CMD_FRAME_START) {
        mlx90520_abort_frame_read();
        return MLX90520_SPI_ERROR;
    }

    mlx90520_frame_started = 1u;
    mlx90520_next_fs = MLX90520_FS_START;
    return MLX90520_OK;
}

void MLX90520_StopFrameRead(void)
{
    mlx90520_abort_frame_read();
}

MLX90520_Status_t MLX90520_ReadFrame2(MLX90520_Frame_t *frame)
{
    uint8_t rx[6];
    uint8_t idx = 0u;
    uint8_t len = 4u;

    if (frame == 0) {
        return MLX90520_ARG_ERROR;
    }

    if (!mlx90520_frame_started) {
        MLX90520_Status_t st = MLX90520_StartFrameRead();
        if (st != MLX90520_OK) {
            return st;
        }
    }

#if MLX90520_READ_FS
    len++;
#endif
#if MLX90520_READ_CRC
    len++;
#endif

    for (uint8_t i = 0; i < len; ++i) {
        rx[i] = mlx90520_spi_xfer(0x00u);
    }

#if MLX90520_READ_FS
    frame->fs = rx[idx++];

    if ((frame->fs & 0xF0u) != (mlx90520_next_fs & 0xF0u)) {
        mlx90520_abort_frame_read();
        return MLX90520_FS_ERROR;
    }

    if ((frame->fs & 0x0Fu) != (mlx90520_next_fs & 0x0Fu)) {
        mlx90520_abort_frame_read();
        return MLX90520_FS_ERROR;
    }
#endif

#if MLX90520_READ_CRC
    if (mlx90520_crc8_ccitt(rx, len) != 0u) {
        mlx90520_abort_frame_read();
        return MLX90520_CRC_ERROR;
    }
#endif

    frame->fc1 = ((uint16_t)rx[idx] << 8) | rx[idx + 1u];
    idx += 2u;
    frame->fc2 = ((uint16_t)rx[idx] << 8) | rx[idx + 1u];

#if MLX90520_READ_FS
    mlx90520_next_fs = (mlx90520_next_fs & 0xF0u) |
                       (uint8_t)((mlx90520_next_fs + 1u) & 0x0Fu);
#endif

    return MLX90520_OK;
}

MLX90520_Status_t MLX90520_ReadRaw16(uint16_t *raw16)
{
    MLX90520_Frame_t frame;

    if (raw16 == 0) {
        return MLX90520_ARG_ERROR;
    }

    MLX90520_Status_t st = MLX90520_ReadFrame2(&frame);
    if (st != MLX90520_OK) {
        return st;
    }

    *raw16 = frame.fc1;
    return MLX90520_OK;
}

MLX90520_Status_t MLX90520_ReadRaw22(uint32_t *raw22)
{
    MLX90520_Frame_t frame;

    if (raw22 == 0) {
        return MLX90520_ARG_ERROR;
    }

    MLX90520_Status_t st = MLX90520_ReadFrame2(&frame);
    if (st != MLX90520_OK) {
        return st;
    }

    // 对应官方 example:
    // raw22 = (data_rx[1] << 16) | data_rx[0]
    // 其中 data_rx[1] 通常是 CVDP/sector，data_rx[0] 是 16-bit PA/角度。
    *raw22 = ((uint32_t)(frame.fc2 & 0x003Fu) << 16) | frame.fc1;
    return MLX90520_OK;
}

