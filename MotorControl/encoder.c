
#include "MyProject.h"

// 用于霍尔编码器
/*****************************************************************************/
// //GPIO1为CS
// #define  SPI_CS0_L   GPIO_ResetBits(GPIOA, GPIO_Pin_0)
// #define  SPI_CS0_H   GPIO_SetBits(GPIOA, GPIO_Pin_0)

//GPIO8为CS
#include <hall_encoder.h>
#define  SPI_CS0_L   GPIO_ResetBits(GPIOB, GPIO_Pin_3)
#define  SPI_CS0_H   GPIO_SetBits(GPIOB, GPIO_Pin_3)

#define  SPI3_TX_OFF() {GPIOC->MODER&=~(3<<(12*2));GPIOC->MODER|=0<<(12*2);}  //PC12(MOSI)输入浮空
#define  SPI3_TX_ON()  {GPIOC->MODER&=~(3<<(12*2));GPIOC->MODER|=2<<(12*2);}  //PC12(MOSI)复用推挽输出
/*****************************************************************************/
ENCODER_CONFIG   encoder_config;

bool index_found_ = false;   //针对ABZ编码器，Z信号
bool is_ready_ = false;      //编码器是否准备就绪。电机上电校准后为true。
int32_t shadow_count_ = 0;   //编码器累计计数。
int32_t count_in_cpr_ = 0;   //编码器当前计数值。
float interpolation_ = 0.0f; //编码器当前插补值。
float pos_estimate_counts_ = 0.0f;  //当前估算的位置值，单位[count]   
float pos_cpr_counts_ = 0.0f;       //当前约束在cpr范围内的位置值，单位[count]
//float delta_pos_cpr_counts_ = 0.0f;  // [count] phase detector result for debug
float vel_estimate_counts_ = 0.0f;  //当前估算转速，单位[count/s]
float pll_kp_ = 0.0f;   // [count/s / count]
float pll_ki_ = 0.0f;   // [(count/s^2) / count]
float calib_scan_response_ = 0.0f; // debug report from offset calib
int32_t  pos_abs_ = 0;  //绝对值编码器的位置
float spi_error_rate_ = 0.0f;

float pos_estimate_ = 0.0f; //当前估算的位置值，单位[turn]
float vel_estimate_ = 0.0f; //当前估算转速，单位[turn/s]
float pos_circular_ = 0.0f; //环形位置模式下当前位置值，单位[turn]

bool pos_estimate_valid_ = false;   //位置估算是否可用
bool vel_estimate_valid_ = false;   //速度估算是否可用

int16_t  tim_cnt_sample_;


bool abs_spi_pos_updated_ = false;  //绝对值编码器角度是否被正确读出
/*****************************************************************************/
/*****************************************************************************/
void encoder_set_error(uint32_t error) 
{
	vel_estimate_valid_ = false;
	pos_estimate_valid_ = false;
	set_error(error);
}
/*****************************************************************************/
void update_pll_gains(void)
{
	pll_kp_ = 2.0f * encoder_config.bandwidth;  // basic conversion to discrete time
	pll_ki_ = 0.25f * (pll_kp_ * pll_kp_); // Critically damped
	// Check that we don't get problems with discrete time approximation
	if (!(current_meas_period * pll_kp_ < 1.0f))encoder_set_error(ERROR_UNSTABLE_GAIN);
}
/*****************************************************************************/

static const float hall_edge_defaults_[6] = {0.0f, 1.0f, 2.0f, 3.0f, 4.0f, 5.0f};
static bool hall_polarity_calibration_active_ = false;
static bool hall_phase_calibration_active_ = false;
static bool hall_sample_states_ = false;
static bool hall_sample_phase_ = false;
static uint32_t hall_states_seen_count_[8];
static uint32_t hall_phase_calib_seen_count_[6];
static uint8_t hall_last_cnt_valid_ = 0u;
static int32_t hall_last_cnt_ = 0;

static void HallEncoder_LoadDefaultEdges(void)
{
	uint32_t i;

	for (i = 0u; i < 6u; ++i)
	{
		encoder_config.hall_edge_phcnt[i] = hall_edge_defaults_[i];
	}
}

static void HallEncoder_CopyConfigToRuntime(void)
{
	uint32_t i;

	g_hall_encoder.config.polarity_xor = encoder_config.hall_polarity_calibrated ? encoder_config.hall_polarity : 0u;
	g_hall_encoder.config.ignore_illegal_state = encoder_config.ignore_illegal_hall_state ? 1u : 0u;
	g_hall_encoder.config.enable_phase_interpolation = encoder_config.enable_phase_interpolation ? 1u : 0u;
	g_hall_encoder.config.bandwidth = encoder_config.bandwidth;
	g_hall_encoder.config.sample_hz = current_meas_hz;
	for (i = 0u; i < 6u; ++i)
	{
		g_hall_encoder.edge_pos[i] = encoder_config.hall_edge_phcnt[i];
	}
}

static void HallEncoder_ResetRuntimeModel(void)
{
	HallEncoder_CopyConfigToRuntime();
	HallEncoder_Reset(&g_hall_encoder);
	shadow_count_ = 0;
	count_in_cpr_ = 0;
	interpolation_ = 0.5f;
	pos_estimate_counts_ = 0.0f;
	pos_cpr_counts_ = 0.0f;
	vel_estimate_counts_ = 0.0f;
	pos_estimate_ = 0.0f;
	vel_estimate_ = 0.0f;
	pos_circular_ = 0.0f;
	pos_estimate_valid_ = false;
	vel_estimate_valid_ = false;
	encoder_config.phase_ = 0.0f;
	encoder_config.phase_vel_ = 0.0f;
	encoder_config.hall_state_ = g_hall_encoder.raw_hall_state;
	is_ready_ = false;
}

static uint8_t HallEncoder_DecodeState(uint8_t hall_state, int32_t *hall_cnt)
{
	switch (hall_state & 0x07u)
	{
		case 0x01u: *hall_cnt = 0; return 1u;
		case 0x03u: *hall_cnt = 1; return 1u;
		case 0x02u: *hall_cnt = 2; return 1u;
		case 0x06u: *hall_cnt = 3; return 1u;
		case 0x04u: *hall_cnt = 4; return 1u;
		case 0x05u: *hall_cnt = 5; return 1u;
		default: return 0u;
	}
}

static uint8_t HallEncoder_FlipDetect(uint8_t states, uint8_t idx)
{
	return (uint8_t)(((~states) & 0xFFu) == ((1u << idx) | (1u << (7u - idx))));
}

static void HallEncoder_ProjectToControlLoop(void)
{
	float interpolated_enc;
	float elec_rad_per_enc;
	float ph;
	float pos_cpr_counts_last = pos_cpr_counts_;

	shadow_count_ = g_hall_encoder.shadow_count;
	count_in_cpr_ = g_hall_encoder.count_in_cpr;
	interpolation_ = g_hall_encoder.interpolation;
	pos_estimate_counts_ = g_hall_encoder.pos_estimate_counts;
	pos_cpr_counts_ = g_hall_encoder.pos_cpr_counts;
	vel_estimate_counts_ = g_hall_encoder.vel_estimate_counts;
	pos_estimate_ = g_hall_encoder.position_turns;
	vel_estimate_ = g_hall_encoder.velocity_turns_per_sec;
	encoder_config.hall_state_ = g_hall_encoder.raw_hall_state;
	pos_circular_ += wrap_pm((g_hall_encoder.pos_cpr_counts - pos_cpr_counts_last) / (float)encoder_config.cpr, 1.0f);
	pos_circular_ = fmodf_pos(pos_circular_, ctrl_config.circular_setpoint_range);
	pos_estimate_valid_ = g_hall_encoder.ready ? true : false;
	vel_estimate_valid_ = g_hall_encoder.ready ? true : false;
	if (encoder_config.pre_calibrated && g_hall_encoder.ready)
	{
		is_ready_ = true;
	}

	interpolated_enc = (float)(count_in_cpr_ - encoder_config.phase_offset) + interpolation_;
	elec_rad_per_enc = motor_config.pole_pairs * 2.0f * M_PI * (1.0f / (float)encoder_config.cpr);
	ph = elec_rad_per_enc * (interpolated_enc - encoder_config.phase_offset_float);

	if (is_ready_)
	{
		encoder_config.phase_ = wrap_pm_pi(ph) * encoder_config.direction;
		encoder_config.phase_vel_ = (2.0f * M_PI) * vel_estimate_ * motor_config.pole_pairs * encoder_config.direction;
	}
	else
	{
		encoder_config.phase_vel_ = 0.0f;
	}
}

/*****************************************************************************/

/******************************************************************************/
/* KTH7112 CRC8 Table (matches HAL reference implementation) */
static const uint8_t CRC8Table[256] = {
	0x00, 0x07, 0x0e, 0x09, 0x1c, 0x1b, 0x12, 0x15, 0x38, 0x3f, 0x36, 0x31, 0x24, 0x23, 0x2a, 0x2d,
	0x70, 0x77, 0x7e, 0x79, 0x6c, 0x6b, 0x62, 0x65, 0x48, 0x4f, 0x46, 0x41, 0x54, 0x53, 0x5a, 0x5d,
	0xe0, 0xe7, 0xee, 0xe9, 0xfc, 0xfb, 0xf2, 0xf5, 0xd8, 0xdf, 0xd6, 0xd1, 0xc4, 0xc3, 0xca, 0xcd,
	0x90, 0x97, 0x9e, 0x99, 0x8c, 0x8b, 0x82, 0x85, 0xa8, 0xaf, 0xa6, 0xa1, 0xb4, 0xb3, 0xba, 0xbd,
	0xc7, 0xc0, 0xc9, 0xce, 0xdb, 0xdc, 0xd5, 0xd2, 0xff, 0xf8, 0xf1, 0xf6, 0xe3, 0xe4, 0xed, 0xea,
	0xb7, 0xb0, 0xb9, 0xbe, 0xab, 0xac, 0xa5, 0xa2, 0x8f, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9d, 0x9a,
	0x27, 0x20, 0x29, 0x2e, 0x3b, 0x3c, 0x35, 0x32, 0x1f, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0d, 0x0a,
	0x57, 0x50, 0x59, 0x5e, 0x4b, 0x4c, 0x45, 0x42, 0x6f, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7d, 0x7a,
	0x89, 0x8e, 0x87, 0x80, 0x95, 0x92, 0x9b, 0x9c, 0xb1, 0xb6, 0xbf, 0xb8, 0xad, 0xaa, 0xa3, 0xa4,
	0xf9, 0xfe, 0xf7, 0xf0, 0xe5, 0xe2, 0xeb, 0xec, 0xc1, 0xc6, 0xcf, 0xc8, 0xdd, 0xda, 0xd3, 0xd4,
	0x69, 0x6e, 0x67, 0x60, 0x75, 0x72, 0x7b, 0x7c, 0x51, 0x56, 0x5f, 0x58, 0x4d, 0x4a, 0x43, 0x44,
	0x19, 0x1e, 0x17, 0x10, 0x05, 0x02, 0x0b, 0x0c, 0x21, 0x26, 0x2f, 0x28, 0x3d, 0x3a, 0x33, 0x34,
	0x4e, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5c, 0x5b, 0x76, 0x71, 0x78, 0x7f, 0x6a, 0x6d, 0x64, 0x63,
	0x3e, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2c, 0x2b, 0x06, 0x01, 0x08, 0x0f, 0x1a, 0x1d, 0x14, 0x13,
	0xae, 0xa9, 0xa0, 0xa7, 0xb2, 0xb5, 0xbc, 0xbb, 0x96, 0x91, 0x98, 0x9f, 0x8a, 0x8d, 0x84, 0x83,
	0xde, 0xd9, 0xd0, 0xd7, 0xc2, 0xc5, 0xcc, 0xcb, 0xe6, 0xe1, 0xe8, 0xef, 0xfa, 0xfd, 0xf4, 0xf3
};

static uint8_t KTH71_IsCRCOK(uint8_t *pbuf, uint8_t buflen)
{
	uint8_t crc = 0x00;
	uint8_t i;
	for (i = 0; i < buflen - 1; i++)
	{
		crc = CRC8Table[crc ^ pbuf[i]];
	}
	crc = crc ^ 0x55;
	return (crc == pbuf[buflen - 1]) ? 1 : 0;
}

static void Encoder_SetupOpenLoopForCalibration(float start_lock_duration)
{
	float max_current_ramp;

	memset(&openloop_controller_, 0, sizeof(OPENLOOP_struct));

	max_current_ramp = motor_config.calibration_current / start_lock_duration * 2.0f;
	openloop_controller_.max_current_ramp_ = max_current_ramp;
	openloop_controller_.max_voltage_ramp_ = max_current_ramp;
	openloop_controller_.max_phase_vel_ramp_ = INFINITY;
	openloop_controller_.target_current_ = motor_config.motor_type != MOTOR_TYPE_GIMBAL ? motor_config.calibration_current : 0.0f;
	openloop_controller_.target_voltage_ = motor_config.motor_type != MOTOR_TYPE_GIMBAL ? 0.0f : motor_config.calibration_current;
	openloop_controller_.target_vel_ = 0.0f;
	openloop_controller_.total_distance_ = 0.0f;
	openloop_controller_.phase_ = wrap_pm_pi(0 - encoder_config.calib_scan_distance / 2.0f);

	Idq_setpoint_src_ = &openloop_controller_.Idq_setpoint_;
	Vdq_setpoint_src_ = &openloop_controller_.Vdq_setpoint_;
	phase_src_ = &openloop_controller_.phase_;
	phase_vel_src_ = &openloop_controller_.phase_vel_;
	motor_phase_vel_src_ = &openloop_controller_.phase_vel_;

	arm();
}

static bool Encoder_WaitCalibrationLock(uint32_t lock_ms)
{
	uint32_t i;

	for (i = 0; i < lock_ms; ++i)
	{
		if (!is_armed_)
		{
			return false;
		}
		delay_us(1000);
	}

	return true;
}

bool run_hall_polarity_calibration(void)
{
	const float start_lock_duration = 1.0f;
	const float spin_duration = 3.0f;
	float finish_distance = encoder_config.calib_scan_omega * spin_duration;
	uint32_t i;
	uint8_t states_seen = 0u;
	uint8_t states_confirmed = 0u;
	uint8_t hall_polarity = 0u;

	Encoder_SetupOpenLoopForCalibration(start_lock_duration);
	if (!Encoder_WaitCalibrationLock((uint32_t)(start_lock_duration * 1000.0f)))
	{
		return false;
	}

	encoder_config.hall_polarity_calibrated = false;
	memset(hall_states_seen_count_, 0, sizeof(hall_states_seen_count_));
	hall_last_cnt_valid_ = 0u;
	hall_sample_states_ = true;
	hall_polarity_calibration_active_ = true;
	openloop_controller_.target_vel_ = encoder_config.calib_scan_omega;
	openloop_controller_.total_distance_ = 0.0f;

	while (is_armed_)
	{
		if (openloop_controller_.total_distance_ >= finish_distance)
		{
			break;
		}
		delay_us(1000);
	}

	hall_sample_states_ = false;
	hall_polarity_calibration_active_ = false;
	disarm();

	if (!is_armed_ && motor_error)
	{
		return false;
	}

	for (i = 0u; i < 8u; ++i)
	{
		if (hall_states_seen_count_[i] > 0u)
		{
			states_seen |= (uint8_t)(1u << i);
		}
		if (hall_states_seen_count_[i] > 50u)
		{
			states_confirmed |= (uint8_t)(1u << i);
		}
	}

	if (states_seen != states_confirmed)
	{
		encoder_set_error(ERROR_ILLEGAL_HALL_STATE);
		return false;
	}

	if (HallEncoder_FlipDetect(states_seen, 0u))
	{
		hall_polarity = 0x00u;
	}
	else if (HallEncoder_FlipDetect(states_seen, 1u))
	{
		hall_polarity = 0x01u;
	}
	else if (HallEncoder_FlipDetect(states_seen, 2u))
	{
		hall_polarity = 0x02u;
	}
	else if (HallEncoder_FlipDetect(states_seen, 3u))
	{
		hall_polarity = 0x04u;
	}
	else
	{
		encoder_set_error(ERROR_ILLEGAL_HALL_STATE);
		return false;
	}

	encoder_config.hall_polarity = hall_polarity;
	encoder_config.hall_polarity_calibrated = true;
	HallEncoder_ResetRuntimeModel();
	return true;
}

bool run_hall_phase_calibration(void)
{
	const float start_lock_duration = 1.0f;
	const float spin_duration = 30.0f;
	float finish_distance = encoder_config.calib_scan_omega * spin_duration;
	float delta_phase = 0.0f;
	float offset;
	uint32_t i;

	if (!encoder_config.hall_polarity_calibrated)
	{
		encoder_set_error(ERROR_HALL_NOT_CALIBRATED_YET);
		return false;
	}

	Encoder_SetupOpenLoopForCalibration(start_lock_duration);
	if (!Encoder_WaitCalibrationLock((uint32_t)(start_lock_duration * 1000.0f)))
	{
		return false;
	}

	HallEncoder_LoadDefaultEdges();
	memset(hall_phase_calib_seen_count_, 0, sizeof(hall_phase_calib_seen_count_));
	hall_last_cnt_valid_ = 0u;
	hall_sample_phase_ = true;
	hall_phase_calibration_active_ = true;
	openloop_controller_.target_vel_ = encoder_config.calib_scan_omega;
	openloop_controller_.total_distance_ = 0.0f;

	while (is_armed_)
	{
		if (openloop_controller_.total_distance_ >= finish_distance)
		{
			break;
		}
		delay_us(1000);
	}

	hall_sample_phase_ = false;
	hall_phase_calibration_active_ = false;
	disarm();

	if (!is_armed_ && motor_error)
	{
		return false;
	}

	for (i = 0u; i < 6u; ++i)
	{
		uint32_t next_i = (i == 5u) ? 0u : (i + 1u);
		if (hall_phase_calib_seen_count_[i] == 0u)
		{
			encoder_set_error(ERROR_ILLEGAL_HALL_STATE);
			HallEncoder_LoadDefaultEdges();
			return false;
		}
		delta_phase += wrap_pm_pi(encoder_config.hall_edge_phcnt[next_i] - encoder_config.hall_edge_phcnt[i]);
	}

	if (delta_phase < 0.0f)
	{
		encoder_config.direction = -1;
		for (i = 0u; i < 6u; ++i)
		{
			encoder_config.hall_edge_phcnt[i] = wrap_pm_pi(-encoder_config.hall_edge_phcnt[i]);
		}
	}
	else
	{
		encoder_config.direction = 1;
	}

	offset = encoder_config.hall_edge_phcnt[0];
	for (i = 0u; i < 6u; ++i)
	{
		encoder_config.hall_edge_phcnt[i] = fmodf_pos((6.0f / (2.0f * M_PI)) * (encoder_config.hall_edge_phcnt[i] - offset), 6.0f);
	}

	HallEncoder_ResetRuntimeModel();
	return true;
}


uint16_t KTH7111_ReadSSIAngle(void)
{
#if 0
	uint8_t pRxData[3] = {0};
	uint16_t angle = 0;
	int i;

	SPI3_ClearOVR_Flag();

	SPI3->CR1 &= ~SPI_CR1_SPE;
	SPI3->CR1 &= ~SPI_CR1_BIDIMODE;
	SPI3->CR1 |= SPI_CR1_RXONLY;
	SPI3->CR1 |= SPI_CR1_SPE;

	for (i = 0; i < 3; i++)
	{
		while (!(SPI3->SR & SPI_SR_RXNE));
		pRxData[i] = (uint8_t)SPI3->DR;
	}

	// while (SPI3->SR & SPI_SR_BSY);
	SPI3->CR1 &= ~SPI_CR1_SPE;

	SPI3_ClearOVR_Flag();

	angle |= (uint16_t)pRxData[0] << 9;
	angle |= (uint16_t)pRxData[1] << 1;
	angle |= (uint16_t)pRxData[2] >> 7;

	return angle;
#else
	uint8_t pRxData[3] = {0};
	uint16_t angle = 0;
	int i;

	SPI1_ClearOVR_Flag();

	SPI1->CR1 &= ~SPI_CR1_SPE;
	SPI1->CR1 &= ~SPI_CR1_BIDIMODE;
	SPI1->CR1 |= SPI_CR1_RXONLY;
	SPI1->CR1 |= SPI_CR1_SPE;

	for (i = 0; i < 3; i++)
	{
		while (!(SPI1->SR & SPI_SR_RXNE));
		pRxData[i] = (uint8_t)SPI1->DR;
	}

	// while (SPI1->SR & SPI_SR_BSY);
	SPI1->CR1 &= ~SPI_CR1_SPE;

	SPI1_ClearOVR_Flag();

	angle |= (uint16_t)pRxData[0] << 9;
	angle |= (uint16_t)pRxData[1] << 1;
	angle |= (uint16_t)pRxData[2] >> 7;

	return angle;
#endif
}



uint16_t KTH7112_ReadAngle(void)
{
	uint8_t pRxData[3] = {0};
	uint32_t tmp;

	// 1. 清 OVR 溢出
	tmp = SPI3->DR;
	tmp = SPI3->SR;
	(void)tmp;

	// 2. 保证 SPI 处于关闭状态，统一配置
	SPI3->CR1 &= ~SPI_CR1_SPE;

	// 3. 配置为：单线、发送模式
	SPI3->CR1 |= SPI_CR1_BIDIOE;
	SPI3->CR1 |= SPI_CR1_SPE;

	// 4. CS 拉低（开始通信）
	GPIOB->BSRR = (uint32_t)GPIO_Pin_3 << 16U;

	// 5. 发送 0x00
	while (!(SPI3->SR & SPI_SR_TXE));
	SPI3->DR = 0x00;
	while (!(SPI3->SR & SPI_SR_TXE));
	while (SPI3->SR & SPI_SR_BSY);

	// 6. 关闭 SPI，切换为接收模式
	SPI3->CR1 &= ~SPI_CR1_SPE;
	SPI3->CR1 &= ~SPI_CR1_BIDIOE;
	SPI3->CR1 |= SPI_CR1_SPE;

	// 7. 连续读取 3 字节（关键：不中断、不延时）
	for (int i = 0; i < 3; i++)
	{
		while (!(SPI3->SR & SPI_SR_RXNE));
		pRxData[i] = (uint8_t)SPI3->DR;
	}

	// 8. 关闭 SPI，停止时钟
	SPI3->CR1 &= ~SPI_CR1_SPE;
	while (SPI3->SR & SPI_SR_BSY);

	// 9. CS 拉高（结束通信）
	GPIOB->BSRR = GPIO_Pin_3;

	// 10. 输出角度（14bit 或 16bit 编码器）
	return ((uint16_t)pRxData[0] << 8) | pRxData[1];
}

// // 全局变量
// uint8_t KTH7112_RxBuf[3] = {0};

uint16_t KTH7112_ReadAngle_DMA(void)
{
	// 清 OVR
	(void)SPI3->DR;
	(void)SPI3->SR;

	// 关闭 DMA
	DMA1_Stream2->CR &= ~DMA_SxCR_EN;
	while(DMA1_Stream2->CR & DMA_SxCR_EN);
	DMA1_Stream2->NDTR = 3;  // 接收3字节

	// CS 拉低
	GPIO_ResetBits(GPIOB, GPIO_Pin_3);

	// 切换 SPI → 发送模式
	SPI3->CR1 |= SPI_CR1_BIDIOE;
	SPI_Cmd(SPI3, ENABLE);

	// 开启 DMA
	DMA1_Stream2->CR |= DMA_SxCR_EN;

	// 发送 0x00
	while(!(SPI3->SR & SPI_SR_TXE));
	SPI3->DR = 0x00;
	while(!(SPI3->SR & SPI_SR_TXE));
	while(SPI3->SR & SPI_SR_BSY);

	// 切换 SPI → 接收模式
	SPI_Cmd(SPI3, DISABLE);
	SPI3->CR1 &= ~SPI_CR1_BIDIOE;
	SPI_Cmd(SPI3, ENABLE);

	// ======================== 关键：用寄存器判断 DMA 完成（兼容所有F4库）
	while(!(DMA1->LISR & (1 << 21)));  // 等待 TCIF2 置位

	// 关闭 DMA
	DMA1_Stream2->CR &= ~DMA_SxCR_EN;

	// 清 DMA 标志
	DMA1->LIFCR |= (1 << 21);

	// 关闭 SPI
	SPI_Cmd(SPI3, DISABLE);
	while(SPI3->SR & SPI_SR_BSY);

	// CS 拉高
	GPIO_SetBits(GPIOB, GPIO_Pin_3);

	return ((uint16_t)KTH7112_RxBuf[0] << 8) | KTH7112_RxBuf[1];
}


void encoder_zero_position(void)
{
	int32_t current_count_in_cpr;

	// 1. 先锁存当前单圈位置
	current_count_in_cpr = count_in_cpr_;

	// 如果是绝对值编码器，也可以直接用当前绝对位置
	if ((encoder_config.mode == MODE_SPI_AS5047P) ||
		(encoder_config.mode == MODE_SPI_MT6701) ||
		(encoder_config.mode == MODE_SPI_MA730) ||
		(encoder_config.mode == MODE_SPI_TLE5012B) ||
		(encoder_config.mode == MODE_SPI_MT6835) ||
		(encoder_config.mode == MODE_SPI_KTH7112))
	{
		current_count_in_cpr = pos_abs_;
	}

	// 2. 多圈累计清零
	shadow_count_ = 0;
	pos_estimate_counts_ = 0.0f;
	pos_estimate_ = 0.0f;

	// 3. 保留当前真实单圈角度，避免下一次采样突变
	count_in_cpr_ = current_count_in_cpr;
	pos_cpr_counts_ = (float)current_count_in_cpr;

	// 4. 速度和插值清零，减少瞬态抖动
	vel_estimate_counts_ = 0.0f;
	vel_estimate_ = 0.0f;
	interpolation_ = 0.5f;
	pos_circular_ = 0.0f;

	// 5. 如果当前在位置控制，目标也同步归零，避免电机因为目标没变而跳动
	input_pos_ = 0.0f;
	input_vel_ = 0.0f;
	input_torque_ = 0.0f;
	input_pos_updated_ = false;

	pos_setpoint_ = 0.0f;
	vel_setpoint_ = 0.0f;
	torque_setpoint_ = 0.0f;
	vel_integrator_torque_ = 0.0f;
}

/*****************************************************************************/
//初始化三种SPI接口的编码器的参数, 初始化I2C接口或者SPI接口
void MagneticSensor_Init(void)
{
	//读取flash内保存的参数
	encoder_config.mode = ENCODER_mode;  //选择编码器型号，参数设置宏定义在MyProject.h文件中
	encoder_config.cpr = ENCODER_cpr;    //编码器cpr
	encoder_config.bandwidth = ENCODER_bandwidth;   //编码器带宽
	encoder_config.calib_range = 0.02f; // Accuracy required to pass encoder cpr check  2%误差
	encoder_config.calib_scan_distance = 16.0f * M_PI; // rad electrical    校准的时候正反转8个极对数
	encoder_config.calib_scan_omega = 4.0f * M_PI;     // rad/s electrical  转速2个极对数/秒，所以正转4秒，反转再4秒
	encoder_config.phase_offset = 0;        // Offset between encoder count and rotor electrical phase
	encoder_config.phase_offset_float = 0.0f; // Sub-count phase alignment offset
	encoder_config.index_offset = 0.0f;
	encoder_config.use_index = false;
	encoder_config.pre_calibrated = false;
	encoder_config.direction = 0; // direction with respect to motor
	encoder_config.use_index_offset = true;
	encoder_config.enable_phase_interpolation = true; // Use velocity to interpolate inside the count state
	encoder_config.find_idx_on_lockin_only = false; // Only be sensitive during lockin scan constant vel state
	encoder_config.ignore_illegal_hall_state = false;
	encoder_config.hall_polarity = 0u;
	encoder_config.hall_polarity_calibrated = false;
	HallEncoder_LoadDefaultEdges();
	if (encoder_config.mode == MODE_HALL)
	{
		encoder_config.cpr = motor_config.pole_pairs * 6;
	}
	
	update_pll_gains();    //锁相环参数整定
	
	switch(encoder_config.mode)
	{
		case MODE_INCREMENTAL:
			TIM3_Encoder_Init();         //ABZ
			break;
		case MODE_HALL: {
			HallEncoderConfig_t hall_cfg;

			hall_cfg.hall_a.port = GPIOB;
			hall_cfg.hall_a.pin = GPIO_Pin_4;
			hall_cfg.hall_b.port = GPIOB;
			hall_cfg.hall_b.pin = GPIO_Pin_5;
			hall_cfg.hall_c.port = GPIOC;
			hall_cfg.hall_c.pin = GPIO_Pin_9;
			hall_cfg.pole_pairs = (uint16_t)motor_config.pole_pairs;
			hall_cfg.sample_hz = current_meas_hz;
			hall_cfg.polarity_xor = encoder_config.hall_polarity_calibrated ? encoder_config.hall_polarity : 0u;
			hall_cfg.ignore_illegal_state = encoder_config.ignore_illegal_hall_state ? 1u : 0u;
			hall_cfg.enable_phase_interpolation = encoder_config.enable_phase_interpolation ? 1u : 0u;
			hall_cfg.bandwidth = encoder_config.bandwidth;
			HallEncoder_Init(&g_hall_encoder, &hall_cfg);
			HallEncoder_CopyConfigToRuntime();
			break;
		}
	        case MODE_SPI_AS5047P:
			SPI3_Init_(SPI_CPOL_Low);    //AS5047P
			break;
		case MODE_SPI_MT6701:
			SPI3_Init_(SPI_CPOL_Low);    //MT6701
			break;
		case MODE_SPI_MA730:
			SPI3_Init_(SPI_CPOL_High);   //MA730
			break;
		case MODE_SPI_TLE5012B:        //TLE5012B
			SPI3_Init_(SPI_CPOL_Low);
			break;
		case MODE_SPI_MT6835:
			SPI3_Init_(SPI_CPOL_High);   //MT6835
			break;
		case MODE_SPI_KTH7112:
			SPI3_Init_KTH7112(SPI_CPOL_High);
		// SPI3_Init_KTH7112(SPI_CPOL_Low);
			break;
	}
}
/*****************************************************************************/
// @brief Turns the motor in one direction for a bit and then in the other
// direction in order to find the offset between the electrical phase 0
// and the encoder state 0.
// float expected_encoder_delta;
bool run_offset_calibration(void)
{
	uint32_t  i;
	const float start_lock_duration = 1.0f;
	
	// Require index found if enabled
	if (encoder_config.use_index && !index_found_)
	{
		encoder_set_error(ERROR_INDEX_NOT_FOUND_YET);
		return false;
	}

	if ((encoder_config.mode == MODE_HALL) && !encoder_config.hall_polarity_calibrated)
	{
		encoder_set_error(ERROR_HALL_NOT_CALIBRATED_YET);
		return false;
	}
	
	// We use shadow_count_ to do the calibration, but the offset is used by count_in_cpr_
	// Therefore we have to sync them for calibration
	shadow_count_ = count_in_cpr_;
	
	// Reset state variables
	memset(&openloop_controller_,0, sizeof(OPENLOOP_struct));   //清零openloop的结构体
//	openloop_controller_.Idq_setpoint_.d = 0.0f;
//	openloop_controller_.Idq_setpoint_.q = 0.0f;
//	openloop_controller_.Vdq_setpoint_.d = 0.0f;
//	openloop_controller_.Vdq_setpoint_.q = 0.0f;
//	openloop_controller_.phase_ = 0.0f;
//	openloop_controller_.phase_vel_ = 0.0f;
	
	float max_current_ramp = motor_config.calibration_current / start_lock_duration * 2.0f;
	openloop_controller_.max_current_ramp_ = max_current_ramp;
	openloop_controller_.max_voltage_ramp_ = max_current_ramp;
	openloop_controller_.max_phase_vel_ramp_ = INFINITY;
	openloop_controller_.target_current_ = motor_config.motor_type != MOTOR_TYPE_GIMBAL ? motor_config.calibration_current : 0.0f;   //功率电机，校准电流为目标电流
	openloop_controller_.target_voltage_ = motor_config.motor_type != MOTOR_TYPE_GIMBAL ? 0.0f : motor_config.calibration_current;   //云台电机，校准电流为目标电压
	openloop_controller_.target_vel_ = 0.0f;
	openloop_controller_.total_distance_ = 0.0f;
	openloop_controller_.phase_ = wrap_pm_pi(0 - encoder_config.calib_scan_distance / 2.0f);
	
	//enable_current_control_src_ = (motor_config.motor_type != MOTOR_TYPE_GIMBAL);
	Idq_setpoint_src_ = &openloop_controller_.Idq_setpoint_;   //指针指向
	Vdq_setpoint_src_ = &openloop_controller_.Vdq_setpoint_;
	phase_src_ = &openloop_controller_.phase_;
	phase_vel_src_ = &openloop_controller_.phase_vel_;
	motor_phase_vel_src_ = &openloop_controller_.phase_vel_;
	
	arm();
	// go to start position of forward scan for start_lock_duration to get ready to scan
	for (i=0; i<1000; i++)   //电机定位1秒钟，电角度0°
	{
		if (!is_armed_)return false; // TODO: return "disarmed" error code
		delay_us(1000);   //1ms
	}
	
	int32_t init_enc_val = shadow_count_;   //读取当前角度。shadow_count_在encoder_update()函数中读角度时更新
	uint32_t num_steps = 0;
	int64_t encvaluesum = 0;
	
	openloop_controller_.target_vel_ = encoder_config.calib_scan_omega;   //设置正转速度 4Pi/s
	openloop_controller_.total_distance_ = 0.0f;
	
	// scan forward
	while (is_armed_)
	{
		if(openloop_controller_.total_distance_ >= encoder_config.calib_scan_distance)break;   //转过16Pi（8个极对数）
		encvaluesum += shadow_count_;   //累加当前机械角度值
		num_steps++;      //大约转了4秒，转完约等于4000。    开环控制启动和停止都需要时间，所以会有误差
		delay_us(1000);   //1ms
	}
	
	// Check response and direction
	if (shadow_count_ > init_enc_val + 8)encoder_config.direction = 1;        //当前角度比初始角度大就是正转
	else if (shadow_count_ < init_enc_val - 8)encoder_config.direction = -1;  //否则就是反转
	else
	{
		encoder_set_error(ERROR_NO_RESPONSE);   //编码器接线不好，或者配置型号不对
		disarm();
		return false;
	}
	
	// Check CPR
	float elec_rad_per_enc = motor_config.pole_pairs * 2 * M_PI * (1.0f / (float)(encoder_config.cpr));
	float expected_encoder_delta = encoder_config.calib_scan_distance / elec_rad_per_enc;   //理论上的角度差值
	// expected_encoder_delta = encoder_config.calib_scan_distance / elec_rad_per_enc;   //理论上的角度差值
	calib_scan_response_ = fabsf(shadow_count_ - init_enc_val);                             //实际的角度差值
	if (fabsf(calib_scan_response_ - expected_encoder_delta) / expected_encoder_delta > encoder_config.calib_range)  //误差率大于2%，认为错误
	{
		encoder_set_error(ERROR_CPR_POLEPAIRS_MISMATCH);
		disarm();
		return false;
	}
	
	openloop_controller_.target_vel_ = -encoder_config.calib_scan_omega;  //设置反转速度 -4Pi/s
	
	// scan backwards
	while (is_armed_)
	{
		if(openloop_controller_.total_distance_ <= 0.0f)break;   //转过16Pi（8个极对数）
		encvaluesum += shadow_count_;   //累加当前机械角度值
		num_steps++;      //大概转了4秒钟，4000。 num_steps此时约为8000
		delay_us(1000);   //1ms
	}
	
	// Motor disarmed because of an error
	if (!is_armed_)return false;
	
	disarm();
	
	encoder_config.phase_offset = encvaluesum / num_steps;   //累加后的角度/累加次数=中间值，比如1——100累加=5050 /100=50.5。在这里，phase_offset就是电角度为8Pi时对应的角度
	int32_t residual = encvaluesum - ((int64_t)encoder_config.phase_offset * (int64_t)num_steps);
	encoder_config.phase_offset_float = (float)residual / (float)num_steps + 0.5f;  // add 0.5 to center-align state to phase  其实用不了这么高的精度，小数部分可有可无
	
	is_ready_ = true;
	return true;
}
/*****************************************************************************/
/*****************************************************************************/
uint8_t ams_parity(uint16_t v)
{
	v ^= v >> 8;
	v ^= v >> 4;
	v ^= v >> 2;
	v ^= v >> 1;
	return v & 1;
}
/*****************************************************************************/
uint8_t crc_high_first(uint8_t *ptr, int len)  //用于MT6835
{
	uint8_t i;
	uint8_t crc=0;
	
	while(len--)
	{
		crc ^= *ptr++;
		
		for(i=0;i<8;i++)
		{
			if(crc&0x80)crc=(crc<<1)^0x07;
			else  crc=(crc<<1);
		}
	}
	
	return crc;
}
/*****************************************************************************/
uint32_t pos,pos_val;
void abs_spi_cb(void)
{
	uint16_t rawVal;
	// uint32_t pos;
	
	switch(encoder_config.mode)
	{
		case MODE_SPI_AS5047P:
			SPI_CS0_L;
			rawVal = SPIx_ReadWriteByte(0xffff);  //encoder.hpp 第144行
			SPI_CS0_H;
		
			// if(ams_parity(rawVal) || ((rawVal >> 14) & 1))return;
			pos = (rawVal & 0x3fff);
			break;
		case MODE_SPI_MA730:
		case MODE_SPI_MT6701:
			SPI_CS0_L;
			rawVal = SPIx_ReadWriteByte(0);
			SPI_CS0_H;
			pos = (rawVal >> 2) & 0x3fff;
			break;
		case MODE_SPI_TLE5012B: {
			SPI_CS0_L;
			SPIx_ReadWriteByte(0x8020);
			SPI3_TX_OFF();
			// __nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();  //Twr_delay=130ns min，实际测试不加延时也可以
			rawVal = SPIx_ReadWriteByte(0xffff);
			SPI_CS0_H;
			SPI3_TX_ON();
			pos = (rawVal & 0x7fff);
		} break;
		case MODE_SPI_MT6835: {
			uint16_t rawVal2=0;
			uint8_t p[3];
			uint8_t crc;
			
			SPI_CS0_L;
			SPIx_ReadWriteByte(0xA003);
			rawVal = SPIx_ReadWriteByte(0);
			rawVal2= SPIx_ReadWriteByte(0);
			SPI_CS0_H;
			
			p[0]=rawVal>>8;
			p[1]=rawVal;
			p[2]=rawVal2>>8;
			crc=rawVal2;
			
			if(crc_high_first(p,3)!=crc)return;
			pos = ((rawVal<<5)|(rawVal2>>11));
		} break;
		case MODE_SPI_KTH7112:{
			// rawVal = KTH7112_ReadAngle();
				rawVal = KTH7112_ReadAngle_DMA();
				pos_val = rawVal;
				pos = rawVal;
		}break;
		case MODE_INCREMENTAL:
			encoder_set_error(ERROR_UNSUPPORTED_ENCODER_MODE);
			break;
	}
	
	pos_abs_ = pos;
	abs_spi_pos_updated_ = 1;
	if (encoder_config.pre_calibrated) is_ready_ = 1;
}
/*****************************************************************************/
void sample_now(void)
{
	switch(encoder_config.mode)
	{
		case MODE_INCREMENTAL:
			tim_cnt_sample_ = TIM3->CNT;
			break;
		case MODE_HALL:
			HallEncoder_SampleNow(&g_hall_encoder);
			break;
		case MODE_SPI_AS5047P:
		case MODE_SPI_MT6701:
		case MODE_SPI_MA730:
		case MODE_SPI_TLE5012B:
		case MODE_SPI_MT6835:
		case MODE_SPI_KTH7112:
			abs_spi_cb();
			break;
	}
}
/****************************************************************************/

#if 0
bool encoder_update(void)
{
	// update internal encoder state.
	int32_t delta_enc = 0;
	int32_t pos_abs_latched = pos_abs_; //LATCH
	
	switch(encoder_config.mode)
	{
		case MODE_INCREMENTAL: {
			//TODO: use count_in_cpr_ instead as shadow_count_ can overflow or use 64 bit
			int16_t delta_enc_16 = (int16_t)tim_cnt_sample_ - (int16_t)shadow_count_;
			delta_enc = (int32_t)delta_enc_16; //sign extend
		} break;
		
		case MODE_SPI_AS5047P:
		case MODE_SPI_MT6701:
		case MODE_SPI_MA730:
		case MODE_SPI_TLE5012B:
		case MODE_SPI_MT6835: {
			if(abs_spi_pos_updated_ == false)  //正常应该每次都为true，因为sample_now()刚被执行过。
			{
				// Low pass filter the error
				spi_error_rate_ += current_meas_period * (1.0f - spi_error_rate_);
				if (spi_error_rate_ > 0.05f)
				{
					encoder_set_error(ERROR_ABS_SPI_COM_FAIL);
					return 0;
				}
			}
			else
			{
				// Low pass filter the error
				spi_error_rate_ += current_meas_period * (0.0f - spi_error_rate_);
			}
			
			abs_spi_pos_updated_ = false;
			delta_enc = pos_abs_latched - count_in_cpr_; //LATCH
			delta_enc = mod(delta_enc, encoder_config.cpr);
			if(delta_enc > encoder_config.cpr/2)delta_enc -= encoder_config.cpr;
		} break;
		
		default:
			encoder_set_error(ERROR_UNSUPPORTED_ENCODER_MODE);
			return 0;
			//break;
	}
	
	shadow_count_ += delta_enc;
	count_in_cpr_ += delta_enc;
	count_in_cpr_ = mod(count_in_cpr_, encoder_config.cpr);
	
	if((encoder_config.mode==MODE_SPI_AS5047P)||(encoder_config.mode==MODE_SPI_MT6701)||(encoder_config.mode==MODE_SPI_MA730)||(encoder_config.mode==MODE_SPI_TLE5012B)||(encoder_config.mode==MODE_SPI_MT6835))
		count_in_cpr_ = pos_abs_latched;
	
	// Memory for pos_circular
	float pos_cpr_counts_last = pos_cpr_counts_;
	
	// run pll (for now pll is in units of encoder counts)
	// Predict current pos
	pos_estimate_counts_ += current_meas_period * vel_estimate_counts_;
	pos_cpr_counts_      += current_meas_period * vel_estimate_counts_;
	
	// discrete phase detector
	float delta_pos_counts = (float)(shadow_count_ - (int32_t)pos_estimate_counts_);
	float delta_pos_cpr_counts = (float)(count_in_cpr_ - (int32_t)pos_cpr_counts_);
	delta_pos_cpr_counts = wrap_pm(delta_pos_cpr_counts, (float)(encoder_config.cpr));
	//delta_pos_cpr_counts_ += 0.1f * (delta_pos_cpr_counts - delta_pos_cpr_counts_); // for debug
	// pll feedback
	pos_estimate_counts_ += current_meas_period * pll_kp_ * delta_pos_counts;
	pos_cpr_counts_ += current_meas_period * pll_kp_ * delta_pos_cpr_counts;
	pos_cpr_counts_ = fmodf_pos(pos_cpr_counts_, (float)(encoder_config.cpr));
	vel_estimate_counts_ += current_meas_period * pll_ki_ * delta_pos_cpr_counts;
	uint8_t snap_to_zero_vel = false;
	if (fabsf(vel_estimate_counts_) < 0.5f * current_meas_period * pll_ki_)
	{
		vel_estimate_counts_ = 0.0f;  //align delta-sigma on zero to prevent jitter
		snap_to_zero_vel = true;
	}
	
	// Outputs from Encoder for Controller
	pos_estimate_ = pos_estimate_counts_ / (float)encoder_config.cpr;
	vel_estimate_ = vel_estimate_counts_ / (float)encoder_config.cpr;
	
	// TODO: we should strictly require that this value is from the previous iteration
	// to avoid spinout scenarios. However that requires a proper way to reset
	// the encoder from error states.
	float pos_circular = pos_circular_;  //.any().value_or(0.0f);
	pos_circular +=  wrap_pm((pos_cpr_counts_ - pos_cpr_counts_last) / (float)encoder_config.cpr, 1.0f);
	pos_circular = fmodf_pos(pos_circular, ctrl_config.circular_setpoint_range);    //单圈循环模式
	pos_circular_ = pos_circular;
	
	//// run encoder count interpolation
	int32_t corrected_enc = count_in_cpr_ - encoder_config.phase_offset;
	// if we are stopped, make sure we don't randomly drift
	if(snap_to_zero_vel || !encoder_config.enable_phase_interpolation)
	{
		interpolation_ = 0.5f;
		// reset interpolation if encoder edge comes
    // TODO: This isn't correct. At high velocities the first phase in this count may very well not be at the edge.
	}
	else if(delta_enc > 0){
		interpolation_ = 0.0f;
	}else if(delta_enc < 0){
		interpolation_ = 1.0f;
	}
	else {
		// Interpolate (predict) between encoder counts using vel_estimate,
		interpolation_ += current_meas_period * vel_estimate_counts_;
		// don't allow interpolation indicated position outside of [enc, enc+1)
		if (interpolation_ > 1.0f) interpolation_ = 1.0f;
		if (interpolation_ < 0.0f) interpolation_ = 0.0f;
	}
	float interpolated_enc = corrected_enc + interpolation_;
	
	//// compute electrical phase
	//TODO avoid recomputing elec_rad_per_enc every time
	float elec_rad_per_enc = motor_config.pole_pairs * 2 * M_PI * (1.0f / (float)(encoder_config.cpr));
	float ph = elec_rad_per_enc * (interpolated_enc - encoder_config.phase_offset_float);
	
	if(is_ready_)   //校准完成后才能得到角度
	{
		encoder_config.phase_ = wrap_pm_pi(ph) * encoder_config.direction;
		encoder_config.phase_vel_ = (2*M_PI) * vel_estimate_ * motor_config.pole_pairs * encoder_config.direction;
	}
	
	return 1;
}
#else
bool encoder_update(void)
{
    // 更新内部编码器状态
    int32_t delta_enc = 0;  // 编码器计数变化量
    int32_t pos_abs_latched = pos_abs_; // 锁存绝对位置值
    
    // 根据编码器配置模式进行处理
    switch(encoder_config.mode)
    {
        case MODE_INCREMENTAL: {  // 增量式编码器模式
            // TODO: 使用count_in_cpr_代替shadow_count_，因为shadow_count_可能溢出，或者使用64位
            // 计算两次采样之间的编码器计数变化量（16位有符号差值）
            int16_t delta_enc_16 = (int16_t)tim_cnt_sample_ - (int16_t)shadow_count_;
            delta_enc = (int32_t)delta_enc_16; // 符号扩展为32位
        } break;
        
        // 各种SPI绝对式编码器模式
			case MODE_HALL: {
				int32_t hall_cnt;

				if (hall_polarity_calibration_active_)
				{
					if (hall_sample_states_)
					{
						hall_states_seen_count_[g_hall_encoder.raw_hall_state & 0x07u]++;
					}
					return 1;
				}

				if (hall_phase_calibration_active_)
				{
					if (HallEncoder_DecodeState(g_hall_encoder.raw_hall_state ^ encoder_config.hall_polarity, &hall_cnt))
					{
						if (hall_sample_phase_ && hall_last_cnt_valid_)
						{
							int32_t mod_hall_cnt = mod(hall_cnt - hall_last_cnt_, 6);
							uint32_t edge_idx;
							float *edge_phase;

							if (mod_hall_cnt == 0)
							{
								goto hall_phase_skip;
							}
							else if (mod_hall_cnt == 1)
							{
								edge_idx = (uint32_t)hall_cnt;
							}
							else if (mod_hall_cnt == 5)
							{
								edge_idx = (uint32_t)hall_last_cnt_;
							}
							else
							{
								encoder_set_error(ERROR_ILLEGAL_HALL_STATE);
								return 0;
							}

							hall_phase_calib_seen_count_[edge_idx]++;
							edge_phase = &encoder_config.hall_edge_phcnt[edge_idx];
							if (hall_phase_calib_seen_count_[edge_idx] == 1u)
							{
								*edge_phase = openloop_controller_.phase_;
							}
							else
							{
								*edge_phase += (openloop_controller_.phase_ - *edge_phase) / (float)hall_phase_calib_seen_count_[edge_idx];
								*edge_phase = wrap_pm_pi(*edge_phase);
							}
						}

						hall_phase_skip:
						hall_last_cnt_ = hall_cnt;
						hall_last_cnt_valid_ = 1u;
						return 1;
					}

					if (!encoder_config.ignore_illegal_hall_state)
					{
						encoder_set_error(ERROR_ILLEGAL_HALL_STATE);
						return 0;
					}
					return 1;
				}

				HallEncoder_CopyConfigToRuntime();
				if (!HallEncoder_Update(&g_hall_encoder))
				{
					encoder_set_error(ERROR_INVALID_ESTIMATE);
					return 0;
				}
				HallEncoder_ProjectToControlLoop();
				return 1;
			} break;

	        case MODE_SPI_AS5047P:
        case MODE_SPI_MT6701:
        case MODE_SPI_MA730:
        case MODE_SPI_TLE5012B:
		case MODE_SPI_KTH7112:
        case MODE_SPI_MT6835: {
            // 检查绝对位置是否已更新（正常情况下每次应为true，因为sample_now()刚被执行过）
            if(abs_spi_pos_updated_ == false)  
            {
                // 对错误进行低通滤波
                spi_error_rate_ += current_meas_period * (1.0f - spi_error_rate_);
                // 如果错误率超过阈值
                if (spi_error_rate_ > 0.05f)
                {
                    // 设置编码器错误标志
                    encoder_set_error(ERROR_ABS_SPI_COM_FAIL);
                    return 0;  // 返回失败
                }
            }
            else
            {
                // 对错误进行低通滤波（无错误时）
                spi_error_rate_ += current_meas_period * (0.0f - spi_error_rate_);
            }
            
            // 重置绝对位置更新标志
            abs_spi_pos_updated_ = false;
            // 计算绝对位置与CPR内位置的差值
            delta_enc = pos_abs_latched - count_in_cpr_; //LATCH
            // 对差值进行模运算（保持在CPR范围内）
            delta_enc = mod(delta_enc, encoder_config.cpr);
            // 处理差值，确保在[-CPR/2, CPR/2]范围内
            if(delta_enc > encoder_config.cpr/2) delta_enc -= encoder_config.cpr;
        } break;
        
        default:  // 不支持的编码器模式
            encoder_set_error(ERROR_UNSUPPORTED_ENCODER_MODE);
            return 0;  // 返回失败
    }
    
    // 更新影子计数器和CPR内计数
    shadow_count_ += delta_enc;
    count_in_cpr_ += delta_enc;
    // 确保count_in_cpr_在CPR范围内
    count_in_cpr_ = mod(count_in_cpr_, encoder_config.cpr);
    
    // 对于SPI绝对式编码器，直接使用锁存的绝对位置
    if((encoder_config.mode==MODE_SPI_AS5047P)||(encoder_config.mode==MODE_SPI_MT6701)||
       (encoder_config.mode==MODE_SPI_MA730)||(encoder_config.mode==MODE_SPI_TLE5012B)||
       (encoder_config.mode==MODE_SPI_MT6835)||(encoder_config.mode==MODE_SPI_KTH7112))
        count_in_cpr_ = pos_abs_latched;
    
    // 保存上一次的CPR内位置（用于计算循环位置）
    float pos_cpr_counts_last = pos_cpr_counts_;
    
    // 运行PLL（锁相环，单位是编码器计数）
    // 预测当前位置
    pos_estimate_counts_ += current_meas_period * vel_estimate_counts_;
    pos_cpr_counts_      += current_meas_period * vel_estimate_counts_;
    
    // 离散相位检测器
    float delta_pos_counts = (float)(shadow_count_ - (int32_t)pos_estimate_counts_);
    float delta_pos_cpr_counts = (float)(count_in_cpr_ - (int32_t)pos_cpr_counts_);
    // 对CPR内位置差值进行环绕处理
    delta_pos_cpr_counts = wrap_pm(delta_pos_cpr_counts, (float)(encoder_config.cpr));
    
    // PLL反馈
    pos_estimate_counts_ += current_meas_period * pll_kp_ * delta_pos_counts;
    pos_cpr_counts_ += current_meas_period * pll_kp_ * delta_pos_cpr_counts;
    // 确保CPR内位置在[0, CPR)范围内
    pos_cpr_counts_ = fmodf_pos(pos_cpr_counts_, (float)(encoder_config.cpr));
    // 更新速度估计
    vel_estimate_counts_ += current_meas_period * pll_ki_ * delta_pos_cpr_counts;
    
    // 检查是否需要将速度归零（防止抖动）
    uint8_t snap_to_zero_vel = false;
    if (fabsf(vel_estimate_counts_) < 0.5f * current_meas_period * pll_ki_)
    {
        vel_estimate_counts_ = 0.0f;  // 将delta-sigma对齐到零以防止抖动
        snap_to_zero_vel = true;
    }
    
    // 为控制器提供编码器输出
    // 位置估计（归一化到[0,1)）
    pos_estimate_ = pos_estimate_counts_ / (float)encoder_config.cpr;
    // 速度估计（归一化）
    vel_estimate_ = vel_estimate_counts_ / (float)encoder_config.cpr;
    
    // 计算循环位置（用于多圈应用）
    // TODO: 应该严格要求这个值来自上一次迭代，以避免失控情况
    float pos_circular = pos_circular_;  // 获取上一次的循环位置
    // 计算位置变化并处理环绕
    pos_circular += wrap_pm((pos_cpr_counts_ - pos_cpr_counts_last) / (float)encoder_config.cpr, 1.0f);
    // 确保循环位置在设定范围内
    pos_circular = fmodf_pos(pos_circular, ctrl_config.circular_setpoint_range);
    pos_circular_ = pos_circular;  // 更新循环位置
    
    // 运行编码器计数插值
    int32_t corrected_enc = count_in_cpr_ - encoder_config.phase_offset;
    // 如果停止或禁用相位插值，重置插值
    if(snap_to_zero_vel || !encoder_config.enable_phase_interpolation)
    {
        interpolation_ = 0.5f;
        // TODO: 这不完全正确。在高速时，这个计数的第一个相位可能不在边缘
    }
    // 如果编码器计数增加，重置插值为0
    else if(delta_enc > 0){
        interpolation_ = 0.0f;
    }
    // 如果编码器计数减少，重置插值为1
    else if(delta_enc < 0){
        interpolation_ = 1.0f;
    }
    // 否则根据速度估计进行插值
    else {
        // 使用速度估计在编码器计数之间插值（预测）
        interpolation_ += current_meas_period * vel_estimate_counts_;
        // 确保插值位置在[enc, enc+1)范围内
        if (interpolation_ > 1.0f) interpolation_ = 1.0f;
        if (interpolation_ < 0.0f) interpolation_ = 0.0f;
    }
    // 计算插值后的编码器位置
    float interpolated_enc = corrected_enc + interpolation_;
    
    // 计算电角度
    // 计算每个编码器计数对应的电角度弧度数
    float elec_rad_per_enc = motor_config.pole_pairs * 2 * M_PI * (1.0f / (float)(encoder_config.cpr));
    // 计算相位角度
    float ph = elec_rad_per_enc * (interpolated_enc - encoder_config.phase_offset_float);
    
    // 只有在校准完成后才更新角度
    if(is_ready_)
    {
        // 处理相位角度到[-π, π]范围并考虑方向
        encoder_config.phase_ = wrap_pm_pi(ph) * encoder_config.direction;
        // 计算电角速度（考虑极对数和方向）
        encoder_config.phase_vel_ = (2*M_PI) * vel_estimate_ * motor_config.pole_pairs * encoder_config.direction;
    }
    
    return 1;  // 返回成功
}
#endif
/****************************************************************************/



