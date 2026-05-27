#include "Machine_Vision.h"
#include <string.h>

typedef enum {
	MV_PARSE_WAIT_HEAD1 = 0,
	MV_PARSE_WAIT_HEAD2,
	MV_PARSE_WAIT_TYPE,
	MV_PARSE_WAIT_LEN,
	MV_PARSE_WAIT_PAYLOAD,
	MV_PARSE_WAIT_CRC
} MV_ParseState_e;

static UART_HandleTypeDef *s_mv_uart = NULL;

__attribute__((aligned(32))) static uint8_t s_dma_rx_buf[MV_DMA_RX_BUFFER_SIZE];
static uint16_t s_dma_old_pos = 0;

static uint8_t s_ring_buf[MV_RING_BUFFER_SIZE];
static volatile uint16_t s_ring_head = 0;
static volatile uint16_t s_ring_tail = 0;

static MV_ParseState_e s_parse_state = MV_PARSE_WAIT_HEAD1;
static uint8_t s_cur_type = 0;
static uint8_t s_cur_len = 0;
static uint8_t s_payload_buf[MV_MAX_PAYLOAD_LEN];
static uint8_t s_payload_idx = 0;

static MV_Stats_t s_mv_stats = {0};

static uint16_t mv_ring_next(uint16_t idx)
{
	idx++;
	if (idx >= MV_RING_BUFFER_SIZE) {
		idx = 0;
	}
	return idx;
}

static void mv_ring_push(uint8_t byte)
{
	uint16_t next = mv_ring_next(s_ring_head);
	if (next == s_ring_tail) {
		s_mv_stats.ring_overflow_bytes++;
		return;
	}

	s_ring_buf[s_ring_head] = byte;
	s_ring_head = next;
}

static uint8_t mv_ring_pop(uint8_t *out_byte)
{
	if ((out_byte == NULL) || (s_ring_head == s_ring_tail)) {
		return 0U;
	}

	*out_byte = s_ring_buf[s_ring_tail];
	s_ring_tail = mv_ring_next(s_ring_tail);
	return 1U;
}

/* CRC-8/MAXIM（与 Dallas 1-Wire 一致）：RefIn/RefOut=true，等价于对每字节按位 LSB-first，poly 反射为 0x8C */
static uint8_t mv_crc8_maxim(const uint8_t *data, uint16_t len)
{
	uint8_t crc = 0x00U;
	uint16_t i;
	uint8_t j;

	for (i = 0; i < len; i++) {
		crc ^= data[i];
		for (j = 0; j < 8U; j++) {
			if (crc & 0x01U) {
				crc = (uint8_t)((crc >> 1U) ^ 0x8CU);
			} else {
				crc >>= 1U;
			}
		}
	}

	return crc;
}

static int16_t mv_read_i16_le(const uint8_t *p)
{
	return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static uint16_t mv_read_u16_le(const uint8_t *p)
{
	return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static uint8_t mv_decode_frame(MV_ParsedFrame_t *out_frame)
{
	if (out_frame == NULL) {
		return 0U;
	}

	if (s_cur_type == MV_TYPE_VISION) {
		if (s_cur_len != MV_VISION_PAYLOAD_LEN) {
			s_mv_stats.format_error_frames++;
			return 0U;
		}

		out_frame->type = MV_FRAME_VISION;
		out_frame->data.vision.seq = s_payload_buf[0];
		out_frame->data.vision.tag_id = s_payload_buf[1];
		out_frame->data.vision.tag_type = s_payload_buf[2];
		out_frame->data.vision.yaw_cdeg = mv_read_i16_le(&s_payload_buf[3]);
		out_frame->data.vision.pitch_cdeg = mv_read_i16_le(&s_payload_buf[5]);
		out_frame->data.vision.dist_mm = mv_read_u16_le(&s_payload_buf[7]);
		out_frame->data.vision.tx_mm = mv_read_i16_le(&s_payload_buf[9]);
		out_frame->data.vision.ty_mm = mv_read_i16_le(&s_payload_buf[11]);
		out_frame->data.vision.tz_mm = mv_read_i16_le(&s_payload_buf[13]);
		out_frame->data.vision.flags = s_payload_buf[15];
		s_mv_stats.parsed_vision_frames++;
		return 1U;
	}

	if (s_cur_type == MV_TYPE_CMD) {
		if (s_cur_len != MV_CMD_PAYLOAD_LEN) {
			s_mv_stats.format_error_frames++;
			return 0U;
		}

		out_frame->type = MV_FRAME_CMD;
		out_frame->data.cmd.seq = s_payload_buf[0];
		out_frame->data.cmd.action = s_payload_buf[1];
		out_frame->data.cmd.v_0p1pct = mv_read_i16_le(&s_payload_buf[2]);
		out_frame->data.cmd.omega_0p1pct = mv_read_i16_le(&s_payload_buf[4]);
		out_frame->data.cmd.duration_ms = mv_read_u16_le(&s_payload_buf[6]);
		out_frame->data.cmd.reserved = mv_read_u16_le(&s_payload_buf[8]);
		s_mv_stats.parsed_cmd_frames++;
		return 1U;
	}

	s_mv_stats.format_error_frames++;
	return 0U;
}

void MachineVision_Init(UART_HandleTypeDef *huart)
{
	s_mv_uart = huart;
	s_dma_old_pos = 0;
	s_ring_head = 0;
	s_ring_tail = 0;
	s_parse_state = MV_PARSE_WAIT_HEAD1;
	s_cur_type = 0;
	s_cur_len = 0;
	s_payload_idx = 0;
	(void)memset(s_payload_buf, 0, sizeof(s_payload_buf));
	(void)memset(&s_mv_stats, 0, sizeof(s_mv_stats));

	if (s_mv_uart != NULL) {
		(void)HAL_UARTEx_ReceiveToIdle_DMA(s_mv_uart, s_dma_rx_buf, MV_DMA_RX_BUFFER_SIZE);
	}
}

void MachineVision_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
	uint16_t new_pos;
	uint16_t length;
	uint16_t i;

	if ((s_mv_uart == NULL) || (huart == NULL) || (huart->Instance != s_mv_uart->Instance)) {
		return;
	}

	SCB_InvalidateDCache_by_Addr((uint32_t *)((uint32_t)s_dma_rx_buf & ~0x1FU),
								 (MV_DMA_RX_BUFFER_SIZE + 31U) & ~0x1FU);

	new_pos = size;
	length = (uint16_t)((new_pos + MV_DMA_RX_BUFFER_SIZE - s_dma_old_pos) % MV_DMA_RX_BUFFER_SIZE);
	if ((length == 0U) && (new_pos != s_dma_old_pos)) {
		length = MV_DMA_RX_BUFFER_SIZE;
	}

	for (i = 0; i < length; i++) {
		mv_ring_push(s_dma_rx_buf[s_dma_old_pos]);
		s_dma_old_pos++;
		if (s_dma_old_pos >= MV_DMA_RX_BUFFER_SIZE) {
			s_dma_old_pos = 0;
		}
		s_mv_stats.total_rx_bytes++;
	}
}

uint8_t MachineVision_GetFrame(MV_ParsedFrame_t *out_frame)
{
	uint8_t byte = 0;
	uint8_t crc_input[2U + MV_MAX_PAYLOAD_LEN];
	uint8_t expect_crc;

	if (out_frame == NULL) {
		return 0U;
	}

	while (mv_ring_pop(&byte) != 0U) {
		switch (s_parse_state) {
			case MV_PARSE_WAIT_HEAD1:
				if (byte == MV_FRAME_HEAD_1) {
					s_parse_state = MV_PARSE_WAIT_HEAD2;
				}
				break;

			case MV_PARSE_WAIT_HEAD2:
				if (byte == MV_FRAME_HEAD_2) {
					s_parse_state = MV_PARSE_WAIT_TYPE;
				} else if (byte == MV_FRAME_HEAD_1) {
					/* 第二个字节仍是 AA：视为新一帧帧首，继续等 55 */
					s_parse_state = MV_PARSE_WAIT_HEAD2;
				} else {
					s_parse_state = MV_PARSE_WAIT_HEAD1;
				}
				break;

			case MV_PARSE_WAIT_TYPE:
				s_cur_type = byte;
				s_parse_state = MV_PARSE_WAIT_LEN;
				break;

			case MV_PARSE_WAIT_LEN:
				s_cur_len = byte;
				s_payload_idx = 0;
				if (s_cur_len > MV_MAX_PAYLOAD_LEN) {
					s_mv_stats.format_error_frames++;
					s_parse_state = MV_PARSE_WAIT_HEAD1;
				} else if (s_cur_len == 0U) {
					s_parse_state = MV_PARSE_WAIT_CRC;
				} else {
					s_parse_state = MV_PARSE_WAIT_PAYLOAD;
				}
				break;

			case MV_PARSE_WAIT_PAYLOAD:
				s_payload_buf[s_payload_idx++] = byte;
				if (s_payload_idx >= s_cur_len) {
					s_parse_state = MV_PARSE_WAIT_CRC;
				}
				break;

			case MV_PARSE_WAIT_CRC:
				crc_input[0] = s_cur_type;
				crc_input[1] = s_cur_len;
				if (s_cur_len > 0U) {
					(void)memcpy(&crc_input[2], s_payload_buf, s_cur_len);
				}

				expect_crc = mv_crc8_maxim(crc_input, (uint16_t)(2U + s_cur_len));
				s_parse_state = MV_PARSE_WAIT_HEAD1;

				if (expect_crc != byte) {
					s_mv_stats.crc_error_frames++;
					break;
				}

				if (mv_decode_frame(out_frame) != 0U) {
					return 1U;
				}
				break;

			default:
				s_parse_state = MV_PARSE_WAIT_HEAD1;
				break;
		}
	}

	return 0U;
}

void MachineVision_GetStats(MV_Stats_t *out_stats)
{
	if (out_stats == NULL) {
		return;
	}

	*out_stats = s_mv_stats;
}
