#ifndef MACHINE_VISION_H
#define MACHINE_VISION_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "main.h"

#define MV_DMA_RX_BUFFER_SIZE   128U
#define MV_RING_BUFFER_SIZE     512U
#define MV_MAX_PAYLOAD_LEN      32U

#define MV_FRAME_HEAD_1         0xAAU
#define MV_FRAME_HEAD_2         0x55U

#define MV_TYPE_VISION          0x01U
#define MV_TYPE_CMD             0x02U

#define MV_VISION_PAYLOAD_LEN   16U
#define MV_CMD_PAYLOAD_LEN      10U

typedef enum {
	MV_FRAME_NONE = 0,
	MV_FRAME_VISION = 1,
	MV_FRAME_CMD = 2
} MV_FrameType_e;

typedef struct {
	uint8_t seq;
	uint8_t tag_id;
	uint8_t tag_type;
	int16_t yaw_cdeg;
	int16_t pitch_cdeg;
	uint16_t dist_mm;
	int16_t tx_mm;
	int16_t ty_mm;
	int16_t tz_mm;
	uint8_t flags;
} MV_VisionFrame_t;

typedef struct {
	uint8_t seq;
	uint8_t action;
	int16_t v_0p1pct;
	int16_t omega_0p1pct;
	uint16_t duration_ms;
	uint16_t reserved;
} MV_CmdFrame_t;

typedef struct {
	MV_FrameType_e type;
	union {
		MV_VisionFrame_t vision;
		MV_CmdFrame_t cmd;
	} data;
} MV_ParsedFrame_t;

typedef struct {
	uint32_t total_rx_bytes;
	uint32_t parsed_vision_frames;
	uint32_t parsed_cmd_frames;
	uint32_t crc_error_frames;
	uint32_t format_error_frames;
	uint32_t ring_overflow_bytes;
} MV_Stats_t;

void MachineVision_Init(UART_HandleTypeDef *huart);
void MachineVision_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size);
uint8_t MachineVision_GetFrame(MV_ParsedFrame_t *out_frame);
void MachineVision_GetStats(MV_Stats_t *out_stats);

#ifdef __cplusplus
}
#endif

#endif /* MACHINE_VISION_H */
