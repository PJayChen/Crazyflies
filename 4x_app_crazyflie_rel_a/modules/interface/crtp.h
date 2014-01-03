/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * crtp.h - CrazyRealtimeTransferProtocol stack
 */

#ifndef CRTP_H_
#define CRTP_H_

#include <stdbool.h>
#include <stdint.h>

#define CRTP_MAX_DATA_SIZE 30

#define CRTP_HEADER(port, channel) (((port & 0x0F) << 4) | (channel & 0x0F))

#define CRTP_IS_NULL_PACKET(P) ((P.header&0xF3)==0xF3)

typedef enum {
  CRTP_PORT_CONSOLE     = 0x00,
  CRTP_PORT_PARAM       = 0x02,
  CRTP_PORT_COMMANDER   = 0x03,
  CRTP_PORT_LOG         = 0x05,
	CRTP_PORT_PIDCTRL     = 0x06,
  CRTP_PORT_LINK        = 0x0F,
} CRTPPort;

#pragma anon_unions  //������������ṹ��

typedef __packed struct _CRTPPacket
{
  uint8_t size;
  __packed union {
  __packed  struct {
  __packed    union {
        uint8_t header;
  __packed      struct {
#ifndef CRTP_HEADER_COMPAT
          uint8_t channel     : 2;
          uint8_t reserved    : 2;
          uint8_t port        : 4;
#else
          uint8_t channel  : 2;
          uint8_t port     : 4;
          uint8_t reserved : 2;
#endif
        };
      };
      uint8_t data[CRTP_MAX_DATA_SIZE];
    };
    uint8_t raw[CRTP_MAX_DATA_SIZE+1];
  };
} CRTPPacket;//__attribute__((packed)) 

typedef void (*CrtpCallback)(CRTPPacket *);  //����ص�����

/**
 * Initialize the CRTP stack
 */
void crtpInit(void);

bool crtpTest(void);

/**
 * Initializes the queue and dispatch of an task.
 *
 * @param[in] taskId The id of the CRTP task
 */
void crtpInitTaskQueue(CRTPPort taskId);

/**
 * Register a callback to be called for a particular port.
 *
 * @param[in] port Crtp port for which the callback is set
 * @param[in] cb Callback that will be called when a packet is received on
 *            'port'.
 *
 * @note Only one callback can be registered per port! The last callback
 *       registered will be the one called
 */
void crtpRegisterPortCB(int port, CrtpCallback cb);

/**
 * Put a packet in the TX task
 *
 * If the TX stack is full, the oldest lowest priority packet is dropped
 *
 * @param[in] p CRTPPacket to send
 */
int crtpSendPacket(CRTPPacket *p);

/**
 * Put a packet in the TX task
 *
 * If the TX stack is full, the function block until one place is free (Good for console implementation)
 */
int crtpSendPacketBlock(CRTPPacket *p);

/**
 * Fetch a packet with a specidied task ID.
 *
 * @param[in]  taskId The id of the CRTP task
 * @param[out] p      The CRTP Packet with infomation (unchanged if nothing to fetch)
 *
 * @returns status of fetch from queue
 */
int crtpReceivePacket(CRTPPort taskId, CRTPPacket *p);

/**
 * Fetch a packet with a specidied task ID. Wait some time befor giving up
 *
 * @param[in]  taskId The id of the CRTP task
 * @param[out] p      The CRTP Packet with infomation (unchanged if nothing to fetch)
 * @param[in] wait    Wait time in milisecond
 *
 * @returns status of fetch from queue
 */
int crtpReceivePacketWait(CRTPPort taskId, CRTPPacket *p, int wait);

/**
 * Wait for a packet to arrive for the specified taskID
 *
 * @param[in]  taskId The id of the CRTP task
 * @paran[out] p      The CRTP Packet with information
 *
 * @return status of fetch from queue
 */
int crtpReceivePacketBlock(CRTPPort taskId, CRTPPacket *p);

void crtpPacketReveived(CRTPPacket *p);

/**
 * Function pointer structure to be filled by the CRTP link to permits CRTP to
 * use manu link
 */
struct crtpLinkOperations
{
  int (*setEnable)(bool enable);
  int (*sendPacket)(CRTPPacket *pk);
  int (*receivePacket)(CRTPPacket *pk);
};

void crtpSetLink(struct crtpLinkOperations * lk);

#endif /*CRTP_H_*/
