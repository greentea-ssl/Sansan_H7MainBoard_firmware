/*
 * UARTCommunication.hpp
 *
 *  Created on: 2022/10/01
 *      Author: Eater
 */

#ifndef INC_UARTCOMMUNICATION_HPP_
#define INC_UARTCOMMUNICATION_HPP_

#include "TypeDefinition.hpp"
#include "cobs.h"

#include "main.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#define PACKET_LENGTH (4)
#define BALLINFO_RX_BUF_SIZE (1 << 7) // 64
#define HEAD_BYTE (0x00)

typedef enum{
  BALLINFO_DECODE_NULL_POINTER        = 0x01,
  BALLINFO_DECODE_OUT_BUFFER_OVERFLOW = 0x02,
  BALLINFO_DECODE_ZERO_BYTE_IN_INPUT  = 0x04,
  BALLINFO_DECODE_INPUT_TOO_SHORT     = 0x08,
  BALLINFO_DECODE_UNKNOWN,
  BALLINFO_SUCCESS,
  BALLINFO_FRAME_FAIL,
  BALLINFO_FRAME_INVALID_LENGTH,
  BALLINFO_TIMEOUT
}BallInformationResult;


class BallInfoCommunication{
//private:
private:
	uint8_t tx_buffer[BALLINFO_LENGTH];
	uint8_t rx_ringbuffer[BALLINFO_RX_BUF_SIZE];
	uint8_t rx_frame[255];

	const uint32_t rx_buf_size; //this should be the same as the size of rx_ringbuffer. Changing the ring buffer to dynamic allocation is under consideration.
	const uint16_t ball_info_length;

	uint8_t packet_bytes[PACKET_LENGTH];
	uint8_t packet_bytes_ptr;

	uint8_t packet_head;
	uint8_t packet_tail;
	uint8_t seek_packet_start;
	uint8_t seek_packet_end;

	uint16_t timeout_counter;

	uint16_t timeout_threshold;


	UART_HandleTypeDef* phuart;

	uint32_t rd_ptr; // the pointer to the head of  rx ring buffer

public:
	BallInfoCommunication(UART_HandleTypeDef* huart, uint16_t timeout_threshold = 50): rx_buf_size(BALLINFO_RX_BUF_SIZE), ball_info_length(sizeof(BallInformation)){
		phuart = huart;
		packet_bytes_ptr = 0;

		timeout_counter = 0;
		this->timeout_threshold = timeout_threshold;
	}

	void init(){
		rd_ptr = 0;
    packet_head = 0;
    packet_tail = 0;
    seek_packet_start = 0;
    seek_packet_end = 0;
    HAL_UART_Receive_DMA(phuart, rx_ringbuffer, rx_buf_size);
    timeout_counter = 0;
	}

	inline uint32_t getDMAWritePtr(){ // calculating the tail of the ring buffer
		return (rx_buf_size - __HAL_DMA_GET_COUNTER(phuart->hdmarx)) & (rx_buf_size - 1);
	}

	bool isRxbufEmpty(){
		return (rd_ptr == getDMAWritePtr());
	}

	void sendBallData(BallInformation* pball_info){
		memcpy(tx_buffer, (uint8_t*) pball_info, BALLINFO_LENGTH);
		sendBytes(tx_buffer, BALLINFO_LENGTH);
	}

	void sendBytes(uint8_t* tx, uint32_t size){//send bytes with desired size
		HAL_UART_Transmit_DMA(phuart, tx, size);
	}

	uint32_t receiveBytes(uint8_t* rx, uint32_t size){ // read bytes with desired size from the rx ring buffer
		if(size > rx_buf_size){
			size = rx_buf_size;
		}
    uint32_t read_size = min((getDMAWritePtr() - rd_ptr) & (rx_buf_size - 1), size);
//    printf("ndtr, wd_ptr, rd_ptr,size, read_size:[%d, (%d, %d), %d, %d]\n\r", phuart->hdmarx->Instance->NDTR, getDMAWritePtr(), rd_ptr, size, read_size);

    memcpy(rx, &rx_ringbuffer[rd_ptr], read_size);
    rd_ptr += read_size;
    rd_ptr &= (rx_buf_size - 1);

    return read_size;
	}


  int16_t ReceiveFrame(uint8_t* frame_data){ // return -1 if fail otherwise return the size
//    bool r = false;
    bool is_delimiter_found = false;
    int16_t length;

    int16_t tx_ptr = (getDMAWritePtr() - 1) & (rx_buf_size - 1);

    if(((tx_ptr) & (rx_buf_size - 1)) == ((rd_ptr) & (rx_buf_size - 1))){
      return -1;
    }

//    printf("heading from%d to %ld: ", tx_ptr, rd_ptr);
    for(uint8_t i = 0; i < BALLINFO_RX_BUF_SIZE; i++){ // search for delimiter from tx pointer to rx pointer(inverse)
//      printf("0x%02X ", rx_ringbuffer[(tx_ptr - i) & (rx_buf_size - 1)]);
      if(rx_ringbuffer[(tx_ptr - i) & (rx_buf_size - 1)] == HEAD_BYTE){
        //the delimiter is found
        if(rx_ringbuffer[(tx_ptr - i - 1) & (rx_buf_size - 1)] != HEAD_BYTE){
          seek_packet_end = (tx_ptr - i - 1) & (rx_buf_size - 1);
          is_delimiter_found = true;
          break;
        }
      }
      if(((tx_ptr - i) & (rx_buf_size - 1)) == ((rd_ptr) & (rx_buf_size - 1))){
        break;
      }
    }
//    printf("\n\r");

    if(!is_delimiter_found){
      return -1;
    }
    length = 0;
//    printf("rx frame: ");
    for(uint8_t i = 0; i < BALLINFO_RX_BUF_SIZE; i++){ // search for delimiter
      if(((rd_ptr) & (rx_buf_size - 1)) == ((seek_packet_end - i) & (rx_buf_size - 1))){ // reach the head of the ring buffer
        return -1;
        break;
      }
//      printf("0x%02X ", rx_ringbuffer[(seek_packet_end - i) & (rx_buf_size - 1)]);
      if(rx_ringbuffer[(seek_packet_end - i) & (rx_buf_size - 1)] == HEAD_BYTE){
        seek_packet_start = (seek_packet_end - i + 1) & (rx_buf_size - 1);
        length = i;
        rd_ptr = (seek_packet_end + 1) & (rx_buf_size - 1);
        break;
      }
    }
//    printf("\n\r");
//    printf("next rd_ptr:%ld\n\r", rd_ptr);

//    int16_t length = seek_packet_end - seek_packet_start + 1;
    memcpy(frame_data, &rx_ringbuffer[seek_packet_start], length);

    return length;
  }

  BallInformationResult ReceiveBallInformation(BallInformation* ball_data){
    int16_t result = ReceiveFrame(rx_frame);

    if(timeout_counter >= timeout_threshold){
    	return BALLINFO_TIMEOUT;
    }
    timeout_counter ++;

    if(result < 0){
//      printf("ERROR: BALLINFO_FRAME_FAIL\n\r");
      return BALLINFO_FRAME_FAIL;
//    } else if(result != ball_info_length){
//      printf("length: %d\n\r", result);
//      return BALLINFO_FRAME_INVALID_LENGTH;
    }else{
      uint8_t tmp[ball_info_length];
      //printf("rx:");
//      print_bytes(rx_frame, result);
      //printf("\n\r");
      cobs_decode_result dec_rst = cobs_decode(tmp, BALLINFO_RX_BUF_SIZE, rx_frame, result);
      switch(dec_rst.status){
      case COBS_DECODE_OK:
        memcpy(ball_data, tmp, rx_buf_size);
        timeout_counter = 0;
        return BALLINFO_SUCCESS;
        break;
      case COBS_DECODE_NULL_POINTER:
        return BALLINFO_DECODE_NULL_POINTER;
        break;
      case COBS_DECODE_OUT_BUFFER_OVERFLOW:
        return BALLINFO_DECODE_OUT_BUFFER_OVERFLOW;
        break;
      case COBS_DECODE_ZERO_BYTE_IN_INPUT:
        return BALLINFO_DECODE_ZERO_BYTE_IN_INPUT;
        break;
      case COBS_DECODE_INPUT_TOO_SHORT:
        return BALLINFO_DECODE_INPUT_TOO_SHORT;
      default:
        printf("ERROR:%d", dec_rst.status);
        return BALLINFO_DECODE_UNKNOWN;
      }
    }
  }

private:
	template<typename T> T min(T a, T b){
		if(a < b)
			return a;
		else
			return b;
	}
	void print_bytes(unsigned char* bytes, uint32_t len){
	  for(uint32_t i = 0; i <len; i++){
	    printf("0x%02X ", bytes[i] & 0xff);
	  }
	}

};






#endif /* INC_UARTCOMMUNICATION_HPP_ */
