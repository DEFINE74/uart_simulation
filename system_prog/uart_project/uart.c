// One-thread UART User-Space Simulation
/*
 * This file consists realisation of UART
 * transmitting and receiving UART frames
 * with a ring buffers
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#define UART_STOP_BIT_POS  9
#define UART_START_BIT_POS 0
#define BUFFER_CAPACITY	   4

_Static_assert(((BUFFER_CAPACITY & (BUFFER_CAPACITY - 1)) == 0),
	       "BUFFER_CAPACITY must be a power of two!");

typedef enum {
	UART_SUCCESS = 0,
	UART_ERROR_OVERFLOW,
	UART_ERROR_EMPTY,
	UART_ERROR_NULLPTR,
	UART_ERROR_FRAME,
	UART_ERROR_BAUDRATE
} uart_status_t;

struct uart_ring_buffer {
	uint8_t buffer[BUFFER_CAPACITY];
	volatile uint32_t head;
	volatile uint32_t tail;
	volatile uint32_t count;
};

struct uart_device {
	struct uart_ring_buffer buffer_rx;
	struct uart_ring_buffer buffer_tx;
	uint32_t baud_speed;
	char ID;
};

struct uart_frame {
	uint16_t data;
};

/* Ring buffer functions */
uart_status_t buffer_init(struct uart_ring_buffer *buffer)
{
	if (!buffer)
		return UART_ERROR_NULLPTR;
	*buffer = (struct uart_ring_buffer) {0};
	return UART_SUCCESS;
}

uart_status_t push_buffer(struct uart_ring_buffer *buffer, uint8_t data)
{
	if (!buffer) {
		return UART_ERROR_NULLPTR;
	}
	if (buffer->count == BUFFER_CAPACITY) {
		return UART_ERROR_OVERFLOW;
	}
	buffer->buffer[buffer->head] = data;

	buffer->head = (buffer->head + 1) & (BUFFER_CAPACITY - 1);
	buffer->count++;

	return UART_SUCCESS;
}

uart_status_t pop_buffer(struct uart_ring_buffer *buffer, uint8_t *out_data)
{
	if (!buffer || !out_data) {
		return UART_ERROR_NULLPTR;
	}
	if (buffer->count == 0) {
		return UART_ERROR_EMPTY;
	}
	*out_data = buffer->buffer[buffer->tail];
	buffer->tail = (buffer->tail + 1) & (BUFFER_CAPACITY - 1);
	buffer->count--;

	return UART_SUCCESS;
}

/* Service functions for print information */
void print_bin(uint32_t number, uint8_t bits)
{
	for (int i = bits - 1; i >= 0; i--) {
		if (number & (1U << i)) {
			printf("1");
		} else {
			printf("0");
		}
		if (i % 4 == 0)
			printf(" ");
	}
	printf("\n");
}

void print_buffer(const struct uart_ring_buffer *buffer)
{
	for (size_t i = 0; i < BUFFER_CAPACITY; i++) {
		printf("[%u] ", buffer->buffer[i]);
	}
	printf("\n");
}

const char *uart_status_to_str(uart_status_t status)
{
	switch (status) {
	case UART_SUCCESS:
		return "SUCCESS";
	case UART_ERROR_OVERFLOW:
		return "ERROR_OVERFLOW";
	case UART_ERROR_EMPTY:
		return "ERROR_EMPTY";
	case UART_ERROR_NULLPTR:
		return "ERROR_NULLPTR";
	case UART_ERROR_FRAME:
		return "ERROR_FRAME";
	case UART_ERROR_BAUDRATE:
		return "ERROR_BAUDRATE";
	default:
		return "UNKNOWN_ERROR";
	}
}

/* Device life cycle functions */
struct uart_device *init_device(uint32_t baud_speed, char ID)
{
	struct uart_device *device = malloc(sizeof(struct uart_device));
	if (!device) {
		return NULL;
	}

	if (buffer_init(&device->buffer_rx) != UART_SUCCESS ||
	    buffer_init(&device->buffer_tx) != UART_SUCCESS) {
		printf("[INIT_DEVICE]: Error initialisation buffer\n");
		free(device);
		return NULL;
	}

	device->ID = ID;
	device->baud_speed = baud_speed;

	printf("[INIT_DEVICE]: Initialised Device %c - [address: %p, buffer "
	       "size: %u, baud speed: %u]\n",
	       device->ID, device, BUFFER_CAPACITY, device->baud_speed);
	return device;
}

void destroy_device(struct uart_device **device)
{
	if (!*(device)) {
		return;
	}
	printf("[DESTROY_DEVICE]: Destroying Device %c...\n", (*device)->ID);
	free(*device);

	*device = NULL;
}

/* UART functions */
struct uart_frame uart_pack_frame(uint8_t data)
{
	struct uart_frame frame = {0};
	frame.data |= (1U << UART_STOP_BIT_POS);
	frame.data |= (data << 1);

	printf("[PACK_FRAME]: Created UART Frame from original data: ");
	print_bin(data, 8);
	printf("[PACK_FRAME]: UART Frame: ");
	print_bin(frame.data, 16);

	return frame;
}

uart_status_t uart_unpack_frame(struct uart_frame frame, uint8_t *rx_data)
{
	// if we dont have stop bit or start bit - our frame is corrupted
	if (!(frame.data & (1U << UART_STOP_BIT_POS)) ||
	    frame.data & (1U << UART_START_BIT_POS)) {
		printf("[UNPACK_FRAME]: Error, incorrect frame\n");
		return UART_ERROR_FRAME;
	}

	*rx_data = (frame.data >> 1) & 0xFF;
	return UART_SUCCESS;
}

uart_status_t uart_transmit(struct uart_device *tx, struct uart_device *rx)
{
	// checking correctness of devices and baud speed
	if (!tx) {
		printf("[UART_TRANSMIT]: Error, invalid TX device - %s\n",
		       uart_status_to_str(UART_ERROR_NULLPTR));
		return UART_ERROR_NULLPTR;
	}
	if (!rx) {
		printf("[UART_TRANSMIT]: Error, invalid RX device - %s\n",
		       uart_status_to_str(UART_ERROR_NULLPTR));
		return UART_ERROR_NULLPTR;
	}
	if (rx->baud_speed != tx->baud_speed) {
		printf("[UART_TRANSMIT]: Error, not same baud speed - %s\n",
		       uart_status_to_str(UART_ERROR_BAUDRATE));
		return UART_ERROR_BAUDRATE;
	}

	// tx procedure
	uint8_t tx_data = 0;
	uart_status_t status = pop_buffer(&tx->buffer_tx, &tx_data);

	if (status != UART_SUCCESS) {
		printf("[UART_TRANSMIT]: Error, can not pop the data from TX "
		       "Buffer of Device %c - %s\n",
		       tx->ID, uart_status_to_str(status));
		return status;
	}

	struct uart_frame tx_frame = uart_pack_frame(tx_data);

	printf("[UART_TRANSMIT]: Device %c transmitted the data to Device %c\n",
	       tx->ID, rx->ID);
	printf("[UART_TRANSMIT]: TX Buffer of Device %c: ", tx->ID);
	print_buffer(&tx->buffer_tx);

	// rx procedure
	uint8_t rx_data = 0;

	if (uart_unpack_frame(tx_frame, &rx_data) != UART_SUCCESS) {
		printf("[UART_TRANSMIT]: Error, corrupted frame\n");
		return UART_ERROR_FRAME;
	}
	if (push_buffer(&rx->buffer_rx, rx_data) != UART_SUCCESS) {
		printf(
		    "[UART_TRANSMIT]: Error, RX Buffer of Device %c is full\n",
		    rx->ID);
		return UART_ERROR_OVERFLOW;
	}
	printf("[UART_TRANSMIT]: Device %c received the data from "
	       "Device %c\n",
	       rx->ID, tx->ID);
	printf("[UART_TRANSMIT]: RX Buffer of Device %c: ", rx->ID);
	print_buffer(&rx->buffer_rx);

	return UART_SUCCESS;
}
uart_status_t uart_read_data(struct uart_device *rx, uint8_t *data)
{
	if (!rx) {
		printf("[UART_READ]: Error, invalid RX device - %s\n",
		       uart_status_to_str(UART_ERROR_NULLPTR));
		return UART_ERROR_NULLPTR;
	}

	uart_status_t status = pop_buffer(&rx->buffer_rx, data);
	if (status != UART_SUCCESS) {
		printf("[UART_READ]: Error, can not pop the data from RX "
		       "Buffer of Device %c - %s",
		       rx->ID, uart_status_to_str(status));
		return status;
	}
	return UART_SUCCESS;
}
uart_status_t uart_put_tx_data(struct uart_device *device, uint8_t data)
{
	uart_status_t status = push_buffer(&device->buffer_tx, data);
	if (status != UART_SUCCESS) {
		printf("[PUT_TX]: Error, TX Buffer of Device %c is full\n",
		       device->ID);
		return status;
	}
	printf("[PUT_TX]: Successfully added data - %i, into TX Buffer of "
	       "Device %c\n",
	       data, device->ID);

	return UART_SUCCESS;
}

/* Testing realisation */
int main(void)
{
	const uint32_t baud_speed = 9600;
	struct uart_device *device_a = init_device(baud_speed, 'A');
	struct uart_device *device_b = init_device(baud_speed, 'B');

	uart_put_tx_data(device_a, 52);
	uart_put_tx_data(device_a, 1);
	uart_put_tx_data(device_a, 2);

	uart_transmit(device_a, device_b);
	uart_transmit(device_a, device_b);

	uint8_t received_byte = 0;
	uart_read_data(device_b, &received_byte);
	printf("[MAIN]: Device %c proccessed byte: %u\n", device_b->ID,
	       received_byte);
	uart_read_data(device_b, &received_byte);
	printf("[MAIN]: Device %c proccessed byte: %u\n", device_b->ID,
	       received_byte);

	uart_transmit(device_a, device_b);

	uart_put_tx_data(device_a, 7);
	uart_transmit(device_a, device_b);

	destroy_device(&device_a);
	destroy_device(&device_b);
	return 0;
}