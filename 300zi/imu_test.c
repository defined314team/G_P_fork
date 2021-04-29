#include <libserialport.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define OPENIMU_300ZI_DEV_NAME   "/dev/ttyUSB0"

#define OPENIMU_300ZI_BAUDRATE   230400

#define E2_PACKET_PREAMBLE_1ST   0x55
#define E2_PACKET_PREAMBLE_2ND   0x55
#define E2_PACKET_TYPE       0x6532 


#define E2_PACKET_PREAMBLE_LEN  2
#define E2_PACKET_PACKET_CODE_LEN   2

#define E2_PACKET_PACKET_LEN_LEN    1 
#define E2_PACKET_PAYLOAD_LEN         123
#define E2_PACKET_CRC_LEN           2

#define E2_PACKET_REST_LEN (E2_PACKET_PACKET_LEN_LEN + E2_PACKET_PAYLOAD_LEN + E2_PACKET_CRC_LEN)

#define E2_PACKET_TOTAL_LEN (E2_PACKET_PREAMBLE_LEN + E2_PACKET_PACKET_CODE_LEN + E2_PACKET_REST_LEN)


typedef struct __attribute__((__packed__)){
    unsigned short preamble;
    unsigned short code;
    unsigned char len;
    
    unsigned int system_time;         // 0
    double  system_time_d;  // 4
    float roll; // 12
    float pitch; // 16
    float yaw; // 20
    float x_accel; // 24
    float y_accel; // 28
    float z_accel; // 32
    float x_accel_bias; // 36
    float y_accel_bias;  // 40
    float z_accel_bias; // 44
	float x_gyro; // 48
	float y_gyro; // 52
	float z_gyro; // 56	
	float x_gyro_bias; // 60
	float y_gyro_bias; // 64
	float z_gyro_bias; // 68	
	float north_vel; // 72
	float east_vel;  // 76
	float down_vel;  // 80
	float x_mag; // 84
	float y_mag; // 88
	float z_mag; // 92
	double latitude; // 96
	double longitude; // 104
    double altitude; // 112
    unsigned char opmode; // 120
    unsigned char lin_accel_switch; // 121
    unsigned char trun_switch; // 122
    
    unsigned short crc;
}type_e2_packet;



typedef enum 
{
	E2_RECEIVER_PREAMBLE_1ST,  // Wait for Preamble 1st byte
	E2_RECEIVER_PREAMBLE_2ND,  // Wait for Preamble 2nd byte	
	E2_RECEIVER_PACKET_CODE, // Wait for Packet Code
	E2_RECEIVER_REST, // Wait for len, payload, crc
} type_e2_receiver_state;


/* Example of how to send and receive data.
 *
 * This example file is released to the public domain. */

/* Helper function for error handling. */
int check(enum sp_return result);

int main(int argc, char **argv)
{
	/* This example can be used with one or two ports. With one port, it
	 * will send data and try to receive it on the same port. This can be
	 * done by connecting a single wire between the TX and RX pins of the
	 * port.
	 *
	 * Alternatively it can be used with two serial ports connected to each
	 * other, so that data can be sent on one and received on the other.
	 * This can be done with two ports with TX/RX cross-connected, e.g. by
	 * a "null modem" cable, or with a pair of interconnected virtual ports,
	 * such as those created by com0com on Windows or tty0tty on Linux. */

	/* The ports we will use. */
	struct sp_port *ports;
	
	/* The set of events we will wait for. */	
	struct sp_event_set *event_set; 


	type_e2_receiver_state rcv_state = E2_RECEIVER_PREAMBLE_1ST;

	unsigned char rcv_buf[E2_PACKET_TOTAL_LEN];	
	int buf_idx = 0;

	int bytes_waiting;

	type_e2_packet e2_packet;
	

	/* Open and configure each port. */
	printf("Looking for port %s.\n", OPENIMU_300ZI_DEV_NAME);
	check(sp_get_port_by_name(OPENIMU_300ZI_DEV_NAME, &ports));

    printf("Opening port.\n");
	check(sp_open(ports, SP_MODE_READ_WRITE));

	printf("Setting port to %d 8N1, no flow control.\n", OPENIMU_300ZI_BAUDRATE);
	check(sp_set_baudrate(ports, OPENIMU_300ZI_BAUDRATE));
	check(sp_set_bits(ports, 8));
	check(sp_set_parity(ports, SP_PARITY_NONE));
	check(sp_set_stopbits(ports, 1));
	check(sp_set_flowcontrol(ports, SP_FLOWCONTROL_NONE));

	/* Allocate the event set. */	
	check(sp_new_event_set(&event_set));

	printf("Adding port RX event to event set.\n");		
	check(sp_add_port_events(event_set, ports, SP_EVENT_RX_READY));


	for(;;)
	{
		/* Now we can call sp_wait() to await any event in the set.	 
		* It will return when an event occurs, or the timeout elapses. */	
		check(sp_wait(event_set, 1000));

		/* Get number of bytes waiting. */		
		bytes_waiting = check(sp_input_waiting(ports));		
		
		if(bytes_waiting < 1) continue;

		printf("state = %d, bytes_wating = %d\n", rcv_state, bytes_waiting);

		switch(rcv_state)
		{
			case E2_RECEIVER_PREAMBLE_1ST:  // Wait for Preamble
				buf_idx = 0;

				if(sp_nonblocking_read(ports, &rcv_buf[buf_idx], 1) == 1)
				{
					if(rcv_buf[buf_idx] == E2_PACKET_PREAMBLE_1ST)
					{
						buf_idx++;
						rcv_state = E2_RECEIVER_PREAMBLE_2ND;
						
					}
				}
				break;

			case E2_RECEIVER_PREAMBLE_2ND:  // Wait for Preamble

				if(sp_nonblocking_read	(ports, &rcv_buf[buf_idx], 1) == 1)
				{
					if(rcv_buf[buf_idx] == E2_PACKET_PREAMBLE_2ND)
					{
						buf_idx++;
						rcv_state = E2_RECEIVER_PACKET_CODE;
						
					}
				}
				break;				
				
			case E2_RECEIVER_PACKET_CODE:
				if(bytes_waiting < E2_PACKET_PACKET_CODE_LEN) continue;
				
				if(sp_nonblocking_read	(ports, &rcv_buf[buf_idx], 2) == 2)
				{
					if((rcv_buf[buf_idx] == ((E2_PACKET_TYPE >> 8) & 0xFF))
							&& (rcv_buf[buf_idx+1] == (E2_PACKET_TYPE & 0xFF)))
					{
						buf_idx+=2;
						rcv_state = E2_RECEIVER_REST;
					}
					else
					{
						rcv_state = E2_RECEIVER_PREAMBLE_1ST;
					}
					
				}
				else
				{
					rcv_state = E2_RECEIVER_PREAMBLE_1ST;
				}
				break;

			case E2_RECEIVER_REST:
				if(bytes_waiting < E2_PACKET_REST_LEN) continue;
				
				if(sp_nonblocking_read	(ports, &rcv_buf[buf_idx], E2_PACKET_REST_LEN) == E2_PACKET_REST_LEN)
				{
					// All E2 Frame Received...

					memcpy(&e2_packet, rcv_buf, sizeof(e2_packet));
					

					for(int i=0; i<E2_PACKET_TOTAL_LEN; i++)
					{
						if(i%20 == 0) printf("\n");
						printf( "%02x", rcv_buf[i]);
					}
					printf("%x, %x %x\n", e2_packet.preamble, e2_packet.code, e2_packet.len);	
 					printf("%d, %lf\n", e2_packet.system_time, e2_packet.system_time_d);	
 					printf("%f, %f %f\n", e2_packet.roll, e2_packet.pitch, e2_packet.yaw);	
 					printf("%f, %f %f\n", e2_packet.x_accel, e2_packet.y_accel, e2_packet.z_accel);	
 					printf("%f, %f %f\n", e2_packet.x_accel_bias, e2_packet.y_accel_bias, e2_packet.z_accel_bias);	

					printf("All e2_packet Received, lat = %lf, lon = %lf\n", e2_packet.latitude, e2_packet.longitude);

					rcv_state = E2_RECEIVER_PREAMBLE_1ST;
				}
				else
				{
					rcv_state = E2_RECEIVER_PREAMBLE_1ST;
				}
				break;

				
			default:
				break;
		}
	}

	sp_free_event_set(event_set);

	/* Close ports and free resources. */
	check(sp_close(ports));
	sp_free_port(ports);

	return 0;
}

/* Helper function for error handling. */
int check(enum sp_return result)
{
	/* For this example we'll just exit on any error by calling abort(). */
	char *error_message;

	switch (result) {
	case SP_ERR_ARG:
		printf("Error: Invalid argument.\n");
		abort();
	case SP_ERR_FAIL:
		error_message = sp_last_error_message();
		printf("Error: Failed: %s\n", error_message);
		sp_free_error_message(error_message);
		abort();
	case SP_ERR_SUPP:
		printf("Error: Not supported.\n");
		abort();
	case SP_ERR_MEM:
		printf("Error: Couldn't allocate memory.\n");
		abort();
	case SP_OK:
	default:
		return result;
	}
}
