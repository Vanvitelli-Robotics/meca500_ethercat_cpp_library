#ifndef CUSTOM_PDO_NAME_H
#define CUSTOM_PDO_NAME_H

//-------------------------------------------------------------------//
//                                                                   //
//     This file has been created by the Easy Configurator tool      //
//                                                                   //
//     Easy Configurator project EasyCAT_Gateway.prj
//                                                                   //
//-------------------------------------------------------------------//


#define CUST_BYTE_NUM_OUT	6
#define CUST_BYTE_NUM_IN	36
#define TOT_BYTE_NUM_ROUND_OUT	8
#define TOT_BYTE_NUM_ROUND_IN	36


typedef union												//---- output buffer ----
{
	uint8_t  Byte [TOT_BYTE_NUM_ROUND_OUT];
	struct PACKED
	{
		uint32_t    sample_count;
		uint16_t    command;
	}Cust;
} PROCBUFFER_OUT;


typedef union												//---- input buffer ----
{
	uint8_t  Byte [TOT_BYTE_NUM_ROUND_IN];
	struct PACKED
	{
		uint32_t    rdt_sequence;
		uint32_t    ft_sequence;
		uint32_t    status;
		int32_t     Fx;
		int32_t     Fy;
		int32_t     Fz;
		int32_t     Tx;
		int32_t     Ty;
		int32_t     Tz;
	}Cust;
} PROCBUFFER_IN;

#endif