#ifndef __AWINIC_ALGO_H__
#define __AWINIC_ALGO_H__

/* The operation completed with no errors. */
/* Success.*/
#define ADSP_EOK          ( 0  )
/** General failure. */
#define ADSP_EFAILED      ( -1  )
/** Bad operation parameter. */
#define ADSP_EBADPARAM    ( -2  )
/** Unsupported routine or operation. */
#define ADSP_EUNSUPPORTED ( -3  )
/** Unsupported version. */
#define ADSP_EVERSION     ( -4  )
/** Unexpected problem encountered. */
#define ADSP_EUNEXPECTED  ( -5  )

typedef struct media_info{
    unsigned int num_channels;
    unsigned int bits_per_sample;
    unsigned int bit_qactor_sample;
    unsigned int sampling_rate;
    unsigned int data_is_signed;
}media_info_t;

int AwinicSetMediaInfo(void *env_ptr,void *info);
typedef unsigned long (*AwGetSize_t)(void);
typedef int (*AwInit_t)(void *,const char*);
typedef int (*AwEnd_t)(void *);
typedef int (*AwReset_t)(void *);
typedef int (*AwHandle_t)(void *,void *,unsigned long);
typedef int (*AwSetMediaInfo_t)(void*,void*);
typedef int (*AwSetVmax_t)(void*,void*);
typedef int (*AwGetVmax_t)(void*,void*);
typedef int (*AwGetActiveFlag_t)(void*,void*);

typedef struct AWINIC_SKT_ALGO{
	AwGetSize_t getSize;
	AwInit_t init;
	AwEnd_t end;
	AwReset_t reset;
	AwHandle_t process;
	AwSetMediaInfo_t setMediaInfo;
	AwSetVmax_t setVmax;
	AwGetVmax_t getVmax;
	AwGetActiveFlag_t getActiveFlag;
	media_info_t info;
	bool  is_module_ready;
	bool  is_module_enable;
	char*  module_context_buffer;
	char*  audio_data_buffer;
	void *awinic_lib;
}aw_skt_t;

#endif

