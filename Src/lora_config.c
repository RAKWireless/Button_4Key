/*
 * lora_config.c
 *
 *  Created on: 2019年7月22日
 *      Author: dell
 */
#include "lora_config.h"
#include "stddef.h"
#include "stdio.h"
#include "string.h"
#include "main.h"
#include "eeprom.h"
#include "LoRaMac.h"

typedef enum {
  CFG_READ,
  CFG_WRITE
}cfg_op;

#define MAX_ARGV 20
lora_config_t lora_config;




//static int handle_lora_config(lora_config_t *config, int argc, char *argv[], char *in);
static int analysis_second_string(int argc , char * argv[],cfg_op op);
static int handle_device_config(int argc , char * argv[],cfg_op op);
static int handle_lora_config(int argc , char * argv[],cfg_op op);
static int parse2_args(char* str, char* argv[]);


static void McpsConfirm( McpsConfirm_t *mcpsConfirm );
static void McpsIndication( McpsIndication_t *mcpsIndication );
static void MlmeConfirm( MlmeConfirm_t *mlmeConfirm );
static void MlmeIndication( MlmeIndication_t *mlmeIndication );


static int dev_eui(int argc , char * argv[],cfg_op op);

struct config_cmd
{
    char *name;
    int (*function) (int argc, char *argv[],cfg_op op);//int (*function) (lora_config_t *config, int argc, char *argv[], char *in, cfg_op op);
};

struct config_cmd last_cmd_str[]=
{



		{"dev_eui",dev_eui},



};

struct device_cfgt
{
    /** The name of the CONFIG command */
    char *name;
    /** The help text associated with the command */
    //const char *help;
    /** The function that should be invoked for this command. */
    int (*function) (int argc , char * argv[],cfg_op op);
};

struct device_cfgt device_cfg[] =
{
		{"device",                handle_device_config},
		{"lora",                  handle_lora_config},
};

int read_config(int argc , char * argv[])
{
		int ret;
		//cfg_op CFG_READ;

	    ret = analysis_second_string(argc ,argv, CFG_READ);


	    if (ret < 0)
	    {
	        return ret;
	    }
	    return ret;
}

int set_config(int argc , char * argv[])
{
		int ret;
		//cfg_op CFG_WRITE;
	    ret = analysis_second_string(argc ,argv,CFG_WRITE);

	    if (ret < 0)
	    {
	        return ret;
	    }
	    return ret;
}



static int parse2_args(char* str, char* argv[])
{
    int i = 0;
    char* ch = str;

    while(*ch != '\0')
    {
        i++;
        /*Check if length exceeds*/
        if (i > MAX_ARGV) {
            return 0;
        }

        argv[i-1] = ch;

        while(*ch != ',' && *ch != '\0' && *ch != '&')
        {
            if(*ch == ':' && i == 1)
            {
            	*ch=0;
                break;
            }
            else
                ch++;
        }
        ch++;


   }
    return i;
}

static int analysis_second_string(int argc , char * argv[],cfg_op op)    //后半截字符传入   每次到分号重新解析
{
    int i;
    int ret;
    char* str= argv[1];
    char *argv_temp[MAX_ARGV];

    //printf("FUNC %s	  argv[1]	%s\r\n",__func__,argv[1]);

        argc = parse2_args(str, argv_temp);    //
        printf("(2) argc:%d			argv[0]:%s		argv[1]:%s\r\n",argc,argv_temp[0],argv_temp[1]);
        if (argc > 2) {
            printf("Too many parameters.\n");
            return -1;
        }else if(argc < 2)
        {
            printf("Too few parameters.\n");
            return -1;
        }
//
        for (i = 0; i < sizeof(device_cfg)/sizeof(struct device_cfgt); i++)
        {
            if (strcmp(argv_temp[0], device_cfg[i].name) == 0)
            {
                ret = device_cfg[i].function(argc,argv_temp,op);
                if (ret < 0)
                {
                    printf("AT CMD parameters error.\n");
                    return ret;
                }
                break;
            }
        }
        if (i == sizeof(device_cfg)/sizeof(struct device_cfgt))
        {
            printf("AT CMD not found.\n");
            return -1;
        }

    return 0;
}



static int handle_lora_config(int argc , char * argv[],cfg_op op)
{
	printf("handle_lora_config\r\n");
	return 0;
}


static int handle_device_config(int argc , char * argv[],cfg_op op)
{
	    int i;
	    int ret;

	    char *argv_temp[MAX_ARGV];

	    //printf("str 	%s\r\n",argv[1]);

	    //printf("handle_device_config\r\n");
	    argc = parse2_args(argv[1],argv_temp);    //
	    printf("(3) argc:%d			argv[0]:%s		argv[1]:%s\r\n",argc,argv_temp[0],argv_temp[1]);
	    if (argc > 2)
	    {
	                printf("Too many parameters.\n");
	                return -1;
	    }else if(argc < 2)
	    {
	                printf("Too few parameters.\n");
	                return -1;
	    }
	    for (i = 0; i < sizeof(last_cmd_str)/sizeof(struct config_cmd); i++)
	          {
	              if (strcmp(argv_temp[0], last_cmd_str[i].name) == 0)
	              {
	                  ret = last_cmd_str[i].function(argc,argv_temp,op);
	                  if (ret < 0)
	                  {
	                      printf("AT CMD parameters error.\n");
	                      return ret;
	                  }
	                  break;
	              }
	          }
	          if (i == sizeof(last_cmd_str)/sizeof(struct config_cmd))
	          {
	              printf("AT CMD not found.\n");
	              return -1;
	          }
	   return 0;


}



static int dev_eui(int argc , char * argv[],cfg_op op)
{
	//printf("dev_eui(int argc , char * argv[],cfg_op op)\r\n");
	return 0;
}





void InitLora()
{
	  	LoRaMacPrimitives_t LoRaMacPrimitives;
	  	LoRaMacCallback_t LoRaMacCallbacks;
	  	MibRequestConfirm_t mibReq;


	  	LoRaMacPrimitives.MacMcpsConfirm = McpsConfirm;
	  	LoRaMacPrimitives.MacMcpsIndication = McpsIndication;
	  	LoRaMacPrimitives.MacMlmeConfirm = MlmeConfirm;
	  	LoRaMacPrimitives.MacMlmeIndication = MlmeIndication;
	  //LoRaMacCallbacks.GetBatteryLevel = BoardGetBatteryLevel;
	  	LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks, LORAMAC_REGION_EU868 );

		mibReq.Type = MIB_ADR;
		mibReq.Param.AdrEnable = true;
		LoRaMacMibSetRequestConfirm( &mibReq );

		mibReq.Type = MIB_PUBLIC_NETWORK;
		mibReq.Param.EnablePublicNetwork = true;
		LoRaMacMibSetRequestConfirm( &mibReq );
}

/*!
 * \brief   MLME-Indication event function
 *
 * \param   [IN] mlmeIndication - Pointer to the indication structure.
 */
static void MlmeIndication( MlmeIndication_t *mlmeIndication )
{

}

static void MlmeConfirm( MlmeConfirm_t *mlmeConfirm )
{
	switch( mlmeConfirm->MlmeRequest )
	    {
	        case MLME_JOIN:
	        {
	            if( mlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK )
	            {
	                // Status is OK, node has joined the network
	            	printf("Status is OK, node has joined the network\r\n");

	            }
	            else
	            {
	            	printf("Join was not successful. Try to join again\r\n");
	                // Join was not successful. Try to join again
//	                MlmeReq_t mlmeReq;
//	                mlmeReq.Type = MLME_JOIN;
//	                mlmeReq.Req.Join.DevEui = DevEui;
//	                mlmeReq.Req.Join.AppEui = AppEui;
//	                mlmeReq.Req.Join.AppKey = AppKey;
//	                mlmeReq.Req.Join.Datarate = LORAWAN_DEFAULT_DATARATE;
//
//	                if( LoRaMacMlmeRequest( &mlmeReq ) == LORAMAC_STATUS_OK )
//	                {
//	                    DeviceState = DEVICE_STATE_SLEEP;
//	                    printf("%s	%s	%d\r\n",__FILE__,__func__,__LINE__);
//	                }
//	                else
//	                {
//	                    DeviceState = DEVICE_STATE_CYCLE;
//	                    printf("%s	%s	%d\r\n",__FILE__,__func__,__LINE__);
//	                }
//	            }
//	            break;
	        }
	        }
	    }
}

static void McpsIndication( McpsIndication_t *mcpsIndication )
{

}

static void McpsConfirm( McpsConfirm_t *mcpsConfirm )
{

}




void lora_join(int argc, char *argv[])
{
 unsigned char i=0;
 unsigned char temp_dev_eui[]={ 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xA1 };
 unsigned char temp_app_eui[]={0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x01, 0xA2, 0xA1 };
 unsigned char temp_app_key[]={0x5A, 0x0C, 0xA2, 0xF8, 0xC9, 0xB8, 0xE4, 0x55, 0x33, 0x43, 0x5F, 0xC2, 0xA1, 0x3B, 0xA0, 0x73};

 for(i=0;i<8;i++)
 {
	 lora_config.dev_eui[i]=temp_dev_eui[i];
	 lora_config.app_eui[i]=temp_app_eui[i];
 }

 for(i=0;i<16;i++)
  {
 	 lora_config.app_key[i]=temp_app_key[i];

  }


 MlmeReq_t mlmeReq;	                // Initialize LoRaMac device unique ID
 //BoardGetUniqueId( DevEui );
 mlmeReq.Type = MLME_JOIN;

 mlmeReq.Req.Join.DevEui = lora_config.dev_eui;
 mlmeReq.Req.Join.AppEui = lora_config.app_eui;
 mlmeReq.Req.Join.AppKey = lora_config.app_key;
 mlmeReq.Req.Join.Datarate = DR_1;
 LoRaMacStatus_t state;
 state = LoRaMacMlmeRequest( &mlmeReq );
if( state!= LORAMAC_STATUS_OK )
{
	printf("LoRaMacMlmeRequest( &mlmeReq ) != LORAMAC_STATUS_OK  %d\r\n",state);
}
}





