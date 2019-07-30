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
#include "command.h"



#define MAX_ARGV 20
lora_config_t lora_config;

LoRaMacPrimitives_t LoRaMacPrimitives;
LoRaMacCallback_t LoRaMacCallbacks;
MibRequestConfirm_t mibReq;
MlmeReq_t mlmeReq;
McpsReq_t mcpsReq;


//static int handle_lora_config(lora_config_t *config, int argc, char *argv[], char *in);
static int analysis_second_string(int argc , char * argv[],cfg_op op);
//static int handle_device_config(int argc , char * argv[],cfg_op op);
static int handle_lora_config(int argc , char * argv[],cfg_op op);
static int parse2_args(char* str, char* argv[]);


static void McpsConfirm( McpsConfirm_t *mcpsConfirm );
static void McpsIndication( McpsIndication_t *mcpsIndication );
static void MlmeConfirm( MlmeConfirm_t *mlmeConfirm );
static void MlmeIndication( MlmeIndication_t *mlmeIndication );


static int dev_eui(int argc , char * argv[],cfg_op op);
static int app_eui(int argc , char * argv[],cfg_op op);
static int app_key(int argc , char * argv[],cfg_op op);
static int dev_addr(int argc , char * argv[],cfg_op op);
static int apps_key(int argc , char * argv[],cfg_op op);
static int nwks_key(int argc , char * argv[],cfg_op op);

static int join_mode(int argc , char * argv[],cfg_op op);

struct config_cmd
{
    char *name;
    int (*function) (int argc, char *argv[],cfg_op op);//int (*function) (lora_config_t *config, int argc, char *argv[], char *in, cfg_op op);
};

struct config_cmd last_cmd_str[]=
{


		{"dev_addr",dev_addr},
		{"apps_key",apps_key},
		{"nwks_key",nwks_key},
		{"dev_eui",dev_eui},
		{"join_mode",join_mode},
		{"app_eui",app_eui},
		{"app_key",app_key}




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
		//{"device",                handle_device_config},
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
        //printf("(2) argc:%d			argv[0]:%s		argv[1]:%s\r\n",argc,argv_temp[0],argv_temp[1]);
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
	        int i;
		    int ret;

		    char *argv_temp[MAX_ARGV];

		    //printf("str 	%s\r\n",argv[1]);

		    //printf("handle_device_config\r\n");
		    argc = parse2_args(argv[1],argv_temp);    //
		    //printf("(3) argc:%d			argv[0]:%s		argv[1]:%s\r\n",argc,argv_temp[0],argv_temp[1]);
		    if (argc > 2||(argc > 1&&op==CFG_READ ))
		    {
		                printf("Too many parameters.\n");
		                return -1;
		    }else if(argc < 2&&op==CFG_WRITE)
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


//static int handle_device_config(int argc , char * argv[],cfg_op op)
//{
////	    int i;
////	    int ret;
////
////	    char *argv_temp[MAX_ARGV];
////
////	    //printf("str 	%s\r\n",argv[1]);
////
////	    //printf("handle_device_config\r\n");
////	    argc = parse2_args(argv[1],argv_temp);    //
////	    //printf("(3) argc:%d			argv[0]:%s		argv[1]:%s\r\n",argc,argv_temp[0],argv_temp[1]);
////	    if (argc > 2)  //写参数的时候也应该加以限制
////	    {
////	                printf("Too many parameters.\n");
////	                return -1;
////	    }else if(argc < 2&&op==CFG_WRITE)
////	    {
////	                printf("Too few parameters.\n");
////	                return -1;
////	    }
////	    for (i = 0; i < sizeof(last_cmd_str)/sizeof(struct config_cmd); i++)
////	          {
////	              if (strcmp(argv_temp[0], last_cmd_str[i].name) == 0)
////	              {
////	                  ret = last_cmd_str[i].function(argc,argv_temp,op);
////	                  if (ret < 0)
////	                  {
////	                      printf("AT CMD parameters error.\n");
////	                      return ret;
////	                  }
////	                  break;
////	              }
////	          }
////	          if (i == sizeof(last_cmd_str)/sizeof(struct config_cmd))
////	          {
////	              printf("AT CMD not found.\n");
////	              return -1;
////	          }
////	   return 0;
//
//
//}



static int dev_eui(int argc , char * argv[],cfg_op op)
{

	if(op==CFG_READ)
	{
		printf("dev_eui:%02X%02X%02X%02X%02X%02X%02X%02X\r\n",lora_config.dev_eui[0],lora_config.dev_eui[1],lora_config.dev_eui[2],lora_config.dev_eui[3],lora_config.dev_eui[4],
				lora_config.dev_eui[5],lora_config.dev_eui[6],lora_config.dev_eui[7]);
	}
	else
	{
		AsciiToHex((unsigned char *)argv[1], (unsigned char *)lora_config.dev_eui, strlen(argv[1]));
		Flash_write(FLASH_USER_START_ADDR,&lora_config,sizeof(lora_config_t));
		printf("ok\r\n");
//		printf("app_eui:%02X %02X %02X %02X %02X %02X %02X %02X\r\n",lora_config.app_eui[0],lora_config.app_eui[1],lora_config.app_eui[2],lora_config.app_eui[3],lora_config.app_eui[4],
//		lora_config.app_eui[5],lora_config.app_eui[6],lora_config.app_eui[7]);


	}

	//printf("dev_eui(int argc , char * argv[],cfg_op op)\r\n");
	return 0;
}

static int app_eui(int argc , char * argv[],cfg_op op)
{

	if(op==CFG_READ)
	{
		printf("app_eui:%02X%02X%02X%02X%02X%02X%02X%02X\r\n",lora_config.app_eui[0],lora_config.app_eui[1],lora_config.app_eui[2],lora_config.app_eui[3],lora_config.app_eui[4],
				lora_config.app_eui[5],lora_config.app_eui[6],lora_config.app_eui[7]);
	}
	else
	{
		AsciiToHex((unsigned char *)argv[1], (unsigned char *)lora_config.app_eui, strlen(argv[1]));
		Flash_write(FLASH_USER_START_ADDR,&lora_config,sizeof(lora_config_t));
		printf("ok\r\n");
//		printf("app_eui:%02X %02X %02X %02X %02X %02X %02X %02X\r\n",lora_config.app_eui[0],lora_config.app_eui[1],lora_config.app_eui[2],lora_config.app_eui[3],lora_config.app_eui[4],
//		lora_config.app_eui[5],lora_config.app_eui[6],lora_config.app_eui[7]);


	}

	//printf("dev_eui(int argc , char * argv[],cfg_op op)\r\n");
	return 0;
}






void InitLora()
{



	  	LoRaMacPrimitives.MacMcpsConfirm = McpsConfirm;
	  	LoRaMacPrimitives.MacMcpsIndication = McpsIndication;
	  	LoRaMacPrimitives.MacMlmeConfirm = MlmeConfirm;
	  	LoRaMacPrimitives.MacMlmeIndication = MlmeIndication;
	  //LoRaMacCallbacks.GetBatteryLevel = BoardGetBatteryLevel;
	  	LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks, LORAMAC_REGION_EU868 );

	  	 mibReq.Type = MIB_ADR;     //ADR 没有问题
	  	 mibReq.Param.AdrEnable = 1;
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
	printf("%s	%s	%d\r\n",__FILE__,__func__,__LINE__);
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
//	            	LED_Init();
//	            	LED4_State(0);
//	            	HAL_Delay(500);
//	            	LED4_State(1);
//	            	HAL_Delay(200);
//	            	LED4_State(0);
//	            	HAL_Delay(500);
//	            	LED4_State(1);

	            }
	            else
	            {
	            	printf("Join was not successful. Try to join again\r\n");

	            }
	            break;

	        }
	    }
}

static void McpsIndication( McpsIndication_t *mcpsIndication )
{
	//printf("%s	%s	%d\r\n",__FILE__,__func__,__LINE__);

}

static void McpsConfirm( McpsConfirm_t *mcpsConfirm )
{

	//printf("%s	%s	%d	%d\r\n",__FILE__,__func__,__LINE__,mcpsConfirm->McpsRequest);
	if( mcpsConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK )
	    {

	        switch( mcpsConfirm->McpsRequest )
	        {
	            case MCPS_UNCONFIRMED:
	            {
	                // Check Datarate
	                // Check TxPower
	                break;
	            }
	            case MCPS_CONFIRMED:
	            {

	            	Serial_PutString("MCPS confirmed succseeful\r\n");
	            	//while(1);
	                // Check Datarate
	                // Check TxPower
	                // Check AckReceived
	                // Check NbTrials
	                break;
	            }
	            case MCPS_PROPRIETARY:
	            {
	                break;
	            }
	            default:
	                break;
	        }
	    }
	else
	{
		Serial_PutString("Rx timeout\r\n");
		LED_RED();
	}


}




void lora_join(int argc, char *argv[])
{
 unsigned char i=0;
// unsigned char temp_dev_eui[]={ 0x47, 0xAA, 0xC8, 0x68, 0x00, 0x43, 0x00, 0x28 };
// unsigned char temp_app_eui[]={ 0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x01, 0xA1, 0xE2 };
// unsigned char temp_app_key[]={ 0x8A, 0xFE, 0x71, 0xA1, 0x45, 0xB2, 0x53, 0xE4, 0x9C, 0x30, 0x31, 0xAD, 0x06, 0x82, 0x77, 0xA1 };

// for(i=0;i<8;i++)
// {
//	 lora_config.dev_eui[i]=temp_dev_eui[i];
//	 lora_config.app_eui[i]=temp_app_eui[i];
// }
//
// for(i=0;i<16;i++)
//  {
// 	 lora_config.app_key[i]=temp_app_key[i];
//
//  }

 LoRaMacStatus_t state;
 //MlmeReq_t mlmeReq;	                // Initialize LoRaMac device unique ID

 if(lora_config.join_state==OATT)
 {
 //BoardGetUniqueId( DevEui );
 mlmeReq.Type = MLME_JOIN;
// for(i=0;i<8;i++)
//              {
//	 mlmeReq.Req.Join.DevEui[i]=lora_config.dev_eui[i];
//
//              }
// memcpy1( mlmeReq.Req.Join.DevEui,lora_config.dev_eui,8);


 mlmeReq.Req.Join.DevEui = lora_config.dev_eui;
// for(i=0;i<8;i++)
//              {
//	 mlmeReq.Req.Join.AppEui[i]=lora_config.app_eui[i];
//
//              }
// memcpy1(mlmeReq.Req.Join.AppEui,lora_config.app_eui,8);

 mlmeReq.Req.Join.AppEui = lora_config.app_eui;
// for(i=0;i<16;i++)
//              {
//	 mlmeReq.Req.Join.AppKey[i]=lora_config.app_key[i];
//
//              }
// memcpy1( mlmeReq.Req.Join.AppKey,lora_config.app_key,16);

 mlmeReq.Req.Join.AppKey = lora_config.app_key;
 mlmeReq.Req.Join.Datarate = DR_1;

 state = LoRaMacMlmeRequest( &mlmeReq );
if( state!= LORAMAC_STATUS_OK )
{
	printf("+MEMSREQ:%d\r\nERROR\r\n",state);
}
else
{
   printf("Joining ... ...\r\n");
}
 }
 else
 {


//    	 for(i=0;i<3;i++)
//    	 {
//    		mibReq.Param.DevAddr[i]=lora_config.dev_addr[i];
//
//     	 }
//	 	 	 mibReq.Type = MIB_NET_ID;
//	         mibReq.Param.NetID = 0;
//	         LoRaMacMibSetRequestConfirm( &mibReq );


 	 		 mibReq.Param.DevAddr=(uint32_t)lora_config.dev_addr[3]+  ((uint32_t)lora_config.dev_addr[2]<<8)+
 	 				((uint32_t)lora_config.dev_addr[1]<<16) + ((uint32_t)lora_config.dev_addr[0]<<24);


             mibReq.Type = MIB_DEV_ADDR;

//            mibReq.Param.DevAddr = lora_config.dev_addr;
             LoRaMacMibSetRequestConfirm( &mibReq );
//


//             for(i=0;i<16;i++)
//             {
//            	 mibReq.Param.NwkSKey[i]=lora_config.nwks_key[i];
//
//             }

             mibReq.Type = MIB_NWK_SKEY;
             mibReq.Param.NwkSKey = lora_config.nwks_key;
             LoRaMacMibSetRequestConfirm( &mibReq );


//             for(i=0;i<16;i++)
//             {
//            	 mibReq.Param.AppSKey[i]=lora_config.apps_key[i];
//
//             }

             mibReq.Type = MIB_APP_SKEY;
             mibReq.Param.AppSKey = lora_config.apps_key;
             LoRaMacMibSetRequestConfirm( &mibReq );



             mibReq.Type = MIB_NETWORK_JOINED;
             mibReq.Param.IsNetworkJoined = true;
             state=LoRaMacMibSetRequestConfirm( &mibReq );



             if( state!= LORAMAC_STATUS_OK )
             {
             	printf("+MIBREQ:%d\r\nERROR\r\n",state);
             }

             printf("ok\r\n");
 }
}



void lora_send(int port,const unsigned char* Appdata)
{
	mcpsReq.Type = MCPS_CONFIRMED;
	mcpsReq.Req.Unconfirmed.fPort = port;
	mcpsReq.Req.Unconfirmed.fBuffer = Appdata;
	mcpsReq.Req.Unconfirmed.fBufferSize = 1;
	mcpsReq.Req.Unconfirmed.Datarate = DR_1;
	LoRaMacStatus_t state;
	state =LoRaMacMcpsRequest( &mcpsReq ) ;
	if( state!= LORAMAC_STATUS_OK )
	{
		printf("+MCPSREQ:%d\r\nERROR\r\n",state);
		LED_RED();
	}

}


static int join_mode(int argc , char * argv[],cfg_op op)
{
	if(op==CFG_READ)
	{
		if(lora_config.join_state==OATT)
			printf("join_mode: OATT\r\n");
		else
			printf("join_mode: ABP\r\n");
	}
	else
	{
		if(argv[1][0]==0x30)
		{
			lora_config.join_state=OATT;
			Flash_write(FLASH_USER_START_ADDR,&lora_config,sizeof(lora_config_t));
			printf("ok\r\n");
		}
		else
		{
			lora_config.join_state=ABP;
			Flash_write(FLASH_USER_START_ADDR,&lora_config,sizeof(lora_config_t));
			printf("ok\r\n");
		}


	}

	return 0;

}



static int app_key(int argc , char * argv[],cfg_op op)
{
	char i;
	if(op==CFG_READ)
		{
		printf("app_key:");
			for(i=0;i<16;i++)
			{
				printf("%02X",lora_config.app_key[i]);

			}
			printf("\r\n");

		}
		else
		{
			AsciiToHex((unsigned char *)argv[1], (unsigned char *)lora_config.app_key, strlen(argv[1]));
			Flash_write(FLASH_USER_START_ADDR,&lora_config,sizeof(lora_config_t));

			printf("ok\r\n");
		}

		return 0;
}

static int apps_key(int argc , char * argv[],cfg_op op)
{
	char i;
	if(op==CFG_READ)
		{
			printf("apps_key:");
			for(i=0;i<16;i++)
			{
				printf("%02X",lora_config.apps_key[i]);

			}
			printf("\r\n");

		}
		else
		{
			AsciiToHex((unsigned char *)argv[1], (unsigned char *)lora_config.apps_key, strlen(argv[1]));

			Flash_write(FLASH_USER_START_ADDR,&lora_config,sizeof(lora_config_t));
			printf("ok\r\n");
		}

		return 0;
}

static int nwks_key(int argc , char * argv[],cfg_op op)
{
	char i;
	if(op==CFG_READ)
		{
			printf("nwks_key:");
			for(i=0;i<16;i++)
			{
				printf("%02X",lora_config.nwks_key[i]);

			}
			printf("\r\n");
		}
		else
		{
			AsciiToHex((unsigned char *)argv[1], (unsigned char *)lora_config.nwks_key, strlen(argv[1]));

			Flash_write(FLASH_USER_START_ADDR,&lora_config,sizeof(lora_config_t));
			printf("ok\r\n");
		}

		return 0;
}

static int dev_addr(int argc , char * argv[],cfg_op op)
{
	char i;
	if(op==CFG_READ)
		{
			printf("dev_addr:");
			for(i=0;i<4;i++)
			{
				printf("%02X",lora_config.dev_addr[i]);

			}
			printf("\r\n");

		}
		else
		{
			AsciiToHex((unsigned char *)argv[1], (unsigned char *)lora_config.dev_addr, strlen(argv[1]));

			Flash_write(FLASH_USER_START_ADDR,&lora_config,sizeof(lora_config_t));
			printf("ok\r\n");
		}

		return 0;
}










