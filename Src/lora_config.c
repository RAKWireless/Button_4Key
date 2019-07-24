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








