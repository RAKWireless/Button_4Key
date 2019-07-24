/*
 * command.c
 *
 *  Created on: 2019年7月22日
 *      Author: RAK
 */

#include "command.h"
#include "string.h"
#include "stdio.h"
#include "stddef.h"
#include "lora_config.h"


static void commmon_read_config(int argc, char *argv[]);
static void commmon_set_config(int argc, char *argv[]);
static int parse_args(char* str, char* argv[]);


struct cli_cmds {
    /** The name of the CLI command */
    const char *name;
    /** The help text associated with the command */
    //const char *help;
    /** The function that should be invoked for this command. */
    void (*function) (int argc, char *argv[]);
};

struct cli_cmds First_cmds[] =
{
		{"join",				lora_join},
		{"get_config",		commmon_read_config},
		{"set_config",		commmon_set_config},

};

void CMD_Process( unsigned char* rxChar)
{
	int i;
	int argc;
	char* argv[MAX_ARGV]={0};

	    if ((strncmp((char*)rxChar, "at+", 3) != 0) || rxChar[3] == '\0')   //解析at头
	    {
	    	printf("AT Command ERROR\r\n");
	        return;
	    }
	    printf("[Echo cmd:] %s\r\n", rxChar);
	    rxChar += 3;
	    argc = parse_args((char*)rxChar, argv);
	    //printf("(1) argc:%d	argv[0]:%s	argv[1]:%s\r\n",argc,argv[0],argv[1]);


	    if (argc > 0)
	    {
	        for (i = 0; i < sizeof(First_cmds)/sizeof(struct cli_cmds); i++)
	        {
	            if (strcmp(argv[0], First_cmds[i].name) == 0)
	            {
	            	//printf("(1) argc:%d			argv[0]:%s	argv[1]:%s\r\n",argc,argv[0],argv[1]);   //argc传入一个无意义的值，只是为了函数格式统一
	            	First_cmds[i].function(argc, argv);

	                break;
	            }

	        }
	        if (i == sizeof(First_cmds)/sizeof(struct cli_cmds)) {
	        	 printf("AT Command ERROR\r\n");
	        }
	    }
	    else
	    {
	       printf("AT Command ERROR\r\n");
	    }

}


static int parse_args(char* str, char* argv[])
{
	        int i = 0;
	        char* ch = str;

	        while(*ch != '\0')
	        {
	            i++;
	            /*Check if length exceeds*/
	            if (i > MAX_ARGV)
	            {
	            	printf("Error:AT Out ofMAX_ARGV\r\n");
	                return 0;
	            }

	            argv[i-1] = ch;    //取参数


	            while(*ch != ',' && *ch != '\0' && *ch != '\r')
	            {
	                if (*ch == ':')
	                {
	                    return i;
	                }

	                if(*ch == '=' && i == 1)
	                {
	                    break;
	                }
	                else
	                    ch++;
	            }
	            if (*ch == '\r')
	                break;
	            if (*ch != '\0')   //为字符串加末尾0字符
	            {
	                *ch = '\0';
	                ch++;
//	                while(*ch == ',')
//	                {
//	                    ch++;
//	                }
	            }
	        }
	        return i;
}




static void commmon_read_config(int argc, char *argv[])
{
    int ret;
    if (argc != 2)
    {
        printf("Parameter format error.\r\n");
        return;
    }

    ret = read_config(argc,argv);    //后半截字符串传入
    if (ret < 0)
    {
        return;
    }
}

static void commmon_set_config(int argc, char *argv[])
{
    int ret;
    if (argc != 2)
    {
        printf("Parameter format error.\r\n");
        return;
    }

    ret = set_config(argc,argv);    //后半截字符串传入
    if (ret < 0)
    {
        return;
    }
}

