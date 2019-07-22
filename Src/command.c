/*
 * command.c
 *
 *  Created on: 2019年7月22日
 *      Author: RAK
 */

#include "command.h"
#include "string.h"
#include "stdio.h"

static void Test_Printf(int argc, char *argv[]);

struct cli_cmd {
    /** The name of the CLI command */
    const char *name;
    /** The help text associated with the command */
    //const char *help;
    /** The function that should be invoked for this command. */
    void (*function) (int argc, char *argv[]);
};

struct cli_cmd cli_cmds[] = {
                            	"test",	Test_Printf,
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
	    if (argc > 0)
	    {
	        for (i = 0; i < sizeof(cli_cmds)/sizeof(struct cli_cmd); i++) {
	            if (strcmp(argv[0], cli_cmds[i].name) == 0) {
	                cli_cmds[i].function(argc, argv);
	                break;
	            }

	        }
	        if (i == sizeof(cli_cmds)/sizeof(struct cli_cmd)) {
	        	 printf("AT ERROR");
	        }
	    }
	    else
	    {
	       printf("AT Command ERROR\r\n");
	    }
	    //return 0;

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

	            argv[i-1] = ch;


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
	            if (*ch != '\0')
	            {
	                *ch = '\0';
	                ch++;
	                while(*ch == ',') {
	                    ch++;
	                }
	            }
	        }
	        return i;
}


static void Test_Printf(int argc, char *argv[])
{
	printf("AT Test OK\r\n");
}

