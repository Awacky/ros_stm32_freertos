#ifndef _TERMINAL_H_
#define _TERMINAL_H_

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>  

void terminal_process_string(char *str);
void terminal_register_command_callback(
		const char* command,
		const char *help,
		const char *arg_names,
		void(*cbf)(int argc, const char **argv));
void terminal_unregister_callback(void(*cbf)(int argc, const char **argv));
#endif 
		
		
		
		