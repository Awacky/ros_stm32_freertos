#include "terminal.h"
#include "hw_config.h"
#define CALLBACK_LEN						40
// Private types
typedef struct _terminal_callback_struct {
	const char *command;
	const char *help;
	const char *arg_names;
	void(*cbf)(int argc, const char **argv);
} terminal_callback_struct;
// Private variables
static terminal_callback_struct callbacks[CALLBACK_LEN];
static int callback_write = 0;
static void terminalHelp(void);
#ifdef Custom
void Terminal_send(uint8_t *buf_t,uint16_t leng_t) {

}
void Terminal_printf(char* fmt,...)  {  

}
#endif
void terminal_process_string(char *str) {
	enum { kMaxArgs = 64 };
	int argc = 0;
	char *argv[kMaxArgs];
	char *p2 = strtok(str, " ");
	while (p2 && argc < kMaxArgs) {
		argv[argc++] = p2;
		p2 = strtok(0, " ");
	}
	if (argc == 0) {
		Terminal_printf("No command received\n");
		return;
	}
	if (strcmp(argv[0], "ping") == 0) {
		Terminal_printf("pong\n");
	} else if (strcmp(argv[0], "help") == 0) {
		terminalHelp();
	}

	for (int i = 0;i < callback_write;i++) {
		if (callbacks[i].cbf != 0 &&\
			strcmp(argv[0], callbacks[i].command) == 0) {
			callbacks[i].cbf(argc, (const char**)argv);
			return;
		}
	}

}
/**
 * Register a custom command  callback to the terminal. If the command
 * is already registered the old command callback will be replaced.
 *
 * @param command
 * The command name.
 *
 * @param help
 * A help text for the command. Can be NULL.
 *
 * @param arg_names
 * The argument names for the command, e.g. [arg_a] [arg_b]
 * Can be NULL.
 *
 * @param cbf
 * The callback function for the command.
 */
void terminal_register_command_callback(
		const char* command,
		const char *help,
		const char *arg_names,
		void(*cbf)(int argc, const char **argv)) {
	int callback_num = callback_write;
	for (int i = 0;i < callback_write;i++) {
		// First check the address in case the same callback is registered more than once.
		if (callbacks[i].command == command) {
			callback_num = i;
			break;
		}
		// Check by string comparison.
		if (strcmp(callbacks[i].command, command) == 0) {
			callback_num = i;
			break;
		}
		// Check if the callback is empty (unregistered)
		if (callbacks[i].cbf == 0) {
			callback_num = i;
			break;
		}
	}
	callbacks[callback_num].command = command;
	callbacks[callback_num].help = help;
	callbacks[callback_num].arg_names = arg_names;
	callbacks[callback_num].cbf = cbf;
	if (callback_num == callback_write) {
		callback_write++;
		if (callback_write >= CALLBACK_LEN) {
			callback_write = 0;
		}
	}
}

void terminal_unregister_callback(void(*cbf)(int argc, const char **argv)) {
	for (int i = 0;i < callback_write;i++) {
		if (callbacks[i].cbf == cbf) {
			callbacks[i].cbf = 0;
		}
	}
}
static void terminalHelp(void){
	Terminal_printf("Valid commands are:");
	Terminal_printf("help");
	Terminal_printf("  Show this help");
	Terminal_printf("ping");
	Terminal_printf("  Print pong here to see if the reply works");
	for (int i = 0;i < callback_write;i++) {
		if (callbacks[i].cbf == 0) {
			continue;
		}
		if (callbacks[i].arg_names) {
			Terminal_printf("%s %s", callbacks[i].command, callbacks[i].arg_names);
		} else {
			Terminal_printf((char *)callbacks[i].command);
		}

		if (callbacks[i].help) {
			Terminal_printf("  %s", callbacks[i].help);
		} else {
			Terminal_printf("  There is no help available for this command.");
		}
	}
	Terminal_printf(" ");
}