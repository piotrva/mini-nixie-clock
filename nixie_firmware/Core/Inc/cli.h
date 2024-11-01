#ifndef CLI_H_
#define CLI_H_

#include <stdbool.h>
#include <stdint.h>

#include "main.h"
#include "serial.h"

extern UART_HandleTypeDef huart2;

/// type definition for the user callback function
typedef void(*CLICallbackFunc)(char *args);


/// CLI command descriptor
typedef struct CLI_CommandItem {
	/// pointer to a callback function
	CLICallbackFunc callback;
	/// pointer to comand name (string)
	char *commandName;
	/// pointer to command description (string)
	char *description;
	/// pointer to next command descriptor on the command list (uni-directional list)
	struct CLI_CommandItem* next;
} CLI_CommandItem;

/**
 * @brief This function converts string to a uppercase
 *
 * @param dst pointer where converted null terminated string will be stored
 * @param src pointer to string which will be converted
 */
void CLI_StringToUpper(char *dst, const char *src);

/** 
 * @brief CLI processing function. This function should be called repeatedly
 */
void CLI_Proc(void);

/** 
 * @brief This function registers new command in the CLI command list
 *
 * @param item pointer to a @ref CLI_CommandItem structure describing the command
 *
 * @retval true if new command was succesfully added
 * @retval false if an error occurred
 */
bool CLI_AddCommand(CLI_CommandItem *item);

/**
 * @bref This function prints information about all commands registered in the CLI command list.
 */
void CLI_PrintAllCommands(void);

int32_t getNumber(char* str, int32_t* idx);

#endif // CLI_H_
