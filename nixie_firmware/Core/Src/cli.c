#include "cli.h"
#include <string.h>
#include <ctype.h>
#include <stdlib.h>
#include <ctype.h>

// head of CLI command list
static CLI_CommandItem *head = NULL;

// char buffer where command will be stored
static char commandBuffer[256];

/**
 * This function searches the CLI command list and tries to find a descriptor for the provided command.
 * The command format is case-insensitive.
 * Returns pointer to @ref CLI_MenuItem structure if given command was found in the CLI command list.
 *
 * @param command pointer to command (string)
 *
 * @retval pointer to @ref CLI_MenuItem structure desrcibing the command, if command was found
 * @retval NULL if the given command does not match any command regitstered in the CLI command list 
 */
static CLI_CommandItem* CLI_GetMenuItemByCommandName(char *command);

/**
 * @bref This function is responsible for collecting the command reading in from USART.
 *
 * This function should check wether the USART interface has some new data. If it does
 * this function reads new characters and stores them in a command buffer. This function
 * is also responsible for providing echo of the input characters back to the buffer.
 *
 * The function exits when:
 * - no more characters are available to read from USART - the function returns false
 * - new line was detected - this function returns true
 *
 * @retval true if command was collected
 * @retval false if command is yet incomplete
 */
static bool CLI_StoreCommand(void);

void CLI_StringToUpper(char *dst, const char *src){

	while((uint8_t)*src){
		if((*src)==' ') break;
		(*dst) = toupper((uint8_t)*src);
		dst++;
		src++;
	}
}
	
int32_t getNumber(char* str, int32_t* idx){
	int32_t number = atoi(str+(*idx));

	while(*(str+(*idx)) != ';' && *(str+(*idx)) != '\0') (*idx)++;

	if(*(str+(*idx)) == ';') (*idx)++;

	return number;
}
	
void CLI_Proc(void){
	if(CLI_StoreCommand()){
		CLI_CommandItem* command = CLI_GetMenuItemByCommandName(commandBuffer);
		// UART_WriteString(&huart2, "\r\n");
		if(command != NULL){
				command->callback(&commandBuffer[strlen(command->commandName)+1]);
		}
		// UART_WriteString(&huart2, "> ");
		memset(commandBuffer, 0, (sizeof(commandBuffer)/sizeof(commandBuffer[0])));
	}
}

bool CLI_AddCommand(CLI_CommandItem *item){
	CLI_CommandItem *ptr = head;

	if (item == NULL)
		return false;
	if (item->callback == NULL || item->commandName == NULL)
		return false;
	if (strlen(item->commandName) == 0)
		return false;
	
	if (head == NULL) {
		head = item;
		return true;
	}
	
	while (ptr->next != NULL)
		ptr = ptr->next;
	ptr->next = item;
	item->next = NULL;
	
	return true;
}

void CLI_PrintAllCommands(void){
	CLI_CommandItem *ptr = head;
	
	while (ptr != NULL) {
		UART_WriteString(&huart2, "  ");
		UART_WriteString(&huart2, ptr->commandName);
		if (ptr->description){
			UART_WriteString(&huart2, " ");
			UART_WriteString(&huart2, ptr->description);
		}
		UART_WriteString(&huart2, "\r\n");
		ptr = ptr->next;
	}
}

CLI_CommandItem* CLI_GetMenuItemByCommandName(char *command){
	volatile CLI_CommandItem *actual = head;
	CLI_StringToUpper(command, command);
	while(actual != NULL){
		if(memcmp(actual->commandName, command, strlen(actual->commandName)) == 0){
			if(command[strlen(actual->commandName)]==' '
				||command[strlen(actual->commandName)]=='\0'){
				return (CLI_CommandItem*)actual;
			}
		}
		actual = actual->next;
	}
	return NULL;
};

bool CLI_StoreCommand(void){
	static uint32_t idx = 0;
	char c;
	if(idx >= (sizeof(commandBuffer)/sizeof(commandBuffer[0])) - 1){
		idx = 0;
		memset(commandBuffer, 0, (sizeof(commandBuffer)/sizeof(commandBuffer[0])));
		// UART_WriteString(&huart2, "Command buffer overflow\r\n");
	}
	if(UART_GetChar(&huart2, &c)){
		if(c=='\n'||c=='\r'){
			idx = 0;
			return true;
		}else if(c==127 || c=='\b'){
			if(idx > 0){
				idx--;
				commandBuffer[idx] = 0;
			}
			// UART_PutChar(&huart2, c);
		}else{
			commandBuffer[idx] = c;
			idx++;
			// UART_PutChar(&huart2, c);
		}
	}		
	return false;
}
