#ifndef ERROR_CODES_H
#define ERROR_CODES_H

/* 
return_codes.h

Integer return codes used for returning state of execution, either error or success

*/

// command return status
#define COMMAND_OK 0
#define COMMAND_FAILED 1
#define COMMAND_TIMEOUT 2
#define COMMAND_FORBIDDEN 3
#define COMMAND_NOT_FOUND 4
#define COMMAND_EXPANSION_BOARD_NOT_FOUND 5
#define COMMAND_MISSING_ARGUMENT 6
#define COMMAND_ARGUMENT_OUT_OF_BOUNDS 7

#endif