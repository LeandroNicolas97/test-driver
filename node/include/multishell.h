/***************************************************************************
 *   copyright            : (C) by Innovex Tecnologias Ltda.               *
 *   email                : development@innovex.cl                         *
 *                                                                         *
 *   This program is property of Innovex Tecnologias Ltda. Chile.          *
 *   Copyright (C) 2011-2014. Innovex.                                     *
 ***************************************************************************/

#ifndef MULTISHELL_H_
#define MULTISHELL_H_

/**
 * Shell buffer configuration
 */
#define SHELL_BUFFER_SIZE 64

/**
 * Error codes
 */
#define SHELL_CMD_NOT_FOUND -1

/*
 * Used to relate a command text with a function.
 */
struct shell_command {
    char *text;
    int (*function)(char *args);
};

/**
 * Initialize the shell with the list of commands and a name
 */
void shell_init(const struct shell_command *command_list, char *name);

/**
 * Call this function every time a char for the shell is received
 */
int shell_char_received(char c);

/**
 * Flush the shell internal buffer.
 */
void shell_flush(void);

/**
 * Show the shell prompt
 */
void shell_prompt(void);

/**
 * Set the shell to be alone on the bus. Do not use the identifier when receiving commands.
 */
void shell_set_alone(void);

#endif /* MULTISHELL_H_ */
