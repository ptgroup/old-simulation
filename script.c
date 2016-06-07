#include "script.h"

#include <stdio.h>
#include <string.h>

FILE *script_file;

#define MAX_CMDS 10
#define CMD_BUFLEN 40

static char commands[MAX_CMDS][CMD_BUFLEN];

int script_fopen(char *filename) {
    script_file = fopen(filename, "r");
    return !script_file;
}

int script_readline() {
    int read = 0;
    int cmd = 0;
    int pos = 0;
    while (true) {
        char c = fgetc(script_file);
        if (c == EOF) {
            commands[cmd][pos] = 0;
            return read;
        } else if (c == '\n' || c == '\r') {
            if (cmd == 0 && pos == 0) {
                continue;
            } else {
                // Make sure to null-terminate the string
                commands[cmd][pos] = 0;
                return read;
            }
        } else if (c == '#') {
            // Comment
            if (cmd == 0 && pos == 0) {
                // Skip to next line
                do {
                    c = fgetc(script_file);
                } while (c != '\n');
                continue;
            } else {
                commands[cmd][pos] = 0;
                return read;
            }
        } else if (c == ' ' || c == '\t') {
            if (cmd == 0 && pos == 0) {
                continue;
            }
            
            commands[cmd++][pos] = 0;
            pos = 0;
        } else {
            // Next character in command
            if (pos < CMD_BUFLEN - 1) {
                commands[cmd][pos++] = c;
                read++;
            } else if (cmd < MAX_CMDS - 1) {
                commands[cmd++][pos] = 0;
                pos = 0;
            } else {
                return read;
            }
        }
    }
}

bool script_cmdequ(char *command) {
    return strcmp(commands[0], command) == 0;
}

char *script_getarg(int n) {
    return commands[n + 1];
}

void script_fclose() {
    fclose(script_file);
}
