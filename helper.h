// Useful helper functions

#ifndef _HELPER_H
#define _HELPER_H

void strip_newline(char *str); // Modifies str to only refer to its first line, without trailing newlines
void strip_extension(char *str); // Strips the file extension from str
int get_port(char *port_name); // Computes the port number from COM (e.g. COM8 -> 7)

#endif
