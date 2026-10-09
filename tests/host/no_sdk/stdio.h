#ifndef HOST_STDIO_H
#define HOST_STDIO_H
#include <stddef.h>
int puts(const char *text);
#define snprintf _snprintf
int _snprintf(char *text, size_t size, const char *format, ...);
#endif
