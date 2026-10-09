#ifndef HOST_ERRNO_H
#define HOST_ERRNO_H
int *_errno(void);
#define errno (*_errno())
#define ERANGE 34
#endif
