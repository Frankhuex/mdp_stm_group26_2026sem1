void HostTestFail(const char *file, unsigned int line);
#define assert(condition) ((condition) ? (void)0 : HostTestFail(__FILE__, __LINE__))
