/* Run pure C controller tests without an installed Windows SDK/UCRT.
 * Only these three documented Windows APIs are imported; firmware is unchanged. */
__declspec(dllimport) void __stdcall ExitProcess(unsigned int code);
__declspec(dllimport) void *__stdcall GetStdHandle(unsigned long id);
__declspec(dllimport) int __stdcall WriteFile(void *handle, const void *buffer,
    unsigned long length, unsigned long *written, void *overlapped);
int _fltused = 0;
int main(void);
static void write_text(const char *text)
{
    unsigned long length = 0, written;
    while (text[length]) ++length;
    WriteFile(GetStdHandle((unsigned long)-11), text, length, &written, (void *)0);
}
int puts(const char *text) { write_text(text); write_text("\r\n"); return 0; }
void HostTestFail(const char *file, unsigned int line)
{
    char number[12];
    unsigned int i = 11, value = line;
    number[i] = 0;
    do { number[--i] = (char)('0' + value % 10); value /= 10; } while (value);
    write_text("FAIL: "); write_text(file); write_text(":"); puts(number + i);
    ExitProcess(line ? line : 1);
}
void TestEntry(void) { ExitProcess((unsigned int)main()); }
