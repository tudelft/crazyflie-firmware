#include <sys/stat.h>
#include <errno.h>

int _write(int file, char *ptr, int len) {
  (void)file;
  (void)ptr;
  (void)len;
  errno = ENOSYS;
  return -1;
}

int _read(int file, char *ptr, int len) {
  (void)file;
  (void)ptr;
  (void)len;
  errno = ENOSYS;
  return -1;
}

int _close(int file) {
  (void)file;
  errno = ENOSYS;
  return -1;
}

int _lseek(int file, int ptr, int dir) {
  (void)file;
  (void)ptr;
  (void)dir;
  errno = ENOSYS;
  return -1;
}

int _fstat(int file, struct stat *st) {
  (void)file;
  st->st_mode = S_IFCHR;
  return 0;
}

int _isatty(int file) {
  (void)file;
  return 1;
}

void *_sbrk(int incr) {
  (void)incr;
  errno = ENOMEM;
  return (void *)-1;
}