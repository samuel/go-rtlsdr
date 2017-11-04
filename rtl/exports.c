#include "exports.h"

void cbAsync(unsigned char *buf, uint32_t len, void *ctx) {
	cbAsyncGo(buf, len, ctx);
}

const rtlsdr_read_async_cb_t *cbAsyncPtr = (rtlsdr_read_async_cb_t*)&cbAsync;

// read_async exists to avoid having to pass a void* for context when we really just want an integer.
// Go's GC rules mean that there shouldn't be pointers in uintptr and vice versa in Go code. Find in C though.
int read_async(rtlsdr_dev_t *dev, rtlsdr_read_async_cb_t cb, intptr_t ctx, uint32_t buf_num, uint32_t buf_len) {
	return rtlsdr_read_async(dev, cb, (void*)ctx, buf_num, buf_len);
}
