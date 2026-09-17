/* Persistent RFC1950 inflater, owned by an ingress stream, never by an image.
 * Firmware zlib 1.1.4 uses 32-bit uLong/pointers. Tests supply the host ABI. */
#ifndef CFW_ZSTREAM
typedef struct {
    const uint8_t *next_in;
    uint32_t avail_in, total_in;
    uint8_t *next_out;
    uint32_t avail_out, total_out;
    const char *msg;
    void *state;
    void *(*zalloc)(void *, uint32_t, uint32_t);
    void (*zfree)(void *, void *);
    void *opaque;
    int32_t data_type;
    uint32_t adler, reserved;
} cfw_zstream;
_Static_assert(sizeof(cfw_zstream) == 0x38, "firmware zlib ABI");
#define CFW_ZSTREAM cfw_zstream
#define CFW_ZINIT(s) ((int (*)(void *, int, const char *, int))0x005d6167u)(s, 15, (const char *)0x007b75f8u, 0x38)
#define CFW_ZINFLATE(s) ((int (*)(void *, int))0x005d6235u)(s, 0)
#define CFW_ZEND(s) ((int (*)(void *))0x005d612bu)(s)
#endif

static void *cfw_zalloc(void *opaque, uint32_t items, uint32_t size) {
    (void)opaque;
    if (size && items > 0xffffffffu / size) return 0;
    return CFW_MESSAGE_MALLOC(items * size);
}
static void cfw_zfree(void *opaque, void *ptr) {
    (void)opaque;
    CFW_MESSAGE_FREE(ptr);
}
static void cfw_inflate_reset(cfw_message_stream *stream) {
    if (stream->inflater) {
        CFW_ZEND((CFW_ZSTREAM *)stream->inflater);
        CFW_MESSAGE_FREE(stream->inflater);
        stream->inflater = 0;
    }
}
static int cfw_inflate_message(cfw_message_stream *stream, uint8_t *output, uint32_t *size) {
    if (!stream->inflater) {
        CFW_ZSTREAM *z = CFW_MESSAGE_MALLOC(sizeof(*z));
        if (!z) return 0;
        for (uint32_t i = 0; i < sizeof(*z); ++i) ((uint8_t *)z)[i] = 0;
        z->zalloc = cfw_zalloc;
        z->zfree = cfw_zfree;
        if (CFW_ZINIT(z) != 0) { CFW_MESSAGE_FREE(z); return 0; }
        stream->inflater = z;
    }
    CFW_ZSTREAM *z = stream->inflater;
    uint32_t n = stream->size;
    const uint8_t *input = stream->buffer;
    /* Every record must end in a complete SYNC_FLUSH, never Z_FINISH. */
    if (n < 4 || input[n-4] || input[n-3] || input[n-2] != 255 || input[n-1] != 255) return 0;
    z->next_in = (void *)input;
    z->avail_in = n;
    z->next_out = output;
    z->avail_out = CFW_MESSAGE_MAX + 1u; /* one extra byte detects expansion overflow */
    while (z->avail_in) {
        uint32_t before = z->avail_in, space = z->avail_out;
        int result = CFW_ZINFLATE(z);
        if (result != 0 || !z->avail_out ||
                (before == z->avail_in && space == z->avail_out)) return 0;
    }
    *size = CFW_MESSAGE_MAX + 1u - z->avail_out;
    /* Do not retain borrowed input/output pointers between messages. */
    z->next_in = 0;
    z->next_out = 0;
    return *size <= CFW_MESSAGE_MAX;
}
