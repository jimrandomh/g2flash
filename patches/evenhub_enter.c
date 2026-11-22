#include <stdint.h>

/* Image-handler mode 31: put the glasses into EvenHub showing Faceclaw's fixed
 * startup page, from whatever display state they were in, and ACK.
 *
 * The phone used to do this with a stock sid-0xe0 Cmd=0 CreateStartUpPage, which
 * the stock parser (evenhub_data_parser, 0x4f01a4) only answers when it actually
 * starts a page. On the master lens it branches three ways:
 *   - EvenHub not running: REQUEST_DISPLAY_START_UP(0xe0, pb); the EvenHub UI
 *     replies after creating the page.
 *   - running with a different widget id: stop it, then restart with pb after
 *     800 ms (AsyncRequestDisplayStartUp); replies after creating the page.
 *   - running with the same widget id: logs "do nothing" and never replies.
 * The last case is a reconnect to glasses that kept Faceclaw's page alive, and
 * the missing reply made the phone time out and reconnect again.
 *
 * This handler queues the same request the phone would have sent, through the
 * same SendUserDataToThreadPool call the BLE receive path uses (it copies the
 * buffer), so the stock parser still makes all three decisions in its normal
 * worker context. The private transport ACKs as soon as the request is queued;
 * the page itself may appear asynchronously in the first two cases.
 *
 * The stock reply, when there is one, carries magic 99. Faceclaw allocates
 * magics from 100..255 and ignores replies outside that range. */
#define EVENHUB_ENTER_APP_ID 0xe0u
#define EVENHUB_ENTER_SIDE ((uint32_t (*)(void))0x00465d4du) /* FUN_00465d4c: 1=right (master), 2=left */
/* FUN_00465f10 SendUserDataToThreadPool(app_id, data, len, event): copies data,
 * then a worker calls the app's common_data_handler(event, copy, len). Event 0
 * is data from the phone, which evenhub_common_data_handler passes to the parser. */
#define EVENHUB_ENTER_SUBMIT \
    ((int (*)(uint32_t, const uint8_t *, uint32_t, uint16_t))0x00465f11u)

/* EvenHub{Cmd=0, MagicRandom=99, CreateMessage{ContainerTotalNum=1,
 * TextObject{x=0, y=0, w=576, h=288, ContainerID=1, ContainerName="dashboard",
 * IsEventCapture=1, Content=" "}, widgetId=10000}}. Byte-identical to Faceclaw's
 * BleProtocol.buildCreateInputPage apart from the magic. */
static const uint8_t evenhub_enter_page[] = {
    0x08, 0x00, 0x10, 0x63, 0x1a, 0x23, 0x08, 0x01, 0x1a, 0x1c, 0x08, 0x00,
    0x10, 0x00, 0x18, 0xc0, 0x04, 0x20, 0xa0, 0x02, 0x48, 0x01, 0x52, 0x09,
    'd',  'a',  's',  'h',  'b',  'o',  'a',  'r',  'd',  0x58, 0x01, 0x62,
    0x01, ' ',  0x28, 0x90, 0x4e,
};

/* [31]. Only the master lens owns the EvenHub lifecycle (the stock parser ignores
 * the create on the left), so the left lens just ACKs. A failed queue submission
 * NACKs, and the phone replays the message, which is safe because it is idempotent. */
int evenhub_enter_control(const uint8_t *src, uint32_t len) {
    if (!src || len != 1 || src[0] != 31) return -1;
    if (EVENHUB_ENTER_SIDE() != 1) return 0;
    return EVENHUB_ENTER_SUBMIT(EVENHUB_ENTER_APP_ID, evenhub_enter_page,
                                (uint32_t)sizeof(evenhub_enter_page), 0) == 0 ? 0 : -1;
}

/* Image mode 34 (Faceclaw/44): start a Faceclaw session in one message, in
 * place of the phone's stock sid-0x01 prelude, mode 31 and separate ALS and
 * brightness messages, which each cost a round trip on every wake:
 *
 *   [34]{len:u8, message[len]}*
 *
 * Each embedded message is a complete image-mode message, mode 16 (ALS) or 30
 * (brightness) only, run here on every lens as it would run on its own. Then
 * the master lens does what the prelude and mode 31 did, in their order.
 *
 * The prelude is a stock Dashboard (app 0x01, UI registry row 3) data package.
 * The phone sends it, waits for the Dashboard's reply, and only then asks for
 * the EvenHub page, which the stock EvenHub parser ignores until the prelude
 * has run. Queued separately, the two could run out of order: the stock thread
 * pool has eight workers on one queue (openCFW worker_thread.c). So one pool
 * job runs both, in order, through the dispatcher every pool job uses
 * (0x44407c: the registry row's data handler(event, data, len); stock
 * SendUserDataToThreadPool queues 0x465eb4, which calls it, then frees the
 * copy). Each handler gets a copy on the worker's stack, as stock hands it a
 * heap copy, so it may write to its input. Event 0 is data from the phone.
 *
 * The Dashboard replies to the phone with sid 0x01, magic 98. Faceclaw
 * allocates magics from 100..255; mode 31's EvenHub reply carries 99. */
#define SESSION_START_MODE 34u
#define SESSION_START_DASHBOARD_APP_ID 0x01u
#define SESSION_START_EVENT_FROM_PHONE 0u
/* FUN_004a8d98 thread_pool submit(fn, a, b, c, event): queues {fn, a, event, b, c};
 * a worker calls fn(a, b, c, event). 0 when queued (3 s timeout). */
#define SESSION_START_POOL_SUBMIT \
    ((int (*)(void (*)(uint32_t, uint32_t, uint32_t, uint32_t), uint32_t, uint32_t, uint32_t, uint32_t))0x004a8d99u)
/* FUN_0044407c: find app_id's UI registry row and call its data handler(event, data, len). */
#define SESSION_START_DISPATCH \
    ((void (*)(uint32_t, uint8_t *, uint32_t, uint32_t))0x0044407du)

/* DashboardDataPackage{1: 2, magic: 98, 4: {3: {2: {2: {1: 0, 2: 0}}}}}: the
 * phone's BleProtocol PRELUDE_F5872_PAYLOAD apart from the magic. */
static const uint8_t session_start_prelude[] = {
    0x08, 0x02, 0x10, 0x62, 0x22, 0x0a, 0x1a, 0x08,
    0x12, 0x06, 0x12, 0x04, 0x08, 0x00, 0x10, 0x00,
};

static void session_start_job(uint32_t unused_a, uint32_t unused_b, uint32_t unused_c,
                              uint32_t unused_event) {
    (void)unused_a; (void)unused_b; (void)unused_c; (void)unused_event;
    uint8_t prelude[sizeof(session_start_prelude)];
    uint8_t page[sizeof(evenhub_enter_page)];
    memcpy(prelude, session_start_prelude, sizeof(prelude));
    memcpy(page, evenhub_enter_page, sizeof(page));
    SESSION_START_DISPATCH(SESSION_START_DASHBOARD_APP_ID, prelude, sizeof(prelude),
                           SESSION_START_EVENT_FROM_PHONE);
    SESSION_START_DISPATCH(EVENHUB_ENTER_APP_ID, page, sizeof(page),
                           SESSION_START_EVENT_FROM_PHONE);
}

/* Structure only: [34]{len:u8, message[len]}*, each message mode 16 or 30.
 * image_dispatch runs the messages, then evenhub_session_start_submit. A
 * failure NACKs, and the phone replays the whole message, which is safe: each
 * part is idempotent. */
int evenhub_session_start_check(const uint8_t *src, uint32_t len) {
    if (!src || len < 1 || src[0] != SESSION_START_MODE) return -1;
    for (uint32_t pos = 1; pos < len; ) {
        uint32_t size = src[pos];
        if (size < 1 || size > len - pos - 1) return -1;
        uint8_t mode = src[pos + 1];
        if (mode != CFW_MSG_AMBIENT_LIGHT && mode != CFW_MSG_BRIGHTNESS) return -1;
        pos += 1 + size;
    }
    return 0;
}

/* Only the master lens owns the EvenHub lifecycle; the left lens just ACKs. */
int evenhub_session_start_submit(void) {
    if (EVENHUB_ENTER_SIDE() != 1) return 0;
    return SESSION_START_POOL_SUBMIT(&session_start_job, 0, 0, 0, 0) == 0 ? 0 : -1;
}
