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
