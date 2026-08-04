#include <string.h>
#include "protocol_raw.h"

void raw_parser_init(raw_parser_t *p)
{
    memset(p, 0, sizeof(*p));
}

void raw_parser_feed(raw_parser_t *p, uint8_t channel_id, const uint8_t *data, size_t len,
                      protocol_passthrough_cb_t passthrough_cb, void *cb_ctx)
{
    p->state.bytes_total += len;
    if (passthrough_cb) passthrough_cb(channel_id, data, len, cb_ctx);
}
