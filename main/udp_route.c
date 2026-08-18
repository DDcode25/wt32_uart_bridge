#include "udp_route.h"

udp_route_plan_t udp_route_plan(const udp_dest_t *dests, size_t count,
                                bool has_learned_peer)
{
    udp_route_plan_t plan = { .kind = UDP_ROUTE_NONE, .explicit_count = 0 };

    if (dests) {
        if (count > UDP_ROUTE_MAX_DESTINATIONS) count = UDP_ROUTE_MAX_DESTINATIONS;
        for (size_t i = 0; i < count; i++) {
            const udp_dest_t *d = &dests[i];
            if (!d->enabled || d->port == 0 || d->ip == 0) continue;
            plan.idx[plan.explicit_count++] = (uint8_t)i;
        }
    }

    if (plan.explicit_count) {
        plan.kind = UDP_ROUTE_EXPLICIT;
    } else if (has_learned_peer) {
        plan.kind = UDP_ROUTE_LEARNED;
    }
    return plan;
}
