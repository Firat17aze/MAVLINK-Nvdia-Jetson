from __future__ import annotations

from .models import HealthState


class HealthMonitor:
    def build(
        self,
        now_s: float,
        cpu_pct: float,
        inference_latency_ms: float,
        link_rtt_ms: float,
        companion_alive: bool,
        mavlink_heartbeat_age_s: float,
    ) -> HealthState:
        return HealthState(
            timestamp_s=now_s,
            cpu_pct=cpu_pct,
            inference_latency_ms=inference_latency_ms,
            link_rtt_ms=link_rtt_ms,
            companion_alive=companion_alive,
            mavlink_heartbeat_age_s=mavlink_heartbeat_age_s,
        )
