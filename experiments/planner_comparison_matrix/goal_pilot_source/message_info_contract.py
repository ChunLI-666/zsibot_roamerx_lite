"""Pure helpers; importing this module does not import ROS or create a context."""
from collections import Counter
from collections.abc import Mapping

UNSUPPORTED_SEQUENCE = (1 << 64) - 1
INFO_FIELDS = (
    "source_timestamp", "received_timestamp", "publication_sequence_number",
    "reception_sequence_number", "publisher_gid", "from_intra_process",
)


def _field(info, name):
    return info.get(name) if isinstance(info, Mapping) else getattr(info, name, None)


def _json_value(value):
    if value is None or isinstance(value, (str, int, float, bool)):
        return value
    if isinstance(value, (bytes, bytearray, memoryview, list, tuple)):
        return [_json_value(x) for x in value]
    try:
        return [_json_value(x) for x in value]
    except TypeError:
        return repr(value)


def positive_int(value):
    """Zero, missing, negative and non-integer timestamps remain unknown."""
    return value if isinstance(value, int) and not isinstance(value, bool) and value > 0 else None


def sequence(value):
    value = positive_int(value)
    return value if value is not None and value != UNSUPPORTED_SEQUENCE else None


def message_info_sample(info, callback_enter_unix_ns):
    raw = {name: _json_value(_field(info, name)) for name in INFO_FIELDS}
    source = positive_int(raw["source_timestamp"])
    received = positive_int(raw["received_timestamp"])
    return {
        "raw": raw,
        "source_timestamp_ns": source,
        "received_timestamp_ns": received,
        "publication_sequence_number": sequence(raw["publication_sequence_number"]),
        "reception_sequence_number": sequence(raw["reception_sequence_number"]),
        "clock_basis_verified": False,
        "arithmetic_only_deltas_ns": {
            "callback_enter_minus_received": None if received is None else callback_enter_unix_ns - received,
            "received_minus_source": None if received is None or source is None else received - source,
            "callback_enter_minus_source": None if source is None else callback_enter_unix_ns - source,
        },
        "interpretation": "Validate timestamp clock basis against wall/header/MCAP before interpreting deltas. Received-to-callback includes take/deserialization/executor/OS scheduling; it is not pure Python CPU time.",
    }


class DurationHistogram:
    """Constant-size aggregation, with no per-spin disk writes or sorting."""
    LIMITS_NS = (50_000, 100_000, 250_000, 500_000, 1_000_000,
                 2_000_000, 5_000_000, 10_000_000, 20_000_000,
                 50_000_000, 100_000_000, 250_000_000)

    def __init__(self):
        self.count = 0
        self.total_ns = 0
        self.minimum_ns = None
        self.maximum_ns = 0
        self.bins = Counter()

    def add(self, duration_ns):
        self.count += 1
        self.total_ns += duration_ns
        self.minimum_ns = duration_ns if self.minimum_ns is None else min(self.minimum_ns, duration_ns)
        self.maximum_ns = max(self.maximum_ns, duration_ns)
        index = next((i for i, limit in enumerate(self.LIMITS_NS) if duration_ns <= limit), len(self.LIMITS_NS))
        self.bins[index] += 1

    def summary(self):
        return {"count": self.count, "total_ns": self.total_ns, "minimum_ns": self.minimum_ns,
                "maximum_ns": self.maximum_ns, "upper_bounds_ns": list(self.LIMITS_NS) + [None],
                "bins": [self.bins[i] for i in range(len(self.LIMITS_NS) + 1)]}
