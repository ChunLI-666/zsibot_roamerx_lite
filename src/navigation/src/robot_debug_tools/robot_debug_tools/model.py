"""A bounded read model: identity and validity, never algorithmic health or root cause."""
from collections import OrderedDict
import time


def uuid_key(goal_id):
    return bytes(goal_id.uuid).hex()


class ObservationStore:
    def __init__(self, checker_timeout=1.0, clock=time.monotonic):
        self.actions = OrderedDict()
        self.observations = OrderedDict()
        self.revisions = {}
        self.dropped = 0
        self.received = {}
        self.checker_timeout = checker_timeout
        self.clock = clock

    def clear(self):
        self.actions.clear()
        self.observations.clear()
        self.revisions.clear()
        self.received.clear()

    def action(self, endpoint, status, observed_at, build=None, binding_valid=False):
        from robot_debug_msgs.msg import ActionState
        key = (endpoint, uuid_key(status.goal_info.goal_id))
        item = ActionState(label=endpoint + '/' + key[1], value=status.status,
                           endpoint=endpoint, goal_id=status.goal_info.goal_id,
                           goal_accepted_stamp=status.goal_info.stamp,
                           observed_at=observed_at, source_binding_valid=binding_valid)
        if build is not None:
            item.source_build = build
        self.actions[key] = item
        self.actions.move_to_end(key)
        if len(self.actions) > 64:
            # Native status is event-driven: a long-running task need not emit
            # another sample while short planner goals keep completing. Prefer
            # evicting inactive history over ACCEPTED/EXECUTING/CANCELING goals.
            # An all-active overflow still has the same bound and drop counter.
            oldest = next(iter(self.actions))
            removed = next((candidate for candidate, state in self.actions.items()
                            if state.value not in (1, 2, 3)), oldest)
            self.actions.pop(removed)
            self.dropped += 1

    def observation(self, msg):
        if msg.checker:
            slot = 'checker'
        elif msg.branch:
            slot = 'branch/' + msg.branch[0].site
        elif msg.gate:
            slot = 'gate'
        elif msg.transport:
            slot = 'transport'
        elif msg.execution:
            slot = 'execution'
        else:
            return
        key = (msg.component, msg.source_instance, slot)
        self.observations[key] = msg
        self.received[key] = self.clock()
        self.observations.move_to_end(key)
        if len(self.observations) > 32:
            removed, _ = self.observations.popitem(last=False)
            self.received.pop(removed, None)
            self.dropped += 1
        if msg.goal_id_valid and (msg.checker or (
                msg.branch and msg.branch[0].site == 'path_update')):
            revision_key = (msg.source_instance, uuid_key(msg.goal_id))
            self.revisions[revision_key] = max(msg.path_revision,
                                               self.revisions.get(revision_key, 0))
            if len(self.revisions) > 128:
                self.revisions.pop(next(iter(self.revisions)))

    def current_checker(self):
        active = {(endpoint, goal) for (endpoint, goal), state in self.actions.items()
                  if endpoint.rsplit('/', 1)[-1] == 'follow_path' and state.value == 2}
        if not active:
            return None, 'no_observed_executing_follow_path'
        if len(active) != 1:
            return None, 'ambiguous_executing_follow_path'
        endpoint, goal = next(iter(active))
        candidates = [(key, msg) for key, msg in self.observations.items()
                      if msg.checker and msg.goal_id_valid and uuid_key(msg.goal_id) == goal
                      and msg.endpoint.rstrip('/') == endpoint]
        instances = {msg.source_instance for _, msg in candidates}
        if len(instances) > 1:
            return None, 'ambiguous_checker_source_instance'
        if not candidates:
            return None, 'no_observed_checker_for_goal'
        key, msg = candidates[-1]
        if self.clock() - self.received[key] > self.checker_timeout:
            return None, 'checker_observation_not_recent'
        if msg.path_revision != self.revisions.get((msg.source_instance, goal)):
            return None, 'path_replaced_since_evaluation'
        if not msg.checker[0].evaluated or not msg.checker[0].result_valid:
            return None, 'checker_not_evaluated'
        return msg, 'actual_checker_return'
