"""Pluggable "wait for the item to be loaded" step. The organisers have not published the loading protocol yet;
drop a new adapter in here and select it with the mission param `loading_adapter`.

An adapter gets the mission node (for `wait`, subscriptions, logging) and its params, and `run()` returns
(result, detail): result 'done' (protocol confirmed) or 'waited' (gave up after the timeout and carried on).
"""
from std_msgs.msg import Empty


class StubLoadingAdapter:
    """PLACEHOLDER protocol: wait `load_wait_s`, or less if anything is published on /loading/done."""

    def __init__(self, node, params):
        self.node, self.wait_s = node, float(params.get('load_wait_s', 6.0))
        self.done = False
        node.create_subscription(Empty, '/loading/done', self._on_done, 1)

    def _on_done(self, _msg):
        self.done = True

    def run(self):
        self.done = False
        ok = self.node.wait(lambda: self.done, self.wait_s)
        return ('done', 'loading/done received') if ok else ('waited', f'stub: waited {self.wait_s:.1f} s')


ADAPTERS = {'stub': StubLoadingAdapter}


def make_loading_adapter(kind, node, params):
    if kind not in ADAPTERS:
        node.get_logger().error(f'unknown loading_adapter "{kind}", using stub')
        kind = 'stub'
    return ADAPTERS[kind](node, params)
