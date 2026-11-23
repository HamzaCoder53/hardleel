"""Minimal behaviour tree engine (no external dependencies - py_trees is not installed on the Pi).

Every node has tick() -> SUCCESS | FAILURE | RUNNING. A RUNNING node is ticked again on the next tick and
keeps its internal state; when a parent finishes or restarts, it calls reset() on its children.

Composites:  Sequence (all children in order; fails on the first failure)
             Fallback (children in order until one succeeds = "selector")
Decorators:  Retry(n), Timeout(s), Inverter, ForEach (runs the child once per item, item -> blackboard)
Leaves:      Condition(fn), Action subclasses (start() / update() / stop())
"""
import time

SUCCESS, FAILURE, RUNNING = 'SUCCESS', 'FAILURE', 'RUNNING'


class Node:
    def __init__(self, name):
        self.name = name
        self.status = None

    def tick(self, bb):
        self.status = self._tick(bb)
        return self.status

    def _tick(self, bb):
        raise NotImplementedError

    def reset(self):
        self.status = None

    def children(self):
        return []

    def describe(self, indent=0):
        """Tree as text with each node's last status (published on /mission/tree)."""
        mark = {SUCCESS: '[OK]', FAILURE: '[X] ', RUNNING: '[..]', None: '[  ]'}[self.status]
        lines = [f'{"  " * indent}{mark} {type(self).__name__}: {self.name}']
        for c in self.children():
            lines.extend(c.describe(indent + 1))
        return lines


class Sequence(Node):
    def __init__(self, name, children):
        super().__init__(name)
        self.kids, self.i = list(children), 0

    def children(self):
        return self.kids

    def _tick(self, bb):
        while self.i < len(self.kids):
            s = self.kids[self.i].tick(bb)
            if s == RUNNING:
                return RUNNING
            if s == FAILURE:
                self.reset()
                return FAILURE
            self.i += 1
        self.reset()
        return SUCCESS

    def reset(self):
        for c in self.kids:
            c.reset()
        self.i = 0


class Fallback(Node):
    def __init__(self, name, children):
        super().__init__(name)
        self.kids, self.i = list(children), 0

    def children(self):
        return self.kids

    def _tick(self, bb):
        while self.i < len(self.kids):
            s = self.kids[self.i].tick(bb)
            if s == RUNNING:
                return RUNNING
            if s == SUCCESS:
                self.reset()
                return SUCCESS
            self.i += 1
        self.reset()
        return FAILURE

    def reset(self):
        for c in self.kids:
            c.reset()
        self.i = 0


class Decorator(Node):
    def __init__(self, name, child):
        super().__init__(name)
        self.child = child

    def children(self):
        return [self.child]

    def reset(self):
        self.child.reset()


class Retry(Decorator):
    """Run the child up to n times until it succeeds."""
    def __init__(self, name, n, child):
        super().__init__(name, child)
        self.n, self.tries = n, 0

    def _tick(self, bb):
        s = self.child.tick(bb)
        if s == FAILURE:
            self.tries += 1
            self.child.reset()
            if self.tries < self.n:
                bb.log(f'{self.name}: attempt {self.tries} failed, retrying ({self.tries + 1}/{self.n})')
                return RUNNING
            self.tries = 0
            return FAILURE
        if s == SUCCESS:
            self.tries = 0
        return s

    def reset(self):
        super().reset()
        self.tries = 0


class Timeout(Decorator):
    def __init__(self, name, seconds, child):
        super().__init__(name, child)
        self.seconds, self.t0 = seconds, None

    def _tick(self, bb):
        self.t0 = self.t0 or time.monotonic()
        s = self.child.tick(bb)
        if s == RUNNING and time.monotonic() - self.t0 > self.seconds:
            bb.log(f'{self.name}: timed out after {self.seconds:.0f} s')
            self.child.reset()
            self.t0 = None
            return FAILURE
        if s != RUNNING:
            self.t0 = None
        return s

    def reset(self):
        super().reset()
        self.t0 = None


class Inverter(Decorator):
    def _tick(self, bb):
        s = self.child.tick(bb)
        return {SUCCESS: FAILURE, FAILURE: SUCCESS}.get(s, s)


class ForEach(Decorator):
    """Run the child once for every item of bb[list_key] (item stored in bb[item_key]). A failed item is
    logged and skipped, so one bad waste item does not end the mission. Succeeds when all were tried."""
    def __init__(self, name, list_key, item_key, child):
        super().__init__(name, child)
        self.list_key, self.item_key, self.i = list_key, item_key, 0

    def _tick(self, bb):
        items = bb[self.list_key]
        while self.i < len(items):
            bb[self.item_key] = items[self.i]
            s = self.child.tick(bb)
            if s == RUNNING:
                return RUNNING
            bb.setdefault('results', []).append((items[self.i], s))
            self.child.reset()
            self.i += 1
        self.i = 0
        return SUCCESS

    def reset(self):
        super().reset()
        self.i = 0


class Condition(Node):
    def __init__(self, name, fn):
        super().__init__(name)
        self.fn = fn

    def _tick(self, bb):
        return SUCCESS if self.fn(bb) else FAILURE


class Action(Node):
    """Leaf with a lifecycle: start(bb) once, then update(bb) every tick until it returns SUCCESS/FAILURE.
    stop() is called if the action is interrupted (parent reset while RUNNING)."""
    def __init__(self, name):
        super().__init__(name)
        self.started = False

    def _tick(self, bb):
        if not self.started:
            self.started = True
            s = self.start(bb)
            if s in (SUCCESS, FAILURE):
                self.started = False
                return s
        s = self.update(bb)
        if s != RUNNING:
            self.started = False
        return s

    def reset(self):
        if self.started:
            self.stop()
        self.started = False
        super().reset()

    def start(self, bb):
        return RUNNING

    def update(self, bb):
        return SUCCESS

    def stop(self):
        pass


class Blackboard(dict):
    """Shared memory between nodes. log() goes to the ROS logger (set by the mission node)."""
    def __init__(self, logger=print, **kw):
        super().__init__(**kw)
        self._logger = logger

    def log(self, msg):
        self._logger(msg)
