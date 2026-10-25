"""Pluggable sender of mission events to the judges system. The real format is UNKNOWN: everything here is
PLACEHOLDER until the organisers publish it. A sender must never raise or block the mission.
"""
import json
import urllib.request


class JudgesSender:
    def __init__(self, logger=print):
        self.log = logger

    def send_event(self, robot_id, event, payload):
        raise NotImplementedError

    def send_summary(self, robot_id, summary):
        raise NotImplementedError


class StubSender(JudgesSender):
    def send_event(self, robot_id, event, payload):
        try:
            self.log('would send event %s/%s: %s' % (robot_id, event, json.dumps(payload, default=str)))
        except Exception:       # noqa: BLE001
            pass
        return True

    def send_summary(self, robot_id, summary):
        try:
            self.log('would send summary %s: %s' % (robot_id, json.dumps(summary, default=str)))
        except Exception:       # noqa: BLE001
            pass
        return True


class HttpJsonSender(JudgesSender):
    """POST JSON to `url` (PLACEHOLDER body shape). Errors are logged and swallowed; blocks <= timeout."""

    def __init__(self, url, timeout=1.0, logger=print):
        super().__init__(logger)
        self.url = url
        self.timeout = float(timeout)

    def _post(self, body):
        if not self.url:
            self.log('http sender: no url set, skipping')
            return False
        try:
            req = urllib.request.Request(self.url, data=json.dumps(body, default=str).encode('utf-8'),
                                         headers={'Content-Type': 'application/json'}, method='POST')
            with urllib.request.urlopen(req, timeout=self.timeout) as r:
                r.read(256)
            return True
        except Exception as e:      # noqa: BLE001 - never break the mission
            self.log('http sender error: %s' % e)
            return False

    def send_event(self, robot_id, event, payload):
        return self._post({'robot_id': robot_id, 'event': event, 'payload': payload})   # PLACEHOLDER shape

    def send_summary(self, robot_id, summary):
        return self._post({'robot_id': robot_id, 'summary': summary})                   # PLACEHOLDER shape


def make_sender(kind='stub', **kw):
    """kind: 'stub' | 'http' (kw: url, timeout, logger). Unknown kinds fall back to stub."""
    logger = kw.get('logger') or print
    if kind == 'http':
        return HttpJsonSender(kw.get('url', ''), kw.get('timeout', 1.0), logger)
    return StubSender(logger)
