"""Helper: set parameters on /motor_bridge via the rcl_interfaces SetParameters service.

Polls the future, so something else must be spinning the node (a spinner thread or a
multithreaded executor)."""
import time

from rcl_interfaces.msg import Parameter as ParamMsg, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters


def _pv(v):
    if isinstance(v, bool):
        return ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=v)
    if isinstance(v, int):
        return ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=v)
    return ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(v))


def set_bridge_params(node, params, target='/motor_bridge', timeout=3.0, client=None):
    """params: {name: value}. Returns (ok, message)."""
    cli = client or node.create_client(SetParameters, target + '/set_parameters')
    t0 = time.monotonic()
    while not cli.service_is_ready():
        if time.monotonic() - t0 > timeout:
            return False, f'{target} set_parameters service not available'
        time.sleep(0.05)
    req = SetParameters.Request()
    req.parameters = [ParamMsg(name=k, value=_pv(v)) for k, v in params.items()]
    fut = cli.call_async(req)
    t0 = time.monotonic()
    while not fut.done() and time.monotonic() - t0 < timeout:
        time.sleep(0.01)
    if not fut.done():
        return False, 'timeout setting parameters'
    bad = [f'{k}: {r.reason}' for k, r in zip(params, fut.result().results) if not r.successful]
    return (not bad), ('; '.join(bad) if bad else 'ok')
