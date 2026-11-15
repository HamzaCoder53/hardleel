#!/usr/bin/env python3
"""hardleel tuning GUI (laptop, WSL): change the parameters of every running node live.

* Edits are ROS parameter sets: they change only the running node (gone after its restart / a new run).
* A value a node refuses live ("read at start only") is kept as PENDING: Save stores it for the next start.
* Save      -> ~/.hardleel/saved_params.yaml on the laptop AND (via tuning_helper) on the Pi, plus
               Cartographer values; the launch files load them at the next start.
* Clear saved -> deletes those files on both machines: next run = code defaults.
* Cartographer tab: lua values (Cartographer reads them only at start). "Apply" restarts Cartographer on
  the Pi with them for this run only (the map starts again); "Save" keeps them for later runs.
"""
import sys
import threading
import time

import rclpy
import yaml
from python_qt_binding.QtCore import Qt, QTimer
from python_qt_binding.QtGui import QBrush, QColor
from python_qt_binding.QtWidgets import (QAbstractItemView, QApplication, QHBoxLayout, QHeaderView, QLabel,
                                         QLineEdit, QListWidget, QMainWindow, QMessageBox, QPushButton,
                                         QSplitter, QTableWidget, QTableWidgetItem, QTabWidget, QVBoxLayout,
                                         QWidget)
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import DescribeParameters, GetParameters, ListParameters, SetParameters
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from robot_slam import tuning_store as ts
from robot_slam.carto_help import CARTO_HELP

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
HIDE_PARAMS = ('use_sim_time', 'qos_overrides.', 'start_type_description_service', 'tf_buffer')
HIDE_NODES = ('_ros2cli', 'transform_listener_impl', 'launch_ros', 'hardleel_tuning_gui', 'rviz',
              'teleop_twist_keyboard')   # substrings: NOT plain 'teleop' (that hid esp32_teleop_bridge)
# Nodes known to apply parameter changes while running; others usually need a restart (Save first).
LIVE_NODES = ('esp32_teleop_bridge', 'controller_server', 'planner_server', 'local_costmap', 'global_costmap',
              'velocity_smoother', 'amcl', 'behavior_server', 'collision_monitor', 'smoother_server',
              'bt_navigator', 'waypoint_follower', 'imu_serial_node', 'frontier_explorer')
# Never written by Save: the explorer must not start driving by itself on the next launch.
NEVER_SAVE = {('/frontier_explorer', 'enabled')}
GREEN, YELLOW, RED, GREY = QColor(200, 240, 200), QColor(255, 240, 180), QColor(255, 205, 205), QColor(225, 225, 225)


def to_python(pv):
    t = pv.type
    return {ParameterType.PARAMETER_BOOL: pv.bool_value, ParameterType.PARAMETER_INTEGER: pv.integer_value,
            ParameterType.PARAMETER_DOUBLE: pv.double_value, ParameterType.PARAMETER_STRING: pv.string_value,
            ParameterType.PARAMETER_BOOL_ARRAY: list(pv.bool_array_value),
            ParameterType.PARAMETER_INTEGER_ARRAY: list(pv.integer_array_value),
            ParameterType.PARAMETER_DOUBLE_ARRAY: list(pv.double_array_value),
            ParameterType.PARAMETER_STRING_ARRAY: list(pv.string_array_value)}.get(t)


TYPE_NAMES = {1: 'bool', 2: 'int', 3: 'double', 4: 'string', 6: 'bool[]', 7: 'int[]', 8: 'double[]', 9: 'string[]'}


def parse(text, ptype):
    """Text from the table -> value of the parameter's type (raises ValueError)."""
    text = text.strip()
    if ptype == ParameterType.PARAMETER_BOOL:
        if text.lower() in ('true', '1', 'yes', 'on'):
            return True
        if text.lower() in ('false', '0', 'no', 'off'):
            return False
        raise ValueError('use true/false')
    if ptype == ParameterType.PARAMETER_INTEGER:
        return int(text)
    if ptype == ParameterType.PARAMETER_DOUBLE:
        return float(text)
    if ptype == ParameterType.PARAMETER_STRING:
        return text
    val = yaml.safe_load(text)
    if not isinstance(val, list):
        raise ValueError('use a list like [1, 2, 3]')
    conv = {6: bool, 7: int, 8: float, 9: str}[ptype]
    return [conv(v) for v in val]


def to_msg_value(value, ptype):
    pv = ParameterValue(type=ptype)
    attr = {1: 'bool_value', 2: 'integer_value', 3: 'double_value', 4: 'string_value', 6: 'bool_array_value',
            7: 'integer_array_value', 8: 'double_array_value', 9: 'string_array_value'}[ptype]
    setattr(pv, attr, value)
    return pv


def fmt(v):
    if isinstance(v, bool):
        return 'true' if v else 'false'
    if isinstance(v, float):
        return f'{v:g}'
    return yaml.safe_dump(v, default_flow_style=True).strip().removesuffix('...').strip() if isinstance(v, list) \
        else str(v)


class RosSide(Node):
    def __init__(self):
        super().__init__('hardleel_tuning_gui')
        self.carto_info = None
        self.pi_saved = None
        self.statuses = []
        self.save_pub = self.create_publisher(String, '/hardleel_tuning/save', 10)
        self.apply_pub = self.create_publisher(String, '/hardleel_tuning/carto_apply', 10)
        self.create_subscription(String, '/hardleel_tuning/carto_info', self.on_info, LATCHED)
        self.create_subscription(String, '/hardleel_tuning/saved_params', self.on_saved, LATCHED)
        self.create_subscription(String, '/hardleel_tuning/status', lambda m: self.statuses.append(m.data), 10)
        # auto mapping (frontier_explorer on the laptop, map saving via tuning_helper on the Pi)
        self.explore_status = None
        self.explore_pub = self.create_publisher(String, '/explore/command', 10)
        self.save_map_pub = self.create_publisher(String, '/hardleel_tuning/save_map', 10)
        self.create_subscription(String, '/explore/status', self.on_explore_status, LATCHED)

    def on_explore_status(self, msg):
        self.explore_status = msg.data

    def on_info(self, msg):
        self.carto_info = yaml.safe_load(msg.data) or {}

    def on_saved(self, msg):
        self.pi_saved = yaml.safe_load(msg.data) or {}

    def call(self, srv_type, name, req, timeout=4.0):
        cli = self.create_client(srv_type, name)
        try:
            if not cli.wait_for_service(timeout_sec=timeout):
                raise TimeoutError(f'{name} not available')
            fut = cli.call_async(req)
            done = threading.Event()
            fut.add_done_callback(lambda _: done.set())
            if not done.wait(timeout):
                raise TimeoutError(f'{name} timed out')
            return fut.result()
        finally:
            self.destroy_client(cli)

    def nodes(self):
        out = []
        for name, ns in self.get_node_names_and_namespaces():
            full = (ns.rstrip('/') + '/' + name) if ns != '/' else '/' + name
            if not any(h in full for h in HIDE_NODES):
                out.append(full)
        return sorted(set(out))

    def read_params(self, node):
        names = self.call(ListParameters, f'{node}/list_parameters', ListParameters.Request()).result.names
        names = sorted(n for n in names if not n.startswith(HIDE_PARAMS))
        if not names:
            return []
        vals = self.call(GetParameters, f'{node}/get_parameters', GetParameters.Request(names=names)).values
        descs = self.call(DescribeParameters, f'{node}/describe_parameters',
                          DescribeParameters.Request(names=names)).descriptors
        return [(n, v.type, to_python(v), d) for n, v, d in zip(names, vals, descs) if v.type in TYPE_NAMES]

    def set_param(self, node, name, ptype, value):
        req = SetParameters.Request(parameters=[Parameter(name=name, value=to_msg_value(value, ptype))])
        r = self.call(SetParameters, f'{node}/set_parameters', req).results[0]
        return r.successful, r.reason


class TuningWindow(QMainWindow):
    def __init__(self, ros):
        super().__init__()
        self.ros = ros
        self.defaults = {}    # (node, param) -> value at first sight this run (= code default or saved)
        self.current = {}     # (node, param) -> value the node has now
        self.pending = {}     # (node, param) -> value the node refused live (stored on Save)
        self.types = {}
        self.node = None
        self.loading = False
        self.setWindowTitle('hardleel tuning - live changes are temporary until you press Save')
        self.resize(1250, 760)

        tabs = QTabWidget()
        tabs.addTab(self.build_params_tab(), 'Node parameters (live)')
        tabs.addTab(self.build_carto_tab(), 'Cartographer (lua, restart)')
        tabs.addTab(self.build_auto_tab(), 'Auto mapping')
        central = QWidget()
        lay = QVBoxLayout(central)
        lay.addWidget(tabs)
        bar = QHBoxLayout()
        for text, fn in (('Save all changes', self.save), ('Clear saved (next run = defaults)', self.clear_saved)):
            b = QPushButton(text)
            b.clicked.connect(fn)
            bar.addWidget(b)
        bar.addStretch(1)
        self.status = QLabel('Pick a node on the left.')
        bar.addWidget(self.status, 3)
        lay.addLayout(bar)
        self.setCentralWidget(central)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.poll)
        self.timer.start(500)
        QTimer.singleShot(300, self.refresh_nodes)

    # ---- node parameters tab -------------------------------------------------------------------
    def build_params_tab(self):
        w = QWidget()
        split = QSplitter()
        left = QWidget()
        ll = QVBoxLayout(left)
        self.node_list = QListWidget()
        self.node_list.currentTextChanged.connect(self.select_node)
        ll.addWidget(QLabel('Running nodes (Pi + laptop)'))
        ll.addWidget(self.node_list)
        b = QPushButton('Refresh nodes')
        b.clicked.connect(self.refresh_nodes)
        ll.addWidget(b)
        right = QWidget()
        rl = QVBoxLayout(right)
        top = QHBoxLayout()
        self.filter = QLineEdit()
        self.filter.setPlaceholderText('filter parameters (e.g. inflation, max_vel, timeout)')
        self.filter.textChanged.connect(self.apply_filter)
        top.addWidget(self.filter)
        for text, fn in (('Reload', lambda: self.select_node(self.node)), ('Reset this node', self.reset_node),
                         ('Reset ALL nodes', self.reset_all)):
            bb = QPushButton(text)
            bb.clicked.connect(fn)
            top.addWidget(bb)
        rl.addLayout(top)
        self.table = QTableWidget(0, 5)
        self.table.setHorizontalHeaderLabels(['Parameter', 'Value (edit)', 'Default this run', 'Type', 'Status'])
        self.table.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeToContents)
        self.table.horizontalHeader().setSectionResizeMode(4, QHeaderView.Stretch)
        self.table.setEditTriggers(QAbstractItemView.DoubleClicked | QAbstractItemView.EditKeyPressed)
        self.table.itemChanged.connect(self.edited)
        rl.addWidget(self.table)
        self.hint = QLabel()
        rl.addWidget(self.hint)
        split.addWidget(left)
        split.addWidget(right)
        split.setSizes([300, 950])
        QVBoxLayout(w).addWidget(split)
        return w

    def refresh_nodes(self):
        keep = self.node
        self.node_list.clear()
        self.node_list.addItems(self.ros.nodes())
        if keep:
            items = self.node_list.findItems(keep, Qt.MatchExactly)
            if items:
                self.node_list.setCurrentItem(items[0])
        self.say(f'{self.node_list.count()} nodes found')

    def select_node(self, node):
        if not node:
            return
        self.node = node
        try:
            params = self.ros.read_params(node)
        except Exception as e:  # noqa: BLE001
            self.say(f'{node}: {e}')
            return
        self.loading = True
        self.table.setRowCount(0)
        for name, ptype, value, desc in params:
            key = (node, name)
            self.defaults.setdefault(key, value)
            self.current[key] = value
            self.types[key] = ptype
            r = self.table.rowCount()
            self.table.insertRow(r)
            shown = self.pending.get(key, value)
            cells = [name, fmt(shown), fmt(self.defaults[key]), TYPE_NAMES[ptype], '']
            for c, text in enumerate(cells):
                it = QTableWidgetItem(text)
                if c != 1 or desc.read_only:
                    it.setFlags(it.flags() & ~Qt.ItemIsEditable)
                if desc.description:
                    it.setToolTip(desc.description)
                self.table.setItem(r, c, it)
            self.paint(r, key, 'read only' if desc.read_only else None)
        self.loading = False
        live = any(n in node for n in LIVE_NODES)
        self.hint.setText('This node applies changes LIVE.' if live else
                          'This node probably reads most values only at start: change, Save, then restart it.')
        self.apply_filter()
        self.say(f'{node}: {len(params)} parameters')

    def paint(self, row, key, note=None):
        if key in self.pending:
            color, text = YELLOW, f'PENDING (refused live) - Save to use at next start'
        elif self.current.get(key) != self.defaults.get(key):
            color, text = GREEN, 'changed (live, not saved)'
        else:
            color, text = None, ''
        if note:
            color, text = GREY, note
        self.table.item(row, 4).setText(text)
        for c in range(5):
            self.table.item(row, c).setBackground(QBrush(color) if color else QBrush())

    def apply_filter(self):
        f = self.filter.text().lower()
        for r in range(self.table.rowCount()):
            self.table.setRowHidden(r, f not in self.table.item(r, 0).text().lower())

    def edited(self, item):
        if self.loading or item.column() != 1:
            return
        row = item.row()
        name = self.table.item(row, 0).text()
        key = (self.node, name)
        try:
            value = parse(item.text(), self.types[key])
        except ValueError as e:
            self.say(f'{name}: bad value ({e})')
            self.loading = True
            item.setText(fmt(self.pending.get(key, self.current[key])))
            self.loading = False
            return
        self.apply_value(self.node, name, value, row)

    def apply_value(self, node, name, value, row=None):
        key = (node, name)
        try:
            ok, reason = self.ros.set_param(node, name, self.types[key], value)
        except Exception as e:  # noqa: BLE001
            ok, reason = False, str(e)
        if ok:
            self.current[key] = value
            self.pending.pop(key, None)
            self.say(f'{node} {name} = {fmt(value)} (live)')
        else:
            if value == self.current.get(key):
                self.pending.pop(key, None)
            else:
                self.pending[key] = value
            self.say(f'{node} {name}: {reason or "refused"}')
        if row is not None and node == self.node:
            self.loading = True
            self.table.item(row, 1).setText(fmt(self.pending.get(key, self.current[key])))
            self.paint(row, key)
            self.loading = False
        return ok

    def reset_keys(self, keys):
        n = 0
        for key in keys:
            self.pending.pop(key, None)
            if self.current.get(key) != self.defaults.get(key):
                n += self.apply_value(key[0], key[1], self.defaults[key])
        return n

    def reset_node(self):
        if self.node:
            n = self.reset_keys([k for k in self.defaults if k[0] == self.node])
            self.select_node(self.node)
            self.say(f'{self.node}: {n} value(s) back to this run\'s defaults')

    def reset_all(self):
        n = self.reset_keys(list(self.defaults))
        if self.node:
            self.select_node(self.node)
        self.say(f'{n} value(s) back to this run\'s defaults on all nodes')

    # ---- Cartographer tab ----------------------------------------------------------------------
    def build_carto_tab(self):
        w = QWidget()
        lay = QVBoxLayout(w)
        lay.addWidget(QLabel('Cartographer reads these only at start. "Apply" restarts Cartographer on the Pi with '
                             'them for THIS RUN (the current map is lost - save it first). Values: numbers, '
                             'true/false or math.rad(x).'))
        self.carto = QTableWidget(0, 5)
        self.carto.setHorizontalHeaderLabels(['Lua key', 'Value (edit)', 'Default', 'Meaning (hover for all)',
                                              'Status'])
        self.carto.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeToContents)
        self.carto.horizontalHeader().setSectionResizeMode(3, QHeaderView.Stretch)
        self.carto.horizontalHeader().setSectionResizeMode(4, QHeaderView.ResizeToContents)
        self.carto.itemChanged.connect(self.carto_edited)
        lay.addWidget(self.carto)
        bar = QHBoxLayout()
        for text, fn in (('Apply now (restart Cartographer)', self.carto_apply),
                         ('Reset table to defaults', self.carto_reset)):
            b = QPushButton(text)
            b.clicked.connect(fn)
            bar.addWidget(b)
        bar.addStretch(1)
        lay.addLayout(bar)
        self.carto_loaded = False
        return w

    def fill_carto(self, info):
        d, saved, session = info.get('defaults', {}), info.get('saved', {}), info.get('session', {})
        self.loading = True
        self.carto.setRowCount(0)
        order = [k for k in CARTO_HELP if k in d] + sorted(k for k in d if k not in CARTO_HELP)
        for key in order:
            val = str(session.get(key, saved.get(key, d[key])))
            r = self.carto.rowCount()
            self.carto.insertRow(r)
            help_text = CARTO_HELP.get(key, '')
            for c, text in enumerate([key, val, d[key], help_text, '']):
                it = QTableWidgetItem(text)
                if c != 1:
                    it.setFlags(it.flags() & ~Qt.ItemIsEditable)
                if help_text:
                    it.setToolTip(help_text)
                self.carto.setItem(r, c, it)
            self.carto_paint(r, key in session, key in saved)
        self.loading = False
        self.carto_loaded = True

    def carto_paint(self, r, in_session=False, in_saved=False):
        changed = self.carto.item(r, 1).text().strip() != self.carto.item(r, 2).text().strip()
        text = 'this run (applied)' if in_session else 'saved' if in_saved else 'edited, not applied' \
            if changed else ''
        self.carto.item(r, 4).setText(text)
        color = GREEN if in_session else YELLOW if (changed or in_saved) else None
        for c in range(5):
            self.carto.item(r, c).setBackground(QBrush(color) if color else QBrush())

    def carto_edited(self, item):
        if self.loading or item.column() != 1:
            return
        if not ts.LUA_VALUE.match(item.text()):
            self.say(f'{self.carto.item(item.row(), 0).text()}: use a number, true/false or math.rad(x)')
        self.loading = True
        self.carto_paint(item.row())
        self.loading = False

    def carto_values(self):
        """{key: value} for every row that differs from the default (= the overrides wanted)."""
        out = {}
        for r in range(self.carto.rowCount()):
            k, v, d = (self.carto.item(r, c).text().strip() for c in range(3))
            if v != d:
                out[k] = v
        return out

    def carto_apply(self):
        if self.ros.carto_info is None:
            self.say('Cartographer values not received - is tuning_helper running on the Pi?')
            return
        vals = self.carto_values()
        bad = [k for k, v in vals.items() if not ts.LUA_VALUE.match(v)]
        if bad:
            self.say(f'invalid values: {bad}')
            return
        saved = self.ros.carto_info.get('saved', {})
        session = dict(vals)
        session.update({k: self.carto_default(k) for k in saved if k not in vals})  # undo saved ones set back
        if QMessageBox.question(self, 'Restart Cartographer?',
                                'Cartographer restarts with these values and the current map starts again.\n'
                                'Save the map first if you need it. Continue?') != QMessageBox.Yes:
            return
        self.ros.apply_pub.publish(String(data=yaml.safe_dump(session)))
        self.say('Cartographer restart requested on the Pi ...')

    def carto_default(self, key):
        return str(self.ros.carto_info.get('defaults', {}).get(key, ''))

    def carto_reset(self):
        self.loading = True
        for r in range(self.carto.rowCount()):
            self.carto.item(r, 1).setText(self.carto.item(r, 2).text())
            self.carto_paint(r)
        self.loading = False
        self.say('Cartographer table reset to defaults (press Apply to use them now, Save to keep)')

    # ---- auto mapping tab ----------------------------------------------------------------------
    def build_auto_tab(self):
        w = QWidget()
        lay = QVBoxLayout(w)
        lay.addWidget(QLabel(
            'Auto mapping: Pi "bash ~/start_robot.sh auto", laptop "hauto". The robot explores until no unknown '
            'edges (frontiers) are left.\nStay with the robot - Stop (or teleop k / Ctrl+C in the hauto window) '
            'ends it. Explorer and Nav2 settings: Node parameters tab -> /frontier_explorer, /controller_server, '
            '/local_costmap/local_costmap, /velocity_smoother ...'))
        self.auto_status = QLabel('explorer: not running (start hauto on the laptop)')
        self.auto_status.setStyleSheet('font-size: 15px; font-weight: bold; padding: 8px;')
        self.auto_status.setWordWrap(True)
        lay.addWidget(self.auto_status)
        row = QHBoxLayout()
        for text, cmd in (('Start exploring', 'start'), ('STOP', 'stop'), ('Return to start', 'home')):
            b = QPushButton(text)
            b.setMinimumHeight(48)
            if cmd == 'stop':
                b.setStyleSheet('background: #d33; color: white; font-weight: bold;')
            b.clicked.connect(lambda _=False, c=cmd: self.explore_cmd(c))
            row.addWidget(b)
        lay.addLayout(row)
        row2 = QHBoxLayout()
        row2.addWidget(QLabel('Map name:'))
        self.map_name = QLineEdit(time.strftime('auto_%Y%m%d_%H%M'))
        row2.addWidget(self.map_name)
        b = QPushButton('Save map on the Pi (~/maps, mapping keeps running)')
        b.clicked.connect(self.save_map)
        row2.addWidget(b)
        lay.addLayout(row2)
        lay.addStretch(1)
        return w

    def explore_cmd(self, cmd):
        if cmd == 'start' and QMessageBox.question(
                self, 'Start auto mapping?', 'The robot will drive by itself.\nAre you with the robot and is '
                'the area clear?') != QMessageBox.Yes:
            return
        self.ros.explore_pub.publish(String(data=cmd))
        self.say(f'explorer: {cmd} sent')

    def save_map(self):
        name = ''.join(ch for ch in self.map_name.text().strip() if ch.isalnum() or ch in '_-')
        if not name:
            self.say('give the map a name (letters, digits, _ or -)')
            return
        self.ros.save_map_pub.publish(String(data=name))
        self.say(f'saving map "{name}" on the Pi ...')

    # ---- save / clear ---------------------------------------------------------------------------
    def wanted_overrides(self):
        """Everything that differs from this run's start (live changes + pending), merged over what is saved."""
        flat = ts.load_saved_flat()
        for key, d in self.defaults.items():
            val = self.pending.get(key, self.current.get(key))
            if val != d:
                flat.setdefault(key[0], {})[key[1]] = val
        for key, val in self.pending.items():
            flat.setdefault(key[0], {})[key[1]] = val
        for node, name in NEVER_SAVE:
            flat.get(node, {}).pop(name, None)
        return {n: p for n, p in flat.items() if p}

    def save(self):
        flat = self.wanted_overrides()
        carto = self.carto_values() if self.carto_loaded else ts.load(ts.SAVED_CARTO)
        ts.save_flat(flat)                       # laptop copy (Nav2, rf2o, EKF on the laptop)
        ts.write(ts.SAVED_CARTO, carto)
        self.ros.save_pub.publish(String(data=yaml.safe_dump({'params': flat, 'carto': carto})))  # Pi copy
        n = sum(len(p) for p in flat.values())
        self.say(f'Saved {n} parameter(s) + {len(carto)} Cartographer value(s) on the laptop; sent to the Pi ...')

    def clear_saved(self):
        if QMessageBox.question(self, 'Clear saved values?',
                                'Delete the saved values on the laptop and the Pi?\nThe next run starts from the '
                                'code defaults (this run is not changed).') != QMessageBox.Yes:
            return
        ts.save_flat({})
        ts.write(ts.SAVED_CARTO, {})
        self.ros.save_pub.publish(String(data=yaml.safe_dump({'params': {}, 'carto': {}})))
        self.say('Saved values cleared on the laptop; clearing on the Pi ...')

    # ---- misc ----------------------------------------------------------------------------------
    def poll(self):
        if self.ros.explore_status is not None:
            self.auto_status.setText('explorer: ' + self.ros.explore_status)
        if self.ros.carto_info is not None and not self.carto_loaded:
            self.fill_carto(self.ros.carto_info)
        while self.ros.statuses:
            msg = self.ros.statuses.pop(0)
            self.say(msg)
            if self.ros.carto_info is not None:
                self.fill_carto(self.ros.carto_info)

    def say(self, text):
        self.status.setText(f'{time.strftime("%H:%M:%S")}  {text}')


def main():
    rclpy.init()
    ros = RosSide()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(ros)
    threading.Thread(target=ex.spin, daemon=True).start()
    app = QApplication(sys.argv)
    win = TuningWindow(ros)
    win.show()
    code = app.exec_()
    ex.shutdown()
    ros.destroy_node()
    rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
