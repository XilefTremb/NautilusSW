"""Read-only mission telemetry page."""
import time

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QColor
from .state_graph import StateGraph
from PyQt5.QtWidgets import (
    QWidget, QLabel, QVBoxLayout, QHBoxLayout, QListWidget, QListWidgetItem,
    QProgressBar, QTableWidget, QTableWidgetItem, QHeaderView, QAbstractItemView,
    QSplitter, QSizePolicy,
)


class MissionMonitorPage(QWidget):
    COLORS = {'COMPLETED': '#00c853', 'ACTIVE': '#4fc3f7',
              'SKIPPED': '#ffb74d', 'PENDING': '#8391a7'}

    def __init__(self, data):
        super().__init__()
        self.last_received = None
        self.last_status = None
        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 6, 8, 6)
        layout.setSpacing(4)
        header = QHBoxLayout()
        header.setSpacing(12)
        layout.addLayout(header)
        title = QLabel('Mission Monitor')
        title.setStyleSheet('font-size: 16px; font-weight: bold; padding: 0px; margin: 0px;')
        header.addWidget(title)
        self.summary = QLabel('No mission telemetry')
        header.addWidget(self.summary)
        header.addStretch(1)
        self.connection = QLabel('Waiting for /mission/status')
        header.addWidget(self.connection)
        status_row = QHBoxLayout()
        layout.addLayout(status_row)
        self.current = QLabel('—')
        self.current.setStyleSheet('font-weight: bold; padding: 0px; margin: 0px;')
        self.current.setWordWrap(True)
        status_row.addWidget(self.current, 1)
        self.timing = QLabel('—')
        status_row.addWidget(self.timing)
        for label in (title, self.summary, self.connection, self.current, self.timing):
            label.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Maximum)
        vertical = QSplitter(Qt.Vertical)
        vertical.setChildrenCollapsible(False)
        layout.addWidget(vertical, 1)
        body = QSplitter(Qt.Horizontal)
        body.setChildrenCollapsible(False)
        vertical.addWidget(body)
        left_panel = QWidget()
        right_panel = QWidget()
        left = QVBoxLayout(left_panel)
        right = QVBoxLayout(right_panel)
        left.setContentsMargins(0, 0, 0, 0)
        right.setContentsMargins(0, 0, 0, 0)
        right.setSpacing(0)
        left.setSpacing(3)
        body.addWidget(left_panel)
        body.addWidget(right_panel)
        body.setStretchFactor(0, 1)
        body.setStretchFactor(1, 4)
        body.setSizes([240, 960])
        left.addWidget(QLabel('MISSION SEQUENCE'))
        self.sequence = QListWidget()
        self.sequence.setSelectionMode(QAbstractItemView.NoSelection)
        left.addWidget(self.sequence)
        self.progress = QProgressBar()
        left.addWidget(self.progress)
        self.graph = StateGraph()
        right.addWidget(self.graph, 3)
        details = QSplitter(Qt.Horizontal)
        details.setChildrenCollapsible(False)
        vertical.addWidget(details)
        details.setMinimumHeight(150)
        vertical.setStretchFactor(0, 3)
        vertical.setStretchFactor(1, 2)
        config_panel = QWidget()
        config_layout = QVBoxLayout(config_panel)
        config_layout.setContentsMargins(0, 0, 0, 0)
        config_layout.setSpacing(3)
        config_layout.addWidget(QLabel('ACTIVE CONFIGURATION AND RUNTIME'))
        history_panel = QWidget()
        history_layout = QVBoxLayout(history_panel)
        history_layout.setContentsMargins(0, 0, 0, 0)
        history_layout.setSpacing(3)
        history_layout.addWidget(QLabel('RECENT STATE TRANSITIONS'))
        details.addWidget(config_panel)
        details.addWidget(history_panel)
        self.parameters = QTableWidget(0, 2)
        self.parameters.setHorizontalHeaderLabels(['Parameter', 'Value'])
        self.parameters.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeToContents)
        self.parameters.horizontalHeader().setSectionResizeMode(1, QHeaderView.Stretch)
        self.parameters.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.parameters.setAlternatingRowColors(True)
        config_layout.addWidget(self.parameters, 1)
        self.events = QListWidget()
        history_layout.addWidget(self.events, 1)
        details.setStretchFactor(0, 1)
        details.setStretchFactor(1, 1)
        details.setSizes([600, 600])
        vertical.setSizes([480, 320])
        data.mission_updated.connect(self.render)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.check_connection)
        self.timer.start(500)
        if data.mission_status is not None:
            self.render(data.mission_status)

    def check_connection(self):
        if self.last_received is not None and time.monotonic() - self.last_received > 3:
            self.connection.setText('STALE — last known values')
            self.connection.setStyleSheet('color: #ffb74d;')
            self.graph.set_stale(True)

    @staticmethod
    def flatten(values, prefix=''):
        for key, value in values.items():
            name = prefix + key
            if isinstance(value, dict):
                yield from MissionMonitorPage.flatten(value, name + '.')
            elif value is not None:
                if isinstance(value, list):
                    value = ', '.join(map(str, value)) or 'None'
                elif isinstance(value, float):
                    value = f'{value:.3f}'
                yield name, str(value)

    def render(self, status):
        self.last_received = time.monotonic()
        self.connection.setText('LIVE — /mission/status')
        self.connection.setStyleSheet('color: #00c853;')
        objectives = status.get('objectives', [])
        index = status.get('current_index')
        state = status.get('state', 'UNKNOWN')
        self.summary.setText(f"Mission elapsed: {status.get('mission_elapsed_s', 0):.1f} s")
        done = sum(item['status'] == 'COMPLETED' for item in objectives)
        skipped = sum(item['status'] == 'SKIPPED' for item in objectives)
        self.progress.setRange(0, max(1, len(objectives)))
        self.progress.setValue(done + skipped)
        self.progress.setFormat(f'{done} completed / {skipped} skipped / {len(objectives)} total')
        previous = self.last_status or {}
        if objectives != previous.get('objectives'):
            self.sequence.clear()
            for i, objective in enumerate(objectives):
                outcome = objective['status']
                item = QListWidgetItem(f"{i+1:02d}   {objective['name']}   [{outcome}]")
                item.setForeground(QColor(self.COLORS.get(outcome, '#dce6f2')))
                self.sequence.addItem(item)
                if i == index:
                    self.sequence.scrollToItem(item)
        name = objectives[index]['name'] if isinstance(index, int) and 0 <= index < len(objectives) else 'No active objective'
        self.current.setText(f'{name}  |  {state}')
        elapsed = status.get('objective_elapsed_s')
        objective_time = f'{elapsed:.1f} s' if elapsed is not None else '—'
        self.timing.setText(f"State: {status.get('state_elapsed_s', 0):.1f} s   |   Objective: {objective_time}")
        rows = list(self.flatten(status.get('parameters', {}), 'config.'))
        rows += list(self.flatten(status.get('runtime', {}), 'live.'))
        self.parameters.setRowCount(len(rows))
        for i, (key, value) in enumerate(rows):
            for col, text in enumerate((key, value)):
                item = self.parameters.item(i, col)
                if item is None:
                    self.parameters.setItem(i, col, QTableWidgetItem(text))
                elif item.text() != text:
                    item.setText(text)
        events = status.get('events', [])
        self.graph.set_status(state, events[-1] if events else None, status.get('state_elapsed_s', 0))
        if events != previous.get('events'):
            self.events.clear()
            for event in reversed(events):
                self.events.addItem(f"{event['elapsed_s']:7.1f} s   {event['objective']}   {event['transition']} ({event['trigger']})")
        self.last_status = status
