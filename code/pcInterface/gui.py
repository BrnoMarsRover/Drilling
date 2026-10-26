"""PyQt6 interface for the Drilling HMI.

The communication worker and binary drill protocol remain independent of Qt.
"""
import time
import queue
import math
from collections import deque

import numpy as np
import pyqtgraph as pg

import serial.tools.list_ports
from PyQt6.QtCore import Qt, QTimer, QSettings, QPoint, QRect, QEvent
from PyQt6.QtGui import QFont, QAction
from PyQt6.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QLabel,
    QPushButton, QLineEdit, QComboBox, QRadioButton, QButtonGroup,
    QGroupBox, QScrollArea, QPlainTextEdit, QCheckBox, QMessageBox,
    QSplitter, QSizePolicy, QFrame, QApplication, QSpinBox, QMdiArea, QMdiSubWindow, QToolButton, QMenu, QGraphicsDropShadowEffect
)
import protocol

POLL_INTERVAL_MS = 100
STATE_POLL_INTERVAL_MS = 500
DEFAULT_UDP_SEND_PORT = 5610
DEFAULT_UDP_LISTEN_PORT = 5611


class TimeWindowAxis(pg.AxisItem):
    """Display the moving X window in seconds or minutes without changing data."""

    def __init__(self):
        super().__init__(orientation='bottom')
        self.window_seconds = 300

    def set_window(self, seconds):
        self.window_seconds = int(seconds)
        self.setLabel('')
        self.update()

    def tickStrings(self, values, scale, spacing):
        divisor = 60.0 if self.window_seconds > 300 else 1.0
        step = abs(spacing * scale / divisor)
        digits = 0 if step >= 1 else (1 if step >= .1 else 2)
        return [f'{abs(v * scale) / divisor:.{digits}f}' for v in values]


class Value:
    """Minimal get/set adapter to keep protocol-facing handler logic unchanged."""
    def __init__(self, widget):
        self.widget = widget

    def get(self):
        return self.widget.text()

    def set(self, value):
        self.widget.setText(str(value))


class PanelHeader(QFrame):
    """Modern custom title strip; drag with mouse or double-click to maximize."""
    def __init__(self, panel, title):
        super().__init__()
        self.panel = panel
        self.setObjectName('panelHeader')
        self.setFixedHeight(40)
        self._drag_pos = None
        line = QHBoxLayout(self)
        line.setContentsMargins(12, 0, 6, 0)
        line.setSpacing(5)
        dot = QLabel('●')
        dot.setObjectName('panelDot')
        dot.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents, True)
        line.addWidget(dot)
        name = QLabel(title)
        name.setObjectName('panelTitle')
        name.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents, True)
        line.addWidget(name)
        self.extras = QHBoxLayout()
        self.extras.setSpacing(6)
        line.addLayout(self.extras)
        line.addStretch()
        for caption, tip, method in (('—', 'Minimize', self.panel.showMinimized),
                                     ('□', 'Maximize / restore', self.panel.toggle_maximize),
                                     ('×', 'Hide panel', self.panel.close)):
            btn = QToolButton(self)
            btn.setText(caption)
            btn.setToolTip(tip)
            btn.setObjectName('panelClose' if caption == '×' else 'panelControl')
            btn.clicked.connect(method)
            line.addWidget(btn)

    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            if self.panel.isMaximized():
                self.panel.showNormal()
            self._drag_pos = self.panel.parentWidget().mapFromGlobal(event.globalPosition().toPoint()) - self.panel.pos()
            self.panel.raise_()
            event.accept()

    def mouseMoveEvent(self, event):
        if self._drag_pos is not None and event.buttons() & Qt.MouseButton.LeftButton:
            self.panel.move(self.panel.parentWidget().mapFromGlobal(event.globalPosition().toPoint()) - self._drag_pos)
            event.accept()

    def mouseReleaseEvent(self, event):
        self._drag_pos = None
        super().mouseReleaseEvent(event)

    def mouseDoubleClickEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            self.panel.toggle_maximize()
            event.accept()


class PanelResizeHandle(QWidget):
    """A visible-to-the-mouse, transparent grip over a frameless MDI edge.

    Child widgets otherwise swallow hover/mouse events before they reach the
    QMdiSubWindow.  Dedicated grips guarantee that the resize cursor actually
    appears at the position where dragging will work.
    """
    def __init__(self, panel, edges):
        super().__init__(panel)
        self.panel = panel
        self.edges = edges  # (left, right, top, bottom)
        left, right, top, bottom = edges
        if (left or right) and (top or bottom):
            cursor = (Qt.CursorShape.SizeFDiagCursor if left == top
                      else Qt.CursorShape.SizeBDiagCursor)
        elif left or right:
            cursor = Qt.CursorShape.SizeHorCursor
        else:
            cursor = Qt.CursorShape.SizeVerCursor
        self.setCursor(cursor)
        self.setMouseTracking(True)
        self.setToolTip('Drag to resize panel')

    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            self.panel._begin_resize(self.edges, event.globalPosition().toPoint())
            event.accept()
        else:
            super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        if self.panel._resize_origin is not None:
            self.panel._perform_resize(event.globalPosition().toPoint())
            event.accept()
        else:
            super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            self.panel._finish_resize()
            event.accept()
        else:
            super().mouseReleaseEvent(event)


class PanelWindow(QMdiSubWindow):
    """Frameless movable/resizable MDI panel with an application-owned titlebar."""
    EDGE = 7

    def __init__(self, title, content):
        super().__init__()
        self.setWindowTitle(title)
        self.setWindowFlags(Qt.WindowType.SubWindow | Qt.WindowType.FramelessWindowHint)
        self.setMouseTracking(True)
        self._resize_edges = (False, False, False, False)
        self._resize_origin = None
        self._resize_rect = None
        outer = QWidget()
        outer.setObjectName('panelBody')
        layout = QVBoxLayout(outer)
        layout.setContentsMargins(1, 1, 1, 1)
        layout.setSpacing(0)
        self.header = PanelHeader(self, title)
        layout.addWidget(self.header)
        content.setParent(outer)
        layout.addWidget(content, 1)
        self.setWidget(outer)
        self.setMinimumSize(170, 110)
        # Eight transparent overlay grips make the resize cursor reliable even
        # when the content widget completely fills the frameless subwindow.
        self._resize_handles = {}
        edge_sets = {
            'left': (True, False, False, False),
            'right': (False, True, False, False),
            'top': (False, False, True, False),
            'bottom': (False, False, False, True),
            'top_left': (True, False, True, False),
            'top_right': (False, True, True, False),
            'bottom_left': (True, False, False, True),
            'bottom_right': (False, True, False, True),
        }
        for key, edges in edge_sets.items():
            self._resize_handles[key] = PanelResizeHandle(self, edges)
        self._place_resize_handles()

    def toggle_maximize(self):
        self.showNormal() if self.isMaximized() else self.showMaximized()

    def _edges_at(self, pos):
        x, y = pos.x(), pos.y()
        e = self.EDGE
        return (x <= e, x >= self.width() - e, y <= e, y >= self.height() - e)

    def _resize_handle_allowed(self, edges):
        owner = getattr(self, '_owner', None)
        if owner is None or not owner._auto_dashboard:
            return True
        # Dashboard resize is performed on *shared* separators only. Prevent
        # grips on the outer border that would otherwise appear to do nothing.
        if sum(bool(v) for v in edges) > 1:
            return False
        left, right, top, bottom = edges
        name = self.windowTitle()
        if name == 'Drill status':
            return right or bottom
        if name == 'Communication log':
            return left or bottom
        allowed = {
            'Manual controls': (False, True, True, False),
            'Calibration': (True, True, True, True),
            'Weight': (True, True, True, False),
            'Autonomy': (True, True, True, True),
            'Height measurement': (True, True, True, False),
            'Sample handling': (True, True, True, False),
            'Device status': (True, False, True, False),
        }.get(name, (False, False, False, False))
        return any(a and e for a, e in zip(allowed, edges))

    def _place_resize_handles(self):
        if not hasattr(self, '_resize_handles'):
            return
        w, h, e = self.width(), self.height(), 11
        shapes = {
            'left': (0, e, e, max(1, h-2*e)),
            'right': (w-e, e, e, max(1, h-2*e)),
            'top': (e, 0, max(1, w-2*e), e),
            'bottom': (e, h-e, max(1, w-2*e), e),
            'top_left': (0, 0, e, e),
            'top_right': (w-e, 0, e, e),
            'bottom_left': (0, h-e, e, e),
            'bottom_right': (w-e, h-e, e, e),
        }
        for key, handle in self._resize_handles.items():
            handle.setGeometry(*shapes[key])
            if self._resize_handle_allowed(handle.edges) and not self.isMaximized():
                handle.show()
                handle.raise_()
            else:
                handle.hide()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._place_resize_handles()

    def _begin_resize(self, edges, global_pos):
        if self.isMaximized() or not self._resize_handle_allowed(edges):
            return
        self._resize_edges = edges
        self._resize_origin = global_pos
        self._resize_rect = QRect(self.geometry())
        self.raise_()

    def _perform_resize(self, global_pos):
        if self._resize_origin is None:
            return
        delta = global_pos - self._resize_origin
        left, right, top, bottom = self._resize_edges
        r = QRect(self._resize_rect)
        if left: r.setLeft(min(r.right()-self.minimumWidth()+1, r.left()+delta.x()))
        if right: r.setRight(max(r.left()+self.minimumWidth()-1, r.right()+delta.x()))
        if top: r.setTop(min(r.bottom()-self.minimumHeight()+1, r.top()+delta.y()))
        if bottom: r.setBottom(max(r.top()+self.minimumHeight()-1, r.bottom()+delta.y()))
        previous = QRect(self.geometry())
        if hasattr(self, '_owner') and self._owner._resize_neighbor_panels(self, previous, r):
            return
        self.setGeometry(r)

    def _finish_resize(self):
        self._resize_origin = None
        self._resize_rect = None
        if hasattr(self, '_owner'):
            self._owner._flush_dashboard_resize()

    def mouseMoveEvent(self, event):
        # Fallback for subwindow borders not covered by the Qt child widgets.
        if self._resize_origin is not None:
            self._perform_resize(event.globalPosition().toPoint())
            event.accept()
            return
        left, right, top, bottom = self._edges_at(event.position().toPoint())
        edges = (left, right, top, bottom)
        if self._resize_handle_allowed(edges):
            if (left or right) and (top or bottom):
                self.setCursor(Qt.CursorShape.SizeFDiagCursor if left == top else Qt.CursorShape.SizeBDiagCursor)
            elif left or right:
                self.setCursor(Qt.CursorShape.SizeHorCursor)
            elif top or bottom:
                self.setCursor(Qt.CursorShape.SizeVerCursor)
            else:
                self.unsetCursor()
        else:
            self.unsetCursor()
        super().mouseMoveEvent(event)

    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton and not self.isMaximized():
            edges = self._edges_at(event.position().toPoint())
            if any(edges) and self._resize_handle_allowed(edges):
                self._begin_resize(edges, event.globalPosition().toPoint())
                event.accept()
                return
        super().mousePressEvent(event)

    def mouseReleaseEvent(self, event):
        self._finish_resize()
        super().mouseReleaseEvent(event)

    def closeEvent(self, event):
        event.ignore()
        self.hide()
        owner = self._owner
        if hasattr(owner, '_panel_actions'):
            action = owner._panel_actions.get(self.windowTitle())
            if action is not None:
                action.setChecked(False)


class App(QMainWindow):
    _CARRIAGE_TOP_TO_SPIRAL_TIP_MM = 720
    _LIN_AXIS_ZERO_TO_SENSOR_MM = 775

    def __init__(self, worker):
        super().__init__()
        self.worker = worker
        self._last_carriage_depth_mm = None
        self._last_height_above_ground_mm = None
        self.setWindowTitle('Drilling HMI — PyQt6')
        self.resize(1560, 900)
        self._auto_dashboard = False
        self._dashboard_relayout_pending = False
        # Shared Dashboard boundaries. Resizing one panel moves its linked neighbors.
        self._dashboard_bottom_ratio = None  # None uses a compact content-aware default.
        self._dashboard_chart_ratio = .77
        self._dashboard_columns = [.235, .175, .18, .175, .235]
        self._dashboard_calibration_ratio = .60
        self._dashboard_autonomy_ratio = .55
        # Limit relayouts to ~60 Hz while dragging, rather than rebuilding all
        # nine panel geometries on every individual mouse event.
        self._dashboard_resize_timer = QTimer(self)
        self._dashboard_resize_timer.setSingleShot(True)
        self._dashboard_resize_timer.setInterval(16)
        self._dashboard_resize_timer.timeout.connect(self._flush_dashboard_resize)
        self._pending_dashboard_scope = None
        self.setMinimumSize(900, 640)
        self._plot_started = time.monotonic()
        self._samples = deque(maxlen=7200)  # 60 min at 500 ms
        self._theme = str(QSettings('DrillingHMI', 'DrillingHMI').value('theme', 'dark')).lower()
        if self._theme not in ('dark', 'light'):
            self._theme = 'dark'
        self._build_ui()
        self._apply_style()
        self._refresh_ports()
        self._on_mode_change()
        self.rx_timer = QTimer(self)
        self.rx_timer.timeout.connect(self._poll_rx)
        self.rx_timer.start(POLL_INTERVAL_MS)
        self.state_timer = QTimer(self)
        self.state_timer.timeout.connect(self._poll_state)
        self.state_timer.start(STATE_POLL_INTERVAL_MS)

    def _apply_style(self):
        dark_qss = '''
            QMainWindow, QWidget#central { background: #111a29; color: #e5eaf3; }
            QGroupBox { border: none; border-radius: 7px;
                        margin-top: 0px; padding: 0px; font-weight: 650;
                        background: #1b2739; color: #dbeafe; }
            QGroupBox::title { color: transparent; }
            QLabel { color: #d3deee; }
            QLabel#readout { font-weight: 650; color: #8cdbff; }
            QLineEdit, QComboBox { background: #101a29; color: #ecf4ff;
                     border: 1px solid #445570; border-radius: 5px; padding: 5px; min-height: 24px; }
            QLineEdit:focus, QComboBox:focus { border: 1px solid #45a3f8; }
            QPushButton { border-radius: 6px; background: #304561; color: #f2f6fc;
                          border: 1px solid #435c7d; padding: 7px 9px; font-weight: 550; }
            QPushButton:hover { background: #3c5a7e; }
            QPushButton:pressed { background: #233850; }
            QPushButton:disabled { color: #77869e; background: #233145; }
            QPushButton#primary { background: #1666a5; border-color: #2485c8; }
            QPushButton#danger { background: #853438; border-color: #ab494d; }
            QPlainTextEdit { background: #0d1522; color: #bfe1f4;
                        border: 1px solid #344359; border-radius: 6px; padding: 6px; }
            QScrollArea { border: 0; background: transparent; }
            QScrollBar:vertical { background: #162338; width: 11px; }
            QScrollBar::handle:vertical { background: #415572; border-radius: 4px; min-height: 28px; }
            QSplitter::handle { background: #172234; width: 5px; }
            QMdiArea#workspace { border: 1px solid #25354a; border-radius: 12px; }
            QMdiSubWindow { background: #172438; border: 1px solid #3a5069;
                            border-radius: 10px; }
            QWidget#panelBody { background: #18273a; border-radius: 10px; }
            QFrame#panelHeader { background: #233650; border-top-left-radius: 9px;
                                 border-top-right-radius: 9px; }
            QLabel#panelTitle { color: #f2f6fd; font-weight: 700; font-size: 13px; }
            QLabel#panelDot { color: #4caaf8; font-size: 12px; }
            QToolButton#panelControl, QToolButton#panelClose {
                border: none; background: transparent; color: #b8c9de;
                border-radius: 5px; min-width: 27px; max-width: 27px;
                min-height: 26px; max-height: 26px; font-size: 16px;
            }
            QToolButton#panelControl:hover { background: #3a526d; color: white; }
            QToolButton#panelClose:hover { background: #b33b4c; color: white; }
            QToolButton#navButton { background: #253b58; border: 1px solid #46617f;
                border-radius: 7px; color: #eaf2ff; padding: 8px 14px; font-weight: 650; }
            QToolButton#navButton:hover, QToolButton#navButton:checked { background: #34638a; }
            QMenu { background: #20324b; color: #eef5fe; border: 1px solid #49617d; padding: 6px; }
            QMenu::item { padding: 8px 19px; border-radius: 5px; }
            QMenu::item:selected { background: #345d85; }
            QMenu::indicator:checked { background: #45a4ed; }

        '''
        if self._theme == 'dark':
            qss = dark_qss
        else:
            # Consistent light palette while retaining semantic primary/danger buttons.
            replacements = {
                '#111a29':'#f0f4f9', '#e5eaf3':'#172a41',
                '#1b2739':'#ffffff', '#dbeafe':'#263f59',
                '#d3deee':'#33465b', '#8cdbff':'#096caa',
                '#101a29':'#f8fafc', '#ecf4ff':'#20334c',
                '#445570':'#a3b4c9', '#45a3f8':'#1678ca',
                '#304561':'#e4eaf3', '#f2f6fc':'#213750',
                '#435c7d':'#afbdd0', '#3c5a7e':'#d7e4f3',
                '#233850':'#c6d7e8', '#77869e':'#8090a4',
                '#233145':'#e4e9f0', '#0d1522':'#ffffff',
                '#bfe1f4':'#263f59', '#344359':'#c5d0df',
                '#162338':'#ecf0f5', '#415572':'#acbbcc',
                '#172234':'#d5e0ec', '#25354a':'#b6c7da',
                '#172438':'#ffffff', '#3a5069':'#bdcbd9',
                '#18273a':'#f8fafc', '#233650':'#e0e9f4',
                '#f2f6fd':'#243c58', '#4caaf8':'#238ad1',
                '#b8c9de':'#45617f', '#3a526d':'#d1dfec',
                '#253b58':'#e4edf8', '#46617f':'#a9bed3',
                '#eaf2ff':'#1e3952', '#34638a':'#d0e4f5',
                '#20324b':'#ffffff', '#eef5fe':'#263c55',
                '#49617d':'#b8c9dc', '#345d85':'#d6eaf9'
            }
            qss = dark_qss
            for old, new in replacements.items():
                qss = qss.replace(old, f'__CLR_{old[1:]}__')
            for old, new in replacements.items():
                qss = qss.replace(f'__CLR_{old[1:]}__', new)
        if self._theme == 'light':
            qss += """
                QPushButton { color: #14283f; background: #e9eff6; border: 1px solid #a5b6cb; font-weight: 650; }
                QPushButton:hover { background: #d4e5f6; color: #10263f; }
                QPushButton:pressed { background: #bdd6f0; }
                QPushButton#primary { background: #1766a6; border-color: #12598f; color: #ffffff; }
                QPushButton#primary:hover { background: #115588; color: #ffffff; }
                QPushButton#danger { background: #aa3745; border-color: #8c2835; color: #ffffff; }
                QPushButton#danger:hover { background: #912b38; color: #ffffff; }
                QToolButton#navButton { color: #18334e; background: #ffffff; border-color: #adbfd2; }
                QToolButton#navButton:hover { background: #dcebf8; }
                QWidget#panelBody { background: #f8fafc; }
                QGroupBox { background: #f8fafc; }
                QFrame#panelHeader { background: #dae5f0; }
                QPlainTextEdit { background: #ffffff; color: #1e3046; }
            """
        self.setStyleSheet(qss)
        if hasattr(self, 'workspace'):
            self.workspace.setBackground(pg.mkColor('#111a29' if self._theme == 'dark' else '#f0f4f9'))
        if hasattr(self, 'chart'):
            self.chart.setBackground('#101a29' if self._theme == 'dark' else '#ffffff')
            self.chart.getPlotItem().getAxis('bottom').setTextPen('#aabbd0' if self._theme == 'dark' else '#425a72')
            if hasattr(self, 'unit_name_labels'):
                for unit, label in self.unit_name_labels.items():
                    label.setStyleSheet(f'color: {self.unit_colors[unit]}; font-size: 10px; font-weight: 700; background: transparent;')
        if hasattr(self, 'settings_container'):
            self.settings_container.setStyleSheet('''
                QWidget#connectionPopup { background: %s; border: 1px solid %s; border-radius: 10px; }
            ''' % (('#1b2739', '#4a678b') if self._theme == 'dark' else ('#ffffff', '#afbdd0')))
        if hasattr(self, 'theme_btn'):
            self.theme_btn.setText('☀' if self._theme == 'dark' else '☾')
            self.theme_btn.setToolTip('Switch to light mode' if self._theme == 'dark' else 'Switch to dark mode')

    def _toggle_theme(self):
        self._theme = 'light' if self._theme == 'dark' else 'dark'
        QSettings('DrillingHMI', 'DrillingHMI').setValue('theme', self._theme)
        self._apply_style()

    def _build_ui(self):
        central = QWidget()
        central.setObjectName('central')
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(9, 8, 9, 8)
        root.setSpacing(6)

        # Compact, permanently visible navigation; connection controls collapse.
        toolbar = QHBoxLayout()
        self.settings_btn = QToolButton()
        self.settings_btn.setText('◉  Connection  ▾')
        self.settings_btn.setCheckable(True)
        self.settings_btn.setToolButtonStyle(Qt.ToolButtonStyle.ToolButtonTextOnly)
        self.settings_btn.toggled.connect(self._toggle_settings)
        self.settings_btn.setObjectName('navButton')
        toolbar.addWidget(self.settings_btn)
        self.windows_btn = QToolButton()
        self.windows_btn.setText('▦  Windows  ▾')
        self.windows_btn.setObjectName('navButton')
        self.windows_btn.setPopupMode(QToolButton.ToolButtonPopupMode.InstantPopup)
        self.windows_menu = QMenu(self.windows_btn)
        self.windows_btn.setMenu(self.windows_menu)
        toolbar.addWidget(self.windows_btn)
        toolbar.addStretch(1)
        self.header_connection = QLabel('● Disconnected')
        self.header_connection.setStyleSheet('color: #ff777c; font-weight: 650;')
        toolbar.addWidget(self.header_connection)
        self.theme_btn = QToolButton()
        self.theme_btn.setObjectName('navButton')
        self.theme_btn.setText('☀' if self._theme == 'dark' else '☾')
        self.theme_btn.setToolTip('Switch between dark and light mode')
        self.theme_btn.setFixedSize(44, 38)
        self.theme_btn.clicked.connect(self._toggle_theme)
        toolbar.addWidget(self.theme_btn)
        root.addLayout(toolbar)

        self.settings_container = QWidget()
        settings_layout = QVBoxLayout(self.settings_container)
        settings_layout.setContentsMargins(0, 0, 0, 0)
        self._build_connection_bar(settings_layout)
        # Floating popover: NOT in the root layout, so opening it never moves the workspace.
        self.settings_container.setParent(central)
        self.settings_container.setObjectName('connectionPopup')
        self.settings_container.setWindowFlags(Qt.WindowType.Widget)
        self.settings_container.setStyleSheet('''
            QWidget#connectionPopup { background: #1b2739; border: 1px solid #4a678b;
                border-radius: 10px; }
        ''')
        self.settings_container.hide()
        QApplication.instance().installEventFilter(self)

        self.workspace = QMdiArea(self)
        self.workspace.setObjectName('workspace')
        self.workspace.setViewMode(QMdiArea.ViewMode.SubWindowView)
        self.workspace.setOption(QMdiArea.AreaOption.DontMaximizeSubWindowOnActivation, True)
        self.workspace.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)
        self.workspace.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)
        self.workspace.setBackground(pg.mkColor('#111a29'))
        root.addWidget(self.workspace, 1)
        self._panels = {}
        self._panel_actions = {}

        status = QWidget()
        status_layout = QVBoxLayout(status)
        status_layout.setContentsMargins(2, 2, 2, 2)
        self._build_status_panel(status_layout)
        self._register_panel('Drill status', status, (8, 8, 1090, 410))

        self._build_manual_controls()
        self._build_height_panel()
        self._build_autonomy_controls()
        self._build_sample_controls()
        self._build_device_status_panel()
        self._build_weight_panel()
        self._build_calibration_panel()
        self._build_log()

        self.windows_menu.addSeparator()
        self.windows_menu.addAction('▣  Dashboard layout', self._reset_dashboard_layout)
        self.windows_menu.addAction('Reset layout', self._reset_panel_layout)
        self.windows_menu.addAction('Show all windows', self._show_all_panels)
        self._restore_panel_layout()
        # Apply the new dashboard once when migrating from earlier layouts.
        settings = QSettings('DrillingHMI', 'DrillingHMI')
        if settings.value('dashboard_layout_version', 0, type=int) < 13:
            QTimer.singleShot(0, lambda: self._apply_preset('dashboard'))
            settings.setValue('dashboard_layout_version', 13)

    def _toggle_settings(self, show):
        if show:
            self._position_connection_popup()
            self.settings_container.show()
            self.settings_container.raise_()
        else:
            self.settings_container.hide()

    def _position_connection_popup(self):
        popup = self.settings_container
        parent = self.centralWidget()
        popup.adjustSize()
        popup_w = min(max(popup.sizeHint().width(), 610), max(300, parent.width() - 20))
        popup_h = popup.sizeHint().height()
        anchor = self.settings_btn.mapTo(parent, QPoint(0, self.settings_btn.height() + 7))
        x = max(8, min(anchor.x(), parent.width() - popup_w - 8))
        popup.setGeometry(x, anchor.y(), popup_w, popup_h)

    def _hide_connection_popup(self):
        self.settings_btn.blockSignals(True)
        self.settings_btn.setChecked(False)
        self.settings_btn.blockSignals(False)
        self.settings_container.hide()

    def eventFilter(self, obj, event):
        if event.type() == QEvent.Type.MouseButtonPress and self.settings_container.isVisible():
            p = event.globalPosition().toPoint()
            popup = self.settings_container
            popup_area = QRect(popup.mapToGlobal(QPoint(0, 0)), popup.size())
            button_area = QRect(self.settings_btn.mapToGlobal(QPoint(0, 0)), self.settings_btn.size())
            if not popup_area.contains(p) and not button_area.contains(p):
                self._hide_connection_popup()
        if hasattr(self, 'chart') and obj is self.chart and event.type() == QEvent.Type.Resize:
            QTimer.singleShot(0, self._position_axis_units)
        return super().eventFilter(obj, event)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        if hasattr(self, 'settings_container') and self.settings_container.isVisible():
            self._position_connection_popup()
        if getattr(self, '_auto_dashboard', False) and hasattr(self, 'workspace'):
            QTimer.singleShot(0, self._resize_dashboard)

    def _resize_dashboard(self):
        if self._auto_dashboard and hasattr(self, '_panels') and self._panels:
            self._apply_preset('dashboard')

    def _register_panel(self, title, content, geometry):
        if isinstance(content, QGroupBox):
            content.setTitle('')  # The custom header already displays this title.
        win = PanelWindow(title, content)
        if title in ('Height measurement', 'Weight'):
            win.setMinimumSize(170, 76)
        elif title == 'Communication log':
            win.setMinimumSize(200, 190)
        elif title == 'Drill status':
            win.setMinimumSize(440, 250)
        elif title == 'Calibration':
            win.setMinimumSize(185, 155)
        elif title in ('Sample handling', 'Manual controls'):
            win.setMinimumSize(190, 200)
        elif title == 'Device status':
            win.setMinimumSize(195, 200)
        else:
            win.setMinimumSize(180, 185)
        win._owner = self
        win._place_resize_handles()
        win.setAttribute(Qt.WidgetAttribute.WA_DeleteOnClose, False)
        self.workspace.addSubWindow(win)
        if title == 'Drill status' and hasattr(self, '_chart_toolbar'):
            win.header.extras.addWidget(self._chart_toolbar)
        x, y, w, h = geometry
        win.setGeometry(x, y, w, h)
        win.show()
        action = QAction(title, self)
        action.setCheckable(True)
        action.setChecked(True)
        action.toggled.connect(lambda enabled, key=title: self._show_panel(key, enabled))
        self.windows_menu.addAction(action)
        self._panel_actions[title] = action
        self._panels[title] = (win, geometry)
        return win

    def _schedule_dashboard_resize(self, scope='all'):
        if self._pending_dashboard_scope != 'all':
            self._pending_dashboard_scope = scope
        if not self._dashboard_resize_timer.isActive():
            self._dashboard_resize_timer.start()

    def _flush_dashboard_resize(self):
        if not getattr(self, '_dashboard_resize_timer', None):
            return
        if self._dashboard_resize_timer.isActive():
            self._dashboard_resize_timer.stop()
        scope = self._pending_dashboard_scope
        self._pending_dashboard_scope = None
        if scope is None or not self._auto_dashboard:
            return
        if scope == 'top':
            # The common graph/log divider only needs TWO geometry changes.
            # Avoid expensive updates of the seven lower controls while
            # dragging this particularly frequently used separator.
            gap = 7
            w = max(1, self.workspace.viewport().width())
            available = w - 3*gap
            chart_w = round(available * self._dashboard_chart_ratio)
            chart = self._panels['Drill status'][0]
            log = self._panels['Communication log'][0]
            chart.setGeometry(gap, gap, chart_w, chart.height())
            log.setGeometry(chart_w+2*gap, gap, round(available-chart_w), log.height())
        else:
            self._apply_preset('dashboard')

    def _sync_resize_handles(self):
        for win, _ in self._panels.values():
            win._place_resize_handles()

    def _resize_neighbor_panels(self, resized, old, new):
        """Resize the *shared* Dashboard boundaries, rather than a single panel.

        The top pair have one common height and one adjustable width separator.
        All bottom columns start at that pair's lower edge and share a bottom edge.
        Stacked panels may adjust their own internal divider. Resizing either
        panel in a shared boundary updates *every* panel in that partition.

        Returns True if the mouse resize was consumed by the Dashboard layout.
        """
        if not getattr(self, '_auto_dashboard', False):
            return False
        name = resized.windowTitle()
        area = self.workspace.viewport().size()
        w, h, gap = max(1, area.width()), max(1, area.height()), 7
        if h < 1 or w < 1:
            return True
        top_names = {'Drill status', 'Communication log'}
        bottom_columns = [
            ('Manual controls',), ('Calibration', 'Weight'),
            ('Autonomy', 'Height measurement'), ('Sample handling',),
            ('Device status',)
        ]
        cols = {panel: idx for idx, column in enumerate(bottom_columns) for panel in column}
        min_bottom = min(270, max(195, h - 320))
        min_top = min(320, max(190, h - min_bottom - 21))
        modified = False
        scope = 'all'

        # The common horizontal separator connects BOTH graph and log to ALL
        # the lower panels. Never move one of those edges in isolation.
        if name in top_names and new.bottom() != old.bottom():
            new_top_h = new.height()
            desired_bottom = h - new_top_h - 3 * gap
            desired_bottom = max(min_bottom, min(h - 3*gap - min_top, desired_bottom))
            self._dashboard_bottom_ratio = desired_bottom / h
            modified = True
        elif name in cols and new.top() != old.top():
            index = cols[name]
            if (index in (1, 2) and name in ('Weight', 'Height measurement')):
                # Bottom cell's top edge is the within-column divider, not the
                # full dashboard boundary.
                column_top = self._panels['Manual controls'][0].geometry().top()
                column_h = self._panels['Manual controls'][0].height()
                ratio = (new.top() - column_top - gap) / max(1, column_h-gap)
                key = '_dashboard_calibration_ratio' if index == 1 else '_dashboard_autonomy_ratio'
                setattr(self, key, max(.42, min(.75, ratio)))
            else:
                # Dragging the top edge of a full-height bottom panel moves the
                # horizontal split for every Dashboard column at once.
                desired_bottom = h - new.top() - gap
                desired_bottom = max(min_bottom, min(h - 3*gap - min_top, desired_bottom))
                self._dashboard_bottom_ratio = desired_bottom / h
            modified = True
        elif name in ('Calibration', 'Autonomy') and new.bottom() != old.bottom():
            # Resize the internal stack: Weight / Height fills the remainder.
            column_h = self._panels['Manual controls'][0].height()
            ratio = new.height() / max(1, column_h - gap)
            key = '_dashboard_calibration_ratio' if name == 'Calibration' else '_dashboard_autonomy_ratio'
            setattr(self, key, max(.42, min(.75, ratio)))
            modified = True

        if not modified and name in top_names:
            # One vertical separator for chart and log, always the same height.
            if name == 'Drill status' and new.right() != old.right():
                available = w - 3 * gap
                # Honor real minimum widths of BOTH panels at small resolutions.
                min_ratio = max(.50, self._panels['Drill status'][0].minimumWidth() / max(1, available))
                max_ratio = min(.85, 1 - self._panels['Communication log'][0].minimumWidth() / max(1, available))
                self._dashboard_chart_ratio = max(min_ratio, min(max_ratio, new.width()/max(1, available)))
                modified = True
                scope = 'top' 
            elif name == 'Communication log' and new.left() != old.left():
                available = w - 3 * gap
                width_chart = new.left() - 2 * gap
                min_ratio = max(.50, self._panels['Drill status'][0].minimumWidth() / max(1, available))
                max_ratio = min(.85, 1 - self._panels['Communication log'][0].minimumWidth() / max(1, available))
                self._dashboard_chart_ratio = max(min_ratio, min(max_ratio, width_chart/max(1, available)))
                modified = True
                scope = 'top' 

        if not modified and name in cols:
            # All panels belonging to the same bottom column share its X range;
            # dragging a column boundary also changes the adjacent column.
            idx = cols[name]
            boundary = None
            delta = 0
            if new.right() != old.right() and idx < 4:
                boundary = idx
                delta = new.right() - old.right()
            elif new.left() != old.left() and idx > 0:
                boundary = idx - 1
                delta = new.left() - old.left()
            if boundary is not None:
                fractions = list(self._dashboard_columns)
                effective = max(1, w - 6 * gap)
                advance = delta / effective
                left_index, right_index = boundary, boundary + 1
                # Keep both panels usable and their shared boundary touching.
                minimum = max(.105, 145 / effective)
                advance = max(minimum - fractions[left_index],
                              min(fractions[right_index] - minimum, advance))
                fractions[left_index] += advance
                fractions[right_index] -= advance
                self._dashboard_columns = fractions
                modified = True

        if modified:
            self._schedule_dashboard_resize(scope)
            return True
        # Dashboard uses fixed shared edges; letting an independent drag through
        # would break alignment. Ignore unsupported external-edge drags.
        return True

    def _show_panel(self, title, enabled):
        win = self._panels[title][0]
        if enabled:
            win.showNormal()
            win.show()
            self.workspace.setActiveSubWindow(win)
        else:
            win.hide()

    def _show_all_panels(self):
        for action in self._panel_actions.values():
            action.setChecked(True)

    def _reset_dashboard_layout(self):
        """Restore factory Dashboard geometry, ignoring previously dragged dividers."""
        self._dashboard_resize_timer.stop()
        self._pending_dashboard_scope = None
        self._dashboard_bottom_ratio = None
        self._dashboard_chart_ratio = .77
        self._dashboard_columns = [.235, .175, .18, .175, .235]
        self._dashboard_calibration_ratio = .60
        self._dashboard_autonomy_ratio = .55
        self._show_all_panels()
        # The viewport may resize when hidden windows are restored. Defer once,
        # so the preset is computed using its final size, not the previous one.
        self._apply_preset('dashboard')
        QTimer.singleShot(0, lambda: self._apply_preset('dashboard')
                          if self._auto_dashboard else None)

    def _reset_panel_layout(self):
        """Show all nine panels at individually sized, compact dimensions."""
        self._dashboard_resize_timer.stop()
        self._pending_dashboard_scope = None
        self._show_all_panels()
        self._apply_preset('compact')
        QTimer.singleShot(0, lambda: self._apply_preset('compact')
                          if not self._auto_dashboard else None)

    def _apply_preset(self, preset):
        """Arrange currently visible panels in a snapped, gap-based workspace grid."""
        names = [name for name in self._panels if self._panel_actions[name].isChecked()]
        if not names:
            self._show_all_panels()
            names = list(self._panels)
        for name in names:
            win = self._panels[name][0]
            if win.isMaximized() or win.isMinimized():
                win.showNormal()
        area = self.workspace.viewport().size()
        # Fit the dashboard to the ACTUAL viewport, not a synthetic 1750x1370
        # scene that puts all of the lower panels below the visible window.
        w, h = max(1, area.width()), max(1, area.height())
        gap = 7

        def place(name, x, y, width, height):
            if name in names:
                win = self._panels[name][0]
                win.setGeometry(round(x), round(y), round(width), round(height))
                win.show()

        def grid(items, x, y, width, height, columns):
            if not items: return
            rows = math.ceil(len(items) / columns)
            cw = (width - gap * (columns - 1)) / columns
            ch = (height - gap * (rows - 1)) / rows
            for i, name in enumerate(items):
                place(name, x + (i % columns)*(cw + gap), y + (i // columns)*(ch + gap), cw, ch)

        if preset == 'dashboard':
            self._auto_dashboard = True
            # Use the available viewport instead of forcing a tall scrollable
            # scene. The upper chart and log get ALL height not needed by the
            # compact control row. Their top and bottom edges are identical.
            default_bottom = min(max(268, h * .27), max(200, h - 310))
            min_bottom = min(270, max(195, h - 320))
            min_top = min(320, max(190, h - min_bottom - 3*gap))
            bottom_h = (default_bottom if self._dashboard_bottom_ratio is None
                        else h * self._dashboard_bottom_ratio)
            bottom_h = max(min_bottom, min(h - 3*gap - min_top, bottom_h))
            top_h = h - bottom_h - 3*gap
            bottom_y = gap + top_h + gap

            usable_top = w - 3*gap
            chart_min = self._panels['Drill status'][0].minimumWidth()
            log_min = self._panels['Communication log'][0].minimumWidth()
            chart_w = min(usable_top - log_min,
                          max(chart_min, usable_top * self._dashboard_chart_ratio))
            place('Drill status', gap, gap, chart_w, top_h)
            place('Communication log', chart_w + 2*gap, gap,
                  usable_top - chart_w, top_h)

            usable_w = w - 6*gap
            widths = [usable_w * fraction for fraction in self._dashboard_columns]
            xs = []
            x = gap
            for width in widths:
                xs.append(x)
                x += width + gap
            # Full-height lower panels have exactly the same Y and height.
            place('Manual controls', xs[0], bottom_y, widths[0], bottom_h)
            upper_cal = round((bottom_h-gap) * self._dashboard_calibration_ratio)
            place('Calibration', xs[1], bottom_y, widths[1], upper_cal)
            place('Weight', xs[1], bottom_y+upper_cal+gap, widths[1], bottom_h-upper_cal-gap)
            upper_auto = round((bottom_h-gap) * self._dashboard_autonomy_ratio)
            place('Autonomy', xs[2], bottom_y, widths[2], upper_auto)
            place('Height measurement', xs[2], bottom_y+upper_auto+gap,
                  widths[2], bottom_h-upper_auto-gap)
            place('Sample handling', xs[3], bottom_y, widths[3], bottom_h)
            place('Device status', xs[4], bottom_y, widths[4], bottom_h)
        elif preset == 'compact':
            self._auto_dashboard = False
            # Genuine minimum-size reset: use small, CONTENT-SPECIFIC window
            # sizes, never stretch all windows to fill the entire viewport.
            # Columns are packed vertically with only a narrow gap between
            # panels. Scrollbars are available on genuinely small monitors.
            compact_sizes = {
                'Drill status': (645, 425),
                'Communication log': (345, 215),
                'Weight': (295, 110),
                'Manual controls': (390, 250),
                'Sample handling': (345, 230),
                'Height measurement': (320, 120),
                'Device status': (370, 265),
                'Calibration': (350, 190),
                'Autonomy': (335, 165),
            }
            compact_columns = (
                ('Drill status', 'Communication log', 'Weight'),
                ('Manual controls', 'Sample handling', 'Height measurement'),
                ('Device status', 'Calibration', 'Autonomy'),
            )
            x = gap
            for column in compact_columns:
                y = gap
                for name in column:
                    cw, ch = compact_sizes[name]
                    place(name, x, y, cw, ch)
                    y += ch + gap
                x += max(compact_sizes[name][0] for name in column) + gap
        elif preset == 'chart' and 'Drill status' in names:
            remaining = [name for name in names if name != 'Drill status']
            chart_w = w * .73
            place('Drill status', gap, gap, chart_w-gap, h-2*gap)
            grid(remaining, chart_w+gap, gap, w-chart_w-2*gap, h-2*gap, 1)
        elif preset == 'two':
            grid(names, gap, gap, w-2*gap, h-2*gap, 2)
        elif preset == 'three':
            grid(names, gap, gap, w-2*gap, h-2*gap, 3)
        else:
            grid(names, gap, gap, w-2*gap, h-2*gap, max(1, math.ceil(math.sqrt(len(names)))))
        self._sync_resize_handles()

    def _restore_panel_layout(self):
        settings = QSettings('DrillingHMI', 'DrillingHMI')
        saved = settings.value('main_geometry')
        if saved is not None:
            self.restoreGeometry(saved)
        for title, (win, _) in self._panels.items():
            rect = settings.value('panels/' + title + '/geometry')
            if rect is not None and rect.width() >= 225 and rect.height() >= 150:
                win.setGeometry(rect)
            if str(settings.value('panels/' + title + '/visible', 'true')).lower() == 'false':
                self._panel_actions[title].setChecked(False)

    def _group(self, column, title):
        group = QGroupBox(title)
        layout = QGridLayout(group)
        layout.setContentsMargins(9, 9, 9, 9)
        layout.setHorizontalSpacing(7)
        layout.setVerticalSpacing(7)
        initial = {
            'Manual controls': (10, 432, 410, 310),
            'Height measurement': (425, 432, 340, 195),
            'Autonomy': (770, 432, 330, 250),
            'Sample handling': (1110, 435, 330, 290),
            'Device status': (1445, 8, 350, 390),
            'Weight': (1445, 404, 330, 155),
            'Calibration': (1445, 564, 330, 270),
        }
        self._register_panel(title, group, initial[title])
        return layout

    @staticmethod
    def _label(text):
        return QLabel(text)

    def _readout(self, layout, row, caption, attr, unit='', initial='—'):
        layout.addWidget(self._label(caption + ':'), row, 0)
        label = QLabel(initial)
        label.setObjectName('readout')
        label.setMinimumWidth(68)
        label.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        setattr(self, attr, Value(label))
        layout.addWidget(label, row, 1)
        if unit:
            layout.addWidget(self._label(unit), row, 2)
        return label

    def _entry(self, layout, row, caption, attr, default):
        layout.addWidget(self._label(caption), row, 0)
        edit = QLineEdit(str(default))
        edit.setMaximumWidth(110)
        setattr(self, attr, Value(edit))
        layout.addWidget(edit, row, 1)
        return edit

    def _button(self, layout, text, callback, row, col=0, col_span=1, danger=False, primary=False):
        btn = QPushButton(text)
        if danger:
            btn.setObjectName('danger')
        elif primary:
            btn.setObjectName('primary')
        btn.clicked.connect(callback)
        layout.addWidget(btn, row, col, 1, col_span)
        return btn

    def _equal_button_row(self, layout, row, specs, columns, **kwargs):
        """A two-button row whose text cannot skew the column widths."""
        container = QWidget()
        strip = QHBoxLayout(container)
        strip.setContentsMargins(0, 0, 0, 0)
        strip.setSpacing(7)
        buttons = []
        for text, callback, flavor in specs:
            btn = QPushButton(text)
            if flavor:
                btn.setObjectName(flavor)
            btn.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Preferred)
            btn.setMinimumWidth(0)
            btn.clicked.connect(callback)
            strip.addWidget(btn, 1)
            buttons.append(btn)
        layout.addWidget(container, row, 0, 1, columns)
        return buttons

    def _separator(self, layout, row, span=4):
        line = QFrame()
        line.setFrameShape(QFrame.Shape.HLine)
        line.setStyleSheet('background: #35455b; max-height: 1px;')
        layout.addWidget(line, row, 0, 1, span)

    def _build_connection_bar(self, root):
        group = QGroupBox('Connection')
        root.addWidget(group)
        layout = QHBoxLayout(group)
        layout.setContentsMargins(12, 18, 12, 10)
        self.serial_radio = QRadioButton('USB Serial')
        self.udp_radio = QRadioButton('UDP (localhost)')
        self.serial_radio.setChecked(True)
        self.mode_group = QButtonGroup(self)
        self.mode_group.addButton(self.serial_radio)
        self.mode_group.addButton(self.udp_radio)
        self.serial_radio.toggled.connect(self._on_mode_change)
        layout.addWidget(self.serial_radio)
        layout.addWidget(self.udp_radio)
        self.serial_frame = QWidget()
        serial_lay = QHBoxLayout(self.serial_frame)
        serial_lay.setContentsMargins(0, 0, 0, 0)
        serial_lay.addWidget(self._label('COM port:'))
        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(92)
        serial_lay.addWidget(self.port_combo)
        self.refresh_btn = QPushButton('Refresh')
        self.refresh_btn.clicked.connect(self._refresh_ports)
        serial_lay.addWidget(self.refresh_btn)
        layout.addWidget(self.serial_frame)
        self.udp_frame = QWidget()
        udp_lay = QHBoxLayout(self.udp_frame)
        udp_lay.setContentsMargins(0, 0, 0, 0)
        udp_lay.addWidget(self._label('Listen:'))
        self.udp_local_port = QLineEdit(str(DEFAULT_UDP_LISTEN_PORT))
        self.udp_local_port.setMaximumWidth(70)
        udp_lay.addWidget(self.udp_local_port)
        udp_lay.addWidget(self._label('Send:'))
        self.udp_remote_port = QLineEdit(str(DEFAULT_UDP_SEND_PORT))
        self.udp_remote_port.setMaximumWidth(70)
        udp_lay.addWidget(self.udp_remote_port)
        layout.addWidget(self.udp_frame)
        layout.addStretch(1)
        self.connect_btn = QPushButton('Connect')
        self.connect_btn.setObjectName('primary')
        self.connect_btn.clicked.connect(self._toggle_connection)
        layout.addWidget(self.connect_btn)
        self.conn_status = QLabel('● Disconnected')
        self.conn_status.setStyleSheet('color: #ff777c;')
        layout.addWidget(self.conn_status)

    def _on_mode_change(self):
        serial_mode = self.serial_radio.isChecked()
        self.serial_frame.setVisible(serial_mode)
        self.udp_frame.setVisible(not serial_mode)

    def _set_mode_controls_enabled(self, enabled):
        for w in (self.serial_radio, self.udp_radio, self.port_combo,
                  self.refresh_btn, self.udp_local_port, self.udp_remote_port):
            w.setEnabled(enabled)

    def _refresh_ports(self):
        current = self.port_combo.currentText()
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo.clear()
        self.port_combo.addItems(ports)
        if current in ports:
            self.port_combo.setCurrentText(current)

    def _sync_connection_ui(self):
        connected = self.worker.is_connected()
        self.connect_btn.setText('Disconnect' if connected else 'Connect')
        self.connect_btn.setObjectName('danger' if connected else 'primary')
        self.connect_btn.style().unpolish(self.connect_btn)
        self.connect_btn.style().polish(self.connect_btn)
        self._set_mode_controls_enabled(not connected)
        if not connected:
            self.conn_status.setText('● Disconnected')
            self.conn_status.setStyleSheet('color: #ff777c;')
            self.header_connection.setText('● Disconnected')
            self.header_connection.setStyleSheet('color: #ff777c; font-weight: 650;')

    def _toggle_connection(self):
        if self.worker.is_connected():
            self.worker.disconnect()
            self._sync_connection_ui()
            self._log('Disconnected.')
            return
        if self.serial_radio.isChecked():
            port = self.port_combo.currentText()
            if not port:
                QMessageBox.warning(self, 'Error', 'No COM port selected.')
                return
            ok = self.worker.connect_serial(port)
            description = port
        else:
            try:
                local_port = int(self.udp_local_port.text())
                remote_port = int(self.udp_remote_port.text())
            except ValueError:
                QMessageBox.warning(self, 'Error', 'UDP ports must be numbers.')
                return
            if not (0 < local_port <= 65535 and 0 < remote_port <= 65535):
                QMessageBox.warning(self, 'Error', 'UDP ports must be between 1 and 65535.')
                return
            if local_port == remote_port:
                QMessageBox.warning(self, 'Error', 'Listen port and send port must be different.')
                return
            ok = self.worker.connect_udp(local_port, remote_port)
            description = f'UDP — listen {local_port} / send {remote_port}'
        self._sync_connection_ui()
        if ok:
            self.conn_status.setText(f'● Connected ({description})')
            self.conn_status.setStyleSheet('color: #64d9a0;')
            self.header_connection.setText('● Connected: ' + description)
            self.header_connection.setStyleSheet('color: #64d9a0; font-weight: 650;')
            self._log(f'Connected: {description}.')
        else:
            self._log(f'Failed to connect: {description}.')

    # Graph colors are deliberately unique and stable across sessions.
    STATUS_FIELDS = [
        ('Carriage depth', 'carriage_depth_var', 'mm', '#40c9ff', True),
        ('Depth under rover', 'depth_under_rover_var', 'mm', '#58e89a', True),
        ('Depth under surface', 'depth_under_surface_var', 'mm', '#ffbd69', True),
        ('Vertical speed', 'vert_speed_actual_var', 'mm/s', '#e879f9', True),
        ('Stepper current', 'current_var', 'A', '#fc727c', False),
        ('Motor speed', 'rpm_var', 'RPM', '#f5de64', False),
        ('Motor winding current', 'motor_winding_current_var', 'A', '#ac96ff', False),
        ('Motor current draw', 'motor_current_draw_var', 'A', '#ff8c50', False),
        ('Motor torque', 'motor_torque_var', 'Nm', '#55d9d4', False),
        ('Motor temp', 'temp_var', '°C', '#ec91ba', False),
        ('Tray angle', 'tray_var', '°', '#c0dc77', False),
    ]

    def _build_status_panel(self, root):
        group = QGroupBox('Drill status — live trends (500 ms)')
        root.addWidget(group)
        outer = QVBoxLayout(group)
        outer.setContentsMargins(9, 9, 9, 8)
        outer.setSpacing(7)

        # Controls live in the PANEL TITLEBAR; they do not consume plot height.
        self._chart_toolbar = QWidget()
        options = QHBoxLayout(self._chart_toolbar)
        options.setContentsMargins(0, 0, 0, 0)
        options.setSpacing(5)
        options.addWidget(QLabel('Time:'))
        self.chart_window = QComboBox()
        for label, secs in [('30 s', 30), ('1 min', 60), ('2 min', 120),
                            ('5 min', 300), ('10 min', 600), ('30 min', 1800), ('60 min', 3600)]:
            self.chart_window.addItem(label, secs)
        self.chart_window.setCurrentIndex(3)
        self.chart_window.currentIndexChanged.connect(self._update_chart)
        options.addWidget(self.chart_window)
        reset = QPushButton('Clear history')
        reset.clicked.connect(self._clear_history)
        options.addWidget(reset)
        self.chart_info = QLabel('Waiting for telemetry…')
        self.chart_info.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        options.addWidget(self.chart_info)
        self.chart_info.setMinimumWidth(100)

        center = QHBoxLayout()
        self.time_axis = TimeWindowAxis()
        self.chart = pg.PlotWidget(background='#101a29', axisItems={'bottom': self.time_axis})
        self.chart.setMinimumHeight(150)
        # Each enabled physical-unit axis draws its own horizontal grid.
        self.chart.showGrid(x=True, y=False, alpha=0.22)
        self.chart.setLabel('bottom', '')
        self.chart.setLabel('left', '')
        plot_item = self.chart.getPlotItem()
        plot_item.getAxis('left').setTextPen('#40c9ff')
        plot_item.getAxis('bottom').setTextPen('#aabbd0')
        self.time_axis.set_window(int(self.chart_window.currentData()))
        self.chart.setMouseEnabled(x=True, y=False)
        self.chart.setMenuEnabled(False)
        # The tiny 'A' that appears on hover is pyqtgraph's automatic range button.
        # Our chart already calculates X/Y limits, so keep this hidden permanently.
        plot_item.hideButtons()
        plot_item.setClipToView(True)
        plot_item.showAxis('right', False)
        plot_item.getViewBox().setDefaultPadding(0)
        self.curves = {}
        self.chart_checks = {}
        self.chart_values = {}
        self.chart_axes = {}           # field -> axis for its physical unit
        self.chart_views = {}          # field -> ViewBox for its physical unit
        self.chart_units = {}
        self.unit_axes = {}
        self.unit_views = {}
        self.unit_fields = {}
        self.unit_colors = {}
        self.unit_name_labels = {}
        main_view = plot_item.getViewBox()
        # Only visible units occupy axis columns; hidden units reserve no space.
        ordered_units = list(dict.fromkeys(field[2] for field in self.STATUS_FIELDS))
        for unit_index, unit in enumerate(ordered_units):
            first = next(field for field in self.STATUS_FIELDS if field[2] == unit)
            color = first[3]
            self.unit_colors[unit] = color
            self.unit_fields[unit] = []
            if unit_index == 0:
                view = main_view
                axis = plot_item.getAxis('left')
                axis.setLabel('')
                axis.setPen(pg.mkPen(color))
                axis.setTextPen(pg.mkPen(color))
            else:
                # Only active unit columns use width; newly enabled axes expand
                # the right chart gutter without reserving unused space.
                axis = pg.AxisItem(orientation='right', pen=pg.mkPen(color),
                                   textPen=pg.mkPen(color))
                axis.setWidth(0)
                axis.hide()
                axis.setStyle(tickTextOffset=4, autoExpandTextSpace=False)
                axis.setLabel('')
                plot_item.layout.addItem(axis, 2, unit_index + 1)
                view = pg.ViewBox(enableMenu=False)
                view.setMouseEnabled(x=False, y=False)
                view.setDefaultPadding(0)
                plot_item.scene().addItem(view)
                view.setXLink(main_view)
                axis.linkToView(view)
            # Horizontal unit annotation at the bottom of this Y-axis.
            unit_label = QLabel(unit, self.chart)
            unit_label.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents)
            unit_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            unit_label.setStyleSheet(f'color: {color}; font-size: 10px; font-weight: 700; background: transparent;')
            unit_label.setFixedHeight(16)
            self.unit_name_labels[unit] = unit_label
            self.unit_axes[unit] = axis
            self.unit_views[unit] = view
        for caption, attr, unit, color, checked in self.STATUS_FIELDS:
            view = self.unit_views[unit]
            self.chart_axes[attr] = self.unit_axes[unit]
            self.chart_views[attr] = view
            self.chart_units[attr] = unit
            self.unit_fields[unit].append(attr)
            curve = pg.PlotDataItem([], [], pen=pg.mkPen(color=color, width=2),
                                    connect='finite', skipFiniteCheck=False)
            view.addItem(curve)
            curve.setVisible(checked)
            self.curves[attr] = curve
        # Checkbox widgets are built below: refresh axes only after their
        # initial checked state exists (otherwise default axes stay blank).
        main_view.sigResized.connect(self._sync_chart_views)
        self.chart.installEventFilter(self)
        # The main X scale always fills the selected history window.
        self.chart.setXRange(-300, 0, padding=0)
        plot_item.getAxis('bottom').setHeight(56)
        center.addWidget(self.chart, 1)

        side = QWidget()
        side.setMinimumWidth(190)
        side.setMaximumWidth(270)
        side_lay = QVBoxLayout(side)
        side_lay.setContentsMargins(7, 0, 2, 0)
        side_lay.setSpacing(3)
        side_lay.addWidget(QLabel('VISIBLE SIGNALS / LATEST VALUE'))
        for caption, attr, unit, color, checked in self.STATUS_FIELDS:
            row = QHBoxLayout()
            box = QCheckBox(caption)
            box.setObjectName('signalToggle')
            box.setToolTip('Red filled circle = visible in graph; empty circle = hidden')
            box.setChecked(checked)
            # Always-visible, high-contrast ON/OFF indicator. Unlike a tiny
            # check mark, its filled red state is immediately recognizable.
            box.setStyleSheet(f"""
                QCheckBox {{ color: {color}; spacing: 8px; }}
                QCheckBox:checked {{ font-weight: 700; }}
                QCheckBox::indicator {{ width: 16px; height: 16px;
                    border-radius: 9px; border: 2px solid #748398;
                    background: transparent; }}
                QCheckBox::indicator:checked {{ background: #ff405a;
                    border: 2px solid #ff405a; }}
                QCheckBox::indicator:unchecked:hover {{ border: 2px solid #ff405a; }}
            """)
            box.toggled.connect(lambda enabled, a=attr: self._set_curve_visible(a, enabled))
            self.chart_checks[attr] = box
            row.addWidget(box, 1)
            value = QLabel('—')
            value.setObjectName('readout')
            value.setStyleSheet(f'color: {color}; font-weight: 700;')
            value.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
            setattr(self, attr, Value(value))
            self.chart_values[attr] = value
            row.addWidget(value)
            row.addWidget(QLabel(unit))
            side_lay.addLayout(row)
        side_lay.addStretch(1)
        center.addWidget(side)
        outer.addLayout(center, 1)
        self._refresh_unit_axes()
        # Finish scene geometry/axis linking once the workspace is shown.
        QTimer.singleShot(0, self._sync_chart_views)
        QTimer.singleShot(0, self._refresh_unit_axes)

        state_group = QGroupBox('Controller states')
        state_group.setToolTip('Live operational states of the three drill control state machines (not physical measurements).')
        states = QHBoxLayout(state_group)
        states.setContentsMargins(5, 3, 5, 3)
        state_header = QLabel('Controller states:')
        state_header.setToolTip('These are the operating states of the control logic, not physical measurements.')
        states.addWidget(state_header)
        for caption, attr in [('DeepSampler', 'deep_sampler_state_var'),
                              ('DrillController', 'drill_controller_state_var'),
                              ('DeepSampleHolder', 'deep_sample_holder_state_var')]:
            states.addWidget(QLabel(caption + ':'))
            val = QLabel('—')
            val.setObjectName('readout')
            val.setWordWrap(True)
            setattr(self, attr, Value(val))
            states.addWidget(val, 1)
        outer.addWidget(state_group)

    def _sync_chart_views(self):
        """Match all unit-specific views to the same plot drawing rectangle."""
        main = self.chart.getPlotItem().getViewBox()
        for view in self.unit_views.values():
            if view is not main:
                view.setGeometry(main.sceneBoundingRect())
                view.linkedViewChanged(main, view.XAxis)
        self._position_axis_units()

    def _position_axis_units(self):
        """Pin unit names in the bottom gutter directly beneath their Y axes.

        Unlike pyqtgraph's vertical axis title, these are horizontal labels.
        Their X positions are based on the actual chart ViewBox edges, not
        the overall PlotWidget center. Only visible right-hand axes use space.
        """
        if not hasattr(self, 'unit_name_labels'):
            return
        main = self.chart.getPlotItem().getViewBox()
        rect = main.sceneBoundingRect()
        if rect.width() < 1 or rect.height() < 1:
            return
        left_px = self.chart.mapFromScene(rect.topLeft()).x()
        right_px = self.chart.mapFromScene(rect.topRight()).x()
        bottom_px = self.chart.mapFromScene(rect.bottomLeft()).y()
        y = round(bottom_px + 30)  # below the numerical labels of the X axis

        # The left mm axis sits LEFT of the drawing rectangle. Give its
        # unit label the far-left part of that actual 62px axis column.
        left_axis = self.unit_axes.get('mm')
        if left_axis is not None:
            label = self.unit_name_labels['mm']
            if left_axis.isVisible():
                axis_width = max(45, int(left_axis.width()))
                x = round(left_px - axis_width + 4)
                label.setGeometry(max(0, x), y, 38, 16)
                label.show()
                label.raise_()
            else:
                label.hide()

        # pyqtgraph places active right-hand axes consecutively on the right
        # edge of the plot. Advance by each visible column's real width,
        # so mm/s, A, RPM, Nm, etc. stay under THEIR OWN axis after toggling.
        cursor = float(right_px)
        for unit, axis in self.unit_axes.items():
            if unit == 'mm':
                continue
            label = self.unit_name_labels[unit]
            if not axis.isVisible():
                label.hide()
                continue
            width = max(1, float(axis.width()))
            center_x = cursor + width / 2
            label.setGeometry(round(center_x - 27), y, 54, 16)
            label.show()
            label.raise_()
            cursor += width

    def _refresh_unit_axes(self):
        """Only enabled unit axes consume horizontal room in the plot grid."""
        for unit, axis in self.unit_axes.items():
            enabled = any(self.chart_checks.get(attr) is not None and
                          self.chart_checks[attr].isChecked()
                          for attr in self.unit_fields[unit])
            color = self.unit_colors[unit]
            axis.setLabel('')
            if enabled:
                axis.setWidth(64 if unit != 'mm' else 62)
                axis.setTextPen(pg.mkPen(color))
                axis.setPen(pg.mkPen(color))
                axis.setTicks(None)
                # The axis draws horizontal grid lines using *its own* ticks.
                # Multiple Y units therefore have their own independent grids.
                axis.setGrid(48 if unit != 'mm' else 68)
                axis.show()
            else:
                axis.setGrid(False)
                axis.hide()
                axis.setWidth(0)
                self.unit_name_labels[unit].hide()
        # Updating the layout may alter the ViewBox geometry.
        QTimer.singleShot(0, self._sync_chart_views)

    def _set_curve_visible(self, attr, enabled):
        self.curves[attr].setVisible(enabled)
        self._refresh_unit_axes()
        self._update_chart()

    def _clear_history(self):
        self._samples.clear()
        self._update_chart()

    def _record_state(self):
        # Real timestamp and values, captured on the incoming STATE response.
        values = {}
        for _, attr, _, _, _ in self.STATUS_FIELDS:
            try:
                value = float(getattr(self, attr).get())
                values[attr] = value if math.isfinite(value) else float('nan')
            except (ValueError, TypeError):
                values[attr] = float('nan')
        self._samples.append((time.monotonic() - self._plot_started, values))
        self._update_chart()

    @staticmethod
    def _axis_limits(values):
        """Auto-fit to the current time window, with a zero-based Y axis."""
        valid = values[np.isfinite(values)]
        if not len(valid):
            return 0.0, 1.0
        maximum = float(np.max(valid))
        # These graph scales represent non-negative magnitudes. Values below
        # zero are preserved in the underlying history but clipped from view.
        return 0.0, max(0.1, maximum * 1.18)

    def _update_chart(self):
        window = int(self.chart_window.currentData())
        self.time_axis.set_window(window)
        if not self._samples:
            for curve in self.curves.values():
                curve.setData([], [])
            for view in self.unit_views.values():
                view.setYRange(0, 1, padding=0)
            self.chart.setXRange(-window, 0, padding=0)
            self.chart_info.setText('Waiting for telemetry…')
            return

        newest = self._samples[-1][0]
        # IMPORTANT: filtering only affects rendering. _samples retains a full
        # hour, so 2 min -> 10 min restores previous measurements instantly.
        selected = [row for row in self._samples if row[0] >= newest - window]
        times = np.asarray([t - newest for t, _ in selected], dtype=float)
        active = 0
        for unit, attrs in self.unit_fields.items():
            series_for_range = []
            for attr in attrs:
                curve = self.curves[attr]
                if not self.chart_checks[attr].isChecked():
                    curve.setData([], [])
                    continue
                active += 1
                values = np.asarray([row[attr] for _, row in selected], dtype=float)
                curve.setData(times, values)
                series_for_range.append(values)
            if series_for_range:
                # The entire unit shares a scale: include ALL currently visible
                # variables before fitting, even if one is much larger.
                limits = self._axis_limits(np.concatenate(series_for_range))
                self.unit_views[unit].setYRange(*limits, padding=0)
                self.unit_views[unit].enableAutoRange(axis=pg.ViewBox.YAxis, enable=False)

        # The right end follows the latest sample. Older samples scroll off
        # the left edge without being deleted from the recording buffer.
        self.chart.setXRange(-window, 0, padding=0)
        self._sync_chart_views()
        self.chart_info.setText(
            f'{len(selected)} visible / {len(self._samples)} saved · '
            f'{window // 60 if window > 300 else window} {"min" if window > 300 else "s"} · {active} signals')

    def _build_manual_controls(self):
        layout = self._group(0, 'Manual controls')
        self._entry(layout, 0, 'Drill speed (RPM):', 'drill_speed_var', 0)
        self._button(layout, 'Set', self._set_drill_speed, 0, 2)
        self._button(layout, 'Stop', self._stop_drill_speed, 0, 3, danger=True)
        self._entry(layout, 1, 'Vertical speed (mm/s):', 'vert_speed_request_var', 0)
        self._button(layout, 'Set', self._set_vertical_speed, 1, 2)
        self._button(layout, 'Stop', self._stop_vertical_speed, 1, 3, danger=True)
        self._entry(layout, 2, 'Storage position:', 'storage_pos_var', 0)
        self._button(layout, 'Set', self._set_storage_position, 2, 2, 2)
        self._separator(layout, 3)
        self._button(layout, 'Calibrate carriage depth', self._calibrate_height, 4, 0, 4)
        self._button(layout, 'RESTART', self._restart, 5, 0, 4, danger=True)

    def _build_height_panel(self):
        layout = self._group(1, 'Height measurement')
        layout.setContentsMargins(7, 2, 7, 2)
        layout.setVerticalSpacing(3)
        self._readout(layout, 0, 'Height above ground', 'height_above_ground_var', 'mm')
        measure, fetch = self._equal_button_row(layout, 1, (
            ('Measure height', self._measure_height, None),
            ('Get height', self._get_height, None)), 3)
        measure.setToolTip('Measure height above ground')
        fetch.setToolTip('Get height above ground')
        layout.setColumnStretch(0, 1)
        layout.setColumnStretch(2, 1)

    def _build_autonomy_controls(self):
        layout = self._group(1, 'Autonomy')
        self._entry(layout, 0, 'Target depth (cm):', 'drill_depth_var', 10)
        self._equal_button_row(layout, 1, (
            ('Start auto drill', self._drill_auto, 'primary'),
            ('Start auto store', self._store_auto, None)), 2)
        self._button(layout, 'Stop auto', self._stop_auto, 2, 0, 2, danger=True)
        layout.setColumnStretch(0, 1)
        layout.setColumnStretch(1, 1)

    def _build_sample_controls(self):
        layout = self._group(1, 'Sample handling')
        pairs = [
            ('Weigh deep', self._weigh_deep, 'Get deep', self._get_weight_deep),
            ('Weigh surface', self._weigh_surface, 'Get surface', self._get_weight_surface),
            ('Rock open', self._rock_open, 'Rock close', self._rock_close),
            ('Sand open', self._sand_open, 'Sand close', self._sand_close),
            ('Set hold', self._set_hold_mode, 'Clear hold', self._clear_hold_mode),
        ]
        for i, (left, left_action, right, right_action) in enumerate(pairs):
            self._button(layout, left, left_action, i, 0)
            self._button(layout, right, right_action, i, 1)
        layout.setColumnStretch(0, 1)
        layout.setColumnStretch(1, 1)

    def _build_device_status_panel(self):
        layout = self._group(2, 'Device status')
        self._button(layout, 'Check devices', self._start_dev_check, 0, 0, 2)
        self._button(layout, 'Get device status', self._get_device_status, 0, 2, 2)
        self._separator(layout, 1, 4)
        self._device_indicators = []
        # Two columns of peripherals, each with a dedicated status indicator.
        # Pair label and LED in the same cell so Qt can distribute space evenly.
        for i, name in enumerate(protocol.DEVICE_NAMES):
            item = QWidget()
            line = QHBoxLayout(item)
            line.setContentsMargins(2, 1, 4, 1)
            line.setSpacing(5)
            led = QLabel('●')
            led.setFixedWidth(17)
            led.setStyleSheet('font-size: 16px; color: #77869e;')
            line.addWidget(led)
            caption = QLabel(name)
            caption.setWordWrap(True)
            line.addWidget(caption, 1)
            layout.addWidget(item, 2 + i // 2, (i % 2) * 2, 1, 2)
            self._device_indicators.append(led)
        layout.setColumnStretch(1, 1)
        layout.setColumnStretch(3, 1)

    def _update_device_indicators(self, devices):
        for led, device in zip(self._device_indicators, devices):
            led.setStyleSheet('font-size: 17px; color: ' +
                              ('#52df93;' if device['ok'] else '#ff6973;'))

    def _build_weight_panel(self):
        layout = self._group(2, 'Weight')
        layout.setContentsMargins(7, 2, 7, 2)
        layout.setVerticalSpacing(3)
        self._readout(layout, 0, 'Weight (deep)', 'weight_deep_var', 'g')
        self._readout(layout, 1, 'Weight (surface)', 'weight_surface_var', 'g')

    def _build_calibration_panel(self):
        layout = self._group(2, 'Calibration')
        label = QLabel('Calibration weight (g):')
        layout.addWidget(label, 0, 0)
        edit = QLineEdit('100.0')
        edit.setMaximumWidth(110)
        setattr(self, 'calib_weight_var', Value(edit))
        layout.addWidget(edit, 0, 1)
        self._equal_button_row(layout, 1, (
            ('Cal 0g — deep', self._calibrate_0_deep, None),
            ('Cal 0g — surface', self._calibrate_0_surface, None)), 2)
        self._equal_button_row(layout, 2, (
            ('Cal Xg — deep', self._calibrate_x_deep, None),
            ('Cal Xg — surface', self._calibrate_x_surface, None)), 2)
        layout.setColumnStretch(0, 1)
        layout.setColumnStretch(1, 1)

    def _build_log(self):
        group = QGroupBox('Communication log')
        self._register_panel('Communication log', group, (1110, 10, 330, 420))
        layout = QVBoxLayout(group)
        self.auto_scroll = QCheckBox('Auto-scroll')
        self.auto_scroll.setChecked(True)
        log_actions = QHBoxLayout()
        log_actions.addWidget(self.auto_scroll)
        log_actions.addStretch(1)
        clear_btn = QPushButton('Clear log')
        clear_btn.clicked.connect(self.log_text_clear)
        log_actions.addWidget(clear_btn)
        layout.addLayout(log_actions)
        self.log_text = QPlainTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setLineWrapMode(QPlainTextEdit.LineWrapMode.WidgetWidth)
        self.log_text.setFont(QFont('Consolas', 9))
        layout.addWidget(self.log_text, 1)

    def log_text_clear(self):
        self.log_text.clear()

    def _log(self, message):
        self.log_text.appendPlainText(f'[{time.strftime("%H:%M:%S")}] {message}')
        if self.auto_scroll.isChecked():
            self.log_text.verticalScrollBar().setValue(self.log_text.verticalScrollBar().maximum())

    def _update_computed_depths(self):
        if self._last_carriage_depth_mm is None:
            return
        offset = (self._last_carriage_depth_mm + self._CARRIAGE_TOP_TO_SPIRAL_TIP_MM
                  - self._LIN_AXIS_ZERO_TO_SENSOR_MM)
        self.depth_under_rover_var.set(str(offset))
        if self._last_height_above_ground_mm is not None:
            self.depth_under_surface_var.set(str(offset - self._last_height_above_ground_mm))
        else:
            self.depth_under_surface_var.set('— no height')

    def _poll_rx(self):
        # Limit the batch to keep Qt responsive if the transport floods the queue.
        for _ in range(300):
            try:
                msg = self.worker.rx_queue.get_nowait()
            except queue.Empty:
                break
            try:
                self._handle_message(msg)
            except Exception as exc:
                self._log(f'ERROR handling response: {exc}')
        if not self.worker.is_connected() and self.connect_btn.text() == 'Disconnect':
            self._sync_connection_ui()
            self._log('Connection lost.')

    def _poll_state(self):
        if self.worker.is_connected():
            self.worker.send(protocol.cmd_state())

    def closeEvent(self, event):
        settings = QSettings('DrillingHMI', 'DrillingHMI')
        settings.setValue('main_geometry', self.saveGeometry())
        for title, (win, _) in self._panels.items():
            settings.setValue('panels/' + title + '/geometry', win.normalGeometry())
            settings.setValue('panels/' + title + '/visible',
                              self._panel_actions[title].isChecked())
        self.rx_timer.stop()
        self.state_timer.stop()
        self.worker.disconnect()
        event.accept()

    def _handle_message(self, msg):
        if 'debug' in msg:
            self._log(f"[ESP] {msg['debug']}")
            return
        if 'error' in msg:
            self._log(f"ERROR: {msg['error']}")
            return
        if msg.get('nack'):
            self._log('NACK — drill refused the command.')
            return
        code = msg.get('code')
        if code == protocol.CMD_STATE and 'state' in msg:
            s = msg['state']
            self._last_carriage_depth_mm = s['height_mm']
            self.carriage_depth_var.set(str(s['height_mm']))
            self._update_computed_depths()
            self.vert_speed_actual_var.set(f"{s['vert_speed']:.1f}")
            self.current_var.set(f"{s['current_a']:.2f}")
            self.rpm_var.set(str(s['rpm']))
            self.motor_winding_current_var.set(f"{s['motor_winding_current_a']:.2f}")
            self.motor_current_draw_var.set(f"{s['motor_current_draw_a']:.2f}")
            self.motor_torque_var.set(f"{s['motor_torque_nm']:.2f}")
            self.temp_var.set(str(s['temp_c']))
            self.tray_var.set(str(s['tray_angle']))
            self.deep_sampler_state_var.set(s['deep_sampler_state_str'])
            self.drill_controller_state_var.set(s['drill_controller_state_str'])
            self.deep_sample_holder_state_var.set(s['deep_sample_holder_state_str'])
            self._record_state()
        elif code == protocol.CMD_GET_HEIGHT and 'height_mm' in msg:
            self._last_height_above_ground_mm = msg['height_mm']
            self.height_above_ground_var.set(str(msg['height_mm']))
            self._update_computed_depths()
            self._log(f"Height above ground: {msg['height_mm']} mm")
        elif code == protocol.CMD_GET_WEIGHT_DEEP and 'weight' in msg:
            self.weight_deep_var.set(f"{msg['weight']:.2f}")
            self._log(f"Deep sample weight: {msg['weight']:.2f} g  (raw ADC: {msg['adc_raw']})")
        elif code == protocol.CMD_GET_WEIGHT_SURFACE and 'weight' in msg:
            self.weight_surface_var.set(f"{msg['weight']:.2f}")
            self._log(f"Surface sample weight: {msg['weight']:.2f} g  (raw ADC: {msg['adc_raw']})")
        elif code == protocol.CMD_GET_DEVICE_STATUS and 'device_status' in msg:
            self._update_device_indicators(msg['device_status'])
            failed = [d['name'] for d in msg['device_status'] if not d['ok']]
            self._log(f"Device check — failed: {', '.join(failed)}" if failed
                      else 'Device check — all devices OK.')
        else:
            self._log(f'ACK: command 0x{code:02X}' if code is not None
                      else f'Unknown response: {msg}')

    def _send(self, raw, description):
        if not self.worker.is_connected():
            self._log('Not connected.')
            return
        self.worker.send(raw)
        self._log(f'Sent: {description}')

    def _restart(self):
        result = QMessageBox.question(self, 'Confirm', 'Restart the drill controller?',
                                      QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No)
        if result == QMessageBox.StandardButton.Yes:
            self._send(protocol.cmd_restart(), 'RESTART')

    def _calibrate_height(self):
        self._send(protocol.cmd_calibrate_height(), 'CALIBRATE HEIGHT')

    def _measure_height(self):
        self._send(protocol.cmd_measure_height(), 'MEASURE HEIGHT')

    def _get_height(self):
        self._send(protocol.cmd_get_height(), 'GET HEIGHT')

    def _start_dev_check(self):
        self._send(protocol.cmd_start_dev_check(), 'START DEVICE CHECK')

    def _get_device_status(self):
        self._send(protocol.cmd_get_device_status(), 'GET DEVICE STATUS')

    def _set_drill_speed(self):
        try:
            rpm = int(self.drill_speed_var.get())
            self._send(protocol.cmd_drill_speed(rpm), f'DRILL SPEED {rpm} RPM')
        except (ValueError, OverflowError):
            self._log('Invalid RPM value (allowed -32768 to 32767).')

    def _set_vertical_speed(self):
        try:
            speed = float(self.vert_speed_request_var.get())
            if not (-12.8 <= speed <= 12.7):
                self._log('Vertical speed must be between -12.8 and 12.7 mm/s.')
                return
            self._send(protocol.cmd_vertical_speed(speed), f'VERTICAL SPEED {speed} mm/s')
        except (ValueError, OverflowError):
            self._log('Invalid speed value.')

    def _stop_drill_speed(self):
        self._send(protocol.cmd_drill_speed(0), 'DRILL SPEED 0 RPM')

    def _stop_vertical_speed(self):
        self._send(protocol.cmd_vertical_speed(0.0), 'VERTICAL SPEED 0 mm/s')

    def _set_storage_position(self):
        try:
            pos = int(self.storage_pos_var.get())
            self._send(protocol.cmd_storage_position(pos), f'STORAGE POSITION {pos}')
        except (ValueError, OverflowError):
            self._log('Invalid position value (allowed 0 to 255).')

    def _drill_auto(self):
        try:
            depth = int(self.drill_depth_var.get())
            if not (0 <= depth <= 255):
                self._log('Depth must be between 0 and 255 cm.')
                return
            self._send(protocol.cmd_drill_auto(depth), f'DRILL AUTO depth={depth} cm')
        except (ValueError, OverflowError):
            self._log('Invalid depth value.')

    def _stop_auto(self):
        self._send(protocol.cmd_stop_auto(), 'STOP AUTO')

    def _store_auto(self):
        self._send(protocol.cmd_store_auto(), 'STORE AUTO')

    def _weigh_deep(self):
        self._send(protocol.cmd_weigh_deep(), 'WEIGH DEEP')

    def _weigh_surface(self):
        self._send(protocol.cmd_weigh_surface(), 'WEIGH SURFACE')

    def _get_weight_deep(self):
        self._send(protocol.cmd_get_weight_deep(), 'GET WEIGHT DEEP')

    def _get_weight_surface(self):
        self._send(protocol.cmd_get_weight_surface(), 'GET WEIGHT SURFACE')

    def _calibrate_0_deep(self):
        self._send(protocol.cmd_calibrate_0_deep(), 'CALIBRATE 0 DEEP')

    def _calibrate_x_deep(self):
        try:
            weight = float(self.calib_weight_var.get())
            self._send(protocol.cmd_calibrate_x_deep(weight), f'CALIBRATE X DEEP ({weight} g)')
        except (ValueError, OverflowError):
            self._log('Invalid calibration weight.')

    def _calibrate_0_surface(self):
        self._send(protocol.cmd_calibrate_0_surface(), 'CALIBRATE 0 SURFACE')

    def _calibrate_x_surface(self):
        try:
            weight = float(self.calib_weight_var.get())
            self._send(protocol.cmd_calibrate_x_surface(weight), f'CALIBRATE X SURFACE ({weight} g)')
        except (ValueError, OverflowError):
            self._log('Invalid calibration weight.')

    def _set_hold_mode(self):
        self._send(protocol.cmd_set_hold_mode(), 'SET HOLD MODE')

    def _clear_hold_mode(self):
        self._send(protocol.cmd_clear_hold_mode(), 'CLEAR HOLD MODE')

    def _rock_open(self):
        self._send(protocol.cmd_rock_open(), 'ROCK OPEN')

    def _rock_close(self):
        self._send(protocol.cmd_rock_close(), 'ROCK CLOSE')

    def _sand_open(self):
        self._send(protocol.cmd_sand_open(), 'SAND OPEN')

    def _sand_close(self):
        self._send(protocol.cmd_sand_close(), 'SAND CLOSE')
