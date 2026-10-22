"""Presentation-only theme for the sf-Motion desktop console."""

from string import Template

from PyQt5 import QtGui


COLORS = {
    "background": "#F3F5F8", "panel": "#FFFFFF", "text": "#172033",
    "muted": "#526176", "border": "#CDD5E0", "nav": "#111827",
    "nav_text": "#DCE4F0", "nav_selected": "#263954",
    "primary": "#2563EB", "primary_hover": "#1D4ED8",
    "primary_pressed": "#1E40AF", "tint": "#EFF6FF",
    "danger": "#B91C1C", "danger_hover": "#991B1B",
    "disabled": "#E8ECF2", "disabled_text": "#657084",
    "terminal": "#111827", "terminal_input": "#1C2739",
    "terminal_text": "#E2E8F0", "terminal_muted": "#A8B8CF",
}

SPACE = {"small": 8, "medium": 12, "section": 16, "page": 24}


def ui_font():
    font = QtGui.QFont("Segoe UI", 10)
    font.setStyleHint(QtGui.QFont.SansSerif)
    return font


def terminal_font():
    font = QtGui.QFont("Consolas", 10)
    font.setStyleHint(QtGui.QFont.Monospace)
    return font


def stylesheet():
    return Template("""
        QWidget { color: $text; font-family: 'Segoe UI'; font-size: 10pt; }
        QMainWindow, QDialog, QStackedWidget, QWidget#Page, QWidget#PageContent {
            background: $background;
        }
        QLabel { background: transparent; }
        QLabel#PageTitle { font-size: 18pt; font-weight: 600; }
        QLabel#SubtleText { color: $muted; }
        QLabel#MetricValue { font-size: 20pt; font-weight: 600; }
        QLabel#Brand { color: white; font-size: 16pt; font-weight: 600; }
        QLabel#BrandCaption { color: $terminal_muted; font-size: 9pt; }
        QFrame#Navigation { background: $nav; border: none; }
        QListWidget#NavigationList {
            background: $nav; color: $nav_text; border: none; padding: 8px;
        }
        QListWidget#NavigationList::item {
            padding: 6px 12px; margin: 2px 0; border: 2px solid transparent;
            border-radius: 6px;
        }
        QListWidget#NavigationList::item:hover { background: $nav_selected; }
        QListWidget#NavigationList::item:selected {
            background: $nav_selected; color: white; border-left-color: #60A5FA;
        }
        QListWidget#NavigationList:focus::item:selected { border-color: #93C5FD; }
        QToolBar {
            background: $panel; border: none; border-bottom: 1px solid $border;
            padding: 8px; spacing: 8px;
        }
        QToolBar::separator { background: $border; width: 1px; margin: 4px; }
        QPushButton, QToolButton {
            background: $panel; color: $text; border: 2px solid $border;
            border-radius: 6px; padding: 5px 11px; min-height: 18px;
            outline: none;
        }
        QPushButton:hover, QToolButton:hover { background: $tint; border-color: $primary; }
        QPushButton:pressed, QToolButton:pressed { background: #DBEAFE; border-color: $primary_pressed; }
        QPushButton:checked, QToolButton:checked {
            background: $primary; color: white; border-color: $primary;
        }
        QPushButton#PrimaryButton { background: $primary; border-color: $primary; color: white; }
        QPushButton#PrimaryButton:hover { background: $primary_hover; }
        QPushButton#PrimaryButton:pressed { background: $primary_pressed; }
        QPushButton#DangerButton, QToolButton#DangerButton {
            background: $danger; border-color: $danger; color: white; font-weight: 600;
        }
        QPushButton#DangerButton:hover, QToolButton#DangerButton:hover { background: $danger_hover; }
        QPushButton#DangerButton:pressed, QToolButton#DangerButton:pressed { background: #7F1D1D; }
        QPushButton:disabled, QToolButton:disabled,
        QPushButton#PrimaryButton:disabled, QPushButton#DangerButton:disabled,
        QToolButton#DangerButton:disabled {
            background: $disabled; color: $disabled_text; border-color: $border;
        }
        QPushButton:focus, QToolButton:focus {
            border-color: $primary;
        }
        QPushButton#PrimaryButton:focus, QPushButton#DangerButton:focus,
        QToolButton#DangerButton:focus {
            border-color: $text;
        }
        QGroupBox {
            background: $panel; border: 1px solid $border; border-radius: 6px;
            margin-top: 12px; padding: 16px 12px 12px; font-weight: 600;
        }
        QGroupBox::title { subcontrol-origin: margin; left: 12px; padding: 0 6px; }
        QLineEdit, QDoubleSpinBox, QSpinBox, QComboBox, QPlainTextEdit {
            background: $panel; color: $text; border: 1px solid $border;
            border-radius: 6px; padding: 5px 8px; min-height: 18px;
            selection-background-color: $primary; selection-color: white;
        }
        QLineEdit:hover, QDoubleSpinBox:hover, QSpinBox:hover, QComboBox:hover { border-color: $muted; }
        QLineEdit:focus, QDoubleSpinBox:focus, QSpinBox:focus,
        QComboBox:focus, QPlainTextEdit:focus { border-color: $primary; }
        QLineEdit:disabled, QDoubleSpinBox:disabled, QSpinBox:disabled, QComboBox:disabled {
            background: $disabled; color: $disabled_text;
        }
        QComboBox QAbstractItemView { background: $panel; color: $text; selection-background-color: $tint; }
        QCheckBox { spacing: 8px; padding: 4px 0; }
        QCheckBox:disabled { color: $disabled_text; }
        QCheckBox:focus { outline: 2px solid $primary; }
        QScrollArea { border: none; background: transparent; }
        QScrollArea > QWidget > QWidget { background: transparent; }
        QScrollBar:vertical { background: $background; width: 12px; margin: 0; }
        QScrollBar::handle:vertical { background: #A5B2C5; min-height: 24px; border-radius: 6px; }
        QScrollBar::handle:vertical:hover { background: $muted; }
        QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0; }
        QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical { background: transparent; }
        QSplitter::handle { background: $border; }
        QSplitter::handle:hover { background: $primary; }
        QSplitter#TerminalSplitter::handle {
            background: $border; border-top: 3px solid $background;
            border-bottom: 3px solid $background;
        }
        QSplitter#TerminalSplitter::handle:hover { background: $primary; }
        QFrame#ParameterActions {
            background: $panel; border: 1px solid $border; border-radius: 6px;
        }
        QFrame#RealtimeStrip { background: $panel; border-top: 1px solid $border; }
        QLabel#RealtimeName { color: $muted; font-size: 9pt; }
        QLabel#RealtimeValue { color: $text; font-size: 11pt; font-weight: 600; }
        QFrame#TerminalPanel { background: $terminal; border-top: 1px solid $border; }
        QFrame#TerminalPanel QLabel { color: $terminal_muted; }
        QFrame#TerminalPanel QPushButton {
            background: $terminal_input; color: $terminal_text; border-color: #526176;
            padding: 2px 11px; min-height: 16px;
        }
        QFrame#TerminalPanel QPushButton:hover { background: $nav_selected; border-color: #93C5FD; }
        QFrame#TerminalPanel QPushButton:focus { border-color: #93C5FD; }
        QPlainTextEdit#TerminalOutput, QLineEdit#TerminalInput {
            background: $terminal_input; color: $terminal_text; border-color: #526176;
            font-family: Consolas; font-size: 10pt;
        }
        QPlainTextEdit#TerminalOutput:focus, QLineEdit#TerminalInput:focus { border-color: #93C5FD; }
        QPlainTextEdit#TerminalOutput { min-height: 48px; }
        QStatusBar { background: $panel; color: $muted; border-top: 1px solid $border; }
        QStatusBar::item { border: none; }
        QToolTip { background: $nav; color: white; border: 1px solid $border; padding: 6px; }
    """).substitute(COLORS)


def style_plot(plot):
    """Style existing axes without altering curves, ranges, or sampling."""
    plot.setBackground(COLORS["panel"])
    for position in ("left", "bottom"):
        axis = plot.getAxis(position)
        pen = QtGui.QPen(QtGui.QColor(COLORS["muted"]))
        pen.setCosmetic(True)
        axis.setPen(pen)
        axis.setTextPen(COLORS["muted"])
        axis.setTickFont(ui_font())
        axis.setTickDensity(1.5)
        axis.setStyle(maxTickLevel=2, maxTextLevel=1)
    plot.setRenderHint(QtGui.QPainter.Antialiasing, True)
    plot.showGrid(x=True, y=True, alpha=0.35)
