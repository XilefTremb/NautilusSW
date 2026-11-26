"""Read-only state-machine diagram for the mission monitor."""
import math

from PyQt5.QtCore import Qt, QPointF, QRectF
from PyQt5.QtGui import QColor, QPainter, QPen, QBrush, QPolygonF, QFont
from PyQt5.QtWidgets import QWidget, QSizePolicy


class StateGraph(QWidget):
    # Coordinates in a fixed logical canvas, scaled to fit the widget.
    NODES = {
        'IDLE': (95, 65),
        'LOAD_OBJECTIVE': (315, 65),
        'MISSION_COMPLETE': (595, 65),
        'SEARCH_TARGET': (315, 205),
        'CENTER_TARGET': (595, 205),
        'APPROACH_TARGET': (595, 345),
        'EXECUTE_ACTION': (315, 345),
    }
    # Explicit transitions from state_machine.py. Wildcard transitions
    # share a separate ANY STATE hub to avoid a dense mesh of arrows.
    EDGES = [
        ('IDLE', 'LOAD_OBJECTIVE', 'start_mission', [(177,65),(233,65)], (205,48)),
        ('LOAD_OBJECTIVE','MISSION_COMPLETE','load_next_objective',[(397,65),(513,65)],(455,48)),
        ('LOAD_OBJECTIVE','SEARCH_TARGET','load_next_objective',[(315,88),(315,182)],(390,125)),
        ('SEARCH_TARGET','CENTER_TARGET','target_found',[(397,195),(513,195)],(455,179)),
        ('CENTER_TARGET','SEARCH_TARGET','target_lost',[(513,218),(397,218)],(455,237)),
        ('CENTER_TARGET','APPROACH_TARGET','target_centered_event',[(595,228),(595,322)],(680,275)),
        ('APPROACH_TARGET','SEARCH_TARGET','target_lost',[(513,333),(455,333),(455,260),(355,260),(355,228)],(414,278)),
        ('APPROACH_TARGET','EXECUTE_ACTION','target_reached',[(513,345),(397,345)],(455,363)),
        ('EXECUTE_ACTION','LOAD_OBJECTIVE','action_done',[(233,345),(200,345),(200,95),(275,95),(275,88)],(157,300)),
    ]
    GLOBAL = [
        ('EXECUTE_ACTION','no_target_to_be_reached',[(95,425),(95,395),(315,395),(315,368)],(205,413)),
        ('LOAD_OBJECTIVE','skip_to_next_objective',[(13,448),(5,448),(5,15),(315,15),(315,42)],(165,8)),
        ('MISSION_COMPLETE','finish_mission',[(177,448),(735,448),(735,65),(677,65)],(650,435)),
    ]

    def __init__(self, parent=None):
        super().__init__(parent)
        self.state = None
        self.last = None
        self.elapsed = 0
        self.stale = False
        self.setMinimumHeight(240)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def set_status(self, state, last, elapsed):
        self.state, self.last, self.elapsed = state, last, elapsed
        self.stale = False
        self.update()

    def set_stale(self, stale):
        if stale != self.stale:
            self.stale = stale
            self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        scale = min(self.width()/750, self.height()/520)
        p.translate((self.width()-750*scale)/2, (self.height()-520*scale)/2)
        p.scale(scale, scale)
        muted = '#77869a'
        active = '#4fc3f7'
        recent = '#ffb74d'
        last = self.last or {}
        transition = last.get('transition', '')
        source, _, dest = transition.partition(' → ')
        trigger = last.get('trigger', '')

        def label(x, y, value, color, size=10, width=180):
            p.setPen(QColor(color))
            p.setFont(QFont('Sans Serif', size))
            p.drawText(QRectF(x-width/2,y-10,width,20), Qt.AlignCenter, value)

        def arrow(points, color, dashed=False, thick=False):
            pen = QPen(QColor(color), 3 if thick else 1.4)
            if dashed:
                pen.setStyle(Qt.DashLine)
            p.setPen(pen)
            p.setBrush(Qt.NoBrush)
            pts = [QPointF(*xy) for xy in points]
            p.drawPolyline(QPolygonF(pts))
            a,b = pts[-2],pts[-1]
            angle = math.atan2(b.y()-a.y(), b.x()-a.x())
            tip = [b, QPointF(b.x()-9*math.cos(angle-.45), b.y()-9*math.sin(angle-.45)),
                   QPointF(b.x()-9*math.cos(angle+.45), b.y()-9*math.sin(angle+.45))]
            p.setBrush(QColor(color))
            p.drawPolygon(QPolygonF(tip))

        for src, dst, name, points, pos in self.EDGES:
            selected = (src,dst,name)==(source,dest,trigger)
            color = recent if selected else muted
            arrow(points,color,thick=selected)
            label(*pos,name,color,9)

        for dst,name,points,pos in self.GLOBAL:
            selected = (dst,name)==(dest,trigger)
            color = recent if selected else muted
            arrow(points,color,dashed=True,thick=selected)
            label(*pos,name,color,9)

        for name,(x,y) in self.NODES.items():
            selected = name == self.state
            color = active if selected and not self.stale else muted
            p.setPen(QPen(QColor(color), 3 if selected else 1.4))
            p.setBrush(QColor('#16384a' if selected and not self.stale else '#202a38'))
            p.drawRoundedRect(QRectF(x-82,y-23,164,46),8,8)
            label(x,y,name,color if selected else '#dce6f2',10,164)

        p.setPen(QPen(QColor(muted),1.4,Qt.DashLine))
        p.setBrush(QColor('#202a38'))
        p.drawRoundedRect(QRectF(13,425,164,46),8,8)
        label(95,448,'ANY STATE', '#dce6f2',10)
        if trigger in {item[1] for item in self.GLOBAL} and source in self.NODES:
            # The highlighted wildcard arrow represents the actual source
            # named below; outline that source without obscuring active state.
            x,y=self.NODES[source]
            p.setPen(QPen(QColor(recent),2,Qt.DashLine))
            p.setBrush(Qt.NoBrush)
            p.drawRoundedRect(QRectF(x-86,y-27,172,54),10,10)

        label(375,485, ('STALE — last known state' if self.stale else
                        f'Current: {self.state or "Waiting for telemetry"}  |  {self.elapsed:.1f} s'),
              muted if self.stale else active,11,740)
        label(375,505, f'Last: {transition} ({trigger})' if transition else 'Last: —',
              recent if transition else muted,10,740)
        p.end()
