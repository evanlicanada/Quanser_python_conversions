"""
scope.py: An optimized version of Quanser's real-time plotting and visualization module.

Key optimizations over standard pal.utilities.scope:
1. High-Performance Signal Rendering:
   Instead of instantiating thousands of individual pg.PlotDataItem chunks on every
   frame, each signal maintains a single persistent curve (PlotDataItem) updated
   in-place with curve.setData(). Points outside the visible timeWindow are
   pruned on each frame.
2. Safe Event Loop Handling:
   MultiScope.refreshAll() checks QThread.currentThread().loopLevel() to only call
   processEvents() when running outside an active Qt event loop (e.g. simple while
   loops), preventing nested event loop recursion and freezes in PyQt/QTimer apps.
"""
import sys
import numpy as np
from array import array
from collections import deque
from time import time
import pyqtgraph as pg
pg.setConfigOptions(useOpenGL=True, antialias=False)

from pyqtgraph.Qt import QtWidgets

if 'PyQt6' in sys.modules:
    from PyQt6.QtCore import Qt, QThread
elif 'PyQt5' in sys.modules:
    from PyQt5.QtCore import Qt, QThread
else:
    # Try importing PyQt6 first, then PyQt5
    try:
        from PyQt6.QtCore import Qt, QThread
    except ImportError:
        try:
            from PyQt5.QtCore import Qt, QThread
        except ImportError:
            raise ImportError("Scope requires either PyQt5 or PyQt6")


#region : Descriptor Classes
class _ScopeInfo():
    def __init__(self, **kwargs):
        self.title = kwargs.pop('title', None)
        self.rows = kwargs.pop('rows', 1)
        self.cols = kwargs.pop('cols', 1)

        self.maxSampleRate = kwargs.pop('maxSampleRate', 1024) # samples/second
        self.fps = kwargs.pop('fps', 30)

        self.axes = []


class _AxisInfo():
    def __init__(self, **kwargs):
        self._title = kwargs.pop('title', None)
        if self._title is None:
            self._title = kwargs.pop('_title', None)

        self._xLabel = kwargs.pop('xLabel', None)
        if self._xLabel is None:
            self._xLabel = kwargs.pop('_xLabel', None)

        self._yLabel = kwargs.pop('yLabel', None)
        if self._yLabel is None:
            self._yLabel = kwargs.pop('_yLabel', None)

        self._yLim = kwargs.pop('yLim', None)
        if self._yLim is None:
            self._yLim = kwargs.pop('_yLim', None)

        self.row = kwargs.pop('row', 0)
        self.col = kwargs.pop('col', 0)
        self.rowSpan = kwargs.pop('rowSpan', 1)
        self.colSpan = kwargs.pop('colSpan', 1)

        self.signals = []


class _XYAxisInfo(_AxisInfo):
    def __init__(self, **kwargs):
        self._xLim = kwargs.pop('xLim', None)
        if self._xLim is None:
            self._xLim = kwargs.pop('_xLim', None)

        super().__init__(**kwargs)

        self.images = []


class _TYAxisInfo(_AxisInfo):
    def __init__(self, **kwargs):
        self.timeWindow = kwargs.pop('timeWindow', 10)
        self.displayMode = kwargs.pop('displayMode', 0)
        super().__init__(**kwargs)


class _BatchAxisInfo(_AxisInfo):
    def __init__(self, **kwargs):
        self.arrayLen = kwargs.pop('arrayLen', None)
        self._xLim = kwargs.pop('xLim', None)
        if self._xLim is None:
            self._xLim = kwargs.pop('_xLim', None)
        super().__init__(**kwargs)


class _SignalInfo():
    def __init__(self, **kwargs):
        self.name = kwargs.pop('name', None)
        self.color = kwargs.pop('color', None)
        self.lineStyle = kwargs.pop('lineStyle', None)
        self.width = kwargs.pop('width', None)
        self.scale = kwargs.pop('scale', 1)
        self.offset = kwargs.pop('offset', 0)


class _ImageInfo():
    def __init__(self, **kwargs):
        self._scale = kwargs.pop('scale', (1,1))
        if self._scale is None:
            self._scale = kwargs.pop('_scale', None)

        self._offset = kwargs.pop('offset', (0,0))
        if self._offset is None:
            self._offset = kwargs.pop('_offset', None)

        self._rotation = kwargs.pop('rotation', 0)
        if self._rotation is None:
            self._rotation = kwargs.pop('_rotation', None)

        self._levels = kwargs.pop('levels', (0, 1))
        if self._levels is None:
            self._levels = kwargs.pop('_levels', None)
#endregion


#region : Primary Scope Classes
class MultiScope(_ScopeInfo):
    """MultiScope: A class for creating and managing a scope with multiple axes.

    This class inherits from _ScopeInfo and allows for the creation and
    management of a scope with multiple axes in a grid layout, supporting both
    time-y and x-y axes.

    Attributes:
        activeScopes (list): A list of active MultiScope instances.
        spf (float): Time to wait between frames.
        tpf (float): Time of the last frame.
    """

    activeScopes = []
    spf = None # time to wait between frames
    tpf = 0 # time of last frame

    def __init__(self, *args, **kwargs):
        """Initialize a MultiScope instance."""
        graphicsLayoutWidget = kwargs.pop('graphicsLayoutWidget', None)

        if args and isinstance(args[0], _ScopeInfo):
            super().__init__(**vars(args[0]))
            axes = args[0].axes
        else:
            super().__init__(**kwargs)
            axes = []

        MultiScope.activeScopes.append(self)
        if MultiScope.spf is None:
            MultiScope.spf = 1 / self.fps
        else:
            MultiScope.spf = np.minimum(
                MultiScope.spf,
                1 / self.fps
            )

        if 'bufferSize' in kwargs:
            self._bufferSize = kwargs['bufferSize']
        else:
            self._bufferSize = int(np.ceil(2 * self.maxSampleRate / self.fps))

        if graphicsLayoutWidget is None:
            self.graphicsLayoutWidget = pg.GraphicsLayoutWidget(
                show=True,
                title=self.title
            )
        else:
            self.graphicsLayoutWidget = graphicsLayoutWidget

        self.cells = []
        for i in range(self.rows):
            self.cells.append([None] * self.cols)

        for axis in axes:
            self._addAxis(axis)

    def _cells_available(self, row, col, rowSpan, colSpan):
        if (row < 0
            or col < 0
            or row + rowSpan > self.rows
            or col + colSpan > self.cols):
            return False

        for i in range(rowSpan):
            for j in range(colSpan):
                if self.cells[row + i][col + j] is not None:
                    return False

        return True

    def _populate_cells(self, axis):
        for i in range(axis.rowSpan):
            for j in range(axis.colSpan):
                self.cells[axis.row + i][axis.col + j] = axis

    def _addAxis(self, axisInfo):
        if isinstance(axisInfo, _TYAxisInfo):
            AxisType = TYAxis
        elif isinstance(axisInfo, _XYAxisInfo):
            AxisType = XYAxis
        elif isinstance(axisInfo, _BatchAxisInfo):
            AxisType = BatchAxis
        else:
            raise TypeError('Invalid Axis type provided')

        if not self._cells_available(
                axisInfo.row,
                axisInfo.col,
                axisInfo.rowSpan,
                axisInfo.colSpan):
            raise Exception("Scope cell range invalid or already occupied")

        bufferSize = getattr(axisInfo, 'arrayLen', self._bufferSize)

        axis_kwargs = vars(axisInfo).copy()
        axis_kwargs.pop('arrayLen', None)

        axis = AxisType(
            graphicsLayoutWidget=self.graphicsLayoutWidget,
            bufferSize=bufferSize,
            **axis_kwargs
        )

        for sig in axisInfo.signals:
            axis.attachSignal(sig)

        if isinstance(axisInfo, _XYAxisInfo):
            for img in axisInfo.images:
                axis.attachImage(img)

        self.axes.append(axis)

    def addAxis(self, *args, **kwargs):
        """Add a time-y axis to the MultiScope instance."""
        if args:
            if isinstance(args[0], _TYAxisInfo):
                self._addAxis(args[0])
            else:
                raise TypeError(
                    'Must use keyword arguments, unless '
                    + 'providing a _TYAxisInfo object.'
                )
        else:
            self._addAxis(_TYAxisInfo(**kwargs))

    def addXYAxis(self, *args, **kwargs):
        """Add an x-y axis to the MultiScope instance."""
        if args:
            if isinstance(args[0], _XYAxisInfo):
                self._addAxis(args[0])
            else:
                raise TypeError(
                    'Must use keyword arguments, unless '
                    + 'providing a _XYAxisInfo object.'
                )
        else:
            self._addAxis(_XYAxisInfo(**kwargs))

    def addBatchAxis(self, *args, **kwargs):
        """Add a batch axis to the MultiScope instance."""
        if args:
            if isinstance(args[0], _BatchAxisInfo):
                self._addAxis(args[0])
            else:
                raise TypeError(
                    'Must use keyword arguments, unless '
                    + 'providing a _BatchAxisInfo object.'
                )
        else:
            if 'arrayLen' not in kwargs:
                kwargs['arrayLen'] = self._bufferSize
            self._addAxis(_BatchAxisInfo(**kwargs))

    def refresh(self):
        """Refresh the MultiScope instance, updating contained axes."""
        MultiScope.refreshAll()

    @staticmethod
    def refreshAll():
        """Refresh all active MultiScope instances, updating all scope plots.
        Checks loopLevel to safely avoid recursive event loops when running in Qt.
        """
        if time() - MultiScope.tpf >= MultiScope.spf:
            MultiScope.tpf = time()
            flush = True
        else:
            flush = False

        for scope in MultiScope.activeScopes:
            for axis in scope.axes:
                axis.refresh(flush)

        if flush:
            # Only process events manually if no Qt event loop (e.g. app.exec) is already running
            if QThread.currentThread().loopLevel() == 0:
                QtWidgets.QApplication.processEvents()


class Axis(_AxisInfo):
    """Axis: A base class for creating and managing scope plot axes."""

    def __init__(self, graphicsLayoutWidget, bufferSize, **kwargs):
        super().__init__(**kwargs)

        self.plot = graphicsLayoutWidget.addPlot(
            self.row,
            self.col,
            self.rowSpan,
            self.colSpan,
            labels={
                'bottom': (self._xLabel,),
                'left': (self._yLabel,)
            }
        )
        self.plot.showGrid(x=True, y=True)
        self.plot.addLegend()
        self.yLim = self._yLim

        self._bufferSize = bufferSize
        self._tBuffer = array('f', range(bufferSize))
        self._dataBuffer = []
        self._iBuffer = -1
        self._sampleQueue = deque()

        self.refresh = self._initial_refresh

    def attachSignal(self, *args, **kwargs):
        """Attach a signal to the Axis instance."""
        di = len(self.signals)
        s = Signal(self.plot, di=di, *args, **kwargs)
        self.signals.append(s)

    def sample(self, t, data):
        """Sample data for the Axis instance."""
        data = [data] if np.isscalar(data) else data
        self._sampleQueue.appendleft([t, data])

    def clean(self):
        self._sampleQueue.clear()

    def clear(self):
        """Clear the signals in the Axis instance."""
        for s in self.signals:
            s.clear()

    def _initial_refresh(self, flush):
        if len(self._sampleQueue) < 2:
            return
        self.refresh = self._post_initial_refresh
        self.refresh(flush)

    def _post_initial_refresh(self, flush):
        pass

    @property
    def xLabel(self):
        return self._xLabel
    @xLabel.setter
    def xLabel(self, newXLabel):
        self._xLabel = newXLabel
        self.plot.setLabel('bottom', newXLabel)

    @property
    def yLabel(self):
        return self._yLabel
    @yLabel.setter
    def yLabel(self, newYLabel):
        self._yLabel = newYLabel
        self.plot.setLabel('left', newYLabel)

    @property
    def xLim(self):
        return self._xLim
    @xLim.setter
    def xLim(self, newXLimits):
        if newXLimits is None:
            self.plot.enableAutoRange(axis='x')
            return
        self._xLim = newXLimits
        self.plot.setXRange(newXLimits[0], newXLimits[1])

    @property
    def yLim(self):
        return self._yLim
    @yLim.setter
    def yLim(self, newYLimits):
        if newYLimits is None:
            self.plot.enableAutoRange(axis='y')
            return
        self._yLim = newYLimits
        self.plot.setYRange(newYLimits[0], newYLimits[1])


class XYAxis(Axis, _XYAxisInfo):
    """XYAxis: A class for creating and managing an x-y axis in a MultiScope."""

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.xLim = self._xLim

    def attachSignal(self, *args, **kwargs):
        self._dataBuffer.append(array('f', range(2 * self._bufferSize)))
        super().attachSignal(*args, **kwargs)

    def attachImage(self, *args, **kwargs):
        self.plot.showGrid(x=False, y=False)
        img = Image(self.plot, *args, **kwargs)
        self.images.append(img)

    def _post_initial_refresh(self, flush):
        while True:
            try:
                if self._iBuffer >= self._bufferSize - 1:
                    flush = True
                    break
                sample = self._sampleQueue.pop()
                self._iBuffer += 1
                self._tBuffer[self._iBuffer] = sample[0]
                for i in range(len(self.signals)):
                    self._dataBuffer[i][self._iBuffer] = sample[1][i][0]
                    self._dataBuffer[i][self._iBuffer + self._bufferSize] = sample[1][i][1]
            except IndexError:
                break

        if flush and self._iBuffer > 0:
            for i, signal in enumerate(self.signals):
                signal.add_chunk(
                    self._dataBuffer[i][0:self._iBuffer + 1],
                    self._dataBuffer[i][self._bufferSize:self._bufferSize + self._iBuffer + 1]
                )
                self._dataBuffer[i][0] = self._dataBuffer[i][self._iBuffer]
                self._dataBuffer[i][self._bufferSize] = self._dataBuffer[i][self._bufferSize + self._iBuffer]

            self._tBuffer[0] = self._tBuffer[self._iBuffer]
            self._iBuffer = 0


class TYAxis(Axis, _TYAxisInfo):
    """TYAxis: A class for creating and managing a time-y axis in a MultiScope."""

    def __init__(self, **kwargs):
        super().__init__(**kwargs)

    def attachSignal(self, *args, **kwargs):
        self._dataBuffer.append(array('f', range(self._bufferSize)))
        super().attachSignal(*args, **kwargs)
        # Give signal access to timeWindow for automatic pruning
        self.signals[-1].timeWindow = self.timeWindow + 0.5

    def _post_initial_refresh(self, flush):
        while True:
            try:
                if self._iBuffer >= self._bufferSize - 1:
                    flush = True
                    break
                sample = self._sampleQueue.pop()
                self._iBuffer += 1
                self._tBuffer[self._iBuffer] = sample[0]
                for i in range(len(self.signals)):
                    self._dataBuffer[i][self._iBuffer] = sample[1][i]
            except IndexError:
                break

        if flush and self._iBuffer > 0:
            self.plot.setXRange(
                np.max([0, self._tBuffer[self._iBuffer] - self.timeWindow]),
                np.max([self._tBuffer[self._iBuffer], self.timeWindow]),
                padding=0
            )
            for i, signal in enumerate(self.signals):
                signal.add_chunk(
                    self._tBuffer[0:self._iBuffer + 1],
                    self._dataBuffer[i][0:self._iBuffer + 1]
                )
                self._dataBuffer[i][0] = self._dataBuffer[i][self._iBuffer]

            self._tBuffer[0] = self._tBuffer[self._iBuffer]
            self._iBuffer = 0


class BatchAxis(Axis, _BatchAxisInfo):
    """BatchAxis: Plots a full array each update, X axis is array index."""
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self._dataBuffer.append(np.zeros(self._bufferSize, dtype=np.float32))
        if self._xLim is not None:
            self.xLim = self._xLim
        else:
            self.xLim = (0, self._bufferSize - 1)
        self._xBuffer = np.arange(self._bufferSize, dtype=np.float32)

    def attachSignal(self, *args, **kwargs):
        super().attachSignal(*args, **kwargs)
        if not hasattr(self, '_dataBuffer'):
            self._dataBuffer = []
        self._dataBuffer.append(np.zeros(self._bufferSize, dtype=np.float32))

    def sample(self, data):
        self.clear()
        self._sampleQueue.appendleft(data)

    def _post_initial_refresh(self, flush):
        while True:
            try:
                samples = self._sampleQueue.pop()
                if isinstance(samples, dict):
                    x_array = samples['x']
                    y_arrays = samples['y']
                elif isinstance(samples, tuple) and len(samples) == 2:
                    x_array, y_arrays = samples
                else:
                    x_array = np.arange(len(samples[0]))
                    y_arrays = samples

                self._xBuffer = x_array
                for i, signal in enumerate(self.signals):
                    arr = y_arrays[i]
                    self._dataBuffer[i][:] = arr
            except IndexError:
                break

        if flush:
            for i, signal in enumerate(self.signals):
                signal.add_chunk(
                    self._xBuffer,
                    self._dataBuffer[i]
                )


class Signal(_SignalInfo):
    """Signal: Represents a plotted curve. Optimized with rolling buffer."""
    maxChunks = 1024

    defaultStyles = [
        _SignalInfo(color=[196,78,82], width=1.5, lineStyle='-'),
        _SignalInfo(color=[85,85,85], width=1.5, lineStyle='-.'),
        _SignalInfo(color=[85,168,104], width=1.5, lineStyle=':'),
        _SignalInfo(color=[76,114,176], width=1.5, lineStyle='--'),
        _SignalInfo(color=[248,217,86], width=1.5, lineStyle='-.'),
        _SignalInfo(color=[129,114,179], width=1.5, lineStyle='-..'),
        _SignalInfo(color=[221,132,82], width=1.5, lineStyle='-'),
    ]

    def __init__(self, plot, *args, **kwargs):
        di = kwargs.pop('di', 0)

        if args and isinstance(args[0], _SignalInfo):
            super().__init__(**vars(args[0]))
        else:
            super().__init__(**kwargs)
        self.plot = plot
        self._pDIs = deque()
        self._xData = deque()
        self._yData = deque()
        self.timeWindow = None

        if self.color is None:
            self.color = Signal.defaultStyles[di].color
        if self.width is None:
            self.width = Signal.defaultStyles[di].width
        if self.lineStyle is None:
            self.lineStyle = Signal.defaultStyles[di].lineStyle

        if 'PyQt6' in sys.modules:
            if self.lineStyle == ':':
                lineStyle = Qt.PenStyle.DotLine
            elif self.lineStyle == '--':
                lineStyle = Qt.PenStyle.DashLine
            elif self.lineStyle == '-.':
                lineStyle = Qt.PenStyle.DashDotLine
            elif self.lineStyle == '-..':
                lineStyle = Qt.PenStyle.DashDotDotLine
            else:
                lineStyle = Qt.PenStyle.SolidLine
        else:
            if self.lineStyle == ':':
                lineStyle = Qt.DotLine
            elif self.lineStyle == '--':
                lineStyle = Qt.DashLine
            elif self.lineStyle == '-.':
                lineStyle = Qt.DashDotLine
            elif self.lineStyle == '-..':
                lineStyle = Qt.DashDotDotLine
            else:
                lineStyle = Qt.SolidLine

        self.pen = pg.mkPen(
            color=pg.mkColor(self.color),
            width=self.width,
            style=lineStyle
        )

        # Single persistent PlotDataItem used for both legend and real-time line rendering
        self.curve = pg.PlotDataItem(name=self.name, pen=self.pen)
        self.plot.addItem(self.curve)
        self._pDI4Legend = self.curve

    def add_chunk(self, x, y):
        """Append chunk data to rolling buffer and update single curve in-place."""
        if len(self._xData) > 0 and len(x) > 1 and x[0] == self._xData[-1]:
            self._xData.extend(x[1:])
            self._yData.extend(y[1:])
        else:
            self._xData.extend(x)
            self._yData.extend(y)

        # Prune points outside timeWindow if timeWindow is defined
        if self.timeWindow is not None and len(self._xData) > 0:
            cutoff = self._xData[-1] - self.timeWindow
            while self._xData and self._xData[0] < cutoff:
                self._xData.popleft()
                self._yData.popleft()

        # Update curve data in-place
        self.curve.setData(
            np.fromiter(self._xData, dtype=np.float32),
            np.fromiter(self._yData, dtype=np.float32)
        )

    def clear(self):
        """Clear all points from the curve."""
        self._xData.clear()
        self._yData.clear()
        self.curve.setData([], [])
        while len(self._pDIs) > 0:
            pdi = self._pDIs.popleft()
            self.plot.removeItem(pdi)


class Image(_ImageInfo):
    """Image: A class for containing and managing images to be plotted."""

    def __init__(self, plot, *args, **kwargs):
        if args and isinstance(args[0], _ImageInfo):
            super().__init__(**vars(args[0]))
        else:
            super().__init__(**kwargs)

        self.imageItem = pg.ImageItem(
            levels=self._levels,
            axisOrder='row-major'
        )
        plot.addItem(self.imageItem)

        self._tr = pg.QtGui.QTransform()
        self.scale = self._scale
        self.offset = self._offset
        self.rotation = self._rotation

    def setImage(self, image):
        self.imageItem.setImage(
            image=image,
            levels=self._levels,
        )

    @property
    def scale(self):
        return self._scale
    @scale.setter
    def scale(self, newScale):
        self._scale = newScale
        self._tr.scale(*self._scale)
        self.imageItem.setTransform(self._tr)

    @property
    def offset(self):
        return self._offset
    @offset.setter
    def offset(self, newOffset):
        self._offset = newOffset
        self._tr.translate(*self._offset)
        self.imageItem.setTransform(self._tr)

    @property
    def rotation(self):
        return self._rotation
    @rotation.setter
    def rotation(self, newRotation):
        self._rotation = newRotation
        self._tr.rotate(self._rotation)
        self.imageItem.setTransform(self._tr)

    @property
    def levels(self):
        return self._levels
    @levels.setter
    def levels(self, newLevels):
        self._levels = newLevels
        self.imageItem.setLevels(self._levels)

#endregion


#region : Scope Wrappers
class Scope():
    """Scope: A class for real-time plotting of 1D signals."""

    def __init__(self, title=None, **kwargs):
        self._ms = MultiScope(
            title=title,
            graphicsLayoutWidget=kwargs.pop('graphicsLayoutWidget', None),
        )
        self._ms.addAxis(**kwargs)

    def attachSignal(self, **kwargs):
        self._ms.axes[0].attachSignal(**kwargs)

    def sample(self, t, data):
        self._ms.axes[0].sample(t, data)

    def clean(self):
        self._ms.axes[0].clean()

    def clear(self):
        self._ms.axes[0].clear()

    def refresh(self):
        self._ms.refresh()

    @staticmethod
    def refreshAll():
        MultiScope.refreshAll()


class XYScope():
    """XYScope: A class for real-time display of 2D signals and images."""

    def __init__(self, title=None, **kwargs):
        self._ms = MultiScope(
            title=title,
            graphicsLayoutWidget=kwargs.pop('graphicsLayoutWidget', None),
        )
        self._ms.addXYAxis(**kwargs)
        self.images = []

    def attachSignal(self, **kwargs):
        self._ms.axes[0].attachSignal(**kwargs)

    def attachImage(self, *args, **kwargs):
        self._ms.axes[0].attachImage(*args, **kwargs)
        self.images.append(self._ms.axes[0].images[-1])

    def sample(self, t, data):
        self._ms.axes[0].sample(t, data)

    def clear(self):
        self._ms.axes[0].clear()

    def refresh(self):
        self._ms.refresh()

    @staticmethod
    def refreshAll():
        MultiScope.refreshAll()


class BatchScope():
    """BatchScope: For plotting a full array each update."""
    def __init__(self, title=None, arrayLen=4096, **kwargs):
        self._ms = MultiScope(
            title=title,
            graphicsLayoutWidget=kwargs.pop('graphicsLayoutWidget', None),
            arrayLen=arrayLen,
        )
        self._ms.addBatchAxis(arrayLen=arrayLen, **kwargs)

    def attachSignal(self, **kwargs):
        self._ms.axes[0].attachSignal(**kwargs)

    def sample(self, data):
        self.clear()
        self._ms.axes[0].sample(data)

    def clear(self):
        self._ms.axes[0].clear()

    def refresh(self):
        self._ms.refresh()

    @staticmethod
    def refreshAll():
        MultiScope.refreshAll()
#endregion
