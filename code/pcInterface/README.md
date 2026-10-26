# Drilling HMI — Dashboard v13

PyQt6-based application for controlling and monitoring the rover drill. The communication protocol and transport worker are unchanged from the original project.

## Install and run

Python 3.9+ required. In the folder containing `main.py`:

```bash
python -m pip install -r requirements.txt
python main.py
```

USB mode requires an appropriate serial driver and a connected drill controller. UDP mode connects to the main rover communications application running on the same PC.

## Connection

The **Connection** button in the top bar opens a floating connection panel, without moving the dashboard. Clicking outside closes it. Select USB Serial (38,400 baud) or UDP localhost (default send port 5610 / listen port 5611), then connect. The connection status always remains visible in the top bar. Sun/moon switches between the dark and light themes.

## Live Drill Status

* The graph plots real measurements as the STATE responses arrive (the controller is polled every 500 ms while connected).
* Each value has its own stable color and a circular visibility switch: **filled bright-red circle = ON**, **empty gray circle = OFF**. The latest numerical value appears beside the signal name.
* Signals with the same unit share a Y axis; different units have their own real-valued Y axes. Active axis labels are shown below their corresponding axes. Scales update using the enabled measurements within the displayed window.
* The time window can be set to 30 s, 1, 2, 5, 10, 30 or 60 min. The X axis uses seconds up to 5 min and minutes for longer ranges.
* The graph scrolls as data arrives, while up to 60 minutes / 7,200 samples remain buffered. Switching to a longer window can reveal previous samples. **Clear history** explicitly clears the buffer.
* Controller states are three separate state machines (DeepSampler, DrillController, DeepSampleHolder), not sensor measurements.

## Workspace: Windows menu

**Dashboard layout** restores the default responsive dashboard with the large Drill Status graph and equally tall Communication Log above the control panels. The chart takes roughly 77% of the upper row's width, subject to minimum panel widths.

**Reset layout** is intentionally different: it shows **all nine panels at their individual compact dimensions**, packed into three columns near the upper-left corner. These panels are **not stretched to fill the viewport**. On smaller screens the workspace may need scrollbars.

**Show all windows** unhides every panel, and individual entries in the Windows menu control panel visibility.

Move a panel by dragging its header. Resize by positioning the pointer over its frame edge, where a **horizontal or vertical double-arrow cursor** appears, then dragging. In Dashboard mode, only shared edges are resizable, so adjacent panels remain aligned. Dragging the divider between the graph and log resizes just those two panels, with updates capped near 60 Hz for smoother feedback. Reset layout allows free panel-edge resizing. Panels can also be minimized, maximized and hidden via their header buttons.

The Calibration, Height Measurement and Autonomy panels retain equally wide paired action buttons. Device Status lists peripherals in two columns. **Clear log** clears the communication log independently of graph history.

The app saves panel positions and the theme using QSettings. Choosing **Dashboard layout** always resets factory widths and heights; choosing **Reset layout** always resets compact sizes.

## Project files

- `main.py` — application entry point
- `gui.py` — Qt workspace, chart, controls and layout logic
- `comm_worker.py` — threaded serial/UDP transport
- `protocol.py` — drill message framing and parsing
- `requirements.txt` — Python dependencies

## Verification

Python source syntax and panel geometry/regression logic were tested without the drill controller. On-machine visual rendering and live hardware communication still need to be checked with PyQt6 installed.
