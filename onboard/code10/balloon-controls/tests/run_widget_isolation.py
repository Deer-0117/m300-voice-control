#!/usr/bin/env python3
"""Compatibility runner for the current actual-source widget scenarios."""
from pathlib import Path
import subprocess
import sys
subprocess.run([sys.executable, '-B', '-m', 'unittest', 'discover', '-s',
                str(Path(__file__).resolve().parent), '-p', 'test_flight_panel_widget.py', '-v'], check=True)
