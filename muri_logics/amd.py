"""
Compatibility shim so tests that do `from amd import AMD` continue to work.

This module re-exports the AMD class implemented in the `vision` package.
"""
from vision.aruco_marker_detection import AMD

__all__ = ["AMD"]
