#!/opt/homebrew/bin/python3
"""
Shim to keep the old standalone entry-point while reusing the unified Rosbot
controller that now lives in controllers/rosbot/rosbot.py.
"""

from pathlib import Path
import sys

# Ensure the shared rosbot package is on sys.path
sys.path.append(str(Path(__file__).resolve().parent.parent))

from rosbot.rosbot import main  # type: ignore


if __name__ == "__main__":
    main()
