#!/usr/bin/env python3
"""UAV Route Planner v3 - Desktop Application Entry Point"""

import sys
from PyQt6.QtWidgets import QApplication
from ui.main_window import MainWindow


def main():
    """Main entry point."""
    app = QApplication(sys.argv)
    app.setApplicationName('UAV Route Planner')
    app.setApplicationVersion('3.0')
    
    window = MainWindow()
    window.show()
    
    sys.exit(app.exec())


if __name__ == '__main__':
    main()
