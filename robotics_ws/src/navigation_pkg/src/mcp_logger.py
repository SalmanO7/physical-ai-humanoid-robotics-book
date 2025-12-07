"""
MCP Server Integration for Navigation Data Logging

This module provides functionality to log navigation data to the MCP server
for versioning and analysis.
"""

import json
import requests
from datetime import datetime
from typing import Dict, Any, Optional


class MCPServerLogger:
    """
    Logger for sending navigation data to MCP server.
    """

    def __init__(self, server_url: str = "http://localhost:8080"):
        self.server_url = server_url.rstrip('/')
        self.session = requests.Session()

    def log_navigation_event(self, event_type: str, data: Dict[str, Any]) -> bool:
        """
        Log a navigation event to the MCP server.

        Args:
            event_type: Type of navigation event (e.g., 'navigation_start', 'navigation_success', 'navigation_failure')
            data: Dictionary containing navigation data to log

        Returns:
            True if logging was successful, False otherwise
        """
        log_entry = {
            'timestamp': datetime.utcnow().isoformat(),
            'event_type': event_type,
            'data': data,
            'source': 'navigation_system'
        }

        try:
            response = self.session.post(
                f"{self.server_url}/api/logs/navigation",
                json=log_entry,
                headers={'Content-Type': 'application/json'},
                timeout=5
            )
            response.raise_for_status()
            return True
        except requests.exceptions.RequestException as e:
            print(f"Failed to log navigation event to MCP server: {e}")
            return False

    def log_navigation_metrics(self, metrics: Dict[str, float]) -> bool:
        """
        Log navigation performance metrics to the MCP server.

        Args:
            metrics: Dictionary containing navigation performance metrics

        Returns:
            True if logging was successful, False otherwise
        """
        log_entry = {
            'timestamp': datetime.utcnow().isoformat(),
            'event_type': 'navigation_metrics',
            'data': metrics,
            'source': 'navigation_system'
        }

        try:
            response = self.session.post(
                f"{self.server_url}/api/metrics/navigation",
                json=log_entry,
                headers={'Content-Type': 'application/json'},
                timeout=5
            )
            response.raise_for_status()
            return True
        except requests.exceptions.RequestException as e:
            print(f"Failed to log navigation metrics to MCP server: {e}")
            return False

    def log_path_data(self, path_points: list, metadata: Dict[str, Any]) -> bool:
        """
        Log path planning data to the MCP server.

        Args:
            path_points: List of path points (x, y coordinates)
            metadata: Additional metadata about the path

        Returns:
            True if logging was successful, False otherwise
        """
        log_entry = {
            'timestamp': datetime.utcnow().isoformat(),
            'event_type': 'path_data',
            'data': {
                'path_points': path_points,
                'metadata': metadata
            },
            'source': 'navigation_system'
        }

        try:
            response = self.session.post(
                f"{self.server_url}/api/data/navigation/path",
                json=log_entry,
                headers={'Content-Type': 'application/json'},
                timeout=5
            )
            response.raise_for_status()
            return True
        except requests.exceptions.RequestException as e:
            print(f"Failed to log path data to MCP server: {e}")
            return False