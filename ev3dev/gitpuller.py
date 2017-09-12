"""Pulls from git automatically as a cronjob."""

import subprocess

subprocess.run(['git pull'])
