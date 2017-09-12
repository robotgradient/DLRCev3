"""Pulls from git automatically as a cronjob."""

import subprocess

subprocess.call(['git pull'])
