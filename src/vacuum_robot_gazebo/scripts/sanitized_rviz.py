#!/usr/bin/env python3
# Copyright 2026 Adaptive Pivot-G2 Research Team
# Licensed under the Apache License, Version 2.0

"""Start RViz without desktop-Snap GTK/GDK library injection."""

import os
import shutil
import sys

from environment_manager import sanitized_rviz_environment


def main() -> None:
    environment = sanitized_rviz_environment()
    executable = shutil.which(
        'rviz2', path=environment.get('PATH')
    )
    if executable is None:
        raise FileNotFoundError('Cannot find the rviz2 executable')
    os.execve(
        executable,
        [executable, *sys.argv[1:]],
        environment,
    )


if __name__ == '__main__':
    main()
