"""Portable default paths for the UMI dataset project."""

import os
from pathlib import Path


def project_root():
    configured = os.environ.get('UMI_ORB_SLAM3_ROOT')
    if configured:
        return Path(configured).expanduser().resolve()

    candidates = [Path.cwd().resolve(), *Path.cwd().resolve().parents]
    candidates.extend(Path(__file__).resolve().parents)
    for candidate in candidates:
        if (
            (candidate / 'ros2_ws' / 'src' / 'umi_dataset_tools').is_dir()
            and (candidate / 'core_ws' / 'src' / 'ORB_SLAM3').is_dir()
        ):
            return candidate
    return (Path.home() / 'umi_ORB_SLAM3').resolve()


def data_root():
    configured = os.environ.get('UMI_DATA_ROOT')
    if configured:
        return Path(configured).expanduser().resolve()
    return project_root() / 'data'
