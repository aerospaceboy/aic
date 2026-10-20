"""Match campaign participants to the manual launcher's ROS transport."""

import os
from pathlib import Path
from typing import MutableMapping, Optional


def configure_campaign_transport(
    workspace: Path,
    environ: Optional[MutableMapping[str, str]] = None,
) -> None:
    """Configure DDS before importing rclpy or creating any participant.

    A child shell cannot export its transport back into the Python parent.
    Configure every participant alike, preserving a caller-selected domain.
    """
    configure_ros_transport(
        Path(workspace) / 'install/tools_demo/share/tools_demo/config', environ,
    )


def configure_ros_transport(config_dir, environ=None):
    """Select local CycloneDDS, or an explicit diagnostic FastDDS override."""
    env = os.environ if environ is None else environ
    implementation = env.get('YZBOT_RMW', 'rmw_cyclonedds_cpp')
    profiles = {
        'rmw_cyclonedds_cpp': 'cyclonedds_local.xml',
        'rmw_fastrtps_cpp': 'fastdds_udp.xml',
    }
    if implementation not in profiles:
        raise ValueError(f'Unsupported YZBOT_RMW: {implementation}')
    profile = Path(config_dir) / profiles[implementation]
    if not profile.is_file():
        raise FileNotFoundError(f'Campaign DDS profile is missing: {profile}')
    env.setdefault('ROS_DOMAIN_ID', '0')
    env['RMW_IMPLEMENTATION'] = implementation
    if implementation == 'rmw_cyclonedds_cpp':
        env.pop('FASTRTPS_DEFAULT_PROFILES_FILE', None)
        env['CYCLONEDDS_URI'] = profile.resolve().as_uri()
    else:
        env.pop('CYCLONEDDS_URI', None)
        env['FASTRTPS_DEFAULT_PROFILES_FILE'] = str(profile)
