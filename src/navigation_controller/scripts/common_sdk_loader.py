#!/usr/bin/env python3
"""
Shared Unitree SDK bootstrap for ROS2 control scripts.

Design goals:
- Keep startup behavior identical to previous scripts.
- Centralize sys.path probing logic to avoid duplicated maintenance.
- Return explicit status so caller can keep existing logs/branches.
"""

import os
import sys


def bootstrap_unitree_sdk():
    """
    Try to make `unitree_sdk2py` importable and return runtime symbols.

    Returns:
        dict with keys:
            available (bool),
            loco_client_cls (type|None),
            channel_factory_init (callable|None),
            path_added (bool)
    """
    path_added = False

    try:
        from ament_index_python.packages import get_package_prefix
        try:
            sdk_prefix = get_package_prefix('unitree_sdk2py')
            sdk_site_packages = os.path.join(
                sdk_prefix, 'lib', 'python3.10', 'site-packages'
            )
            if os.path.exists(sdk_site_packages):
                if sdk_site_packages not in sys.path:
                    sys.path.insert(0, sdk_site_packages)
                path_added = True
                print(f"[SDK路径] 方法1成功: {sdk_site_packages}", file=sys.stderr)
        except Exception as e1:
            print(f"[SDK路径] 方法1失败: {e1}", file=sys.stderr)
            script_dir = os.path.dirname(os.path.abspath(__file__))
            workspace_root = os.path.realpath(
                os.path.join(script_dir, '..', '..', '..', '..')
            )
            sdk_site_packages = os.path.join(
                workspace_root,
                'install',
                'unitree_sdk2py',
                'lib',
                'python3.10',
                'site-packages'
            )
            if os.path.exists(sdk_site_packages):
                if sdk_site_packages not in sys.path:
                    sys.path.insert(0, sdk_site_packages)
                path_added = True
                print(f"[SDK路径] 方法2成功: {sdk_site_packages}", file=sys.stderr)
            else:
                print(f"[SDK路径] 方法2失败: 路径不存在 {sdk_site_packages}", file=sys.stderr)
    except Exception as e:
        print(f"[SDK路径] 致命错误: {e}", file=sys.stderr)

    try:
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize
        from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
        print("[SDK导入] ✅ unitree_sdk2py 导入成功", file=sys.stderr)
        return {
            'available': True,
            'loco_client_cls': LocoClient,
            'channel_factory_init': ChannelFactoryInitialize,
            'path_added': path_added
        }
    except ImportError as e:
        print(f"[SDK导入] ❌ 导入失败: {e}", file=sys.stderr)
        print(f"[SDK导入] sys.path: {[p for p in sys.path if 'unitree' in p]}", file=sys.stderr)
        return {
            'available': False,
            'loco_client_cls': None,
            'channel_factory_init': None,
            'path_added': path_added
        }
