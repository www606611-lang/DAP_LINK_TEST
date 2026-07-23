"""Single entry point for IDE and /sdcard power-on execution."""

import sys


PROJECT_DEVICE_DIR = "/sdcard/K230_GIMBAL"
if PROJECT_DEVICE_DIR not in sys.path:
    sys.path.insert(0, PROJECT_DEVICE_DIR)

import config
import vision


def main():
    if config.GIMBAL_MOTION_ENABLED:
        raise RuntimeError(
            "commissioning entry point requires gimbal motion disabled"
        )
    vision.run()


if __name__ == "__main__":
    main()
