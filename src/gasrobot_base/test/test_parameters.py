"""底盘桥接参数校验测试。"""

import pytest

from gasrobot_base.parameters import BridgeConfig


@pytest.mark.parametrize(
    "overrides, expected_name",
    [
        ({"baud": 0}, "baud"),
        ({"tx_rate": 0.0}, "tx_rate"),
        ({"gyro_z_deadband": -0.1}, "gyro_z_deadband"),
        ({"port": ""}, "port"),
    ],
)
def test_invalid_configuration_is_rejected(overrides, expected_name):
    values = BridgeConfig().__dict__.copy()
    values.update(overrides)

    with pytest.raises(ValueError, match=expected_name):
        BridgeConfig(**values).validate()


def test_default_configuration_is_valid():
    BridgeConfig().validate()
