import platform
import pytest
import dartpy as dart


def test_constructors():
    x1 = dart.math.SO3()
    x2 = dart.math.SO3.Random()
    x3 = dart.math.SO3.Identity()
    x4 = x1 * x2
    v1 = x4 * [0, 1, 2]


if __name__ == "__main__":
    pytest.main()
