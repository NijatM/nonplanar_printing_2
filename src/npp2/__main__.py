import platform

import compas
import pybullet
import compas_fab

if __name__ == "__main__":
    print()
    print("Yay! NPP2 is installed correctly!")
    print()
    print("COMPAS FAB: {}".format(compas_fab.__version__))
    print("COMPAS: {}".format(compas.__version__))
    print(
        "Python: {} ({})".format(
            platform.python_version(), platform.python_implementation()
        )
    )
