CSI2 HM0360 ISP Example
================

How to run
----------

1. Configure CMake with the desired camera resolution

Format QVGA:
.. code-block:: bash

    cmake -B build "-DCONFIG_DRIVER_HM0360_FORMAT_QVGA=y"

Format VGA:
.. code-block:: bash

    cmake -B build "-DCONFIG_DRIVER_HM0360_FORMAT_VGA=y"


2. Build and run the application

.. code-block:: bash

    cmake --build build -t run

3. Display the image

.. code-block:: bash

    feh Out.ppm

