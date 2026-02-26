***************
Code Formatting
***************

We use ``clang-format-6.0`` to automatically format source files and headers *prior* to committing any files. The format is specified in the ``.clang-format`` file provided in the root of the repository.

You can install ``clang-format-6.0`` e.g. via your package managers:

.. code-block:: bash

    sudo apt-get install clang-format-6.0

In order to format your changes prior to committing, please execute script `apply_format.sh` in the repository root.
This will run:

.. code-block:: bash

    find -name '*.cpp' -o -name '*.h' -o -name '*.hpp' | xargs clang-format-6.0 -style=file -i
