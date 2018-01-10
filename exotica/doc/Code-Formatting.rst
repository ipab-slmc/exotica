***************
Code Formatting
***************

We use ``clang-format-3.8`` to automatically format source files and headers *prior* to committing any files. The format is specified in the ``.clang-format`` file provided in the root of the repository.

You can install ``clang-format-3.8`` e.g. via your package managers:

.. code-block:: bash

    sudo apt-get install clang-format-3.8

In order to format your changes prior to committing, please run the following from the root of the repository:

.. code-block:: bash

    find -name '*.cpp' -o -name '*.h' | xargs clang-format-3.8 -style=file -i
