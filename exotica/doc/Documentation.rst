*************
Documentation
*************

Our documentation is built using `Sphinx <http://www.sphinx-doc.org>`_.

In order to build the documentation offline, you require the ``sphinx`` and ``sphinx_rtd_theme`` packages to be installed on your machine:

.. code:: bash

    sudo pip install -U sphinx sphinx_rtd_theme

Navigating to ``exotica/doc``, you can create the documentation by running ``make html``. It can be found in ``_build/html``.

The documentation is written in reStructured Text. A great reference can be found `here <http://www.sphinx-doc.org/en/stable/rest.html>`_.

The generated documentation for the current master release can always be found `here <https://ipab-slmc.github.io/exotica/>`_.


C++ API Documentation
=====================

In order to build the Doxygen C++ API documentation, you require ``doxygen`` to be installed on your system (e.g. via ``sudo apt-get install doxygen``). After navigating to the exotica package you can then run ``doxygen`` to generate the C++ API documentation. It will be created in ``doc/doxygen_cpp/doxygen_cpp``.

It can also be found `here <https://ipab-slmc.github.io/exotica/doxygen_cpp/>`_ for the current master release.

Both the C++ API documentation and the documentation in the ``doc`` subdirectory of the exotica package are generated automatically by our continuous integration services once Pull Requests/branches are merged to master.
