*************************************
Code Styleguide and Naming Convention
*************************************

Packages, files and directories
===============================

- Package names in lower case with underscores and exotica prefix:

 ::

        exotica
        exotica_python
        exotica_examples
        exotica_core_taskmaps
        exotica_aico_solver
        exotica_ik_solver
        exotica_collision_scene_fcl_latest
        ...

- All solver packages with ``_solver`` suffix.
- All task map packages with ``_taskmaps`` suffix (or ``_taskmap`` if it contains only a single task map).
- File names: ``with_underscores.cpp``, ``with_underscores.h``, ``with_underscores.py`` (do not use \*.cc, \*.hpp, ... file types)
- Directory structure:

  ::

        package_name\
        package_name\launch\ (*.launch)
        package_name\src\ (*.cpp)
        package_name\include\package_name\ (*.h)
        package_name\scripts\ (*.py - with correct shebang and without file endings)
        package_name\init\ (*.in)
        package_name\cmake\ (*.cmake and supporting scripts)
        package_name\doc\ (doxygen and other files)
        package_name\resources\ (urdf/srdf/meshes/scenes/trajectories/...)
        package_name\test\ (C++/python test scripts)
        package_name\test\resources\ (test resources)

For files in ``scripts/``, if they are Python file:

- Make the script executable (``chmod +x filename``).
- Add an appropriate shebang.
- Do not include the file ending.

C++
===

We follow the `Google C++ style guide <https://google.github.io/styleguide/cppguide.html#Naming>`__ with minor amendments (cf. file naming above). A good reference on notation terminology is the `Drake style guide <https://drake.mit.edu/doxygen_cxx/group__multibody__notation.html>`__.

Where the Google style guide leaves multiple options, here are our choices:

- Use ``override`` instead of ``virtual`` when overriding a function in a child class.
- Order output parameters after inputs if returning values via arguments.
- Avoid boost.
- Always use C++11.
- Type names: ``CamelCasedWithFirstCapitalLetter``
- Variable names: ``with_underscores``
- Member variables: ``with_underscores_and_trailing_underscore_``
- Struct data members: ``with_underscores``
- Constants: ``kCamelCasedWithPrefix``
- Function names: ``CamelCasedWithFirstCapitalLetter``
- Namespace names (always single word): ``lowercase``
- Enum names: ``CamelCasedWithFirstCapitalLetter``
- Enum data members: ``ALL_CAPS``
- Macro names: ``ALL_CAPS``
- Comments: ``// always use double slash``
- File comments - include author, copyright, and license
- File/class/function/variable comments: ``/// always use doxygen documentation style comments``
- Implementation comments: ``// always use regular double slash comments on a separate line``
- TODO: ``// TODO(githubusername or email) use comments with enough detail - the name references who added it or who is responsible for fixing it``

Formatting
~~~~~~~~~~~

- No line length limit (use your own judgment for readability).
- Use the provided clang-format -- cf. :ref:`Code Formatting`.
- Use UTF-8 encoding.
- Use 4 spaces instead of tabs.
- Opening ``{`` on a new line on the same level as the conditional/function/loop and the closing brace.
- Use vertical white space to separate code blocks/declarations.

Python
======

- Packages/Modules: Short lower case (avoid underscores), e.g., ``pyexotica``
- Class names: ``CamelCased``
- Function and variable names: ``lower_case_with_underscores``
- Non-public members: ``_lower_case_with_leading_underscore``
- Constants: ``ALL_CAPS``
- Getters/setters: always replace with attributes
- Always include explicit exports (``__all__``)
- Apply `PEP8 <https://www.python.org/dev/peps/pep-0008/#naming-conventions>`__ formatting
