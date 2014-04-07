
#
# Components, types and plugins.
#
# The CMake 'target' names are identical to the first argument of the
# macros below, except for orocos_typegen_headers, where the target is fully
# controlled by generated code of 'typegen'.
#

# Creates a component library libcartCommFri-<target>.so
# and installs in the directory lib/orocos/cartCommFri/
#
orocos_component(CartesianPositionController CartesianPositionController.hpp CartesianPositionController.cpp) # ...you may add multiple source files
#
# You may add multiple orocos_component statements.

#
# Additional headers:
#
# Installs in the include/orocos/cartCommFri/ directory
#
orocos_install_headers(CartesianPositionController.hpp) # ...you may add multiple header files
#
# You may add multiple orocos_install_headers statements.

