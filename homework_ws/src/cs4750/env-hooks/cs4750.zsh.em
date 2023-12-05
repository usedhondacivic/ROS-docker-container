mviz() {
    rviz -d $(rospack find cs4750)/config/default.rviz
}

export CORNELL_TOOLS_SHELL_FLAVOR=zsh
@[if DEVELSPACE]@
. @(CMAKE_CURRENT_SOURCE_DIR)/config/env.sh
export SOURCES_ROOT="@(CMAKE_CURRENT_SOURCE_DIR)/src/cornell_tools"

export PATH="@(CMAKE_CURRENT_SOURCE_DIR)/scripts/available_client/:$PATH"
. "@(CMAKE_CURRENT_SOURCE_DIR)/scripts/cornell_tools_client"

@[else]@
# TODO(nickswalker): Add install space support
@[end if]@