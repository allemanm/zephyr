# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources(mcp980x.c)
zephyr_library_sources_ifdef(CONFIG_MCP980X_TRIGGER mcp980x_trigger.c)
