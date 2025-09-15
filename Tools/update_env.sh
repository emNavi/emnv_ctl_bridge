#!/bin/bash

if [[ $# -ne 2 ]]; then
    echo "usage: $0 <SETUP_BASH_FILE> <SCRIPT_PATH>"
    exit 1
fi

SETUP_BASH_FILE="$1"
SCRIPT_PATH="$2"
# 读取 setup.bash 内容
if [[ -f "$SETUP_BASH_FILE" ]]; then
    SETUP_BASH_CONTENT=$(<"$SETUP_BASH_FILE")
else
    echo "setup.bash not found: $SETUP_BASH_FILE"
    exit 1
fi

# 检查 SCRIPT_PATH 是否已存在于 setup.bash
if ! grep -Fxq "source $SCRIPT_PATH" "$SETUP_BASH_FILE"; then
    echo "source $SCRIPT_PATH" >> "$SETUP_BASH_FILE"
    echo "source $SCRIPT_PATH added to $SETUP_BASH_FILE"
else
    echo "source $SCRIPT_PATH already exists in $SETUP_BASH_FILE"
fi

echo "HelpFuncTarget setup completed."