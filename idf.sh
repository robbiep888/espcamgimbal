#!/bin/zsh
set -e

# Run ESP-IDF v5.5.4 without manually activating its environment first.
#
# The toolchain is shared with the neighbouring optimizedcam project so it is
# not downloaded twice. Set GIMBAL_IDF_PATH to override.
project_directory="${0:A:h}"

if [[ -n "$GIMBAL_IDF_PATH" ]]; then
    idf_directory="$GIMBAL_IDF_PATH"
elif [[ -d "$project_directory/.tools/esp-idf-v5.5.4" ]]; then
    idf_directory="$project_directory/.tools/esp-idf-v5.5.4"
else
    idf_directory="${project_directory:h}/optimizedcam/.tools/esp-idf-v5.5.4"
fi

if [[ ! -f "$idf_directory/export.sh" ]]; then
    print -u2 "ESP-IDF not found at: $idf_directory"
    print -u2 "Set GIMBAL_IDF_PATH to your ESP-IDF v5.5 checkout and retry."
    exit 1
fi

source "$idf_directory/export.sh" >/dev/null

exec idf.py "$@"
