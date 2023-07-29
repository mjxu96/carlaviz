#!/usr/bin/env bash

DESTINATION_FOLDER=${1}
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

echo "copying ${SCRIPT_DIR}/clang14, ${SCRIPT_DIR}/gcc11 and ${SCRIPT_DIR}/msvc17 to ${DESTINATION_FOLDER}/"
cp ${SCRIPT_DIR}/clang14 ${SCRIPT_DIR}/gcc11 ${SCRIPT_DIR}/msvc17 ${DESTINATION_FOLDER}/
