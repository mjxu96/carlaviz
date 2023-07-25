#!/usr/bin/env bash

DESTINATION_FOLDER=${1}
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

echo "copying ${SCRIPT_DIR}/clang14 and ${SCRIPT_DIR}/gcc11 to ${DESTINATION_FOLDER}/"
cp ${SCRIPT_DIR}/clang14 ${SCRIPT_DIR}/gcc11 ${DESTINATION_FOLDER}/
