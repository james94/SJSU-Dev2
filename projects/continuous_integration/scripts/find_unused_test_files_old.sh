#!/bin/bash

# Import SJSU-Dev2 common shell script functions
# Get base path
COMMON_SCRIPT_DIRECTORY=$(dirname "$0")
SJBASE=$(cd "${COMMON_SCRIPT_DIRECTORY}/../../../" ; pwd -P)

. ${COMMON_SCRIPT_DIRECTORY}/common.sh

missing_test_totals=0

for unused_test_file in "$@"; do
  echo -e "${RED}Missing from library test suite:${RESET} ${unused_test_file}"
  missing_test_totals=$(($missing_test_totals + 1))
done

if [[ ${missing_test_totals} -eq "0" ]]; then
  echo -e "${BOLD_GREEN}Everything clear! No files unused tests found!${RESET}"
  exit 0
else
  echo
  echo -ne "Total number of unused tests: "
  echo -e "${BOLD_RED}${missing_test_totals}${RESET}"
  exit ${missing_test_totals}
fi
