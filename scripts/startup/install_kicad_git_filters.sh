#!/bin/bash

git config --local filter.kicad_sch.clean "sed -E 's/#(PWR|FLG)[0-9]+/#\1?/' "
git config --local filter.kicad_sch.smudge cat

echo "*.kicad_sch filter=kicad_sch" >> .git/info/attributes