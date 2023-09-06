#!/bin/bash
# this is only needed for assembly line programming
set -e
DST=psoc-programmer-stm32
rm -rf $DST
git clone git@github.com:BelaPlatform/psoc-programmer-linux.git $DST
cd $DST
git checkout 20b6fc4801b70d6d636d4c9d847c0d92794d2e10
