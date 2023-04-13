#!/bin/bash

dts devel build -f -H taobot.local &
dts devel run -H taobot.local -- --privileged &
