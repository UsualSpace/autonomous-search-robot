#!/bin/bash

#file that setups runtime enviroments, acts as a wrapper
set -e

echo "Provided Arguements: $@"

exec $@


