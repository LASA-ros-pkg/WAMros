#!/bin/bash -x

rosservice call /wam/moveToCart "{cart: {position: [$1, $2, $3], euler: [$4, $5, $6]}}"