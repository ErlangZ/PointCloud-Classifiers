#!/bin/bash
MULTIBOOST_ROOT=/home/erlangz/MultiBoost/build/
$MULTIBOOST_ROOT/multiboost \
    --d "\t" \
    --fileformat simple \
    --verbose 3 \
    --learnertype SingleStumpLearner \
    --outputinfo result.data --shypname shyp.xml \
    --traintest ../data/train/X-axis ../data/test/X-axis 10 \
    --examplename
