#!/bin/bash
train_model() {
    TrainFile=$1
    IterNumber=$2
    ./Multiboost/build/multiboost \
        --d "\t" \
        --fileformat simple \
        --examplename \
        --verbose 3 \
        --learnertype SingleStumpLearner \
        --outputinfo ./model/result.data \
        --shypname ./model/shyp.xml \
        --train $TrainFile $IterNumber 
}

test_model() {
    TestFile=$1
    IterNumber=$2
    ./Multiboost/build/multiboost \
        --d "\t" \
        --fileformat simple \
        --examplename \
        --verbose 3 \
        --learnertype SingleStumpLearner \
        --test $TestFile ./model/shyp.xml $IterNumber 
}

mkdir -p model
