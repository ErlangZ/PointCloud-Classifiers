#!/bin/bash
train_test_model() {
    TrainFile=$1
    TestFile=$2
    WEAK_LEARNER=$3
    PARAM=$4
    IterNumber=$5
    ./Multiboost/build/multiboost \
        --d "\t" \
        --fileformat simple \
        --examplename \
        --verbose 4 \
        --learnertype $WEAK_LEARNER $PARAM \
        --outputinfo ./model/result.data \
        --shypname ./model/shyp.xml \
        --traintest $TrainFile $TestFile $IterNumber 
}

test_model() {
    TestFile=$1
    WEAK_LEARNER=$2
    PARAM=$3
    IterNumber=$4
    ./Multiboost/build/multiboost \
        --d "\t" \
        --fileformat simple \
        --examplename \
        --verbose 4 \
        --learnertype $WEAK_LEARNER $PARAM \
        --test $TestFile ./model/shyp.xml $IterNumber 
}

mkdir -p model 
#Error=20%
#WEAK_LEARNER=SingleStumpLearner
#Error=40%
#WEAK_LEARNER=AdaLineLearner
#Error=35%
#WEAK_LEARNER=SigmoidSingleStumpLearner
#Error=22%(Vehicle=4% Ped=8%)
#WEAK_LEARNER=ProductLearner
#PARAM="--baselearnertype SingleStumpLearner 3 --noloop"
#Error=22%(Vehicle=4% Ped=8%)
WEAK_LEARNER=TreeLearner
PARAM="--baselearnertype SingleStumpLearner 3 --noloop"

# USE BoundBox Only
#train_model ../data/train/bounding_box_data $WEAK_LEARNER 10
#test_model ../data/test/bounding_box_data $WEAK_LEARNER 10

train_test_model ../data/train/data ../data/test/data $WEAK_LEARNER "$PARAM" 200
test_model ../data/test/data $WEAK_LEARNER "$PARAM" 10

