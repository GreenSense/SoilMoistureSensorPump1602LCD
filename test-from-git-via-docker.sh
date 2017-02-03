BRANCH=$1

if [ -z "$BRANCH" ]; then
    BRANCH=$(git branch | sed -n -e 's/^\* \(.*\)/\1/p')
fi

if [ -z "$BRANCH" ]; then
    BRANCH="master"
fi

docker run -it -v $PWD:/easypost-src compulsivecoder/ubuntu-mono /bin/bash -c "git clone /easypost-src /easypost-dest/ && cd /easypost-dest/ && sh init.sh && sh build-and-test-all.sh $BRANCH"
