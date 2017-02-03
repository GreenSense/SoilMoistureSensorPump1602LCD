BRANCH=$1

if [ -z "$BRANCH" ]; then
    BRANCH=$(git branch | sed -n -e 's/^\* \(.*\)/\1/p')
fi

if [ -z "$BRANCH" ]; then
    BRANCH="master"
fi

docker run -it -v $PWD:/SoilMoistureSensorPump1602LCD-src compulsivecoder/ubuntu-platformio /bin/bash -c "git clone /SoilMoistureSensorPump1602LCD-src /SoilMoistureSensorPump1602LCD-dest/ && cd /SoilMoistureSensorPump1602LCD-dest/ && sh init.sh && sh build.sh $BRANCH"
