echo "RUN THIS SCRIPT AS SUDO OR EVERYTHING WILL EXPLODE!"
ssh -t aa274@elroy.local "sudo date --set=\"1984-07-04 10:05:59.990\" "
date --set="1984-07-04 10:05:59.990"
ssh aa274@elroy.local
