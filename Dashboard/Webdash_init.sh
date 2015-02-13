#!/bin/bash

# Source function library
#. /etc/init.d/functions
 
#the service executable
SNAME=webdash

PROG=/usr/local/bin/$SNAME

# start function
start() {
    #check the daemon status first
    pid=`ps -ef | grep $PROG | grep -v -m 1 "grep" |  awk '{ print $1 }'`
    echo $pid > /pidout.txt
    if [ "$pid"X != "X" ]
    then
        echo "$SNAME is already started!"
        exit 0;
    else
        echo $"Starting $SNAME ..."
        $PROG &
        echo $"$SNAME started."
        exit 0;
    fi
}
 
#stop function
stop() {
    echo "Stopping $SNAME ..."
    pid=`ps -ef | grep $PROG | grep -v -m 1 "grep" |  awk '{ print $1 }'`
    [ "$pid"X != "X" ] && kill $pid
}

case "$1" in
start)
  start
  ;;
stop)
  stop
  ;;
reload|restart)
  stop
  start
  ;;
status)
  pid=`ps -ef | grep $PROG | grep -v -m 1 "grep" |  awk '{ print $1 }'`
  if [ "$pid"X = "X" ]; then
      echo "$SNAME is stopped."
  else
      echo "$SNAME (pid $pid) is running..."
  fi
  ;;
*)
  echo $"\nUsage: $0 {start|stop|restart|status}"
  exit 1
esac